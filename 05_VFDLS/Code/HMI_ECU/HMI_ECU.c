/******************************************************************************
 * File: HMI_ECU.c
 *
 * Purpose:
 *     Application layer of the HMI ECU (ATmega32 #1) in the Vehicle Fault
 *     Detection and Logging System (VFDLS).
 *
 * Responsibilities:
 *     - Show the main menu on the 4x16 LCD and read the driver's choice
 *       from the 4x4 keypad.
 *     - Send the command to the Control ECU over the UART and show its answer
 *       (see vfdls_protocol.h for the frame formats).
 *     - Use Timer1 in compare mode as a 1-second time base for the
 *       10-second live display and the 10-second "returning to menu" wait.
 *
 * Main menu:
 *     1 -> Start Operation (start monitoring, message for 10 seconds)
 *     2 -> Display Values  (live sensor data for 10 seconds, then "Display again?")
 *     3 -> Retrieve Faults (scroll the DTCs saved in the EEPROM, then "Display again?")
 *     4 -> Stop Monitoring
 *
 * Pin map (8 MHz), as in the Proteus schematic:
 *     PC0..PC7 -> LCD DB0..DB7     PA0 -> LCD RS     PA1 -> LCD E
 *     PB0..PB3 -> Keypad rows A..D PB4..PB7 -> Keypad columns 1..4
 *     PD0/PD1  -> UART RXD/TXD to the Control ECU
 *
 * PC2..PC5 are the JTAG pins and carry LCD DB2..DB5, so JTAG is disabled
 * at the start of main().
 ******************************************************************************/

#include <avr/io.h>              /* SREG and MCUCSR registers. */
#include <util/delay.h>          /* _delay_ms() (uses F_CPU). */
#include <util/atomic.h>         /* ATOMIC_BLOCK() for a safe 16-bit read. */

#include "std_types.h"
#include "common_macros.h"
#include "gpio.h"
#include "uart.h"
#include "lcd.h"
#include "keypad.h"
#include "timer.h"
#include "vfdls_protocol.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/*
 * Timer1 compare value for a 1-second tick:
 *   Timer clock = 8 MHz / 1024 = 7812.5 Hz
 *   In CTC mode the timer counts 0..OCR1A, so OCR1A = 7812 -> 7813 counts = 1.0 s.
 */
#define TIMER1_ONE_SECOND_COMPARE      7812

/* Durations from the specification, in seconds. */
#define LIVE_DISPLAY_TIME_S            10    /* Display Values: 10-second cycle. */
#define START_MESSAGE_TIME_S           10    /* Start Operation: back to menu after 10 s. */
#define STOP_MESSAGE_TIME_S            10    /* Stop Monitoring: back to menu after 10 s. */
#define SCROLL_STEP_TIME_S             1     /* Fault list: one line up every second. */
#define FAULT_LIST_TIME_S              10    /* Retrieve Faults: list shown for 10 s. */
#define INFO_MESSAGE_TIME_S            2     /* Short messages (errors, monitoring off). */

/* Live display refresh period while showing the values. */
#define LIVE_REFRESH_MS                500

/* Maximum time to wait for each byte of a Control ECU response. */
#define RESPONSE_TIMEOUT_MS            1500

/* Delay after a key press so one press is not read twice. */
#define KEY_DEBOUNCE_MS                300

/* The ASCII code 0xDF shows the degree symbol on HD44780 LCDs. */
#define LCD_DEGREE_SYMBOL              0xDF

/* Lines in the fault list: title + up to MAX_LOGGED_FAULTS DTCs + end line. */
#define FAULT_LIST_MAX_LINES           (MAX_LOGGED_FAULTS + 2)

/*******************************************************************************
 *                           Global Variables                                  *
 *******************************************************************************/

/*
 * Seconds counted by the Timer1 callback.
 * volatile: it is changed inside the interrupt and read by the main code.
 */
static volatile uint16 g_seconds = 0;

/* Values received in the last values frame. */
typedef struct
{
	uint8  monitoring;      /* TRUE if the Control ECU is monitoring. */
	uint8  temperature;     /* Degrees Celsius. */
	uint16 distance;        /* Centimeters. */
	uint8  window1;         /* WINDOW_OPEN or WINDOW_CLOSED. */
	uint8  window2;
	uint8  activeFaults;    /* ACTIVE_FAULT_P001_BIT / ACTIVE_FAULT_P002_BIT. */
}SensorValues;

/*******************************************************************************
 *                      Private Functions Prototypes                           *
 *******************************************************************************/

static void JTAG_disable(void);
static void App_init(void);
static void Timer1_tick(void);
static uint16 Time_now(void);
static void Wait_seconds(uint16 seconds);
static void Menu_show(void);
static void Option_startOperation(void);
static void Option_displayValues(void);
static void Option_retrieveFaults(void);
static void Option_stopMonitoring(void);
static void Show_message(const char *line1, const char *line2, uint16 seconds);
static void Comm_flushReceiveBuffer(void);
static uint8 Comm_receiveByte(uint8 *data);
static uint8 Comm_sendCommandWaitAck(uint8 command);
static uint8 Comm_getValues(SensorValues *values);
static uint8 Comm_getFaults(uint8 *faults, uint8 *count);
static void Lcd_printPadded(uint8 row, const char *text);
static const char *Dtc_text(uint8 dtc);

/*******************************************************************************
 *                                 Main                                        *
 *******************************************************************************/

int main(void)
{
	uint8 key;   /* Key pressed by the driver. */

	/* Step 1: Free PC2..PC5 from JTAG, then initialize all drivers. */
	JTAG_disable();
	App_init();

	/* Step 2: Super-loop: show the menu, wait for a key, run the option. */
	while (1)
	{
		Menu_show();

		/* Step 2.1: Wait for a key (the keypad driver blocks until a press). */
		key = KEYPAD_getPressedKey();
		_delay_ms(KEY_DEBOUNCE_MS);

		/* Step 2.2: Run the selected option; other keys just redraw the menu. */
		switch (key)
		{
			case 1: Option_startOperation(); break;
			case 2: Option_displayValues();  break;
			case 3: Option_retrieveFaults(); break;
			case 4: Option_stopMonitoring(); break;
			default: break;
		}
	}
}

/*******************************************************************************
 *                      Private Functions Definitions                          *
 *******************************************************************************/

/*
 * Disable the JTAG interface so PC2..PC5 work as normal I/O pins.
 * The datasheet requires writing JTD = 1 twice within 4 clock cycles,
 * so the value is prepared first and then written twice in a row.
 */
static void JTAG_disable(void)
{
	uint8 value = MCUCSR | (1 << JTD);
	MCUCSR = value;
	MCUCSR = value;
}

/* Initialize the LCD, the UART and Timer1 (1-second compare interrupt). */
static void App_init(void)
{
	/* UART: 8 data bits, no parity, 1 stop bit, 9600 bps (same as the Control ECU). */
	UART_ConfigType uart_config = {UART_DATA_BITS_8, UART_PARITY_DISABLED, UART_STOP_BIT_1, 9600};

	/*
	 * Timer1, compare mode, clock = F_CPU/1024, interrupt every 1 second.
	 * Order of the members: initial value, compare value, timer ID, clock, mode.
	 */
	Timer_ConfigType timer_config = {0, TIMER1_ONE_SECOND_COMPARE, TIMER1_ID, TIMER_F_CPU_1024, TIMER_COMPARE_MODE};

	/* Step 1: LCD and UART. */
	LCD_init();
	UART_init(&uart_config);

	/* Step 2: Timer1: register the callback first, then start the timer. */
	Timer_setCallBack(Timer1_tick, TIMER1_ID);
	Timer_init(&timer_config);

	/* Step 3: Enable global interrupts (UART receive + Timer1 compare). */
	SREG |= (1 << 7);

	/* Step 4: Short welcome screen while the Control ECU starts up. */
	Show_message("     VFDLS", "  Starting...", INFO_MESSAGE_TIME_S);
}

/* Timer1 callback: called from the compare-match ISR once every second. */
static void Timer1_tick(void)
{
	g_seconds++;
}

/*
 * Return the seconds counter safely.
 * g_seconds is 16 bits, but the AVR reads it as two separate bytes. If the
 * Timer1 interrupt changed it between the two reads (e.g. 255 -> 256), the
 * result would be wrong, so interrupts are blocked during the read.
 */
static uint16 Time_now(void)
{
	uint16 now;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		now = g_seconds;
	}
	return now;
}

/* Wait the given number of seconds, counted by the Timer1 interrupt. */
static void Wait_seconds(uint16 seconds)
{
	uint16 start = Time_now();   /* Time when the wait started. */

	/*
	 * The subtraction also works when g_seconds wraps from 65535 to 0,
	 * because unsigned arithmetic wraps the same way.
	 */
	while ((uint16)(Time_now() - start) < seconds)
	{
		/* Wait for the next Timer1 ticks. */
	}
}

/* Draw the main menu on the four LCD lines. */
static void Menu_show(void)
{
	LCD_clearScreen();
	/* Each line is 16 characters at most, the width of the LCD. */
	LCD_displayStringRowColumn(0, 0, "1.StartOperation");
	LCD_displayStringRowColumn(1, 0, "2.Display Values");
	LCD_displayStringRowColumn(2, 0, "3.RetrieveFaults");
	LCD_displayStringRowColumn(3, 0, "4.StopMonitoring");
}

/*
 * Key 1 (spec 3.2): ask the Control ECU to start monitoring, show
 * "Operation Started! Monitoring Active..." and go back to the menu after 10 s.
 * The message is split over 4 lines because each line holds 16 characters.
 */
static void Option_startOperation(void)
{
	if (Comm_sendCommandWaitAck(CMD_START_MONITORING) == FALSE)
	{
		Show_message("Comm Error!", "No ACK received", INFO_MESSAGE_TIME_S);
		return;
	}

	LCD_clearScreen();
	LCD_displayStringRowColumn(0, 0, "Operation");
	LCD_displayStringRowColumn(1, 0, "Started!");
	LCD_displayStringRowColumn(2, 0, "Monitoring");
	LCD_displayStringRowColumn(3, 0, "Active...");
	Wait_seconds(START_MESSAGE_TIME_S);
}

/*
 * Key 2 (spec 3.5): show the live values for 10 seconds:
 *   Line 1 -> Temp: XX C
 *   Line 2 -> Distance: XX cm
 *   Line 3 -> Win1: Open / Closed
 *   Line 4 -> Win2: Open / Closed
 * (Window 1 and window 2 are on separate lines because
 *  "Win1: Closed  Win2: Open" is longer than 16 characters.)
 * Then ask "Display again?": key 2 repeats, any other key returns to the menu.
 */
static void Option_displayValues(void)
{
	SensorValues values;
	uint16 start;
	uint8 key;

	do
	{
		/* Step 1: First request, to know if monitoring is running at all. */
		if (Comm_getValues(&values) == FALSE)
		{
			Show_message("Comm Error!", "No data received", INFO_MESSAGE_TIME_S);
			return;
		}

		if (values.monitoring == FALSE)
		{
			Show_message("Monitoring is OFF", "Press 1 to start", INFO_MESSAGE_TIME_S);
			return;
		}

		/* Step 2: Refresh the values until 10 seconds have passed. */
		LCD_clearScreen();
		start = Time_now();
		while ((uint16)(Time_now() - start) < LIVE_DISPLAY_TIME_S)
		{
			/* Line 1: temperature, with the degree symbol. */
			LCD_moveCursor(0, 0);
			LCD_displayString("Temp: ");
			LCD_intgerToString(values.temperature);
			LCD_displayCharacter(LCD_DEGREE_SYMBOL);
			LCD_displayString("C    ");             /* Spaces erase old longer numbers. */

			/* Line 2: distance. */
			LCD_moveCursor(1, 0);
			LCD_displayString("Distance: ");
			LCD_intgerToString(values.distance);
			LCD_displayString(" cm  ");

			/* Lines 3 and 4: window states. */
			Lcd_printPadded(2, (values.window1 == WINDOW_OPEN) ? "Win1: Open" : "Win1: Closed");
			Lcd_printPadded(3, (values.window2 == WINDOW_OPEN) ? "Win2: Open" : "Win2: Closed");

			/* Wait, then ask for new values (keep the old ones if the request fails). */
			_delay_ms(LIVE_REFRESH_MS);
			Comm_getValues(&values);
		}

		/* Step 3: Ask whether to display the values again. */
		LCD_clearScreen();
		LCD_displayStringRowColumn(0, 0, "Display again?");
		LCD_displayStringRowColumn(1, 0, "Press 2 = YES");
		LCD_displayStringRowColumn(2, 0, "Other key =");
		LCD_displayStringRowColumn(3, 0, "MAIN MENU");

		key = KEYPAD_getPressedKey();
		_delay_ms(KEY_DEBOUNCE_MS);

	} while (key == 2);   /* Key 2 repeats the display; any other key returns to the menu. */
}

/*
 * Key 3 (spec 3.6): read the saved DTCs from the Control ECU and show them:
 *   Logged Faults:
 *   P001:TooClose
 *   P002:Overheat
 *   --End of List--
 * The LCD shows 4 lines, so with more than 2 codes the list scrolls up one
 * line every second until the last line is shown. After 10 seconds the LCD
 * asks "Display again?": key 3 repeats, any other key returns to the menu.
 */
static void Option_retrieveFaults(void)
{
	uint8 faults[MAX_LOGGED_FAULTS];
	uint8 count;
	const char *lines[FAULT_LIST_MAX_LINES];
	uint8 lines_count;
	uint8 first_line;
	uint8 row;
	uint8 i;
	uint8 key;
	uint16 start;

	do
	{
		/* Step 1: Request the list. */
		count = 0;
		if (Comm_getFaults(faults, &count) == FALSE)
		{
			Show_message("Comm Error!", "No data received", INFO_MESSAGE_TIME_S);
			return;
		}

		/* Step 2: Build the text lines to show. */
		lines_count = 0;
		lines[lines_count++] = "Logged Faults:";
		if (count == 0)
		{
			lines[lines_count++] = "No Faults Found";
		}
		for (i = 0; i < count; i++)
		{
			lines[lines_count++] = Dtc_text(faults[i]);
		}
		lines[lines_count++] = "--End of List--";

		/*
		 * Step 3: Show the list. Every second the shown window moves one line
		 * down the list, until the last line is on the bottom row.
		 * With 2 codes or fewer everything fits, so nothing scrolls.
		 */
		start = Time_now();
		first_line = 0;
		while (1)
		{
			for (row = 0; row < LCD_ROWS; row++)
			{
				if ((first_line + row) < lines_count)
				{
					Lcd_printPadded(row, lines[first_line + row]);
				}
				else
				{
					Lcd_printPadded(row, "");    /* Empty line below the end of the list. */
				}
			}

			/* Stop when the last line is already shown on the bottom row. */
			if ((first_line + LCD_ROWS) >= lines_count)
			{
				break;
			}

			Wait_seconds(SCROLL_STEP_TIME_S);
			first_line++;
		}

		/* Step 4: Keep the list on the screen until 10 seconds have passed. */
		while ((uint16)(Time_now() - start) < FAULT_LIST_TIME_S)
		{
			/* Wait for the Timer1 ticks. */
		}

		/* Step 5: Ask whether to display the list again. */
		LCD_clearScreen();
		LCD_displayStringRowColumn(0, 0, "Display again?");
		LCD_displayStringRowColumn(1, 0, "Press 3 = YES");
		LCD_displayStringRowColumn(2, 0, "Other key =");
		LCD_displayStringRowColumn(3, 0, "MAIN MENU");

		key = KEYPAD_getPressedKey();
		_delay_ms(KEY_DEBOUNCE_MS);

	} while (key == 3);   /* Key 3 shows the list again; any other key returns to the menu. */
}

/* Key 4 (spec 3.7): stop monitoring, show the message for 10 seconds, then back to the menu. */
static void Option_stopMonitoring(void)
{
	if (Comm_sendCommandWaitAck(CMD_STOP_MONITORING) == FALSE)
	{
		Show_message("Comm Error!", "No ACK received", INFO_MESSAGE_TIME_S);
		return;
	}

	LCD_clearScreen();
	LCD_displayStringRowColumn(0, 0, "System Monitoring");
	LCD_displayStringRowColumn(1, 0, "Stopped!");
	LCD_displayStringRowColumn(2, 0, "Returning to");
	LCD_displayStringRowColumn(3, 0, "Menu...");
	Wait_seconds(STOP_MESSAGE_TIME_S);
}

/* Clear the LCD, show up to two lines, and keep them for the given seconds. */
static void Show_message(const char *line1, const char *line2, uint16 seconds)
{
	LCD_clearScreen();
	LCD_displayStringRowColumn(0, 0, line1);
	LCD_displayStringRowColumn(1, 0, line2);
	Wait_seconds(seconds);
}

/* Throw away any old bytes, so the next response starts from a clean buffer. */
static void Comm_flushReceiveBuffer(void)
{
	uint8 dummy;
	while (UART_receiveByte(&dummy))
	{
		/* Discard. */
	}
}

/*
 * Wait up to RESPONSE_TIMEOUT_MS for one byte from the Control ECU.
 * Returns TRUE and the byte in *data, or FALSE after the timeout.
 */
static uint8 Comm_receiveByte(uint8 *data)
{
	uint16 timeout = RESPONSE_TIMEOUT_MS;

	while (timeout > 0)
	{
		/* The UART driver returns 1 as soon as its RX buffer has a byte. */
		if (UART_receiveByte(data))
		{
			return TRUE;
		}
		_delay_ms(1);
		timeout--;
	}
	return FALSE;
}

/* Send a command and wait for its ACK frame: [RSP_ACK] [command] [checksum]. */
static uint8 Comm_sendCommandWaitAck(uint8 command)
{
	uint8 header, echo, checksum;

	/* Step 1: Clean buffer, then send the command byte. */
	Comm_flushReceiveBuffer();
	UART_sendByte(command);

	/* Step 2: Receive the three bytes of the ACK frame. */
	if ((Comm_receiveByte(&header) == FALSE) ||
	    (Comm_receiveByte(&echo) == FALSE) ||
	    (Comm_receiveByte(&checksum) == FALSE))
	{
		return FALSE;
	}

	/* Step 3: Check the header, the echoed command and the checksum. */
	if ((header == RSP_ACK) && (echo == command) && (checksum == (uint8)(header ^ echo)))
	{
		return TRUE;
	}
	return FALSE;
}

/* Request the values frame and decode it into *values. Returns TRUE on success. */
static uint8 Comm_getValues(SensorValues *values)
{
	uint8 frame[VALUES_FRAME_SIZE];
	uint8 checksum = 0;
	uint8 i;

	/* Step 1: Send the request. */
	Comm_flushReceiveBuffer();
	UART_sendByte(CMD_GET_VALUES);

	/* Step 2: Receive the whole frame. */
	for (i = 0; i < VALUES_FRAME_SIZE; i++)
	{
		if (Comm_receiveByte(&frame[i]) == FALSE)
		{
			return FALSE;
		}
	}

	/* Step 3: Check the header and the checksum (XOR of the first 8 bytes). */
	for (i = 0; i < (VALUES_FRAME_SIZE - 1); i++)
	{
		checksum ^= frame[i];
	}
	if ((frame[0] != RSP_VALUES) || (checksum != frame[VALUES_FRAME_SIZE - 1]))
	{
		return FALSE;
	}

	/* Step 4: Decode the fields (the distance is sent high byte first). */
	values->monitoring   = BIT_IS_SET(frame[1], STATUS_MONITORING_BIT) ? TRUE : FALSE;
	values->temperature  = frame[2];
	values->distance     = (uint16)(((uint16)frame[3] << 8) | frame[4]);
	values->window1      = frame[5];
	values->window2      = frame[6];
	values->activeFaults = frame[7];
	return TRUE;
}

/*
 * Request the faults frame: [RSP_FAULTS] [N] [DTC 1..N] [checksum].
 * Copies the DTCs into faults[] and their number into *count.
 */
static uint8 Comm_getFaults(uint8 *faults, uint8 *count)
{
	uint8 header, n, received_checksum;
	uint8 checksum;
	uint8 i;

	/* Step 1: Send the request. */
	Comm_flushReceiveBuffer();
	UART_sendByte(CMD_GET_FAULTS);

	/* Step 2: Header and count; reject a count larger than the buffer. */
	if ((Comm_receiveByte(&header) == FALSE) || (Comm_receiveByte(&n) == FALSE))
	{
		return FALSE;
	}
	if ((header != RSP_FAULTS) || (n > MAX_LOGGED_FAULTS))
	{
		return FALSE;
	}

	/* Step 3: The DTC bytes, accumulating the checksum. */
	checksum = header ^ n;
	for (i = 0; i < n; i++)
	{
		if (Comm_receiveByte(&faults[i]) == FALSE)
		{
			return FALSE;
		}
		checksum ^= faults[i];
	}

	/* Step 4: Compare with the received checksum. */
	if ((Comm_receiveByte(&received_checksum) == FALSE) || (received_checksum != checksum))
	{
		return FALSE;
	}

	*count = n;
	return TRUE;
}

/*
 * Write a text on one LCD row and fill the rest of the row with spaces,
 * so nothing from the previous screen stays visible.
 */
static void Lcd_printPadded(uint8 row, const char *text)
{
	uint8 col = 0;

	LCD_moveCursor(row, 0);
	while ((text[col] != '\0') && (col < LCD_COLUMNS))
	{
		LCD_displayCharacter((uint8)text[col]);
		col++;
	}
	while (col < LCD_COLUMNS)
	{
		LCD_displayCharacter(' ');
		col++;
	}
}

/* Text shown on the LCD for each DTC (at most 16 characters). */
static const char *Dtc_text(uint8 dtc)
{
	switch (dtc)
	{
		case DTC_P001_ACCIDENT_MIGHT_HAPPENED: return "P001:TooClose";
		case DTC_P002_ENGINE_HIGH_TEMPERATURE: return "P002:Overheat";
		default:                               return "Unknown DTC";
	}
}
