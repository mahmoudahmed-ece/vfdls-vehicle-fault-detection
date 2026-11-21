/******************************************************************************
 * File: CONTROL_ECU.c
 *
 * Purpose:
 *     Application layer of the Control ECU (ATmega32 #2) in the Vehicle Fault
 *     Detection and Logging System (VFDLS).
 *
 * Responsibilities:
 *     - Read the engine temperature (LM35 on the ADC) and the parking
 *       distance (HC-SR04 through the ICU).
 *     - Detect the faults and log every DTC once in the external EEPROM:
 *         P001 ACCIDENT_MIGHT_HAPPENED  -> distance < 10 cm
 *         P002 ENGINE_HIGH_TEMPERATURE  -> temperature > 90 C
 *     - Control the two car windows (DC motors) from four push buttons
 *       while monitoring is active.
 *     - Answer the commands of the HMI ECU over the UART
 *       (see vfdls_protocol.h for the frame formats).
 *
 * Pin map (8 MHz), as in the Proteus schematic:
 *     PA1      -> LM35 output (ADC channel 1)
 *     PC2      -> Window 1 OPEN (ON) button  PC3 -> Window 1 CLOSE (OFF) button
 *     PC4      -> Window 2 OPEN (ON) button  PC5 -> Window 2 CLOSE (OFF) button
 *                 (buttons connect the pin to GND when pressed; the internal
 *                  pull-up resistors keep them HIGH when released)
 *     PB0/PB1  -> Window 1 motor IN1/IN2   PB4/PB5 -> Window 2 motor IN3/IN4
 *     PB3      -> OC0 PWM to the motor driver enable pins (always 100 %)
 *     PC0/PC1  -> I2C SCL/SDA to the 24C16 EEPROM
 *     PD0/PD1  -> UART RXD/TXD to the HMI ECU
 *     PD6      -> HC-SR04 ECHO (ICP1)      PD7 -> HC-SR04 TRIG
 *
 * PC2..PC5 are the JTAG pins and carry the window buttons, so JTAG is
 * disabled at the start of main().
 ******************************************************************************/

#include <avr/io.h>              /* SREG and MCUCSR registers. */
#include <util/delay.h>          /* _delay_ms() (uses F_CPU). */

#include "std_types.h"
#include "common_macros.h"
#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "lm35_sensor.h"
#include "ultrasonic.h"
#include "dc_motor.h"
#include "twi.h"
#include "external_eeprom.h"
#include "vfdls_protocol.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* Fault thresholds from the specification. */
#define DISTANCE_FAULT_LIMIT_CM        10    /* P001 when distance < 10 cm. */
#define TEMPERATURE_FAULT_LIMIT_C      90    /* P002 when temperature > 90 C. */

/* Window buttons: all on PORTC, active LOW. */
#define BUTTONS_PORT_ID                PORTC_ID
#define WIN1_OPEN_BUTTON_PIN_ID        PIN2_ID
#define WIN1_CLOSE_BUTTON_PIN_ID       PIN3_ID
#define WIN2_OPEN_BUTTON_PIN_ID        PIN4_ID
#define WIN2_CLOSE_BUTTON_PIN_ID       PIN5_ID
#define BUTTON_PRESSED                 LOGIC_LOW

/* How long a window motor runs to fully open or close the window. */
#define WINDOW_MOVE_TIME_MS            1000

/* The motors always run at full speed (Timer0 PWM duty cycle = 100 %). */
#define MOTOR_MAX_SPEED                100

/*
 * Main loop period. The HC-SR04 needs at least about 60 ms between two
 * measurements, and one measurement itself takes up to 40 ms.
 */
#define MAIN_LOOP_DELAY_MS             50

/*
 * External EEPROM layout:
 *   address 0x000            -> number of saved DTCs (0..MAX_LOGGED_FAULTS)
 *   address 0x001 .. 0x00A   -> the saved DTC numbers, in the order they happened
 * A new (blank) 24C16 reads 0xFF everywhere, so a count larger than
 * MAX_LOGGED_FAULTS means "empty memory" and is reset to 0.
 */
#define EEPROM_FAULT_COUNT_ADDRESS     0x0000
#define EEPROM_FAULT_LIST_ADDRESS      0x0001

/* Time the 24C16 needs to finish an internal write cycle (datasheet: max 10 ms). */
#define EEPROM_WRITE_CYCLE_MS          10

/*******************************************************************************
 *                           Global Variables                                  *
 *******************************************************************************/

/* TRUE while the HMI has started monitoring (key 1) and not stopped it (key 4). */
static uint8 g_monitoring = FALSE;

/* Latest sensor readings, sent to the HMI on CMD_GET_VALUES. */
static uint8  g_temperature = 0;     /* Degrees Celsius. */
static uint16 g_distance = 0;        /* Centimeters (0 = no valid reading yet). */

/* Faults that are present right now (bits ACTIVE_FAULT_P001_BIT / P002_BIT). */
static uint8 g_activeFaults = 0;

/* Current state of each window: WINDOW_OPEN or WINDOW_CLOSED. */
static uint8 g_window1State = WINDOW_CLOSED;
static uint8 g_window2State = WINDOW_CLOSED;

/*
 * Copy of the DTC list saved in the EEPROM, kept in RAM so the application
 * can check quickly whether a DTC is already logged, without reading the
 * EEPROM on every loop.
 */
static uint8 g_faultCount = 0;
static uint8 g_faultList[MAX_LOGGED_FAULTS];

/* Previous button readings, used to act only once per press (edge detection). */
static uint8 g_prevButtons[4] = {LOGIC_HIGH, LOGIC_HIGH, LOGIC_HIGH, LOGIC_HIGH};

/*******************************************************************************
 *                      Private Functions Prototypes                           *
 *******************************************************************************/

static void JTAG_disable(void);
static void App_init(void);
static void Faults_loadFromEeprom(void);
static void Faults_log(uint8 dtc);
static void Monitoring_update(void);
static void Windows_update(void);
static void Window_move(DC_Motor_ID motor, uint8 *state, uint8 new_state);
static void Comm_handleCommand(uint8 command);
static void Comm_sendAck(uint8 command);
static void Comm_sendValues(void);
static void Comm_sendFaults(void);

/*******************************************************************************
 *                                 Main                                        *
 *******************************************************************************/

int main(void)
{
	uint8 command;   /* Command byte received from the HMI ECU. */

	/* Step 1: Free PC2..PC5 from JTAG, then initialize the drivers, windows and fault list. */
	JTAG_disable();
	App_init();

	/* Step 2: Super-loop. */
	while (1)
	{
		/*
		 * Step 2.1: Handle every command waiting in the UART receive buffer.
		 * The UART driver stores incoming bytes in its buffer from the RX
		 * interrupt, so no byte is lost while the loop is busy elsewhere.
		 */
		while (UART_receiveByte(&command))
		{
			Comm_handleCommand(command);
		}

		/*
		 * Step 2.2: While monitoring (spec 3.2: Start Operation activates the
		 * sensors and the actuators):
		 *   - open or close the windows when their buttons are pressed,
		 *   - read the sensors, detect faults and log new DTCs.
		 */
		if (g_monitoring == TRUE)
		{
			Windows_update();
			Monitoring_update();
		}

		/* Step 2.3: Wait before the next cycle (also spaces the ultrasonic readings). */
		_delay_ms(MAIN_LOOP_DELAY_MS);
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

/*
 * Initialize the drivers in the order they depend on each other, then load
 * the DTCs already saved in the EEPROM.
 */
static void App_init(void)
{
	/* UART: 8 data bits, no parity, 1 stop bit, 9600 bps (same as the HMI ECU). */
	UART_ConfigType uart_config = {UART_DATA_BITS_8, UART_PARITY_DISABLED, UART_STOP_BIT_1, 9600};

	/*
	 * ADC: internal 2.56 V reference (must match ADC_REF_VOLT_VALUE in adc.h)
	 * and prescaler 64 -> ADC clock = 8 MHz / 64 = 125 kHz (inside 50..200 kHz).
	 */
	ADC_ConfigType adc_config = {ADC_INTERNAL_2_56V, ADC_PRESCALER_64};

	/*
	 * TWI: own address 0x01 (only used if this ECU is addressed as a slave)
	 * and SCL = 100 kHz -> TWBR = (8 MHz / 100 kHz - 16) / 2 = 32.
	 */
	TWI_ConfigType twi_config = {0x01, 100000};

	/* Step 1: Window buttons as inputs with their internal pull-up resistors. */
	GPIO_setupPinDirection(BUTTONS_PORT_ID, WIN1_OPEN_BUTTON_PIN_ID, PIN_INPUT);
	GPIO_setupPinDirection(BUTTONS_PORT_ID, WIN1_CLOSE_BUTTON_PIN_ID, PIN_INPUT);
	GPIO_setupPinDirection(BUTTONS_PORT_ID, WIN2_OPEN_BUTTON_PIN_ID, PIN_INPUT);
	GPIO_setupPinDirection(BUTTONS_PORT_ID, WIN2_CLOSE_BUTTON_PIN_ID, PIN_INPUT);
	GPIO_writePin(BUTTONS_PORT_ID, WIN1_OPEN_BUTTON_PIN_ID, LOGIC_HIGH);    /* Enable pull-up. */
	GPIO_writePin(BUTTONS_PORT_ID, WIN1_CLOSE_BUTTON_PIN_ID, LOGIC_HIGH);
	GPIO_writePin(BUTTONS_PORT_ID, WIN2_OPEN_BUTTON_PIN_ID, LOGIC_HIGH);
	GPIO_writePin(BUTTONS_PORT_ID, WIN2_CLOSE_BUTTON_PIN_ID, LOGIC_HIGH);

	/* Step 2: Motors stopped, PWM running at full duty (both windows start closed). */
	DcMotor_Init();
	DcMotor_Rotate(DC_MOTOR_1, DC_Motor_stop, MOTOR_MAX_SPEED);
	DcMotor_Rotate(DC_MOTOR_2, DC_Motor_stop, MOTOR_MAX_SPEED);

	/* Step 3: Peripheral drivers. */
	ADC_init(&adc_config);
	TWI_init(&twi_config);
	UART_init(&uart_config);
	Ultrasonic_init();

	/*
	 * Step 4: Enable global interrupts (I-bit in SREG): needed by the UART
	 * receive interrupt and by the ICU interrupt used by the ultrasonic driver.
	 */
	SREG |= (1 << 7);

	/* Step 5: Read the DTCs saved before the last power-off. */
	Faults_loadFromEeprom();
}

/* Read the saved DTC count and list from the EEPROM into RAM. */
static void Faults_loadFromEeprom(void)
{
	uint8 i;
	uint8 count = 0;

	/* Step 1: Read the count. If the read fails, start with an empty list. */
	if (EEPROM_readByte(EEPROM_FAULT_COUNT_ADDRESS, &count) == ERROR)
	{
		count = 0;
	}

	/* Step 2: A count out of range means a blank EEPROM: reset it to 0. */
	if (count > MAX_LOGGED_FAULTS)
	{
		count = 0;
		EEPROM_writeByte(EEPROM_FAULT_COUNT_ADDRESS, 0);
		_delay_ms(EEPROM_WRITE_CYCLE_MS);
	}

	/* Step 3: Read every saved DTC. */
	for (i = 0; i < count; i++)
	{
		EEPROM_readByte(EEPROM_FAULT_LIST_ADDRESS + i, &g_faultList[i]);
	}
	g_faultCount = count;
}

/*
 * Log one DTC in the EEPROM, only if it is not already saved
 * ("every error should be logged once in case of occurrence").
 */
static void Faults_log(uint8 dtc)
{
	uint8 i;

	/* Step 1: Do nothing if this DTC is already in the list. */
	for (i = 0; i < g_faultCount; i++)
	{
		if (g_faultList[i] == dtc)
		{
			return;
		}
	}

	/* Step 2: Do nothing if the list is full. */
	if (g_faultCount >= MAX_LOGGED_FAULTS)
	{
		return;
	}

	/*
	 * Step 3: Save the DTC at the next free address, then the new count.
	 * The EEPROM needs its write cycle time after each byte.
	 */
	EEPROM_writeByte(EEPROM_FAULT_LIST_ADDRESS + g_faultCount, dtc);
	_delay_ms(EEPROM_WRITE_CYCLE_MS);
	EEPROM_writeByte(EEPROM_FAULT_COUNT_ADDRESS, g_faultCount + 1);
	_delay_ms(EEPROM_WRITE_CYCLE_MS);

	/* Step 4: Update the RAM copy. */
	g_faultList[g_faultCount] = dtc;
	g_faultCount++;
}

/* Read both sensors, update the active faults and log new DTCs. */
static void Monitoring_update(void)
{
	/* Step 1: Read the engine temperature and the parking distance. */
	g_temperature = LM35_getTemperature();
	g_distance = Ultrasonic_readDistance();

	/*
	 * Step 2: P001 - distance below 10 cm.
	 * A distance of 0 means the sensor has not answered yet, so it is ignored.
	 */
	if ((g_distance > 0) && (g_distance < DISTANCE_FAULT_LIMIT_CM))
	{
		SET_BIT(g_activeFaults, ACTIVE_FAULT_P001_BIT);
		Faults_log(DTC_P001_ACCIDENT_MIGHT_HAPPENED);
	}
	else
	{
		CLEAR_BIT(g_activeFaults, ACTIVE_FAULT_P001_BIT);
	}

	/* Step 3: P002 - engine temperature above 90 C. */
	if (g_temperature > TEMPERATURE_FAULT_LIMIT_C)
	{
		SET_BIT(g_activeFaults, ACTIVE_FAULT_P002_BIT);
		Faults_log(DTC_P002_ENGINE_HIGH_TEMPERATURE);
	}
	else
	{
		CLEAR_BIT(g_activeFaults, ACTIVE_FAULT_P002_BIT);
	}
}

/*
 * Check the four window buttons. A window moves only on a new press
 * (HIGH -> LOW change), so holding a button does not repeat the action.
 */
static void Windows_update(void)
{
	uint8 now[4];   /* Current readings: win1 open, win1 close, win2 open, win2 close. */
	uint8 i;

	/* Step 1: Read all buttons. */
	now[0] = GPIO_readPin(BUTTONS_PORT_ID, WIN1_OPEN_BUTTON_PIN_ID);
	now[1] = GPIO_readPin(BUTTONS_PORT_ID, WIN1_CLOSE_BUTTON_PIN_ID);
	now[2] = GPIO_readPin(BUTTONS_PORT_ID, WIN2_OPEN_BUTTON_PIN_ID);
	now[3] = GPIO_readPin(BUTTONS_PORT_ID, WIN2_CLOSE_BUTTON_PIN_ID);

	/* Step 2: Act on each button that has just been pressed. */
	for (i = 0; i < 4; i++)
	{
		if ((now[i] == BUTTON_PRESSED) && (g_prevButtons[i] != BUTTON_PRESSED))
		{
			switch (i)
			{
				case 0: Window_move(DC_MOTOR_1, &g_window1State, WINDOW_OPEN);   break;
				case 1: Window_move(DC_MOTOR_1, &g_window1State, WINDOW_CLOSED); break;
				case 2: Window_move(DC_MOTOR_2, &g_window2State, WINDOW_OPEN);   break;
				case 3: Window_move(DC_MOTOR_2, &g_window2State, WINDOW_CLOSED); break;
				default: break;
			}
		}

		/* Step 3: Remember this reading for the next check. */
		g_prevButtons[i] = now[i];
	}
}

/*
 * Move one window to the requested state:
 *   open  -> motor clockwise for WINDOW_MOVE_TIME_MS
 *   close -> motor anticlockwise for WINDOW_MOVE_TIME_MS
 * Nothing happens if the window is already in that state.
 */
static void Window_move(DC_Motor_ID motor, uint8 *state, uint8 new_state)
{
	/* Step 1: Ignore the request if the window is already there. */
	if (*state == new_state)
	{
		return;
	}

	/* Step 2: Run the motor in the right direction at full speed. */
	if (new_state == WINDOW_OPEN)
	{
		DcMotor_Rotate(motor, DC_Motor_clockwise, MOTOR_MAX_SPEED);
	}
	else
	{
		DcMotor_Rotate(motor, DC_Motor_Anticlockwise, MOTOR_MAX_SPEED);
	}

	/* Step 3: Wait until the window reaches its end position, then stop. */
	_delay_ms(WINDOW_MOVE_TIME_MS);
	DcMotor_Rotate(motor, DC_Motor_stop, MOTOR_MAX_SPEED);

	/* Step 4: Save the new state (reported to the HMI in the values frame). */
	*state = new_state;
}

/* Execute one command from the HMI ECU and send its response. */
static void Comm_handleCommand(uint8 command)
{
	switch (command)
	{
		case CMD_START_MONITORING:
			/* Key 1: start reading the sensors and detecting faults. */
			g_monitoring = TRUE;

			/*
			 * Take the current button levels as the "previous" readings, so a
			 * button held down while monitoring was off is not seen as a new press.
			 */
			g_prevButtons[0] = GPIO_readPin(BUTTONS_PORT_ID, WIN1_OPEN_BUTTON_PIN_ID);
			g_prevButtons[1] = GPIO_readPin(BUTTONS_PORT_ID, WIN1_CLOSE_BUTTON_PIN_ID);
			g_prevButtons[2] = GPIO_readPin(BUTTONS_PORT_ID, WIN2_OPEN_BUTTON_PIN_ID);
			g_prevButtons[3] = GPIO_readPin(BUTTONS_PORT_ID, WIN2_CLOSE_BUTTON_PIN_ID);

			Monitoring_update();          /* First reading right away. */
			Comm_sendAck(command);
			break;

		case CMD_GET_VALUES:
			/* Key 2: send the latest readings. */
			Comm_sendValues();
			break;

		case CMD_GET_FAULTS:
			/* Key 3: send the DTC list saved in the EEPROM. */
			Comm_sendFaults();
			break;

		case CMD_STOP_MONITORING:
			/* Key 4: halt monitoring; the active faults are no longer valid. */
			g_monitoring = FALSE;
			g_activeFaults = 0;
			Comm_sendAck(command);
			break;

		default:
			/* Unknown byte (noise or a partial frame): ignore it. */
			break;
	}
}

/* ACK frame: [RSP_ACK] [command] [checksum]. */
static void Comm_sendAck(uint8 command)
{
	UART_sendByte(RSP_ACK);
	UART_sendByte(command);
	UART_sendByte(RSP_ACK ^ command);
}

/*
 * Values frame:
 * [RSP_VALUES] [status] [temp] [dist high] [dist low] [win1] [win2] [faults] [checksum]
 */
static void Comm_sendValues(void)
{
	uint8 frame[VALUES_FRAME_SIZE];
	uint8 checksum = 0;
	uint8 i;

	/* Step 1: Fill the frame. */
	frame[0] = RSP_VALUES;
	frame[1] = (g_monitoring == TRUE) ? (1 << STATUS_MONITORING_BIT) : 0;
	frame[2] = g_temperature;
	frame[3] = (uint8)(g_distance >> 8);      /* High byte of the 16-bit distance. */
	frame[4] = (uint8)(g_distance & 0xFF);    /* Low byte. */
	frame[5] = g_window1State;
	frame[6] = g_window2State;
	frame[7] = g_activeFaults;

	/* Step 2: Checksum = XOR of the first 8 bytes. */
	for (i = 0; i < (VALUES_FRAME_SIZE - 1); i++)
	{
		checksum ^= frame[i];
	}
	frame[VALUES_FRAME_SIZE - 1] = checksum;

	/* Step 3: Send all the bytes. */
	for (i = 0; i < VALUES_FRAME_SIZE; i++)
	{
		UART_sendByte(frame[i]);
	}
}

/* Faults frame: [RSP_FAULTS] [count N] [DTC 1] ... [DTC N] [checksum]. */
static void Comm_sendFaults(void)
{
	uint8 checksum = RSP_FAULTS ^ g_faultCount;
	uint8 i;

	/* Step 1: Header and count. */
	UART_sendByte(RSP_FAULTS);
	UART_sendByte(g_faultCount);

	/* Step 2: Every saved DTC, in the order it was logged. */
	for (i = 0; i < g_faultCount; i++)
	{
		UART_sendByte(g_faultList[i]);
		checksum ^= g_faultList[i];
	}

	/* Step 3: Checksum. */
	UART_sendByte(checksum);
}
