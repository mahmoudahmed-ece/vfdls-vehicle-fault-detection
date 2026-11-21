/*
 * File: uart.c
 * Purpose:
 *     ATmega32 UART driver implementation.
 *
 * Receive path:
 *     UART hardware -> receive interrupt (ISR) -> software ring buffer -> application.
 *
 * Transmit path:
 *     Application -> UART_sendByte() -> polling of UDRE -> UART hardware.
 */

#include "uart.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "common_macros.h"

/*
 * UART_RX_BUFFER_SIZE:
 *     Total number of slots allocated for received bytes.
 *
 * Why 64?
 *     A power-of-two size lets us wrap an index using a bit mask rather than
 *     using division or a more expensive modulo operation.
 *
 * Why only 63 bytes usable?
 *     This ring buffer keeps one slot empty to distinguish these two states:
 *       - Empty: head == tail
 *       - Full:  next_head == tail
 *     Therefore, a 64-slot buffer can store 63 unread bytes.
 */
#define UART_RX_BUFFER_SIZE 64U

/*
 * UART_RX_BUFFER_MASK:
 *     For a 64-slot buffer, the mask is 63 (binary 00111111).
 *     Applying (index & mask) wraps index 64 back to 0.
 */
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1U)

/*
 * Compile-time check:
 *     The ring-buffer index trick above only works when the size is a power of two.
 *     Stop compilation if UART_RX_BUFFER_SIZE is changed to an invalid value.
 */
#if ((UART_RX_BUFFER_SIZE & (UART_RX_BUFFER_SIZE - 1U)) != 0)
#error UART_RX_BUFFER_SIZE_must_be_a_power_of_two
#endif

/*
 * rx_buffer:
 *     Stores received data bytes until the application reads them.
 *     volatile is used because the ISR writes this array while main() reads it.
 *
 * rx_head:
 *     Index where the ISR will write the next received byte.
 *
 * rx_tail:
 *     Index of the oldest unread byte. UART_receiveByte() advances this index
 *     after it copies a byte to the application.
 *
 * rx_overflow_count:
 *     Counts bytes dropped because the ring buffer had no free slot.
 *
 * rx_error_count:
 *     Counts receive events with a hardware error flag set.
 *
 * These variables are private to uart.c. The application accesses them only
 * through the public UART functions in uart.h.
 */
static volatile uint8 rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint8 rx_head = 0U;
static volatile uint8 rx_tail = 0U;
static volatile uint8 rx_overflow_count = 0U;
static volatile uint8 rx_error_count = 0U;

void UART_init(const UART_ConfigType *Config_Ptr)
{
    /*
     * ubrr_value:
     *     The calculated baud-rate divisor written to UBRRH and UBRRL.
     *
     * ucsra_value:
     *     Value prepared for UCSRA. In this driver, only U2X is configured here.
     *
     * ucsrb_value:
     *     Value prepared for UCSRB to enable RX interrupt, receiver, and transmitter.
     *
     * ucsrc_value:
     *     Value prepared for UCSRC to select asynchronous mode and configure frame
     *     format. URSEL must be 1 when writing UCSRC on ATmega32 because UCSRC
     *     shares an I/O address with UBRRH.
     */
    uint16 ubrr_value;
    uint8 ucsra_value = 0U;
    uint8 ucsrb_value = 0U;
    uint8 ucsrc_value = (1U << URSEL);

    /* Do not configure UART if the configuration pointer is invalid or baud rate is zero. */
    if ((Config_Ptr == 0) || (Config_Ptr->baud_rate == 0UL))
    {
        return;
    }

    /*
     * UCSRA — USART Control and Status Register A:
     *     U2X = 1: select double-speed mode.
     *     U2X = 0: select normal-speed mode.
     *
     *     RXC, TXC, UDRE, FE, DOR, and PE are hardware status flags.
     *     They are not frame-format configuration fields.
     *
     * Baud-rate divisor formula for asynchronous mode:
     *     Normal speed: UBRR = F_CPU / (16 * baud_rate) - 1
     *     Double speed: UBRR = F_CPU / ( 8 * baud_rate) - 1
     *
     * F_CPU must match the actual CPU clock and be defined consistently in the project.
     */
    /*
     * Double-speed mode is always used:
     *     U2X = 1 -> UBRR = F_CPU / (8 * baud_rate) - 1
     * Example at 8 MHz and 9600 bps: UBRR = 103 (error about 0.2 %).
     */
    SET_BIT(ucsra_value, U2X);
    ubrr_value = (uint16)((F_CPU / (8UL * Config_Ptr->baud_rate)) - 1UL);

    /*
     * UCSRB — USART Control and Status Register B:
     *     RXCIE = 1: enable Receive Complete Interrupt.
     *     RXEN  = 1: enable the UART receiver.
     *     TXEN  = 1: enable the UART transmitter.
     *     UCSZ2 remains 0, so 9-bit data mode is not enabled.
     *
     * This enables the peripheral's receive interrupt, but the global interrupt
     * flag is separate. main() must call sei() for interrupts to run.
     */
    SET_BIT(ucsrb_value, RXCIE);
    SET_BIT(ucsrb_value, RXEN);
    SET_BIT(ucsrb_value, TXEN);

    /*
     * UCSRC — USART Control and Status Register C:
     *     URSEL = 1 selects UCSRC when writing the shared UCSRC/UBRRH address.
     *     UMSEL = 0 selects asynchronous UART mode.
     *     UCSZ1:UCSZ0 select data size:
     *         00 = 5 bits, 01 = 6 bits, 10 = 7 bits, 11 = 8 bits.
     *     UPM1:UPM0 select parity:
     *         00 = disabled, 10 = even, 11 = odd.
     *     USBS = 0 means one stop bit; USBS = 1 means two stop bits.
     *     UCPOL is not needed in asynchronous mode.
     *
     * All fields start at zero except URSEL, so asynchronous mode, no parity,
     * and one stop bit are the defaults unless changed below.
     */
    switch (Config_Ptr->bit_data)
    {
        case UART_DATA_BITS_5:
            /* UCSZ1:UCSZ0 = 00: keep both bits clear for 5 data bits. */
            break;

        case UART_DATA_BITS_6:
            /* UCSZ1:UCSZ0 = 01: set UCSZ0 for 6 data bits. */
            SET_BIT(ucsrc_value, UCSZ0);
            break;

        case UART_DATA_BITS_7:
            /* UCSZ1:UCSZ0 = 10: set UCSZ1 for 7 data bits. */
            SET_BIT(ucsrc_value, UCSZ1);
            break;

        case UART_DATA_BITS_8:
            /* UCSZ1:UCSZ0 = 11: set both bits for 8 data bits. */
            SET_BIT(ucsrc_value, UCSZ1);
            SET_BIT(ucsrc_value, UCSZ0);
            break;

        default:
            /* Unsupported data-size option. */
            return;
    }

    /* UPM1:UPM0 = 00 means parity disabled. */
    if (Config_Ptr->parity == UART_PARITY_EVEN)
    {
        /* UPM1:UPM0 = 10 selects even parity. */
        SET_BIT(ucsrc_value, UPM1);
    }
    else if (Config_Ptr->parity == UART_PARITY_ODD)
    {
        /* UPM1:UPM0 = 11 selects odd parity. */
        SET_BIT(ucsrc_value, UPM1);
        SET_BIT(ucsrc_value, UPM0);
    }
    else if (Config_Ptr->parity != UART_PARITY_DISABLED)
    {
        /* Unsupported parity option. */
        return;
    }

    if (Config_Ptr->stop_bit == UART_STOP_BIT_2)
    {
        /* USBS = 1 selects two stop bits. */
        SET_BIT(ucsrc_value, USBS);
    }
    else if (Config_Ptr->stop_bit != UART_STOP_BIT_1)
    {
        /* Unsupported stop-bit option. */
        return;
    }

    /*
     * UBRRH and UBRRL — USART Baud Rate Registers:
     *     Together they hold the UBRR divisor (12 bits on ATmega32).
     *     UBRRH receives the high 4 bits; UBRRL receives the low 8 bits.
     *     Because UBRRH shares an address with UCSRC, URSEL must be 0 when
     *     writing UBRRH. The assignment below writes only the divisor bits.
     */
    UCSRC = ucsrc_value;
    UBRRH = (uint8)(ubrr_value >> 8);
    UBRRL = (uint8)(ubrr_value & 0xFFU);

    /*
     * Apply the U2X setting in UCSRA.
     * The prepared value writes U2X only; other status flags are written as zero.
     */
    UCSRA = ucsra_value;

    /* Apply the receiver, transmitter, and receive-interrupt settings. */
    UCSRB = ucsrb_value;

    /*
     * Reset software state:
     *     head == tail means the receive buffer starts empty.
     *     Both counters start at zero for a fresh initialization.
     */
    rx_head = 0U;
    rx_tail = 0U;
    rx_overflow_count = 0U;
    rx_error_count = 0U;
}

void UART_sendByte(const uint8 data)
{
    /*
     * UCSRA.UDRE — USART Data Register Empty:
     *     UDRE = 1 means UDR can accept a new transmit byte.
     *     UDRE = 0 means the data register is not ready for another byte.
     *
     * BIT_IS_CLEAR is provided by common_macros.h and checks whether a bit is 0.
     * This loop is polling: the CPU waits here until the UART hardware is ready.
     */
    while (BIT_IS_CLEAR(UCSRA, UDRE))
    {
        /* Wait for the UART data register to become empty. */
    }

    /*
     * UDR — USART Data Register:
     *     Writing the byte to UDR starts or queues its transmission.
     *     The UART hardware generates the start bit, data bits, parity (if enabled),
     *     and stop bit(s) according to the configuration.
     */
    UDR = data;
}

void UART_sendString(const uint8 *Str)
{
    /*
     * i:
     *     Index used to visit each character in the string in order.
     *     uint16 is used for the index rather than uint8 so the index has more range.
     */
    uint16 i = 0U;

    /* Protect against a null pointer instead of trying to read from address zero. */
    if (Str == 0)
    {
        return;
    }

    /*
     * In C, a string ends with the null character '\0'.
     * Send each character separately; UART_sendByte() waits for hardware readiness.
     */
    while (Str[i] != '\0')
    {
        UART_sendByte(Str[i]);
        i++;
    }
}

uint8 UART_receiveByte(uint8 *data)
{
    /*
     * data:
     *     Address supplied by the application. The received byte is copied into
     *     the variable at this address.
     *
     * This function does not wait for data. The ISR collects bytes in the background.
     * It returns immediately so main() can do other work if the buffer is empty.
     */

    /* A null pointer cannot be used as a destination for the received byte. */
    if (data == 0)
    {
        return 0U;
    }

    /*
     * Empty-buffer condition:
     *     head is the next write position, tail is the oldest unread position.
     *     If they are equal, there are no unread bytes.
     */
    if (rx_head == rx_tail)
    {
        return 0U;
    }

    /*
     * Copy the oldest unread byte into the application's variable.
     * rx_buffer is volatile because the ISR writes to it asynchronously.
     */
    *data = rx_buffer[rx_tail];

    /*
     * Move tail to the next unread slot.
     * The mask wraps the index back to zero when it reaches the end of the buffer.
     */
    rx_tail = (uint8)((rx_tail + 1U) & UART_RX_BUFFER_MASK);

    /* Return 1 to tell the application that a byte was successfully received. */
    return 1U;
}

uint8 UART_getAndClearRxOverflowCount(void)
{
    /*
     * Save the current overflow count before clearing it.
     * The counter increases when the buffer is full and a new byte must be dropped.
     */
    uint8 count = rx_overflow_count;

    /* Clear the counter so the next call reports only new overflow events. */
    rx_overflow_count = 0U;

    /* Return the number of dropped bytes since the previous read/reset. */
    return count;
}

uint8 UART_getAndClearRxErrorCount(void)
{
    /*
     * Save the current receive-error count.
     * The ISR increments this counter when FE, DOR, or PE is set for a received byte.
     */
    uint8 count = rx_error_count;

    /* Reset the counter so future checks can monitor new errors. */
    rx_error_count = 0U;

    /* Return the number of receive-error events since the previous read/reset. */
    return count;
}

/*
 * ISR(USART_RXC_vect):
 *     This is the Receive Complete Interrupt Service Routine for ATmega32.
 *     The CPU enters it when a received byte is ready and receive/global interrupts
 *     are enabled.
 *
 * status:
 *     Snapshot of UCSRA status flags for the received byte.
 *
 * data:
 *     Byte read from UDR. Reading UDR consumes the hardware receive data.
 *
 * next_head:
 *     Candidate position for the next write. It is calculated before writing so
 *     the ISR can detect a full buffer without overwriting unread bytes.
 *
 * Keep ISR work short: read the hardware, record errors if any, and store the byte.
 * String assembly, command comparison, and other application logic belong in main().
 */
ISR(USART_RXC_vect)
{
    uint8 status = UCSRA;
    uint8 data = UDR;
    uint8 next_head;

    /*
     * Error flags:
     *     FE  = Frame Error: stop-bit/frame format was not received correctly.
     *     DOR = Data OverRun: another byte arrived before the previous data was read.
     *     PE  = Parity Error: received parity did not match the configured parity.
     *
     * Count the event and discard this byte rather than passing questionable data
     * to the application. Saturate at 255 so the 8-bit counter does not wrap.
     */
    if (status & ((1U << FE) | (1U << DOR) | (1U << PE)))
    {
        if (rx_error_count < 255U)
        {
            rx_error_count++;
        }
        return;
    }

    /* Find the slot after the current head, wrapping around at the buffer end. */
    next_head = (uint8)((rx_head + 1U) & UART_RX_BUFFER_MASK);

    /*
     * If next_head equals tail, all usable slots contain unread data.
     * Do not overwrite old data; drop this newest byte and record the overflow.
     */
    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = data;
        rx_head = next_head;
    }
    else
    {
        if (rx_overflow_count < 255U)
        {
            rx_overflow_count++;
        }
    }
}
