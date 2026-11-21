/*
 * File: uart.h
 * Purpose:
 *     Public interface of the ATmega32 UART driver.
 *
 * Driver design:
 *     - Receiving is interrupt-based.
 *     - Received bytes are stored in a software ring buffer.
 *     - Sending is polling-based.
 *     - Supported data sizes are 5, 6, 7, and 8 bits. 9-bit mode is not supported.
 */

#ifndef UART_H_
#define UART_H_

#include "std_types.h"

/*
 * UART_BitDataType:
 * Selects the number of data bits transmitted in each UART frame.
 * The UART frame also contains the start bit, optional parity bit, and stop bit(s).
 */
typedef enum
{
    UART_DATA_BITS_5 = 0,  /* 5 data bits per frame. */
    UART_DATA_BITS_6,      /* 6 data bits per frame. */
    UART_DATA_BITS_7,      /* 7 data bits per frame. */
    UART_DATA_BITS_8       /* 8 data bits per frame; commonly used for ASCII text. */
} UART_BitDataType;

/*
 * UART_ParityType:
 * Parity adds one bit that can help detect some transmission errors.
 * It must be configured the same way on both communicating devices.
 */
typedef enum
{
    UART_PARITY_DISABLED = 0, /* No parity bit is transmitted or checked. */
    UART_PARITY_EVEN,         /* Parity bit makes the number of 1s even. */
    UART_PARITY_ODD           /* Parity bit makes the number of 1s odd. */
} UART_ParityType;

/*
 * UART_StopBitType:
 * Stop bits mark the end of a UART frame.
 * Both devices must use the same stop-bit setting.
 */
typedef enum
{
    UART_STOP_BIT_1 = 0, /* Use one stop bit. */
    UART_STOP_BIT_2      /* Use two stop bits. */
} UART_StopBitType;

/*
 * UART_BaudRateType:
 * Communication speed in bits per second, e.g. 9600.
 * uint32 is used because common baud rates (57600, 115200) do not fit in 16 bits.
 */
typedef uint32 UART_BaudRateType;

/*
 * UART_ConfigType:
 * Groups all UART settings in one structure, so initialization can be configured
 * from the application without editing the driver source file.
 * The members and their order follow the project specification:
 *
 * bit_data:  Number of data bits per frame.
 * parity:    Parity mode.
 * stop_bit:  Number of stop bits.
 * baud_rate: Required communication speed, e.g. 9600.
 *
 * The driver always uses the double-speed mode (U2X = 1), which gives a more
 * accurate baud rate at common clock frequencies such as 8 MHz.
 */
typedef struct
{
    UART_BitDataType bit_data;
    UART_ParityType parity;
    UART_StopBitType stop_bit;
    UART_BaudRateType baud_rate;
} UART_ConfigType;

/*
 * UART_init:
 * Configures the UART registers and enables the receiver, transmitter,
 * and receive-complete interrupt.
 *
 * Important:
 * The AVR global interrupt flag must also be enabled from main(), usually by
 * calling sei() after initializing the peripherals.
 */
void UART_init(const UART_ConfigType *Config_Ptr);

/*
 * UART_sendByte:
 * Sends one byte.
 * The function waits until the hardware transmit data register is empty.
 * Therefore, transmission uses polling.
 */
void UART_sendByte(const uint8 data);

/*
 * UART_sendString:
 * Sends a null-terminated C string, one character at a time.
 * The string must end with '\0'. This function does not add a message delimiter
 * such as '#'; the application should send that separately if its protocol needs it.
 */
void UART_sendString(const uint8 *Str);

/*
 * UART_receiveByte:
 * Checks whether a byte is waiting in the driver's software receive buffer.
 *
 * data: Address of the variable where the received byte will be copied.
 *
 * Return value:
 *     1: A byte was available and copied into *data.
 *     0: No byte was available, or data was a null pointer.
 *
 * Although the function has a simple name, it does not wait for a byte.
 * The receive interrupt places incoming bytes in the buffer; main() can call
 * this function repeatedly to fetch bytes when they are available.
 */
uint8 UART_receiveByte(uint8 *data);

/*
 * UART_getAndClearRxOverflowCount:
 * Returns the number of received bytes dropped because the software buffer was full,
 * then resets that counter to zero.
 */
uint8 UART_getAndClearRxOverflowCount(void);

/*
 * UART_getAndClearRxErrorCount:
 * Returns the number of UART receive events that had a hardware error
 * (frame error, data overrun, or parity error), then resets the counter to zero.
 */
uint8 UART_getAndClearRxErrorCount(void);

#endif /* UART_H_ */
