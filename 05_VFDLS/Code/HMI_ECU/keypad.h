/******************************************************************************
 * File: keypad.h
 *
 * Purpose:
 *     Configuration and public interface of the 4x4 keypad driver.
 *     The HMI ECU reads the driver's menu choices from this keypad.
 *
 * HMI ECU wiring, as in the Proteus schematic:
 *     Rows    A..D -> PB0..PB3
 *     Columns 1..4 -> PB4..PB7 (each with an external 10k pull-up to VCC)
 *
 * The LCD uses PORTA/PORTC and the UART uses PD0/PD1, so nothing overlaps.
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "std_types.h"   /* uint8 type. */
#include "gpio.h"        /* Port/pin IDs used in the configuration below. */

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* Keypad size: 4 rows x 4 columns. */
#define KEYPAD_NUM_COLS                   4
#define KEYPAD_NUM_ROWS                   4

/*
 * Rows and columns each use consecutive pins of one port,
 * starting from the "first" pin below.
 */
#define KEYPAD_ROW_PORT_ID                PORTB_ID
#define KEYPAD_FIRST_ROW_PIN_ID           PIN0_ID

#define KEYPAD_COL_PORT_ID                PORTB_ID
#define KEYPAD_FIRST_COL_PIN_ID           PIN4_ID

/*
 * Button logic: the active row is driven LOW, and the columns are inputs
 * with pull-up resistors. A pressed button connects its column to the LOW
 * row, so the column reads LOW.
 */
#define KEYPAD_BUTTON_PRESSED            LOGIC_LOW
#define KEYPAD_BUTTON_RELEASED           LOGIC_HIGH

/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

/*
 * Wait until a button is pressed and return its value.
 * Number buttons return 0..9; the other buttons return the ASCII code of
 * their symbol ('%', '*', '-', '=', '+') or 13 for the "ON/C" (Enter) button.
 * Note: this function blocks until a key is pressed.
 */
uint8 KEYPAD_getPressedKey(void);

#endif /* KEYPAD_H_ */
