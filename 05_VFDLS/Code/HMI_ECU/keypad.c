/******************************************************************************
 * File: keypad.c
 *
 * Purpose:
 *     Implementation of the 4x4 keypad driver (row scanning).
 *
 * How the scan works:
 *     1. All row and column pins start as inputs.
 *     2. One row at a time is made an output and driven LOW.
 *     3. Every column is read: a LOW column means the button at
 *        (this row, this column) is pressed.
 *     4. The row goes back to input and the next row is tested.
 ******************************************************************************/

#include "keypad.h"          /* Keypad configuration and prototype. */
#include "gpio.h"            /* GPIO functions for the row/column pins. */
#include <util/delay.h>      /* _delay_ms() / _delay_us() (use F_CPU). */

/* Time the column lines get to settle after a row is driven LOW. */
#define KEYPAD_SETTLE_TIME_US    500

/*******************************************************************************
 *                      Functions Prototypes (Private)                         *
 *******************************************************************************/

#if (KEYPAD_NUM_COLS == 3)
/* Convert the button number (1..12) into the value printed on a 4x3 keypad. */
static uint8 KEYPAD_4x3_adjustKeyNumber(uint8 button_number);
#elif (KEYPAD_NUM_COLS == 4)
/* Convert the button number (1..16) into the value printed on a 4x4 keypad. */
static uint8 KEYPAD_4x4_adjustKeyNumber(uint8 button_number);
#endif

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

uint8 KEYPAD_getPressedKey(void)
{
	uint8 col, row;   /* Loop counters for the scan. */

	/* Step 1: Make all row pins inputs (released state). */
	for (row = 0; row < KEYPAD_NUM_ROWS; row++)
	{
		GPIO_setupPinDirection(KEYPAD_ROW_PORT_ID, KEYPAD_FIRST_ROW_PIN_ID + row, PIN_INPUT);
	}

	/*
	 * Step 2: Make all column pins inputs and enable their internal pull-up
	 * resistors (writing HIGH to an input pin enables its pull-up), so an
	 * unpressed column always reads HIGH.
	 */
	for (col = 0; col < KEYPAD_NUM_COLS; col++)
	{
		GPIO_setupPinDirection(KEYPAD_COL_PORT_ID, KEYPAD_FIRST_COL_PIN_ID + col, PIN_INPUT);
		GPIO_writePin(KEYPAD_COL_PORT_ID, KEYPAD_FIRST_COL_PIN_ID + col, LOGIC_HIGH);
	}

	/* Step 3: Keep scanning until a button is found. */
	while (1)
	{
		for (row = 0; row < KEYPAD_NUM_ROWS; row++)
		{
			/* Step 3.1: Only the current row becomes an output and is driven LOW. */
			GPIO_setupPinDirection(KEYPAD_ROW_PORT_ID, KEYPAD_FIRST_ROW_PIN_ID + row, PIN_OUTPUT);
			GPIO_writePin(KEYPAD_ROW_PORT_ID, KEYPAD_FIRST_ROW_PIN_ID + row, KEYPAD_BUTTON_PRESSED);

			/*
			 * Step 3.2: Give the column lines time to settle before reading them.
			 * The columns are pulled up by resistors; when a button connects a
			 * column to the LOW row, the line needs a short time to fall
			 * (wire capacitance on real hardware, and the analog resistors in
			 * Proteus). Reading immediately could miss the press.
			 */
			_delay_us(KEYPAD_SETTLE_TIME_US);

			/* Step 3.3: Check every column of this row. */
			for (col = 0; col < KEYPAD_NUM_COLS; col++)
			{
				if (GPIO_readPin(KEYPAD_COL_PORT_ID, KEYPAD_FIRST_COL_PIN_ID + col) == KEYPAD_BUTTON_PRESSED)
				{
					/*
					 * Button found. Its number counts from 1, row by row:
					 * number = row * columns + column + 1.
					 * Release the row first, then return the printed value.
					 */
					GPIO_setupPinDirection(KEYPAD_ROW_PORT_ID, KEYPAD_FIRST_ROW_PIN_ID + row, PIN_INPUT);
#if (KEYPAD_NUM_COLS == 3)
					return KEYPAD_4x3_adjustKeyNumber((row * KEYPAD_NUM_COLS) + col + 1);
#elif (KEYPAD_NUM_COLS == 4)
					return KEYPAD_4x4_adjustKeyNumber((row * KEYPAD_NUM_COLS) + col + 1);
#endif
				}
			}

			/* Step 3.4: Release this row (back to input) before testing the next one. */
			GPIO_setupPinDirection(KEYPAD_ROW_PORT_ID, KEYPAD_FIRST_ROW_PIN_ID + row, PIN_INPUT);

			/* Small delay so the scan does not load the CPU in Proteus. */
			_delay_ms(10);
		}
	}
}

#if (KEYPAD_NUM_COLS == 3)

static uint8 KEYPAD_4x3_adjustKeyNumber(uint8 button_number)
{
	uint8 keypad_button = 0;   /* Value printed on the button. */

	/* Buttons 1..9 are already correct; only the bottom row needs mapping. */
	switch (button_number)
	{
		case 10: keypad_button = '*'; break;   /* ASCII code of '*'. */
		case 11: keypad_button = 0;   break;   /* The "0" button. */
		case 12: keypad_button = '#'; break;   /* ASCII code of '#'. */
		default: keypad_button = button_number; break;
	}
	return keypad_button;
}

#elif (KEYPAD_NUM_COLS == 4)

static uint8 KEYPAD_4x4_adjustKeyNumber(uint8 button_number)
{
	uint8 keypad_button = 0;   /* Value printed on the button. */

	/*
	 * Layout of the 4x4 keypad used in Proteus:
	 *   7   8   9   %
	 *   4   5   6   *
	 *   1   2   3   -
	 *  ON   0   =   +
	 */
	switch (button_number)
	{
		case 1:  keypad_button = 7;   break;
		case 2:  keypad_button = 8;   break;
		case 3:  keypad_button = 9;   break;
		case 4:  keypad_button = '%'; break;   /* ASCII code of '%'. */
		case 5:  keypad_button = 4;   break;
		case 6:  keypad_button = 5;   break;
		case 7:  keypad_button = 6;   break;
		case 8:  keypad_button = '*'; break;   /* ASCII code of '*'. */
		case 9:  keypad_button = 1;   break;
		case 10: keypad_button = 2;   break;
		case 11: keypad_button = 3;   break;
		case 12: keypad_button = '-'; break;   /* ASCII code of '-'. */
		case 13: keypad_button = 13;  break;   /* ASCII code of Enter (ON/C button). */
		case 14: keypad_button = 0;   break;
		case 15: keypad_button = '='; break;   /* ASCII code of '='. */
		case 16: keypad_button = '+'; break;   /* ASCII code of '+'. */
		default: keypad_button = button_number; break;
	}
	return keypad_button;
}

#endif
