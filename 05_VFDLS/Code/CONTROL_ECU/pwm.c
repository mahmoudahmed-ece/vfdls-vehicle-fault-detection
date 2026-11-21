/******************************************************************************
 * File: pwm.c
 *
 * Purpose:
 *     Implementation of the PWM driver using Timer0 in Fast PWM mode.
 *     TCNT0 counts 0..255 repeatedly; OC0 is HIGH from 0 until TCNT0 reaches
 *     OCR0, so a larger OCR0 means a higher duty cycle.
 ******************************************************************************/

#include "pwm.h"             /* Function prototype. */
#include "gpio.h"            /* GPIO driver, to make OC0 (PB3) an output. */
#include <avr/io.h>          /* Timer0 register names. */

void PWM_Timer0_Start(uint8 duty_cycle)
{
	/* Step 1: Limit the duty cycle to 100 %. */
	if (duty_cycle > 100)
	{
		duty_cycle = 100;
	}

	/* Step 2: Start the counter from 0. */
	TCNT0 = 0;

	/*
	 * Step 3: Convert the duty cycle (0..100 %) into the compare value (0..255).
	 * Example: 100 % -> 255 (always HIGH), 50 % -> 127.
	 */
	OCR0 = (uint8)(((uint16)duty_cycle * 255) / 100);

	/* Step 4: Configure OC0 (PB3) as an output so the PWM signal appears on it. */
	GPIO_setupPinDirection(PORTB_ID, PIN3_ID, PIN_OUTPUT);

	/*
	 * Step 5: Configure and start Timer0 in one write to TCCR0:
	 *   WGM00 + WGM01 -> Fast PWM mode.
	 *   COM01         -> non-inverting (clear OC0 on compare match).
	 *   CS01 + CS00   -> clock = F_CPU / 64.
	 */
	TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01) | (1 << CS00);
}
