/******************************************************************************
 * File: pwm.h
 *
 * Purpose:
 *     Public interface of the PWM driver (Timer0, Fast PWM, non-inverting).
 *     Used by the Control ECU to drive the window motors at full speed.
 *
 * Output pin: OC0 = PB3 (fixed by the ATmega32).
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef PWM_H_
#define PWM_H_

#include "std_types.h"   /* uint8 type. */

/*
 * Start Timer0 in Fast PWM, non-inverting mode:
 *   - Prescaler F_CPU/64 -> PWM frequency = 8 MHz / (64 * 256) = about 488 Hz.
 *   - OC0 (PB3) is configured as the output pin.
 * duty_cycle : percentage from 0 to 100.
 */
void PWM_Timer0_Start(uint8 duty_cycle);

#endif /* PWM_H_ */
