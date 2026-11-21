/******************************************************************************
 * File: timer.c
 *
 * Purpose:
 *     Implementation of the full Timer driver (Timer0, Timer1, Timer2) with
 *     Normal and Compare modes, interrupts and callbacks.
 *
 * Registers used:
 *     Timer0: TCCR0, TCNT0, OCR0          interrupts: TOIE0 / OCIE0
 *     Timer1: TCCR1A, TCCR1B, TCNT1, OCR1A interrupts: TOIE1 / OCIE1A
 *     Timer2: TCCR2, TCNT2, OCR2          interrupts: TOIE2 / OCIE2
 *     TIMSK holds all the interrupt-enable bits above.
 ******************************************************************************/

#include "timer.h"           /* Timer types and prototypes. */
#include "common_macros.h"   /* Bit-manipulation macros. */
#include <avr/io.h>          /* Timer register names. */
#include <avr/interrupt.h>   /* ISR() macro. */

/*******************************************************************************
 *                           Global Variables                                  *
 *******************************************************************************/

/*
 * One callback per timer, saved by Timer_setCallBack().
 * The pointers are volatile because they are used inside the ISRs.
 */
static void (*volatile g_timer0CallBackPtr)(void) = NULL_PTR;
static void (*volatile g_timer1CallBackPtr)(void) = NULL_PTR;
static void (*volatile g_timer2CallBackPtr)(void) = NULL_PTR;

/*******************************************************************************
 *                       Interrupt Service Routines                            *
 *******************************************************************************/

/*
 * Six ISRs: overflow (Normal mode) and compare match (Compare mode) for each
 * timer. Each one only calls the callback of its timer, if one was set.
 */
ISR(TIMER0_OVF_vect)
{
	if (g_timer0CallBackPtr != NULL_PTR) { (*g_timer0CallBackPtr)(); }
}

ISR(TIMER0_COMP_vect)
{
	if (g_timer0CallBackPtr != NULL_PTR) { (*g_timer0CallBackPtr)(); }
}

ISR(TIMER1_OVF_vect)
{
	if (g_timer1CallBackPtr != NULL_PTR) { (*g_timer1CallBackPtr)(); }
}

ISR(TIMER1_COMPA_vect)
{
	if (g_timer1CallBackPtr != NULL_PTR) { (*g_timer1CallBackPtr)(); }
}

ISR(TIMER2_OVF_vect)
{
	if (g_timer2CallBackPtr != NULL_PTR) { (*g_timer2CallBackPtr)(); }
}

ISR(TIMER2_COMP_vect)
{
	if (g_timer2CallBackPtr != NULL_PTR) { (*g_timer2CallBackPtr)(); }
}

/*******************************************************************************
 *                           Private Functions                                 *
 *******************************************************************************/

/*
 * Timer2 has a different prescaler table (it also offers /32 and /128):
 *   CS22:0 = 001 /1, 010 /8, 100 /64, 110 /256, 111 /1024.
 * Translate the common clock enum into Timer2's bit pattern.
 */
static uint8 Timer2_clockBits(Timer_ClockType clock)
{
	switch (clock)
	{
		case TIMER_F_CPU_CLOCK: return 1;
		case TIMER_F_CPU_8:     return 2;
		case TIMER_F_CPU_64:    return 4;
		case TIMER_F_CPU_256:   return 6;
		case TIMER_F_CPU_1024:  return 7;
		default:                return 0;   /* No clock: timer stopped. */
	}
}

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

void Timer_init(const Timer_ConfigType * Config_Ptr)
{
	switch (Config_Ptr->timer_ID)
	{
	case TIMER0_ID:
		/* Step 1: Start counting from the initial value (8-bit timer). */
		TCNT0 = (uint8)Config_Ptr->timer_InitialValue;

		if (Config_Ptr->timer_mode == TIMER_NORMAL_MODE)
		{
			/*
			 * Step 2 (Normal): FOC0 = 1 (non-PWM mode), WGM01:0 = 00,
			 * OC0 disconnected, clock bits CS02:0 from the configuration.
			 */
			TCCR0 = (1 << FOC0) | (Config_Ptr->timer_clock & 0x07);
			/* Step 3: Enable the overflow interrupt only. */
			TIMSK = (TIMSK & ~(1 << OCIE0)) | (1 << TOIE0);
		}
		else
		{
			/* Step 2 (Compare): set the compare value, then CTC mode (WGM01 = 1). */
			OCR0 = (uint8)Config_Ptr->timer_compare_MatchValue;
			TCCR0 = (1 << FOC0) | (1 << WGM01) | (Config_Ptr->timer_clock & 0x07);
			/* Step 3: Enable the compare-match interrupt only. */
			TIMSK = (TIMSK & ~(1 << TOIE0)) | (1 << OCIE0);
		}
		break;

	case TIMER1_ID:
		/* Step 1: Start counting from the initial value (16-bit timer). */
		TCNT1 = Config_Ptr->timer_InitialValue;

		/* Step 2: Non-PWM mode, OC1A/OC1B disconnected (WGM11:10 = 00). */
		TCCR1A = (1 << FOC1A) | (1 << FOC1B);

		if (Config_Ptr->timer_mode == TIMER_NORMAL_MODE)
		{
			/* Step 3 (Normal): WGM13:12 = 00, clock bits CS12:0. */
			TCCR1B = (Config_Ptr->timer_clock & 0x07);
			/* Step 4: Enable the overflow interrupt only. */
			TIMSK = (TIMSK & ~(1 << OCIE1A)) | (1 << TOIE1);
		}
		else
		{
			/* Step 3 (Compare): compare value in OCR1A, CTC mode 4 (WGM12 = 1). */
			OCR1A = Config_Ptr->timer_compare_MatchValue;
			TCCR1B = (1 << WGM12) | (Config_Ptr->timer_clock & 0x07);
			/* Step 4: Enable the compare-match A interrupt only. */
			TIMSK = (TIMSK & ~(1 << TOIE1)) | (1 << OCIE1A);
		}
		break;

	case TIMER2_ID:
		/* Step 1: Start counting from the initial value (8-bit timer). */
		TCNT2 = (uint8)Config_Ptr->timer_InitialValue;

		if (Config_Ptr->timer_mode == TIMER_NORMAL_MODE)
		{
			/* Step 2 (Normal): FOC2 = 1, WGM21:0 = 00, Timer2 clock bits. */
			TCCR2 = (1 << FOC2) | Timer2_clockBits(Config_Ptr->timer_clock);
			/* Step 3: Enable the overflow interrupt only. */
			TIMSK = (TIMSK & ~(1 << OCIE2)) | (1 << TOIE2);
		}
		else
		{
			/* Step 2 (Compare): set the compare value, then CTC mode (WGM21 = 1). */
			OCR2 = (uint8)Config_Ptr->timer_compare_MatchValue;
			TCCR2 = (1 << FOC2) | (1 << WGM21) | Timer2_clockBits(Config_Ptr->timer_clock);
			/* Step 3: Enable the compare-match interrupt only. */
			TIMSK = (TIMSK & ~(1 << TOIE2)) | (1 << OCIE2);
		}
		break;
	}
}

void Timer_deInit(Timer_ID_Type timer_type)
{
	switch (timer_type)
	{
	case TIMER0_ID:
		/* Stop the clock, clear the registers, disable both interrupts, remove the callback. */
		TCCR0 = 0;
		TCNT0 = 0;
		OCR0 = 0;
		TIMSK &= ~((1 << TOIE0) | (1 << OCIE0));
		g_timer0CallBackPtr = NULL_PTR;
		break;

	case TIMER1_ID:
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1 = 0;
		OCR1A = 0;
		TIMSK &= ~((1 << TOIE1) | (1 << OCIE1A));
		g_timer1CallBackPtr = NULL_PTR;
		break;

	case TIMER2_ID:
		TCCR2 = 0;
		TCNT2 = 0;
		OCR2 = 0;
		TIMSK &= ~((1 << TOIE2) | (1 << OCIE2));
		g_timer2CallBackPtr = NULL_PTR;
		break;
	}
}

void Timer_setCallBack(void(*a_ptr)(void), Timer_ID_Type a_timer_ID)
{
	/* Save the callback in the variable that belongs to the selected timer. */
	switch (a_timer_ID)
	{
	case TIMER0_ID: g_timer0CallBackPtr = a_ptr; break;
	case TIMER1_ID: g_timer1CallBackPtr = a_ptr; break;
	case TIMER2_ID: g_timer2CallBackPtr = a_ptr; break;
	}
}
