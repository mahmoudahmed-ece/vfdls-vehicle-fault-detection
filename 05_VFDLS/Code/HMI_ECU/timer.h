/******************************************************************************
 * File: timer.h
 *
 * Purpose:
 *     Public interface of the full Timer driver for the ATmega32
 *     (Timer0, Timer1 and Timer2), with Normal (overflow) and Compare (CTC)
 *     modes. Every timer event calls an application callback from its ISR.
 *     Used by the HMI ECU (Timer1, compare mode) to measure time intervals.
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef TIMER_H_
#define TIMER_H_

#include "std_types.h"   /* uint8 / uint16 types and NULL_PTR. */

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/

/* Which hardware timer to use. */
typedef enum
{
	TIMER0_ID, TIMER1_ID, TIMER2_ID
}Timer_ID_Type;

/*
 * Timer clock source. The driver translates these values into the CSn2:0
 * bits of each timer (Timer2 uses a different bit pattern than Timer0/1).
 */
typedef enum
{
	TIMER_NO_CLOCK, TIMER_F_CPU_CLOCK, TIMER_F_CPU_8, TIMER_F_CPU_64, TIMER_F_CPU_256, TIMER_F_CPU_1024
}Timer_ClockType;

/*
 * Timer mode:
 *   TIMER_NORMAL_MODE  -> counts up to the maximum and interrupts on overflow.
 *   TIMER_COMPARE_MODE -> CTC: counts up to the compare value, interrupts,
 *                         and restarts from 0 automatically.
 */
typedef enum
{
	TIMER_NORMAL_MODE, TIMER_COMPARE_MODE
}Timer_ModeType;

/* Timer settings passed to Timer_init(). */
typedef struct
{
	uint16 timer_InitialValue;        /* Value the counter starts from. */
	uint16 timer_compare_MatchValue;  /* It will be used in compare mode only. */
	Timer_ID_Type timer_ID;           /* Timer0, Timer1 or Timer2. */
	Timer_ClockType timer_clock;      /* Prescaler / clock source. */
	Timer_ModeType timer_mode;        /* Normal or compare mode. */
}Timer_ConfigType;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Initialize and start the selected timer with the given configuration,
 * and enable its overflow or compare-match interrupt.
 * Timer0 and Timer2 are 8-bit, so only the low byte of the initial and
 * compare values is used for them.
 * Global interrupts must be enabled by the application.
 */
void Timer_init(const Timer_ConfigType * Config_Ptr);

/* Stop the selected timer, disable its interrupts and remove its callback. */
void Timer_deInit(Timer_ID_Type timer_type);

/* Save the function that the selected timer's ISR will call on each event. */
void Timer_setCallBack(void(*a_ptr)(void), Timer_ID_Type a_timer_ID);

#endif /* TIMER_H_ */
