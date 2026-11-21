/******************************************************************************
 * File: adc.h
 *
 * Purpose:
 *     Public interface of the ADC driver for the ATmega32 (polling mode).
 *     Used by the Control ECU to read the LM35 temperature sensor.
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef ADC_H_
#define ADC_H_

#include "std_types.h"   /* uint8 / uint16 types. */

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* 10-bit ADC: readings go from 0 to 1023. */
#define ADC_MAXIMUM_VALUE    1023

/*
 * Reference voltage in volts. It must match the reference selected in
 * ADC_init(); the sensor drivers use it to convert readings into volts.
 */
#define ADC_REF_VOLT_VALUE   2.56

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/

/*
 * Reference voltage. The values match the REFS1:0 bits in ADMUX:
 *   0 = AREF pin, 1 = AVCC, 3 = internal 2.56 V.
 */
typedef enum
{
	ADC_AREF = 0, ADC_AVCC = 1, ADC_INTERNAL_2_56V = 3
}ADC_ReferenceVolatge;

/*
 * ADC clock prescaler. The values match the ADPS2:0 bits in ADCSRA.
 * The ADC clock must be between 50 and 200 kHz for full accuracy:
 * at 8 MHz, prescaler 64 gives 125 kHz.
 */
typedef enum
{
	ADC_PRESCALER_2 = 1, ADC_PRESCALER_4, ADC_PRESCALER_8, ADC_PRESCALER_16,
	ADC_PRESCALER_32, ADC_PRESCALER_64, ADC_PRESCALER_128
}ADC_Prescaler;

/* ADC settings passed to ADC_init(). */
typedef struct
{
	ADC_ReferenceVolatge ref_volt;   /* Reference voltage source. */
	ADC_Prescaler prescaler;         /* ADC clock prescaler. */
}ADC_ConfigType;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/* Select the reference and the prescaler, then enable the ADC. */
void ADC_init(const ADC_ConfigType * Config_Ptr);

/* Convert the given channel (0..7) and return the 10-bit result (blocking). */
uint16 ADC_readChannel(uint8 channel_num);

#endif /* ADC_H_ */
