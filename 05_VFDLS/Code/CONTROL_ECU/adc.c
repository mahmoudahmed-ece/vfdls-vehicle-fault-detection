/******************************************************************************
 * File: adc.c
 *
 * Purpose:
 *     Implementation of the ADC driver in polling mode.
 *
 * Registers used:
 *     ADMUX  -> REFS1:0 reference selection, MUX4:0 channel selection.
 *     ADCSRA -> ADEN enable, ADSC start conversion, ADIF conversion done,
 *               ADPS2:0 prescaler.
 *     ADC    -> 10-bit conversion result (ADCL + ADCH).
 ******************************************************************************/

#include "adc.h"             /* ADC types and prototypes. */
#include "common_macros.h"   /* SET_BIT / BIT_IS_CLEAR macros. */
#include <avr/io.h>          /* ADC register names. */

void ADC_init(const ADC_ConfigType * Config_Ptr)
{
	/*
	 * Step 1: ADMUX:
	 *   REFS1:0 (bits 7:6) = reference from the configuration.
	 *   ADLAR = 0 -> the result is right-adjusted.
	 *   MUX4:0 = 0 -> channel 0 for now; ADC_readChannel() selects the real one.
	 */
	ADMUX = (uint8)((Config_Ptr->ref_volt & 0x03) << 6);

	/*
	 * Step 2: ADCSRA:
	 *   ADEN = 1 -> enable the ADC.
	 *   ADIE = 0 -> no interrupt, the driver uses polling.
	 *   ADPS2:0 = prescaler from the configuration.
	 */
	ADCSRA = (1 << ADEN) | (Config_Ptr->prescaler & 0x07);
}

uint16 ADC_readChannel(uint8 channel_num)
{
	/* Step 1: Keep the reference bits and write the channel number into MUX4:0. */
	ADMUX = (ADMUX & 0xE0) | (channel_num & 0x07);

	/* Step 2: Start the conversion. */
	SET_BIT(ADCSRA, ADSC);

	/* Step 3: Wait until the conversion is complete (ADIF = 1). */
	while (BIT_IS_CLEAR(ADCSRA, ADIF));

	/* Step 4: Clear ADIF by writing 1 to it, ready for the next conversion. */
	SET_BIT(ADCSRA, ADIF);

	/* Step 5: Return the 10-bit result. */
	return ADC;
}
