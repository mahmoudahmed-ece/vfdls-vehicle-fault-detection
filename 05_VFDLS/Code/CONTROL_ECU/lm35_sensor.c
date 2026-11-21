/******************************************************************************
 * File: lm35_sensor.c
 *
 * Purpose:
 *     Implementation of the LM35 temperature sensor driver.
 ******************************************************************************/

#include "lm35_sensor.h"   /* Sensor configuration and prototype. */
#include "adc.h"           /* ADC_readChannel() and the reference voltage. */

uint8 LM35_getTemperature(void)
{
	uint8 temp_value = 0;   /* Temperature in degrees Celsius. */
	uint16 adc_value = 0;   /* Raw ADC reading (0..1023). */

	/* Step 1: Read the LM35 channel. */
	adc_value = ADC_readChannel(SENSOR_CHANNEL_ID);

	/*
	 * Step 2: Convert the reading to degrees Celsius:
	 *   Voltage     = adc_value * Vref / 1023
	 *   Temperature = Voltage * (150 C / 1.5 V)
	 * Combined into one expression; the cast to uint32 avoids overflow in
	 * the multiplication. With Vref = 2.56 V each step is 0.25 C.
	 */
	temp_value = (uint8)(((uint32)adc_value * SENSOR_MAX_TEMPERATURE * ADC_REF_VOLT_VALUE) /
	                     (ADC_MAXIMUM_VALUE * SENSOR_MAX_VOLT_VALUE));

	/* Step 3: Return the temperature. */
	return temp_value;
}
