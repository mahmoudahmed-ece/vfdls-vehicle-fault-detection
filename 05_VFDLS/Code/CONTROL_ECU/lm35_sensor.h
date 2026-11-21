/******************************************************************************
 * File: lm35_sensor.h
 *
 * Purpose:
 *     Public interface and configuration of the LM35 temperature sensor
 *     driver. The Control ECU uses it to monitor the engine temperature
 *     (fault P002 when the temperature is above 90 C).
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef LM35_SENSOR_H_
#define LM35_SENSOR_H_

#include "std_types.h"   /* uint8 type. */

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* ADC channel connected to the LM35 output (channel 1 = PA1). */
#define SENSOR_CHANNEL_ID          1

/*
 * LM35 output: 10 mV per degree Celsius.
 * At the maximum rated temperature (150 C) the output is 1.5 V.
 */
#define SENSOR_MAX_VOLT_VALUE      1.5
#define SENSOR_MAX_TEMPERATURE     150

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/* Read the LM35 through the ADC and return the temperature in degrees Celsius. */
uint8 LM35_getTemperature(void);

#endif /* LM35_SENSOR_H_ */
