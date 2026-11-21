/******************************************************************************
 * File: dc_motor.c
 *
 * Purpose:
 *     Implementation of the multi-motor DC motor driver.
 *     The pins of every motor are kept in a small table, so the same code
 *     controls any motor by its ID.
 ******************************************************************************/

#include "dc_motor.h"   /* Motor configuration, types and prototypes. */
#include "gpio.h"       /* GPIO functions for the direction pins. */
#include "pwm.h"        /* Timer0 PWM for the speed. */

/* Port and the two direction pins of one motor. */
typedef struct
{
	uint8 port;
	uint8 in1;
	uint8 in2;
}DC_Motor_Pins;

/* Pin table: entry 0 = DC_MOTOR_1, entry 1 = DC_MOTOR_2 (filled from dc_motor.h). */
static const DC_Motor_Pins g_motors[DC_MOTORS_COUNT] =
{
	{DC_MOTOR1_PORT_ID, DC_MOTOR1_IN1_PIN_ID, DC_MOTOR1_IN2_PIN_ID},
	{DC_MOTOR2_PORT_ID, DC_MOTOR2_IN1_PIN_ID, DC_MOTOR2_IN2_PIN_ID}
};

void DcMotor_Init(void)
{
	uint8 i;

	for (i = 0; i < DC_MOTORS_COUNT; i++)
	{
		/* Step 1: Configure both direction pins of this motor as outputs. */
		GPIO_setupPinDirection(g_motors[i].port, g_motors[i].in1, PIN_OUTPUT);
		GPIO_setupPinDirection(g_motors[i].port, g_motors[i].in2, PIN_OUTPUT);

		/* Step 2: Both pins LOW -> the motor starts stopped. */
		GPIO_writePin(g_motors[i].port, g_motors[i].in1, LOGIC_LOW);
		GPIO_writePin(g_motors[i].port, g_motors[i].in2, LOGIC_LOW);
	}
}

void DcMotor_Rotate(DC_Motor_ID motor, DC_Motor_state state, uint8 speed)
{
	/* Step 1: Ignore an invalid motor ID. */
	if (motor >= DC_MOTORS_COUNT)
	{
		return;
	}

	/*
	 * Step 2: Set the direction pins from the state:
	 *   bit 0 of the state -> IN1, bit 1 of the state -> IN2.
	 *   stop = 00, clockwise = 01 (IN1 HIGH), anticlockwise = 10 (IN2 HIGH).
	 */
	GPIO_writePin(g_motors[motor].port, g_motors[motor].in1, (state & 0x01) ? LOGIC_HIGH : LOGIC_LOW);
	GPIO_writePin(g_motors[motor].port, g_motors[motor].in2, (state & 0x02) ? LOGIC_HIGH : LOGIC_LOW);

	/* Step 3: Set the speed through the Timer0 PWM signal on OC0. */
	PWM_Timer0_Start(speed);
}
