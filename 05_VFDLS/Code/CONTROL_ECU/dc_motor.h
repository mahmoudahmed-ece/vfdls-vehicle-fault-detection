/******************************************************************************
 * File: dc_motor.h
 *
 * Purpose:
 *     Public interface and configuration of the DC motor driver.
 *     Based on the Smart Home driver, extended to control more than one
 *     motor: the Control ECU uses two motors for the two car windows.
 *
 * Hardware connection (H-bridge, e.g. L293D):
 *     Window 1 motor: IN1 -> PB0, IN2 -> PB1
 *     Window 2 motor: IN3 -> PB4, IN4 -> PB5
 *     EN1 and EN2     -> PB3 (OC0, Timer0 PWM, shared by both motors)
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include "std_types.h"   /* uint8 type. */
#include "gpio.h"        /* Port/pin IDs used in the configuration below. */

/*******************************************************************************
 *                                Configuration                                *
 *******************************************************************************/

/* Number of motors handled by the driver. */
#define DC_MOTORS_COUNT          2

/* Direction pins of each motor (change these to move a motor to other pins). */
#define DC_MOTOR1_PORT_ID        PORTB_ID
#define DC_MOTOR1_IN1_PIN_ID     PIN0_ID
#define DC_MOTOR1_IN2_PIN_ID     PIN1_ID

#define DC_MOTOR2_PORT_ID        PORTB_ID
#define DC_MOTOR2_IN1_PIN_ID     PIN4_ID
#define DC_MOTOR2_IN2_PIN_ID     PIN5_ID

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/

/* Motor identifiers. */
typedef enum
{
	DC_MOTOR_1, DC_MOTOR_2
}DC_Motor_ID;

/*
 * Motor states:
 *   DC_Motor_stop          -> IN1 = 0, IN2 = 0
 *   DC_Motor_clockwise     -> IN1 = 1, IN2 = 0
 *   DC_Motor_Anticlockwise -> IN1 = 0, IN2 = 1
 */
typedef enum
{
	DC_Motor_stop, DC_Motor_clockwise, DC_Motor_Anticlockwise
}DC_Motor_state;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/* Configure the direction pins of all motors as outputs and stop them. */
void DcMotor_Init(void);

/*
 * Set the direction of one motor and the PWM speed.
 * motor : DC_MOTOR_1 or DC_MOTOR_2.
 * state : stop / clockwise / anticlockwise.
 * speed : duty cycle in percent (0..100). The PWM output is shared, so the
 *         speed applies to both motors (the project always uses 100 %).
 */
void DcMotor_Rotate(DC_Motor_ID motor, DC_Motor_state state, uint8 speed);

#endif /* DC_MOTOR_H_ */
