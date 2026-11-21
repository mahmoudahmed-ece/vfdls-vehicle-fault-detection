/******************************************************************************
 * File: twi.c
 *
 * Purpose:
 *     Implementation of the TWI (I2C) driver in Master mode, using polling.
 *
 * Registers used:
 *     TWBR -> Bit-rate register (sets the SCL frequency).
 *     TWSR -> Status register; bits TWPS1:0 are the bit-rate prescaler.
 *     TWAR -> Own slave address.
 *     TWDR -> Data register (byte to send / byte received).
 *     TWCR -> Control register:
 *             TWINT = start the operation / operation finished flag,
 *             TWSTA = send START, TWSTO = send STOP,
 *             TWEA  = send ACK after receiving, TWEN = enable the module.
 ******************************************************************************/

#include "twi.h"             /* TWI types and prototypes. */
#include "common_macros.h"   /* BIT_IS_CLEAR macro. */
#include <avr/io.h>          /* TWI register names. */

void TWI_init(const TWI_ConfigType * Config_Ptr)
{
	/*
	 * Step 1: Bit-rate prescaler = 1 (TWPS1:0 = 00), so the SCL formula is:
	 *     SCL = F_CPU / (16 + 2 * TWBR)
	 *     =>  TWBR = (F_CPU / SCL - 16) / 2
	 * Example at 8 MHz and 400 kHz: TWBR = (20 - 16) / 2 = 2.
	 */
	TWSR = 0x00;
	TWBR = (uint8)(((F_CPU / Config_Ptr->bit_rate) - 16UL) / 2UL);

	/*
	 * Step 2: Own slave address in bits 7:1 of TWAR.
	 * Bit 0 (TWGCE) = 0, so General Call is disabled.
	 */
	TWAR = (uint8)(Config_Ptr->address << 1);

	/* Step 3: Enable the TWI module. */
	TWCR = (1 << TWEN);
}

void TWI_start(void)
{
	/*
	 * Clear TWINT (by writing 1), request a START condition (TWSTA)
	 * and keep the module enabled (TWEN).
	 */
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

	/* Wait until TWINT is set again: the START condition has been sent. */
	while (BIT_IS_CLEAR(TWCR, TWINT));
}

void TWI_stop(void)
{
	/*
	 * Clear TWINT and request a STOP condition (TWSTO).
	 * No wait is needed: the hardware clears TWSTO when the STOP is sent.
	 */
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void TWI_writeByte(uint8 data)
{
	/* Step 1: Put the byte in the data register. */
	TWDR = data;

	/* Step 2: Clear TWINT to start sending it. */
	TWCR = (1 << TWINT) | (1 << TWEN);

	/* Step 3: Wait until the byte has been sent. */
	while (BIT_IS_CLEAR(TWCR, TWINT));
}

uint8 TWI_readByteWithACK(void)
{
	/* Step 1: Clear TWINT to receive a byte, and enable ACK (TWEA) after it. */
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

	/* Step 2: Wait until the byte has been received. */
	while (BIT_IS_CLEAR(TWCR, TWINT));

	/* Step 3: Return the received byte. */
	return TWDR;
}

uint8 TWI_readByteWithNACK(void)
{
	/* Step 1: Clear TWINT to receive a byte, without ACK (TWEA = 0) -> NACK. */
	TWCR = (1 << TWINT) | (1 << TWEN);

	/* Step 2: Wait until the byte has been received. */
	while (BIT_IS_CLEAR(TWCR, TWINT));

	/* Step 3: Return the received byte. */
	return TWDR;
}

uint8 TWI_getStatus(void)
{
	/* Mask out the two prescaler bits (TWPS1:0) and keep the 5 status bits. */
	return (uint8)(TWSR & 0xF8);
}
