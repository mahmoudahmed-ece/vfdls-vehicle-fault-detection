/******************************************************************************
 * File: external_eeprom.c
 *
 * Purpose:
 *     Implementation of the 24C16 external EEPROM driver using the TWI driver.
 *     After every bus step the TWI status is checked; if the EEPROM does not
 *     answer as expected, the function returns ERROR.
 ******************************************************************************/

#include "external_eeprom.h"   /* EEPROM prototypes and result codes. */
#include "twi.h"               /* TWI functions and status codes. */

/*
 * Build the device address byte for the 24C16:
 *     1010 A10 A9 A8 R/W
 * 0xA0 is the fixed part (1010 0000); bits 10:8 of the memory address are
 * shifted into bits 3:1. R/W (bit 0) is added by the caller.
 */
#define EEPROM_DEVICE_ADDRESS(addr)   ((uint8)(0xA0 | (((addr) & 0x0700) >> 7)))

uint8 EEPROM_writeByte(uint16 u16addr, uint8 u8data)
{
	/* Step 1: Send the START condition. */
	TWI_start();
	if (TWI_getStatus() != TWI_START)
		return ERROR;

	/* Step 2: Send the device address with R/W = 0 (write). */
	TWI_writeByte(EEPROM_DEVICE_ADDRESS(u16addr));
	if (TWI_getStatus() != TWI_MT_SLA_W_ACK)
		return ERROR;

	/* Step 3: Send the low 8 bits of the memory address. */
	TWI_writeByte((uint8)(u16addr));
	if (TWI_getStatus() != TWI_MT_DATA_ACK)
		return ERROR;

	/* Step 4: Send the data byte to store. */
	TWI_writeByte(u8data);
	if (TWI_getStatus() != TWI_MT_DATA_ACK)
		return ERROR;

	/* Step 5: Send the STOP condition; the EEPROM now starts its internal write. */
	TWI_stop();

	return SUCCESS;
}

uint8 EEPROM_readByte(uint16 u16addr, uint8 *u8data)
{
	/* Step 1: Send the START condition. */
	TWI_start();
	if (TWI_getStatus() != TWI_START)
		return ERROR;

	/* Step 2: Send the device address with R/W = 0, to write the memory address first. */
	TWI_writeByte(EEPROM_DEVICE_ADDRESS(u16addr));
	if (TWI_getStatus() != TWI_MT_SLA_W_ACK)
		return ERROR;

	/* Step 3: Send the low 8 bits of the memory address. */
	TWI_writeByte((uint8)(u16addr));
	if (TWI_getStatus() != TWI_MT_DATA_ACK)
		return ERROR;

	/* Step 4: Send a Repeated START to switch from writing to reading. */
	TWI_start();
	if (TWI_getStatus() != TWI_REP_START)
		return ERROR;

	/* Step 5: Send the device address again, now with R/W = 1 (read). */
	TWI_writeByte((uint8)(EEPROM_DEVICE_ADDRESS(u16addr) | 1));
	if (TWI_getStatus() != TWI_MT_SLA_R_ACK)
		return ERROR;

	/* Step 6: Read the byte and answer NACK, because it is the only byte we need. */
	*u8data = TWI_readByteWithNACK();
	if (TWI_getStatus() != TWI_MR_DATA_NACK)
		return ERROR;

	/* Step 7: Send the STOP condition. */
	TWI_stop();

	return SUCCESS;
}
