/******************************************************************************
 * File: twi.h
 *
 * Purpose:
 *     Public interface of the TWI (I2C) driver for the ATmega32.
 *     Used by the Control ECU to talk to the external EEPROM (24C16).
 *
 * Hardware connection (fixed by the ATmega32):
 *     SCL -> PC0
 *     SDA -> PC1
 *     Both lines need pull-up resistors (e.g. 4.7 kOhm to +5 V).
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef TWI_H_
#define TWI_H_

#include "std_types.h"   /* uint8 / uint32 types. */

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/*
 * TWI status codes (upper 5 bits of TWSR) for Master Transmitter and
 * Master Receiver modes, taken from the ATmega32 datasheet.
 */
#define TWI_START         0x08 /* START condition has been sent. */
#define TWI_REP_START     0x10 /* Repeated START condition has been sent. */
#define TWI_MT_SLA_W_ACK  0x18 /* Slave address + Write sent, ACK received. */
#define TWI_MT_SLA_R_ACK  0x40 /* Slave address + Read sent, ACK received. */
#define TWI_MT_DATA_ACK   0x28 /* Data byte sent, ACK received from slave. */
#define TWI_MR_DATA_ACK   0x50 /* Data byte received, ACK returned by master. */
#define TWI_MR_DATA_NACK  0x58 /* Data byte received, NACK returned by master. */

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/

/* Own 7-bit slave address of this device (used only if it is addressed as a slave). */
typedef uint8 TWI_AddressType;

/* SCL clock frequency in Hz, e.g. 400000 for Fast mode (400 kHz). */
typedef uint32 TWI_BaudRateType;

/* TWI settings passed to TWI_init(). */
typedef struct
{
	TWI_AddressType address;    /* Own slave address (7 bits). */
	TWI_BaudRateType bit_rate;  /* SCL frequency in Hz. */
}TWI_ConfigType;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/* Set the SCL frequency and the own address, then enable the TWI module. */
void TWI_init(const TWI_ConfigType * Config_Ptr);

/* Send a START condition on the bus. */
void TWI_start(void);

/* Send a STOP condition and release the bus. */
void TWI_stop(void);

/* Send one byte (an address or data) on the bus. */
void TWI_writeByte(uint8 data);

/* Receive one byte and answer with ACK (more bytes will be read). */
uint8 TWI_readByteWithACK(void);

/* Receive one byte and answer with NACK (this is the last byte). */
uint8 TWI_readByteWithNACK(void);

/* Return the current TWI status code (TWSR with the prescaler bits masked out). */
uint8 TWI_getStatus(void);

#endif /* TWI_H_ */
