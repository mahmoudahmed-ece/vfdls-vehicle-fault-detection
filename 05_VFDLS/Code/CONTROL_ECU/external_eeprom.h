/******************************************************************************
 * File: external_eeprom.h
 *
 * Purpose:
 *     Public interface of the external EEPROM driver (24C16, 2 KB) over I2C.
 *     The Control ECU stores the Diagnostic Trouble Codes (DTCs) in it, so
 *     they survive a power cycle.
 *
 * 24C16 addressing:
 *     2048 bytes = 8 blocks of 256 bytes. An 11-bit memory address is split:
 *       bits 10:8 -> go inside the device address byte: 1010 A10 A9 A8 R/W
 *       bits  7:0 -> sent as a separate word-address byte.
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef EXTERNAL_EEPROM_H_
#define EXTERNAL_EEPROM_H_

#include "std_types.h"   /* uint8 / uint16 types. */

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

#define ERROR      0   /* The operation failed (no ACK / wrong status). */
#define SUCCESS    1   /* The operation completed correctly. */

/* Last valid memory address of the 24C16 (2 KB). */
#define EEPROM_MAX_ADDRESS   0x07FF

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * Write one byte to the given memory address (0x000 .. 0x7FF).
 * Returns SUCCESS or ERROR.
 * Note: the EEPROM needs about 10 ms to finish writing internally before
 * the next write (or read) can be accepted.
 */
uint8 EEPROM_writeByte(uint16 u16addr, uint8 u8data);

/*
 * Read one byte from the given memory address (0x000 .. 0x7FF) into *u8data.
 * Returns SUCCESS or ERROR.
 */
uint8 EEPROM_readByte(uint16 u16addr, uint8 *u8data);

#endif /* EXTERNAL_EEPROM_H_ */
