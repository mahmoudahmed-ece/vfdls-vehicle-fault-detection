/******************************************************************************
 * File: vfdls_protocol.h
 *
 * Purpose:
 *     Communication protocol between the HMI ECU and the Control ECU over
 *     the UART. The SAME copy of this file is used in both projects, so the
 *     two ECUs always agree on the command codes and the frame formats.
 *
 * UART settings (both ECUs): 9600 bps, 8 data bits, no parity, 1 stop bit.
 *
 * Communication model: request / response.
 *     - Only the HMI ECU starts a conversation by sending one command byte.
 *     - The Control ECU answers every command with exactly one response frame.
 *
 * Response frames (every frame starts with a header byte and ends with a
 * checksum byte):
 *
 *   1) ACK frame, answer to START / STOP:
 *        [RSP_ACK] [command] [checksum]
 *
 *   2) Values frame, answer to GET_VALUES:
 *        [RSP_VALUES] [status] [temperature] [distance high] [distance low]
 *        [window 1 state] [window 2 state] [active faults] [checksum]
 *
 *   3) Faults frame, answer to GET_FAULTS:
 *        [RSP_FAULTS] [count N] [DTC 1] ... [DTC N] [checksum]
 *
 * Checksum = XOR of all the frame bytes before it (header included).
 * The receiver recalculates it; a different value means a corrupted frame.
 ******************************************************************************/

/* Include guard: prevents this header from being processed twice in one file. */
#ifndef VFDLS_PROTOCOL_H_
#define VFDLS_PROTOCOL_H_

/*******************************************************************************
 *                        Commands (HMI ECU -> Control ECU)                    *
 *******************************************************************************/

#define CMD_START_MONITORING     0xC1   /* Key 1: start reading sensors and detecting faults. */
#define CMD_GET_VALUES           0xC2   /* Key 2: send the latest sensor values. */
#define CMD_GET_FAULTS           0xC3   /* Key 3: send all DTCs saved in the EEPROM. */
#define CMD_STOP_MONITORING      0xC4   /* Key 4: stop monitoring. */

/*******************************************************************************
 *                     Response headers (Control ECU -> HMI ECU)               *
 *******************************************************************************/

#define RSP_ACK                  0xA0   /* Command received and executed. */
#define RSP_VALUES               0xA1   /* Sensor values frame. */
#define RSP_FAULTS               0xA2   /* Saved faults frame. */

/*******************************************************************************
 *                               Frame contents                                *
 *******************************************************************************/

/* Number of bytes in a values frame, header and checksum included. */
#define VALUES_FRAME_SIZE        9

/* Bit 0 of the status byte in the values frame: 1 = monitoring is running. */
#define STATUS_MONITORING_BIT    0

/* Window states sent in the values frame. */
#define WINDOW_CLOSED            0
#define WINDOW_OPEN              1

/*
 * Diagnostic Trouble Codes. Each DTC is saved in the EEPROM as one byte
 * holding its number (P001 -> 1, P002 -> 2).
 */
#define DTC_P001_ACCIDENT_MIGHT_HAPPENED     1   /* Distance < 10 cm. */
#define DTC_P002_ENGINE_HIGH_TEMPERATURE     2   /* Temperature > 90 C. */

/* Bits of the "active faults" byte in the values frame (fault present right now). */
#define ACTIVE_FAULT_P001_BIT    0
#define ACTIVE_FAULT_P002_BIT    1

/* Maximum number of DTCs that can be saved and sent in one faults frame. */
#define MAX_LOGGED_FAULTS        10

#endif /* VFDLS_PROTOCOL_H_ */
