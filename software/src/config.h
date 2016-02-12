/* can-bricklet
 * Copyright (C) 2016 Matthias Bolte <matthias@tinkerforge.com>
 *
 * config.h: CAN Bricklet specific configuration
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#include "bricklib/drivers/board/sam3s/SAM3S.h"
#include "bricklib/drivers/pio/pio.h"

#include "can.h"

#define BOARD_MCK 64000000

#define BRICKLET_FIRMWARE_VERSION_MAJOR 2
#define BRICKLET_FIRMWARE_VERSION_MINOR 0
#define BRICKLET_FIRMWARE_VERSION_REVISION 0

#define BRICKLET_HARDWARE_VERSION_MAJOR 1
#define BRICKLET_HARDWARE_VERSION_MINOR 0
#define BRICKLET_HARDWARE_VERSION_REVISION 0

#define BRICKLET_DEVICE_IDENTIFIER 270

#define SPI_CLK (BS->pin1_ad)
#define SPI_SDI (BS->pin2_da)
#define SPI_SDO (BS->pin3_pwm)
#define SPI_CS  (BS->pin4_io)

#define LOGGING_LEVEL LOGGING_DEBUG
#define DEBUG_BRICKLET 0

#define INVOCATION_IN_BRICKLET_CODE

#define INST_RESET                           0b11000000
#define INST_READ                            0b00000011
#define INST_READ_RX_BUFFER_RXB0SIDH         0b10010000
#define INST_READ_RX_BUFFER_RXB0D0           0b10010010
#define INST_READ_RX_BUFFER_RXB1SIDH         0b10010100
#define INST_READ_RX_BUFFER_RXB1D0           0b10010110
#define INST_WRITE                           0b00000010
#define INST_WRITE_TX_BUFFER_TXB0SIDH        0b01000000
#define INST_WRITE_TX_BUFFER_TXB0D0          0b01000001
#define INST_WRITE_TX_BUFFER_TXB1SIDH        0b01000010
#define INST_WRITE_TX_BUFFER_TXB1D0          0b01000011
#define INST_WRITE_TX_BUFFER_TXB2SIDH        0b01000100
#define INST_WRITE_TX_BUFFER_TXB2D0          0b01000101
#define INST_RTS_TXB0                        0b10000001
#define INST_RTS_TXB1                        0b10000010
#define INST_RTS_TXB2                        0b10000100
#define INST_READ_STATUS                     0b10100000
#define INST_READ_RX_STATUS                  0b10110000
#define INST_WRITE_BITS                      0b00000101

#define INST_READ_STATUS_CANINTF_TX2IF       0b10000000
#define INST_READ_STATUS_TXB2CTRL_TXREQ      0b01000000
#define INST_READ_STATUS_CANINTF_TX1IF       0b00100000
#define INST_READ_STATUS_TXB1CTRL_TXREQ      0b00010000
#define INST_READ_STATUS_CANINTF_TX0IF       0b00001000
#define INST_READ_STATUS_TXB0CTRL_TXREQ      0b00000100
#define INST_READ_STATUS_CANINTF_RX1IF       0b00000010
#define INST_READ_STATUS_CANINTF_RX0IF       0b00000001

#define REG_BFPCTRL                          0x0C
#define REG_TXRTSCTRL                        0x0D
#define REG_CANSTAT                          0x0E
#define REG_CANCTRL                          0x0F
#define REG_TEC                              0x1C
#define REG_REC                              0x1D
#define REG_CNF3                             0x28
#define REG_CNF2                             0x28
#define REG_CNF1                             0x2A
#define REG_CANINTE                          0x2B
#define REG_CANINTF                          0x2C
#define REG_EFLG                             0x2D

#define REG_RXF0SIDH                         0x00
#define REG_RXF0SIDL                         0x01
#define REG_RXF0EID8                         0x02
#define REG_RXF0EID0                         0x03

#define REG_RXF1SIDH                         0x04
#define REG_RXF1SIDL                         0x05
#define REG_RXF1EID8                         0x06
#define REG_RXF1EID0                         0x07

#define REG_RXF2SIDH                         0x08
#define REG_RXF2SIDL                         0x09
#define REG_RXF2EID8                         0x0A
#define REG_RXF2EID0                         0x0B

#define REG_RXF3SIDH                         0x10
#define REG_RXF3SIDL                         0x11
#define REG_RXF3EID8                         0x12
#define REG_RXF3EID0                         0x13

#define REG_RXF4SIDH                         0x14
#define REG_RXF4SIDL                         0x15
#define REG_RXF4EID8                         0x16
#define REG_RXF4EID0                         0x17

#define REG_RXF5SIDH                         0x18
#define REG_RXF5SIDL                         0x19
#define REG_RXF5EID8                         0x1A
#define REG_RXF5EID0                         0x1B

#define REG_RXM0SIDH                         0x20
#define REG_RXM0SIDL                         0x21
#define REG_RXM0EID8                         0x22
#define REG_RXM0EID0                         0x23

#define REG_RXM1SIDH                         0x24
#define REG_RXM1SIDL                         0x25
#define REG_RXM1EID8                         0x26
#define REG_RXM1EID0                         0x27

#define REG_TXB0CTRL                         0x30
#define REG_TXB0SIDH                         0x31

#define REG_TXB1CTRL                         0x40
#define REG_TXB1SIDH                         0x41

#define REG_TXB2CTRL                         0x50
#define REG_TXB2SIDH                         0x51

#define REG_RXB0CTRL                         0x60
#define REG_RXB0SIDH                         0x61

#define REG_RXB1CTRL                         0x70
#define REG_RXB1SIDH                         0x71

#define REG_CANSTAT_OPMOD_mask               0b11100000
#define REG_CANSTAT_OPMOD_NORMAL             0b00000000
#define REG_CANSTAT_OPMOD_SLEEP              0b00100000
#define REG_CANSTAT_OPMOD_LOOPBACK           0b01000000
#define REG_CANSTAT_OPMOD_READ_ONLY          0b01100000
#define REG_CANSTAT_OPMOD_CONFIG             0b10000000

#define REG_CANSTAT_ICOD_mask                0b00001110

#define REG_CANCTRL_REQOP_mask               0b11100000
#define REG_CANCTRL_REQOP_NORMAL             0b00000000
#define REG_CANCTRL_REQOP_SLEEP              0b00100000
#define REG_CANCTRL_REQOP_LOOPBACK           0b01000000
#define REG_CANCTRL_REQOP_READ_ONLY          0b01100000
#define REG_CANCTRL_REQOP_CONFIG             0b10000000
#define REG_CANCTRL_ABAT                     0b00010000
#define REG_CANCTRL_OSM                      0b00001000
#define REG_CANCTRL_CLKEN                    0b00000100
#define REG_CANCTRL_CLKPRE_mask              0b00000011
#define REG_CANCTRL_CLKPRE_offset            0

#define REG_CNF3_PHSEG2_mask                 0b00000111
#define REG_CNF3_PHSEG2_offset               0
#define REG_CNF3_WAKFIL                      0b01000000
#define REG_CNF3_SOF                         0b10000000

#define REG_CNF2_PRSEG_mask                  0b00000111
#define REG_CNF2_PRSEG_offset                0
#define REG_CNF2_PHSEG1_mask                 0b00111000
#define REG_CNF2_PHSEG1_offset               3
#define REG_CNF2_SAM                         0b01000000
#define REG_CNF2_BTLMODE                     0b10000000

#define REG_CNF1_BRP_mask                    0b00111111
#define REG_CNF1_BRP_offset                  0
#define REG_CNF1_SJW_mask                    0b11000000
#define REG_CNF1_SJW_offset                  6

#define REG_CANINTF_MERRF                    0b10000000
#define REG_CANINTF_WAKIF                    0b01000000
#define REG_CANINTF_ERRIF                    0b00100000
#define REG_CANINTF_TX2IF                    0b00010000
#define REG_CANINTF_TX1IF                    0b00001000
#define REG_CANINTF_TX0IF                    0b00000100
#define REG_CANINTF_RX1IF                    0b00000010
#define REG_CANINTF_RX0IF                    0b00000001

#define REG_CANINTE_MERRE                    0b10000000
#define REG_CANINTE_WAKIE                    0b01000000
#define REG_CANINTE_ERRIE                    0b00100000
#define REG_CANINTE_TX2IE                    0b00010000
#define REG_CANINTE_TX1IE                    0b00001000
#define REG_CANINTE_TX0IE                    0b00000100
#define REG_CANINTE_RX1IE                    0b00000010
#define REG_CANINTE_RX0IE                    0b00000001

#define REG_EFLG_RX1OVR                      0b10000000
#define REG_EFLG_RX0OVR                      0b01000000
#define REG_EFLG_TXBO                        0b00100000
#define REG_EFLG_TXEP                        0b00010000
#define REG_EFLG_RXEP                        0b00001000
#define REG_EFLG_TXWAR                       0b00000100
#define REG_EFLG_RXWAR                       0b00000010
#define REG_EFLG_EWARN                       0b00000001

#define REG_TXBnCTRL_ABTF                    0b01000000
#define REG_TXBnCTRL_MLOA                    0b00100000
#define REG_TXBnCTRL_TXERR                   0b00010000
#define REG_TXBnCTRL_TXREQ                   0b00001000
#define REG_TXBnCTRL_TXP_mask                0b00000011
#define REG_TXBnCTRL_TXP_offset              0

#define REG_TXBnSIDL_EXIDE                   0b00001000

#define REG_TXBnDLC_RTR                      0b01000000
#define REG_TXBnDLC_DLC_mask                 0b00001111
#define REG_TXBnDLC_DLC_offset               0

#define REG_RXBnCTRL_RXM_mask                0b01100000
#define REG_RXBnCTRL_RXM_DISABLED            0b01100000
#define REG_RXBnCTRL_RXM_EXTENDED_ONLY       0b01000000
#define REG_RXBnCTRL_RXM_STANDARD_ONLY       0b00100000
#define REG_RXBnCTRL_RXM_BOTH                0b00000000

#define REG_RXB0CTRL_BUKT                    0b00000100

#define REG_RXBnSIDL_SRR                     0b00010000
#define REG_RXBnSIDL_IDE                     0b00001000

#define REG_RXBnDLC_RTR                      0b01000000
#define REG_RXBnDLC_DLC_mask                 0b00001111
#define REG_RXBnDLC_DLC_offset               0

#define BUFFER_COUNT 8
#define BUFFER_LENGTH 13

#define STATUS_FRAME_READ_CALLBACK_ENABLED   0b00000001
#define STATUS_ENTERING_CONFIG_MODE          0b00000010
#define STATUS_LEAVING_CONFIG_MODE           0b00000100
#define STATUS_VALID_TXB0_HEADER             0b00001000

typedef struct {
	uint8_t status;

	uint8_t txb[BUFFER_COUNT][BUFFER_LENGTH];
	uint8_t txb_start;
	uint8_t txb_end;
	uint8_t txb0_header[5];

	uint8_t rxb[BUFFER_COUNT][BUFFER_LENGTH];
	uint8_t rxb_start;
	uint8_t rxb_end;

	uint32_t write_timeout_count;
	uint32_t read_register_overflow_count;
	uint32_t read_buffer_overflow_count;

	uint8_t baud_rate;
	uint8_t transceiver_mode;
	int32_t write_timeout; // > 0 (timeout), 0 (infinite), < 0 (one-shot)
	int32_t write_timeout_counter;

	uint8_t filter_mode;
	uint32_t filter_mask;
	uint32_t filter1;
	uint32_t filter2;
} __attribute__((__packed__)) BrickContext;

#endif
