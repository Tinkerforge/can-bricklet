/* can-bricklet
 * Copyright (C) 2016 Matthias Bolte <matthias@tinkerforge.com>
 *
 * can.h: Implementation of CAN Bricklet messages
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

#ifndef CAN_H
#define CAN_H

#include <stdint.h>

#include "bricklib/com/com_common.h"

#define FID_WRITE_FRAME 1
#define FID_READ_FRAME 2
#define FID_SET_CONFIGURATION 3
#define FID_GET_CONFIGURATION 4
#define FID_ERROR 5

#define BAUD_RATE_10000 0
#define BAUD_RATE_20000 1
#define BAUD_RATE_50000 2
#define BAUD_RATE_125000 3
#define BAUD_RATE_250000 4
#define BAUD_RATE_500000 5
#define BAUD_RATE_800000 6
#define BAUD_RATE_1000000 7

#define TRANSCEIVER_MODE_NORMAL 0
#define TRANSCEIVER_MODE_LOOPBACK 1
#define TRANSCEIVER_MODE_READ_ONLY 2

#define FRAME_TYPE_STANDARD_DATA 0
#define FRAME_TYPE_STANDARD_REMOTE 1
#define FRAME_TYPE_EXTENDED_DATA 2
#define FRAME_TYPE_EXTENDED_REMOTE 3

#define FILTER_MODE_DISABLED 0
#define FILTER_MODE_MATCH_STANDARD 1
#define FILTER_MODE_MATCH_STANDARD_AND_DATA 2
#define FILTER_MODE_MATCH_EXTENDED 3

#define ERROR_READ_REGISTER_FULL (1 << 0)
#define ERROR_READ_BUFFER_FULL (1 << 1)

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) StandardMessage;

typedef struct {
	MessageHeader header;
	uint8_t frame_type;
	uint32_t identifier;
	uint8_t data[8];
	uint8_t length;
} __attribute__((__packed__)) WriteFrame;

typedef struct {
	MessageHeader header;
	bool success;
} __attribute__((__packed__)) WriteFrameReturn;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) ReadFrame;

typedef struct {
	MessageHeader header;
	bool success;
	uint8_t frame_type;
	uint32_t identifier;
	uint8_t data[8];
	uint8_t length;
} __attribute__((__packed__)) ReadFrameReturn;

typedef struct {
	MessageHeader header;
	uint8_t baud_rate;
	uint8_t transceiver_mode;
	int32_t write_timeout;
} __attribute__((__packed__)) SetConfiguration;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) GetConfiguration;

typedef struct {
	MessageHeader header;
	uint8_t baud_rate;
	uint8_t transceiver_mode;
	int32_t write_timeout;
} __attribute__((__packed__)) GetConfigurationReturn;

typedef struct {
	MessageHeader header;
	uint32_t error_mask;
} __attribute__((__packed__)) Error;

void constructor(void);
void destructor(void);
void invocation(const ComType com, const uint8_t *data);
void tick(const uint8_t tick_type);

uint8_t spibb_transceive_byte(const uint8_t value);

void mcp2515_instruction(const uint8_t inst, const uint8_t *req, const uint8_t req_length,
                         uint8_t *res, const uint8_t res_length);
void mcp2515_reset(void);
uint8_t mcp2515_read_register(const uint8_t reg);
void mcp2515_read_registers(const uint8_t reg, uint8_t *data, const uint8_t length);
void mcp2515_write_register(const uint8_t reg, const uint8_t data);
void mcp2515_write_registers(const uint8_t reg, const uint8_t *data, const uint8_t length);
void mcp2515_load_tx_buffer(const uint8_t index, const uint8_t *data, const uint8_t length);
uint8_t mcp2515_read_status(void);
uint8_t mcp2515_rx_status(void);
void mcp2515_bit_modify(const uint8_t reg, const uint8_t mask, const uint8_t data);

void write_frame(const ComType com, const WriteFrame *data);
void read_frame(const ComType com, const ReadFrame *data);

void set_configuration(const ComType com, const SetConfiguration *data);
void get_configuration(const ComType com, const GetConfiguration *data);

#endif
