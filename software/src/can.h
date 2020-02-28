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
#define FID_ENABLE_FRAME_READ_CALLBACK 3
#define FID_DISABLE_FRAME_READ_CALLBACK 4
#define FID_IS_FRAME_READ_CALLBACK_ENABLED 5
#define FID_SET_CONFIGURATION 6
#define FID_GET_CONFIGURATION 7
#define FID_SET_READ_FILTER 8
#define FID_GET_READ_FILTER 9
#define FID_GET_ERROR_LOG 10
#define FID_FRAME_READ 11
#define FID_SET_FRAME_READABLE_CALLBACK_CONFIGURATION 12
#define FID_GET_FRAME_READABLE_CALLBACK_CONFIGURATION 13
#define FID_FRAME_READABLE 14

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
#define FILTER_MODE_ACCEPT_ALL 1
#define FILTER_MODE_MATCH_STANDARD 2
#define FILTER_MODE_MATCH_STANDARD_AND_DATA 3
#define FILTER_MODE_MATCH_EXTENDED 4

typedef struct {
	uint8_t frame_type;
	uint32_t identifier;
	uint8_t data[8];
	uint8_t length;
} __attribute__((__packed__)) Frame;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) StandardMessage;

typedef struct {
	MessageHeader header;
	Frame frame;
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
	Frame frame;
} __attribute__((__packed__)) ReadFrameReturn;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) EnableFrameReadCallback;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) DisableFrameReadCallback;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) IsFrameReadCallbackEnabled;

typedef struct {
	MessageHeader header;
	bool enabled;
} __attribute__((__packed__)) IsFrameReadCallbackEnabledReturn;

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
	uint8_t mode;
	uint32_t mask;
	uint32_t filter1;
	uint32_t filter2;
} __attribute__((__packed__)) SetReadFilter;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) GetReadFilter;

typedef struct {
	MessageHeader header;
	uint8_t mode;
	uint32_t mask;
	uint32_t filter1;
	uint32_t filter2;
} __attribute__((__packed__)) GetReadFilterReturn;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) GetErrorLog;

typedef struct {
	MessageHeader header;
	uint8_t write_error_level;
	uint8_t read_error_level;
	bool transceiver_disabled;
	uint32_t write_timeout_count;
	uint32_t read_register_overflow_count;
	uint32_t read_buffer_overflow_count;
} __attribute__((__packed__)) GetErrorLogReturn;

typedef struct {
	MessageHeader header;
	Frame frame;
} __attribute__((__packed__)) FrameRead;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) FrameReadable;

typedef struct {
	MessageHeader header;
	bool enabled;
} __attribute__((__packed__)) SetFrameReadableCallbackConfiguration;

typedef struct {
	MessageHeader header;
} __attribute__((__packed__)) GetFrameReadableCallbackConfiguration;

typedef struct {
	MessageHeader header;
	bool enabled;
} __attribute__((__packed__)) GetFrameReadableCallbackConfigurationReturn;

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
void mcp2515_read_rxb0(uint8_t *rxb);
void mcp2515_clear_rxb0(void);
void mcp2515_write_register(const uint8_t reg, const uint8_t data);
void mcp2515_write_registers(const uint8_t reg, const uint8_t *data, const uint8_t length);
void mcp2515_write_txb0(const uint8_t *txb);
uint8_t mcp2515_read_status(void);
uint8_t mcp2515_read_rx_status(void);
void mcp2515_write_bits(const uint8_t reg, const uint8_t mask, const uint8_t data);
void mcp2515_rts_txb0(void);

void apply_config(void);

void compose_identifier(uint8_t *buffer, const uint32_t identifier,
                        const bool extended, const uint8_t extended_bit);

bool txb_enqueue(const Frame *frame);
bool rxb_dequeue(Frame *frame);

void write_frame(const ComType com, const WriteFrame *data);
void read_frame(const ComType com, const ReadFrame *data);

void enable_frame_read_callback(const ComType com, const EnableFrameReadCallback *data);
void disable_frame_read_callback(const ComType com, const DisableFrameReadCallback *data);
void is_frame_read_callback_enabled(const ComType com, const IsFrameReadCallbackEnabled *data);

void set_configuration(const ComType com, const SetConfiguration *data);
void get_configuration(const ComType com, const GetConfiguration *data);

void set_read_filter(const ComType com, const SetReadFilter *data);
void get_read_filter(const ComType com, const GetReadFilter *data);

void get_error_log(const ComType com, const GetErrorLog *data);

void set_frame_readable_callback_configuration(const ComType com, const SetFrameReadableCallbackConfiguration *data);
void get_frame_readable_callback_configuration(const ComType com, const GetFrameReadableCallbackConfiguration *data);

#endif
