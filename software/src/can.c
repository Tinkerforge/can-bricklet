/* can-bricklet
 * Copyright (C) 2016 Matthias Bolte <matthias@tinkerforge.com>
 *
 * can.c: Implementation of CAN Bricklet messages
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

#include "can.h"

#include "brickletlib/bricklet_entry.h"
#include "bricklib/bricklet/bricklet_communication.h"
#include "bricklib/utility/util_definitions.h"
#include "config.h"

// Definitions:
// F_OSC = 16MHz
// Baud Rate Prescaler (BRP) = [0..63]
// PRSEG = [0..7]
// PHSEG1 = [0..7]
// PHSEG2 = [0..7]
// Time Quanta (T_Q) = 2 * (BRP + 1) / F_OSC
// FixedSyncSeg = 1 * T_Q
// PropSeg = (PRSEG + 1) * T_Q
// PS1 = (PHSEG1 + 1) * T_Q
// PS2 = (PHSEG2 + 1) * T_Q
// Nominal Bit Time (NBT) = FixedSyncSeg + PropSeg + PS1 + PS2
//
// Rules:
// PropSeg + PS1 >= PS2
// PropSeg + PS1 >= T_DELAY
// PS2 > SJW
//
// Recommendation:
// Synchronization Jump Width = 1 * T_Q -> SJW = 1
// SamplingPoint = 60% to 70% of NBT
//
// Typical:
// T_DELAY = 1 * T_Q to 2 * T_Q

// FIXME: CANopen specification 301 recommends to put the SamplingPoint at 87.5%
const uint8_t baud_rate_cnf[][3] = { // CNF3, CNF2, CNF1
	// BAUD_RATE_10000 (NBT = 100µs): BRP = 49 -> T_Q = 6.25µs -> NBT = 16 * T_Q,
	// PRSEG = 1 -> PropSeg = 2 * T_Q, PHSEG1 = 6 -> PS1 = 7 * T_Q, PHSEG2 = 5 -> PS2 = 6 * T_Q, SamplingPoint = 62.5%
	{ 5 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (6 << REG_CNF2_PHSEG1_offset) | (1 << REG_CNF2_PRSEG_offset),
	  49 << REG_CNF1_BRP_offset },

	// BAUD_RATE_20000 (NBT = 50µs): BRP = 24 -> T_Q = 3.125µs -> NBT = 16 * T_Q,
	// PRSEG = 1 -> PropSeg = 2 * T_Q, PHSEG1 = 6 -> PS1 = 7 * T_Q, PHSEG2 = 5 -> PS2 = 6 * T_Q, SamplingPoint = 62.5%
	{ 5 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (6 << REG_CNF2_PHSEG1_offset) | (1 << REG_CNF2_PRSEG_offset),
	  24 << REG_CNF1_BRP_offset },

	// BAUD_RATE_50000 (NBT = 20µs): BRP = 9 -> T_Q = 1.25µs -> NBT = 16 * T_Q,
	// PRSEG = 1 -> PropSeg = 2 * T_Q, PHSEG1 = 6 -> PS1 = 7 * T_Q, PHSEG2 = 5 -> PS2 = 6 * T_Q, SamplingPoint = 62.5%
	{ 5 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (6 << REG_CNF2_PHSEG1_offset) | (1 << REG_CNF2_PRSEG_offset),
	  9 << REG_CNF1_BRP_offset },

	// BAUD_RATE_125000 (NBT = 8µs): BRP = 3 -> T_Q = 0.5µs -> NBT = 16 * T_Q,
	// PRSEG = 1 -> PropSeg = 2 * T_Q, PHSEG1 = 6 -> PS1 = 7 * T_Q, PHSEG2 = 5 -> PS2 = 6 * T_Q, SamplingPoint = 62.5%
	{ 5 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (6 << REG_CNF2_PHSEG1_offset) | (1 << REG_CNF2_PRSEG_offset),
	  3 << REG_CNF1_BRP_offset },

	// BAUD_RATE_250000 (NBT = 4µs): BRP = 1 -> T_Q = 0.25µs -> NBT = 16 * T_Q,
	// PRSEG = 1 -> PropSeg = 2 * T_Q, PHSEG1 = 6 -> PS1 = 7 * T_Q, PHSEG2 = 5 -> PS2 = 6 * T_Q, SamplingPoint = 62.5%
	{ 5 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (6 << REG_CNF2_PHSEG1_offset) | (1 << REG_CNF2_PRSEG_offset),
	  1 << REG_CNF1_BRP_offset },

	// BAUD_RATE_500000 (NBT = 2µs): BRP = 0 -> T_Q = 0.125µs -> NBT = 16 * T_Q,
	// PRSEG = 1 -> PropSeg = 2 * T_Q, PHSEG1 = 6 -> PS1 = 7 * T_Q, PHSEG2 = 5 -> PS2 = 6 * T_Q, SamplingPoint = 62.5%
	{ 5 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (6 << REG_CNF2_PHSEG1_offset) | (1 << REG_CNF2_PRSEG_offset),
	  0 },

	// BAUD_RATE_800000 (NBT = 1.25µs): BRP = 0 -> T_Q = 0.125µs -> NBT = 10 * T_Q,
	// PRSEG = 0 -> PropSeg = 1 * T_Q, PHSEG1 = 3 -> PS1 = 4 * T_Q, PHSEG2 = 3 -> PS2 = 4 * T_Q, SamplingPoint = 60.0%
	{ 3 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (3 << REG_CNF2_PHSEG1_offset),
	  0 },

	// BAUD_RATE_1000000 (NBT = 1µs): BRP = 0 -> T_Q = 0.125µs -> NBT = 8 * T_Q,
	// PRSEG = 0 -> PropSeg = 1 * T_Q, PHSEG1 = 2 -> PS1 = 3 * T_Q, PHSEG2 = 2 -> PS2 = 3 * T_Q, SamplingPoint = 62.5%
	{ 2 << REG_CNF3_PHSEG2_offset,
	  REG_CNF2_BTLMODE | (2 << REG_CNF3_PHSEG2_offset),
	  0 }
};

const uint8_t transceiver_mode_canctrl[] = {
	REG_CANCTRL_REQOP_NORMAL, // TRANSCEIVER_MODE_NORMAL
	REG_CANCTRL_REQOP_LOOPBACK, // TRANSCEIVER_MODE_LOOPBACK
	REG_CANCTRL_REQOP_READ_ONLY, // TRANSCEIVER_MODE_READ_ONLY
};

void constructor(void) {
	_Static_assert(sizeof(BrickContext) <= BRICKLET_CONTEXT_MAX_SIZE,
	               "BrickContext too big");

	BrickContext *bc = BC;

	bc->status = 0;

	bc->txb_start = 0;
	bc->txb_end = 0;

	bc->rxb_start = 0;
	bc->rxb_end = 0;

	bc->write_timeout_count = 0;
	bc->read_register_overflow_count = 0;
	bc->read_buffer_overflow_count = 0;

	bc->baud_rate = BAUD_RATE_125000;
	bc->transceiver_mode = TRANSCEIVER_MODE_NORMAL;
	bc->write_timeout = 0;
	bc->write_timeout_counter = 0;

	bc->filter_mode = FILTER_MODE_ACCEPT_ALL;
	bc->filter_mask = 0;
	bc->filter1 = 0;
	bc->filter2 = 0;

	// Pins
	SPI_CS.type = PIO_OUTPUT_1;
	SPI_CS.attribute = PIO_DEFAULT;
	BA->PIO_Configure(&SPI_CS, 1);

	SPI_CLK.type = PIO_OUTPUT_1;
	SPI_CLK.attribute = PIO_DEFAULT;
	BA->PIO_Configure(&SPI_CLK, 1);

	SPI_SDI.type = PIO_OUTPUT_1;
	SPI_SDI.attribute = PIO_DEFAULT;
	BA->PIO_Configure(&SPI_SDI, 1);

	SPI_SDO.type = PIO_INPUT;
	SPI_SDO.attribute = PIO_DEFAULT;
	BA->PIO_Configure(&SPI_SDO, 1);

	SLEEP_MS(2);
	mcp2515_reset(); // Enters config mode
	SLEEP_US(200);

	apply_config();
}

void destructor(void) {
}

void invocation(const ComType com, const uint8_t *data) {
	switch (((StandardMessage*)data)->header.fid) {
		case FID_WRITE_FRAME:                    write_frame(com, (WriteFrame *)data); break;
		case FID_READ_FRAME:                     read_frame(com, (ReadFrame *)data); break;
		case FID_ENABLE_FRAME_READ_CALLBACK:     enable_frame_read_callback(com, (EnableFrameReadCallback*)data); break;
		case FID_DISABLE_FRAME_READ_CALLBACK:    disable_frame_read_callback(com, (DisableFrameReadCallback*)data); break;
		case FID_IS_FRAME_READ_CALLBACK_ENABLED: is_frame_read_callback_enabled(com, (IsFrameReadCallbackEnabled*)data); break;
		case FID_SET_CONFIGURATION:              set_configuration(com, (SetConfiguration *)data); break;
		case FID_GET_CONFIGURATION:              get_configuration(com, (GetConfiguration *)data); break;
		case FID_SET_READ_FILTER:                set_read_filter(com, (SetReadFilter *)data); break;
		case FID_GET_READ_FILTER:                get_read_filter(com, (GetReadFilter *)data); break;
		case FID_GET_ERROR_LOG:                  get_error_log(com, (GetErrorLog *)data); break;
		default:                                 BA->com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_NOT_SUPPORTED, com); break;
	}
}

void tick(const uint8_t tick_type) {
	BrickContext *bc = BC;
	const BrickletAPI *ba = BA;

	if (tick_type & TICK_TASK_TYPE_CALCULATION) {
		if ((bc->status & STATUS_ENTERING_CONFIG_MODE) != 0) {
			mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_REQOP_mask, REG_CANCTRL_REQOP_CONFIG); // 4 bytes // FIXME: could do 3 bytes register write by caching CANCTRL

			const uint8_t canstat = mcp2515_read_register(REG_CANSTAT); // 3 bytes

			if ((canstat & REG_CANSTAT_OPMOD_mask) == REG_CANSTAT_OPMOD_CONFIG) {
				apply_config();

				bc->status &= ~STATUS_ENTERING_CONFIG_MODE;
				bc->status |= STATUS_LEAVING_CONFIG_MODE;
			}
		}

		if ((bc->status & STATUS_LEAVING_CONFIG_MODE) != 0) {
			mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_REQOP_mask,
			                   transceiver_mode_canctrl[bc->transceiver_mode]); // 4 bytes // FIXME: could do 3 bytes register write by caching CANCTRL

			const uint8_t canstat = mcp2515_read_register(REG_CANSTAT); // 3 bytes

			if ((canstat & REG_CANSTAT_OPMOD_mask) == transceiver_mode_canctrl[bc->transceiver_mode]) {
				bc->status &= ~STATUS_LEAVING_CONFIG_MODE;
			}
		}

		if ((bc->status & (STATUS_ENTERING_CONFIG_MODE | STATUS_LEAVING_CONFIG_MODE)) == 0) {
			const uint8_t status = mcp2515_read_status(); // 2 bytes

			// FIXME: maybe don't read and write a frame in the same tick to
			//        avoid making the tick too long

			// Read frame from RXB0. There is no point in using RXB1 in
			// overflow-mode to increase throughput. At best 2000 frames per
			// second could be received this way. But only 1000 frames per
			// second can be send to the user program. Also trying to read
			// the data in correct order from RXB0 and RXB1 is really tricky
			// and full of race conditions. I've tried to come up with an
			// algorithm for this, but have failed. For now I keep it simple
			// and just completely ignore RXB1. Also there is not enough time
			// per tick to read more than one RXB per tick over SPI at 400 kHz.

			// Reading RXB0 requires to transceive X bytes:
			//
			// X = 4 bytes (buffer overflow)
			// X = 7 bytes (register and buffer overflow)
			// X = 9 to 17 bytes (data length 0 to 8)
			// X = 12 to 20 bytes (register overflow and data length 0 to 8)
			//
			// At 400 kHz transceiving one byte takes 20 µs over SPI.
			if ((status & INST_READ_STATUS_CANINTF_RX0IF) != 0) {
				const uint8_t eflg = mcp2515_read_register(REG_EFLG); // 3 bytes

				if ((eflg & REG_EFLG_RX0OVR) != 0) {
					bc->read_register_overflow_count++;

					mcp2515_write_register(REG_EFLG, 0); // 3 bytes
				}

				if ((bc->rxb_end + 1) % BUFFER_COUNT != bc->rxb_start) {
					uint8_t *rxb = bc->rxb[bc->rxb_end];
					bc->rxb_end = (bc->rxb_end + 1) % BUFFER_COUNT;

					mcp2515_read_rxb0(rxb); // 6-14 bytes
				} else {
					bc->read_buffer_overflow_count++;

					mcp2515_clear_rxb0(); // 1 bytes
				}
			}

#if 0
			// FIXME: this block is just for debugging
			if ((status & INST_READ_STATUS_CANINTF_RX1IF) != 0) {
				ba->printf("R1 ignore\n\r");
				mcp2515_write_bits(REG_CANINTF, REG_CANINTF_RX1IF, 0); // 4 bytes
			}
#endif

			// Handling a normal write timeout (> 0) requires to transceive
			// 4 bytes in the tick in that the timeout was detected and another
			// 4 bytes in the next tick. Detecting a one-shot timeout (< 0)
			// requires to transceive 3 bytes.
			if (bc->write_timeout_counter > 0) {
				bc->write_timeout_counter--;

				if (bc->write_timeout_counter == 0 &&
				    (status & INST_READ_STATUS_TXB0CTRL_TXREQ) != 0) {
					mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_ABAT, REG_CANCTRL_ABAT); // 4 bytes // FIXME: could do 3 bytes register write by caching CANCTRL

					bc->status |= STATUS_WRITE_ABORTED;
					bc->write_timeout_count++;
				}
			}

			if ((status & INST_READ_STATUS_TXB0CTRL_TXREQ) == 0) {
				if ((bc->status & STATUS_WRITE_ABORTED) != 0) {
					mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_ABAT, 0); // 4 bytes // FIXME: could do 3 bytes register write by caching CANCTRL

					bc->status &= ~STATUS_WRITE_ABORTED;
				}

				if ((bc->status & STATUS_WRITE_PENDING) != 0) {
					if (bc->write_timeout < 0) {
						const uint8_t txb0ctrl = mcp2515_read_register(REG_TXB0CTRL); // 3 bytes

						if ((txb0ctrl & REG_TXBnCTRL_ABTF) != 0) {
							bc->write_timeout_count++;
						}
					}

					bc->status &= ~STATUS_WRITE_PENDING;
				}

				// Write frame to TXB0. Currently TXB1 and TXB2 are not used
				// because the required priority management to ensure that all
				// frames are transmitted eventually is complex. Ensuring that
				// frames are send in the requested order within the specified
				// timeout is even more complex. Also a standard data frame is
				// 44 + 8N + 3 bits long for N data bytes. This means that for
				// a full standard data frame 111 bits have to be transmitted.
				// At 125 kbit/s this takes 888 µs. Therefore, using more than
				// one TXB at 125 kbit/s or slower would not increase the
				// throughput significantly at all. Also the user program can
				// send at most 1000 frames per second. Therefore, using more
				// than one TXB would not result in a significant increase in
				// throughput even at higher baud rates than 125 kbit/s, as
				// there are only 1000 frames per second to be transmitted.

				// Writing TXB0 requires to transceive X bytes:
				//
				// X = 1 to 9 bytes (header cached and data length 0 to 8)
				// X = 7 to 15 bytes (data length 0 to 8)
				//
				// At 400 kHz transceiving one byte takes 20 µs over SPI.
				if (bc->txb_start != bc->txb_end) {
					const uint8_t *txb = bc->txb[bc->txb_start];
					bc->txb_start = (bc->txb_start + 1) % BUFFER_COUNT;

					mcp2515_write_txb0(txb); // 0-14 bytes
					mcp2515_rts_txb0(); // 1 bytes

					bc->status |= STATUS_WRITE_PENDING;
					bc->write_timeout_counter = bc->write_timeout;
				}
			}
		}
	}

	if (tick_type & TICK_TASK_TYPE_MESSAGE) {
		if ((bc->status & STATUS_FRAME_READ_CALLBACK_ENABLED) != 0 &&
		    bc->rxb_start != bc->rxb_end) {
			FrameRead fr;

			ba->com_make_default_header(&fr, BS->uid, sizeof(fr), FID_FRAME_READ);

			rxb_dequeue(&fr.frame);

			ba->send_blocking_with_timeout(&fr, sizeof(fr), *ba->com_current);
		}
	}
}

uint8_t spibb_transceive_byte(const uint8_t req) { // cpol=1,cpha=1
	uint8_t res = 0;

	for (int8_t i = 7; i >= 0; i--) {
		SPI_CLK.pio->PIO_CODR = SPI_CLK.mask;

		if ((req & (1 << i)) != 0) {
			SPI_SDI.pio->PIO_SODR = SPI_SDI.mask;
		} else {
			SPI_SDI.pio->PIO_CODR = SPI_SDI.mask;
		}

		SLEEP_US(1);

		if ((SPI_SDO.pio->PIO_PDSR & SPI_SDO.mask) != 0) {
			res |= (1 << i);
		}

		SPI_CLK.pio->PIO_SODR = SPI_CLK.mask;
		SLEEP_US(1);
	}

	return res;
}

void mcp2515_instruction(const uint8_t inst, const uint8_t *req, const uint8_t req_length,
                         uint8_t *res, const uint8_t res_length) {
	SLEEP_US(1);
	SPI_CS.pio->PIO_CODR = SPI_CS.mask;

	spibb_transceive_byte(inst);

	for (uint8_t i = 0; i < req_length; i++) {
		spibb_transceive_byte(req[i]);
	}

	for (uint8_t i = 0; i < res_length; i++) {
		res[i] = spibb_transceive_byte(0);
	}

	SPI_CS.pio->PIO_SODR = SPI_CS.mask;
}

void mcp2515_reset(void) {
	mcp2515_instruction(INST_RESET, NULL, 0, NULL, 0);
}

uint8_t mcp2515_read_register(const uint8_t reg) {
	uint8_t data;

	mcp2515_instruction(INST_READ, &reg, 1, &data, 1);

	return data;
}

void mcp2515_read_registers(const uint8_t reg, uint8_t *data, const uint8_t length) {
	mcp2515_instruction(INST_READ, &reg, 1, data, length);
}

void mcp2515_read_rxb0(uint8_t *rxb) {
	SLEEP_US(1);
	SPI_CS.pio->PIO_CODR = SPI_CS.mask;

	spibb_transceive_byte(INST_READ_RX_BUFFER_RXB0SIDH);

	for (uint8_t i = 0; i < 5; i++) {
		rxb[i] = spibb_transceive_byte(0);
	}

	const uint8_t dlc = (rxb[4] & REG_RXBnDLC_DLC_mask) >> REG_RXBnDLC_DLC_offset;

	for (uint8_t i = 5; i < 5 + dlc && i < BUFFER_LENGTH; i++) {
		rxb[i] = spibb_transceive_byte(0);
	}

	SPI_CS.pio->PIO_SODR = SPI_CS.mask; // clears the corresponding CANINTF.RXnIF bit
}

void mcp2515_clear_rxb0(void) {
	mcp2515_instruction(INST_READ_RX_BUFFER_RXB0SIDH, NULL, 0, NULL, 0);
}

void mcp2515_write_register(const uint8_t reg, const uint8_t data) {
	const uint8_t req[2] = { reg, data };

	mcp2515_instruction(INST_WRITE, req, 2, NULL, 0);
}

void mcp2515_write_registers(const uint8_t reg, const uint8_t *data, const uint8_t length) {
	SLEEP_US(1);
	SPI_CS.pio->PIO_CODR = SPI_CS.mask;

	spibb_transceive_byte(INST_WRITE);
	spibb_transceive_byte(reg);

	for (uint8_t i = 0; i < length; i++) {
		spibb_transceive_byte(data[i]);
	}

	SPI_CS.pio->PIO_SODR = SPI_CS.mask;
}

void mcp2515_write_txb0(const uint8_t *txb) {
	BrickContext *bc = BC;
	const uint8_t dlc = (txb[4] & REG_TXBnDLC_RTR) == 0
	                    ? MIN((txb[4] & REG_TXBnDLC_DLC_mask) >> REG_TXBnDLC_DLC_offset, 8)
	                    : 0;

	// FIXME: maybe also cache data segment?
	if ((bc->status & STATUS_VALID_TXB0_HEADER) != 0 &&
	    bc->txb0_header[0] == txb[0] &&
	    bc->txb0_header[1] == txb[1] &&
	    bc->txb0_header[2] == txb[2] &&
	    bc->txb0_header[3] == txb[3] &&
	    bc->txb0_header[4] == txb[4]) {
		if (dlc > 0) {
			mcp2515_instruction(INST_WRITE_TX_BUFFER_TXB0D0, txb + 5, dlc, NULL, 0);
		}
	} else {
		bc->status |= STATUS_VALID_TXB0_HEADER;
		bc->txb0_header[0] = txb[0];
		bc->txb0_header[1] = txb[1];
		bc->txb0_header[2] = txb[2];
		bc->txb0_header[3] = txb[3];
		bc->txb0_header[4] = txb[4];

		mcp2515_instruction(INST_WRITE_TX_BUFFER_TXB0SIDH, txb, 5 + dlc, NULL, 0);
	}
}

uint8_t mcp2515_read_status(void) {
	uint8_t data;

	mcp2515_instruction(INST_READ_STATUS, NULL, 0, &data, 1);

	return data;
}

uint8_t mcp2515_read_rx_status(void) {
	uint8_t data;

	mcp2515_instruction(INST_READ_RX_STATUS, NULL, 0, &data, 1);

	return data;
}

void mcp2515_write_bits(const uint8_t reg, const uint8_t mask, const uint8_t data) {
	const uint8_t req[3] = { reg, mask, data };

	mcp2515_instruction(INST_WRITE_BITS, req, 3, NULL, 0);
}

void mcp2515_rts_txb0(void) {
	mcp2515_instruction(INST_RTS_TXB0, NULL, 0, NULL, 0);
}

void apply_config(void) {
	const BrickContext *bc = BC;

	// Set baud rate
	mcp2515_write_registers(REG_CNF3, baud_rate_cnf[bc->baud_rate], 3); // 5 bytes

	// Configure filter
	uint8_t mode = REG_RXBnCTRL_RXM_BOTH;
	bool extended = false;
	uint8_t extended_bit = 0;

	uint8_t mask[4] = {
		0, 0, 0, 0
	};

	uint8_t filters[8] = {
		0, 0, 0, 0,
		0, REG_RXFnSIDL_EXIDE, 0, 0,
	};

	if (bc->filter_mode == FILTER_MODE_DISABLED) {
		mode = REG_RXBnCTRL_RXM_DISABLED;
	} else if (bc->filter_mode == FILTER_MODE_ACCEPT_ALL) {
		// There is one important aspect to the filter configuration that
		// is not explained well in the MCP2515 datasheet.
		//
		// After reset RXBnCTRL.RXM is set to 00 ("Receive all valid
		// messages using either standard or extended identifiers that meet
		// filter criteria.") and RXMnSIDH to RXMnEID0 is set to all zero.
		// So all masks are zero. This implied to me that all valid standard
		// and extended messages should be accepted.
		//
		// But this is not the case. In reality the RXFnSIDL.EXIDE bit
		// defines if a standard or extended message is accepted even if
		// the mask is all zero. But all the RXFnSIDL registers have
		// undefined values after reset. This means that it is undefined
		// whether or not a particular filter will accept standard or
		// extended message.
		//
		// To create a true accept-all mode the filter have to be configured
		// in a way that there is a filter with the RXFnSIDL.EXIDE bit set
		// and another filter with the RXFnSIDL.EXIDE cleared for each RXB.
		mode = REG_RXBnCTRL_RXM_BOTH;
	} else if (bc->filter_mode == FILTER_MODE_MATCH_STANDARD) {
		mode = REG_RXBnCTRL_RXM_STANDARD_ONLY;
	} else if (bc->filter_mode == FILTER_MODE_MATCH_STANDARD_AND_DATA) {
		mode = REG_RXBnCTRL_RXM_BOTH;
		extended = true;
	} else if (bc->filter_mode == FILTER_MODE_MATCH_EXTENDED) {
		mode = REG_RXBnCTRL_RXM_EXTENDED_ONLY;
		extended_bit = REG_RXFnSIDL_EXIDE;
	}

	if (bc->filter_mode != FILTER_MODE_ACCEPT_ALL) {
		compose_identifier(mask, bc->filter_mask, extended, 0);

		compose_identifier(filters, bc->filter1, extended, extended_bit);
		compose_identifier(filters + 4, bc->filter2, extended, extended_bit);
	}

	mcp2515_write_registers(REG_RXM0SIDH, mask, 4); // 6 bytes
	mcp2515_write_registers(REG_RXM1SIDH, mask, 4); // 6 bytes

	mcp2515_write_registers(REG_RXF0SIDH, filters, 4); // 10 bytes
	mcp2515_write_registers(REG_RXF1SIDH, filters + 4, 4); // 10 bytes
	mcp2515_write_registers(REG_RXF2SIDH, filters, 4); // 10 bytes
	mcp2515_write_registers(REG_RXF3SIDH, filters + 4, 4); // 10 bytes
	mcp2515_write_registers(REG_RXF4SIDH, filters, 4); // 10 bytes
	mcp2515_write_registers(REG_RXF5SIDH, filters + 4, 4); // 10 bytes

	mcp2515_write_bits(REG_RXB0CTRL, REG_RXBnCTRL_RXM_mask, mode); // 4 bytes
	mcp2515_write_bits(REG_RXB1CTRL, REG_RXBnCTRL_RXM_mask, mode); // 4 bytes

	// Configure one-shot mode and switch to requested transceiver mode
	mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_REQOP_mask | REG_CANCTRL_OSM,
	                   transceiver_mode_canctrl[bc->transceiver_mode] |
	                   (bc->write_timeout < 0 ? REG_CANCTRL_OSM : 0)); // 4 bytes
}

void compose_identifier(uint8_t *buffer, const uint32_t identifier,
                        const bool extended, const uint8_t extended_bit) {
	// SIDH
	buffer[0] = (identifier >> 3) & 0xFF;

	// SIDL
	buffer[1] = ((identifier & 0b00000111) << 5) | extended_bit |
	            (extended ? (identifier >> 27) & 0b00000011 : 0);

	// EID8
	buffer[2] = extended ? (identifier >> 19) & 0xFF : 0;

	// EID0
	buffer[3] = extended ? (identifier >> 11) & 0xFF : 0;
}

bool txb_enqueue(const Frame *frame) {
	BrickContext *bc = BC;

	if ((bc->txb_end + 1) % BUFFER_COUNT == bc->txb_start) {
		return false;
	}

	uint8_t *txb = bc->txb[bc->txb_end];
	bc->txb_end = (bc->txb_end + 1) % BUFFER_COUNT;

	// TXBnSIDH to TXBnEID0
	const bool extended = frame->frame_type == FRAME_TYPE_EXTENDED_DATA ||
	                      frame->frame_type == FRAME_TYPE_EXTENDED_REMOTE;

	compose_identifier(txb, frame->identifier, extended, extended ? REG_TXBnSIDL_EXIDE : 0);

	// TXBnDLC
	const bool remote = frame->frame_type == FRAME_TYPE_STANDARD_REMOTE ||
	                    frame->frame_type == FRAME_TYPE_EXTENDED_REMOTE;

	txb[4] = (remote ? REG_TXBnDLC_RTR : 0) |
	         ((frame->length << REG_TXBnDLC_DLC_offset) & REG_TXBnDLC_DLC_mask);

	// TXBnDm
	for (uint8_t i = 0; i < frame->length && i < 8; ++i) {
		txb[5 + i] = frame->data[i];
	}

	return true;
}

bool rxb_dequeue(Frame *frame) {
	BrickContext *bc = BC;

	if (bc->rxb_start == bc->rxb_end) {
		return false;
	}

	const uint8_t *rxb = bc->rxb[bc->rxb_start];
	bc->rxb_start = (bc->rxb_start + 1) % BUFFER_COUNT;

	// frame type
	if ((rxb[1] & REG_RXBnSIDL_IDE) == 0) {
		if ((rxb[1] & REG_RXBnSIDL_SRR) == 0) {
			frame->frame_type = FRAME_TYPE_STANDARD_DATA;
		} else {
			frame->frame_type = FRAME_TYPE_STANDARD_REMOTE;
		}
	} else {
		if ((rxb[4] & REG_RXBnDLC_RTR) == 0) {
			frame->frame_type = FRAME_TYPE_EXTENDED_DATA;
		} else {
			frame->frame_type = FRAME_TYPE_EXTENDED_REMOTE;
		}
	}

	// identifier
	frame->identifier = ((uint32_t)rxb[0] << 3) | (rxb[1] >> 5);

	if (frame->frame_type == FRAME_TYPE_EXTENDED_DATA ||
	    frame->frame_type == FRAME_TYPE_EXTENDED_REMOTE) {
		frame->identifier |= (((uint32_t)rxb[1] & 0b00000011) << 27) |
		                     ((uint32_t)rxb[2] << 19) |
		                     ((uint32_t)rxb[3] << 11);
	}

	// length
	frame->length = (rxb[4] & REG_RXBnDLC_DLC_mask) >> REG_RXBnDLC_DLC_offset;

	// data
	if (frame->frame_type == FRAME_TYPE_STANDARD_DATA ||
	    frame->frame_type == FRAME_TYPE_EXTENDED_DATA) {
		for (uint8_t i = 0; i < frame->length && i < 8; ++i) {
			frame->data[i] = rxb[5 + i];
		}

		for (uint8_t i = frame->length; i < 8; ++i) {
			frame->data[i] = 0;
		}
	} else {
		for (uint8_t i = 0; i < 8; ++i) {
			frame->data[i] = 0;
		}
	}

	return true;
}

void write_frame(const ComType com, const WriteFrame *data) {
	if (data->frame.frame_type > FRAME_TYPE_EXTENDED_REMOTE ||
	    data->frame.length > 15) {
		BA->com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	WriteFrameReturn wfr;

	wfr.header        = data->header;
	wfr.header.length = sizeof(wfr);
	wfr.success       = txb_enqueue(&data->frame);

	BA->send_blocking_with_timeout(&wfr, sizeof(wfr), com);
}

void read_frame(const ComType com, const ReadFrame *data) {
	// Need to zero whole message here because rxb_dequeue will
	// not touch it if there is no frame to be read
	ReadFrameReturn rfr = {{0}};

	rfr.header        = data->header;
	rfr.header.length = sizeof(rfr);
	rfr.success       = rxb_dequeue(&rfr.frame);

	BA->send_blocking_with_timeout(&rfr, sizeof(rfr), com);
}

void enable_frame_read_callback(const ComType com, const EnableFrameReadCallback *data) {
	BC->status |= STATUS_FRAME_READ_CALLBACK_ENABLED;
	BA->com_return_setter(com, data);
}

void disable_frame_read_callback(const ComType com, const DisableFrameReadCallback *data) {
	BC->status &= ~STATUS_FRAME_READ_CALLBACK_ENABLED;
	BA->com_return_setter(com, data);
}

void is_frame_read_callback_enabled(const ComType com, const IsFrameReadCallbackEnabled *data) {
	IsFrameReadCallbackEnabledReturn ifrcer;

	ifrcer.header         = data->header;
	ifrcer.header.length  = sizeof(ifrcer);
	ifrcer.enabled        = (BC->status & STATUS_FRAME_READ_CALLBACK_ENABLED) != 0;

	BA->send_blocking_with_timeout(&ifrcer, sizeof(ifrcer), com);
}

void set_configuration(const ComType com, const SetConfiguration *data) {
	BrickContext *bc = BC;

	if (data->baud_rate > BAUD_RATE_1000000 ||
	    data->transceiver_mode > TRANSCEIVER_MODE_READ_ONLY) {
		BA->com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	bc->baud_rate        = data->baud_rate;
	bc->transceiver_mode = data->transceiver_mode;
	bc->write_timeout    = MAX(data->write_timeout, -1);

	bc->status |= STATUS_ENTERING_CONFIG_MODE;
	bc->status &= ~STATUS_LEAVING_CONFIG_MODE;

	BA->com_return_setter(com, data);
}

void get_configuration(const ComType com, const GetConfiguration *data) {
	const BrickContext *bc = BC;
	GetConfigurationReturn gcr;

	gcr.header           = data->header;
	gcr.header.length    = sizeof(gcr);
	gcr.baud_rate        = bc->baud_rate;
	gcr.transceiver_mode = bc->transceiver_mode;
	gcr.write_timeout    = bc->write_timeout;

	BA->send_blocking_with_timeout(&gcr, sizeof(gcr), com);
}

void set_read_filter(const ComType com, const SetReadFilter *data) {
	BrickContext *bc = BC;

	if (data->mode > FILTER_MODE_MATCH_EXTENDED) {
		BA->com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	bc->filter_mode = data->mode;

	if (data->mode > FILTER_MODE_ACCEPT_ALL) {
		bc->filter_mask = data->mask;
		bc->filter1     = data->filter1;
		bc->filter2     = data->filter2;
	} else {
		bc->filter_mask = 0;
		bc->filter1     = 0;
		bc->filter2     = 0;
	}

	bc->status |= STATUS_ENTERING_CONFIG_MODE;
	bc->status &= ~STATUS_LEAVING_CONFIG_MODE;

	BA->com_return_setter(com, data);
}

void get_read_filter(const ComType com, const GetReadFilter *data) {
	const BrickContext *bc = BC;
	GetReadFilterReturn grfr;

	grfr.header        = data->header;
	grfr.header.length = sizeof(grfr);
	grfr.mode          = bc->filter_mode;
	grfr.mask          = bc->filter_mask;
	grfr.filter1       = bc->filter1;
	grfr.filter2       = bc->filter2;

	BA->send_blocking_with_timeout(&grfr, sizeof(grfr), com);
}

void get_error_log(const ComType com, const GetErrorLog *data) {
	const BrickContext *bc = BC;
	GetErrorLogReturn gelr;

	gelr.header                       = data->header;
	gelr.header.length                = sizeof(gelr);

	uint8_t tec_rec[2];
	mcp2515_read_registers(REG_TEC, tec_rec, 2); // 4 bytes

	const uint8_t eflg = mcp2515_read_register(REG_EFLG); // 3 bytes

	gelr.write_error_level            = tec_rec[0];
	gelr.read_error_level             = tec_rec[1];
	gelr.transceiver_disabled         = (eflg & REG_EFLG_TXBO) != 0;
	gelr.write_timeout_count          = bc->write_timeout_count;
	gelr.read_register_overflow_count = bc->read_register_overflow_count;
	gelr.read_buffer_overflow_count   = bc->read_buffer_overflow_count;

	BA->send_blocking_with_timeout(&gelr, sizeof(gelr), com);
}
