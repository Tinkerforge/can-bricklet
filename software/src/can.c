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

	BC->txb_start = 0;
	BC->txb_end = 0;

	BC->rxb_start = 0;
	BC->rxb_end = 0;

	BC->read_callback_enabled = false;

	BC->error_mask = 0;

	BC->entering_config_mode = false;
	BC->leaving_config_mode = false;

	BC->baud_rate = BAUD_RATE_125000;
	BC->transceiver_mode = TRANSCEIVER_MODE_NORMAL;
	BC->write_timeout = 0;

	BC->filter_mode = FILTER_MODE_DISABLED;
	BC->filter_mask = 0;
	BC->filter1 = 0;
	BC->filter2 = 0;

	BC->tick = 0;

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

	// Reset
	SLEEP_MS(2);
	mcp2515_reset(); // Enters config mode
	SLEEP_US(100);

	// Set baud rate
	mcp2515_write_registers(REG_CNF3, baud_rate_cnf[BC->baud_rate], 3);

	// Enable RXB0 to RXB1 rollover
	//mcp2515_bit_modify(REG_RXB0CTRL, REG_RXB0CTRL_BUKT, REG_RXB0CTRL_BUKT);

	// Configure filters
	//mcp2515_bit_modify(REG_RXB1CTRL, REG_RXBnCTRL_RXM_mask, REG_RXBnCTRL_RXM_DISABLED);
	//mcp2515_bit_modify(REG_RXB0CTRL, REG_RXBnCTRL_RXM_mask, REG_RXBnCTRL_RXM_DISABLED);

	// Configure one-shot mode and switch to requested mode
	mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_REQOP_mask | REG_CANCTRL_OSM,
	                   transceiver_mode_canctrl[BC->transceiver_mode] |
	                   (BC->write_timeout < 0 ? REG_CANCTRL_OSM : 0));
}

void destructor(void) {
}

void invocation(const ComType com, const uint8_t *data) {
	switch (((StandardMessage*)data)->header.fid) {
		case FID_WRITE_FRAME:       write_frame(com, (WriteFrame *)data); break;
		case FID_READ_FRAME:        read_frame(com, (ReadFrame *)data); break;
		case FID_SET_CONFIGURATION: set_configuration(com, (SetConfiguration *)data); break;
		case FID_GET_CONFIGURATION: get_configuration(com, (GetConfiguration *)data); break;
		default: BA->com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_NOT_SUPPORTED, com); break;
	}
}

void tick(const uint8_t tick_type) {
	if (tick_type & TICK_TASK_TYPE_CALCULATION) {
		BC->tick++;

		if (BC->entering_config_mode) {
			BA->printf("%u: EC\n\r", BC->tick);

			mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_REQOP_mask, REG_CANCTRL_REQOP_CONFIG);

			const uint8_t canstat = mcp2515_read_register(REG_CANSTAT);

			if ((canstat & REG_CANSTAT_OPMOD_mask) == REG_CANSTAT_OPMOD_CONFIG) {
				mcp2515_write_registers(REG_CNF3, baud_rate_cnf[BC->baud_rate], 3);
				mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_REQOP_mask | REG_CANCTRL_OSM,
				                   transceiver_mode_canctrl[BC->transceiver_mode] |
				                   (BC->write_timeout < 0 ? REG_CANCTRL_OSM : 0));

				BC->entering_config_mode = false;
				BC->leaving_config_mode = true;
			}
		}

		if (BC->leaving_config_mode) {
			BA->printf("%u: LC\n\r", BC->tick);

			mcp2515_write_bits(REG_CANCTRL, REG_CANCTRL_REQOP_mask,
			                   transceiver_mode_canctrl[BC->transceiver_mode]);

			const uint8_t canstat = mcp2515_read_register(REG_CANSTAT);

			if ((canstat & REG_CANSTAT_OPMOD_mask) == transceiver_mode_canctrl[BC->transceiver_mode]) {
				BC->leaving_config_mode = false;
			}
		}

		if (!BC->entering_config_mode && !BC->leaving_config_mode) {
			const uint8_t status = mcp2515_read_status();

			// Write frame
			if (BC->txb_start != BC->txb_end) {
				BA->printf("%u: WQ\n\r", BC->tick);

				// FIXME: currently only using the first TX buffer for simplicity
				// FIXME: add some sort of timeout handling?
				// FIXME: read and report/deal with TXBnCTRL.MLOA or TXBnCTRL.TXERR?
				if ((status & INST_READ_STATUS_TXB0CTRL_TXREQ) == 0) {
					const uint8_t *txb = BC->txb[BC->txb_start];
					BC->txb_start = (BC->txb_start + 1) % BUFFER_COUNT;

					mcp2515_write_tx_buffer(0, txb, BUFFER_LENGTH);
					mcp2515_write_bits(REG_TXB0CTRL, REG_TXBnCTRL_TXREQ, REG_TXBnCTRL_TXREQ);

					BA->printf("%u: W0\n\r", BC->tick);
				}
			}

			// Check if read-registers are full
			if ((status & (INST_READ_STATUS_CANINTF_RX1IF | INST_READ_STATUS_CANINTF_RX0IF)) != 0) {
				const uint8_t eflg = mcp2515_read_register(REG_EFLG);
				uint8_t eflg_mask = 0;

				if ((eflg & REG_EFLG_RX1OVR) != 0) {
					// FIXME: report read register overflow
					BA->printf("%u: R1 rf\n\r", BC->tick);

					eflg_mask |= REG_EFLG_RX1OVR;
					BC->error_mask |= ERROR_READ_REGISTER_FULL;
				}

				if ((eflg & REG_EFLG_RX0OVR) != 0) {
					// FIXME: report read register overflow
					BA->printf("%u: R0 rf\n\r", BC->tick);

					eflg_mask |= REG_EFLG_RX0OVR;
					BC->error_mask |= ERROR_READ_REGISTER_FULL;
				}

				mcp2515_write_bits(REG_EFLG, eflg_mask, 0);
			}

			// Read frame
			// FIXME: cannot use the read-rx-buffer instruction here, because it
			//        would clear RXnIF bit and would allow RXB0 to overflow into
			//        RXB1 between reading RXB1 and RXB0
			uint8_t canintf_mask = 0;

			if ((status & INST_READ_STATUS_CANINTF_RX1IF) != 0) {
				BA->printf("%u: R1\n\r", BC->tick);

				uint8_t *rxb;
				uint8_t dropped[BUFFER_LENGTH];

				// If the read-buffer is full drop the new frame
				if ((BC->rxb_end + 1) % BUFFER_COUNT != BC->rxb_start) {
					rxb = BC->rxb[BC->rxb_end];
					BC->rxb_end = (BC->rxb_end + 1) % BUFFER_COUNT;
				} else {
					BA->printf("%u: R1 bf\n\r", BC->tick);
					rxb = dropped;
					BC->error_mask |= ERROR_READ_BUFFER_FULL;
				}

				mcp2515_read_registers(REG_RXB1SIDH, rxb, BUFFER_LENGTH);

				canintf_mask |= REG_CANINTF_RX1IF;
			}

			if ((status & INST_READ_STATUS_CANINTF_RX0IF) != 0) {
				BA->printf("%u: R0\n\r", BC->tick);

				uint8_t *rxb;
				uint8_t dropped[BUFFER_LENGTH];

				// If the read-buffer is full drop the new frame
				if ((BC->rxb_end + 1) % BUFFER_COUNT != BC->rxb_start) {
					rxb = BC->rxb[BC->rxb_end];
					BC->rxb_end = (BC->rxb_end + 1) % BUFFER_COUNT;
				} else {
					BA->printf("%u: R0 bf\n\r", BC->tick);
					rxb = dropped;
					BC->error_mask |= ERROR_READ_BUFFER_FULL;
				}

				mcp2515_read_registers(REG_RXB0SIDH, rxb, BUFFER_LENGTH);

				canintf_mask |= REG_CANINTF_RX0IF;
			}

			mcp2515_write_bits(REG_CANINTF, canintf_mask, 0);
		}
	}

	if (tick_type & TICK_TASK_TYPE_MESSAGE) {
		if (BC->error_mask != 0) {
			BA->printf("%u: E %x\n\r", BC->tick, BC->error_mask);

			Error e;

			BA->com_make_default_header(&e, BS->uid, sizeof(e), FID_ERROR);

			e.error_mask = BC->error_mask;

			BA->send_blocking_with_timeout(&e, sizeof(e), *BA->com_current);

			BC->error_mask = 0;
		}
	}
}

uint8_t spibb_transceive_byte(const uint8_t value) { // cpol=1,cpha=1
	uint8_t recv = 0;

	for(int8_t i = 7; i >= 0; i--) {
		SPI_CLK.pio->PIO_CODR = SPI_CLK.mask;

		if((value >> i) & 1) {
			SPI_SDI.pio->PIO_SODR = SPI_SDI.mask;
		} else {
			SPI_SDI.pio->PIO_CODR = SPI_SDI.mask;
		}

		SLEEP_US(1);

		if(SPI_SDO.pio->PIO_PDSR & SPI_SDO.mask) {
			recv |= (1 << i);
		}

		SPI_CLK.pio->PIO_SODR = SPI_CLK.mask;
		SLEEP_US(1);
	}

	return recv;
}

void mcp2515_instruction(const uint8_t inst, const uint8_t *req, const uint8_t req_length,
                         uint8_t *res, const uint8_t res_length) {
	SLEEP_US(1);
	SPI_CS.pio->PIO_CODR = SPI_CS.mask;

	spibb_transceive_byte(inst);

	for(uint8_t i = 0; i < req_length; i++) {
		spibb_transceive_byte(req[i]);
	}

	for(uint8_t i = 0; i < res_length; i++) {
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

void mcp2515_read_rx_buffer(const uint8_t index, const uint8_t *data, const uint8_t length) {
	// also clears the corresponding CANINTF.RXnIF bit
	mcp2515_instruction(INST_READ_RX_BUFFER_base | (index * 2), data, length, NULL, 0);
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

	for(uint8_t i = 0; i < length; i++) {
		spibb_transceive_byte(data[i]);
	}

	SPI_CS.pio->PIO_SODR = SPI_CS.mask;
}

void mcp2515_write_tx_buffer(const uint8_t index, const uint8_t *data, const uint8_t length) {
	mcp2515_instruction(INST_WRITE_TX_BUFFER_base | (index * 2), data, length, NULL, 0);
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

void write_frame(const ComType com, const WriteFrame *data) {
	if (data->frame_type > FRAME_TYPE_EXTENDED_REMOTE ||
	    ((data->frame_type == FRAME_TYPE_STANDARD_DATA || data->frame_type == FRAME_TYPE_STANDARD_REMOTE) && data->identifier > 0x7FF) ||
	    data->identifier > 0x1FFFFFFF ||
	    data->length > 0b00001111) {
		BA->com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	WriteFrameReturn wfr;

	wfr.header        = data->header;
	wfr.header.length = sizeof(wfr);
	wfr.success       = (BC->txb_end + 1) % BUFFER_COUNT != BC->txb_start;

	if (wfr.success) {
		uint8_t *txb = BC->txb[BC->txb_end];
		BC->txb_end = (BC->txb_end + 1) % BUFFER_COUNT;

		// TXBnSIDH
		txb[0] = (data->identifier >> 3) & 0xFF;

		// TXBnSIDL
		txb[1] = ((data->identifier & 0b00000111) << 5) |
		         (data->frame_type == FRAME_TYPE_EXTENDED_DATA ||
		          data->frame_type == FRAME_TYPE_EXTENDED_REMOTE ? REG_TXBnSIDL_EXIDE : 0) |
		         ((data->identifier >> 27) & 0b00000011);

		// TXBnEID8
		txb[2] = (data->identifier >> 19) & 0xFF;

		// TXBnEID0
		txb[3] = (data->identifier >> 11) & 0xFF;

		// TXBnDLC
		txb[4] = (data->frame_type == FRAME_TYPE_STANDARD_REMOTE ||
		          data->frame_type == FRAME_TYPE_EXTENDED_REMOTE ? REG_TXBnDLC_RTR : 0) |
		          data->length;

		// TXBnDm
		for (uint8_t i = 0; i < 8; ++i) {
			txb[5 + i] = data->data[i];
		}
	}

	BA->send_blocking_with_timeout(&wfr, sizeof(wfr), com);
}

void read_frame(const ComType com, const ReadFrame *data) {
	ReadFrameReturn rfr = {{0}};

	rfr.header        = data->header;
	rfr.header.length = sizeof(rfr);
	rfr.success       = BC->rxb_start != BC->rxb_end;

	if (rfr.success) {
		const uint8_t *rxb = BC->rxb[BC->rxb_start];
		BC->rxb_start = (BC->rxb_start + 1) % BUFFER_COUNT;

		// frame type
		if ((rxb[1] & REG_RXBnSIDL_IDE) == 0) {
			if ((rxb[1] & REG_RXBnSIDL_SRR) == 0) {
				rfr.frame_type = FRAME_TYPE_STANDARD_DATA;
			} else {
				rfr.frame_type = FRAME_TYPE_STANDARD_REMOTE;
			}
		} else {
			if ((rxb[4] & REG_RXBnDLC_RTR) == 0) {
				rfr.frame_type = FRAME_TYPE_EXTENDED_DATA;
			} else {
				rfr.frame_type = FRAME_TYPE_EXTENDED_REMOTE;
			}
		}

		// identifier
		rfr.identifier = ((uint32_t)rxb[0] << 3) | (rxb[1] >> 5);

		if (rfr.frame_type == FRAME_TYPE_EXTENDED_DATA || rfr.frame_type == FRAME_TYPE_EXTENDED_REMOTE) {
			rfr.identifier |= (((uint32_t)rxb[1] & 0b00000011) << 16) | ((uint32_t)rxb[2] << 8) | rxb[3];
		}

		// length
		rfr.length = rxb[4] & 0b00001111;

		// data
		for (uint8_t i = 0; i < rfr.length && i < 8; ++i) {
			rfr.data[i] = rxb[5 + i];
		}

		for (uint8_t i = rfr.length; i < 8; ++i) {
			rfr.data[i] = 0;
		}
	}

	BA->send_blocking_with_timeout(&rfr, sizeof(rfr), com);
}

void set_configuration(const ComType com, const SetConfiguration *data) {
	if (data->baud_rate > BAUD_RATE_1000000 ||
	    data->transceiver_mode > TRANSCEIVER_MODE_READ_ONLY) {
		BA->com_return_error(data, sizeof(MessageHeader), MESSAGE_ERROR_CODE_INVALID_PARAMETER, com);
		return;
	}

	BC->baud_rate        = data->baud_rate;
	BC->transceiver_mode = data->transceiver_mode;
	BC->write_timeout    = data->write_timeout;

	BC->entering_config_mode = true;
	BC->leaving_config_mode = false;

	BA->com_return_setter(com, data);
}

void get_configuration(const ComType com, const GetConfiguration *data) {
	GetConfigurationReturn gcr;

	gcr.header           = data->header;
	gcr.header.length    = sizeof(gcr);
	gcr.baud_rate        = BC->baud_rate;
	gcr.transceiver_mode = BC->transceiver_mode;
	gcr.write_timeout    = BC->write_timeout;

	BA->send_blocking_with_timeout(&gcr, sizeof(gcr), com);
}
