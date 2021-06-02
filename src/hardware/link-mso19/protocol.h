/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2011 Daniel Ribeiro <drwyrm@gmail.com>
 * Copyright (C) 2012 Renato Caldas <rmsc@fe.up.pt>
 * Copyright (C) 2013 Lior Elazary <lelazary@yahoo.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_LINK_MSO19_PROTOCOL_H
#define LIBSIGROK_HARDWARE_LINK_MSO19_PROTOCOL_H

#include <stdint.h>
#include <string.h>
#include <glib.h>
#include <libudev.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "link-mso19"

#define USB_VENDOR		"3195"
#define USB_PRODUCT		"f190"

#define NUM_TRIGGER_STAGES	4
#define TRIGGER_TYPE 		"01"	//the first r/f is used for the whole group
#define SERIALCOMM		"460800/8n1/flow=2"
#define SERIALCONN		"/dev/ttyUSB0"
#define CLOCK_RATE		SR_MHZ(100)
#define MIN_NUM_SAMPLES		4

#define MSO_TRIGGER_UNKNOWN	'!'
#define MSO_TRIGGER_UNKNOWN1	'1'
#define MSO_TRIGGER_UNKNOWN2	'2'
#define MSO_TRIGGER_UNKNOWN3	'3'
#define MSO_TRIGGER_WAIT	'4'
#define MSO_TRIGGER_FIRED	'5'
#define MSO_TRIGGER_DATAREADY	'6'

enum trigger_slopes {
	SLOPE_POSITIVE = 0,
	SLOPE_NEGATIVE,
};

/* Structure for the pattern generator state */
struct mso_patgen {
	/* Pattern generator clock config */
	uint16_t clock;
	/* Buffer start address */
	uint16_t start;
	/* Buffer end address */
	uint16_t end;
	/* Pattern generator config */
	uint8_t config;
	/* Samples buffer */
	uint8_t buffer[1024];
	/* Input/output configuration for the samples buffer (?) */
	uint8_t io[1024];
	/* Number of loops for the pattern generator */
	uint8_t loops;
	/* Bit enable mask for the I/O lines */
	uint8_t mask;
};

/* Data structure for the protocol trigger state */
struct mso_prototrig {
	/* Word match buffer */
	uint8_t word[4];
	/* Masks for the wordmatch buffer */
	uint8_t mask[4];
	/* SPI mode 0, 1, 2, 3. Set to 0 for I2C */
	uint8_t spimode;
};

struct dev_context {
	/* info */
	uint8_t hwmodel;
	uint8_t hwrev;
//      uint8_t num_sample_rates;
	/* calibration */
	double vbit;
	uint16_t dac_offset;
	double offset_vbit;

	uint64_t limit_samples;
	uint64_t num_samples;

	uint8_t ctlbase2; // remove this, it is just slowmode?

	uint8_t la_threshold;
	uint64_t cur_rate;
	uint16_t cur_rate_regval;
	uint8_t slowmode;
	float dso_probe_attn;
	int8_t use_trigger;
	uint8_t trigger_chan;
	uint8_t trigger_slope;
	uint8_t trigger_outsrc;
	uint8_t trigger_state;
	uint8_t trigger_holdoff[2];
	uint8_t la_trigger;
	uint8_t la_trigger_mask;
	double dso_trigger_voltage;
	uint16_t dso_trigger_width;
	struct mso_prototrig protocol_trigger;
	gboolean dc_coupling; // TODO SR_CONF_COUPLING
	float v_offset; // TODO
	uint16_t buffer_n;
	char buffer[4096];
};

SR_PRIV int mso_parse_serial(const char *iSerial, const char *iProduct,
			     struct dev_context *ctx);
SR_PRIV int mso_check_trigger(const struct sr_dev_inst *sdi, uint8_t * info);
SR_PRIV int mso_reset_adc(struct sr_dev_inst *sdi);
SR_PRIV int mso_set_rate(const struct sr_dev_inst *sdi, uint32_t rate);
SR_PRIV int mso_receive_data(int fd, int revents, void *cb_data);
SR_PRIV int mso_configure_hw(const struct sr_dev_inst *sdi);
SR_PRIV int mso_read_buffer(struct sr_dev_inst *sdi);
SR_PRIV int mso_arm(const struct sr_dev_inst *sdi);
SR_PRIV int mso_force_capture(struct sr_dev_inst *sdi);
SR_PRIV uint16_t mso_calc_raw_from_mv(struct dev_context *devc, float mv);
SR_PRIV int mso_reset_fsm(struct sr_dev_inst *sdi);
SR_PRIV int mso_toggle_led(struct sr_dev_inst *sdi, int state);

SR_PRIV int mso_acquisition_stop(struct sr_dev_inst *sdi);
SR_PRIV uint64_t *mso_get_sample_rates(size_t *ret_len);


#endif
