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

#include <config.h>
#include "protocol.h"

/* bank agnostic registers */
#define REG_CTL2		15

/* bank 0 registers */
// read:
#define REG_BUFFER		0x1
#define REG_TRIGGER		0x2
// write:
#define REG_CLKRATE1	0x9
#define REG_CLKRATE2	0xa
#define REG_DAC1		0xc
#define REG_DAC2		0xd
/* possibly bank agnostic: */
#define REG_CTL1		0xe

/* bank 2 registers (SPI/I2C protocol trigger) */
#define REG_PT_WORD(x)		(x)
#define REG_PT_MASK(x)		(x + 4)
#define REG_PT_SPIMODE		8

/* bits - REG_CTL1 */
#define BIT_CTL1_RESETFSM	(1 << 0) // FSMReset 1= resets DSO FSM (pulse on write to reg)
#define BIT_CTL1_ARM		(1 << 1) // Armed DSO 1= Arms DSO for capture (pulse on write to reg)
#define BIT_CTL1_READMODE	(1 << 2) // ReadMode 1= Buffer Read Back, 0= DSO
#define BIT_CTL1_FORCE		(1 << 3) // TrigEnd 1= Forces a fake trig (pulse on write to reg)
#define BIT_CTL1_BASE		(1 << 4) // PwrDn 1= power down default = 0
#define BIT_CTL1_CONFIGDAC	(1 << 5) // configure voltage offset and trigger voltage ("unused" in mso28 register list?)
#define BIT_CTL1_RESETADC	(1 << 6) // ADCRst 1= Reset ADC default = 0 needs software pulse
#define BIT_CTL1_LED		(1 << 7) // mso28 registers says "unused"




/* bits - REG_CTL2 */
#define BITS_CTL2_BANK(x)	(x & 0x3)
#define BIT_CTL2_SLOWMODE	(1 << 5)

struct rate_map {
	uint32_t rate;
	uint16_t val;
	uint8_t slowmode;
};

/* rate_map table matches this:
# python
for rate, val, slowmode in rate_map:
    clkdiv = (val & 0xff) | ((val >> 2) & 0xff00)
    if rate >= 10000:
        basef = SR_MHZ(100)
        extra = 2
    else:
        basef = SR_MHZ(1)
        extra = 1
    print("%15d Hz 0x%04x clkdiv 0x%04x %-3d %f"
        % (rate, val, clkdiv, clkdiv, basef/(clkdiv+extra)))

200000000 Hz 0x0205 clkdiv 0x0005 5   14285714.285714
100000000 Hz 0x0105 clkdiv 0x0005 5   14285714.285714
 50000000 Hz 0x0005 clkdiv 0x0005 5   14285714.285714
 20000000 Hz 0x0303 clkdiv 0x0003 3   20000000.000000
 10000000 Hz 0x0308 clkdiv 0x0008 8   10000000.000000
  5000000 Hz 0x0312 clkdiv 0x0012 18  5000000.000000
  2000000 Hz 0x0330 clkdiv 0x0030 48  2000000.000000
  1000000 Hz 0x0362 clkdiv 0x0062 98  1000000.000000
   500000 Hz 0x03c6 clkdiv 0x00c6 198 500000.000000
   200000 Hz 0x07f2 clkdiv 0x01f2 498 200000.000000
   100000 Hz 0x0fe6 clkdiv 0x03e6 998 100000.000000
    50000 Hz 0x1fce clkdiv 0x07ce 1998 50000.000000
    20000 Hz 0x4f86 clkdiv 0x1386 4998 20000.000000
    10000 Hz 0x9f0e clkdiv 0x270e 9998 10000.000000
     5000 Hz 0x03c7 clkdiv 0x00c7 199 5000.000000
     2000 Hz 0x07f3 clkdiv 0x01f3 499 2000.000000
     1000 Hz 0x0fe7 clkdiv 0x03e7 999 1000.000000
      500 Hz 0x1fcf clkdiv 0x07cf 1999 500.000000
      200 Hz 0x4f87 clkdiv 0x1387 4999 200.000000
      100 Hz 0x9f0f clkdiv 0x270f 9999 100.000000
*/


static const struct rate_map rate_map[] = {
	{ SR_MHZ(200), 0x0205, 0 },
	{ SR_MHZ(100), 0x0105, 0 },
	{ SR_MHZ(50),  0x0005, 0 },
	{ SR_MHZ(20),  0x0303, 0 },
	{ SR_MHZ(10),  0x0308, 0 },
	{ SR_MHZ(5),   0x0312, 0 },
	{ SR_MHZ(2),   0x0330, 0 },
	{ SR_MHZ(1),   0x0362, 0 },
	{ SR_KHZ(500), 0x03c6, 0 },
	{ SR_KHZ(200), 0x07f2, 0 },
	{ SR_KHZ(100), 0x0fe6, 0 },
	{ SR_KHZ(50),  0x1fce, 0 },
	{ SR_KHZ(20),  0x4f86, 0 },
	{ SR_KHZ(10),  0x9f0e, 0 },
	{ SR_KHZ(5),   0x03c7, 0x20 },
	{ SR_KHZ(2),   0x07f3, 0x20 },
	{ SR_KHZ(1),   0x0fe7, 0x20 },
	{ SR_HZ(500),  0x1fcf, 0x20 },
	{ SR_HZ(200),  0x4f87, 0x20 },
	{ SR_HZ(100),  0x9f0f, 0x20 },
};

static const struct {
	float voltage;
	uint16_t val;
} la_threshold_map[] = {
	{1.2, 0x8600},
	{1.5, 0x8770},
	{1.8, 0x88ff},
	{2.5, 0x8c70},
	{3.0, 0x8eff},
	{3.3, 0x8eff},
};
static const int DEFAULT_LA_THRESHOLD = 5; // 3.3v

/* serial protocol */
#define mso_trans(a, v) \
	(((v) & 0x3f) | (((v) & 0xc0) << 6) | (((a) & 0xf) << 8) | \
	((~(v) & 0x20) << 1) | ((~(v) & 0x80) << 7))

static const char mso_head[] = { 0x40, 0x4c, 0x44, 0x53, 0x7e };
static const char mso_foot[] = { 0x7e };

static uint16_t mso_calc_offset_raw_from_mv(struct dev_context *devc, float mv);
static int mso_dac_out(const struct sr_dev_inst *sdi, uint16_t val);
static int mso_configure_trigger(const struct sr_dev_inst *sdi);
static int mso_clkrate_out(const struct sr_dev_inst *sdi);
static int mso_configure_dac_offset(const struct sr_dev_inst *sdi);
static int mso_configure_threshold_level(const struct sr_dev_inst *sdi);
static int mso_configure_channels(const struct sr_dev_inst *sdi);

static int mso_send_control_message(struct sr_serial_dev_inst *serial,
				     uint16_t payload[], int n)
{
	int i, w, ret, s = n * 2 + sizeof(mso_head) + sizeof(mso_foot);
	char *p, *buf;

	ret = SR_ERR;

	// if (serial->fd < 0)
	// 	goto ret;

	buf = g_malloc(s);

	p = buf;
	memcpy(p, mso_head, sizeof(mso_head));
	p += sizeof(mso_head);

	for (i = 0; i < n; i++) {
		WB16(p, payload[i]);
		p += sizeof(uint16_t);
	}
	memcpy(p, mso_foot, sizeof(mso_foot));

	w = 0;
	while (w < s) {
		ret = serial_write_blocking(serial, buf + w, s - w, 1000); // TODO: timeout?
		if (ret < 0) {
			ret = SR_ERR;
			goto free;
		}
		w += ret;
	}
	ret = SR_OK;
free:
	g_free(buf);
// ret:
	return ret;
}

SR_PRIV uint64_t *mso_get_sample_rates(size_t *ret_len)
{
	uint64_t *rates = g_new(uint64_t, G_N_ELEMENTS(rate_map));
	for (size_t i = 0; i < G_N_ELEMENTS(rate_map); i++) {
		rates[i] = rate_map[i].rate;
	}

	*ret_len = G_N_ELEMENTS(rate_map);
	return rates;
}

static int mso_configure_trigger(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint16_t threshold_value = mso_calc_raw_from_mv(devc,
		devc->dso_trigger_voltage + devc->v_offset);

	threshold_value = 0x153C;
	uint8_t trigger_config = 0;

	if (devc->trigger_slope)
		trigger_config |= 0x04;	//Trigger on falling edge

	switch (devc->trigger_outsrc) {
	case 1:
		trigger_config |= 0x00;	//Trigger pulse output
		break;
	case 2:
		trigger_config |= 0x08;	//PWM DAC from the pattern generator buffer
		break;
	case 3:
		trigger_config |= 0x18;	//White noise
		break;
	}

	switch (devc->trigger_chan) {
	case 0:
		trigger_config |= 0x00;	//DSO level trigger //b00000000
		break;
	case 1:
		trigger_config |= 0x20;	//DSO level trigger & width < trigger_width
		break;
	case 2:
		trigger_config |= 0x40;	//DSO level trigger & width >= trigger_width
		break;
	case 3:
		trigger_config |= 0x60;	//LA combination trigger
		break;
	}

	//Last bit of trigger config reg 4 needs to be 1 for trigger enable,
	//otherwise the trigger is not enabled
	if (devc->use_trigger)
		trigger_config |= 0x80;

	uint16_t ops[18];
	ops[0] = mso_trans(3, threshold_value & 0xff);
	//The trigger_config also holds the 2 MSB bits from the threshold value
	ops[1] = mso_trans(4, trigger_config | ((threshold_value >> 8) & 0x03));
	ops[2] = mso_trans(5, devc->la_trigger);
	ops[3] = mso_trans(6, devc->la_trigger_mask);
	ops[4] = mso_trans(7, devc->trigger_holdoff[0]);
	ops[5] = mso_trans(8, devc->trigger_holdoff[1]);

	ops[6] = mso_trans(11,
			   devc->dso_trigger_width /
			   SR_HZ_TO_NS(devc->cur_rate));

	/* Select the SPI/I2C trigger config bank */
	ops[7] = mso_trans(REG_CTL2, (devc->ctlbase2 | BITS_CTL2_BANK(2)));
	/* Configure the SPI/I2C protocol trigger */
	ops[8] = mso_trans(REG_PT_WORD(0), devc->protocol_trigger.word[0]);
	ops[9] = mso_trans(REG_PT_WORD(1), devc->protocol_trigger.word[1]);
	ops[10] = mso_trans(REG_PT_WORD(2), devc->protocol_trigger.word[2]);
	ops[11] = mso_trans(REG_PT_WORD(3), devc->protocol_trigger.word[3]);
	ops[12] = mso_trans(REG_PT_MASK(0), devc->protocol_trigger.mask[0]);
	ops[13] = mso_trans(REG_PT_MASK(1), devc->protocol_trigger.mask[1]);
	ops[14] = mso_trans(REG_PT_MASK(2), devc->protocol_trigger.mask[2]);
	ops[15] = mso_trans(REG_PT_MASK(3), devc->protocol_trigger.mask[3]);
	ops[16] = mso_trans(REG_PT_SPIMODE, devc->protocol_trigger.spimode);
	/* Select the default config bank */
	ops[17] = mso_trans(REG_CTL2, devc->ctlbase2);

	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
}

static int mso_configure_threshold_level(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	return mso_dac_out(sdi, la_threshold_map[devc->la_threshold].val);
}

static int mso_configure_dac_offset(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	uint16_t val = mso_calc_offset_raw_from_mv(devc, devc->v_offset * 1000);
	return mso_dac_out(sdi, val);
}

SR_PRIV int mso_read_buffer(struct sr_dev_inst *sdi)
{
	uint16_t ops[] = { mso_trans(REG_BUFFER, 0) };
	struct dev_context *devc = sdi->priv;

	sr_dbg("Requesting buffer dump.");
	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
}

static void mso_calc_mv_from_raw(struct dev_context *devc,
	const uint16_t *sample_in, float *mv_out, size_t nsamps)
{
	const float vbit = devc->vbit;
	const float attn = devc->dso_probe_attn;
	for (size_t i = 0; i < nsamps; i++) {
		mv_out[i] = (512.f - sample_in[i]) * vbit * attn * 0.001f;
	}
}

SR_PRIV int mso_configure_hw(const struct sr_dev_inst *sdi)
{
	int ret;

	if (mso_configure_channels(sdi) != SR_OK) {
		sr_err("Failed to configure channels.");
		return SR_ERR;
	}

	ret = mso_clkrate_out(sdi);
	if (ret != SR_OK)
		return ret;

	/* set dac offset */
	ret = mso_configure_dac_offset(sdi);
	if (ret != SR_OK)
		return ret;

	ret = mso_configure_threshold_level(sdi);
	if (ret != SR_OK)
		return ret;

	ret = mso_configure_trigger(sdi);
	if (ret != SR_OK)
		return ret;

	return SR_OK;

}

SR_PRIV int mso_arm(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	uint8_t bit_coupling = 0;
	if (devc->dc_coupling) {
		bit_coupling = 0x80;
	}

	uint16_t ops[] = {
		mso_trans(REG_CTL1, BIT_CTL1_BASE | bit_coupling | BIT_CTL1_RESETFSM),
		mso_trans(REG_CTL1, BIT_CTL1_BASE | bit_coupling | BIT_CTL1_ARM),
		mso_trans(REG_CTL1, BIT_CTL1_BASE | bit_coupling),
	};

	sr_dbg("Requesting trigger arm.");
	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
}

SR_PRIV int mso_force_capture(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint16_t ops[] = {
		mso_trans(REG_CTL1, BIT_CTL1_BASE | BIT_CTL1_FORCE),
		mso_trans(REG_CTL1, BIT_CTL1_BASE),
	};

	sr_dbg("Requesting forced capture.");
	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
}

static int mso_dac_out(const struct sr_dev_inst *sdi, uint16_t val)
{
	struct dev_context *devc = sdi->priv;
	uint16_t ops[] = {
		mso_trans(REG_DAC1, (val >> 8) & 0xff),
		mso_trans(REG_DAC2, val & 0xff),
		mso_trans(REG_CTL1, BIT_CTL1_BASE | BIT_CTL1_CONFIGDAC),
	};

	sr_dbg("Setting dac word to 0x%x.", val);
	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
}

// For triggers, offset should be already applied to the mv input
SR_PRIV uint16_t mso_calc_raw_from_mv(struct dev_context *devc, float mv)
{
	return (uint16_t) (512 - (uint16_t)((mv / devc->dso_probe_attn) / devc->vbit));
}

// For dso offset config.
static uint16_t mso_calc_offset_raw_from_mv(struct dev_context *devc, float mv)
{
	int val = devc->dac_offset - ((mv / devc->dso_probe_attn) / devc->offset_vbit);
	return CLAMP(val, 0, 0xfff);
}

SR_PRIV int mso_parse_serial(const char *iSerial, const char *iProduct,
			     struct dev_context *devc)
{
	unsigned int u1, u2, u3, u4, u5, u6;

	(void)iProduct;

	/* FIXME: This code is in the original app, but I think its
	 * used only for the GUI */
	/*    if (strstr(iProduct, "REV_02") || strstr(iProduct, "REV_03"))
	   devc->num_sample_rates = 0x16;
	   else
	   devc->num_sample_rates = 0x10; */

	/* parse iSerial */
	if (iSerial[0] != '4' || sscanf(iSerial, "%5u%3u%3u%1u%1u%6u",
					&u1, &u2, &u3, &u4, &u5, &u6) != 6)
		return SR_ERR;
	devc->hwmodel = u4;
	devc->hwrev = u5;
	devc->vbit = (double)u1 / 10000;
	devc->dac_offset = u2;
	devc->offset_vbit = 3000.0 / u3;

	printf("vbit %f, dac_offset %hu, offset_vbit %f\n",
		devc->vbit, devc->dac_offset, devc->offset_vbit);

	// if (devc->vbit == 0)
	// 	devc->vbit = 4.19195; // TODO remove
	// if (devc->dac_offset == 0)
	// 	devc->dac_offset = 0x1ff; // TODO remove
	// if (devc->offset_range == 0)
	// 	devc->offset_range = 0x17d; // TODO remove

	/*
	 * FIXME: There is more code on the original software to handle
	 * bigger iSerial strings, but as I can't test on my device
	 * I will not implement it yet.

	 * This corresponds to vbit200 etc settings in mso19fcgi Parse19Sn().
	 */

	return SR_OK;
}

SR_PRIV int mso_reset_adc(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint16_t ops[2];

	ops[0] = mso_trans(REG_CTL1, BIT_CTL1_RESETADC);
	ops[1] = mso_trans(REG_CTL1, 0);

	sr_dbg("Requesting ADC reset.");
	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
}

SR_PRIV int mso_reset_fsm(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint16_t ops[1];

	ops[0] = mso_trans(REG_CTL1, BIT_CTL1_RESETFSM);

	sr_dbg("Requesting ADC reset.");
	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
}

// SR_PRIV int mso_toggle_led(struct sr_dev_inst *sdi, int state)
// {
// 	struct dev_context *devc = sdi->priv;
// 	uint16_t ops[1];

// 	devc->ctlbase1 &= ~BIT_CTL1_LED;
// 	if (state)
// 		devc->ctlbase1 |= BIT_CTL1_LED;
// 	ops[0] = mso_trans(REG_CTL1, devc->ctlbase1);

// 	sr_dbg("Requesting LED toggle.");
// 	return mso_send_control_message(devc->serial, ARRAY_AND_SIZE(ops));
// }

SR_PRIV void mso_stop_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;


	devc = sdi->priv;
	mso_reset_fsm(sdi);

	// TODO: should it call mso_receive_data to flush input?

	serial_source_remove(sdi->session, devc->serial);

	std_session_send_df_end(sdi);
}

SR_PRIV int mso_clkrate_out(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_serial_dev_inst *serial = devc->serial;

	uint16_t ops[] = {
		mso_trans(REG_CLKRATE1, (devc->cur_rate_regval >> 8) & 0xff),
		mso_trans(REG_CLKRATE2, devc->cur_rate_regval & 0xff),
	};

	sr_dbg("Setting clkrate word to 0x%x.", devc->cur_rate_regval);
	return mso_send_control_message(serial, ARRAY_AND_SIZE(ops));
}

SR_PRIV int mso_set_rate(const struct sr_dev_inst *sdi, uint32_t rate)
{
	struct dev_context *devc = sdi->priv;
	unsigned int i;
	int ret = SR_ERR;

	for (i = 0; i < G_N_ELEMENTS(rate_map); i++) {
		if (rate_map[i].rate == rate) {
			devc->cur_rate = rate;
			devc->cur_rate_regval = rate_map[i].val;
			devc->slowmode = rate_map[i].slowmode;
			if (ret == SR_OK) {
			} else {
				sr_err("Setting rate failed");
			}
			return ret;
		}
	}

	if (ret != SR_OK)
		sr_err("Unsupported rate.");

	return ret;
}

SR_PRIV int mso_check_trigger(struct sr_serial_dev_inst *serial, uint8_t *info)
{
	uint16_t ops[] = { mso_trans(REG_TRIGGER, 0) };
	int ret;

	sr_dbg("Requesting trigger state.");
	ret = mso_send_control_message(serial, ARRAY_AND_SIZE(ops));
	if (!info || ret != SR_OK)
		return ret;

	uint8_t buf = 0;
	if (serial_read_blocking(serial, &buf, 1, 1000) != 1)	/* FIXME: Need timeout */
		ret = SR_ERR;
	if (!info)
		*info = buf;

	sr_dbg("Trigger state is: 0x%x.", *info);
	return ret;
}

SR_PRIV int mso_receive_data(int fd, int revents, void *cb_data)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_dev_inst *sdi = cb_data;
	struct dev_context *devc = sdi->priv;
	int i;

	(void)revents;

	uint8_t in[1024];
	size_t s = serial_read_blocking(devc->serial, in, sizeof(in), 1000); // TODO: timeout

	if (s <= 0)
		return FALSE;

	/* Check if we triggered, then send a command that we are ready
	 * to read the data */
	if (devc->trigger_state != MSO_TRIGGER_DATAREADY) {
		devc->trigger_state = in[0];
		if (devc->trigger_state == MSO_TRIGGER_DATAREADY) {
			mso_read_buffer(sdi);
			devc->buffer_n = 0;
		} else {
			mso_check_trigger(devc->serial, NULL);
		}
		return TRUE;
	}

	/* the hardware always dumps 1024 samples, 24bits each */
	if (devc->buffer_n < 3072) {
		memcpy(devc->buffer + devc->buffer_n, in, s);
		devc->buffer_n += s;
	}
	if (devc->buffer_n < 3072)
		return TRUE;

	/* do the conversion */
	uint8_t logic_out[1024];
	uint16_t analog_out[1024];
	for (i = 0; i < 1024; i++) {
		analog_out[i] = (devc->buffer[i * 3] & 0x3f) |
		    (((uint16_t)devc->buffer[i * 3 + 1] & 0xf) << 6);
		logic_out[i] = ((devc->buffer[i * 3 + 1] & 0x30) >> 4) |
		    ((devc->buffer[i * 3 + 2] & 0x3f) << 2);
	}

	float analog_volts[1024];
	mso_calc_mv_from_raw(devc, analog_out, analog_volts, 1024);

	packet.type = SR_DF_LOGIC;
	packet.payload = &logic;
	logic.length = 1024;
	logic.unitsize = 1;
	logic.data = logic_out;
	sr_session_send(sdi, &packet);

	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	sr_analog_init(&analog, &encoding, &meaning, &spec, 3); // TODO digits
	analog.num_samples = 1024;
	analog.data = analog_volts;
	// first channel is DSO
	struct sr_channel *ch = (struct sr_channel *)sdi->channels->data;
	analog.meaning->channels = g_slist_append(NULL, ch);
	analog.meaning->mq = SR_MQ_VOLTAGE;
	analog.meaning->unit = SR_UNIT_VOLT;
	analog.meaning->mqflags = 0;
	sr_session_send(sdi, &packet);
	g_slist_free(analog.meaning->channels);

	devc->num_samples += 1024;

	if (devc->limit_samples && devc->num_samples >= devc->limit_samples) {
		sr_info("Requested number of samples reached.");
		sr_dev_acquisition_stop(sdi);
	}

	return TRUE;
}

static int mso_configure_channels(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	GSList *l;

	devc = sdi->priv;

	devc->la_trigger_mask = 0xFF;	//the mask for the LA_TRIGGER (bits set to 0 matter, those set to 1 are ignored).
	devc->la_trigger = 0x00;	//The value of the LA byte that generates a trigger event (in that mode).
	devc->dso_trigger_voltage = 3;
	devc->dso_probe_attn = 1; // TODO use SR_CONF_PROBE_FACTOR
	devc->trigger_outsrc = 0;
	devc->trigger_chan = 3;	//LA combination trigger
	devc->use_trigger = FALSE;
	devc->la_threshold = DEFAULT_LA_THRESHOLD;

	for (l = sdi->channels; l; l = l->next) {
		ch = (struct sr_channel *)l->data;
		if (ch->enabled == FALSE)
			continue;

		// TODO matt: figure how trigger API has changed

		// int channel_bit = 1 << (ch->index);
		// if (!(ch->trigger))
		// 	continue;



		// devc->use_trigger = TRUE;
		// //Configure trigger mask and value.
		// for (tc = ch->trigger; *tc; tc++) {
		// 	devc->la_trigger_mask &= ~channel_bit;
		// 	if (*tc == '1')
		// 		devc->la_trigger |= channel_bit;
		// }
	}

	return SR_OK;
}
