/*
 * This file is part of the sigrok project.
 *
 * Copyright (C) 2012 Bert Vermeulen <bert@biot.com>
 * With protocol information from the hantekdso project,
 * Copyright (C) 2008 Oleg Khudyakov <prcoder@gmail.com>
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

#include "libsigrok.h"
#include "libsigrok-internal.h"
#include "dso.h"
#include <string.h>
#include <glib.h>
#include <libusb.h>

extern struct sr_dev_driver hantek_dso_driver_info;

static int send_begin(struct dev_context *devc)
{
	int ret;
	unsigned char buffer[] = {0x0f, 0x03, 0x03, 0x03, 0x68, 0xac, 0xfe,
	0x00, 0x01, 0x00};

	sr_dbg("Sending CTRL_BEGINCOMMAND.");

	if ((ret = libusb_control_transfer(devc->usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR, CTRL_BEGINCOMMAND,
			0, 0, buffer, sizeof(buffer), 200)) != sizeof(buffer)) {
		sr_err("Failed to send begincommand: %d.", ret);
		return SR_ERR;
	}

	return SR_OK;
}

static int send_bulkcmd(struct dev_context *devc, uint8_t *cmdstring, int cmdlen)
{
	int ret, tmp;

	if (send_begin(devc) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(devc->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, cmdlen, &tmp, 200)) != 0)
		return SR_ERR;

	return SR_OK;
}

SR_PRIV int dso_getmps(libusb_device *dev)
{
	struct libusb_device_descriptor des;
	struct libusb_config_descriptor *conf_dsc;
	const struct libusb_interface_descriptor *intf_dsc;
	int mps;

	if (libusb_get_device_descriptor(dev, &des) != 0)
		return 0;

	if (des.bNumConfigurations != 1)
		return 0;

	if (libusb_get_config_descriptor(dev, 0, &conf_dsc) != 0)
		return 0;

	mps = 0;
	intf_dsc = &(conf_dsc->interface[0].altsetting[0]);
	if (intf_dsc->bNumEndpoints != 2)
		goto err;

	if ((intf_dsc->endpoint[0].bEndpointAddress & 0x8f) !=
	    (2 | LIBUSB_ENDPOINT_OUT))
		/* The first endpoint should be 2 (outbound). */
		goto err;

	if ((intf_dsc->endpoint[1].bEndpointAddress & 0x8f) !=
	    (6 | LIBUSB_ENDPOINT_IN))
		/* The second endpoint should be 6 (inbound). */
		goto err;

	mps = intf_dsc->endpoint[1].wMaxPacketSize;

err:
	if (conf_dsc)
		libusb_free_config_descriptor(conf_dsc);

	return mps;
}

SR_PRIV int dso_open(struct sr_dev_inst *sdi)
{
	libusb_device **devlist;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc = hantek_dso_driver_info.priv;
	int err, skip, i;

	devc = sdi->priv;

	if (sdi->status == SR_ST_ACTIVE)
		/* already in use */
		return SR_ERR;

	skip = 0;
	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (i = 0; devlist[i]; i++) {
		if ((err = libusb_get_device_descriptor(devlist[i], &des))) {
			sr_err("Failed to get device descriptor: %d.", err);
			continue;
		}

		if (des.idVendor != devc->profile->fw_vid
		    || des.idProduct != devc->profile->fw_pid)
			continue;

		if (sdi->status == SR_ST_INITIALIZING) {
			if (skip != sdi->index) {
				/* Skip devices of this type that aren't the one we want. */
				skip += 1;
				continue;
			}
		} else if (sdi->status == SR_ST_INACTIVE) {
			/*
			 * This device is fully enumerated, so we need to find
			 * this device by vendor, product, bus and address.
			 */
			if (libusb_get_bus_number(devlist[i]) != devc->usb->bus
				|| libusb_get_device_address(devlist[i]) != devc->usb->address)
				/* this is not the one */
				continue;
		}

		if (!(err = libusb_open(devlist[i], &devc->usb->devhdl))) {
			if (devc->usb->address == 0xff)
				/*
				 * first time we touch this device after firmware upload,
				 * so we don't know the address yet.
				 */
				devc->usb->address = libusb_get_device_address(devlist[i]);

			if (!(devc->epin_maxpacketsize = dso_getmps(devlist[i])))
				sr_err("Wrong endpoint profile.");
			else {
				sdi->status = SR_ST_ACTIVE;
				sr_info("Opened device %d on %d.%d interface %d.",
					sdi->index, devc->usb->bus,
					devc->usb->address, USB_INTERFACE);
			}
		} else {
			sr_err("Failed to open device: %d.", err);
		}

		/* If we made it here, we handled the device (somehow). */
		break;
	}
	libusb_free_device_list(devlist, 1);

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR;

	return SR_OK;
}

SR_PRIV void dso_close(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	if (devc->usb->devhdl == NULL)
		return;

	sr_info("Closing device %d on %d.%d interface %d.", sdi->index,
		devc->usb->bus, devc->usb->address, USB_INTERFACE);
	libusb_release_interface(devc->usb->devhdl, USB_INTERFACE);
	libusb_close(devc->usb->devhdl);
	devc->usb->devhdl = NULL;
	sdi->status = SR_ST_INACTIVE;

}

static int get_channel_offsets(struct dev_context *devc)
{
	GString *gs;
	int chan, v, ret;

	sr_dbg("Getting channel offsets.");

	ret = libusb_control_transfer(devc->usb->devhdl,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
			CTRL_READ_EEPROM, EEPROM_CHANNEL_OFFSETS, 0,
			(unsigned char *)&devc->channel_levels,
			sizeof(devc->channel_levels), 200);
	if (ret != sizeof(devc->channel_levels)) {
		sr_err("Failed to get channel offsets: %d.", ret);
		return SR_ERR;
	}

	/* Comes in as 16-bit numbers with the second byte always 0 on
	 * the DSO-2090. Guessing this is supposed to be big-endian,
	 * since that's how voltage offsets are submitted back to the DSO.
	 * Convert to host order now, so we can use them natively.
	 */
	for (chan = 0; chan < 2; chan++) {
		for (v = 0; v < 9; v++) {
			devc->channel_levels[chan][v][0] =
				g_ntohs(devc->channel_levels[chan][v][0]);
			devc->channel_levels[chan][v][1] =
				g_ntohs(devc->channel_levels[chan][v][1]);
		}
	}

	if (sr_log_loglevel_get() >= SR_LOG_DBG) {
		gs = g_string_sized_new(128);
		for (chan = 0; chan < 2; chan++) {
			g_string_printf(gs, "CH%d:", chan + 1);
			for (v = 0; v < 9; v++) {
				g_string_append_printf(gs, " %.4x-%.4x",
					devc->channel_levels[chan][v][0],
					devc->channel_levels[chan][v][1]);
			}
			sr_dbg(gs->str);
		}
		g_string_free(gs, TRUE);
	}

	return SR_OK;
}

SR_PRIV int dso_set_trigger_samplerate(struct dev_context *devc)
{
	int ret, tmp;
	uint8_t cmdstring[12];
	uint16_t timebase_small[] = { 0xffff, 0xfffc, 0xfff7, 0xffe8, 0xffce,
		0xff9c, 0xff07, 0xfe0d, 0xfc19, 0xf63d, 0xec79, 0xd8f1 };
	uint16_t timebase_large[] = { 0xffff, 0x0000, 0xfffc, 0xfff7, 0xffe8,
		0xffce, 0xff9d, 0xff07, 0xfe0d, 0xfc19, 0xf63d, 0xec79 };

	sr_dbg("Preparing CMD_SET_TRIGGER_SAMPLERATE.");

	memset(cmdstring, 0, sizeof(cmdstring));
	/* Command */
	cmdstring[0] = CMD_SET_TRIGGER_SAMPLERATE;

	/* Trigger source */
	sr_dbg("Trigger source %s.", devc->triggersource);
	if (!strcmp("CH2", devc->triggersource))
		tmp = 0;
	else if (!strcmp("CH1", devc->triggersource))
		tmp = 1;
	else if (!strcmp("EXT", devc->triggersource))
		tmp = 2;
	else {
		sr_err("Invalid trigger source: '%s'.", devc->triggersource);
		return SR_ERR_ARG;
	}
	cmdstring[2] = tmp;

	/* Frame size */
	sr_dbg("Frame size: %d.", devc->framesize);
	cmdstring[2] |= (devc->framesize == FRAMESIZE_SMALL ? 0x01 : 0x02) << 2;

	/* Timebase fast */
	sr_dbg("Time base index: %d.", devc->timebase);
	switch (devc->framesize) {
	case FRAMESIZE_SMALL:
		if (devc->timebase < TIME_20us)
			tmp = 0;
		else if (devc->timebase == TIME_20us)
			tmp = 1;
		else if (devc->timebase == TIME_40us)
			tmp = 2;
		else if (devc->timebase == TIME_100us)
			tmp = 3;
		else if (devc->timebase >= TIME_200us)
			tmp = 4;
		break;
	case FRAMESIZE_LARGE:
		if (devc->timebase < TIME_40us) {
			sr_err("Timebase < 40us only supported with 10K buffer.");
			return SR_ERR_ARG;
		}
		else if (devc->timebase == TIME_40us)
			tmp = 0;
		else if (devc->timebase == TIME_100us)
			tmp = 2;
		else if (devc->timebase == TIME_200us)
			tmp = 3;
		else if (devc->timebase >= TIME_400us)
			tmp = 4;
		break;
	}
	cmdstring[2] |= (tmp & 0x07) << 5;

	/* Enabled channels: 00=CH1 01=CH2 10=both */
	sr_dbg("Channels CH1=%d CH2=%d", devc->ch1_enabled, devc->ch2_enabled);
	tmp = (((devc->ch2_enabled ? 1 : 0) << 1) + (devc->ch1_enabled ? 1 : 0)) - 1;
	cmdstring[3] = tmp;

	/* Fast rates channel */
	/* TODO: Is this right? */
	tmp = devc->timebase < TIME_10us ? 1 : 0;
	cmdstring[3] |= tmp << 2;

	/* Trigger slope: 0=positive 1=negative */
	/* TODO: Does this work? */
	sr_dbg("Trigger slope: %d.", devc->triggerslope);
	cmdstring[3] |= (devc->triggerslope == SLOPE_NEGATIVE ? 1 : 0) << 3;

	/* Timebase slow */
	if (devc->timebase < TIME_100us)
		tmp = 0;
	else if (devc->timebase > TIME_400ms)
		tmp = 0xffed;
	else {
		if (devc->framesize == FRAMESIZE_SMALL)
			tmp = timebase_small[devc->timebase - 3];
		else
			tmp = timebase_large[devc->timebase - 3];
	}
	cmdstring[4] = tmp & 0xff;
	cmdstring[5] = (tmp >> 8) & 0xff;

	/* Horizontal trigger position */
	sr_dbg("Trigger position: %3.2f.", devc->triggerposition);
	tmp = 0x77fff + 0x8000 * devc->triggerposition;
	cmdstring[6] = tmp & 0xff;
	cmdstring[7] = (tmp >> 8) & 0xff;
	cmdstring[10] = (tmp >> 16) & 0xff;

	if (send_begin(devc) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(devc->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to set trigger/samplerate: %d.", ret);
		return SR_ERR;
	}
	sr_dbg("Sent CMD_SET_TRIGGER_SAMPLERATE.");

	return SR_OK;
}

SR_PRIV int dso_set_filters(struct dev_context *devc)
{
	int ret, tmp;
	uint8_t cmdstring[8];

	sr_dbg("Preparing CMD_SET_FILTERS.");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_SET_FILTERS;
	cmdstring[1] = 0x0f;
	if (devc->filter_ch1) {
		sr_dbg("Turning on CH1 filter.");
		cmdstring[2] |= 0x80;
	}
	if (devc->filter_ch2) {
		sr_dbg("Turning on CH2 filter.");
		cmdstring[2] |= 0x40;
	}
	if (devc->filter_trigger) {
		/* TODO: supported on the DSO-2090? */
		sr_dbg("Turning on trigger filter.");
		cmdstring[2] |= 0x20;
	}

	if (send_begin(devc) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(devc->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to set filters: %d", ret);
		return SR_ERR;
	}
	sr_dbg("Sent CMD_SET_FILTERS.");

	return SR_OK;
}

SR_PRIV int dso_set_voltage(struct dev_context *devc)
{
	int ret, tmp;
	uint8_t cmdstring[8];

	sr_dbg("Preparing CMD_SET_VOLTAGE.");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_SET_VOLTAGE;
	cmdstring[1] = 0x0f;
	cmdstring[2] = 0x30;

	/* CH1 volts/div is encoded in bits 0-1 */
	sr_dbg("CH1 vdiv index: %d.", devc->voltage_ch1);
	switch (devc->voltage_ch1) {
	case VDIV_1V:
	case VDIV_100MV:
	case VDIV_10MV:
		cmdstring[2] |= 0x00;
		break;
	case VDIV_2V:
	case VDIV_200MV:
	case VDIV_20MV:
		cmdstring[2] |= 0x01;
		break;
	case VDIV_5V:
	case VDIV_500MV:
	case VDIV_50MV:
		cmdstring[2] |= 0x02;
		break;
	}

	/* CH2 volts/div is encoded in bits 2-3 */
	sr_dbg("CH2 vdiv index: %d.", devc->voltage_ch2);
	switch (devc->voltage_ch2) {
	case VDIV_1V:
	case VDIV_100MV:
	case VDIV_10MV:
		cmdstring[2] |= 0x00;
		break;
	case VDIV_2V:
	case VDIV_200MV:
	case VDIV_20MV:
		cmdstring[2] |= 0x04;
		break;
	case VDIV_5V:
	case VDIV_500MV:
	case VDIV_50MV:
		cmdstring[2] |= 0x08;
		break;
	}

	if (send_begin(devc) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(devc->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to set voltage: %d.", ret);
		return SR_ERR;
	}
	sr_dbg("Sent CMD_SET_VOLTAGE.");

	return SR_OK;
}

SR_PRIV int dso_set_relays(struct dev_context *devc)
{
	GString *gs;
	int ret, i;
	uint8_t relays[17] = { 0x00, 0x04, 0x08, 0x02, 0x20, 0x40, 0x10, 0x01,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	sr_dbg("Preparing CTRL_SETRELAYS.");

	if (devc->voltage_ch1 < VDIV_1V)
		relays[1] = ~relays[1];

	if (devc->voltage_ch1 < VDIV_100MV)
		relays[2] = ~relays[2];

	sr_dbg("CH1 coupling: %d.", devc->coupling_ch1);
	if (devc->coupling_ch1 != COUPLING_AC)
		relays[3] = ~relays[3];

	if (devc->voltage_ch2 < VDIV_1V)
		relays[4] = ~relays[4];

	if (devc->voltage_ch2 < VDIV_100MV)
		relays[5] = ~relays[5];

	sr_dbg("CH2 coupling: %d.", devc->coupling_ch1);
	if (devc->coupling_ch2 != COUPLING_AC)
		relays[6] = ~relays[6];

	if (!strcmp(devc->triggersource, "EXT"))
		relays[7] = ~relays[7];

	if (sr_log_loglevel_get() >= SR_LOG_DBG) {
		gs = g_string_sized_new(128);
		g_string_printf(gs, "Relays:");
		for (i = 0; i < 17; i++)
			g_string_append_printf(gs, " %.2x", relays[i]);
		sr_dbg(gs->str);
		g_string_free(gs, TRUE);
	}

	if ((ret = libusb_control_transfer(devc->usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR, CTRL_SETRELAYS,
			0, 0, relays, 17, 100)) != sizeof(relays)) {
		sr_err("Failed to set relays: %d.", ret);
		return SR_ERR;
	}
	sr_dbg("Sent CTRL_SETRELAYS.");

	return SR_OK;
}

SR_PRIV int dso_set_voffsets(struct dev_context *devc)
{
	int offset, ret;
	uint16_t *ch_levels;
	uint8_t offsets[17];

	sr_dbg("Preparing CTRL_SETOFFSET.");

	memset(offsets, 0, sizeof(offsets));
	/* Channel 1 */
	ch_levels = devc->channel_levels[0][devc->voltage_ch1];
	offset = (ch_levels[1] - ch_levels[0]) * devc->voffset_ch1 + ch_levels[0];
	offsets[0] = (offset >> 8) | 0x20;
	offsets[1] = offset & 0xff;
	sr_dbg("CH1 offset: %3.2f (%.2x%.2x).", devc->voffset_ch1,
	       offsets[0], offsets[1]);

	/* Channel 2 */
	ch_levels = devc->channel_levels[1][devc->voltage_ch2];
	offset = (ch_levels[1] - ch_levels[0]) * devc->voffset_ch2 + ch_levels[0];
	offsets[2] = (offset >> 8) | 0x20;
	offsets[3] = offset & 0xff;
	sr_dbg("CH2 offset: %3.2f (%.2x%.2x).", devc->voffset_ch2,
	       offsets[2], offsets[3]);

	/* Trigger */
	offset = MAX_VERT_TRIGGER * devc->voffset_trigger;
	offsets[4] = (offset >> 8) | 0x20;
	offsets[5] = offset & 0xff;
	sr_dbg("Trigger offset: %3.2f (%.2x%.2x).", devc->voffset_trigger,
			offsets[4], offsets[5]);

	if ((ret = libusb_control_transfer(devc->usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR, CTRL_SETOFFSET,
			0, 0, offsets, sizeof(offsets), 100)) != sizeof(offsets)) {
		sr_err("Failed to set offsets: %d.", ret);
		return SR_ERR;
	}
	sr_dbg("Sent CTRL_SETOFFSET.");

	return SR_OK;
}

SR_PRIV int dso_enable_trigger(struct dev_context *devc)
{
	int ret, tmp;
	uint8_t cmdstring[2];

	sr_dbg("Sending CMD_ENABLE_TRIGGER.");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_ENABLE_TRIGGER;
	cmdstring[1] = 0x00;

	if (send_begin(devc) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(devc->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to enable trigger: %d.", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_force_trigger(struct dev_context *devc)
{
	int ret, tmp;
	uint8_t cmdstring[2];

	sr_dbg("Sending CMD_FORCE_TRIGGER.");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_FORCE_TRIGGER;
	cmdstring[1] = 0x00;

	if (send_begin(devc) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(devc->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to force trigger: %d.", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_init(struct dev_context *devc)
{

	sr_dbg("Initializing DSO.");

	if (get_channel_offsets(devc) != SR_OK)
		return SR_ERR;

	if (dso_set_trigger_samplerate(devc) != SR_OK)
		return SR_ERR;

	if (dso_set_filters(devc) != SR_OK)
		return SR_ERR;

	if (dso_set_voltage(devc) != SR_OK)
		return SR_ERR;

	if (dso_set_relays(devc) != SR_OK)
		return SR_ERR;

	if (dso_set_voffsets(devc) != SR_OK)
		return SR_ERR;

	if (dso_enable_trigger(devc) != SR_OK)
		return SR_ERR;

	return SR_OK;
}

SR_PRIV int dso_get_capturestate(struct dev_context *devc,
				 uint8_t *capturestate,
				 uint32_t *trigger_offset)
{
	int ret, tmp, i;
	unsigned int bitvalue, toff;
	uint8_t cmdstring[2], inbuf[512];

	sr_dbg("Sending CMD_GET_CAPTURESTATE.");

	cmdstring[0] = CMD_GET_CAPTURESTATE;
	cmdstring[1] = 0;

	if ((ret = send_bulkcmd(devc, cmdstring, sizeof(cmdstring))) != SR_OK) {
		sr_dbg("Failed to send get_capturestate command: %d.", ret);
		return SR_ERR;
	}

	if ((ret = libusb_bulk_transfer(devc->usb->devhdl,
			DSO_EP_IN | LIBUSB_ENDPOINT_IN,
			inbuf, 512, &tmp, 100)) != 0) {
		sr_dbg("Failed to get capturestate: %d.", ret);
		return SR_ERR;
	}
	*capturestate = inbuf[0];
	toff = (inbuf[1] << 16) | (inbuf[3] << 8) | inbuf[2];

	/*
	 * This conversion comes from the openhantek project.
	 * Each set bit in the 24-bit value inverts all bits with a lower
	 * value. No idea why the device reports the trigger point this way.
	 */
	bitvalue = 1;
	for (i = 0; i < 24; i++) {
		/* Each set bit inverts all bits with a lower value. */
		if(toff & bitvalue)
			toff ^= bitvalue - 1;
		bitvalue <<= 1;
	}
	*trigger_offset = toff;

	return SR_OK;
}

SR_PRIV int dso_capture_start(struct dev_context *devc)
{
	int ret;
	uint8_t cmdstring[2];

	sr_dbg("Sending CMD_CAPTURE_START.");

	cmdstring[0] = CMD_CAPTURE_START;
	cmdstring[1] = 0;

	if ((ret = send_bulkcmd(devc, cmdstring, sizeof(cmdstring))) != SR_OK) {
		sr_err("Failed to send capture_start command: %d.", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_get_channeldata(struct dev_context *devc, libusb_transfer_cb_fn cb)
{
	struct libusb_transfer *transfer;
	int num_transfers, ret, i;
	uint8_t cmdstring[2];
	unsigned char *buf;

	sr_dbg("Sending CMD_GET_CHANNELDATA.");

	cmdstring[0] = CMD_GET_CHANNELDATA;
	cmdstring[1] = 0;

	if ((ret = send_bulkcmd(devc, cmdstring, sizeof(cmdstring))) != SR_OK) {
		sr_err("Failed to get channel data: %d.", ret);
		return SR_ERR;
	}

	/* TODO: DSO-2xxx only. */
	num_transfers = devc->framesize *
			sizeof(unsigned short) / devc->epin_maxpacketsize;
	sr_dbg("Queueing up %d transfers.", num_transfers);
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(devc->epin_maxpacketsize))) {
			sr_err("Failed to malloc USB endpoint buffer.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, devc->usb->devhdl,
				DSO_EP_IN | LIBUSB_ENDPOINT_IN, buf,
				devc->epin_maxpacketsize, cb, devc, 40);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %d.", ret);
			/* TODO: Free them all. */
			libusb_free_transfer(transfer);
			g_free(buf);
			return SR_ERR;
		}
	}

	return SR_OK;
}
