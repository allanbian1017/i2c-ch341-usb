/*
 * Driver for the CH341 USB-I2C adapter
 *
 * Copyright (c) 2016 Tse Lun Bien
 *
 * Derived from:
 *  i2c-ch341.c
 *  Copyright (c) 2014 Marco Gittler
 *
 *  i2c-tiny-usb.c
 *  Copyright (C) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>

#define DRV_VERSION "1.2"

#define CH341_I2C_LOW_SPEED 0      // low speed - 20kHz
#define CH341_I2C_STANDARD_SPEED 1 // standard speed - 100kHz
#define CH341_I2C_FAST_SPEED 2     // fast speed - 400kHz
#define CH341_I2C_HIGH_SPEED 3     // high speed - 750kHz

#define CH341_CMD_I2C_STREAM 0xAA

#define CH341_CMD_I2C_STM_STA 0x74
#define CH341_CMD_I2C_STM_STO 0x75
#define CH341_CMD_I2C_STM_OUT 0x80
#define CH341_CMD_I2C_STM_IN 0xC0
#define CH341_CMD_I2C_STM_SET 0x60
#define CH341_CMD_I2C_STM_END 0x00

#define CH341_PACKET_SIZE       32
#define CH341_USB_TIMEOUT_MS    2000
#define CH341_MAX_I2C_DATA      4096
#define CH341_MAX_CMD_SIZE      (CH341_MAX_I2C_DATA * 2 + 128)

static uint speed = CH341_I2C_STANDARD_SPEED; // Sets default speed as CH341_I2C_STANDARD_SPEED (100kHz)
module_param(speed, uint, 0644);
MODULE_PARM_DESC(speed, "I2C speed mode: 0=20kHz, 1=100kHz, 2=400kHz, 3=750kHz");

/*
 * The original driver probed the slave address before every transfer.  That
 * adds an extra I2C transaction which the vendor CH34xStreamI2C path does not
 * emit.  Keep it available for debugging/backwards behaviour, but default it
 * off so the bus waveform matches the submitted i2c_msg transaction.
 */
static bool precheck;
module_param(precheck, bool, 0644);
MODULE_PARM_DESC(precheck, "Probe slave address before each transfer (default: false)");

/* Structure to hold all of our device specific stuff */
struct i2c_ch341_usb {
	struct usb_device *usb_dev;  /* the usb device for this device */
	struct usb_interface *iface; /* the interface for this device */
	struct i2c_adapter adapter;  /* i2c related things */

	int ep_in;
	int ep_out;

	u8 in_buf[CH341_PACKET_SIZE];
	u8 out_buf[CH341_PACKET_SIZE];
};

struct ch341_stream_builder {
	u8 *buf;
	int len;
	int cap;
};

static int ch341_xfer(struct i2c_ch341_usb *dev, int out_len, int in_len);
static int ch341_xfer_buf(struct i2c_ch341_usb *dev, const u8 *out,
			  int out_len, u8 *in, int in_len);

/* ----- begin of CH341 stream builder ----------------------------------- */

static int ch341_put(struct ch341_stream_builder *sb, u8 v)
{
	if (sb->len >= sb->cap)
		return -EOVERFLOW;

	sb->buf[sb->len++] = v;
	return 0;
}

static int ch341_pad_to_packet(struct ch341_stream_builder *sb)
{
	int ret;

	while (sb->len % CH341_PACKET_SIZE) {
		ret = ch341_put(sb, 0x00);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ch341_append_stop_only(struct ch341_stream_builder *sb)
{
	int ret;

	ret = ch341_put(sb, CH341_CMD_I2C_STREAM);
	if (ret < 0)
		return ret;
	ret = ch341_put(sb, CH341_CMD_I2C_STM_STO);
	if (ret < 0)
		return ret;
	return ch341_put(sb, CH341_CMD_I2C_STM_END);
}

static u8 ch341_write_payload_byte(const struct i2c_msg *msg, int payload_pos)
{
	if (payload_pos == 0)
		return (u8)(msg->addr << 1);

	return msg->buf[payload_pos - 1];
}

static int ch341_append_write_msg(struct ch341_stream_builder *sb,
				  const struct i2c_msg *msg, bool send_stop)
{
	int total_payload = msg->len + 1; /* address byte + user payload */
	int payload_pos = 0;
	bool first_packet = true;
	bool stop_sent = false;
	int ret;

	while (payload_pos < total_payload) {
		int remaining = total_payload - payload_pos;
		int nonfinal_max = first_packet ? 28 : 29;
		int final_max = first_packet ? 27 : 28;
		int chunk;
		bool stop_in_this_packet = false;
		int i;

		if (send_stop && remaining <= final_max) {
			chunk = remaining;
			stop_in_this_packet = true;
		} else {
			chunk = min(remaining, nonfinal_max);
		}

		if (chunk <= 0 || chunk > nonfinal_max)
			return -EIO;

		ret = ch341_put(sb, CH341_CMD_I2C_STREAM);
		if (ret < 0)
			return ret;

		if (first_packet) {
			ret = ch341_put(sb, CH341_CMD_I2C_STM_STA);
			if (ret < 0)
				return ret;
		}

		ret = ch341_put(sb, CH341_CMD_I2C_STM_OUT | chunk);
		if (ret < 0)
			return ret;

		for (i = 0; i < chunk; i++, payload_pos++) {
			ret = ch341_put(sb, ch341_write_payload_byte(msg, payload_pos));
			if (ret < 0)
				return ret;
		}

		if (stop_in_this_packet) {
			ret = ch341_put(sb, CH341_CMD_I2C_STM_STO);
			if (ret < 0)
				return ret;
			ret = ch341_put(sb, CH341_CMD_I2C_STM_END);
			if (ret < 0)
				return ret;
			stop_sent = true;
		} else {
			ret = ch341_put(sb, CH341_CMD_I2C_STM_END);
			if (ret < 0)
				return ret;
		}

		first_packet = false;
	}

	if (send_stop && !stop_sent)
		return ch341_append_stop_only(sb);

	return 0;
}

static int ch341_append_read_tail(struct ch341_stream_builder *sb,
				  int read_len, bool send_stop)
{
	int remaining = read_len;
	int ret;

	while (remaining > 0) {
		if (remaining > CH341_PACKET_SIZE) {
			/* Read 32 bytes with ACK, terminate this stream segment. */
			ret = ch341_put(sb, CH341_CMD_I2C_STM_IN | CH341_PACKET_SIZE);
			if (ret < 0)
				return ret;
			ret = ch341_put(sb, CH341_CMD_I2C_STM_END);
			if (ret < 0)
				return ret;

			/* Vendor traces place the next AA on a 32-byte USB boundary. */
			ret = ch341_pad_to_packet(sb);
			if (ret < 0)
				return ret;

			remaining -= CH341_PACKET_SIZE;

			ret = ch341_put(sb, CH341_CMD_I2C_STREAM);
			if (ret < 0)
				return ret;
			continue;
		}

		/* Final read part: ACK all but the last byte, then NACK last byte. */
		if (remaining > 1) {
			ret = ch341_put(sb, CH341_CMD_I2C_STM_IN | (remaining - 1));
			if (ret < 0)
				return ret;
		}

		ret = ch341_put(sb, CH341_CMD_I2C_STM_IN); /* final byte, NACK */
		if (ret < 0)
			return ret;

		if (send_stop) {
			ret = ch341_put(sb, CH341_CMD_I2C_STM_STO);
			if (ret < 0)
				return ret;
		}

		ret = ch341_put(sb, CH341_CMD_I2C_STM_END);
		if (ret < 0)
			return ret;

		remaining = 0;
	}

	return 0;
}

static int ch341_append_read_msg(struct ch341_stream_builder *sb,
				 const struct i2c_msg *msg, bool send_stop)
{
	int ret;

	if (!msg->len)
		return -EINVAL;

	ret = ch341_put(sb, CH341_CMD_I2C_STREAM);
	if (ret < 0)
		return ret;
	ret = ch341_put(sb, CH341_CMD_I2C_STM_STA);
	if (ret < 0)
		return ret;
	ret = ch341_put(sb, CH341_CMD_I2C_STM_OUT | 1);
	if (ret < 0)
		return ret;
	ret = ch341_put(sb, (u8)((msg->addr << 1) | 1));
	if (ret < 0)
		return ret;

	return ch341_append_read_tail(sb, msg->len, send_stop);
}

static int ch341_append_write_read_msg(struct ch341_stream_builder *sb,
				       const struct i2c_msg *wmsg,
				       const struct i2c_msg *rmsg)
{
	int total_payload = wmsg->len + 1;
	int read_cmd_len;
	int combined_len;
	int ret;
	int i;

	if (!rmsg->len)
		return -EINVAL;

	/*
	 * Match the vendor CH34xStreamI2C register-read form when it fits:
	 *   AA 74 OUT(W) <sla+w,data...> 74 81 <sla+r> IN... 75 00
	 */
	read_cmd_len = (rmsg->len > CH341_PACKET_SIZE) ? 2 :
			 ((rmsg->len > 1) ? 4 : 3);
	combined_len = 1 + 1 + 1 + total_payload + 1 + 1 + 1 + read_cmd_len;

	if (combined_len <= CH341_PACKET_SIZE) {
		ret = ch341_put(sb, CH341_CMD_I2C_STREAM);
		if (ret < 0)
			return ret;
		ret = ch341_put(sb, CH341_CMD_I2C_STM_STA);
		if (ret < 0)
			return ret;
		ret = ch341_put(sb, CH341_CMD_I2C_STM_OUT | total_payload);
		if (ret < 0)
			return ret;
		ret = ch341_put(sb, (u8)(wmsg->addr << 1));
		if (ret < 0)
			return ret;
		for (i = 0; i < wmsg->len; i++) {
			ret = ch341_put(sb, wmsg->buf[i]);
			if (ret < 0)
				return ret;
		}

		ret = ch341_put(sb, CH341_CMD_I2C_STM_STA);
		if (ret < 0)
			return ret;
		ret = ch341_put(sb, CH341_CMD_I2C_STM_OUT | 1);
		if (ret < 0)
			return ret;
		ret = ch341_put(sb, (u8)((rmsg->addr << 1) | 1));
		if (ret < 0)
			return ret;

		return ch341_append_read_tail(sb, rmsg->len, true);
	}

	/* Generic fallback: no STOP between write phase and repeated START read. */
	ret = ch341_append_write_msg(sb, wmsg, false);
	if (ret < 0)
		return ret;

	return ch341_append_read_msg(sb, rmsg, true);
}

static int ch341_validate_msg(const struct i2c_msg *msg)
{
	if (msg->flags & I2C_M_TEN)
		return -EOPNOTSUPP;
	if (msg->flags & I2C_M_RECV_LEN)
		return -EOPNOTSUPP;
	if (msg->len > CH341_MAX_I2C_DATA)
		return -EMSGSIZE;
	if (msg->len && !msg->buf)
		return -EINVAL;

	return 0;
}

/* ----- end of CH341 stream builder ------------------------------------- */

/* ----- begin of i2c layer ---------------------------------------------- */

static int ch341_i2c_check_dev(struct i2c_ch341_usb *dev, u8 addr)
{
	int retval;

	dev->out_buf[0] = CH341_CMD_I2C_STREAM;
	dev->out_buf[1] = CH341_CMD_I2C_STM_STA;
	dev->out_buf[2] =
CH341_CMD_I2C_STM_OUT; /* NOTE: must be zero length otherwise it
					  messes up the device */
	dev->out_buf[3] = (addr << 1) | 0x1;
	dev->out_buf[4] =
		CH341_CMD_I2C_STM_IN; /* NOTE: zero length here as well */
	dev->out_buf[5] = CH341_CMD_I2C_STM_STO;
	dev->out_buf[6] = CH341_CMD_I2C_STM_END;

	retval = ch341_xfer(dev, 7, 1);
	if (retval < 0)
		return retval;

	if (dev->in_buf[0] & 0x80)
		return -ETIMEDOUT;

	return 0;
}

static int ch341_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
			  int num)
{
	struct i2c_ch341_usb *dev = (struct i2c_ch341_usb *)adapter->algo_data;
	struct ch341_stream_builder sb;
	u8 *cmd;
	int retval;
	int i;

	dev_dbg(&adapter->dev, "master xfer %d messages\n", num);

	if (!msgs || num <= 0)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		retval = ch341_validate_msg(&msgs[i]);
		if (retval < 0)
			return retval;
	}

	if (precheck) {
		retval = ch341_i2c_check_dev(dev, msgs[0].addr);
		if (retval < 0)
			return retval;
	}

	cmd = kmalloc(CH341_MAX_CMD_SIZE, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	sb.buf = cmd;
	sb.len = 0;
	sb.cap = CH341_MAX_CMD_SIZE;

	if (num == 1) {
		if (msgs[0].flags & I2C_M_RD)
			retval = ch341_append_read_msg(&sb, &msgs[0], true);
		else
			retval = ch341_append_write_msg(&sb, &msgs[0], true);
	} else if (num == 2) {
		if (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD))
			retval = ch341_append_write_read_msg(&sb, &msgs[0], &msgs[1]);
		else
			retval = -EOPNOTSUPP;
	} else {
		dev_err(&adapter->dev, "num > 2 is not supported\n");
		retval = -EOPNOTSUPP;
	}

	if (retval < 0)
		goto out_free;

	if (sb.len <= 0 || sb.len > CH341_MAX_CMD_SIZE) {
		retval = -EIO;
		goto out_free;
	}

	if (num == 1 && (msgs[0].flags & I2C_M_RD)) {
		retval = ch341_xfer_buf(dev, cmd, sb.len, msgs[0].buf, msgs[0].len);
	} else if (num == 2 && (msgs[1].flags & I2C_M_RD)) {
		retval = ch341_xfer_buf(dev, cmd, sb.len, msgs[1].buf, msgs[1].len);
	} else {
		retval = ch341_xfer_buf(dev, cmd, sb.len, NULL, 0);
	}

	if (retval < 0)
		goto out_free;

	retval = num;

out_free:
	kfree(cmd);
	return retval;
}

static u32 ch341_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/* ----- end of i2c layer ------------------------------------------------ */

/* ----- begin of usb layer ---------------------------------------------- */

static const struct i2c_algorithm ch341_i2c_algorithm = {
	.master_xfer = ch341_i2c_xfer,
	.functionality = ch341_i2c_func,
};

static const struct usb_device_id i2c_ch341_usb_table[] = {
	{USB_DEVICE(0x1a86, 0x5512)},
	{}};

MODULE_DEVICE_TABLE(usb, i2c_ch341_usb_table);

static int ch341_write_bulk(struct i2c_ch341_usb *dev, const u8 *out,
			    int out_len)
{
	int retval;
	int actual;

	if (out_len <= 0)
		return -EINVAL;

	dev_dbg(&dev->adapter.dev, "bulk_out %d bytes\n", out_len);

	retval = usb_bulk_msg(dev->usb_dev,
			      usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			      (void *)out, out_len, &actual, CH341_USB_TIMEOUT_MS);
	if (retval < 0)
		return retval;
	if (actual != out_len)
		return -EIO;

	return 0;
}

static int ch341_read_bulk(struct i2c_ch341_usb *dev, u8 *in, int in_len)
{
	int remaining = in_len;
	int pos = 0;

	while (remaining > 0) {
		int retval;
		int actual;
		int copy_len;

		memset(dev->in_buf, 0, sizeof(dev->in_buf));
		dev_dbg(&dev->adapter.dev, "bulk_in request %d bytes\n",
			CH341_PACKET_SIZE);

		retval = usb_bulk_msg(dev->usb_dev,
				      usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
				      dev->in_buf, CH341_PACKET_SIZE, &actual,
				      CH341_USB_TIMEOUT_MS);
		if (retval < 0)
			return retval;
		if (actual <= 0)
			return -EIO;

		copy_len = min(actual, remaining);
		memcpy(in + pos, dev->in_buf, copy_len);
		pos += copy_len;
		remaining -= copy_len;

		if (remaining > 0 && actual < CH341_PACKET_SIZE)
			return -EREMOTEIO;
	}

	return pos;
}

static int ch341_xfer_buf(struct i2c_ch341_usb *dev, const u8 *out,
			  int out_len, u8 *in, int in_len)
{
	int retval;

	retval = ch341_write_bulk(dev, out, out_len);
	if (retval < 0)
		return retval;

	if (!in_len)
		return 0;
	if (!in)
		return -EINVAL;

	return ch341_read_bulk(dev, in, in_len);
}

static int ch341_xfer(struct i2c_ch341_usb *dev, int out_len, int in_len)
{
	return ch341_xfer_buf(dev, dev->out_buf, out_len, dev->in_buf, in_len);
}

static void i2c_ch341_usb_free(struct i2c_ch341_usb *dev)
{
	usb_put_dev(dev->usb_dev);
	kfree(dev);
}

static int i2c_ch341_usb_probe(struct usb_interface *iface,
			       const struct usb_device_id *id)
{
	struct i2c_ch341_usb *dev;
	int retval = -ENOMEM;
	int i;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&iface->dev, "Out of memory\n");
		goto error;
	}

	dev->usb_dev = usb_get_dev(interface_to_usbdev(iface));
	dev->iface = iface;

	for (i = 0; i < iface->cur_altsetting->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *ep;

		ep = &iface->cur_altsetting->endpoint[i].desc;
		if (!usb_endpoint_xfer_bulk(ep))
			continue;

		if (usb_endpoint_dir_in(ep))
			dev->ep_in = ep->bEndpointAddress;
		else
			dev->ep_out = ep->bEndpointAddress;
	}

	if (!dev->ep_in || !dev->ep_out) {
		dev_err(&iface->dev, "bulk endpoints not found\n");
		retval = -ENODEV;
		goto error;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(iface, dev);

	/* setup i2c adapter description */
	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &ch341_i2c_algorithm;
	dev->adapter.algo_data = dev;
	snprintf(dev->adapter.name, sizeof(dev->adapter.name),
		 "i2c-ch341-usb at bus %03d device %03d",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	dev->adapter.dev.parent = &dev->iface->dev;

	/* set ch341 i2c speed before exposing the adapter */
	dev->out_buf[0] = CH341_CMD_I2C_STREAM;
	dev->out_buf[1] = CH341_CMD_I2C_STM_SET | (speed & 0x03);
	dev->out_buf[2] = CH341_CMD_I2C_STM_END;
	retval = ch341_xfer(dev, 3, 0);
	if (retval < 0) {
		dev_err(&iface->dev, "failure setting speed\n");
		retval = -EIO;
		goto error;
	}

	/* and finally attach to i2c layer */
	retval = i2c_add_adapter(&dev->adapter);
	if (retval)
		goto error;

	dev_info(&dev->adapter.dev, "connected i2c-ch341-usb device\n");

	return 0;

error:
	if (dev) {
		usb_set_intfdata(iface, NULL);
		i2c_ch341_usb_free(dev);
	}

	return retval;
}

static void i2c_ch341_usb_disconnect(struct usb_interface *iface)
{
	struct i2c_ch341_usb *dev = usb_get_intfdata(iface);

	if (!dev)
		return;

	i2c_del_adapter(&dev->adapter);
	usb_set_intfdata(iface, NULL);
	i2c_ch341_usb_free(dev);

	dev_dbg(&iface->dev, "disconnected\n");
}

static struct usb_driver i2c_ch341_usb_driver = {
	.name = "i2c-ch341-usb",
	.probe = i2c_ch341_usb_probe,
	.disconnect = i2c_ch341_usb_disconnect,
	.id_table = i2c_ch341_usb_table,
};

module_usb_driver(i2c_ch341_usb_driver);

/* ----- end of usb layer ------------------------------------------------ */

MODULE_AUTHOR("Tse Lun Bien <allanbian@gmail.com>");
MODULE_DESCRIPTION("i2c-ch341-usb driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
