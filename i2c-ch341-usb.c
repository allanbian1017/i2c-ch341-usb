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

#define DRV_VERSION "1.1"

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

/* Structure to hold all of our device specific stuff */
struct i2c_ch341_usb {
	struct usb_device *usb_dev;  /* the usb device for this device */
	struct usb_interface *iface; /* the interface for this device */
	struct i2c_adapter adapter;  /* i2c related things */

	int ep_in;
	int ep_out;

	u8 in_buf[32];
	u8 out_buf[32];
};

static int ch341_xfer(struct i2c_ch341_usb *dev, int out_len, int in_len);

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

	retval = ch341_xfer(dev, 6, 1);
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
	int retval;
	int i;
	int l;

	dev_dbg(&adapter->dev, "master xfer %d messages\n", num);

	retval = ch341_i2c_check_dev(dev, msgs[0].addr);
	if (retval < 0)
		return retval;

	if (num == 1) {
		/* size larger than endpoint max transfer size */
		if ((msgs[0].len + 5) > 32)
			return -EIO;

		if (msgs[0].flags & I2C_M_RD) {
			dev->out_buf[0] = CH341_CMD_I2C_STREAM;
			dev->out_buf[1] = CH341_CMD_I2C_STM_STA;
			dev->out_buf[2] = CH341_CMD_I2C_STM_OUT | 0x1;
			dev->out_buf[3] = (msgs[0].addr << 1) | 0x1;

			if (msgs[0].len) {
				for (i = 0, l = msgs[0].len; l > 1; l--, i++)
					dev->out_buf[i + 4] =
						CH341_CMD_I2C_STM_IN | 1;
				dev->out_buf[msgs[0].len + 3] =
					CH341_CMD_I2C_STM_IN;
			}

			dev->out_buf[msgs[0].len + 4] = CH341_CMD_I2C_STM_STO;
			dev->out_buf[msgs[0].len + 5] = CH341_CMD_I2C_STM_END;

			retval = ch341_xfer(dev, msgs[0].len + 5, msgs[0].len);
			if (retval < 0)
				return retval;

			memcpy(msgs[0].buf, dev->in_buf, msgs[0].len);
		} else {
			dev->out_buf[0] = CH341_CMD_I2C_STREAM;
			dev->out_buf[1] = CH341_CMD_I2C_STM_STA;
			dev->out_buf[2] =
				CH341_CMD_I2C_STM_OUT | (msgs[0].len + 1);
			dev->out_buf[3] = msgs[0].addr << 1;

			memcpy(&dev->out_buf[4], msgs[0].buf, msgs[0].len);

			dev->out_buf[msgs[0].len + 4] = CH341_CMD_I2C_STM_STO;
			dev->out_buf[msgs[0].len + 5] = CH341_CMD_I2C_STM_END;

			retval = ch341_xfer(dev, msgs[0].len + 5, 0);
			if (retval < 0)
				return retval;
		}
	} else if (num == 2) {
		if (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD)) {
			/* size larger than endpoint max transfer size */
			if (((msgs[0].len + 3) > 32)
			    || ((msgs[1].len + 5) > 32))
				return -EIO;

			/* write data phase */
			dev->out_buf[0] = CH341_CMD_I2C_STREAM;
			dev->out_buf[1] = CH341_CMD_I2C_STM_STA;
			dev->out_buf[2] =
				CH341_CMD_I2C_STM_OUT | (msgs[0].len + 1);
			dev->out_buf[3] = msgs[0].addr << 1;

			memcpy(&dev->out_buf[4], msgs[0].buf, msgs[0].len);

			retval = ch341_xfer(dev, msgs[0].len + 4, 0);
			if (retval < 0)
				return retval;

			/* read data phase */
			dev->out_buf[0] = CH341_CMD_I2C_STREAM;
			dev->out_buf[1] = CH341_CMD_I2C_STM_STA;
			dev->out_buf[2] = CH341_CMD_I2C_STM_OUT | 0x1;
			dev->out_buf[3] = (msgs[1].addr << 1) | 0x1;

			if (msgs[1].len) {
				for (i = 0, l = msgs[1].len; l > 1; l--, i++)
					dev->out_buf[i + 4] =
						CH341_CMD_I2C_STM_IN | 1;
				dev->out_buf[msgs[1].len + 3] =
					CH341_CMD_I2C_STM_IN;
			}

			dev->out_buf[msgs[1].len + 4] = CH341_CMD_I2C_STM_STO;
			dev->out_buf[msgs[1].len + 5] = CH341_CMD_I2C_STM_END;

			retval = ch341_xfer(dev, msgs[1].len + 5, msgs[1].len);
			if (retval < 0)
				return retval;

			memcpy(msgs[1].buf, dev->in_buf, msgs[1].len);
		} else {
			return -EIO;
		}
	} else {
		dev_err(&adapter->dev,
			"This case(num > 2) has not been support now\n");
		return -EIO;
	}

	return num;
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

static int ch341_xfer(struct i2c_ch341_usb *dev, int out_len, int in_len)
{
	int retval;
	int actual;

	dev_dbg(&dev->adapter.dev, "bulk_out %d bytes, bulk_in %d bytes\n",
		out_len, (in_len == 0) ? 0 : 32);

	retval = usb_bulk_msg(dev->usb_dev,
			      usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			      dev->out_buf, out_len, &actual, 2000);
	if (retval < 0)
		return retval;

	if (in_len == 0)
		return actual;

	memset(dev->in_buf, 0, sizeof(dev->in_buf));
	retval = usb_bulk_msg(dev->usb_dev,
			      usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			      dev->in_buf, 32, &actual, 2000);
	if (retval < 0)
		return retval;

	return actual;
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

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&iface->dev, "Out of memory\n");
		goto error;
	}

	dev->usb_dev = usb_get_dev(interface_to_usbdev(iface));
	dev->iface = iface;

	dev->ep_out = iface->cur_altsetting->endpoint[1].desc.bEndpointAddress;
	dev->ep_in = iface->cur_altsetting->endpoint[0].desc.bEndpointAddress;

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

	/* and finally attach to i2c layer */
	i2c_add_adapter(&dev->adapter);

	/* set ch341 i2c speed */
	dev->out_buf[0] = CH341_CMD_I2C_STREAM;
	dev->out_buf[1] = CH341_CMD_I2C_STM_SET | CH341_I2C_STANDARD_SPEED;
	dev->out_buf[2] = CH341_CMD_I2C_STM_END;
	retval = ch341_xfer(dev, 3, 0);
	if (retval < 0) {
		dev_err(&dev->adapter.dev, "failure setting speed\n");
		retval = -EIO;
		goto error;
	}

	dev_info(&dev->adapter.dev, "connected i2c-ch341-usb device\n");

	return 0;

error:
	if (dev)
		i2c_ch341_usb_free(dev);

	return retval;
}

static void i2c_ch341_usb_disconnect(struct usb_interface *iface)
{
	struct i2c_ch341_usb *dev = usb_get_intfdata(iface);

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
