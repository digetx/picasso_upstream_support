/*
 * Driver for MT9D115 from Aptina
 *
 * Copyright (C) 2013, Dmitry Osipenko <digetx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>

#include <media/v4l2-device.h>

#define MT9D115_CHIP_ID				0x0000
#define MT9D115_PLL_DIVIDERS			0x0010
#define MT9D115_PLL_P_DIVIDERS			0x0012
#define MT9D115_PLL_CONTROL			0x0014
#define MT9D115_CTRL_STATUS			0x0018
#define MT9D115_RST_CTRL_REG			0x001A
#define MT9D115_MCU_ADDRESS			0x098C
#define MT9D115_MCU_DATA_0			0x0990

#define MT9D115_CONTEXT_A_OUTPUT_WIDTH		0x2703
#define MT9D115_CONTEXT_A_OUTPUT_HEIGHT		0x2705
#define MT9D115_CONTEXT_A_ROW_START		0x270D
#define MT9D115_CONTEXT_A_COLUMN_START		0x270F
#define MT9D115_CONTEXT_A_ROW_END		0x2711
#define MT9D115_CONTEXT_A_COLUMN_END		0x2713
#define MT9D115_CONTEXT_A_ROW_SPEED		0x2715
#define MT9D115_CONTEXT_A_READ_MODE		0x2717
#define MT9D115_CONTEXT_A_FRAME_LINES		0x271F
#define MT9D115_CONTEXT_A_LINE_LENGTH		0x2721
#define MT9D115_CONTEXT_A_CROP_X_START		0x2739
#define MT9D115_CONTEXT_A_CROP_X_END		0x273B
#define MT9D115_CONTEXT_A_CROP_Y_START		0x273D
#define MT9D115_CONTEXT_A_CROP_Y_END		0x273F

#define MT9D115_MAX_WIDTH			1600
#define MT9D115_MAX_HEIGHT			1200

#define DEFAULT_HW_TIMEOUT			10

#define MCU(_mcu)	MT9D115_MCU_DATA_ ## _mcu

struct mt9d115 {
	struct v4l2_subdev subdev;
	int is_streaming;
	int pwdn_gpio;
	int reset_gpio;
	int in_error_state;
	u16 current_mcu_address;
	u16 out_width;
	u16 out_height;
};

static struct mt9d115 *to_mt9d115(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct mt9d115, subdev);
}

static int mt9d115_read(struct v4l2_subdev *sd, u16 address)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	if (mt9d115->in_error_state)
		return -EIO;

	address = swab16(address);

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *)&address;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 2;
	msg[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed read 0x%04x",
			__func__, address);
		mt9d115->in_error_state = 1;
		return ret;
	}

	memcpy(&ret, buf, 2);
	return swab16(ret);
}

static int mt9d115_write(struct v4l2_subdev *sd, u16 address, u16 data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	if (mt9d115->in_error_state)
		return -EIO;

	dev_dbg(&client->dev, "%s address: 0x%04x data: 0x%04x\n",
		__func__, address, data);

	address = swab16(address);
	data = swab16(data);

	memcpy(buf + 0, &address, 2);
	memcpy(buf + 2, &data,    2);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 4;
	msg.buf   = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed write 0x%04x -> 0x%04x",
			__func__, data, address);
		mt9d115->in_error_state = 1;
		return ret;
	}

	return 0;
}

static int mt9d115_write_mcu(struct v4l2_subdev *sd,
			     u16 mcu_address, u16 address, u16 data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	int ret;

	if (mt9d115->current_mcu_address != mcu_address) {
		ret = mt9d115_write(sd, MT9D115_MCU_ADDRESS, mcu_address);
		if (ret < 0)
			return ret;
		mt9d115->current_mcu_address = mcu_address;
	}

	return mt9d115_write(sd, address, data);
}

static int mt9d115_read_mcu(struct v4l2_subdev *sd,
			     u16 mcu_address, u16 address)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	int ret;

	if (mt9d115->current_mcu_address != mcu_address) {
		ret = mt9d115_write(sd, MT9D115_MCU_ADDRESS, mcu_address);
		if (ret < 0)
			return ret;
		mt9d115->current_mcu_address = mcu_address;
	}

	return mt9d115_read(sd, address);
}

static void mt9d115_setup_pll(struct v4l2_subdev *sd)
{
	mt9d115_write(sd, MT9D115_PLL_CONTROL, 0x21F9);
	mt9d115_write(sd, MT9D115_PLL_DIVIDERS, 0x0115);
	mt9d115_write(sd, MT9D115_PLL_P_DIVIDERS, 0x00F5);
	mt9d115_write(sd, MT9D115_PLL_CONTROL, 0x2545);
	mt9d115_write(sd, MT9D115_PLL_CONTROL, 0x2547);
	mt9d115_write(sd, MT9D115_PLL_CONTROL, 0x2447);
	msleep(DEFAULT_HW_TIMEOUT);
	mt9d115_write(sd, MT9D115_PLL_CONTROL, 0x2047);
	mt9d115_write(sd, MT9D115_PLL_CONTROL, 0x2046);
}

static void mt9d115_powerup_prepare(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	int max_tries = 10;
	int data;

	data = mt9d115_read(sd, MT9D115_CTRL_STATUS);
	mt9d115_write(sd, MT9D115_CTRL_STATUS, data | 0x0004);

	data = mt9d115_read(sd, MT9D115_CTRL_STATUS);
	mt9d115_write(sd, MT9D115_CTRL_STATUS, data & 0xFFFE);

	do {
		data = mt9d115_read(sd, MT9D115_CTRL_STATUS);
		if (IS_ERR_VALUE(data))
			return;
		if (!(data & 0x4000))
			return;

		msleep(DEFAULT_HW_TIMEOUT);

		dev_dbg(&client->dev, "%s polling %d...\n",
			__func__, max_tries);
	} while (max_tries--);

	dev_err(&client->dev, "%s timeout\n", __func__);

	mt9d115->in_error_state = 1;
}

static void mt9d115_setup_context(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	u16 w = mt9d115->out_width;
	u16 h = mt9d115->out_height;
	u16 crop_x0, crop_x1;
	u16 crop_y0, crop_y1;

	crop_x0 = (MT9D115_MAX_WIDTH - w) / 2;
	crop_x1 = crop_x0 + w - 1;
	crop_y0 = (MT9D115_MAX_HEIGHT - h) / 2;
	crop_y1 = crop_y0 + h - 1;

	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_OUTPUT_WIDTH, MCU(0), w);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_OUTPUT_HEIGHT, MCU(0), h);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_ROW_START, MCU(0), 0x0000);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_COLUMN_START, MCU(0), 0x0000);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_ROW_END, MCU(0), 0x04BB);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_COLUMN_END, MCU(0), 0x064B);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_ROW_SPEED, MCU(0), 0x0111);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_READ_MODE, MCU(0), 0x0024);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_FRAME_LINES, MCU(0), 0x0521);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_LINE_LENGTH, MCU(0), 0x08EC);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_CROP_X_START, MCU(0), crop_x0);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_CROP_X_END, MCU(0), crop_x1);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_CROP_Y_START, MCU(0), crop_y0);
	mt9d115_write_mcu(sd, MT9D115_CONTEXT_A_CROP_Y_END, MCU(0), crop_y1);
}

static void mt9d115_start_stream(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	int max_tries = 10;
	int data;

	data = mt9d115_read(sd, MT9D115_CTRL_STATUS);
	mt9d115_write(sd, MT9D115_CTRL_STATUS, data & 0xFFFB);

	do {
		msleep(DEFAULT_HW_TIMEOUT);

		data = mt9d115_read_mcu(sd, 0xA104, MCU(0));
		if (IS_ERR_VALUE(data))
			break;
		if (data == 0x0003) {
			mt9d115_write_mcu(sd, 0xA103, MCU(0), 0x0006);
			return;
		}

		dev_dbg(&client->dev, "%s polling %d...\n",
			__func__, max_tries);
	} while (max_tries--);

	dev_err(&client->dev, "%s timeout\n", __func__);

	mt9d115->in_error_state = 1;
}

static void mt9d115_set_output_to_mipi(struct v4l2_subdev *sd)
{
	u16 data;

	data = mt9d115_read(sd, MT9D115_RST_CTRL_REG) & 0xFDFF;
	mt9d115_write(sd, MT9D115_RST_CTRL_REG, data | 0x0008);
}

static int mt9d115_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);

	dev_dbg(&client->dev, "%s %s\n", __func__, enable ? "on" : "off");

	if (!enable || mt9d115->is_streaming)
		return 0;

	mt9d115_set_output_to_mipi(sd);
	mt9d115_setup_pll(sd);
	mt9d115_powerup_prepare(sd);
	mt9d115_setup_context(sd);
	mt9d115_start_stream(sd);

	if (mt9d115->in_error_state) {
		dev_err(&client->dev, "capture start failed\n");
		return -EIO;
	}

	return 0;
}

static u32 mt9d115_codes[] = {
	MEDIA_BUS_FMT_UYVY8_2X8,
};

static int mt9d115_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(mt9d115_codes))
		return -EINVAL;

	code->code = mt9d115_codes[code->index];

	return 0;
}

static int mt9d115_check_res(unsigned width, unsigned height)
{
	if (width > MT9D115_MAX_WIDTH || height > MT9D115_MAX_HEIGHT)
		return -EINVAL;

	return 0;
}

static int mt9d115_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);
	int ret;

	dev_dbg(&client->dev, "%s mf->width: %d mf->height: %d\n",
		__func__, mf->width, mf->height);

	if (format->pad)
		return -EINVAL;

	ret = mt9d115_check_res(ALIGN(mf->width, 4), mf->height);
	if (ret)
		return ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *mf;
		return 0;
	}

	mf->width = ALIGN(mf->width, 4);

	dev_dbg(&client->dev, "%s mf->width: %d mf->height: %d\n",
		__func__, mf->width, mf->height);

	mt9d115->out_width = mf->width;
	mt9d115->out_height = mf->height;

	return 0;
}

static struct v4l2_subdev_video_ops mt9d115_video_ops = {
	.s_stream	= mt9d115_s_stream,
};

static int mt9d115_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9d115 *mt9d115 = to_mt9d115(client);

	if (on) {
		gpio_set_value(mt9d115->pwdn_gpio, 0);
		mdelay(1);
		gpio_set_value(mt9d115->reset_gpio, 1);
		mdelay(1);
	} else {
		gpio_set_value(mt9d115->pwdn_gpio, 1);
		gpio_set_value(mt9d115->reset_gpio, 0);

		mt9d115->is_streaming = 0;
		mt9d115->in_error_state = 0;
		mt9d115->current_mcu_address = 0;
		mt9d115->out_width = -1;
		mt9d115->out_height = -1;
	}

	return 0;
}

static struct v4l2_subdev_core_ops mt9d115_core_ops = {
	.s_power	= mt9d115_s_power,
};

static const struct v4l2_subdev_pad_ops mt9d115_subdev_pad_ops = {
	.enum_mbus_code = mt9d115_enum_mbus_code,
	.set_fmt	= mt9d115_set_fmt,
};

static struct v4l2_subdev_ops mt9d115_subdev_ops = {
	.core	= &mt9d115_core_ops,
	.video	= &mt9d115_video_ops,
	.pad	= &mt9d115_subdev_pad_ops,
};

static int mt9d115_registered(struct v4l2_subdev *sd)
{
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	int data;
	int ret = 0;

// 	mt9d115_s_power(sd, 1);
//
// 	/* Read out the chip version register */
// 	data = mt9d115_read(sd, MT9D115_CHIP_ID);
// 	if (data != 0x2580) {
// 		dev_err(&client->dev,
// 			"MT9D115 not detected, wrong version 0x%04x\n", data);
// 		ret = -ENODEV;
// 	} else
// 		dev_info(&client->dev, "MT9D115 detected at address 0x%02x\n",
// 			 client->addr);
//
// 	mt9d115_s_power(sd, 0);

	return ret;
}

static const struct v4l2_subdev_internal_ops mt9d115_subdev_internal_ops = {
	.registered = mt9d115_registered,
};

static int mt9d115_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct mt9d115 *mt9d115;
	enum of_gpio_flags flags;
	int ret;

	mt9d115 = devm_kzalloc(&client->dev, sizeof(*mt9d115), GFP_KERNEL);
	if (!mt9d115)
		return -ENOMEM;

	mt9d115->pwdn_gpio = of_get_named_gpio_flags(np, "pwdn-gpio",
						     0, &flags);

	ret = devm_gpio_request_one(&client->dev, mt9d115->pwdn_gpio,
				    flags | GPIOF_OUT_INIT_HIGH,
				    "mt9d115_pwdn");
	if (ret) {
		dev_err(&client->dev, "cannot get pwdn gpio\n");
		return ret;
	}

	mt9d115->reset_gpio = of_get_named_gpio_flags(np, "rst-gpio",
						      0, &flags);

	ret = devm_gpio_request_one(&client->dev, mt9d115->reset_gpio,
				    flags | GPIOF_OUT_INIT_LOW,
				    "mt9d115_rst");
	if (ret) {
		dev_err(&client->dev, "cannot get reset gpio\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&mt9d115->subdev, client, &mt9d115_subdev_ops);
	mt9d115->subdev.internal_ops = &mt9d115_subdev_internal_ops;

	return v4l2_async_register_subdev(&mt9d115->subdev);
}

static int mt9d115_remove(struct i2c_client *client)
{
	struct mt9d115 *mt9d115 = to_mt9d115(client);

	v4l2_device_unregister_subdev(&mt9d115->subdev);

	return 0;
}

static const struct i2c_device_id mt9d115_id[] = {
	{ "mt9d115", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9d115_id);

static struct of_device_id mt9d115_match[] = {
	{ .compatible = "aptina,mt9d115", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt9d115_match);

static struct i2c_driver mt9d115_i2c_driver = {
	.driver = {
		.name = "mt9d115",
		.of_match_table = mt9d115_match,
	},
	.probe    = mt9d115_probe,
	.remove   = mt9d115_remove,
	.id_table = mt9d115_id,
};

module_i2c_driver(mt9d115_i2c_driver);

MODULE_DESCRIPTION("Aptina MT9D115 Camera driver");
MODULE_AUTHOR("Dmitry Osipenko <digetx@gmail.com>");
MODULE_LICENSE("GPL v2");
