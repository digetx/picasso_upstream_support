/*
 * Driver for MT9P111 from Aptina
 *
 * Copyright (C) 2013, Dmitry Osipenko <digetx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>

#include <media/v4l2-device.h>

/*
 * Sensor core registers
 */
#define MT9P111_CHIP_ID				0x0000
#define MT9P111_PLL_DIVIDERS			0x0010
#define MT9P111_PLL_P_DIVIDERS			0x0012
#define MT9P111_PLL_CONTROL			0x0014
#define MT9P111_STANDBY_CONTROL_AND_STATUS	0x0018
#define MT9P111_RESET				0x001A
#define MT9P111_PAD_SLEW_PAD_CONFIG		0x001E
#define MT9P111_VDD_DIS_COUNTER			0x0022
#define MT9P111_PLL_P4_P5_P6_DIVIDERS		0x002A
#define MT9P111_PLL_P7_DIVIDER			0x002C
#define MT9P111_SENSOR_CLOCK_DIVIDER		0x002E
#define MT9P111_RESET_REGISTER			0x301A
#define MT9P111_MIPI_CONTROL			0x3400
#define MT9P111_TXSS_PARAMETERS			0x3CA0
#define MT9P111_TXC_PARAMETERS			0x3CA2
#define MT9P111_MIPI_STATUS			0x3402
#define MT9P111_MCU_ADDRESS			0x098E

/*
 * MCU registers
 */
#define MT9P111_MCU_CONTEXT_A			0x483A
#define MT9P111_CONTEXT_A_ROW_START		0xC83A
#define MT9P111_CONTEXT_A_COLUMN_START		0xC83C
#define MT9P111_CONTEXT_A_ROW_END		0xC83E
#define MT9P111_CONTEXT_A_COLUMN_END		0xC840
#define MT9P111_CONTEXT_A_ROW_SPEED		0xC842
#define MT9P111_CONTEXT_A_SKIP_X_CORE		0xC844
#define MT9P111_CONTEXT_A_SKIP_Y_CORE		0xC846
#define MT9P111_CONTEXT_A_SKIP_X_PIPE		0xC848
#define MT9P111_CONTEXT_A_SKIP_Y_PIPE		0xC84A
#define MT9P111_CONTEXT_A_POWER_MODE		0xC84C
#define MT9P111_CONTEXT_A_BIN_MODE		0xC84E
#define MT9P111_CONTEXT_A_FRAME_LENGTH_LINES	0xC868
#define MT9P111_CONTEXT_A_LINE_LENGTH		0xC86A
#define MT9P111_CONTEXT_A_OUTPUT_SIZE_WIDTH	0xC86C
#define MT9P111_CONTEXT_A_OUTPUT_SIZE_HEIGHT	0xC86E
#define MT9P111_CONTEXT_A_OUTPUT_WIDTH		0xC8AA
#define MT9P111_CONTEXT_A_OUTPUT_HEIGHT		0xC8AC

#define SEQ_CMD					0x8404
#define SEQ_STATE_CFG_1_FD			0x8417
#define SEQ_COMMON_CFG_CONT_TRACK_SPEED		0x843E
#define SEQ_COMMON_CFG_CONT_JUMP_DIV		0x843F

#define MT9P111_MAX_VIDEO_WIDTH			1280
#define MT9P111_MAX_VIDEO_HEIGHT		720

#define MT9P111_MAX_WIDTH			2592
#define MT9P111_MAX_HEIGHT			1944

#define DEFAULT_HW_TIMEOUT			100

enum {
	MODE_VIDEO,
	MODE_PHOTO,
};

static struct mt9p111_mode {
	u16 skip_core_pipe;
	u16 power_mode;
	u16 bin_mode;
	u16 frame_length_lines;
	u16 line_length;
} mt9p111_modes[] = {
	[MODE_VIDEO] = { 0x0303, 0x00F6, 0x0001, 0x0423, 0x03E8, },
	[MODE_PHOTO] = { 0x0101, 0x00F2, 0x0000, 0x07EF, 0x1E48, },
};

struct mt9p111 {
	struct v4l2_subdev subdev;
	struct mt9p111_mode *mode;
	int is_streaming;
	int pwdn_gpio;
	int reset_gpio;
	int in_error_state;
	u16 current_mcu_address;
	u16 out_width;
	u16 out_height;
};

static struct mt9p111 *to_mt9p111(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct mt9p111, subdev);
}

static int mt9p111_read(struct v4l2_subdev *sd, u16 address)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	if (mt9p111->in_error_state)
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
		dev_err(&client->dev, "%s failed to read 0x%04x",
			__func__, address);
		mt9p111->in_error_state = 1;
		return ret;
	}

	memcpy(&ret, buf, 2);
	return swab16(ret);
}

static int mt9p111_write(struct v4l2_subdev *sd, u16 address, u16 data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	if (mt9p111->in_error_state)
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
		mt9p111->in_error_state = 1;
		return ret;
	}

	return 0;
}

static int mt9p111_write_byte(struct v4l2_subdev *sd, u16 address, u8 data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	if (mt9p111->in_error_state)
		return -EIO;

	dev_dbg(&client->dev, "%s address: 0x%04x data: 0x%02x\n",
		__func__, address, data);

	address = swab16(address);

	memcpy(buf, &address, 2);
	buf[2] = data;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed write 0x%04x -> 0x%04x",
			__func__, data, address);
		mt9p111->in_error_state = 1;
		return ret;
	}

	return 0;
}

static int mt9p111_write_mcu(struct v4l2_subdev *sd,
			     u16 address, u16 mcu_address, u16 data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	int ret;

	if (mt9p111->current_mcu_address != mcu_address) {
		ret = mt9p111_write(sd, MT9P111_MCU_ADDRESS, mcu_address);
		if (ret < 0)
			return ret;
		mt9p111->current_mcu_address = mcu_address;
	}

	return mt9p111_write(sd, address, data);
}

static void mt9t111_set_pll_dividers(struct v4l2_subdev *sd, u8 n, u8 m,
				     u8 p1, u8 p2, u8 p3, u8 p4, u8 p5, u8 p6,
				     u8 p7, u8 w)
{
	mt9p111_write(sd, MT9P111_PLL_DIVIDERS, (n << 8) | m);

	mt9p111_write(sd, MT9P111_PLL_P_DIVIDERS,
		      (w << 12) |  (p3 << 8) | (p2 << 4) | p1);

	mt9p111_write(sd, MT9P111_PLL_P4_P5_P6_DIVIDERS,
		      (!!p6 << 14) | (!!p5 << 13) | (!!p4 << 12) |
		      (p6 << 8)    | (p5 << 4)    | p4);

	mt9p111_write(sd, MT9P111_PLL_P7_DIVIDER, (!!p7 << 12) | p7);

	mt9p111_write(sd, MT9P111_PLL_CONTROL,
		      (0x1 << 0) | /* PLL bypass */
		      (0x1 << 2) | /* PLL lock detector mode selection */
		      (0x2 << 4) | /* Pfd tuning control */
		      (0x1 << 13)); /* Enable hysteresis on clock input pin */

	mt9p111_write(sd, MT9P111_PAD_SLEW_PAD_CONFIG,
		      (0x7 << 0) | /* DOUT* pads slew rate */
		      (0x7 << 4) | /* GPIO pads slew rate */
		      (0x7 << 8)); /* PIXCLK pad slew rate */

	/* Control the delay of vdd_dis counter */
	mt9p111_write(sd, MT9P111_VDD_DIS_COUNTER, 0x0030);

	mt9p111_write(sd, MT9P111_SENSOR_CLOCK_DIVIDER, 0x0000);
}

static void mt9p111_powerup_prepare(struct v4l2_subdev *sd)
{
	mt9p111_write(sd, MT9P111_STANDBY_CONTROL_AND_STATUS,
		      (0x1 << 3) | /* Enable Interrupt request */
		      (0x1 << 14)); /* In Standby state */

	msleep(DEFAULT_HW_TIMEOUT);
}

static void mt9p111_powerup(struct v4l2_subdev *sd)
{
	/* leave standby */
	mt9p111_write(sd, MT9P111_STANDBY_CONTROL_AND_STATUS,
		      (0x1 << 3));

	msleep(DEFAULT_HW_TIMEOUT);
}

/* FIXME: row/column calc, align */
static void mt9p111_set_context(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	struct mt9p111_mode *mode = mt9p111->mode;
	int img_width, img_height;
	u16 row_start = 0x10, column_start = 0x1C;
	u16 row_end, column_end;

	img_width = MT9P111_MAX_WIDTH;
	img_height = MT9P111_MAX_WIDTH * mt9p111->out_height;
	img_height = img_height / mt9p111->out_width;

	if (img_height > MT9P111_MAX_HEIGHT) {
		img_height = MT9P111_MAX_HEIGHT;
		img_width = MT9P111_MAX_HEIGHT * mt9p111->out_width;
		img_width = img_width / mt9p111->out_height;
	}

	column_start += round_down((MT9P111_MAX_WIDTH - img_width) / 2, 2);
	row_start += round_down((MT9P111_MAX_HEIGHT - img_height) / 2, 2);
	column_end = column_start + img_width + 7 - mode->bin_mode * 2;
	row_end = row_start + img_height + 7 - mode->bin_mode * 2;

	img_width = round_up((img_width + 1) / (mode->bin_mode + 1), 8);
	img_height = round_up((img_height + 1) / (mode->bin_mode + 1), 8);

	dev_dbg(&client->dev, "\trow_start: %d column_start: %d\n"
			      "\t\t\trow_end: %d column_end: %d\n"
			      "\t\t\timg_width: %d img_height: %d\n",
		row_start, column_start, row_end, column_end,
		img_width, img_height);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_OUTPUT_SIZE_WIDTH,
			  MT9P111_MCU_CONTEXT_A, img_width);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_OUTPUT_SIZE_HEIGHT,
			  MT9P111_MCU_CONTEXT_A, img_height);

	msleep(DEFAULT_HW_TIMEOUT);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_ROW_START,
			  MT9P111_MCU_CONTEXT_A, row_start);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_COLUMN_START,
			  MT9P111_MCU_CONTEXT_A, column_start);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_ROW_END,
			  MT9P111_MCU_CONTEXT_A, row_end);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_COLUMN_END,
			  MT9P111_MCU_CONTEXT_A, column_end);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_SKIP_X_CORE,
			  MT9P111_MCU_CONTEXT_A, mode->skip_core_pipe);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_SKIP_Y_CORE,
			  MT9P111_MCU_CONTEXT_A, mode->skip_core_pipe);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_SKIP_X_PIPE,
			  MT9P111_MCU_CONTEXT_A, mode->skip_core_pipe);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_SKIP_Y_PIPE,
			  MT9P111_MCU_CONTEXT_A, mode->skip_core_pipe);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_POWER_MODE,
			  MT9P111_MCU_CONTEXT_A, mode->power_mode);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_BIN_MODE,
			  MT9P111_MCU_CONTEXT_A, mode->bin_mode);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_FRAME_LENGTH_LINES,
			  MT9P111_MCU_CONTEXT_A, mode->frame_length_lines);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_LINE_LENGTH,
			  MT9P111_MCU_CONTEXT_A, mode->line_length);

	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_OUTPUT_WIDTH,
			  MT9P111_MCU_CONTEXT_A, mt9p111->out_width);
	mt9p111_write_mcu(sd, MT9P111_CONTEXT_A_OUTPUT_HEIGHT,
			  MT9P111_MCU_CONTEXT_A, mt9p111->out_height);
}

static void mt9p111_set_mipi_mode(struct v4l2_subdev *sd)
{
	mt9p111_write(sd, MT9P111_RESET_REGISTER,
		      (0x1 << 12) | /* Disables SMIA high-speed serialiser and
				       differential output buffers */
		      (0x1 << 6) | /* reset_register_drive_pins */
		      (0x1 << 5) | /* Undocumented */
		      (0x1 << 4) | /* Transition to standby is synchronized to
				      the end of a frame */
		      (0x1 << 3) | /* Enable write for SMIA registers */
		      (0x1 << 2)); /* Place sensor in streaming mode */

	mt9p111_write(sd, MT9P111_RESET,
		      (0x1 << 4) | /* GPIO pad input pd */
		      (0x1 << 3) | /* VGPIO pad input pd */
		      (0x1 << 2)); /* mipi transmitter enable */

	mt9p111_write(sd, MT9P111_TXSS_PARAMETERS,
		      (0x1 << 0)); /* MIPI/CCP Output */

	mt9p111_write(sd, MT9P111_TXC_PARAMETERS,
		      (0x1 << 7) | /* txc_mipi_enable_line_byte_cnt */
		      (0x1 << 1) | /* txc_po_enable_clk_betwn_lines */
		      (0x1 << 0)); /* txc_po_enable_clk_betwn_frames */

	mt9p111_write(sd, MT9P111_MIPI_STATUS,
		      (0x1 << 4) | /* MIPI idle */
		      (0x1 << 0)); /* MIPI in standby */

	mt9p111_write(sd, MT9P111_MIPI_CONTROL,
		      (0x1e << 10) | /* Data Format: YUV422 8-bit */
		      (0x1 << 9) | /* Enable MIPI Transmit */
		      (0x1 << 5)); /* Enable MIPI packing logic */
}

static void mt9p111_start_stream(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	int max_tries = 20;
	int data;

	mt9p111_write(sd, SEQ_CMD, 0x0600);
	mt9p111_write_byte(sd, SEQ_COMMON_CFG_CONT_TRACK_SPEED, 0x20);
	mt9p111_write_byte(sd, SEQ_COMMON_CFG_CONT_JUMP_DIV, 0x01);

	do {
		data = mt9p111_read(sd, 0x8405);
		if (IS_ERR_VALUE(data))
			break;
		if (data == 0x0300) {
			mt9p111->is_streaming = 1;
			return;
		}

		msleep(DEFAULT_HW_TIMEOUT);

		dev_dbg(&client->dev, "%s polling %d...\n",
			__func__, max_tries);
	} while (max_tries--);

	dev_err(&client->dev, "%s timeout\n", __func__);

	mt9p111->in_error_state = 1;
}

static int mt9p111_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);

	dev_dbg(&client->dev, "%s %s\n", __func__, enable ? "on" : "off");

	if (!enable || mt9p111->is_streaming)
		return 0;

	mt9t111_set_pll_dividers(sd, 3, 64, 0, 7, 0, 12, 7, 15, 0, 0);
	mt9p111_powerup_prepare(sd);
	mt9p111_set_context(sd);
	mt9p111_powerup(sd);
	mt9p111_set_mipi_mode(sd);
	mt9p111_start_stream(sd);

	if (mt9p111->in_error_state) {
		dev_err(&client->dev, "capture start failed\n");
		return -EIO;
	}

	return 0;
}

static u32 mt9p111_codes[] = {
	MEDIA_BUS_FMT_UYVY8_2X8,
};

static int mt9p111_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(mt9p111_codes))
		return -EINVAL;

	code->code = mt9p111_codes[code->index];

	return 0;
}

static int mt9p111_check_res(unsigned width, unsigned height)
{
	if (width > MT9P111_MAX_WIDTH || height > MT9P111_MAX_HEIGHT)
		return -EINVAL;

	return 0;
}

static int mt9p111_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);
	int ret;

	dev_dbg(&client->dev, "%s mf->width: %d mf->height: %d\n",
		__func__, mf->width, mf->height);

	if (format->pad)
		return -EINVAL;

	ret = mt9p111_check_res(mf->width, mf->height);
	if (ret)
		return ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *mf;
		return 0;
	}

	if (mf->width > MT9P111_MAX_VIDEO_WIDTH ||
			mf->height > MT9P111_MAX_VIDEO_HEIGHT)
		mt9p111->mode = &mt9p111_modes[MODE_PHOTO];
	else
		mt9p111->mode = &mt9p111_modes[MODE_VIDEO];

	mt9p111->out_width = mf->width;
	mt9p111->out_height = mf->height;

	return 0;
}

static struct v4l2_subdev_video_ops mt9p111_video_ops = {
	.s_stream	= mt9p111_s_stream,
};

static int mt9p111_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p111 *mt9p111 = to_mt9p111(client);

	dev_dbg(&client->dev, "%s %s\n", __func__, on ? "on" : "off");

	if (on) {
		gpio_set_value(mt9p111->pwdn_gpio, 0);
		mdelay(1);
		gpio_set_value(mt9p111->reset_gpio, 1);
		mdelay(1);
	} else {
		gpio_set_value(mt9p111->pwdn_gpio, 1);
		gpio_set_value(mt9p111->reset_gpio, 0);

		mt9p111->is_streaming = 0;
		mt9p111->in_error_state = 0;
		mt9p111->current_mcu_address = 0;
		mt9p111->out_width = -1;
		mt9p111->out_height = -1;
	}

	return 0;
}

static struct v4l2_subdev_core_ops mt9p111_core_ops = {
	.s_power	= mt9p111_s_power,
};

static const struct v4l2_subdev_pad_ops mt9p111_subdev_pad_ops = {
	.enum_mbus_code = mt9p111_enum_mbus_code,
	.set_fmt	= mt9p111_set_fmt,
};

static struct v4l2_subdev_ops mt9p111_subdev_ops = {
	.core	= &mt9p111_core_ops,
	.video	= &mt9p111_video_ops,
	.pad	= &mt9p111_subdev_pad_ops,
};

static int mt9p111_registered(struct v4l2_subdev *sd)
{
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	struct mt9p111 *mt9p111 = to_mt9p111(client);
// 	int data;
	int ret = 0;

// 	mt9p111_s_power(&mt9p111->subdev, 1);
//
// 	/* Read out the chip version register */
// 	data = mt9p111_read(sd, MT9P111_CHIP_ID);
// 	if (data != 0x2880) {
// 		dev_err(&client->dev,
// 			"MT9P111 not detected, wrong version 0x%04x\n", data);
// 		ret = -ENODEV;
// 	} else
// 		dev_info(&client->dev, "MT9P111 detected at address 0x%02x\n",
// 			 client->addr);
//
// 	mt9p111_s_power(&mt9p111->subdev, 0);

	return ret;
}

static const struct v4l2_subdev_internal_ops mt9p111_subdev_internal_ops = {
	.registered = mt9p111_registered,
};

static int mt9p111_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct mt9p111 *mt9p111;
	enum of_gpio_flags flags;
	int ret;

	mt9p111 = devm_kzalloc(&client->dev, sizeof(*mt9p111), GFP_KERNEL);
	if (!mt9p111)
		return -ENOMEM;

	mt9p111->pwdn_gpio = of_get_named_gpio_flags(np, "pwdn-gpio",
						     0, &flags);

	ret = devm_gpio_request_one(&client->dev, mt9p111->pwdn_gpio,
				    flags | GPIOF_OUT_INIT_HIGH,
				    "mt9p111_pwdn");
	if (ret) {
		dev_err(&client->dev, "cannot get pwdn gpio\n");
		return ret;
	}

	mt9p111->reset_gpio = of_get_named_gpio_flags(np, "rst-gpio",
						      0, &flags);

	ret = devm_gpio_request_one(&client->dev, mt9p111->reset_gpio,
				    flags | GPIOF_OUT_INIT_LOW,
				    "mt9p111_rst");
	if (ret) {
		dev_err(&client->dev, "cannot get reset gpio\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&mt9p111->subdev, client, &mt9p111_subdev_ops);
	mt9p111->subdev.internal_ops = &mt9p111_subdev_internal_ops;

	return v4l2_async_register_subdev(&mt9p111->subdev);
}

static int mt9p111_remove(struct i2c_client *client)
{
	struct mt9p111 *mt9p111 = to_mt9p111(client);

	v4l2_device_unregister_subdev(&mt9p111->subdev);

	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "mt9p111", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct of_device_id mt9p111_match[] = {
	{ .compatible = "aptina,mt9p111", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt9p111_match);

static struct i2c_driver mt9p111_i2c_driver = {
	.driver = {
		.name = "mt9p111",
		.of_match_table = mt9p111_match,
	},
	.probe = mt9p111_probe,
	.remove = mt9p111_remove,
	.id_table = sensor_id,
};

module_i2c_driver(mt9p111_i2c_driver);

MODULE_DESCRIPTION("Aptina MT9P111 Camera driver");
MODULE_AUTHOR("Dmitry Osipenko <digetx@gmail.com>");
MODULE_LICENSE("GPL v2");
