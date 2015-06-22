/*
 * MFD driver for Acer A50x embedded controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/of_device.h>

#include <asm/system_misc.h>

#include "ec.h"

/*				addr	timeout */
EC_REG_DATA(SHUTDOWN,		0x52,	0);
EC_REG_DATA(WARM_REBOOT,	0x54,	0);
EC_REG_DATA(COLD_REBOOT,	0x55,	1000);

static DEFINE_MUTEX(ec_mutex);

static struct ec_info {
	struct i2c_client	*client;
	struct notifier_block	panic_notifier;
	int			i2c_retry_count;
} *ec_chip;

inline void ec_lock(void)
{
	mutex_lock(&ec_mutex);
}
EXPORT_SYMBOL_GPL(ec_lock);

inline void ec_unlock(void)
{
	mutex_unlock(&ec_mutex);
}
EXPORT_SYMBOL_GPL(ec_unlock);

#define I2C_ERR_TIMEOUT	500
int ec_read_word_data_locked(struct ec_reg_data *reg_data)
{
	struct i2c_client *client = ec_chip->client;
	int retries = ec_chip->i2c_retry_count;
	s32 ret = 0;

	while (retries > 0) {
		ret = i2c_smbus_read_word_data(client, reg_data->addr);
		if (ret >= 0)
			break;
		msleep(I2C_ERR_TIMEOUT);
		retries--;
	}

	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c read at address 0x%x failed\n",
			__func__, reg_data->addr);
		return ret;
	}

	msleep(reg_data->timeout);

	return le16_to_cpu(ret);
}
EXPORT_SYMBOL_GPL(ec_read_word_data_locked);

int ec_read_word_data(struct ec_reg_data *reg_data)
{
	s32 ret;

	ec_lock();
	ret = ec_read_word_data_locked(reg_data);
	ec_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(ec_read_word_data);

int ec_write_word_data_locked(struct ec_reg_data *reg_data, u16 value)
{
	struct i2c_client *client = ec_chip->client;
	int retries = ec_chip->i2c_retry_count;
	s32 ret = 0;

	while (retries > 0) {
		ret = i2c_smbus_write_word_data(client, reg_data->addr,
						le16_to_cpu(value));
		if (ret >= 0)
			break;
		msleep(I2C_ERR_TIMEOUT);
		retries--;
	}

	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c write to address 0x%x failed\n",
			__func__, reg_data->addr);
		return ret;
	}

	msleep(reg_data->timeout);

	return 0;
}
EXPORT_SYMBOL_GPL(ec_write_word_data_locked);

int ec_write_word_data(struct ec_reg_data *reg_data, u16 value)
{
	s32 ret;

	ec_lock();
	ret = ec_write_word_data_locked(reg_data, value);
	ec_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(ec_write_word_data);

static void ec_poweroff(void)
{
	dev_info(&ec_chip->client->dev, "poweroff ...\n");

	ec_write_word_data(SHUTDOWN, 0);
}

static void ec_reboot(enum reboot_mode mode, const char *cmd)
{
	if (oops_in_progress)
		ec_write_word_data_locked(WARM_REBOOT, 0);

	dev_info(&ec_chip->client->dev, "reboot ...\n");

	ec_write_word_data(COLD_REBOOT, 1);
}

static struct mfd_cell ec_cell[] = {
	{
		.name = "a50x-battery",
		.of_compatible = "acer,a50x-battery",
		.id = 1,
	},
	{
		.name = "a50x-leds",
		.of_compatible = "acer,a50x-leds",
		.id = 1,
	},
};

static int ec_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	int ret;

	ec_chip = devm_kzalloc(&client->dev, sizeof(*ec_chip), GFP_KERNEL);
	if (!ec_chip)
		return -ENOMEM;

	if (of_property_read_u32(np, "ec,i2c-retry-count",
				 &ec_chip->i2c_retry_count))
		ec_chip->i2c_retry_count = 5;

	ec_chip->client = client;

	/* register battery and leds */
	ret = mfd_add_devices(&client->dev, -1,
			      ec_cell, ARRAY_SIZE(ec_cell),
			      NULL, 0, NULL);
	if (ret) {
		dev_err(&client->dev, "Failed to add subdevices\n");
		return ret;
	}

	/* set pm functions */
	if (of_property_read_bool(np, "system-power-controller")) {
		arm_pm_restart = ec_reboot;

		if (!pm_power_off)
			pm_power_off = ec_poweroff;
	}

	dev_dbg(&client->dev, "device registered\n");

	return 0;
}

static const struct of_device_id ec_match[] = {
	{ .compatible = "acer,a50x-ec" },
	{ }
};
MODULE_DEVICE_TABLE(of, ec_match);

static const struct i2c_device_id ec_id[] = {
	{ "a50x_EC", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ec_id);

static struct i2c_driver a50x_ec_driver = {
	.driver = {
		.name = "a50x-ec",
		.of_match_table = ec_match,
		.owner = THIS_MODULE,
	},
	.id_table = ec_id,
	.probe = ec_probe,
};
module_i2c_driver(a50x_ec_driver);

MODULE_DESCRIPTION("Acer A50x EC MFD driver");
MODULE_AUTHOR("Dmitry Osipenko <digetx@gmail.com>");
MODULE_LICENSE("GPL");
