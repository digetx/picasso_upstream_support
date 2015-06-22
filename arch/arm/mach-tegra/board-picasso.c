/*
 * arch/arm/mach-tegra/board-picasso.c
 *
 * Copyright (C) 2015 Dmitry Osipenko <digetx@gmail.com>
 *
 * Based on board-paz00.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio/machine.h>
#include <linux/platform_device.h>
#include <linux/rfkill-gpio.h>

static struct rfkill_gpio_platform_data wifi_rfkill_platform_data = {
	.name	= "wifi_rfkill",
	.type	= RFKILL_TYPE_WLAN,
};

static struct platform_device wifi_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 0,
	.dev	= {
		.platform_data = &wifi_rfkill_platform_data,
	},
};

static struct gpiod_lookup_table wifi_gpio_lookup = {
	.dev_id = "rfkill_gpio.0",
	.table = {
		GPIO_LOOKUP_IDX("tegra-gpio", 86, NULL, 0, 0),
		{ },
	},
};

static struct rfkill_gpio_platform_data bluetooth_rfkill_platform_data = {
	.name	= "bluetooth_rfkill",
	.type	= RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device bluetooth_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 1,
	.dev	= {
		.platform_data = &bluetooth_rfkill_platform_data,
	},
};

static struct gpiod_lookup_table bluetooth_gpio_lookup = {
	.dev_id = "rfkill_gpio.1",
	.table = {
		GPIO_LOOKUP_IDX("tegra-gpio", 160, NULL, 0, 0),
		{ },
	},
};

void __init tegra_picasso_rfkill_init(void)
{
	gpiod_add_lookup_table(&wifi_gpio_lookup);
	gpiod_add_lookup_table(&bluetooth_gpio_lookup);

	platform_device_register(&wifi_rfkill_device);
	platform_device_register(&bluetooth_rfkill_device);
}
