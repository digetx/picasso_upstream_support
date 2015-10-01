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

#include <linux/platform_device.h>
#include <linux/rfkill-gpio.h>

#include "gpio-names.h"

static struct rfkill_gpio_platform_data bluetooth_rfkill_platform_data = {
	.name		= "bluetooth_rfkill",
	.shutdown_gpio	= -1,
	.reset_gpio	= TEGRA_GPIO_PU0,
	.type		= RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device bluetooth_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 1,
	.dev	= {
		.platform_data = &bluetooth_rfkill_platform_data,
	},
};

void __init tegra_picasso_rfkill_init(void)
{
	platform_device_register(&bluetooth_rfkill_device);
}
