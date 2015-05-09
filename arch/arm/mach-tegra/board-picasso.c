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

#include <linux/delay.h>
#include <linux/gpio/machine.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "iomap.h"

static struct property_entry __initdata bluetooth_rfkill_prop[] = {
	PROPERTY_ENTRY_STRING("name", "bluetooth_rfkill"),
	PROPERTY_ENTRY_STRING("type", "bluetooth"),
	{ },
};

static struct platform_device bluetooth_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= 1,
};

static struct gpiod_lookup_table bluetooth_gpio_lookup = {
	.dev_id = "rfkill_gpio.1",
	.table = {
		GPIO_LOOKUP("tegra-gpio", 160, "reset", 0),
		{ },
	},
};

void __init tegra_picasso_rfkill_init(void)
{
	platform_device_add_properties(&bluetooth_rfkill_device,
				       bluetooth_rfkill_prop);
	gpiod_add_lookup_table(&bluetooth_gpio_lookup);
	platform_device_register(&bluetooth_rfkill_device);
}

static int __init tegra_picasso_wifi_pwr_and_reset(void)
{
	if (!of_machine_is_compatible("acer,picasso"))
		return 0;

	writel(0x00004000, IO_TO_VIRT(0x6000D928));
	writel(0x00004040, IO_TO_VIRT(0x6000D918));
	writel(0x00004040, IO_TO_VIRT(0x6000D908));

	writel(0x00000200, IO_TO_VIRT(0x6000D82C));
	writel(0x00000202, IO_TO_VIRT(0x6000D81C));
	writel(0x00000202, IO_TO_VIRT(0x6000D80C));

	writel(0x00004040, IO_TO_VIRT(0x6000D928));
	mdelay(100);

	writel(0x00000202, IO_TO_VIRT(0x6000D82C));
	mdelay(200);

	return 0;
}
arch_initcall_sync(tegra_picasso_wifi_pwr_and_reset);
