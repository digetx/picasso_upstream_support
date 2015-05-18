/*
 * arch/arm/mach-tegra/include/mach/powergate.h
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#ifndef _MACH_TEGRA_DOWNSTREAM_POWERGATE_H_
#define _MACH_TEGRA_DOWNSTREAM_POWERGATE_H_

#include <soc/tegra/pmc.h>

#define tegra_chip_id	tegra_get_chip_id()

int tegra_powergate_mc_disable(int id);
int tegra_powergate_mc_enable(int id);
int tegra_powergate_mc_flush(int id);
int tegra_powergate_mc_flush_done(int id);

#endif /* _MACH_TEGRA_DOWNSTREAM_POWERGATE_H_ */
