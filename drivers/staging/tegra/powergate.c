/*
 * drivers/powergate/tegra-powergate.c
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

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <mach/fuse.h>
#include <mach/powergate.h>

#include <../arch/arm/mach-tegra/iomap.h>

#define TEGRA_MC_BASE		0x7000F000

#define MC_CLIENT_CTRL		0x100
#define  MC_CLIENT_HOTRESETN	0x104
#define  MC_CLIENT_ORRC_BASE	0x140

static DEFINE_SPINLOCK(tegra_powergate_lock);

enum mc_client {
	MC_CLIENT_AVPC		= 0,
	MC_CLIENT_DC		= 1,
	MC_CLIENT_DCB		= 2,
	MC_CLIENT_EPP		= 3,
	MC_CLIENT_G2		= 4,
	MC_CLIENT_HC		= 5,
	MC_CLIENT_ISP		= 6,
	MC_CLIENT_MPCORE	= 7,
	MC_CLIENT_MPEA		= 8,
	MC_CLIENT_MPEB		= 9,
	MC_CLIENT_MPEC		= 10,
	MC_CLIENT_NV		= 11,
	MC_CLIENT_PPCS		= 12,
	MC_CLIENT_VDE		= 13,
	MC_CLIENT_VI		= 14,
	MC_CLIENT_LAST		= -1,
	MC_CLIENT_AFI		= MC_CLIENT_LAST,
};

#define MAX_HOTRESET_CLIENT_NUM		4
#define MAX_CLK_EN_NUM			4
struct powergate_partition {
	enum mc_client hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
};

static struct powergate_partition powergate_partition_info[] = {
	[TEGRA_POWERGATE_CPU]	= { {MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_L2]	= { {MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_3D]	= { {MC_CLIENT_NV, MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_PCIE]	= { {MC_CLIENT_AFI, MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_VDEC]	= { {MC_CLIENT_VDE, MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_MPE]	= { {MC_CLIENT_MPEA, MC_CLIENT_MPEB,
				     MC_CLIENT_MPEC, MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_VENC]	= { {MC_CLIENT_ISP, MC_CLIENT_VI, MC_CLIENT_LAST}, },
};

static void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

static u32 mc_read(unsigned long reg)
{
	return readl(mc + reg);
}

static void mc_write(u32 val, unsigned long reg)
{
	writel(val, mc + reg);
}

int tegra_powergate_mc_disable(int id)
{
	u32 idx, clt_ctrl, orrc_reg;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= ARRAY_SIZE(powergate_partition_info))
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* clear client enable bit */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl &= ~(1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);

		/* wait for outstanding requests to reach 0 */
		orrc_reg = MC_CLIENT_ORRC_BASE + (mcClientBit * 4);
		while (mc_read(orrc_reg) != 0)
			udelay(10);
	}

	return 0;
}

int tegra_powergate_mc_flush(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= ARRAY_SIZE(powergate_partition_info))
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* assert hotreset (client module is currently in reset) */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn &= ~(1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	return 0;
}

int tegra_powergate_mc_flush_done(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= ARRAY_SIZE(powergate_partition_info))
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* deassert hotreset */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn |= (1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	return 0;
}

int tegra_powergate_mc_enable(int id)
{
	u32 idx, clt_ctrl;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= ARRAY_SIZE(powergate_partition_info))
		return -EINVAL;

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* enable client */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl |= (1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	return 0;
}
