/*
 * NVIDIA Tegra20 Video decoder driver
 *
 * Copyright (C) 2016 Dmitry Osipenko <digetx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "uapi.h"
#include "vde.h"

#define SXE(offt)		(0x0000 + (offt))
#define BSEV(offt)		(0x1000 + (offt))
#define MBE(offt)		(0x2000 + (offt))
#define PPE(offt)		(0x2200 + (offt))
#define MCE(offt)		(0x2400 + (offt))
#define TFE(offt)		(0x2600 + (offt))
#define VDMA(offt)		(0x2A00 + (offt))
#define FRAMEID(offt)		(0x3800 + (offt))

#define ICMDQUE_WR		0x00
#define CMDQUE_CONTROL		0x08
#define INTR_STATUS		0x18
#define BSE_INT_ENB		0x40
#define BSE_CONFIG		0x44

#define BSE_ICMDQUE_EMPTY	BIT(3)
#define BSE_DMA_BUSY		BIT(23)

#define TEGRA_VDE_TIMEOUT	(msecs_to_jiffies(1000))

#define VDE_WR(data, addr)				\
do {							\
	pr_debug("%s: %d: 0x%08X => " #addr ")\n",	\
		  __func__, __LINE__, (data));		\
	writel_relaxed(data, addr);			\
} while (0)

struct video_frame {
	struct dma_buf_attachment *y_dmabuf_attachment;
	struct dma_buf_attachment *cb_dmabuf_attachment;
	struct dma_buf_attachment *cr_dmabuf_attachment;
	struct dma_buf_attachment *aux_dmabuf_attachment;
	dma_addr_t y_paddr;
	dma_addr_t cb_paddr;
	dma_addr_t cr_paddr;
	dma_addr_t aux_paddr;
	u32 frame_num;
	u32 flags;
};

struct tegra_vde {
	void __iomem *regs;
	void __iomem *iram;
	struct mutex lock;
	struct miscdevice miscdev;
	struct reset_control *rst;
	struct clk *clk;
	struct completion decode_completion;
	phys_addr_t iram_lists_paddr;
	int irq;
	int decoding_in_progress;
};

static int tegra_vde_reset(struct tegra_vde *vde)
{
	struct device *dev = vde->miscdev.parent;
	int ret;

	ret = reset_control_assert(vde->rst);

	if (ret) {
		dev_err(dev, "Failed to assert reset\n");
		return ret;
	}

	ret = clk_prepare_enable(vde->clk);

	if (ret) {
		dev_err(dev, "Failed to enable clk\n");
		return ret;
	}

	udelay(100);

	reset_control_deassert(vde->rst);

	clk_disable_unprepare(vde->clk);

	return 0;
}

static void tegra_vde_set_bits(void __iomem *regs, u32 mask, u32 offset)
{
	u32 value = readl_relaxed(regs + offset);

	VDE_WR(value | mask, regs + offset);
}

static int tegra_vde_wait_mbe(void __iomem *regs)
{
	u32 tmp;

	return readl_poll_timeout(regs + MBE(0x8C), tmp, (tmp >= 0x10), 1, 100);
}

static int tegra_vde_setup_mbe_frame_idx(void __iomem *regs,
					 int setup_refs, int refs_nb)
{
	u32 frame_idx_enb_mask = 0;
	u32 value;
	int frame_idx;
	int idx;

	VDE_WR(0xD0000000 | (0 << 23), regs + MBE(0x80));
	VDE_WR(0xD0200000 | (0 << 23), regs + MBE(0x80));

	if (tegra_vde_wait_mbe(regs))
		return -EIO;

	if (!setup_refs)
		return 0;

	for (idx = 0, frame_idx = 1; idx < refs_nb; idx++, frame_idx++) {
		VDE_WR(0xD0000000 | (frame_idx << 23), regs + MBE(0x80));
		VDE_WR(0xD0200000 | (frame_idx << 23), regs + MBE(0x80));

		frame_idx_enb_mask |= frame_idx << (6 * (idx % 4));

		if (idx % 4 == 3 || idx == refs_nb -1 ) {
			value = 0xC0000000;
			value |= (idx >> 2) << 24;
			value |= frame_idx_enb_mask;

			VDE_WR(value, regs + MBE(0x80));

			if (tegra_vde_wait_mbe(regs))
				return -EIO;

			frame_idx_enb_mask = 0;
		}
	}

	return 0;
}

static void tegra_vde_mbe_set_0xa_reg(void __iomem *regs, int reg, u32 val)
{
	VDE_WR(0xA0000000 | (reg << 24) | (val & 0xFFFF), regs + MBE(0x80));
	VDE_WR(0xA0000000 | ((reg + 1) << 24) | (val >> 16), regs + MBE(0x80));
}

static int tegra_vde_wait_bsev(struct tegra_vde *vde, int wait_dma)
{
	struct device *dev = vde->miscdev.parent;
	u32 polled;
	int ret;

	ret = readl_poll_timeout(vde->regs + BSEV(INTR_STATUS), polled,
				 !(polled & BIT(2)), 1, 100);
	if (ret) {
		dev_err(dev, "BSEV unknown bit timeout\n");
		return -EIO;
	}

	ret = readl_poll_timeout(vde->regs + BSEV(INTR_STATUS), polled,
				 (polled & BSE_ICMDQUE_EMPTY), 1, 100);
	if (ret) {
		dev_err(dev, "BSEV ICMDQUE flush timeout\n");
		return -EIO;
	}

	if (!wait_dma)
		return 0;

	ret = readl_poll_timeout(vde->regs + BSEV(INTR_STATUS), polled,
				 !(polled & BSE_DMA_BUSY), 1, 100);
	if (ret) {
		dev_err(dev, "BSEV DMA timeout\n");
		return -EIO;
	}

	return 0;
}

static int tegra_vde_push_bsev_icmdqueue(struct tegra_vde *vde,
					 u32 value, int wait_dma)
{
	VDE_WR(value, vde->regs + BSEV(ICMDQUE_WR));

	return tegra_vde_wait_bsev(vde, wait_dma);
}

static void tegra_vde_setup_frameid(void __iomem *regs,
				    struct video_frame *frame,
				    int frameid, u32 mbs_width, u32 mbs_height)
{
	u32 y_paddr  = frame ? frame->y_paddr  : 0xFCDEAD00;
	u32 cb_paddr = frame ? frame->cb_paddr : 0xFCDEAD00;
	u32 cr_paddr = frame ? frame->cr_paddr : 0xFCDEAD00;
	u32 v1 = frame ? ((mbs_width << 16) | mbs_height) : 0;
	u32 v2 = frame ? ((((mbs_width + 1) >> 1) << 6) | 1) : 0;

	VDE_WR( y_paddr >> 8, regs + FRAMEID(0x000 + frameid * 4));
	VDE_WR(cb_paddr >> 8, regs + FRAMEID(0x100 + frameid * 4));
	VDE_WR(cr_paddr >> 8, regs + FRAMEID(0x180 + frameid * 4));
	VDE_WR(v1,            regs + FRAMEID(0x080 + frameid * 4));
	VDE_WR(v2,            regs + FRAMEID(0x280 + frameid * 4));
}

static void tegra_setup_frameidx(void __iomem *regs,
				 struct video_frame *frames,
				 int frames_nb, u32 mbs_width, u32 mbs_height)
{
	int idx;

	for (idx = 0; idx < frames_nb; idx++)
		tegra_vde_setup_frameid(regs, &frames[idx], idx,
					mbs_width, mbs_height);
	for (; idx < 17; idx++)
		tegra_vde_setup_frameid(regs, NULL, idx, 0, 0);
}

static void tegra_vde_write_iram_entry(void __iomem *tables,
				       int table, int row,
				       u32 value1, u32 value2)
{
	VDE_WR(value1, tables + 0x80 * table + row * 8);
	VDE_WR(value2, tables + 0x80 * table + row * 8 + 4);
}

static void tegra_vde_setup_iram_tables(void __iomem *iram_tables,
					struct video_frame *dpb_frames,
					int ref_frames_nb,
					int with_earlier_poc_nb)
{
	u32 value, aux_paddr;
	int with_later_poc_nb;
	int i, k;

	pr_debug("DPB: Frame 0: frame_num = %d\n", dpb_frames[0].frame_num);

	pr_debug("REF L0:\n");

	for (i = 0; i < 16; i++) {
		if (i < ref_frames_nb) {
			aux_paddr = dpb_frames[i + 1].aux_paddr;

			value  = (i + 1) << 26;
			value |= !(dpb_frames[i + 1].flags & FLAG_IS_B_FRAME) << 25;
			value |= 1 << 24;
			value |= dpb_frames[i + 1].frame_num;


			pr_debug("\tFrame %d: frame_num = %d is_B_frame = %d\n",
				 i + 1, dpb_frames[i + 1].frame_num,
				 (dpb_frames[i + 1].flags & FLAG_IS_B_FRAME));
		} else {
			aux_paddr = 0xFADEAD00;
			value = 0;
		}

		tegra_vde_write_iram_entry(iram_tables, 0, i, value, aux_paddr);
		tegra_vde_write_iram_entry(iram_tables, 1, i, value, aux_paddr);
		tegra_vde_write_iram_entry(iram_tables, 2, i, value, aux_paddr);
		tegra_vde_write_iram_entry(iram_tables, 3, i, value, aux_paddr);
	}

	if (!(dpb_frames[0].flags & FLAG_IS_B_FRAME))
		return;

	if (with_earlier_poc_nb >= ref_frames_nb)
		return;

	with_later_poc_nb = ref_frames_nb - with_earlier_poc_nb;

	pr_debug("REF L1: with_later_poc_nb %d with_earlier_poc_nb %d\n",
		 with_later_poc_nb, with_earlier_poc_nb);

	for (i = 0, k = with_earlier_poc_nb; i < with_later_poc_nb; i++, k++) {
		aux_paddr = dpb_frames[k + 1].aux_paddr;

		value  = (k + 1) << 26;
		value |= !(dpb_frames[k + 1].flags & FLAG_IS_B_FRAME) << 25;
		value |= 1 << 24;
		value |= dpb_frames[k + 1].frame_num;

		pr_debug("\tFrame %d: frame_num = %d\n",
			 k + 1, dpb_frames[k + 1].frame_num);

		tegra_vde_write_iram_entry(iram_tables, 2, i, value, aux_paddr);
	}

	for (k = 0; i < ref_frames_nb; i++, k++) {
		aux_paddr = dpb_frames[k + 1].aux_paddr;

		value  = (k + 1) << 26;
		value |= !(dpb_frames[k + 1].flags & FLAG_IS_B_FRAME) << 25;
		value |= 1 << 24;
		value |= dpb_frames[k + 1].frame_num;

		pr_debug("\tFrame %d: frame_num = %d\n",
			 k + 1, dpb_frames[k + 1].frame_num);

		tegra_vde_write_iram_entry(iram_tables, 2, i, value, aux_paddr);
	}
}

static int tegra_vde_setup_context(struct tegra_vde *vde,
				   struct tegra_vde_h264_decoder_ctx *ctx,
				   struct video_frame *dpb_frames,
				   phys_addr_t bitstream_data_paddr,
				   int bitstream_data_size,
				   int macroblocks_nb)
{
	struct device *dev = vde->miscdev.parent;
	u32 value;

	tegra_vde_set_bits(vde->regs,    0xA, SXE(0xF0));
	tegra_vde_set_bits(vde->regs,    0xB, BSEV(CMDQUE_CONTROL));
	tegra_vde_set_bits(vde->regs, 0x8002, MBE(0x50));
	tegra_vde_set_bits(vde->regs,    0xA, MBE(0xA0));
	tegra_vde_set_bits(vde->regs,    0xA, PPE(0x14));
	tegra_vde_set_bits(vde->regs,    0xA, PPE(0x28));
	tegra_vde_set_bits(vde->regs,  0xA00, MCE(0x08));
	tegra_vde_set_bits(vde->regs,    0xA, TFE(0x00));
	tegra_vde_set_bits(vde->regs,    0x5, VDMA(0x04));

	VDE_WR(0x00000000, vde->regs + VDMA(0x1C));
	VDE_WR(0x00000000, vde->regs + VDMA(0x00));
	VDE_WR(0x00000007, vde->regs + VDMA(0x04));
	VDE_WR(0x00000007, vde->regs + FRAMEID(0x200));
	VDE_WR(0x00000005, vde->regs + TFE(0x04));
	VDE_WR(0x00000000, vde->regs + MBE(0x84));
	VDE_WR(0x00000010, vde->regs + SXE(0x08));
	VDE_WR(0x00000150, vde->regs + SXE(0x54));
	VDE_WR(0x0000054C, vde->regs + SXE(0x58));
	VDE_WR(0x00000E34, vde->regs + SXE(0x5C));
	VDE_WR(0x063C063C, vde->regs + MCE(0x10));
	VDE_WR(0x0003FC00, vde->regs + BSEV(INTR_STATUS));
	VDE_WR(0x0000150D, vde->regs + BSEV(BSE_CONFIG));
	VDE_WR(0x00000100, vde->regs + BSEV(BSE_INT_ENB));
	VDE_WR(0x00000000, vde->regs + BSEV(0x98));
	VDE_WR(0x00000060, vde->regs + BSEV(0x9C));

	memset(vde->iram + 512, 0, macroblocks_nb / 2);

	tegra_setup_frameidx(vde->regs, dpb_frames, ctx->dpb_frames_nb,
			     ctx->pic_width_in_mbs, ctx->pic_height_in_mbs);

	tegra_vde_setup_iram_tables(vde->iram, dpb_frames,
				    ctx->dpb_frames_nb - 1,
				    ctx->dpb_ref_frames_with_earlier_poc_nb);

	VDE_WR(0x00000000, vde->regs + BSEV(0x8C));
	VDE_WR(bitstream_data_paddr + bitstream_data_size,
	       vde->regs + BSEV(0x54));

	value = ctx->pic_width_in_mbs << 11 | ctx->pic_height_in_mbs << 3;

	VDE_WR(value, vde->regs + BSEV(0x88));

	if (tegra_vde_wait_bsev(vde, 0))
		return -EIO;

	if (tegra_vde_push_bsev_icmdqueue(vde, 0x800003FC, 0))
		return -EIO;

	value = 0x01500000;
	value |= ((vde->iram_lists_paddr + 512) >> 2) & 0xFFFF;

	if (tegra_vde_push_bsev_icmdqueue(vde, value, 1))
		return -EIO;

	if (tegra_vde_push_bsev_icmdqueue(vde, 0x840F054C, 0))
		return -EIO;

	if (tegra_vde_push_bsev_icmdqueue(vde, 0x80000080, 0))
		return -EIO;

	value = 0x0E340000 | ((vde->iram_lists_paddr >> 2) & 0xFFFF);

	if (tegra_vde_push_bsev_icmdqueue(vde, value, 1))
		return -EIO;

	value = (1 << 23) | 5;
	value |= ctx->pic_width_in_mbs << 11;
	value |= ctx->pic_height_in_mbs << 3;

	VDE_WR(value, vde->regs + SXE(0x10));

	value = !ctx->is_baseline_profile << 17;
	value |= ctx->level_idc << 13;
	value |= ctx->log2_max_pic_order_cnt_lsb << 7;
	value |= ctx->pic_order_cnt_type << 5;
	value |= ctx->log2_max_frame_num;

	VDE_WR(value, vde->regs + SXE(0x40));

	value = ctx->pic_init_qp << 25;
	value |= !!(ctx->deblocking_filter_control_present_flag) << 2;
	value |= !!ctx->pic_order_present_flag;

	VDE_WR(value, vde->regs + SXE(0x44));

	value = ctx->chroma_qp_index_offset;
	value |= ctx->num_ref_idx_l0_active_minus1 << 5;
	value |= ctx->num_ref_idx_l1_active_minus1 << 10;
	value |= !!ctx->constrained_intra_pred_flag << 15;

	VDE_WR(value, vde->regs + SXE(0x48));

	value = 0x0C000000;
	value |= !!(dpb_frames[0].flags & FLAG_IS_B_FRAME) << 24;

	VDE_WR(value, vde->regs + SXE(0x4C));

	value = 0x03800000;
	value |= min(bitstream_data_size, SZ_1M);

	VDE_WR(value, vde->regs + SXE(0x68));

	VDE_WR(bitstream_data_paddr, vde->regs + SXE(0x6C));

	value = (1 << 28) | 5;
	value |= ctx->pic_width_in_mbs << 11;
	value |= ctx->pic_height_in_mbs << 3;

	VDE_WR(value, vde->regs + MBE(0x80));

	value = 0x26800000;
	value |= ctx->level_idc << 4;
	value |= !ctx->is_baseline_profile << 1;
	value |= !!ctx->direct_8x8_inference_flag;

	VDE_WR(value, vde->regs + MBE(0x80));

	VDE_WR(0xF4000001, vde->regs + MBE(0x80));
	VDE_WR(0x20000000, vde->regs + MBE(0x80));
	VDE_WR(0xF4000101, vde->regs + MBE(0x80));

	value = 0x20000000;
	value |= ctx->chroma_qp_index_offset << 8;

	VDE_WR(value, vde->regs + MBE(0x80));

	if (tegra_vde_setup_mbe_frame_idx(vde->regs,
					  (ctx->pic_order_cnt_type == 0),
					  ctx->dpb_frames_nb - 1)) {
		dev_err(dev, "MBE frames setup failed\n");
		return -EIO;
	}

	tegra_vde_mbe_set_0xa_reg(vde->regs, 0, 0x000009FC);
	tegra_vde_mbe_set_0xa_reg(vde->regs, 2, 0xF1DEAD00);
	tegra_vde_mbe_set_0xa_reg(vde->regs, 4, 0xF2DEAD00);
	tegra_vde_mbe_set_0xa_reg(vde->regs, 6, 0xF3DEAD00);
	tegra_vde_mbe_set_0xa_reg(vde->regs, 8, dpb_frames[0].aux_paddr);

	value = 0xFC000000;
	value |= !!(dpb_frames[0].flags & FLAG_IS_B_FRAME) << 2;

	if (!ctx->is_baseline_profile)
		value |= !!(dpb_frames[0].flags & FLAG_IS_REFERENCE) << 1;

	VDE_WR(value, vde->regs + MBE(0x80));

	if (tegra_vde_wait_mbe(vde->regs)) {
		dev_err(dev, "MBE programming failed\n");
		return -EIO;
	}

	return 0;
}

static void tegra_vde_detach_and_put_dmabuf(struct dma_buf_attachment *a)
{
	struct dma_buf *dmabuf = a->dmabuf;

	if (IS_ERR_OR_NULL(a))
		return;

	dma_buf_detach(dmabuf, a);
	dma_buf_put(dmabuf);
}

static int tegra_vde_attach_dmabuf(struct device *dev, int fd,
				   int min_size,
				   struct dma_buf_attachment **a,
				   phys_addr_t *paddr, u32 *size,
				   enum dma_data_direction dma_dir)
{
	struct dma_buf *dmabuf = dma_buf_get(fd);
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;

	*a = NULL;
	*paddr = 0xFBDEAD00;

	if (IS_ERR(dmabuf)) {
		dev_err(dev, "Invalid dmabuf FD\n");
		return PTR_ERR(dmabuf);
	}

	if (min_size > dmabuf->size) {
		dev_err(dev, "Too small dmabuf size %d should be at least %d\n",
			dmabuf->size, min_size);
		return -EINVAL;
	}

	attachment = dma_buf_attach(dmabuf, dev);

	if (IS_ERR(attachment)) {
		dev_err(dev, "Failed to attach dmabuf\n");
		dma_buf_put(dmabuf);
		return PTR_ERR(attachment);
	}

	sgt = dma_buf_map_attachment(attachment, dma_dir);

	if (IS_ERR(sgt)) {
		dev_err(dev, "Failed to get dmabuf sg_table\n");
		dma_buf_detach(dmabuf, attachment);
		dma_buf_put(dmabuf);
		return PTR_ERR(attachment);
	}

	if (sgt->nents != 1) {
		dev_err(dev, "Non-continuous DMA area unsupported\n");
		dma_buf_unmap_attachment(attachment, sgt, dma_dir);
		dma_buf_detach(dmabuf, attachment);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	*paddr = sg_dma_address(sgt->sgl);

	dma_buf_unmap_attachment(attachment, sgt, dma_dir);

	*a = attachment;

	if (size)
		*size = dmabuf->size;

	return 0;
}

static int tegra_vde_attach_frame_dmabufs(struct device *dev,
					  struct video_frame *frame,
					  enum dma_data_direction dma_dir,
					  int y_fd,
					  int cb_fd,
					  int cr_fd,
					  int aux_fd,
					  int is_baseline_profile, int csize)
{
	int ret;

	ret = tegra_vde_attach_dmabuf(dev, y_fd, csize * 4,
				      &frame->y_dmabuf_attachment,
				      &frame->y_paddr, NULL, dma_dir);
	if (ret)
		return ret;

	ret = tegra_vde_attach_dmabuf(dev, cb_fd, csize,
				      &frame->cb_dmabuf_attachment,
				      &frame->cb_paddr, NULL, dma_dir);
	if (ret)
		return ret;

	ret = tegra_vde_attach_dmabuf(dev, cr_fd, csize,
				      &frame->cr_dmabuf_attachment,
				      &frame->cr_paddr, NULL, dma_dir);
	if (ret)
		return ret;

	if (is_baseline_profile)
		frame->aux_paddr = 0xF4DEAD00;
	else
		ret = tegra_vde_attach_dmabuf(dev, aux_fd, csize,
					      &frame->aux_dmabuf_attachment,
					      &frame->aux_paddr, NULL, dma_dir);

	return ret;
}

static void tegra_vde_deattach_frame_dmabufs(struct video_frame *frame)
{
	tegra_vde_detach_and_put_dmabuf(frame->y_dmabuf_attachment);
	tegra_vde_detach_and_put_dmabuf(frame->cb_dmabuf_attachment);
	tegra_vde_detach_and_put_dmabuf(frame->cr_dmabuf_attachment);
	tegra_vde_detach_and_put_dmabuf(frame->aux_dmabuf_attachment);
}

static int tegra_vde_copy_and_validate_frame(struct device *dev,
					     struct tegra_vde_h264_frame *frame,
					     unsigned long vaddr)
{
	if (copy_from_user(frame, (void __user *) vaddr, sizeof(*frame))) {
		dev_err(dev, "Copying of H.264 frame from user failed\n");
		return -EFAULT;
	}

	if (!(frame->flags & FLAG_IS_VALID)) {
		dev_err(dev, "Frame FLAG_IS_VALID is not set\n");
		return -EINVAL;
	}

	if (frame->frame_num > 0x7FFFFF) {
		dev_err(dev, "Bad frame_num %u\n", frame->frame_num);
		return -EINVAL;
	}

	return 0;
}

static int tegra_vde_validate_h264_ctx(struct device *dev,
				       struct tegra_vde_h264_decoder_ctx *ctx)
{
	if (ctx->dpb_frames_nb == 0 || ctx->dpb_frames_nb > 17) {
		dev_err(dev, "Bad DPB size %u\n", ctx->dpb_frames_nb);
		return -EINVAL;
	}

	if (ctx->level_idc > 15) {
		dev_err(dev, "Bad level value %u\n", ctx->level_idc);
		return -EINVAL;
	}

	if (ctx->pic_init_qp > 52) {
		dev_err(dev, "Bad pic_init_qp value %u\n", ctx->pic_init_qp);
		return -EINVAL;
	}

	if (ctx->log2_max_pic_order_cnt_lsb > 16) {
		dev_err(dev, "Bad log2_max_pic_order_cnt_lsb value %u\n",
			ctx->log2_max_pic_order_cnt_lsb);
		return -EINVAL;
	}

	if (ctx->log2_max_frame_num > 16) {
		dev_err(dev, "Bad log2_max_frame_num value %u\n",
			ctx->log2_max_frame_num);
		return -EINVAL;
	}

	if (ctx->chroma_qp_index_offset > 31) {
		dev_err(dev, "Bad chroma_qp_index_offset value %u\n",
			ctx->chroma_qp_index_offset);
		return -EINVAL;
	}

	if (ctx->pic_order_cnt_type > 2) {
		dev_err(dev, "Bad pic_order_cnt_type value %u\n",
			ctx->pic_order_cnt_type);
		return -EINVAL;
	}

	if (ctx->num_ref_idx_l0_active_minus1 > 15) {
		dev_err(dev, "Bad num_ref_idx_l0_active_minus1 value %u\n",
			ctx->num_ref_idx_l0_active_minus1);
		return -EINVAL;
	}

	if (ctx->num_ref_idx_l1_active_minus1 > 15) {
		dev_err(dev, "Bad num_ref_idx_l1_active_minus1 value %u\n",
			ctx->num_ref_idx_l1_active_minus1);
		return -EINVAL;
	}

	if (!ctx->pic_width_in_mbs || ctx->pic_width_in_mbs > 127) {
		dev_err(dev, "Bad pic_width_in_mbs value %u, min 1 max 127\n",
			ctx->pic_width_in_mbs);
		return -EINVAL;
	}

	if (!ctx->pic_height_in_mbs || ctx->pic_height_in_mbs > 127) {
		dev_err(dev, "Bad pic_height_in_mbs value %u, min 1 max 127\n",
			ctx->pic_height_in_mbs);
		return -EINVAL;
	}

	return 0;
}

static int tegra_vde_ioctl_decode_h264(struct tegra_vde *vde,
				       unsigned long vaddr)
{
	struct tegra_vde_h264_decoder_ctx ctx;
	struct tegra_vde_h264_frame frame;
	struct device *dev = vde->miscdev.parent;
	struct video_frame *dpb_frames = NULL;
	struct dma_buf_attachment *bitstream_data_dmabuf_attachment = NULL;
	enum dma_data_direction dma_dir;
	phys_addr_t bitstream_data_paddr;
	phys_addr_t bsev_paddr;
	u32 bitstream_data_size;
	u32 macroblocks_nb;
	int i, ret;

	if (copy_from_user(&ctx, (void __user *) vaddr, sizeof(ctx))) {
		dev_err(dev, "Copying of H.264 ctx from user failed\n");
		return -EFAULT;
	}

	ret = tegra_vde_validate_h264_ctx(dev, &ctx);

	if (ret)
		return -EINVAL;

	mutex_lock(&vde->lock);

	ret = tegra_vde_attach_dmabuf(dev, ctx.bitstream_data_fd, 0,
				      &bitstream_data_dmabuf_attachment,
				      &bitstream_data_paddr,
				      &bitstream_data_size,
				      DMA_TO_DEVICE);
	if (ret)
		goto cleanup;

	dpb_frames = kzalloc(sizeof(struct video_frame) * ctx.dpb_frames_nb,
			     GFP_KERNEL);
	if (!dpb_frames) {
		ret = -ENOMEM;
		goto cleanup;
	}

	macroblocks_nb = ctx.pic_width_in_mbs * ctx.pic_height_in_mbs;
	vaddr = ctx.dpb_frames_ptr;

	for (i = 0; i < ctx.dpb_frames_nb; i++) {
		ret = tegra_vde_copy_and_validate_frame(dev, &frame, vaddr);

		if (ret)
			goto cleanup;

		dpb_frames[i].flags = frame.flags;
		dpb_frames[i].frame_num = frame.frame_num;

		dma_dir = (i == 0) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;

		ret = tegra_vde_attach_frame_dmabufs(dev,
						     &dpb_frames[i],
						     dma_dir,
						     frame.y_fd,
						     frame.cb_fd,
						     frame.cr_fd,
						     frame.aux_fd,
						     ctx.is_baseline_profile,
						     macroblocks_nb * 64);
		if (ret) {
			tegra_vde_deattach_frame_dmabufs(&dpb_frames[i]);
			goto cleanup;
		}

		vaddr += sizeof(frame);
	}

	ret = tegra_vde_reset(vde);

	if (ret)
		goto cleanup;

	ret = clk_prepare_enable(vde->clk);

	if (ret) {
		dev_err(dev, "Failed to enable clk\n");
		goto cleanup;
	}

	ret = tegra_vde_setup_context(vde, &ctx, dpb_frames,
				      bitstream_data_paddr,
				      bitstream_data_size,
				      macroblocks_nb);
	if (ret)
		goto reset_cleanup;

	vde->decoding_in_progress = 1;

	VDE_WR(0x00000001, vde->regs + BSEV(0x8C));
	VDE_WR(0x20000000 | (macroblocks_nb - 1), vde->regs + SXE(0x00));

	ret = !wait_for_completion_io_timeout(&vde->decode_completion,
					      TEGRA_VDE_TIMEOUT);
	if (ret) {
		bsev_paddr = readl(vde->regs + BSEV(0x10));
		macroblocks_nb = readl(vde->regs + SXE(0xC8)) & 0x1FFF;

		dev_err(dev, "Decoding failed, "
				"read 0x%X bytes : %u macroblocks parsed\n",
			bsev_paddr ? bsev_paddr - bitstream_data_paddr : 0,
			macroblocks_nb);

		ret = -EIO;
	}

reset_cleanup:
	reset_control_assert(vde->rst);

	clk_disable_unprepare(vde->clk);

	vde->decoding_in_progress = 0;

cleanup:
	if (dpb_frames)
		while (i--) {
			tegra_vde_deattach_frame_dmabufs(&dpb_frames[i]);
		}

	kfree(dpb_frames);

	tegra_vde_detach_and_put_dmabuf(bitstream_data_dmabuf_attachment);

	mutex_unlock(&vde->lock);

	return ret;
}

static long tegra_vde_unlocked_ioctl(struct file *filp,
				     unsigned int cmd, unsigned long arg)
{
	struct miscdevice *miscdev = filp->private_data;
	struct tegra_vde *vde = container_of(miscdev, struct tegra_vde,
					     miscdev);

	switch (cmd) {
	case TEGRA_VDE_IOCTL_ALLOC_DMA_BUF:
		return tegra_vde_alloc_dma_buf(miscdev->parent, arg);
	case TEGRA_VDE_IOCTL_DECODE_H264:
		return tegra_vde_ioctl_decode_h264(vde, arg);
	}

	dev_err(miscdev->parent, "Invalid IOCTL command %u\n", cmd);

	return -ENOTTY;
}

static const struct file_operations tegra_vde_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= tegra_vde_unlocked_ioctl,
};

static irqreturn_t tegra_vde_isr(int irq, void *data)
{
	struct tegra_vde *vde = data;

	if (!vde->decoding_in_progress)
		return IRQ_NONE;

	tegra_vde_set_bits(vde->regs, 0, FRAMEID(0x208));
	complete(&vde->decode_completion);

	return IRQ_HANDLED;
}

static int tegra_vde_probe(struct platform_device *pdev)
{
	struct resource *res_regs, *res_iram;
	struct device *dev = &pdev->dev;
	struct tegra_vde *vde;
	int ret;

	vde = devm_kzalloc(&pdev->dev, sizeof(*vde), GFP_KERNEL);
	if (!vde)
		return -ENOMEM;

	platform_set_drvdata(pdev, vde);

	res_regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	if (!res_regs)
		return -ENODEV;

	res_iram = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iram");
	if (!res_iram)
		return -ENODEV;

	vde->irq = platform_get_irq_byname(pdev, "sync-token");
	if (vde->irq < 0)
		return -ENODEV;

	vde->regs = devm_ioremap_resource(dev, res_regs);
	if (IS_ERR(vde->regs))
		return PTR_ERR(vde->regs);

	vde->iram = devm_ioremap_resource(dev, res_iram);
	if (IS_ERR(vde->iram))
		return PTR_ERR(vde->iram);

	vde->clk = devm_clk_get(dev, "vde");
	if (IS_ERR(vde->clk)) {
		dev_err(dev, "Could not get VDE clk\n");
		return PTR_ERR(vde->clk);
	}

	vde->rst = devm_reset_control_get(dev, "vde");
	if (IS_ERR(vde->rst)) {
		dev_err(dev, "Could not get VDE reset\n");
		return PTR_ERR(vde->rst);
	}

	ret = devm_request_irq(dev, vde->irq, tegra_vde_isr, IRQF_SHARED,
			       dev_name(dev), vde);
	if (ret)
		return -ENODEV;

	mutex_init(&vde->lock);
	init_completion(&vde->decode_completion);

	vde->iram_lists_paddr = res_iram->start;
	vde->miscdev.minor = MISC_DYNAMIC_MINOR;
	vde->miscdev.name = "tegra_vde";
	vde->miscdev.fops = &tegra_vde_fops;
	vde->miscdev.parent = dev;

	ret = misc_register(&vde->miscdev);

	if (ret)
		return -ENODEV;

	return 0;
}

static int tegra_vde_remove(struct platform_device *pdev)
{
	struct tegra_vde *vde = platform_get_drvdata(pdev);

	misc_deregister(&vde->miscdev);

	return 0;
}

static int tegra_vde_suspend(struct device *dev)
{
	struct tegra_vde *vde = dev_get_drvdata(dev);

	return !vde->decoding_in_progress;
}

static SIMPLE_DEV_PM_OPS(tegra_vde_pm, tegra_vde_suspend, NULL);

static const struct of_device_id tegra_vde_of_match[] = {
	{ .compatible = "nvidia,tegra20-vde", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_vde_of_match);

static struct platform_driver tegra_vde_driver = {
	.probe		= tegra_vde_probe,
	.remove		= tegra_vde_remove,
	.driver		= {
		.name		= "tegra-vde",
		.owner		= THIS_MODULE,
		.of_match_table = tegra_vde_of_match,
		.pm		= &tegra_vde_pm,
	},
};

module_platform_driver(tegra_vde_driver);

MODULE_DEVICE_TABLE(of, tegra_vde_of_match);
MODULE_DESCRIPTION("NVIDIA Terga20 Video Decoder driver");
MODULE_AUTHOR("Dmitry Osipenko");
MODULE_LICENSE("GPL");
