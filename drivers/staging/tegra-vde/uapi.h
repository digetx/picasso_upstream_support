/*
 * Copyright (C) 2016 Dmitry Osipenko <digetx@gmail.com>
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _UAPI_TEGRA_VDE_H_
#define _UAPI_TEGRA_VDE_H_

#include <linux/types.h>
#include <asm/ioctl.h>

#if defined(__cplusplus)
extern "C" {
#endif

#define FLAG_IS_B_FRAME		(1 << 0)
#define FLAG_IS_REFERENCE	(1 << 2)
#define FLAG_IS_VALID		(1 << 7)

struct tegra_vde_h264_frame {
    __s32 y_fd;
    __s32 cb_fd;
    __s32 cr_fd;
    __s32 aux_fd;
    __u32 frame_num;
    __u8  flags;
} __attribute__((packed));

struct tegra_vde_h264_decoder_ctx {
    __s32 bitstream_data_fd;

    __u8  dpb_frames_nb;
    __u32 dpb_frames_ptr;
    __u8  dpb_ref_frames_with_earlier_poc_nb;

    // SPS
    __u8  is_baseline_profile;
    __u8  level_idc;
    __u8  log2_max_pic_order_cnt_lsb;
    __u8  log2_max_frame_num;
    __u8  pic_order_cnt_type;
    __u8  direct_8x8_inference_flag;
    __u8  pic_width_in_mbs;
    __u8  pic_height_in_mbs;

    // PPS
    __u8  pic_init_qp;
    __u8  deblocking_filter_control_present_flag;
    __u8  constrained_intra_pred_flag;
    __u8  chroma_qp_index_offset;
    __u8  pic_order_present_flag;

    // Slice header
    __u8  num_ref_idx_l0_active_minus1;
    __u8  num_ref_idx_l1_active_minus1;
} __attribute__((packed));

#define VDE_IOCTL_BASE			'v'
#define VDE_IO(nr)			_IO(VDE_IOCTL_BASE,nr)
#define VDE_IOR(nr,type)		_IOR(VDE_IOCTL_BASE,nr,type)
#define VDE_IOW(nr,type)		_IOW(VDE_IOCTL_BASE,nr,type)
#define VDE_IOWR(nr,type)		_IOWR(VDE_IOCTL_BASE,nr,type)

#define TEGRA_VDE_ALLOC_DMA_BUF		0x00
#define TEGRA_VDE_DECODE_H264		0x01

#define TEGRA_VDE_IOCTL_ALLOC_DMA_BUF	VDE_IO(VDE_IOCTL_BASE + TEGRA_VDE_ALLOC_DMA_BUF)
#define TEGRA_VDE_IOCTL_DECODE_H264	VDE_IOW(VDE_IOCTL_BASE + TEGRA_VDE_DECODE_H264, struct tegra_vde_h264_decoder_ctx)

#if defined(__cplusplus)
}
#endif

#endif // _UAPI_TEGRA_VDE_H_
