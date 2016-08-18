/*
 * NVIDIA Tegra20 Video decoder driver
 *
 * Copyright (C) 2016 Dmitry Osipenko <digetx@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/slab.h>

#include "vde.h"

struct tegra_vde_bo {
	struct device *dev;
	void *vaddr;
	dma_addr_t paddr;
};

static struct sg_table *
tegra_vde_dmabuf_map(struct dma_buf_attachment *attach,
		     enum dma_data_direction dir)
{
	struct dma_buf *dmabuf = attach->dmabuf;
	struct tegra_vde_bo *bo = dmabuf->priv;
	struct sg_table *sgt;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	if (sg_alloc_table(sgt, 1, GFP_KERNEL))
		goto free;

	sg_dma_address(sgt->sgl) = bo->paddr;
	sg_dma_len(sgt->sgl) = attach->dmabuf->size;

	return sgt;

free:
	kfree(sgt);
	return NULL;
}

static void tegra_vde_unmap_dma_buf(struct dma_buf_attachment *attach,
				    struct sg_table *sgt,
				    enum dma_data_direction dir)
{
	sg_free_table(sgt);
	kfree(sgt);
}

static void tegra_vde_release_dmabuf(struct dma_buf *dmabuf)
{
	struct tegra_vde_bo *bo = dmabuf->priv;

	dma_free_wc(bo->dev, dmabuf->size, bo->vaddr, bo->paddr);
	kfree(bo);
}

static void *tegra_vde_dmabuf_kmap_atomic(struct dma_buf *dmabuf,
					  unsigned long page)
{
	return NULL;
}

static void tegra_vde_dmabuf_kunmap_atomic(struct dma_buf *dmabuf,
					   unsigned long page,
					   void *addr)
{
}

static void *tegra_vde_dmabuf_kmap(struct dma_buf *dmabuf, unsigned long page)
{
	return NULL;
}

static void tegra_vde_dmabuf_kunmap(struct dma_buf *dmabuf, unsigned long page,
				    void *addr)
{
}

static int tegra_vde_dmabuf_mmap(struct dma_buf *dmabuf,
				 struct vm_area_struct *vma)
{
	struct tegra_vde_bo *bo = dmabuf->priv;

	return dma_mmap_wc(bo->dev, vma, bo->vaddr, bo->paddr, dmabuf->size);
}

static void *tegra_vde_dmabuf_vmap(struct dma_buf *dmabuf)
{
	struct tegra_vde_bo *bo = dmabuf->priv;

	return bo->vaddr;
}

static void tegra_vde_dmabuf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
}

static int tegra_vde_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
					     enum dma_data_direction dir)
{
	struct tegra_vde_bo *bo = dmabuf->priv;

	dma_sync_single_for_cpu(bo->dev, bo->paddr, dmabuf->size, dir);

	return 0;
}

static int tegra_vde_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
					   enum dma_data_direction dir)
{
	struct tegra_vde_bo *bo = dmabuf->priv;

	dma_sync_single_for_device(bo->dev, bo->paddr, dmabuf->size, dir);

	return 0;
}

static const struct dma_buf_ops tegra_vde_dmabuf_ops = {
	.map_dma_buf = tegra_vde_dmabuf_map,
	.unmap_dma_buf = tegra_vde_unmap_dma_buf,
	.release = tegra_vde_release_dmabuf,
	.kmap_atomic = tegra_vde_dmabuf_kmap_atomic,
	.kunmap_atomic = tegra_vde_dmabuf_kunmap_atomic,
	.kmap = tegra_vde_dmabuf_kmap,
	.kunmap = tegra_vde_dmabuf_kunmap,
	.mmap = tegra_vde_dmabuf_mmap,
	.vmap = tegra_vde_dmabuf_vmap,
	.vunmap = tegra_vde_dmabuf_vunmap,
	.begin_cpu_access = tegra_vde_dmabuf_begin_cpu_access,
	.end_cpu_access = tegra_vde_dmabuf_end_cpu_access,
};

int tegra_vde_alloc_dma_buf(struct device *dev, size_t size)
{
	struct tegra_vde_bo *bo;
	struct dma_buf *dmabuf;

	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return -ENOMEM;

	size = roundup(size, PAGE_SIZE);

	bo->vaddr = dma_alloc_wc(dev, size, &bo->paddr,
				 GFP_KERNEL | __GFP_NOWARN);
	if (!bo->vaddr) {
		dev_err(dev, "failed to allocate buffer of size %zu\n", size);
		kfree(bo);
		return -ENOMEM;
	}

	bo->dev = dev;

	exp_info.ops = &tegra_vde_dmabuf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR | O_CLOEXEC;
	exp_info.priv = bo;

	dmabuf = dma_buf_export(&exp_info);

	if (IS_ERR(dmabuf)) {
		dma_free_wc(dev, size, bo->vaddr, bo->paddr);
		kfree(bo);
		return PTR_ERR(dmabuf);
	}

	return dma_buf_fd(dmabuf, exp_info.flags);
}
EXPORT_SYMBOL_GPL(tegra_vde_alloc_dma_buf);
