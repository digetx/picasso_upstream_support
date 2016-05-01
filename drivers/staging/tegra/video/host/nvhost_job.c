/*
 * drivers/video/tegra/host/nvhost_job.c
 *
 * Tegra Graphics Host Job
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/err.h>
#include <linux/vmalloc.h>
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"
#include "dev.h"
#include "nvhost_memmgr.h"
#include "chip_support.h"

/* Magic to use to fill freed handle slots */
#define BAD_MAGIC 0xdeadbeef

static size_t job_size(struct nvhost_submit_hdr_ext *hdr)
{
	s64 num_relocs = hdr ? (int)hdr->num_relocs : 0;
	s64 num_waitchks = hdr ? (int)hdr->num_waitchks : 0;
	s64 num_cmdbufs = hdr ? (int)hdr->num_cmdbufs : 0;
	s64 num_pins = (num_cmdbufs + num_relocs) * 2;
	s64 total;

	if(num_relocs < 0 || num_waitchks < 0 || num_cmdbufs < 0)
		return 0;

	total = sizeof(struct nvhost_job)
			+ num_pins * sizeof(struct nvmap_pinarray_elem)
			+ num_pins * sizeof(struct nvmap_handle *)
			+ num_waitchks * sizeof(struct nvhost_waitchk);

	if(total > ULONG_MAX)
		return 0;
	return (size_t)total;
}

static size_t gather_size(int num_cmdbufs)
{
	return num_cmdbufs * sizeof(struct nvhost_job_gather);
}

static void free_gathers(struct nvhost_job *job)
{
	if (job->gathers) {
		mem_op().munmap(job->gather_mem, job->gathers);
		job->gathers = NULL;
	}
	if (job->gather_mem) {
		mem_op().put(job->memmgr, job->gather_mem);
		job->gather_mem = NULL;
	}
}

static int alloc_gathers(struct nvhost_job *job,
		int num_cmdbufs)
{
	int err = 0;

	job->gather_mem = NULL;
	job->gathers = NULL;
	job->gather_mem_size = 0;

	if (num_cmdbufs) {
		/* Allocate memory */
		job->gather_mem = mem_op().alloc(job->memmgr,
				gather_size(num_cmdbufs),
				32, mem_mgr_flag_write_combine);
		if (IS_ERR_OR_NULL(job->gather_mem)) {
			err = job->gather_mem ? PTR_ERR(job->gather_mem) : -ENOMEM;
			job->gather_mem = NULL;
			goto error;
		}
		job->gather_mem_size = gather_size(num_cmdbufs);

		/* Map memory to kernel */
		job->gathers = mem_op().mmap(job->gather_mem);
		if (IS_ERR_OR_NULL(job->gathers)) {
			err = job->gathers ? PTR_ERR(job->gathers) : -ENOMEM;
			job->gathers = NULL;
			goto error;
		}
	}

	return 0;

error:
	free_gathers(job);
	return err;
}

static int realloc_gathers(struct nvhost_job *oldjob,
		struct nvhost_job *newjob,
		int num_cmdbufs)
{
	int err = 0;

	/* Check if we can reuse gather buffer */
	if (oldjob->gather_mem_size < gather_size(num_cmdbufs)
			|| oldjob->memmgr != newjob->memmgr) {
		free_gathers(oldjob);
		err = alloc_gathers(newjob, num_cmdbufs);
	} else {
		newjob->gather_mem = oldjob->gather_mem;
		newjob->gathers = oldjob->gathers;
		newjob->gather_mem_size = oldjob->gather_mem_size;

		oldjob->gather_mem = NULL;
		oldjob->gathers = NULL;
		oldjob->gather_mem_size = 0;
	}
	return err;
}

static void init_fields(struct nvhost_job *job,
		struct nvhost_submit_hdr_ext *hdr,
		int priority, int clientid)
{
	int num_pins = hdr ? (hdr->num_relocs + hdr->num_cmdbufs)*2 : 0;
	int num_waitchks = hdr ? hdr->num_waitchks : 0;
	void *mem = job;

	/* First init set state */
	job->priority = priority;
	job->clientid = clientid;

	/* Redistribute memory to the structs */
	mem += sizeof(struct nvhost_job);
	if (num_pins) {
		job->pinarray = mem;
		mem += num_pins * sizeof(struct nvmap_pinarray_elem);
		job->unpins = mem;
		mem += num_pins * sizeof(struct nvmap_handle *);
	} else {
		job->pinarray = NULL;
		job->unpins = NULL;
	}

	job->waitchk = num_waitchks ? mem : NULL;

	/* Copy information from header */
	if (hdr) {
		job->waitchk_mask = hdr->waitchk_mask;
		job->syncpt_id = hdr->syncpt_id;
		job->syncpt_incrs = hdr->syncpt_incrs;
	}
}

struct nvhost_job *nvhost_job_alloc(struct nvhost_channel *ch,
		struct nvhost_hwctx *hwctx,
		struct nvhost_submit_hdr_ext *hdr,
		struct mem_mgr *memmgr,
		int priority,
		int clientid)
{
	struct nvhost_job *job = NULL;
	int num_cmdbufs = hdr ? hdr->num_cmdbufs : 0;
	size_t size = job_size(hdr);
	int err = 0;

	if (!size)
		goto error;
	job = vzalloc(size);
	if (!job)
		goto error;

	kref_init(&job->ref);
	job->ch = ch;
	job->hwctx = hwctx;
	if (hwctx)
		hwctx->h->get(hwctx);
	job->memmgr = memmgr ? mem_op().get_mgr(memmgr) : NULL;

	err = alloc_gathers(job, num_cmdbufs);
	if (err)
		goto error;

	init_fields(job, hdr, priority, clientid);

	return job;

error:
	if (job)
		nvhost_job_put(job);
	return NULL;
}

struct nvhost_job *nvhost_job_realloc(
		struct nvhost_job *oldjob,
		struct nvhost_hwctx *hwctx,
		struct nvhost_submit_hdr_ext *hdr,
		struct mem_mgr *memmgr,
		int priority,
		int clientid)
{
	struct nvhost_job *newjob = NULL;
	int num_cmdbufs = hdr ? hdr->num_cmdbufs : 0;
	int err = 0;

	newjob = vzalloc(job_size(hdr));
	if (!newjob)
		goto error;
	kref_init(&newjob->ref);
	newjob->ch = oldjob->ch;
	newjob->hwctx = hwctx;
	if (hwctx)
		newjob->hwctx->h->get(newjob->hwctx);
	newjob->timeout = oldjob->timeout;
	newjob->memmgr = memmgr ? mem_op().get_mgr(memmgr) : NULL;

	err = realloc_gathers(oldjob, newjob, num_cmdbufs);
	if (err)
		goto error;

	nvhost_job_put(oldjob);

	init_fields(newjob, hdr, priority, clientid);

	return newjob;

error:
	if (newjob)
		nvhost_job_put(newjob);
	if (oldjob)
		nvhost_job_put(oldjob);
	return NULL;
}

void nvhost_job_get(struct nvhost_job *job)
{
	kref_get(&job->ref);
}

static void job_free(struct kref *ref)
{
	struct nvhost_job *job = container_of(ref, struct nvhost_job, ref);

	if (job->hwctxref)
		job->hwctxref->h->put(job->hwctxref);
	if (job->hwctx)
		job->hwctx->h->put(job->hwctx);
	if (job->gathers)
		mem_op().munmap(job->gather_mem, job->gathers);
	if (job->gather_mem)
		mem_op().put(job->memmgr, job->gather_mem);
	if (job->memmgr)
		mem_op().put_mgr(job->memmgr);
	vfree(job);
}

/* Acquire reference to a hardware context. Used for keeping saved contexts in
 * memory. */
void nvhost_job_get_hwctx(struct nvhost_job *job, struct nvhost_hwctx *hwctx)
{
	BUG_ON(job->hwctxref);

	job->hwctxref = hwctx;
	hwctx->h->get(hwctx);
}

void nvhost_job_put(struct nvhost_job *job)
{
	kref_put(&job->ref, job_free);
}

void nvhost_job_add_gather(struct nvhost_job *job,
		u32 mem_id, u32 words, u32 offset)
{
	struct nvmap_pinarray_elem *pin;
	struct nvhost_job_gather *cur_gather =
			&job->gathers[job->num_gathers];

	pin = &job->pinarray[job->num_pins++];
	pin->patch_mem = (u32)nvmap_ref_to_handle(job->gather_mem);
	pin->patch_offset = (void *)&(cur_gather->mem) - (void *)job->gathers;
	pin->pin_mem = nvmap_convert_handle_u2k(mem_id);
	pin->pin_offset = offset;
	cur_gather->words = words;
	cur_gather->mem_id = mem_id;
	cur_gather->offset = offset;
	cur_gather->ref = mem_op().get(job->memmgr, mem_id);
	job->num_gathers += 1;
}

int nvhost_job_pin(struct nvhost_job *job)
{
	int err = 0;

	/* pin mem handles and patch physical addresses */
	job->num_unpins = nvmap_pin_array((struct nvmap_client *)job->memmgr,
				nvmap_ref_to_handle(job->gather_mem),
				job->pinarray, job->num_pins,
				(struct nvmap_handle **) job->unpins);
	if (job->num_unpins < 0)
		err = job->num_unpins;

	return err;
}

void nvhost_job_unpin(struct nvhost_job *job)
{
	nvmap_unpin_handles((struct nvmap_client *)job->memmgr,
			(struct nvmap_handle **) job->unpins,
			job->num_unpins);
	memset(job->unpins, BAD_MAGIC,
			job->num_unpins * sizeof(struct nvmap_handle *));
	job->num_unpins = 0;
}

/**
 * Debug routine used to dump job entries
 */
void nvhost_job_dump(struct device *dev, struct nvhost_job *job)
{
	dev_dbg(dev, "    SYNCPT_ID   %d\n",
		job->syncpt_id);
	dev_dbg(dev, "    SYNCPT_VAL  %d\n",
		job->syncpt_end);
	dev_dbg(dev, "    FIRST_GET   0x%x\n",
		job->first_get);
	dev_dbg(dev, "    TIMEOUT     %d\n",
		job->timeout);
	dev_dbg(dev, "    CTX 0x%p\n",
		job->hwctx);
	dev_dbg(dev, "    NUM_SLOTS   %d\n",
		job->num_slots);
	dev_dbg(dev, "    NUM_HANDLES %d\n",
		job->num_unpins);
}
