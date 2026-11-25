// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 */

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[blocktag][mmc]" fmt

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/srcu.h>
#include <linux/sched/clock.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/scatterlist.h>
#include "blocktag-internal.h"
#include "blocktag-mmc.h"
#include "mtk-mmc.h"

/* ring trace for debugfs, eMMC & SD */
DEFINE_STATIC_SRCU(mmc_btag_readers);
static DEFINE_MUTEX(mmc_btag_owner_lock);
static struct mtk_blocktag __rcu *mmc_mtk_btag;
#include "mmc_btag_perf.h"


static struct mmc_mtk_bio_context_task *mmc_mtk_bio_get_task(
	struct mmc_mtk_bio_context *ctx, __u16 task_id)
{
	struct mmc_mtk_bio_context_task *tsk = NULL;

	if (!ctx)
		return NULL;

	if (task_id >= MMC_BIOLOG_CONTEXT_TASKS) {
		pr_notice("invalid task id %d\n", task_id);
		return NULL;
	}

	tsk = &ctx->task[task_id];

	return tsk;
}

static struct mmc_mtk_bio_context *mmc_mtk_bio_curr_ctx(struct mtk_blocktag *btag, bool is_sd)
{
	struct mmc_mtk_bio_context *ctx = BTAG_CTX(btag);

	if (is_sd)
		return ctx ? &ctx[1] : NULL;
	else
		return ctx ? &ctx[0] : NULL;
}

static struct mmc_mtk_bio_context_task *mmc_mtk_bio_curr_task(
	struct mtk_blocktag *btag, __u16 task_id, struct mmc_mtk_bio_context **curr_ctx, bool is_sd)
{
	struct mmc_mtk_bio_context *ctx;

	ctx = mmc_mtk_bio_curr_ctx(btag, is_sd);
	if (curr_ctx)
		*curr_ctx = ctx;
	return mmc_mtk_bio_get_task(ctx, task_id);
}

static void btag_mmc_pidlog_insert(struct mtk_blocktag *btag, struct mmc_data *data, bool is_sd,
				   __u64 *tot_len, __u64 *top_len)
{
	struct mmc_mtk_bio_context *ctx;
	struct scatterlist *sg;
	__u16 insert_pid[BTAG_PIDLOG_ENTRIES] = {0};
	__u32 insert_len[BTAG_PIDLOG_ENTRIES] = {0};
	__u32 insert_cnt = 0;
	enum mtk_btag_io_type io_type;
	unsigned long flags;
	unsigned int index;

	*tot_len = 0;
	*top_len = 0;
	if (!data || !data->sg || !data->sg_len)
		return;
	if ((data->flags & (MMC_DATA_READ | MMC_DATA_WRITE)) == MMC_DATA_READ)
		io_type = BTAG_IO_READ;
	else if ((data->flags & (MMC_DATA_READ | MMC_DATA_WRITE)) == MMC_DATA_WRITE)
		io_type = BTAG_IO_WRITE;
	else
		return;
	ctx = mmc_mtk_bio_curr_ctx(btag, is_sd);
	if (!ctx)
		return;

	for_each_sg(data->sg, sg, data->sg_len, index) {
		unsigned long offset = sg->offset;
		unsigned int remaining = sg->length;
		struct page *first = sg_page(sg);

		if (!first)
			continue;
		while (remaining) {
			struct page *page = nth_page(first, offset >> PAGE_SHIFT);
			unsigned int bytes = min_t(unsigned int, remaining,
				PAGE_SIZE - (offset & (PAGE_SIZE - 1)));
			short pid = mtk_btag_page_pidlog_get(page);
			int slot;

			mtk_btag_page_pidlog_set(page, 0);
			*tot_len += bytes;
			if (pid < 0) {
				*top_len += bytes;
				pid = -pid;
			}
			if (pid) {
				for (slot = 0; slot < BTAG_PIDLOG_ENTRIES; slot++) {
					if (!insert_pid[slot]) {
						insert_pid[slot] = pid;
						insert_len[slot] = bytes;
						insert_cnt++;
						break;
					}
					if (insert_pid[slot] == pid) {
						insert_len[slot] += bytes;
						break;
					}
				}
			}
			remaining -= bytes;
			offset += bytes;
		}
	}
	spin_lock_irqsave(&ctx->lock, flags);
	mtk_btag_pidlog_insert(&ctx->pidlog, insert_pid, insert_len, insert_cnt, io_type);
	spin_unlock_irqrestore(&ctx->lock, flags);
}

static void mmc_mtk_biolog_send_command_owned(struct mtk_blocktag *btag, __u16 task_id, struct mmc_request *mrq)
{
	unsigned long flags;
	struct mmc_host *mmc;
	struct mmc_mtk_bio_context *ctx;
	struct mmc_mtk_bio_context_task *tsk;
	bool is_sd;
	__u64 tot_len, top_len;
	struct mmc_data *data;

	if (!mrq)
		return;

	mmc = mrq->host;
	if (!mmc)
		return;


	if (!(mmc->caps2 & MMC_CAP2_NO_MMC))
		is_sd = false;
	else if (!(mmc->caps2 & MMC_CAP2_NO_SD))
		is_sd = true;
	else
		return;

	tsk = mmc_mtk_bio_curr_task(btag, task_id, &ctx, is_sd);
	if (!tsk)
		return;

	data = mrq->data ? mrq->data : (mrq->cmd ? mrq->cmd->data : NULL);
	if (!data)
		return;
	if (is_sd) {
		if (!mrq->cmd || !(mrq->cmd->opcode == MMC_READ_SINGLE_BLOCK ||
		    mrq->cmd->opcode == MMC_READ_MULTIPLE_BLOCK ||
		    mrq->cmd->opcode == MMC_WRITE_BLOCK ||
		    mrq->cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK))
			return;
	} else if (mrq->cmd) {
		return;
	}
	btag_mmc_pidlog_insert(btag, data, is_sd, &tot_len, &top_len);
	tsk->len = data->blksz * data->blocks;
	tsk->dir = MMC_DATA_DIR(!(data->flags & MMC_DATA_READ));

	spin_lock_irqsave(&ctx->lock, flags);
	tsk->t[tsk_send_cmd] = sched_clock();
	tsk->t[tsk_req_compl] = 0;

	if (!ctx->period_start_t)
		ctx->period_start_t = tsk->t[tsk_send_cmd];

	ctx->q_depth++;
	spin_unlock_irqrestore(&ctx->lock, flags);

	/* mictx send logging */
	if (!is_sd)
		mtk_btag_mictx_send_command(btag, tsk->t[tsk_send_cmd],
				    tsk->dir ? BTAG_IO_WRITE : BTAG_IO_READ,
				    tot_len, top_len, task_id, 0);
}

void mmc_mtk_biolog_send_command(__u16 task_id, struct mmc_request *mrq)
{
	struct mtk_blocktag *btag;
	int index;

	index = srcu_read_lock(&mmc_btag_readers);
	btag = srcu_dereference(mmc_mtk_btag, &mmc_btag_readers);
	if (btag)
		mmc_mtk_biolog_send_command_owned(btag, task_id, mrq);
	srcu_read_unlock(&mmc_btag_readers, index);
}
EXPORT_SYMBOL_GPL(mmc_mtk_biolog_send_command);

static void mmc_mtk_biolog_transfer_req_compl_owned(struct mtk_blocktag *btag, struct mmc_host *mmc, __u16 task_id, unsigned long req_mask)
{
	struct mmc_mtk_bio_context *ctx;
	struct mmc_mtk_bio_context_task *tsk;
	unsigned long flags;
	enum mtk_btag_io_type io_type;
	__u64 busy_time, cur_time = sched_clock();
	__u32 size;
	bool is_sd;

	if (!mmc)
		return;
	if (!(mmc->caps2 & MMC_CAP2_NO_MMC))
		is_sd = false;
	else if (!(mmc->caps2 & MMC_CAP2_NO_SD))
		is_sd = true;
	else
		return;

	tsk = mmc_mtk_bio_curr_task(btag, task_id, &ctx, is_sd);
	if (!tsk)
		return;

	/* return if there's no on-going request  */
	if (!tsk->t[tsk_send_cmd])
		return;

	/* mictx complete logging */
	mtk_btag_mictx_complete_command(btag, cur_time, task_id, 0);

	spin_lock_irqsave(&ctx->lock, flags);

	tsk->t[tsk_req_compl] = cur_time;

	io_type = tsk->dir ? BTAG_IO_WRITE : BTAG_IO_READ;

	/* throughput usage := duration of handling this request */
	busy_time = tsk->t[tsk_req_compl] - tsk->t[tsk_send_cmd];

	/* workload statistics */
	ctx->workload.count++;

	if (io_type < BTAG_IO_TYPE_NR) {
		size = tsk->len;
		ctx->throughput[io_type].usage += busy_time;
		ctx->throughput[io_type].size += size;
	}

	if (!req_mask)
		ctx->q_depth = 0;
	else
		ctx->q_depth--;

	/* clear this task */
	tsk->t[tsk_send_cmd] = tsk->t[tsk_req_compl] = 0;

	spin_unlock_irqrestore(&ctx->lock, flags);
}

void mmc_mtk_biolog_transfer_req_compl(struct mmc_host *mmc, __u16 task_id, unsigned long req_mask)
{
	struct mtk_blocktag *btag;
	int index;

	index = srcu_read_lock(&mmc_btag_readers);
	btag = srcu_dereference(mmc_mtk_btag, &mmc_btag_readers);
	if (btag)
		mmc_mtk_biolog_transfer_req_compl_owned(btag, mmc, task_id, req_mask);
	srcu_read_unlock(&mmc_btag_readers, index);
}
EXPORT_SYMBOL_GPL(mmc_mtk_biolog_transfer_req_compl);

/* evaluate throughput and workload of given context */
static void mmc_mtk_bio_context_eval(struct mmc_mtk_bio_context *ctx)
{
	uint64_t period;

	ctx->workload.usage = ctx->period_usage;

	if (ctx->workload.period > (ctx->workload.usage * 100)) {
		ctx->workload.percent = 1;
	} else {
		period = ctx->workload.period;
		do_div(period, 100);
		ctx->workload.percent =
			(__u32)ctx->workload.usage / (__u32)period;
	}
	ctx->last_workload_percent = ctx->workload.percent;
	mtk_btag_throughput_eval(ctx->throughput);
}

/* print context to trace ring buffer */
static struct mtk_btag_trace *mmc_mtk_bio_print_trace(
	struct mtk_blocktag *btag, struct mmc_mtk_bio_context *ctx)
{
	struct mtk_btag_ringtrace *rt = BTAG_RT(btag);
	struct mtk_btag_trace *tr;
	unsigned long flags;

	if (!rt)
		return NULL;

	spin_lock_irqsave(&rt->lock, flags);
	tr = mtk_btag_curr_trace(rt);

	if (!tr)
		goto out;

	memset(tr, 0, sizeof(struct mtk_btag_trace));
	tr->pid = ctx->pid;
	tr->qid = ctx->qid;
	mtk_btag_pidlog_eval(&tr->pidlog, &ctx->pidlog);
	mtk_btag_vmstat_eval(&tr->vmstat);
	mtk_btag_cpu_eval(&tr->cpu);
	memcpy(tr->throughput, ctx->throughput,
	       sizeof(struct mtk_btag_throughput) * BTAG_IO_TYPE_NR);
	memcpy(&tr->workload, &ctx->workload, sizeof(struct mtk_btag_workload));

	tr->time = sched_clock();
	mtk_btag_next_trace(rt);
out:
	spin_unlock_irqrestore(&rt->lock, flags);
	return tr;
}

static void mmc_mtk_bio_ctx_count_usage(struct mmc_mtk_bio_context *ctx,
	__u64 start, __u64 end)
{
	__u64 busy_in_period;

	if (start < ctx->period_start_t)
		busy_in_period = end - ctx->period_start_t;
	else
		busy_in_period = end - start;

	ctx->period_usage += busy_in_period;
}

/* CPU frequency adjust relate */




void set_mmc_perf_mode(struct mmc_host *mmc, bool boost)
{
	struct mmc_perf_owner *owner;
	int index;

	index = srcu_read_lock(&mmc_btag_readers);
	owner = srcu_dereference(mmc_perf_active, &mmc_btag_readers);
	if (owner && owner->mmc == mmc)
		mmc_perf_queue(owner, boost, false);
	srcu_read_unlock(&mmc_btag_readers, index);
}
EXPORT_SYMBOL_GPL(set_mmc_perf_mode);

/* Check requests after set/clear mask. */
static void mmc_mtk_biolog_check_owned(struct mtk_blocktag *btag, struct mmc_host *mmc, unsigned long req_mask)
{
	struct mmc_mtk_bio_context *ctx;
	__u64 end_time, period_time;
	unsigned long flags;
	bool is_sd;

	if (!mmc)
		return;
	if (!(mmc->caps2 & MMC_CAP2_NO_MMC))
		is_sd = false;
	else if (!(mmc->caps2 & MMC_CAP2_NO_SD))
		is_sd = true;
	else
		return;

	ctx = mmc_mtk_bio_curr_ctx(btag, is_sd);
	if (!ctx)
		return;

	end_time = sched_clock();

	/* when io loading is heavy,enable mmc performance mode */
	if (READ_ONCE(ctx->last_workload_percent) >= 90 && req_mask) {
		/* Completions may be in IRQ context; queue the policy/ICC work. */
		/* emmc not boost in set_mmc_perf_mode */
		set_mmc_perf_mode(mmc, true);
	}

	spin_lock_irqsave(&ctx->lock, flags);

	if (ctx->busy_start_t)
		mmc_mtk_bio_ctx_count_usage(ctx, ctx->busy_start_t, end_time);

	ctx->busy_start_t = (req_mask) ? end_time : 0;

	period_time = end_time - ctx->period_start_t;

	if (period_time >= MMC_MTK_BIO_TRACE_LATENCY) {
		ctx->period_end_t = end_time;
		ctx->workload.period = period_time;
		mmc_mtk_bio_context_eval(ctx);
		mmc_mtk_bio_print_trace(btag, ctx);
		ctx->period_start_t = end_time;
		ctx->period_end_t = 0;
		ctx->period_usage = 0;
		memset(ctx->throughput, 0,
		       sizeof(struct mtk_btag_throughput) * BTAG_IO_TYPE_NR);
		memset(&ctx->workload, 0, sizeof(struct mtk_btag_workload));
	}
	spin_unlock_irqrestore(&ctx->lock, flags);
}

void mmc_mtk_biolog_check(struct mmc_host *mmc, unsigned long req_mask)
{
	struct mtk_blocktag *btag;
	int index;

	index = srcu_read_lock(&mmc_btag_readers);
	btag = srcu_dereference(mmc_mtk_btag, &mmc_btag_readers);
	if (btag)
		mmc_mtk_biolog_check_owned(btag, mmc, req_mask);
	srcu_read_unlock(&mmc_btag_readers, index);
}
EXPORT_SYMBOL_GPL(mmc_mtk_biolog_check);

static size_t mmc_mtk_bio_seq_debug_show_info(struct mtk_blocktag *btag,
					      char **buff, unsigned long *size,
					      struct seq_file *seq)
{
	int i;
	struct mmc_mtk_bio_context *ctx = BTAG_CTX(btag);

	if (!ctx)
		return 0;

	for (i = 0; i < MMC_BIOLOG_CONTEXTS; i++)	{
		if (ctx[i].pid == 0)
			continue;
		BTAG_PRINTF(buff, size, seq,
			    "ctx[%d]=ctx_map[%d],pid:%4d,q:%d\n",
			    i,
			    ctx[i].id,
			    ctx[i].pid,
			    ctx[i].qid);
	}

	return 0;
}

static void mmc_mtk_bio_init_ctx(struct mmc_mtk_bio_context *ctx)
{
	int i;

	for (i = 0; i < MMC_BIOLOG_CONTEXTS; i++) {
		memset(ctx + i, 0, sizeof(struct mmc_mtk_bio_context));
		spin_lock_init(&(ctx + i)->lock);
		spin_lock_init(&(ctx + i)->pidlog.lock);
		(ctx + i)->period_start_t = sched_clock();
		(ctx + i)->qid = i;
	}
}

static struct mtk_btag_vops mmc_mtk_btag_vops = {
	.seq_show       = mmc_mtk_bio_seq_debug_show_info,
};

int mmc_mtk_biolog_init(struct mmc_host *mmc)
{
	struct mtk_blocktag *btag;
	struct mmc_perf_owner *perf;
	int ret = 0;

	if (!mmc)
		return -EINVAL;
	mutex_lock(&mmc_btag_owner_lock);
	if (rcu_access_pointer(mmc_mtk_btag)) {
		ret = -EEXIST;
		goto out;
	}
	btag = mtk_btag_alloc("mmc", BTAG_STORAGE_MMC, MMC_BIOLOG_RINGBUF_MAX,
		sizeof(struct mmc_mtk_bio_context), MMC_BIOLOG_CONTEXTS, &mmc_mtk_btag_vops);
	if (IS_ERR_OR_NULL(btag)) {
		ret = IS_ERR(btag) ? PTR_ERR(btag) : -ENOMEM;
		goto out;
	}
	/* Complete both context locks and counters before IRQ-side publication. */
	mmc_mtk_bio_init_ctx(BTAG_CTX(btag));
	perf = mmc_perf_prepare(mmc);
	if (IS_ERR(perf)) {
		ret = PTR_ERR(perf);
		mtk_btag_free(btag);
		goto out;
	}
	rcu_assign_pointer(mmc_perf_active, perf);
	rcu_assign_pointer(mmc_mtk_btag, btag);
out:
	mutex_unlock(&mmc_btag_owner_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mmc_mtk_biolog_init);

int mmc_mtk_biolog_exit(void)
{
	struct mtk_blocktag *btag;
	struct mmc_perf_owner *perf;

	mutex_lock(&mmc_btag_owner_lock);
	btag = rcu_dereference_protected(mmc_mtk_btag, lockdep_is_held(&mmc_btag_owner_lock));
	perf = rcu_dereference_protected(mmc_perf_active, lockdep_is_held(&mmc_btag_owner_lock));
	rcu_assign_pointer(mmc_mtk_btag, NULL);
	rcu_assign_pointer(mmc_perf_active, NULL);
	mmc_perf_close(perf);
	if (btag || perf)
		synchronize_srcu(&mmc_btag_readers);
	mmc_perf_destroy(perf);
	if (btag)
		mtk_btag_free(btag);
	mutex_unlock(&mmc_btag_owner_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mmc_mtk_biolog_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mediatek MMC Block IO Tracer");
