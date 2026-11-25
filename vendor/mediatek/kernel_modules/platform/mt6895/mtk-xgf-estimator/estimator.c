// SPDX-License-Identifier: LicenseRef-MediaTek-Proprietary
/* Typed 6.12 boundary for the retained native XGF arithmetic engine.
 * The prebuilt algorithm is linked as data/code, never loaded as an old module.
 * All published functions are compiled with the current CFI and SCS settings.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/build_bug.h>
#include "xgf.h"
#include "xgf_layout.h"

extern int xgf_blob_xgf_ko_init(void);
extern int xgf_blob_xgf_est_runtime(pid_t, struct xgf_render *, u64 *, u64);
extern int xgf_blob_xgff_est_runtime(pid_t, struct xgf_render *, u64 *, u64);
extern int xgf_blob_xgff_update_start_prev_index(struct xgf_render *);
extern int xgf_blob_fpsgo_xgf2ko_calculate_target_fps(int, u64, int *, u64, u64, int);
extern void xgf_blob_fpsgo_xgf2ko_do_recycle(int, u64);
extern s64 xgf_blob_xgf_ema2_predict(struct xgf_ema2_predictor *, s64);
extern void xgf_blob_xgf_ema2_init(struct xgf_ema2_predictor *);
extern int fpsgo_is_enable(void);
extern int fpsgo_xgf_trace_status(void);

/* Only two audited allocation sites read these opaque cache slots. Their
 * sizes are explicit in x2; the adapter allocates through the current API.
 */
void *xgf_legacy_caches[11];
unsigned long xgf_legacy_canary;
static bool native_ready, native_quarantined;
static int native_fault;
static atomic64_t runtime_calls = ATOMIC64_INIT(0);
static atomic64_t runtime_nonzero = ATOMIC64_INIT(0);
static atomic64_t runtime_errors = ATOMIC64_INIT(0);
module_param(native_ready, bool, 0444);
module_param(native_quarantined, bool, 0444);
module_param(native_fault, int, 0444);

void *xgf_legacy_alloc(void *unused, unsigned int legacy_flags, size_t size)
{
	/* 5.10 0xdc0 = GFP_KERNEL | __GFP_ZERO; do not forward raw GFP bits. */
	if (legacy_flags != 0xdc0 || (size != 688 && size != 472))
		return NULL;
	return kzalloc(size, GFP_KERNEL);
}

static int runtime(pid_t pid, struct xgf_render *render, u64 *ns, u64 ts)
{
	int ret;

	if (!smp_load_acquire(&native_ready) || !render || !ns)
		return -ENODEV;
	atomic64_inc(&runtime_calls);
	ret = xgf_blob_xgf_est_runtime(pid, render, ns, ts);
	if (ret < XGF_NOTIFY_OK || ret > XGF_SLPTIME_OK)
		atomic64_inc(&runtime_errors);
	else if (*ns)
		atomic64_inc(&runtime_nonzero);
	return ret;
}
static int frame_runtime(pid_t pid, struct xgf_render *render, u64 *ns, u64 ts)
{
	return xgf_blob_xgff_est_runtime(pid, render, ns, ts);
}
static int update_index(struct xgf_render *render)
{
	return xgf_blob_xgff_update_start_prev_index(render);
}
static int target_fps(int pid, u64 id, int *margin, u64 deq, u64 queue, int active)
{
	return xgf_blob_fpsgo_xgf2ko_calculate_target_fps(pid, id, margin, deq, queue, active);
}
static void recycle(int pid, u64 id)
{
	xgf_blob_fpsgo_xgf2ko_do_recycle(pid, id);
}
static s64 predict(struct xgf_ema2_predictor *p, s64 x)
{
	return xgf_blob_xgf_ema2_predict(p, x);
}
static void predict_init(struct xgf_ema2_predictor *p)
{
	xgf_blob_xgf_ema2_init(p);
}
static int counters(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%lld %lld %lld\n",
		atomic64_read(&runtime_calls), atomic64_read(&runtime_nonzero),
		atomic64_read(&runtime_errors));
}
static const struct kernel_param_ops counter_ops = { .get = counters };
module_param_cb(runtime_counts, &counter_ops, NULL, 0444);

static int __init estimator_init(void)
{
	int ret;

	/* Layouts accessed by the native engine, checked against 6.12 headers. */
	static_assert(sizeof(struct rb_node) == 24);
	static_assert(sizeof(struct hlist_node) == 16);
	static_assert(sizeof(struct xgf_trace_event) == 24);
	static_assert(sizeof(struct fstb_trace_event) == 32);
	static_assert(offsetof(struct xgf_render, parent) == 0x10);
	static_assert(offsetof(struct xgf_render, render) == 0x14);
	static_assert(offsetof(struct xgf_render, curr_index) == 0x48);
	static_assert(offsetof(struct xgf_render, curr_ts) == 0x50);
	static_assert(offsetof(struct xgf_render, u_wake_r) == 0x60);
	if (!fpsgo_is_enable() || !xgf_event_index || !xgf_ko_enabled)
		return -EPROBE_DEFER;
	if (xgf_est_runtime_fp || xgff_est_runtime_fp ||
	    xgff_update_start_prev_index_fp || xgf_ema2_init_fp ||
	    xgf_ema2_predict_fp || fpsgo_xgf2ko_calculate_target_fps_fp ||
	    fpsgo_xgf2ko_do_recycle_fp || xgf_event_data || fstb_event_data)
		return -EBUSY;
	get_random_bytes(&xgf_legacy_canary, sizeof(xgf_legacy_canary));
	/* Buffers and callbacks retain this owner for the rest of this boot. */
	__module_get(THIS_MODULE);
	ret = xgf_blob_xgf_ko_init();
	if (ret || !xgf_event_data || !fstb_event_data ||
	    xgf_max_events != 32768 || fstb_event_buffer_size != 10000) {
		native_fault = ret ? ret : -EINVAL;
		native_quarantined = true;
		return 0;
	}
	WRITE_ONCE(xgf_est_runtime_fp, runtime);
	WRITE_ONCE(xgff_est_runtime_fp, frame_runtime);
	WRITE_ONCE(xgff_update_start_prev_index_fp, update_index);
	WRITE_ONCE(xgf_ema2_init_fp, predict_init);
	WRITE_ONCE(xgf_ema2_predict_fp, predict);
	WRITE_ONCE(fpsgo_xgf2ko_calculate_target_fps_fp, target_fps);
	WRITE_ONCE(fpsgo_xgf2ko_do_recycle_fp, recycle);
	smp_store_release(&native_ready, true);
	notify_xgf_ko_ready();
	ret = fpsgo_xgf_trace_status();
	if (ret) {
		native_fault = ret;
		native_quarantined = true;
		smp_store_release(&native_ready, false);
	}

	pr_info("XGF native estimator: typed callbacks published, owner pinned\n");
	return 0;
}
module_init(estimator_init);
MODULE_LICENSE("Proprietary");
MODULE_DESCRIPTION("Native XGF estimator with typed kernel compatibility boundary");
