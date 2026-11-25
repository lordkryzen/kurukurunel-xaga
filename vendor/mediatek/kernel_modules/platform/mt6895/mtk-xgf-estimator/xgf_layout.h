/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _XGF_LAYOUT_H_
#define _XGF_LAYOUT_H_

#include <linux/build_bug.h>
#include <linux/stddef.h>
#include "xgf.h"

/* Shared records consumed by the pinned upstream estimator object. */
static_assert(sizeof(struct xgf_render) == 1872);
static_assert(offsetof(struct xgf_render, hlist) == 0);
static_assert(offsetof(struct xgf_render, parent) == 16);
static_assert(offsetof(struct xgf_render, render) == 20);
static_assert(offsetof(struct xgf_render, bufID) == 24);
static_assert(offsetof(struct xgf_render, sector_head) == 32);
static_assert(offsetof(struct xgf_render, sector_nr) == 40);
static_assert(offsetof(struct xgf_render, hw_head) == 48);
static_assert(offsetof(struct xgf_render, prev_index) == 56);
static_assert(offsetof(struct xgf_render, prev_ts) == 64);
static_assert(offsetof(struct xgf_render, curr_index) == 72);
static_assert(offsetof(struct xgf_render, curr_ts) == 80);
static_assert(offsetof(struct xgf_render, event_count) == 88);
static_assert(offsetof(struct xgf_render, frame_count) == 92);
static_assert(offsetof(struct xgf_render, u_wake_r) == 96);
static_assert(offsetof(struct xgf_render, u_wake_r_count) == 100);
static_assert(offsetof(struct xgf_render, deps_list) == 104);
static_assert(offsetof(struct xgf_render, out_deps_list) == 112);
static_assert(offsetof(struct xgf_render, prev_deps_list) == 120);
static_assert(offsetof(struct xgf_render, deque) == 128);
static_assert(offsetof(struct xgf_render, queue) == 160);
static_assert(offsetof(struct xgf_render, ema_runtime) == 192);
static_assert(offsetof(struct xgf_render, pre_u_runtime) == 200);
static_assert(offsetof(struct xgf_render, u_runtime) == 208);
static_assert(offsetof(struct xgf_render, u_avg_runtime) == 1808);
static_assert(offsetof(struct xgf_render, u_runtime_sd) == 1816);
static_assert(offsetof(struct xgf_render, u_runtime_idx) == 1824);
static_assert(offsetof(struct xgf_render, spid) == 1828);
static_assert(offsetof(struct xgf_render, dep_frames) == 1832);
static_assert(offsetof(struct xgf_render, raw_l_runtime) == 1840);
static_assert(offsetof(struct xgf_render, raw_r_runtime) == 1848);
static_assert(offsetof(struct xgf_render, hwui_flag) == 1856);
static_assert(offsetof(struct xgf_render, ema2_pt) == 1864);

static_assert(sizeof(struct xgf_trace_event) == 24);
static_assert(offsetof(struct xgf_trace_event, ts) == 0);
static_assert(offsetof(struct xgf_trace_event, event) == 8);
static_assert(offsetof(struct xgf_trace_event, cpu) == 12);

static_assert(sizeof(struct fstb_trace_event) == 32);
static_assert(offsetof(struct fstb_trace_event, ts) == 0);
static_assert(offsetof(struct fstb_trace_event, event) == 8);
static_assert(offsetof(struct fstb_trace_event, cpu) == 12);
static_assert(offsetof(struct fstb_trace_event, note) == 16);
static_assert(offsetof(struct fstb_trace_event, state) == 20);
static_assert(offsetof(struct fstb_trace_event, pid) == 24);

static_assert(sizeof(struct xgf_ema2_predictor) == 680);
static_assert(offsetof(struct xgf_ema2_predictor, learn_rate) == 0);
static_assert(offsetof(struct xgf_ema2_predictor, beta) == 8);
static_assert(offsetof(struct xgf_ema2_predictor, epsilon) == 16);
static_assert(offsetof(struct xgf_ema2_predictor, alpha) == 24);
static_assert(offsetof(struct xgf_ema2_predictor, mu) == 32);
static_assert(offsetof(struct xgf_ema2_predictor, one) == 40);
static_assert(offsetof(struct xgf_ema2_predictor, L) == 48);
static_assert(offsetof(struct xgf_ema2_predictor, W) == 112);
static_assert(offsetof(struct xgf_ema2_predictor, RMSProp) == 176);
static_assert(offsetof(struct xgf_ema2_predictor, nabla) == 240);
static_assert(offsetof(struct xgf_ema2_predictor, rho) == 304);
static_assert(offsetof(struct xgf_ema2_predictor, x_record) == 368);
static_assert(offsetof(struct xgf_ema2_predictor, acc_x) == 432);
static_assert(offsetof(struct xgf_ema2_predictor, acc_xx) == 440);
static_assert(offsetof(struct xgf_ema2_predictor, x_front) == 512);
static_assert(offsetof(struct xgf_ema2_predictor, xt_last) == 576);
static_assert(offsetof(struct xgf_ema2_predictor, xt_last_valid) == 584);
static_assert(offsetof(struct xgf_ema2_predictor, record_idx) == 592);
static_assert(offsetof(struct xgf_ema2_predictor, ar_error_diff) == 600);
static_assert(offsetof(struct xgf_ema2_predictor, ar_coeff_sum) == 608);
static_assert(offsetof(struct xgf_ema2_predictor, ar_coeff_enable) == 616);
static_assert(offsetof(struct xgf_ema2_predictor, ar_coeff_frames) == 620);
static_assert(offsetof(struct xgf_ema2_predictor, acc_idx) == 624);
static_assert(offsetof(struct xgf_ema2_predictor, coeff_shift) == 628);
static_assert(offsetof(struct xgf_ema2_predictor, order) == 632);
static_assert(offsetof(struct xgf_ema2_predictor, t) == 640);
static_assert(offsetof(struct xgf_ema2_predictor, ar_coeff_valid) == 648);
static_assert(offsetof(struct xgf_ema2_predictor, invalid_th) == 652);
static_assert(offsetof(struct xgf_ema2_predictor, invalid_input_cnt) == 656);
static_assert(offsetof(struct xgf_ema2_predictor, invalid_negative_output_cnt) == 660);
static_assert(offsetof(struct xgf_ema2_predictor, skip_grad_update_cnt) == 664);
static_assert(offsetof(struct xgf_ema2_predictor, rmsprop_initialized) == 668);
static_assert(offsetof(struct xgf_ema2_predictor, x_record_initialized) == 669);
static_assert(offsetof(struct xgf_ema2_predictor, err_code) == 672);

static_assert(sizeof(struct xgf_sub_sect) == 32);
static_assert(offsetof(struct xgf_sub_sect, hlist) == 0);
static_assert(offsetof(struct xgf_sub_sect, start_ts) == 16);
static_assert(offsetof(struct xgf_sub_sect, end_ts) == 24);

static_assert(sizeof(struct xgf_dep) == 40);
static_assert(offsetof(struct xgf_dep, rb_node) == 0);
static_assert(offsetof(struct xgf_dep, tid) == 24);
static_assert(offsetof(struct xgf_dep, render_dep) == 28);
static_assert(offsetof(struct xgf_dep, frame_idx) == 32);

static_assert(sizeof(struct rb_node) == 24);
static_assert(offsetof(struct rb_node, __rb_parent_color) == 0);
static_assert(offsetof(struct rb_node, rb_right) == 8);
static_assert(offsetof(struct rb_node, rb_left) == 16);

static_assert(sizeof(struct hlist_node) == 16);
static_assert(offsetof(struct hlist_node, next) == 0);
static_assert(offsetof(struct hlist_node, pprev) == 8);

#endif
