// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "vdec_fmt_pm.h"
#include "vdec_fmt_dmabuf.h"
#include "vdec_fmt_utils.h"

#include <linux/clk.h>
#include <linux/interconnect.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <soc/mediatek/smi.h>
#include "mtk-interconnect.h"
#include <linux/timekeeping.h>

void fmt_get_module_clock_by_name(struct mtk_vdec_fmt *fmt,
	const char *clkName, struct clk **clk_module)
{
	*clk_module = of_clk_get_by_name(fmt->dev->of_node, clkName);
	if (IS_ERR(*clk_module)) {
		fmt_err("cannot get module clock:%s", clkName);
		*clk_module = NULL;
	} else
		fmt_debug(0, "get module clock:%s", clkName);
}

/* Common Clock Framework */
void fmt_init_pm(struct mtk_vdec_fmt *fmt)
{
	fmt_debug(0, "+");
	fmt_get_module_clock_by_name(fmt, "MT_CG_VDEC",
		&fmt->clk_VDEC);
	fmt_get_module_clock_by_name(fmt, "MT_CG_MINI_MDP",
		&fmt->clk_MINI_MDP);
	pm_runtime_enable(fmt->dev);
}

int32_t fmt_clock_on(struct mtk_vdec_fmt *fmt)
{
 int ret;
 cmdq_mbox_enable(fmt->clt_fmt[0]->chan);
 ret = pm_runtime_resume_and_get(fmt->fmtLarb);
 if (ret < 0) goto mailbox;
 ret = clk_prepare_enable(fmt->clk_VDEC);
 if (ret) goto larb;
 ret = clk_prepare_enable(fmt->clk_MINI_MDP);
 if (ret) goto vdec;
 cmdq_util_prebuilt_init(CMDQ_PREBUILT_VFMT);
 return 0;
vdec: clk_disable_unprepare(fmt->clk_VDEC);
larb: pm_runtime_put_sync(fmt->fmtLarb);
mailbox: cmdq_mbox_disable(fmt->clt_fmt[0]->chan);
 return ret;
}

int32_t fmt_clock_off(struct mtk_vdec_fmt *fmt)
{
	if (fmt->clk_MINI_MDP)
		clk_disable_unprepare(fmt->clk_MINI_MDP);
	if (fmt->clk_VDEC)
		clk_disable_unprepare(fmt->clk_VDEC);
	if (fmt->fmtLarb)
		pm_runtime_put_sync(fmt->fmtLarb);
	cmdq_mbox_disable(fmt->clt_fmt[0]->chan);
	atomic_set(&fmt->fmt_error, 0);
	return 0;
}

void fmt_native_unprepare_qos(struct mtk_vdec_fmt *fmt)
{
 int i;
 for (i = 0; i < FMT_PORT_NUM; i++) {
  if (!IS_ERR_OR_NULL(fmt->fmt_qos_req[i])) icc_put(fmt->fmt_qos_req[i]);
  fmt->fmt_qos_req[i] = NULL;
 }
 if (fmt->native_opp) dev_pm_opp_of_remove_table(fmt->dev);
 fmt->native_opp = false;
}
int fmt_native_prepare_qos(struct mtk_vdec_fmt *fmt)
{
 static const char *names[FMT_PORT_NUM] = {"path_mini_mdp_r0", "path_mini_mdp_r1", "path_mini_mdp_w0", "path_mini_mdp_w1"};
 struct dev_pm_opp *opp;
 unsigned long freq = 0;
 int i, ret = dev_pm_opp_of_add_table(fmt->dev);
 if (ret) return ret;
 fmt->native_opp = true;
 ret = dev_pm_opp_get_opp_count(fmt->dev);
 if (ret <= 0 || ret > MAX_FREQ_STEP) {ret = -EINVAL; goto fail;}
 fmt->fmt_freq_cnt = ret;
 for (i = 0; i < fmt->fmt_freq_cnt; i++) {
  opp = dev_pm_opp_find_freq_ceil(fmt->dev, &freq);
  if (IS_ERR(opp)) {ret = PTR_ERR(opp); goto fail;}
  fmt->fmt_freqs[i] = freq; dev_pm_opp_put(opp); freq++;
 }
 fmt->fmt_reg = devm_regulator_get_optional(fmt->dev, "mmdvfs-dvfsrc-vcore");
 if (IS_ERR(fmt->fmt_reg)) {ret = PTR_ERR(fmt->fmt_reg); goto fail;}
 for (i = 0; i < FMT_PORT_NUM; i++) {
  fmt->fmt_qos_req[i] = of_mtk_icc_get(fmt->dev, names[i]);
  if (IS_ERR_OR_NULL(fmt->fmt_qos_req[i])) {ret = fmt->fmt_qos_req[i] ? PTR_ERR(fmt->fmt_qos_req[i]) : -ENODEV; goto fail;}
 }
 return 0;
fail: fmt_native_unprepare_qos(fmt); return ret;
}
void fmt_prepare_dvfs_emi_bw(struct mtk_vdec_fmt *fmt) {fmt_native_prepare_qos(fmt);}
void fmt_unprepare_dvfs_emi_bw(void) {}

void fmt_start_dvfs_emi_bw(struct mtk_vdec_fmt *fmt, struct fmt_pmqos pmqos_param, int id)
{
	struct dev_pm_opp *opp = 0;
	int volt = 0;
	int ret = 0;
	unsigned long request_freq;
	struct timespec64 curr_time;
	s32 duration;
	u32 bandwidth;

	fmt_debug(1, "tv_sec %d tv_usec %d pixel_size %d rdma_datasize %d wdma_datasize %d",
			pmqos_param.tv_sec,
			pmqos_param.tv_usec,
			pmqos_param.pixel_size,
			pmqos_param.rdma_datasize,
			pmqos_param.wdma_datasize);

	ktime_get_real_ts64(&curr_time);
	fmt_debug(1, "curr time tv_sec %lld tv_nsec %ld", curr_time.tv_sec, curr_time.tv_nsec);

	{
 s64 ms = ((s64)pmqos_param.tv_sec - curr_time.tv_sec) * 1000 +
          ((s64)pmqos_param.tv_usec * 1000 - curr_time.tv_nsec) / 1000000;
 duration = clamp_t(s64, ms, 1, 2000);
 }
	request_freq = (unsigned long)pmqos_param.pixel_size * 1000 / duration;

	fmt_debug(1, "request_freq %lu", request_freq);

	if (request_freq > fmt->fmt_freqs[fmt->fmt_freq_cnt-1]) {
		request_freq = fmt->fmt_freqs[fmt->fmt_freq_cnt-1];
		fmt_debug(1, "request_freq %lu limited by highest fmt_freq %lu",
					request_freq, fmt->fmt_freqs[fmt->fmt_freq_cnt-1]);
	}

	if (!IS_ERR_OR_NULL(fmt->fmt_reg)) {
		opp = dev_pm_opp_find_freq_ceil(fmt->dev,
					&request_freq);
		fmt_debug(1, "actual request freq %lu", request_freq);
		if (IS_ERR(opp)) return;
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		ret = regulator_set_voltage(fmt->fmt_reg, volt, INT_MAX);
		if (ret)
			fmt_debug(0, "Failed to set regulator voltage %d\n",
			volt);
	}
	fmt_debug(1, "rdma cal MMqos (%d, %d, %lu)",
			pmqos_param.rdma_datasize,
			pmqos_param.pixel_size,
			request_freq);
	if (id >= 0 && id < fmt->gce_th_num) {
		FMT_BANDWIDTH(pmqos_param.rdma_datasize, pmqos_param.pixel_size,
			request_freq, bandwidth);
		if (fmt->fmt_qos_req[id] != 0) {
			mtk_icc_set_bw(fmt->fmt_qos_req[id],
			MBps_to_icc(bandwidth), 0);
		}
		fmt_debug(1, "rdma bandwidth %d", bandwidth);
		fmt_debug(1, "wdma cal MMqos (%d, %d, %lu)",
			pmqos_param.wdma_datasize,
			pmqos_param.pixel_size,
			request_freq);
		FMT_BANDWIDTH(pmqos_param.wdma_datasize, pmqos_param.pixel_size,
			request_freq, bandwidth);
		if (fmt->fmt_qos_req[id+2] != 0) {
			mtk_icc_set_bw(fmt->fmt_qos_req[id+2],
			MBps_to_icc(bandwidth), 0);
		}
		fmt_debug(1, "wdma bandwidth %d", bandwidth);
	}
}

void fmt_end_dvfs_emi_bw(struct mtk_vdec_fmt *fmt, int id)
{
	struct dev_pm_opp *opp = 0;
	int volt = 0;
	int ret = 0;

	if (!IS_ERR_OR_NULL(fmt->fmt_reg)) {
		fmt_debug(1, "request freq %lu", fmt->fmt_freqs[0]);

		opp = dev_pm_opp_find_freq_ceil(fmt->dev,
					&fmt->fmt_freqs[0]);
		if (IS_ERR(opp)) return;
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		ret = regulator_set_voltage(fmt->fmt_reg, volt, INT_MAX);
		if (ret) {
			fmt_debug(0, "Failed to set regulator voltage %d\n",
			volt);
		}
	}
	if (id >= 0 && id < fmt->gce_th_num) {
		if (fmt->fmt_qos_req[id] != 0) {
			mtk_icc_set_bw(fmt->fmt_qos_req[id],
				MBps_to_icc(0), 0);
		}
		if (fmt->fmt_qos_req[id+2] != 0) {
			mtk_icc_set_bw(fmt->fmt_qos_req[id+2],
				MBps_to_icc(0), 0);
		}
	}
}

