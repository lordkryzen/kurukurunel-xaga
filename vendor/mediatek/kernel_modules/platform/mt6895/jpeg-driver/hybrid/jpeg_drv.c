// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/overflow.h>
#include <linux/iopoll.h>
#include <linux/sched/signal.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <soc/mediatek/mmdvfs_v3.h>
#include <soc/mediatek/smi.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <mtk_heap.h>

#include "jpeg_drv.h"
#include "jpeg_drv_reg.h"
#include "mtk-interconnect.h"
#include "mtk-smmu-v3.h"
#include "vcp_status.h"

#define JPEG_DEVNAME "mtk_jpeg"

#define JPEG_DEC_PROCESS 0x1

static struct JpegDeviceStruct gJpegqDev;
static atomic_t nodeCount;
static bool jpeg_native_ready;
static bool jpeg_native_faulted;
static bool root_test_only;
module_param(root_test_only, bool, 0444);
MODULE_PARM_DESC(root_test_only, "Restrict decoder test opens to CAP_SYS_ADMIN");
static struct proc_dir_entry *jpeg_native_proc;

static const struct of_device_id jdec_hybrid_of_ids[] = {
	{.compatible = "mediatek,jpgdec",},
	{.compatible = "mediatek,jpgdec_c1",},
	{}
};

/* hybrid decoder */
static wait_queue_head_t hybrid_dec_wait_queue[HW_CORE_NUMBER];
static DEFINE_MUTEX(jpeg_hybrid_dec_lock);

static bool dec_hwlocked[HW_CORE_NUMBER] = {false, false, false};
static bool dec_hw_enable[HW_CORE_NUMBER] = {false, false, false};
static unsigned int _jpeg_hybrid_dec_int_status[HW_CORE_NUMBER];
static unsigned int jpeg_index_entries[HW_CORE_NUMBER];
/* Read-only IRQ latch for diagnosing rejected native index completions. */
module_param_array_named(last_irq_status, _jpeg_hybrid_dec_int_status, uint, NULL, 0444);
static struct dmabuf_info bufInfo[HW_CORE_NUMBER];

static const char *jpegdec_cg_name[] = {
	"MT_CG_VENC_JPGDEC",
	"MT_CG_VENC_JPGDEC_C1",
	"MT_CG_VENC_JPGDEC_C2",
};

static const char *jpegdec_wdma_name[] = {
	"path_jpegdec0_wdma",
	"path_jpegdec1_wdma",
	"path_jpegdec2_wdma",
};

static const char *jpegdec_bsdma_name[] = {
	"path_jpegdec0_bsdma",
	"path_jpegdec1_bsdma",
	"path_jpegdec2_bsdma",
};

static const char *jpegdec_huff_offset_name[] = {
	"path_jpegdec0_huff_offset",
	"path_jpegdec1_huff_offset",
	"path_jpegdec2_huff_offset",
};

int jpg_dbg_level;
module_param(jpg_dbg_level, int, 0644);

int jpg_core_binding = -1;
module_param(jpg_core_binding, int, 0644);

static int _jpeg_isr_hybrid_dec_lisr(int id)
{
	unsigned int tmp = 0;

	if (dec_hwlocked[id] && dec_hw_enable[id]) {
		tmp = IMG_REG_READ(REG_JPGDEC_HYBRID_274(id));
		if (tmp) {
			_jpeg_hybrid_dec_int_status[id] = tmp;
			IMG_REG_WRITE(tmp, REG_JPGDEC_HYBRID_274(id));
			JPEG_LOG(1, "return 0");
			return 0;
		}
	}
	JPEG_LOG(1, "return -1");
	return -1;
}

static int jpeg_isr_hybrid_dec_lisr(int id)
{
	int ret = 0;

	mutex_lock(&jpeg_hybrid_dec_lock);
	ret = _jpeg_isr_hybrid_dec_lisr(id);
	mutex_unlock(&jpeg_hybrid_dec_lock);

	return ret;
}

static inline void jpeg_reg_write_mask(long addr, uint32_t mask, uint32_t val)
{
	uint32_t reg_tmp;

	reg_tmp = IMG_REG_READ(addr);
	reg_tmp = (reg_tmp & ~mask) | (val & mask);
	IMG_REG_WRITE(reg_tmp, addr);
}

static void jpeg_axdomain_set(unsigned int id)
{
	unsigned int larb_idx = gJpegqDev.larb_idx_map[id];

	if (!gJpegqDev.smiLarbBaseVA[larb_idx])
		return;

	JPEG_LOG(1, "[%d] set axdomain(%d) larb: %d(0x%lx) (%d, %d, %d)",
		    id,
		    gJpegqDev.axdomain[larb_idx],
		    larb_idx,
		    gJpegqDev.smiLarbBaseVA[larb_idx],
		    gJpegqDev.larb_port[id][JPEG_DEC_WDMA],
		    gJpegqDev.larb_port[id][JPEG_DEC_BSDMA],
		    gJpegqDev.larb_port[id][JPEG_DEC_HUFF_OFFSET]);

	jpeg_reg_write_mask(REG_JPGDEC_LARB_F00(gJpegqDev.smiLarbBaseVA[larb_idx],
						gJpegqDev.larb_port[id][JPEG_DEC_WDMA]),
			    0x1f0,
			    gJpegqDev.axdomain[larb_idx] << 4);
	jpeg_reg_write_mask(REG_JPGDEC_LARB_F80(gJpegqDev.smiLarbBaseVA[larb_idx],
						gJpegqDev.larb_port[id][JPEG_DEC_WDMA]),
			    0x1f0,
			    gJpegqDev.axdomain[larb_idx] << 4);
	jpeg_reg_write_mask(REG_JPGDEC_LARB_F00(gJpegqDev.smiLarbBaseVA[larb_idx],
						gJpegqDev.larb_port[id][JPEG_DEC_BSDMA]),
			    0x1f0,
			    gJpegqDev.axdomain[larb_idx] << 4);
	jpeg_reg_write_mask(REG_JPGDEC_LARB_F80(gJpegqDev.smiLarbBaseVA[larb_idx],
						gJpegqDev.larb_port[id][JPEG_DEC_BSDMA]),
			    0x1f0,
			    gJpegqDev.axdomain[larb_idx] << 4);
	jpeg_reg_write_mask(REG_JPGDEC_LARB_F00(gJpegqDev.smiLarbBaseVA[larb_idx],
						gJpegqDev.larb_port[id][JPEG_DEC_HUFF_OFFSET]),
			    0x1f0,
			    gJpegqDev.axdomain[larb_idx] << 4);
	jpeg_reg_write_mask(REG_JPGDEC_LARB_F80(gJpegqDev.smiLarbBaseVA[larb_idx],
						gJpegqDev.larb_port[id][JPEG_DEC_HUFF_OFFSET]),
			    0x1f0,
			    gJpegqDev.axdomain[larb_idx] << 4);
}

/* Match the native JPGDEC producer limit, including the high address bits.
 * The reset sentinel is a 32-bit pointer and cannot bound an above-4GiB DMA map. */
static int jpeg_native_bs_end(unsigned int capacity, unsigned int bytes,
		size_t mapped_bytes, u64 base, u64 *end)
{
	u64 limit;

	if (!capacity || !bytes || bytes > capacity || capacity > mapped_bytes ||
	    !IS_ALIGNED(capacity, 128) || !IS_ALIGNED(base, 16))
		return -EINVAL;
	if (check_add_overflow(base, (u64)capacity, &limit) || limit >= (1ULL << 34))
		return -ERANGE;
	*end = limit;
	return 0;
}

/* Native ALPHA: ceil(MCUs-per-row / 16) records per MCU row,
 * 16 bytes per published record, allocation rounded to 128 bytes.
 * Mode 3 builds this index; native 5.10 also emits no pixel-decode EOF IRQ. */
static int jpeg_native_index_count(const unsigned int data[], unsigned int *entries)
{
	u64 mcus = (u64)data[9] + 1, width = data[16], count, size;

	*entries = 0;
	if (data[15] != 3)
		return 0;
	if (!width || data[17] != 16 || mcus % width)
		return -EINVAL;
	count = DIV_ROUND_UP_ULL(width, 16) * (mcus / width);
	size = ALIGN(count * 16, 128);
	if (!count || count > UINT_MAX || size != data[20])
		return -EINVAL;
	*entries = count;
	return 0;
}

static int jpeg_drv_hybrid_dec_start(unsigned int data[],
				     unsigned int id,
				     int *index_buf_fd)
{
	u64 ibuf_iova, obuf_iova, ibuf_end;
	int ret;
	void *ptr;
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(NULL);
	unsigned int node_id, entries;

	JPEG_LOG(1, "+ id:%d", id);
	ret = 0;
	ibuf_iova = 0;
	obuf_iova = 0;
	node_id = gJpegqDev.larb_idx_map[id];

	if (!data[20] || data[20] > INT_MAX / 2) return -EINVAL;
	ret = jpeg_native_index_count(data, &entries);
	if (ret) return ret;
	mutex_lock(&jpeg_hybrid_dec_lock);
	bufInfo[id].o_dbuf = jpg_dmabuf_alloc(data[20], 128, 0);
	bufInfo[id].o_attach = NULL;
	bufInfo[id].o_sgt = NULL;
	if (bufInfo[id].o_dbuf)
		mtk_dma_buf_set_name(bufInfo[id].o_dbuf, "jpg_dec_o_buf");

	bufInfo[id].i_dbuf = jpg_dmabuf_get(data[7]);
	bufInfo[id].i_attach = NULL;
	bufInfo[id].i_sgt = NULL;
	if (bufInfo[id].i_dbuf)
		mtk_dma_buf_set_name(bufInfo[id].i_dbuf, "jpg_dec_i_buf");

	if (!bufInfo[id].o_dbuf) {
		mutex_unlock(&jpeg_hybrid_dec_lock);
		JPEG_LOG(0, "o_dbuf alloc failed");
		return -1;
	}

	if (!bufInfo[id].i_dbuf) {
		mutex_unlock(&jpeg_hybrid_dec_lock);
		JPEG_LOG(0, "i_dbuf null error");
		return -1;
	}

	ret = jpg_dmabuf_get_iova(bufInfo[id].o_dbuf, &obuf_iova, gJpegqDev.smmu_dev[node_id],
	&bufInfo[id].o_attach, &bufInfo[id].o_sgt);
	JPEG_LOG(1, "obuf_iova:0x%llx lsb:0x%lx msb:0x%lx", obuf_iova,
		(unsigned long)(unsigned char *)obuf_iova,
		(unsigned long)(unsigned char *)(obuf_iova>>32));

	if (ret != 0) {
		mutex_unlock(&jpeg_hybrid_dec_lock);
		JPEG_LOG(0, "get obuf_iova fail");
		return ret;
	}

	ret = jpg_dmabuf_vmap(bufInfo[id].o_dbuf, &map);
	ptr = map.vaddr;
	if (ret) { mutex_unlock(&jpeg_hybrid_dec_lock); return ret; }
	if (ptr && data[20]) memset(ptr, 0, data[20]);
	jpg_dmabuf_vunmap(bufInfo[id].o_dbuf, &map);

	ret = jpg_dmabuf_get_iova(bufInfo[id].i_dbuf, &ibuf_iova, gJpegqDev.smmu_dev[node_id],
	&bufInfo[id].i_attach, &bufInfo[id].i_sgt);
	JPEG_LOG(1, "ibuf_iova 0x%llx lsb:0x%lx msb:0x%lx", ibuf_iova,
		(unsigned long)(unsigned char *)ibuf_iova,
		(unsigned long)(unsigned char *)(ibuf_iova>>32));

	if (ret != 0) {
		mutex_unlock(&jpeg_hybrid_dec_lock);
		JPEG_LOG(0, "get iova fail i:0x%llx o:0x%llx", ibuf_iova, obuf_iova);
		return ret;
	}

	ret = jpeg_native_bs_end(data[12], data[18], bufInfo[id].i_dbuf->size,
		ibuf_iova, &ibuf_end);
	if (ret) { mutex_unlock(&jpeg_hybrid_dec_lock); return ret; }

	if (!dec_hwlocked[id] || !dec_hw_enable[id]) {
		mutex_unlock(&jpeg_hybrid_dec_lock);
		JPEG_LOG(0, "hw %d invalid(isLocked: %d, isEnabled: %d), start fail",
			 id, dec_hwlocked[id], dec_hw_enable[id]);
		return -1;
	}

	/* Reserve the number now, publish its file only after both user outputs
	 * have been copied. No unreachable installed fd on copyout failure. */
	*index_buf_fd = get_unused_fd_flags(O_CLOEXEC | O_RDWR);
	if (*index_buf_fd < 0) {
		mutex_unlock(&jpeg_hybrid_dec_lock);
		return *index_buf_fd;
	}


	jpeg_index_entries[id] = entries;
	jpeg_axdomain_set(id);

	IMG_REG_WRITE(data[0], REG_JPGDEC_HYBRID_090(id));
	IMG_REG_WRITE(data[1], REG_JPGDEC_HYBRID_090(id));
	IMG_REG_WRITE(data[2], REG_JPGDEC_HYBRID_0FC(id));
	IMG_REG_WRITE(data[3], REG_JPGDEC_HYBRID_14C(id));
	IMG_REG_WRITE(data[4], REG_JPGDEC_HYBRID_150(id));
	IMG_REG_WRITE(data[5], REG_JPGDEC_HYBRID_154(id));
	IMG_REG_WRITE(data[6], REG_JPGDEC_HYBRID_17C(id));
	IMG_REG_WRITE((unsigned long)(unsigned char *)ibuf_iova, REG_JPGDEC_HYBRID_200(id));
	IMG_REG_WRITE((unsigned long)(unsigned char *)(ibuf_iova>>32), REG_JPGDEC_HYBRID_378(id));
	IMG_REG_WRITE(lower_32_bits(ibuf_end), REG_JPGDEC_HYBRID_248(id));
	IMG_REG_WRITE(upper_32_bits(ibuf_end), REG_JPGDEC_HYBRID_37C(id));
	IMG_REG_WRITE(data[8], REG_JPGDEC_HYBRID_20C(id));
	IMG_REG_WRITE(data[9], REG_JPGDEC_HYBRID_210(id));
	IMG_REG_WRITE(data[10], REG_JPGDEC_HYBRID_224(id));
	IMG_REG_WRITE(data[11], REG_JPGDEC_HYBRID_23C(id));
	IMG_REG_WRITE(data[12], REG_JPGDEC_HYBRID_24C(id));
	IMG_REG_WRITE(data[13], REG_JPGDEC_HYBRID_270(id));
	IMG_REG_WRITE(data[14], REG_JPGDEC_HYBRID_31C(id));
	IMG_REG_WRITE(data[15], REG_JPGDEC_HYBRID_330(id));
	IMG_REG_WRITE(data[16], REG_JPGDEC_HYBRID_334(id));
	IMG_REG_WRITE((unsigned long)(unsigned char *)obuf_iova, REG_JPGDEC_HYBRID_338(id));
	IMG_REG_WRITE((unsigned long)(unsigned char *)(obuf_iova>>32), REG_JPGDEC_HYBRID_384(id));
	IMG_REG_WRITE((unsigned long)(unsigned char *)obuf_iova, REG_JPGDEC_HYBRID_36C(id));
	IMG_REG_WRITE((unsigned long)(unsigned char *)obuf_iova + data[20],
	REG_JPGDEC_HYBRID_370(id));
	IMG_REG_WRITE((unsigned long)(unsigned char *)obuf_iova + data[20]*2,
	REG_JPGDEC_HYBRID_374(id));
	IMG_REG_WRITE(data[17], REG_JPGDEC_HYBRID_33C(id));
	IMG_REG_WRITE(data[18], REG_JPGDEC_HYBRID_344(id));
	IMG_REG_WRITE(data[19], REG_JPGDEC_HYBRID_240(id));

	mutex_unlock(&jpeg_hybrid_dec_lock);

	JPEG_LOG(1, "-");
	return ret;
}

static void jpeg_drv_hybrid_dec_get_p_n_s(
								unsigned int id,
								int *progress_n_status)
{
	int progress, status;

	progress = IMG_REG_READ(REG_JPGDEC_HYBRID_340(id)) - 1;
	status = IMG_REG_READ(REG_JPGDEC_HYBRID_348(id));
	*progress_n_status = progress << 4 | status;
	JPEG_LOG(1, "progress_n_status %d", *progress_n_status);
}

static irqreturn_t jpeg_drv_hybrid_dec_isr(int irq, void *dev_id)
{
	int ret = 0;
	int i;

	JPEG_LOG(1, "JPEG Hybrid Decoder Interrupt %d", irq);
	for (i = 0 ; i < HW_CORE_NUMBER; i++) {
		if (irq == gJpegqDev.hybriddecIrqId[i]) {
			if (!dec_hwlocked[i] || !dec_hw_enable[i]) {
				JPEG_LOG(0,
					"JPEG isr from invalid HW %d(isLocked: %d, isEnabled: %d)",
					i, dec_hwlocked[i], dec_hw_enable[i]);
				return IRQ_HANDLED;
			}
			ret = _jpeg_isr_hybrid_dec_lisr(i);
			if (ret == 0)
				wake_up_interruptible(
				&(hybrid_dec_wait_queue[i]));
			JPEG_LOG(1, "JPEG Hybrid Dec clear Interrupt %d ret %d"
					, irq, ret);
			break;
		}
	}

	return IRQ_HANDLED;
}

int jpeg_drv_hybrid_dec_prepare_dvfs(void)
{
	int ret;
	struct dev_pm_opp *opp = 0;
	unsigned long freq = 0;
	int i = 0;
	struct of_phandle_args spec;

	ret = devm_pm_opp_of_add_table(gJpegqDev.pDev[0]);
	if (ret < 0) {
		JPEG_LOG(0, "Failed to get opp table (%d)", ret);
		return ret;
	}

	gJpegqDev.jpeg_reg = devm_regulator_get_optional(gJpegqDev.pDev[0],
						"mmdvfs-dvfsrc-vcore");
	if (IS_ERR(gJpegqDev.jpeg_reg) && PTR_ERR(gJpegqDev.jpeg_reg) == -EPROBE_DEFER) return -EPROBE_DEFER;
	if (IS_ERR_OR_NULL(gJpegqDev.jpeg_reg)) {
		JPEG_LOG(0, "Failed to get regulator");
		gJpegqDev.jpeg_reg = NULL;
		gJpegqDev.jpeg_dvfs = devm_clk_get(gJpegqDev.pDev[0], "mmdvfs_clk");
		if (IS_ERR_OR_NULL(gJpegqDev.jpeg_dvfs)) {
			JPEG_LOG(0, "Failed to get mmdvfs clk");
			ret = PTR_ERR(gJpegqDev.jpeg_dvfs);
			gJpegqDev.jpeg_dvfs = NULL;
			return ret ? ret : -ENODEV;
		}
	}

	gJpegqDev.jpeg_freq_cnt = dev_pm_opp_get_opp_count(gJpegqDev.pDev[0]);
	if (gJpegqDev.jpeg_freq_cnt <= 0 || gJpegqDev.jpeg_freq_cnt > MAX_FREQ_STEP) return -EINVAL;
	freq = 0;
	while (i < MAX_FREQ_STEP && !IS_ERR(opp =
		dev_pm_opp_find_freq_ceil(gJpegqDev.pDev[0], &freq))) {
		gJpegqDev.jpeg_freqs[i] = freq;
		freq++;
		i++;
		dev_pm_opp_put(opp);
	}

	ret = of_property_read_u32(gJpegqDev.pDev[0]->of_node,
					"dvfs-opp-level",
					&gJpegqDev.dvfs_opp_level);
	if (ret != 0 || gJpegqDev.dvfs_opp_level >= gJpegqDev.jpeg_freq_cnt) {
		gJpegqDev.dvfs_opp_level = gJpegqDev.jpeg_freq_cnt - 1;
	}
	JPEG_LOG(0, "dvfs opp level: %d", gJpegqDev.dvfs_opp_level);

	if (mmdvfs_get_version() >= MMDVFS_VER_V5) {
		i = of_property_match_string(gJpegqDev.pDev[0]->of_node,
						 "clock-names", "mmdvfs_clk");
		ret = of_parse_phandle_with_args(gJpegqDev.pDev[0]->of_node,
							"clocks", "#clock-cells", i, &spec);
		if (!ret) {
			gJpegqDev.mmdvfs_vcp_idx = spec.args[0];
			of_node_put(spec.np);
		}
	} else if (mmdvfs_get_version()) {
		gJpegqDev.mmdvfs_vcp_idx = VCP_PWR_USR_JPEGDEC;
	}
	JPEG_LOG(0, "mmdvfs_vcp_idx: %d", gJpegqDev.mmdvfs_vcp_idx);
	return 0;
}

void jpeg_drv_hybrid_dec_unprepare_dvfs(void)
{
}

void jpeg_drv_hybrid_dec_start_dvfs(void)
{
	struct dev_pm_opp *opp = 0;
	int volt = 0;
	int ret = 0;

	if (gJpegqDev.jpeg_reg) {
		JPEG_LOG(1, "request freq %lu",
				gJpegqDev.jpeg_freqs[gJpegqDev.dvfs_opp_level]);
		opp = dev_pm_opp_find_freq_ceil(gJpegqDev.pDev[0],
		&gJpegqDev.jpeg_freqs[gJpegqDev.dvfs_opp_level]);
		if (IS_ERR(opp)) return;
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		ret = regulator_set_voltage(gJpegqDev.jpeg_reg, volt, INT_MAX);
		if (ret) {
			JPEG_LOG(0, "Failed to set regulator voltage %d",
			volt);
		}
	} else if (gJpegqDev.jpeg_dvfs) {
		JPEG_LOG(1, "request freq %lu",
				gJpegqDev.jpeg_freqs[gJpegqDev.dvfs_opp_level]);
		if (mmdvfs_get_version())
			mtk_mmdvfs_enable_vcp(true, gJpegqDev.mmdvfs_vcp_idx);
		ret = clk_set_rate(gJpegqDev.jpeg_dvfs,
			gJpegqDev.jpeg_freqs[gJpegqDev.dvfs_opp_level]);
		if (ret) {
			JPEG_LOG(0, "Failed to set freq %lu",
			gJpegqDev.jpeg_freqs[gJpegqDev.dvfs_opp_level]);
		}
		if (mmdvfs_get_version())
			mtk_mmdvfs_enable_vcp(false, gJpegqDev.mmdvfs_vcp_idx);
	}

}

void jpeg_drv_hybrid_dec_end_dvfs(void)
{
	struct dev_pm_opp *opp = 0;
	int volt = 0;
	int ret = 0;

	if (gJpegqDev.jpeg_reg) {
		JPEG_LOG(1, "request freq %lu", gJpegqDev.jpeg_freqs[0]);
		opp = dev_pm_opp_find_freq_ceil(gJpegqDev.pDev[0],
					&gJpegqDev.jpeg_freqs[0]);
		if (IS_ERR(opp)) return;
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		ret = regulator_set_voltage(gJpegqDev.jpeg_reg, volt, INT_MAX);
		if (ret) {
			JPEG_LOG(0, "Failed to set regulator voltage %d",
			volt);
		}
	} else if (gJpegqDev.jpeg_dvfs) {
		JPEG_LOG(1, "request freq 0");
		if (mmdvfs_get_version())
			mtk_mmdvfs_enable_vcp(true, gJpegqDev.mmdvfs_vcp_idx);
		ret = clk_set_rate(gJpegqDev.jpeg_dvfs, 0);
		if (ret)
			JPEG_LOG(0, "Failed to set freq 0");
		if (mmdvfs_get_version())
			mtk_mmdvfs_enable_vcp(false, gJpegqDev.mmdvfs_vcp_idx);
	}
}

static void __maybe_unused jpeg_drv_prepare_qos_request(unsigned int id)
{
	unsigned int node_id = gJpegqDev.larb_idx_map[id];

	gJpegqDev.jpeg_path_wdma[id] = of_mtk_icc_get(gJpegqDev.pDev[node_id],
								jpegdec_wdma_name[id]);
	gJpegqDev.jpeg_path_bsdma[id] = of_mtk_icc_get(gJpegqDev.pDev[node_id],
								jpegdec_bsdma_name[id]);
	gJpegqDev.jpeg_path_huff_offset[id] = of_mtk_icc_get(gJpegqDev.pDev[node_id],
								jpegdec_huff_offset_name[id]);
	JPEG_LOG(0, "qos prepare(%p, %p, %p)",
		    gJpegqDev.jpeg_path_wdma[id],
		    gJpegqDev.jpeg_path_bsdma[id],
		    gJpegqDev.jpeg_path_huff_offset[id]);
}

static void jpeg_drv_update_qos_request(unsigned int id)
{
	JPEG_LOG(1, "update qos request id: %d", id);

	if (gJpegqDev.ven0BaseVA && gJpegqDev.is_qos_16_level) {
		IMG_REG_WRITE(0x60,
			      REG_JPGDEC_VENC_318(gJpegqDev.ven0BaseVA,
						  gJpegqDev.larb_port[id][JPEG_DEC_WDMA]));
		IMG_REG_WRITE(0x61,
			      REG_JPGDEC_VENC_318(gJpegqDev.ven0BaseVA,
						  gJpegqDev.larb_port[id][JPEG_DEC_BSDMA]));
		IMG_REG_WRITE(0x60,
			      REG_JPGDEC_VENC_318(gJpegqDev.ven0BaseVA,
						  gJpegqDev.larb_port[id][JPEG_DEC_HUFF_OFFSET]));
		IMG_REG_WRITE(0x90, (gJpegqDev.ven0BaseVA + 0x39c));
	}

	if (gJpegqDev.jpeg_path_wdma[id]) mtk_icc_set_bw(gJpegqDev.jpeg_path_wdma[id], MBps_to_icc(960), 0);
	if (gJpegqDev.jpeg_path_bsdma[id]) mtk_icc_set_bw(gJpegqDev.jpeg_path_bsdma[id], MBps_to_icc(576), 0);
	if (gJpegqDev.jpeg_path_huff_offset[id]) mtk_icc_set_bw(gJpegqDev.jpeg_path_huff_offset[id], MBps_to_icc(40), 0);
}

static void jpeg_drv_end_qos_request(unsigned int id)
{
	JPEG_LOG(1, "end qos request id: %d", id);

	if (gJpegqDev.jpeg_path_wdma[id]) mtk_icc_set_bw(gJpegqDev.jpeg_path_wdma[id], 0, 0);
	if (gJpegqDev.jpeg_path_bsdma[id]) mtk_icc_set_bw(gJpegqDev.jpeg_path_bsdma[id], 0, 0);
	if (gJpegqDev.jpeg_path_huff_offset[id]) mtk_icc_set_bw(gJpegqDev.jpeg_path_huff_offset[id], 0, 0);
}

int jpeg_drv_hybrid_dec_power_on(unsigned int id)
{
	int ret;
	unsigned int ven_res;
	int cnt = 0;

	if (gJpegqDev.first_larb_ref_cnt == 0) {
		if (gJpegqDev.jpegLarb[0]) {
			JPEG_LOG(1, "[%d] power on first larb", id);
			if (gJpegqDev.is_ccf_one_step)
				ret = mtk_smi_larb_enable(gJpegqDev.jpegLarb[0]);
			else
				ret = pm_runtime_resume_and_get(gJpegqDev.jpegLarb[0]);
			if (ret < 0) return ret;
		}

		if (gJpegqDev.ven0BaseVA) {
			IMG_REG_WRITE((BIT(0)), (gJpegqDev.ven0BaseVA + 0x10));
			ven_res = IMG_REG_READ((gJpegqDev.ven0BaseVA + 0x15C));
			while (!(ven_res & BIT(31))) {
				cnt++;
				if (cnt > 10000) {
					JPEG_LOG(0, "[E][%d] venc ddrsrc ack timeout 0x%08x",
						     id, ven_res);
					break;
				}
				udelay(10);
				ven_res = IMG_REG_READ((gJpegqDev.ven0BaseVA + 0x15C));
			}
		}

		jpeg_drv_hybrid_dec_start_dvfs();
	}
	gJpegqDev.first_larb_ref_cnt++;

	if (gJpegqDev.jpegLarb[1] && gJpegqDev.larb_idx_map[id] == 1) {
		JPEG_LOG(1, "[%d] power on second larb", id);
		if (gJpegqDev.is_ccf_one_step)
			ret = mtk_smi_larb_enable(gJpegqDev.jpegLarb[1]);
		else
			ret = pm_runtime_resume_and_get(gJpegqDev.jpegLarb[1]);
		if (ret < 0) goto first_off;
	}

	ret = clk_prepare_enable(gJpegqDev.jpegClk[id]);
	if (ret) {
		if (gJpegqDev.jpegLarb[1] && gJpegqDev.larb_idx_map[id] == 1) {
			if (gJpegqDev.is_ccf_one_step) mtk_smi_larb_disable(gJpegqDev.jpegLarb[1]);
			else pm_runtime_put(gJpegqDev.jpegLarb[1]);
		}
		goto first_off;
	}

	jpeg_drv_update_qos_request(id);

	JPEG_LOG(1, "[%d] JPEG Hybrid Decoder Power On", id);
 return 0;
first_off:
 gJpegqDev.first_larb_ref_cnt--;
 if (!gJpegqDev.first_larb_ref_cnt) {
  jpeg_drv_hybrid_dec_end_dvfs();
  if (gJpegqDev.jpegLarb[0]) {
   if (gJpegqDev.is_ccf_one_step) mtk_smi_larb_disable(gJpegqDev.jpegLarb[0]);
   else pm_runtime_put(gJpegqDev.jpegLarb[0]);
  }
 }
 return ret;
}

void jpeg_drv_hybrid_dec_power_off(unsigned int id)
{
	int ret;

	jpeg_drv_end_qos_request(id);

	clk_disable_unprepare(gJpegqDev.jpegClk[id]);

	if (gJpegqDev.jpegLarb[1] && gJpegqDev.larb_idx_map[id] == 1) {
		JPEG_LOG(1, "[%d] power off second larb", id);
		if (gJpegqDev.is_ccf_one_step)
			ret = mtk_smi_larb_disable(gJpegqDev.jpegLarb[1]);
		else
			ret = pm_runtime_put_sync(gJpegqDev.jpegLarb[1]);
		if (ret)
			JPEG_LOG(0, "[E][%d] second larb off failed %d", id, ret);
	}

	gJpegqDev.first_larb_ref_cnt--;
	if (gJpegqDev.jpegLarb[0] && gJpegqDev.first_larb_ref_cnt == 0) {
		jpeg_drv_hybrid_dec_end_dvfs();

		JPEG_LOG(1, "[%d] power off first larb", id);
		if (gJpegqDev.is_ccf_one_step)
			ret = mtk_smi_larb_disable(gJpegqDev.jpegLarb[0]);
		else
			ret = pm_runtime_put_sync(gJpegqDev.jpegLarb[0]);
		if (ret)
			JPEG_LOG(0, "[%d] first larb off failed %d", id, ret);
	}

	JPEG_LOG(1, "[%d] JPEG Hybrid Decoder Power off", id);
}

static int jpeg_drv_hybrid_dec_lock(int *hwid)
{
	int retValue = 0;
	int id = 0;
	int is_vcp_suspend = is_vcp_suspending_ex();

	mutex_lock(&jpeg_hybrid_dec_lock);

	if (READ_ONCE(jpeg_native_faulted) || gJpegqDev.is_suspending || gJpegqDev.is_shutdowning || is_vcp_suspend) {
		JPEG_LOG(0, "jpeg dec is suspending or shutdowning, %d, %d, %d",
			    gJpegqDev.is_suspending,
			    gJpegqDev.is_shutdowning,
				is_vcp_suspend);
		*hwid = -1;
		mutex_unlock(&jpeg_hybrid_dec_lock);
		return -EBUSY;
	}

	for (id = 0; id < HW_CORE_NUMBER; id++) {
		if (jpg_core_binding != -1 && jpg_core_binding != id) {
			JPEG_LOG(0, "jpeg skip HW core %d", id);
			continue;
		}

		if (dec_hwlocked[id] || !dec_hw_enable[id]) {
			JPEG_LOG(1,
				"jpeg dec HW core %d isn't available(isLocked: %d, isEnabled: %d)",
				 id, dec_hwlocked[id], dec_hw_enable[id]);
			continue;
		}

		*hwid = id;
		JPEG_LOG(1, "jpeg dec get %d HW core", id);
		_jpeg_hybrid_dec_int_status[id] = 0;
		jpeg_index_entries[id] = 0;
		retValue = jpeg_drv_hybrid_dec_power_on(id);
		if (retValue) { dec_hwlocked[id] = false; break; }
		dec_hwlocked[id] = true;
		enable_irq(gJpegqDev.hybriddecIrqId[id]);
		break;
	}

	mutex_unlock(&jpeg_hybrid_dec_lock);
	if (id == HW_CORE_NUMBER) {
		JPEG_LOG(1, "jpeg dec HW core all busy");
		*hwid = -1;
		retValue = -EBUSY;
	}

	return retValue;
}

static void _jpeg_drv_hybrid_dec_unlock(unsigned int hwid)
{
	disable_irq(gJpegqDev.hybriddecIrqId[hwid]);
	dec_hwlocked[hwid] = false;
	jpeg_drv_hybrid_dec_power_off(hwid);
	JPEG_LOG(1, "jpeg dec HW core %d is unlocked", hwid);
	jpg_dmabuf_free_iova(bufInfo[hwid].i_dbuf,
		bufInfo[hwid].i_attach,
		bufInfo[hwid].i_sgt);
	jpg_dmabuf_free_iova(bufInfo[hwid].o_dbuf,
		bufInfo[hwid].o_attach,
		bufInfo[hwid].o_sgt);
	jpg_dmabuf_put(bufInfo[hwid].i_dbuf);
	jpg_dmabuf_put(bufInfo[hwid].o_dbuf);
	memset(&bufInfo[hwid], 0, sizeof(bufInfo[hwid]));
}

static void jpeg_drv_hybrid_dec_unlock(unsigned int hwid)
{
	mutex_lock(&jpeg_hybrid_dec_lock);
	if (!dec_hwlocked[hwid] || !dec_hw_enable[hwid]) {
		JPEG_LOG(0, "try to unlock a invalid core %d(isLocked: %d, isEnabled: %d)",
			 hwid, dec_hwlocked[hwid], dec_hw_enable[hwid]);
	} else {
		_jpeg_drv_hybrid_dec_unlock(hwid);
	}
	mutex_unlock(&jpeg_hybrid_dec_lock);
}

static int jpeg_drv_hybrid_dec_suspend_notifier(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int i, result = NOTIFY_OK;

	mutex_lock(&jpeg_hybrid_dec_lock);
	if (action == PM_SUSPEND_PREPARE) {
		/* A live file owns its DMA maps. Never tear them down from a
		 * suspend notifier racing the user's completion/close path. */
		for (i = 0; i < HW_CORE_NUMBER; i++)
			if (dec_hwlocked[i]) {
				result = NOTIFY_BAD;
				goto out;
			}
		gJpegqDev.is_suspending = true;
	} else if (action == PM_POST_SUSPEND) {
		gJpegqDev.is_suspending = false;
	} else {
		result = NOTIFY_DONE;
	}
out:
	mutex_unlock(&jpeg_hybrid_dec_lock);
	return result;
}

static int jpeg_drv_hybrid_dec_suspend(void)
{
	int i, ret = 0;

	mutex_lock(&jpeg_hybrid_dec_lock);
	for (i = 0; i < HW_CORE_NUMBER; i++)
		if (dec_hwlocked[i]) {
			ret = -EBUSY;
			break;
		}
	mutex_unlock(&jpeg_hybrid_dec_lock);
	return ret;
}

static int jpeg_drv_hybrid_dec_get_status(int hwid)
{
	int p_n_s;

	p_n_s = -1;
	mutex_lock(&jpeg_hybrid_dec_lock);
	if (!dec_hwlocked[hwid] || !dec_hw_enable[hwid]) {
		JPEG_LOG(1, "hw %d invalid(isLocked: %d, isEnabled: %d), return -1 status",
			 hwid, dec_hwlocked[hwid], dec_hw_enable[hwid]);
	} else {
		JPEG_LOG(1, "get p_n_s @ hw %d", hwid);
		jpeg_drv_hybrid_dec_get_p_n_s(hwid, &p_n_s);
	}
	mutex_unlock(&jpeg_hybrid_dec_lock);

	return p_n_s;
}

static unsigned int jpeg_get_node_index(const char *name)
{
	if (strncmp(name, "jpgdec0", strlen("jpgdec0")) == 0) {
		JPEG_LOG(0, "name %s", name);
		return 0;
	} else if (strncmp(name, "jpgdec1", strlen("jpgdec1")) == 0) {
		JPEG_LOG(0, "name %s", name);
		return 1;
	}
	JPEG_LOG(0, "name not found %s", name);
	return 0;
}

/* Snapshot only while the requesting file still owns a powered engine.
 * These diagnostics neither reset nor release DMA on an uncertain completion. */
static unsigned int hw_snapshot[8];
module_param_array(hw_snapshot, uint, NULL, 0444);
static void jpeg_snapshot_status(int hwid)
{
	static const unsigned int offsets[] = {
		0x240, 0x248, 0x274, 0x278, 0x340, 0x348, 0x34c, 0x350
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(offsets); i++)
		hw_snapshot[i] = IMG_REG_READ(JPEG_HYBRID_DEC_BASE(hwid) + offsets[i]);
}

/* The progress register publishes completed DMA records and is already
 * the native userspace readiness contract for reading the index. Require
 * the exact full programmed geometry, no decoder error, and a DMA read barrier.
 * Never interpret a partial record count or an IRQ alone as index completion. */
static int jpeg_native_index_state(int hwid)
{
	unsigned int irq, count;

	if (signal_pending(current))
		return -ERESTARTSYS;
	irq = READ_ONCE(_jpeg_hybrid_dec_int_status[hwid]) |
		IMG_REG_READ(REG_JPGDEC_HYBRID_274(hwid));
	if ((irq & 0x36) || IMG_REG_READ(REG_JPGDEC_HYBRID_348(hwid)))
		return -EIO;
	count = IMG_REG_READ(REG_JPGDEC_HYBRID_340(hwid));
	if (count > jpeg_index_entries[hwid])
		return -EIO;
	if (count != jpeg_index_entries[hwid])
		return 0;
	dma_rmb();
	return 1;
}

static int jpeg_check_dec_done(int hwid)
{
	long ret;
	unsigned int status;

	if (READ_ONCE(jpeg_native_faulted))
		return -EIO;
	if (jpeg_index_entries[hwid]) {
		ret = read_poll_timeout(jpeg_native_index_state, status,
			status != 0, 1000, 3000000, false, hwid);
		jpeg_snapshot_status(hwid);
		if (ret || (int)status < 0) {
			WRITE_ONCE(jpeg_native_faulted, true);
			return ret ? (int)ret : (int)status;
		}
		_jpeg_hybrid_dec_int_status[hwid] = 0;
		return 0;
	}
	if (jpeg_isr_hybrid_dec_lisr(hwid) >= 0)
		ret = 1;
	else
		ret = wait_event_interruptible_timeout(
			hybrid_dec_wait_queue[hwid],
			_jpeg_hybrid_dec_int_status[hwid],
			msecs_to_jiffies(3000));
	jpeg_snapshot_status(hwid);
	if (ret <= 0) {
		/* A reset write alone does not establish DMA quiescence. Retain
		 * core/maps/power until reboot rather than free an active target. */
		WRITE_ONCE(jpeg_native_faulted, true);
		return ret ? (int)ret : -ETIMEDOUT;
	}
	status = READ_ONCE(_jpeg_hybrid_dec_int_status[hwid]);
	/* Same register/EOF contract as native MTK JPGDEC. Error/partial IRQs
	 * are not sufficient proof that all output DMA completed. */
	if (!(status & 0x01) || (status & 0x36)) {
		WRITE_ONCE(jpeg_native_faulted, true);
		return -EIO;
	}
	_jpeg_hybrid_dec_int_status[hwid] = 0;
	return 0;
}

/* Per-file state_lock covers the entire request including waits and copyout.
 * Lock order is file state -> global engine lock; IRQ never takes file state. */
static void jpeg_hybrid_finish(struct JpegPrivData *priv)
{
	int id = priv->hw_id;

	if (READ_ONCE(jpeg_native_faulted)) {
		priv->hw_id = -1;
		priv->state = JPEG_DEC_POWER_OFF;
		return;
	}
	if (id >= 0 && id < HW_CORE_NUMBER) {
		mutex_lock(&jpeg_hybrid_dec_lock);
		if (dec_hwlocked[id] && dec_hw_enable[id]) {
			IMG_REG_WRITE(0x0, REG_JPGDEC_HYBRID_090(id));
			IMG_REG_WRITE(0x00000010, REG_JPGDEC_HYBRID_090(id));
		}
		mutex_unlock(&jpeg_hybrid_dec_lock);
		jpeg_drv_hybrid_dec_unlock(id);
	}
	priv->hw_id = -1;
	/* Permit another sequential job on this still-open file. */
	priv->state = JPEG_DEC_OPEN;
}

static int jpeg_hybrid_dispatch(struct file *file, unsigned int cmd,
		struct JPEG_DEC_DRV_HYBRID_TASK *task,
		struct JPEG_DEC_DRV_HYBRID_P_N_S *pns)
{
	struct JpegPrivData *priv = file->private_data;
	int ret = 0, fd = -1, status, id;

	if (!priv)
		return -EBADF;
	mutex_lock(&priv->state_lock);
	switch (cmd) {
	case JPEG_DEC_IOCTL_HYBRID_START:
		if (task->timeout != 3000) { ret = -EINVAL; break; }
		if (priv->state != JPEG_DEC_OPEN) { ret = -EBUSY; break; }
		ret = jpeg_drv_hybrid_dec_lock(&priv->hw_id);
		if (ret) { priv->hw_id = -1; break; }
		priv->state = JPEG_DEC_POWER_ON;
		ret = jpeg_drv_hybrid_dec_start(task->data, priv->hw_id, &fd);
		if (ret) {
			jpeg_hybrid_finish(priv);
			break;
		}
		priv->state = JPEG_DEC_DECODE;
		if (copy_to_user(task->hwid, &priv->hw_id, sizeof(int)) ||
		    copy_to_user(task->index_buf_fd, &fd, sizeof(int))) {
			/* Hardware may have started. Wait boundedly, reset before release. */
			jpeg_check_dec_done(priv->hw_id);
			put_unused_fd(fd);
			jpeg_hybrid_finish(priv);
			ret = -EFAULT;
			break;
		}
		get_file(bufInfo[priv->hw_id].o_dbuf->file);
		fd_install(fd, bufInfo[priv->hw_id].o_dbuf->file);
		break;
	case JPEG_DEC_IOCTL_HYBRID_WAIT:
	case JPEG_DEC_IOCTL_HYBRID_GET_PROGRESS_STATUS:
		id = pns->hwid;
		if (priv->state != JPEG_DEC_DECODE || id < 0 ||
		    id >= HW_CORE_NUMBER || id != priv->hw_id) {
			ret = -EINVAL;
			break;
		}
		if (cmd == JPEG_DEC_IOCTL_HYBRID_GET_PROGRESS_STATUS) {
			status = jpeg_drv_hybrid_dec_get_status(id);
			ret = copy_to_user(pns->progress_n_status, &status, sizeof(int)) ? -EFAULT : 0;
			break;
		}
		priv->state = JPEG_DEC_WAITING;
		ret = jpeg_check_dec_done(id);
		status = ret ? ret : jpeg_drv_hybrid_dec_get_status(id);
		/* Always reset/unmap/release, including failed status copyout. */
		if (copy_to_user(pns->progress_n_status, &status, sizeof(int)))
			ret = -EFAULT;
		jpeg_hybrid_finish(priv);
		break;
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&priv->state_lock);
	return ret;
}

static long jpeg_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct JPEG_DEC_DRV_HYBRID_TASK task;
	struct JPEG_DEC_DRV_HYBRID_P_N_S pns;

	switch (cmd) {
	case JPEG_DEC_IOCTL_HYBRID_START:
		if (copy_from_user(&task, (void __user *)arg, sizeof(task)))
			return -EFAULT;
		return jpeg_hybrid_dispatch(file, cmd, &task, NULL);
	case JPEG_DEC_IOCTL_HYBRID_WAIT:
	case JPEG_DEC_IOCTL_HYBRID_GET_PROGRESS_STATUS:
		if (copy_from_user(&pns, (void __user *)arg, sizeof(pns)))
			return -EFAULT;
		return jpeg_hybrid_dispatch(file, cmd, NULL, &pns);
	default:
		return -ENOTTY;
	}
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_jpeg_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct compat_JPEG_DEC_DRV_HYBRID_TASK task32;
	struct compat_JPEG_DEC_DRV_HYBRID_P_N_S pns32;
	struct JPEG_DEC_DRV_HYBRID_TASK task;
	struct JPEG_DEC_DRV_HYBRID_P_N_S pns;

	switch (cmd) {
	case COMPAT_JPEG_DEC_IOCTL_HYBRID_START:
		if (copy_from_user(&task32, compat_ptr(arg), sizeof(task32)))
			return -EFAULT;
		task.timeout = task32.timeout;
		task.hwid = compat_ptr(task32.hwid);
		task.index_buf_fd = compat_ptr(task32.index_buf_fd);
		memcpy(task.data, task32.data, sizeof(task.data));
		return jpeg_hybrid_dispatch(file, JPEG_DEC_IOCTL_HYBRID_START, &task, NULL);
	case COMPAT_JPEG_DEC_IOCTL_HYBRID_WAIT:
	case COMPAT_JPEG_DEC_IOCTL_HYBRID_GET_PROGRESS_STATUS:
		if (copy_from_user(&pns32, compat_ptr(arg), sizeof(pns32)))
			return -EFAULT;
		pns.hwid = pns32.hwid;
		pns.progress_n_status = compat_ptr(pns32.progress_n_status);
		return jpeg_hybrid_dispatch(file,
			cmd == COMPAT_JPEG_DEC_IOCTL_HYBRID_WAIT ? JPEG_DEC_IOCTL_HYBRID_WAIT :
			JPEG_DEC_IOCTL_HYBRID_GET_PROGRESS_STATUS, NULL, &pns);
	default:
		return -ENOIOCTLCMD;
	}
}
#endif

static int jpeg_open(struct inode *inode, struct file *file)
{
	struct JpegPrivData *priv_data;

	JPEG_LOG(1, "jpeg_open %p", file);

	if (!READ_ONCE(jpeg_native_ready)) return -ENODEV;
	if (root_test_only && !capable(CAP_SYS_ADMIN)) return -EACCES;
	/* Allocate and initialize private data */
	file->private_data = kmalloc(sizeof(struct JpegPrivData), GFP_ATOMIC);

	if (file->private_data == NULL) {
		JPEG_LOG(0, "Not enough entry for JPEG open operation");
		return -ENOMEM;
	}

	priv_data = (struct JpegPrivData *)file->private_data;
	priv_data->state = JPEG_DEC_OPEN;
	mutex_init(&priv_data->state_lock);
	priv_data->hw_id = -1;

	return 0;
}

static ssize_t jpeg_read(
	struct file *file,
	 char __user *data,
	 size_t len, loff_t *ppos)
{
	JPEG_LOG(1, "jpeg driver read");
	return 0;
}

static int jpeg_release(struct inode *inode, struct file *file)
{
	struct JpegPrivData *priv = file->private_data;

	if (!priv)
		return 0;
	mutex_lock(&priv->state_lock);
	if (priv->hw_id >= 0) {
		jpeg_check_dec_done(priv->hw_id);
		jpeg_hybrid_finish(priv);
	}
	mutex_unlock(&priv->state_lock);
	kfree(priv);
	file->private_data = NULL;
	return 0;
}

/* Kernel interface */
static const struct proc_ops jpeg_fops = {
	.proc_ioctl = jpeg_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.proc_compat_ioctl = compat_jpeg_ioctl,
#endif
	.proc_open = jpeg_open,
	.proc_release = jpeg_release,
	.proc_read = jpeg_read,
};

long jpeg_dev_get_hybrid_decoder_base_VA(int id)
{
	return gJpegqDev.hybriddecRegBaseVA[id];
}

static void jpeg_native_put_larb(void *dev)
{
 put_device(dev);
}

static void jpeg_native_put_path(void *path)
{
 mtk_icc_put(path);
}

static int jpeg_probe(struct platform_device *pdev)
{
 struct device *dev = &pdev->dev;
 struct device_node *node = dev->of_node, *larbnode;
 struct platform_device *larbdev;
 struct JpegDeviceStruct saved = gJpegqDev;
 unsigned int idx, cores, first, i, resource;
 int ret;
 void __iomem *base;
 idx = jpeg_get_node_index(node->name);
 if (idx >= JPEG_LARB_COUNT || gJpegqDev.pDev[idx]) return -EBUSY;
 if (idx && !jpeg_native_ready) return -EPROBE_DEFER;
 first = idx ? gJpegqDev.first_larb_core_num : 0;
 cores = 1;
 if (!idx && of_property_read_u32(node, "core-num", &cores)) cores = 2;
 if (!cores || cores > HW_CORE_NUMBER || first + cores > HW_CORE_NUMBER) return -EINVAL;
 gJpegqDev.pDev[idx] = dev;
 gJpegqDev.smmu_dev[idx] = mtk_smmu_get_shared_device(dev);
 if (!idx) {
  gJpegqDev.first_larb_core_num = cores;
  gJpegqDev.is_qos_16_level = of_property_read_bool(node, "mmqos-16-level");
  gJpegqDev.is_ccf_one_step = of_property_read_bool(node, "ccf-one-step");
 }
 for (i = first; i < first + cores; i++) {
  gJpegqDev.larb_idx_map[i] = idx;
  base = devm_platform_ioremap_resource(pdev, i - first);
  if (IS_ERR(base)) { ret = PTR_ERR(base); goto fail; }
  gJpegqDev.hybriddecRegBaseVA[i] = (unsigned long)base;
  ret = platform_get_irq(pdev, i - first);
  if (ret < 0) goto fail;
  gJpegqDev.hybriddecIrqId[i] = ret;
  init_waitqueue_head(&hybrid_dec_wait_queue[i]);
  ret = devm_request_irq(dev, ret, jpeg_drv_hybrid_dec_isr,
                         IRQF_NO_AUTOEN, "jpeg_dec_driver", dev);
  if (ret) goto fail;
  gJpegqDev.jpegClk[i] = devm_clk_get(dev, jpegdec_cg_name[i]);
  if (IS_ERR(gJpegqDev.jpegClk[i])) { ret = PTR_ERR(gJpegqDev.jpegClk[i]); goto fail; }
  of_property_read_u32_index(node, "larb-port", (i-first)*3, &gJpegqDev.larb_port[i][0]);
  of_property_read_u32_index(node, "larb-port", (i-first)*3+1, &gJpegqDev.larb_port[i][1]);
  of_property_read_u32_index(node, "larb-port", (i-first)*3+2, &gJpegqDev.larb_port[i][2]);
  /* Old native DT has no ICC named paths. Missing optional paths are NULL;
   * errors for declared provider paths, including defer, are not discarded. */
  gJpegqDev.jpeg_path_wdma[i] = of_mtk_icc_get(dev, jpegdec_wdma_name[i]);
  gJpegqDev.jpeg_path_bsdma[i] = of_mtk_icc_get(dev, jpegdec_bsdma_name[i]);
  gJpegqDev.jpeg_path_huff_offset[i] = of_mtk_icc_get(dev, jpegdec_huff_offset_name[i]);
  {
   struct icc_path *paths[] = { gJpegqDev.jpeg_path_wdma[i], gJpegqDev.jpeg_path_bsdma[i], gJpegqDev.jpeg_path_huff_offset[i] };
   unsigned int j;
   for (j = 0; j < ARRAY_SIZE(paths); j++) {
    if (IS_ERR(paths[j])) { ret = PTR_ERR(paths[j]); goto fail; }
    if (paths[j]) { ret = devm_add_action_or_reset(dev, jpeg_native_put_path, paths[j]); if (ret) goto fail; }
   }
  }
 }
 if (!of_property_read_u32(node, "jpeg-set-resource", &resource)) {
  base = devm_platform_ioremap_resource(pdev, resource);
  if (IS_ERR(base)) { ret = PTR_ERR(base); goto fail; }
  gJpegqDev.ven0BaseVA = (unsigned long)base;
 }
 if (!of_property_read_u32(node, "jpeg-set-axdomain-base-idx", &resource)) {
  base = devm_platform_ioremap_resource(pdev, resource);
  if (IS_ERR(base)) { ret = PTR_ERR(base); goto fail; }
  gJpegqDev.smiLarbBaseVA[idx] = (unsigned long)base;
 }
 of_property_read_u32(node, "jpeg-set-axdomain", &gJpegqDev.axdomain[idx]);
 larbnode = of_parse_phandle(node, "mediatek,larbs", 0);
 if (!larbnode) { ret = -EINVAL; goto fail; }
 larbdev = of_find_device_by_node(larbnode);
 of_node_put(larbnode);
 if (!larbdev) { ret = -EPROBE_DEFER; goto fail; }
 gJpegqDev.jpegLarb[idx] = &larbdev->dev;
 ret = devm_add_action_or_reset(dev, jpeg_native_put_larb, &larbdev->dev);
 if (ret) goto fail;
 if (!gJpegqDev.is_ccf_one_step &&
     !device_link_add(dev, &larbdev->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_AUTOREMOVE_CONSUMER)) {
  ret = -ENOMEM; goto fail;
 }
 ret = dma_set_mask_and_coherent(gJpegqDev.smmu_dev[idx], DMA_BIT_MASK(34));
 if (ret) goto fail;
 if (!idx) {
  ret = jpeg_drv_hybrid_dec_prepare_dvfs();
  if (ret) goto fail;
  gJpegqDev.pm_notifier.notifier_call = jpeg_drv_hybrid_dec_suspend_notifier;
  gJpegqDev.pm_notifier.priority = 1;
  ret = register_pm_notifier(&gJpegqDev.pm_notifier);
  if (ret) goto fail;
  jpeg_native_proc = proc_create("mtk_jpeg", 0644, NULL, &jpeg_fops);
  if (!jpeg_native_proc) {
   unregister_pm_notifier(&gJpegqDev.pm_notifier);
   ret = -ENOMEM; goto fail;
  }
 }
 if (!gJpegqDev.is_ccf_one_step) pm_runtime_enable(dev);
 for (i = first; i < first + cores; i++) dec_hw_enable[i] = true;
 atomic_inc(&nodeCount);
 __module_get(THIS_MODULE);
 WRITE_ONCE(jpeg_native_ready, true);
 dev_info(dev, "Native hybrid JPEG decoder ready: cores %u..%u\n", first, first+cores-1);
 return 0;
fail:
 gJpegqDev = saved;
 return ret;
}

static void jpeg_remove(struct platform_device *pdev)
{
	int i, node_index;

	JPEG_LOG(0, "JPEG Codec remove");
	atomic_dec(&nodeCount);

	node_index = jpeg_get_node_index(pdev->dev.of_node->name);

	if (node_index == 0) {
		for (i = 0; i < HW_CORE_NUMBER - 1; i++) {
			free_irq(gJpegqDev.hybriddecIrqId[i], NULL);
			dec_hw_enable[i] = false;
		}
	} else {
		i = HW_CORE_NUMBER - 1;
		free_irq(gJpegqDev.hybriddecIrqId[i], NULL);
		dec_hw_enable[i] = false;
	}
	jpeg_drv_hybrid_dec_unprepare_dvfs();
}

static void jpeg_shutdown(struct platform_device *pdev)
{
	mutex_lock(&jpeg_hybrid_dec_lock);
	gJpegqDev.is_shutdowning = true;
	/* Owners retain their maps and power until the system resets. No
	 * asynchronous forced unlock racing an in-flight decoder request. */
	mutex_unlock(&jpeg_hybrid_dec_lock);
}

/* PM suspend */
static int jpeg_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	int ret;

	ret = jpeg_drv_hybrid_dec_suspend();
	if (ret != 0)
		return ret;
	return 0;
}

/* PM resume */
static int jpeg_resume(struct platform_device *pdev)
{
	return 0;
}

static int jpeg_pm_suspend(struct device *pDevice)
{
	struct platform_device *pdev = to_platform_device(pDevice);

	WARN_ON(pdev == NULL);

	return jpeg_suspend(pdev, PMSG_SUSPEND);
}

static int jpeg_pm_resume(struct device *pDevice)
{
	struct platform_device *pdev = to_platform_device(pDevice);

	WARN_ON(pdev == NULL);

	return jpeg_resume(pdev);
}

static int jpeg_pm_restore_noirq(struct device *pDevice)
{
	return 0;
}

static const struct dev_pm_ops jpeg_pm_ops = {
	.suspend = jpeg_pm_suspend,
	.resume = jpeg_pm_resume,
	.freeze = NULL,
	.thaw = NULL,
	.poweroff = NULL,
	.restore = NULL,
	.restore_noirq = jpeg_pm_restore_noirq,
};

static struct platform_driver jpeg_driver = {
	.probe = jpeg_probe,
	.remove = jpeg_remove,
	.shutdown = jpeg_shutdown,
	.suspend = jpeg_suspend,
	.resume = jpeg_resume,
	.driver = {
		.name = JPEG_DEVNAME,
		.suppress_bind_attrs = true,
		.pm = &jpeg_pm_ops,
		.of_match_table = jdec_hybrid_of_ids,
	},
};

static int __init jpeg_init(void)
{
	int ret;

	JPEG_LOG(0, "Register the JPEG Codec driver");
	atomic_set(&nodeCount, 0);
	if (platform_driver_register(&jpeg_driver)) {
		JPEG_LOG(0, "failed to register jpeg codec driver");
		ret = -ENODEV;
		return ret;
	}

	return 0;
}

static void __exit jpeg_exit(void)
{
	remove_proc_entry("mtk_jpeg", NULL);
	platform_driver_unregister(&jpeg_driver);
}
module_init(jpeg_init);
module_exit(jpeg_exit);
MODULE_AUTHOR("Jason Hsu <yeong-cherng.hsu@mediatek.com>");
MODULE_DESCRIPTION("JPEG Dec Codec Driver");
MODULE_LICENSE("GPL");
