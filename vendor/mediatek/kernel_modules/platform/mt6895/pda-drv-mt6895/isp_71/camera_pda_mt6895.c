// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/compat.h>
#include <linux/jiffies.h>

#include <linux/time.h>  //do_gettimeofday()

#include "mtk-interconnect.h"

// --------- DMA-BUF ----------
#include <linux/dma-heap.h>
#include <linux/dma-direction.h>
#include <linux/scatterlist.h>
#include <linux/dma-buf.h>
// ----------------------------

#include <soc/mediatek/smi.h>  // mtk_smi_larb_get()

#include "camera_pda.h"

// --------- define region --------
// #define FPGA_UT
// #define GET_PDA_TIME
// #define FOR_DEBUG
// #define SMI_LOG
#define CHECK_IRQ_COUNT
#define PDA_MMQOS
// --------------------------------

#ifdef PDA_TF_DUMP_71_3
#include "iommu_debug.h"
#include <dt-bindings/memory/mt6895-larb-port.h>
#endif

#define PDA_DEV_NAME "camera-pda"

#define LOG_INF(format, args...)                                               \
	pr_info(PDA_DEV_NAME " [%s] " format, __func__, ##args)

#ifndef MTRUE
#define MTRUE 1
#endif
#ifndef MFALSE
#define MFALSE 0
#endif

//define the write register function
#define mt_reg_sync_writel(v, a) \
	do {    \
		*(unsigned int *)(a) = (v);    \
		mb();  /*make sure register access in order */ \
	} while (0)
#define PDA_WR32(addr, data) mt_reg_sync_writel(data, addr)
#define PDA_RD32(addr) ioread32(addr)

#ifdef FPGA_UT
void __iomem *CAMSYS_CONFIG_BASE;
#define CAMSYS_MAIN_BASE_ADDR CAMSYS_CONFIG_BASE
#define REG_CAMSYS_CG_SET               (CAMSYS_MAIN_BASE_ADDR + 0x4)
#define REG_CAMSYS_CG_CLR               (CAMSYS_MAIN_BASE_ADDR + 0x8)
#endif

static struct regmap *pda_mraw_regmap;

#define PDA_DONE 0x00000001
#define PDA_ERROR 0x00000002
#define PDA_STATUS_REG 0x00000001
#define PDA_CLEAR_REG 0x00000000
#define PDA_TRIGGER 0x00000003
#define PDA_DOUBLE_BUFFER 0x00000009
#define PDA_MAKE_RESET 0x00000002
#define MASK_BIT_ZERO 0x00000001
#define PDA_RESET_VALUE 0x00000001
#define PDA_HW_RESET 0x00000004

struct device *g_dev1, *g_dev2;


static DEFINE_MUTEX(pda_operation_lock);
/* A timed-out engine retains mappings, a clock/PM lease and module text. */
static unsigned long pda_dma_active_mask;
static bool pda_dma_quarantined;
static bool pda_dma_lease;
static bool pda_online;
static bool pda_driver_ready;
static bool pda_teardown_quarantined;
static unsigned int pda_file_users;
static unsigned int pda_bound_mask;
static unsigned int pda_expected_mask;
static unsigned int pda_owned_clocks;
struct pda_engine_owner {
	struct device *dev;
	bool irq_requested;
	bool pm_enabled;
};
static struct pda_engine_owner pda_owners[2];

static DECLARE_WAIT_QUEUE_HEAD(g_wait_queue_head);

// PDA HW quantity
static int g_PDA_quantity;

#ifdef CHECK_IRQ_COUNT
// Calculate reasonable irq counts
static unsigned int g_reasonable_IRQCount;
static int g_PDA0_IRQCount;
static int g_PDA1_IRQCount;
#endif

static struct PDA_Data_t_v2 g_pda_Pdadata;

// pda device information
// static struct PDA_device *PDA_devs;
static struct PDA_device PDA_devs[PDA_MAX_QUANTITY];

// clock relate
static const char * const clk_names[] = {
	"camsys_mraw_pda0",
	"camsys_mraw_pda1",
	"mraw_larbx",
	"cam_main_cam2mm0_gals_cg_con",
	"cam_main_cam2mm1_gals_cg_con",
	"cam_main_cam_cg_con",
};
#define PDA_CLK_NUM ARRAY_SIZE(clk_names)
struct PDA_CLK_STRUCT pda_clk[PDA_CLK_NUM];

#ifdef PDA_MMQOS
// mmqos relate
static const char * const mmqos_names_rdma[] = {
	"l25_pdai_a0",
	"l25_pdai_a1"
};
#define PDA_MMQOS_RDMA_NUM ARRAY_SIZE(mmqos_names_rdma)
struct icc_path *icc_path_pda_rdma[PDA_MMQOS_RDMA_NUM];

static const char * const mmqos_names_wdma[] = {
	"l25_pdao_a"
};
#define PDA_MMQOS_WDMA_NUM ARRAY_SIZE(mmqos_names_wdma)
struct icc_path *icc_path_pda_wdma[PDA_MMQOS_WDMA_NUM];
#endif

// Enable clock count
static unsigned int g_u4EnableClockCount;

#ifdef GET_PDA_TIME
// Get PDA process time
struct timespec64 time_end;
struct timespec64 total_time_begin, total_time_end;
struct timespec64 pda1_done_b, pda1_done_e;
struct timespec64 pda2_done_b, pda2_done_e;
#endif

// calculate 1024 roi data
unsigned int g_rgn_x_buf[PDA_MAXROI_PER_ROUND];
unsigned int g_rgn_y_buf[PDA_MAXROI_PER_ROUND];
unsigned int g_rgn_h_buf[PDA_MAXROI_PER_ROUND];
unsigned int g_rgn_w_buf[PDA_MAXROI_PER_ROUND];
unsigned int g_rgn_iw_buf[PDA_MAXROI_PER_ROUND];

// buffer mmu
struct pda_mmu g_image_mmu;
struct pda_mmu g_table_mmu;
struct pda_mmu g_output_mmu;

// Output buffer
unsigned long g_Address_LI;
unsigned long g_Address_RI;
unsigned long g_Address_LT;
unsigned long g_Address_RT;
static unsigned long g_OutputBufferAddr;
static unsigned long g_OutputBufferOffset;

#ifdef FOR_DEBUG
// buffer address
unsigned int *g_buf_LI_va;
unsigned int *g_buf_RI_va;
unsigned int *g_buf_LT_va;
unsigned int *g_buf_RT_va;
unsigned int *g_buf_Out_va;
#endif

// current Process ROI number
unsigned int g_CurrentProcRoiNum[PDA_MAX_QUANTITY];

#ifdef PDA_MMQOS
static int pda_mmqos_init(void)
{
	int i, ret;

	for (i = 0; i < PDA_MMQOS_RDMA_NUM; i++) {
		icc_path_pda_rdma[i] = of_mtk_icc_get(g_dev1, mmqos_names_rdma[i]);
		if (IS_ERR_OR_NULL(icc_path_pda_rdma[i])) {
			ret = icc_path_pda_rdma[i] ? PTR_ERR(icc_path_pda_rdma[i]) : -ENODEV;
			icc_path_pda_rdma[i] = NULL;
			return ret;
		}
	}
	for (i = 0; i < PDA_MMQOS_WDMA_NUM; i++) {
		icc_path_pda_wdma[i] = of_mtk_icc_get(g_dev1, mmqos_names_wdma[i]);
		if (IS_ERR_OR_NULL(icc_path_pda_wdma[i])) {
			ret = icc_path_pda_wdma[i] ? PTR_ERR(icc_path_pda_wdma[i]) : -ENODEV;
			icc_path_pda_wdma[i] = NULL;
			return ret;
		}
	}
	return 0;
}

static void pda_mmqos_put(void)
{
	int i;

	for (i = PDA_MMQOS_WDMA_NUM - 1; i >= 0; i--) {
		if (icc_path_pda_wdma[i])
			mtk_icc_put(icc_path_pda_wdma[i]);
		icc_path_pda_wdma[i] = NULL;
	}
	for (i = PDA_MMQOS_RDMA_NUM - 1; i >= 0; i--) {
		if (icc_path_pda_rdma[i])
			mtk_icc_put(icc_path_pda_rdma[i]);
		icc_path_pda_rdma[i] = NULL;
	}
}

static void pda_mmqos_bw_set(void)
{
	int i = 0;
	// int Inter_ROI_Max_Width = 1024;
	// int Inter_ROI_Max_Height = 96;
	int Inter_Frame_Size_Width = 2048;
	int Inter_Frame_Size_Height = 192;

	int Mach_ROI_Max_Width = 2048;
	int Mach_ROI_Max_Height = 96;
	int Mach_Frame_Size_Width = 4096;
	int Mach_Frame_Size_Height = 192;

	double Freqency = 360.0;
	int FOV = 100;
	int ROI_Number = 45;
	int Frame_Rate = 30;
	// int Expected_HW_Compute_time = 15;
	int Search_Range = 40;

	// Intermediate data
	double Operation_Margin = 1.2;
	int Inter_Frame_Size = Inter_Frame_Size_Width * Inter_Frame_Size_Height;
	int Mach_Frame_Size = Mach_Frame_Size_Width * Mach_Frame_Size_Height;
	// int Inter_Factor = 8;
	int Inter_Frame_Size_FOV = Inter_Frame_Size * FOV / 100;
	int Mach_Frame_Size_FOV = Mach_Frame_Size * FOV / 100;
	int Inter_Input_Total_pixel_Itar = Inter_Frame_Size_FOV;
	int Inter_Input_Total_pixel_Iref = Inter_Frame_Size_FOV;
	int Mach_Input_Total_pixel_Itar =
		(ROI_Number*Search_Range*Mach_ROI_Max_Height) +
		Mach_Frame_Size_FOV +
		(1*Mach_ROI_Max_Width);
	// int Mach_Input_Total_pixel_Iref = Mach_Frame_Size_FOV;
	int Required_Operation_Cycle =
		(int)(Mach_Input_Total_pixel_Itar *
		Operation_Margin *
		(Search_Range+1)) /
		Search_Range + 1;
	int WDMA_Data = OUT_BYTE_PER_ROI*ROI_Number;
	int temp = Inter_Input_Total_pixel_Itar+Inter_Input_Total_pixel_Iref;
	int RDMA_Data = temp*20/8;

	double OperationTime = (double)Required_Operation_Cycle / Freqency / 1000.0;

	// WDMA BW estimate
	double WDMA_PEAK_BW = (double)WDMA_Data / (double)OperationTime / 1000.0;
	double WDMA_AVG_BW = (double)WDMA_Data * Frame_Rate * (1.33) / 1000000.0;

	// RDMA BW estimate
	double RDMA_PEAK_BW = (double)RDMA_Data / (double)OperationTime / 1000.0;
	double RDMA_AVG_BW = (double)RDMA_Data / (1000000.0/(double)Frame_Rate);

	// Left/Right RDMA BW
	double IMAGE_TABLE_RDMA_PEAK_BW = RDMA_PEAK_BW / 2;
	double IMAGE_TABLE_RDMA_AVG_BW = RDMA_AVG_BW * (1.33) / 2;

	// pda is not HRT engine, no need to set HRT bw
	IMAGE_TABLE_RDMA_PEAK_BW = 0;
	WDMA_PEAK_BW = 0;

	LOG_INF("RDMA_BW AVG/PEAK: %d/%d, WDMA_BW AVG/PEAK: %d/%d\n",
		(int)MBps_to_icc(IMAGE_TABLE_RDMA_AVG_BW),
		(int)MBps_to_icc(IMAGE_TABLE_RDMA_PEAK_BW),
		(int)MBps_to_icc(WDMA_AVG_BW),
		(int)MBps_to_icc(WDMA_PEAK_BW));

	// MMQOS set bw
	for (i = 0; i < PDA_MMQOS_RDMA_NUM; ++i) {
		if (icc_path_pda_rdma[i]) {
			mtk_icc_set_bw(icc_path_pda_rdma[i],
				(int)MBps_to_icc(IMAGE_TABLE_RDMA_AVG_BW),
				(int)MBps_to_icc(IMAGE_TABLE_RDMA_PEAK_BW));
		}
	}

	for (i = 0; i < PDA_MMQOS_WDMA_NUM; ++i) {
		if (icc_path_pda_wdma[i]) {
			mtk_icc_set_bw(icc_path_pda_wdma[i],
				(int)MBps_to_icc(WDMA_AVG_BW),
				(int)MBps_to_icc(WDMA_PEAK_BW));
		}
	}
}

static void pda_mmqos_bw_reset(void)
{
	int i = 0;

#ifdef FOR_DEBUG
	LOG_INF("mmqos reset\n");
#endif

	// MMQOS set bw
	for (i = 0; i < PDA_MMQOS_RDMA_NUM; ++i) {
		if (icc_path_pda_rdma[i])
			mtk_icc_set_bw(icc_path_pda_rdma[i], 0, 0);
	}
	for (i = 0; i < PDA_MMQOS_WDMA_NUM; ++i) {
		if (icc_path_pda_wdma[i])
			mtk_icc_set_bw(icc_path_pda_wdma[i], 0, 0);
	}
}
#endif

static int pda_link_larbs(struct platform_device *pdev)
{
	struct device_node *node;
	struct platform_device *supplier;
	struct device_link *link;
	int count, i;
	bool bound;

	count = of_count_phandle_with_args(pdev->dev.of_node, "mediatek,larbs", NULL);
	if (count < 0)
		return count;
	if (!count)
		return -ENODEV;
	for (i = 0; i < count; i++) {
		node = of_parse_phandle(pdev->dev.of_node, "mediatek,larbs", i);
		if (!node)
			return -EINVAL;
		supplier = of_find_device_by_node(node);
		of_node_put(node);
		if (!supplier)
			return -EPROBE_DEFER;
		/* Managed links are owned by the core, including failed-probe unwind.
		 * No raw supplier pointer or driver-owned link survives this call.
		 */
		link = device_link_add(&pdev->dev, &supplier->dev,
			DL_FLAG_PM_RUNTIME | DL_FLAG_AUTOREMOVE_CONSUMER);
		bound = device_is_bound(&supplier->dev);
		put_device(&supplier->dev);
		if (!link)
			return -ENOMEM;
		if (!bound)
			return -EPROBE_DEFER;
	}
	return 0;
}

static inline int PDA_Prepare_Enable_ccf_clock(void)
{
	int ret, i;

	if (!g_PDA_quantity || g_PDA_quantity > PDA_MAX_QUANTITY || !g_dev1 ||
	    (g_PDA_quantity > 1 && !g_dev2))
		return -ENODEV;
	for (i = 0; i < PDA_CLK_NUM; i++)
		if (IS_ERR_OR_NULL(pda_clk[i].CG_PDA_TOP_MUX))
			return pda_clk[i].CG_PDA_TOP_MUX ?
				PTR_ERR(pda_clk[i].CG_PDA_TOP_MUX) : -EPROBE_DEFER;

	ret = pm_runtime_resume_and_get(g_dev1);
	if (ret < 0)
		return ret;
	if (g_PDA_quantity > 1) {
		ret = pm_runtime_resume_and_get(g_dev2);
		if (ret < 0)
			goto put_primary;
	}
	for (i = 0; i < PDA_CLK_NUM; i++) {
		ret = clk_prepare_enable(pda_clk[i].CG_PDA_TOP_MUX);
		if (ret)
			goto disable_clocks;
	}
	for (i = 0; i < g_PDA_quantity; i++)
		enable_irq(PDA_devs[i].irq);
	return 0;

disable_clocks:
	while (i--)
		clk_disable_unprepare(pda_clk[i].CG_PDA_TOP_MUX);
	if (g_PDA_quantity > 1)
		pm_runtime_put_sync(g_dev2);
put_primary:
	pm_runtime_put_sync(g_dev1);
	return ret;
}

static inline void PDA_Disable_Unprepare_ccf_clock(void)
{
	int i;

	for (i = 0; i < g_PDA_quantity; i++)
		disable_irq(PDA_devs[i].irq);

	for (i = PDA_CLK_NUM - 1; i >= 0; i--) {
		clk_disable_unprepare(pda_clk[i].CG_PDA_TOP_MUX);
#ifdef FOR_DEBUG
		LOG_INF("clk_disable_unprepare (%s) done\n", clk_names[i]);
#endif
	}

#if IS_ENABLED(CONFIG_OF)
	if (g_PDA_quantity > 1)
		pm_runtime_put_sync(g_dev2);
	if (g_PDA_quantity > 0)
		pm_runtime_put_sync(g_dev1);
#ifdef FOR_DEBUG
	LOG_INF("pm_runtime_put_sync done\n");
#endif
#endif
}

/**************************************************************
 *
 **************************************************************/
static int EnableClock(bool enable)
{
	int ret;

	/* Open/release/ioctl already hold pda_operation_lock. */
	if (enable) {
		if (g_u4EnableClockCount == UINT_MAX)
			return -EOVERFLOW;
		if (!g_u4EnableClockCount) {
#ifndef FPGA_UT
			ret = PDA_Prepare_Enable_ccf_clock();
			if (ret)
				return ret;
#else
			PDA_WR32(REG_CAMSYS_CG_CLR, 0xFFFFFFFF);
#endif
		}
		g_u4EnableClockCount++;
		return 0;
	}
	if (!g_u4EnableClockCount)
		return -EINVAL;
	if (!--g_u4EnableClockCount) {
#ifndef FPGA_UT
		PDA_Disable_Unprepare_ccf_clock();
#else
		PDA_WR32(REG_CAMSYS_CG_SET, 0xFFFFFFFF);
#endif
	}
	return 0;
}

static int pda_reset(int PDA_Index)
{
	unsigned long end = 0;

	end = jiffies + msecs_to_jiffies(100);

	// reset HW status
	WRITE_ONCE(PDA_devs[PDA_Index].HWstatus, 0);

	// clear dma_soft_rst_stat
	PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_DMA_RST_REG,
		PDA_CLEAR_REG);
	// make reset
	PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_DMA_RST_REG,
		PDA_MAKE_RESET);
	wmb(); /* TBC */

	while (time_before(jiffies, end)) {
		if ((PDA_RD32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_DMA_RST_REG) &
			MASK_BIT_ZERO)) {
			// equivalent to hardware reset
			PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_TOP_CTL_REG,
				PDA_HW_RESET);
			// clear reset signal
			PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_DMA_RST_REG,
				PDA_CLEAR_REG);
			wmb(); /* TBC */
			// clear hardware reset signal
			PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_TOP_CTL_REG,
				PDA_CLEAR_REG);
			// LOG_INF("reset PDA%d hw success\n", PDA_Index);
			return 0;
		}

		LOG_INF("PDA%d Wait EMI request, DMA_RST:0x%x\n",
			PDA_Index,
			PDA_RD32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_DMA_RST_REG));

		usleep_range(10, 20);
	}

	LOG_INF("reset PDA%d hw timeout\n", PDA_Index);
	return -ETIMEDOUT;
}

static int pda_nontransaction_reset(int PDA_Index)
{
	unsigned int mask;
	int ret;

	if (PDA_Index < 0 || PDA_Index > 1 || !pda_mraw_regmap)
		return -EINVAL;
	mask = PDA_Index ? BIT(14) | BIT(15) : BIT(12) | BIT(13);
	PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_TOP_CTL_REG,
		 PDA_HW_RESET);
	PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDA_TOP_CTL_REG,
		 PDA_CLEAR_REG);
	/* This is the clock provider's same cached syscon map. Each masked
	 * update locks and rereads the register; never replay an old full word.
	 */
	ret = regmap_update_bits(pda_mraw_regmap, 0xa0, mask, mask);
	if (ret)
		return ret;
	return regmap_update_bits(pda_mraw_regmap, 0xa0, mask, 0);
}

static bool pda_dma_is_linear(struct dma_buf *buffer, struct sg_table *table)
{
	struct scatterlist *sg;
	u64 next = 0, total = 0;
	unsigned int i;

	if (!buffer->size || !table->sgl || !table->nents)
		return false;
	for_each_sgtable_dma_sg(table, sg, i) {
		u64 address = sg_dma_address(sg);
		u64 length = sg_dma_len(sg);

		/* PDA programs one linear 34-bit IOVA per buffer. */
		if (!address || !length || address > DMA_BIT_MASK(34) ||
		    length - 1 > DMA_BIT_MASK(34) - address ||
		    (i && address != next))
			return false;
		next = address + length;
		total += length;
	}
	return total >= buffer->size;
}

static int pda_get_dma_buffer(struct pda_mmu *mmu, int fd)
{
	struct pda_mmu candidate = {};
	int ret;

	if (fd < 0)
		return -EBADF;
	if (!g_dev1)
		return -ENODEV;
	if (mmu->dma_buf || mmu->attach || mmu->sgt)
		return -EBUSY;
	candidate.dma_buf = dma_buf_get(fd);
	if (IS_ERR(candidate.dma_buf))
		return PTR_ERR(candidate.dma_buf);
	candidate.attach = dma_buf_attach(candidate.dma_buf, g_dev1);
	if (IS_ERR(candidate.attach)) {
		ret = PTR_ERR(candidate.attach);
		goto err_put;
	}
	/* 6.12 raw map/unmap require a held reservation lock. */
	candidate.sgt = dma_buf_map_attachment_unlocked(candidate.attach,
			DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(candidate.sgt)) {
		ret = candidate.sgt ? PTR_ERR(candidate.sgt) : -ENOMEM;
		goto err_detach;
	}
	if (!pda_dma_is_linear(candidate.dma_buf, candidate.sgt)) {
		ret = -EINVAL;
		goto err_unmap;
	}
	*mmu = candidate;
	return 0;

err_unmap:
	dma_buf_unmap_attachment_unlocked(candidate.attach, candidate.sgt,
			DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(candidate.dma_buf, candidate.attach);
err_put:
	dma_buf_put(candidate.dma_buf);
	return ret;
}

static void pda_put_dma_buffer(struct pda_mmu *mmu)
{
	struct pda_mmu owned = *mmu;

	memset(mmu, 0, sizeof(*mmu));
	if (owned.attach && owned.sgt)
		dma_buf_unmap_attachment_unlocked(owned.attach, owned.sgt,
				DMA_BIDIRECTIONAL);
	if (owned.dma_buf && owned.attach)
		dma_buf_detach(owned.dma_buf, owned.attach);
	if (owned.dma_buf)
		dma_buf_put(owned.dma_buf);
}

static int pda_begin_dma(void)
{
	int ret;

	/* All request/open/release paths hold pda_operation_lock. */
	if (pda_dma_quarantined || pda_dma_active_mask || pda_dma_lease)
		return -EBUSY;
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
	ret = EnableClock(MTRUE);
	if (ret) {
		module_put(THIS_MODULE);
		return ret;
	}
	pda_dma_lease = true;
	return 0;
}

static int pda_retire_dma(void)
{
	unsigned int i;

	for (i = 0; i < PDA_MAX_QUANTITY; i++) {
		if (!(pda_dma_active_mask & BIT(i)))
			continue;
		/* Nontransaction/MRAW reset alone is not a DMA-stop acknowledgement. */
		if (pda_reset(i))
			continue;
		if (pda_nontransaction_reset(i))
			continue;
		pda_dma_active_mask &= ~BIT(i);
	}
	if (pda_dma_active_mask) {
		pda_dma_quarantined = true;
		return -ETIMEDOUT;
	}

	pda_dma_quarantined = false;
	pda_put_dma_buffer(&g_output_mmu);
	pda_put_dma_buffer(&g_image_mmu);
	pda_put_dma_buffer(&g_table_mmu);
	if (pda_dma_lease) {
		pda_dma_lease = false;
		EnableClock(MFALSE);
		module_put(THIS_MODULE);
	}
	return 0;
}

static bool pda_request_roi_valid(const struct PDA_Data_t_v2 *data)
{
	unsigned int i;

	if (data->ROInumber > PDAROIARRAYMAX ||
	    data->nNumerousROI > FIXROIARRAYMAX ||
	    (!data->ROInumber && !data->nNumerousROI))
		return false;
	for (i = 0; i < data->nNumerousROI; i++) {
		if (!data->xnum[i] || !data->ynum[i] ||
		    data->xnum[i] > 1024 / data->ynum[i])
			return false;
	}
	return true;
}

static bool pda_buffer_holds_planes(const struct pda_mmu *mmu,
		unsigned int plane_size, unsigned int planes)
{
	return mmu->dma_buf && plane_size && planes &&
		plane_size <= mmu->dma_buf->size / planes;
}

static bool pda_output_size_valid(const struct PDA_Data_t_v2 *data,
		const struct pda_mmu *mmu)
{
	u64 rois;
	unsigned int i;

	if (!pda_request_roi_valid(data) || !mmu->dma_buf)
		return false;
	rois = data->ROInumber;
	for (i = 0; i < data->nNumerousROI; i++)
		rois += (u64)data->xnum[i] * data->ynum[i];
	return data->OutputSize <= mmu->dma_buf->size &&
		rois <= data->OutputSize / OUT_BYTE_PER_ROI;
}

static int Get_Input_Addr_From_DMABUF(struct PDA_Data_t_v2 *pda_PdaConfig)
{
	int ret = 0;
	int i = 0;

	// Left image buffer
	ret = pda_get_dma_buffer(&g_image_mmu, pda_PdaConfig->FD_L_Image);
	if (ret < 0) {
		LOG_INF("Left image, pda_get_dma_buffer fail!\n");
		return ret;
	}
	// Left table buffer
	ret = pda_get_dma_buffer(&g_table_mmu, pda_PdaConfig->FD_L_Table);
	if (ret < 0) {
		LOG_INF("Left table, pda_get_dma_buffer fail!\n");
		return ret;
	}
	/* Each imported allocation holds the packed left and right planes. */
	if (!pda_buffer_holds_planes(&g_image_mmu, pda_PdaConfig->ImageSize, 2) ||
	    !pda_buffer_holds_planes(&g_table_mmu, pda_PdaConfig->TableSize, 2))
		return -EINVAL;
	g_Address_LI = (unsigned long) sg_dma_address(g_image_mmu.sgt->sgl);
	pda_PdaConfig->PDA_PDAI_P1_BASE_ADDR = (unsigned int)g_Address_LI;
	for (i = 0; i < g_PDA_quantity; i++) {
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_MSB_REG,
			(unsigned int)(g_Address_LI >> 32));
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_REG,
			(unsigned int)(g_Address_LI));
	}
#ifdef FOR_DEBUG
	LOG_INF("Left image MVA MSB = 0x%lx\n", g_Address_LI);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("Left image MVA MSB = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_MSB_REG));
		LOG_INF("Left image MVA = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_REG));
	}
#endif

	// Right image buffer
	g_Address_RI = g_Address_LI + pda_PdaConfig->ImageSize;
	pda_PdaConfig->PDA_PDAI_P2_BASE_ADDR = (unsigned int)(g_Address_RI);
	for (i = 0; i < g_PDA_quantity; i++) {
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_MSB_REG,
			(unsigned int)(g_Address_RI >> 32));
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_REG,
			(unsigned int)(g_Address_RI));
	}
#ifdef FOR_DEBUG
	LOG_INF("Right image MVA MSB = 0x%lx\n", g_Address_RI);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("Right image MVA MSB = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_MSB_REG));
		LOG_INF("Right image MVA = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_REG));
	}
#endif

	// Left table buffer
	g_Address_LT = (unsigned long) sg_dma_address(g_table_mmu.sgt->sgl);
	pda_PdaConfig->PDA_PDATI_P1_BASE_ADDR = (unsigned int)g_Address_LT;
	for (i = 0; i < g_PDA_quantity; i++) {
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_MSB_REG,
			(unsigned int)(g_Address_LT >> 32));
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_REG,
			(unsigned int)(g_Address_LT));
	}
#ifdef FOR_DEBUG
	LOG_INF("Left table MVA MSB = 0x%lx\n", g_Address_LT);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("Left table MVA MSB = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_MSB_REG));
		LOG_INF("Left table MVA = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_REG));
	}
#endif

	// Right table buffer
	g_Address_RT = g_Address_LT + pda_PdaConfig->TableSize;
	pda_PdaConfig->PDA_PDATI_P2_BASE_ADDR = (unsigned int)(g_Address_RT);
	for (i = 0; i < g_PDA_quantity; i++) {
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_MSB_REG,
			(unsigned int)(g_Address_RT >> 32));
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_REG,
			(unsigned int)(g_Address_RT));
	}
#ifdef FOR_DEBUG
	LOG_INF("Right table MVA MSB = 0x%lx\n", g_Address_RT);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("Right table MVA MSB = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_MSB_REG));
		LOG_INF("Right table MVA = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_REG));
	}
#endif

	return ret;
}

static int Get_Output_Addr_From_DMABUF(struct PDA_Data_t_v2 *pda_PdaConfig)
{
	int ret = 0;
	int i = 0;

	// Output buffer
	ret = pda_get_dma_buffer(&g_output_mmu, pda_PdaConfig->FD_Output);
	if (ret < 0) {
		LOG_INF("Output, pda_get_dma_buffer fail!\n");
		return ret;
	}
	if (!pda_output_size_valid(pda_PdaConfig, &g_output_mmu))
		return -EINVAL;
	g_OutputBufferAddr = (unsigned long) sg_dma_address(g_output_mmu.sgt->sgl);
	pda_PdaConfig->PDA_PDAO_P1_BASE_ADDR = (unsigned int)g_OutputBufferAddr;
	for (i = 0; i < g_PDA_quantity; i++) {
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_MSB_REG,
			(unsigned int)(g_OutputBufferAddr >> 32));
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_REG,
			(unsigned int)(g_OutputBufferAddr));
	}
#ifdef FOR_DEBUG
	LOG_INF("Output buffer MVA MSB = 0x%lx\n", g_OutputBufferAddr);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("Output buffer MVA MSB = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_MSB_REG));
		LOG_INF("Output buffer MVA = 0x%x\n",
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_REG));
	}
#endif

	return ret;
}

static void HWDMASettings(struct PDA_Data_t_v2 *pda_PdaConfig)
{
	int i;

	for (i = 0; i < g_PDA_quantity; i++) {
		// --------- Frame setting part -----------
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_0_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_0.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_1_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_1.Raw);
		// need set roi number every process
		// PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_2_REG,
		//     pda_PdaConfig->PDA_FrameSetting.PDA_CFG_2.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_3_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_3.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_4_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_4.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_5_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_5.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_6_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_6.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_7_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_7.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_8_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_8.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_9_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_9.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_10_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_10.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_11_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_11.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_12_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_12.Raw);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_CFG_13_REG,
			pda_PdaConfig->PDA_FrameSetting.PDA_CFG_13.Raw);

		// --------- Input buffer address -------------
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_REG,
			pda_PdaConfig->PDA_PDAI_P1_BASE_ADDR);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_REG,
			pda_PdaConfig->PDA_PDATI_P1_BASE_ADDR);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_REG,
			pda_PdaConfig->PDA_PDAI_P2_BASE_ADDR);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_REG,
			pda_PdaConfig->PDA_PDATI_P2_BASE_ADDR);

		// --------- DMA Secure part -------------
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_SECURE_REG, 0x9daf851f);

		// --------- config setting hard code part --------------
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_STRIDE_REG, 0x580);

		// Left image
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON0_REG, 0x10000134);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON1_REG, 0x104d004d);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON2_REG, 0x009a009a);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON3_REG, 0x00e700e7);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON4_REG, 0x009a009a);

		// Left table
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON0_REG, 0x1000004c);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON1_REG, 0x10130013);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON2_REG, 0x00260026);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON3_REG, 0x00390039);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON4_REG, 0x00260026);

		// Right image
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON0_REG, 0x10000134);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON1_REG, 0x104d004d);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON2_REG, 0x009a009a);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON3_REG, 0x00e700e7);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON4_REG, 0x009a009a);

		// Right table
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON0_REG, 0x1000004c);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON1_REG, 0x10130013);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON2_REG, 0x00260026);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON3_REG, 0x00390039);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON4_REG, 0x00260026);

		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_XSIZE_REG, 0x0000057f);

		// Output
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON0_REG, 0x10000040);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON1_REG, 0x10100010);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON2_REG, 0x00200020);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON3_REG, 0x00300030);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON4_REG, 0x00200020);

		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_DMA_EN_REG, 0x0000001f);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_DMA_RST_REG, 0x1);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_DMA_TOP_REG, 0x407);

		// DCM all off: 0x0000007F
		// DCM all on:  0x00000000
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_DCM_DIS_REG, 0x0000007F);

		//disable dma error irq: 0x00000000
		//enable dma error irq: 0xffff0000
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_ERR_STAT_REG,
			0x00000000);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_ERR_STAT_REG,
			0x00000000);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_ERR_STAT_REG,
			0x00000000);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_ERR_STAT_REG,
			0x00000000);
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_ERR_STAT_REG,
			0x00000000);

		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_TOP_CTL_REG, 0x00000000);

		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_IRQ_TRIG_REG, 0x0);

		// setting read clear
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_ERR_STAT_EN_REG, 0x00000001);

		// read 0x3b4, avoid the impact of previous data
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_ERR_STAT_REG);
		// read clear dma status
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_ERR_STAT_REG);
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_ERR_STAT_REG);
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_ERR_STAT_REG);
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_ERR_STAT_REG);
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_ERR_STAT_REG);
	}
}

static bool pda_fixed_axis(unsigned int origin, unsigned int extent,
		unsigned int divisions, unsigned int index, unsigned int percent,
		unsigned int *start, unsigned int *span)
{
	u64 cell, product, pad, position, length;
	unsigned int sides;

	if (!divisions || index >= divisions || extent < divisions)
		return false;
	cell = extent / divisions;
	product = cell * percent;
	pad = product / 100;
	position = (u64)origin + cell * index;
	sides = index && index != divisions - 1 ? 2 : 1;
	/* Original interior formula divides after multiplying by two. Keep its
	 * rounding without overflowing a wide product at hostile UINT_MAX input.
	 */
	length = cell + pad * sides + (product % 100) * sides / 100;
	if (index) {
		if (position < pad)
			return false;
		position -= pad;
	}
	if (position > UINT_MAX || length > UINT_MAX)
		return false;
	*start = position;
	*span = length;
	return true;
}

static int ProcessROIData(struct PDA_Data_t_v2 *data,
		unsigned int count, unsigned int first, unsigned int fixed, int set)
{
	unsigned int i, xnum, ynum, total, x, y, pr_x;
	u64 iw;

	if (!count || count > PDA_MAXROI_PER_ROUND || fixed > 1)
		return -EINVAL;
	if (!fixed) {
		if (data->ROInumber > PDAROIARRAYMAX || first > data->ROInumber ||
		    count > data->ROInumber - first)
			return -EINVAL;
		for (i = 0; i < count; i++) {
			g_rgn_x_buf[i] = data->rgn_x[first + i];
			g_rgn_y_buf[i] = data->rgn_y[first + i];
			g_rgn_w_buf[i] = data->rgn_w[first + i];
			g_rgn_h_buf[i] = data->rgn_h[first + i];
			g_rgn_iw_buf[i] = data->rgn_iw[first + i];
		}
		return 0;
	}
	if (set < 0 || set >= FIXROIARRAYMAX ||
	    data->nNumerousROI > FIXROIARRAYMAX || set >= data->nNumerousROI)
		return -EINVAL;
	xnum = data->xnum[set];
	ynum = data->ynum[set];
	pr_x = data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PR_XNUM;
	if (!xnum || !ynum || xnum > 1024 / ynum || !pr_x)
		return -EINVAL;
	total = xnum * ynum;
	if (first > total || count > total - first)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		x = (first + i) % xnum;
		y = (first + i) / xnum;
		if (!pda_fixed_axis(data->fix_rgn_x[set], data->fix_rgn_w[set],
				xnum, x, data->woverlap[set], &g_rgn_x_buf[i], &g_rgn_w_buf[i]) ||
		    !pda_fixed_axis(data->fix_rgn_y[set], data->fix_rgn_h[set],
				ynum, y, data->hoverlap[set], &g_rgn_y_buf[i], &g_rgn_h_buf[i]))
			return -EINVAL;
		iw = (u64)g_rgn_w_buf[i] *
			(data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PAT_WIDTH / pr_x);
		if (iw > UINT_MAX)
			return -EINVAL;
		g_rgn_iw_buf[i] = iw;
	}
	return 0;
}

static int CheckDesignLimitation(struct PDA_Data_t_v2 *PDA_Data,
				unsigned int RoiProcNum,
				unsigned int ROIIndex)
{
	int i = 0;
	int nROIIndex = 0;
	int nTempVar = 0;

#ifdef FOR_DEBUG
		LOG_INF("Check Design Limitation\n");
#endif

	if (!RoiProcNum || RoiProcNum > PDA_MAXROI_PER_ROUND ||
	    ROIIndex > 1024 || RoiProcNum > 1024 - ROIIndex) {
		PDA_Data->Status = -28;
		return -EINVAL;
	}

	// Register Range Limitation check
	nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_0.Bits.PDA_WIDTH;
	if (nTempVar < 20 || nTempVar > 8191) {
		LOG_INF("Frame width (%d) out of range\n", nTempVar);
		PDA_Data->Status = -12;
		return -1;
	}

	nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_0.Bits.PDA_HEIGHT;
	if (nTempVar < 4 || nTempVar > 8191) {
		LOG_INF("Frame height (%d) out of range\n", nTempVar);
		PDA_Data->Status = -13;
		return -1;
	}

	nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PR_XNUM;
	if (nTempVar < 1 || nTempVar > 16) {
		LOG_INF("PDA_PR_XNUM (%d) out of range\n", nTempVar);
		PDA_Data->Status = -14;
		return -1;
	}

	nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PR_YNUM;
	if (nTempVar < 1 || nTempVar > 16) {
		LOG_INF("PDA_PR_YNUM (%d) out of range\n", nTempVar);
		PDA_Data->Status = -15;
		return -1;
	}

	nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PAT_WIDTH;
	if (nTempVar < 1 || nTempVar > 512) {
		LOG_INF("PDA_PAT_WIDTH (%d) out of range\n", nTempVar);
		PDA_Data->Status = -16;
		return -1;
	}

	nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_RNG_ST;
	if (nTempVar > 40) {
		LOG_INF("PDA_RNG_ST (%d) out of range\n", nTempVar);
		PDA_Data->Status = -17;
		return -1;
	}

	nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_2.Bits.PDA_TBL_STRIDE;
	if (nTempVar < 5 || nTempVar > 2048) {
		LOG_INF("PDA_TBL_STRIDE (%d) out of range\n", nTempVar);
		PDA_Data->Status = -18;
		return -1;
	}

	// frame constraint
	if (PDA_Data->PDA_FrameSetting.PDA_CFG_0.Bits.PDA_WIDTH % 4 != 0) {
		LOG_INF("Frame width must be multiple of 4\n");
		PDA_Data->Status = -4;
		return -1;
	}

	// ROI constraint
	for (i = 0; i < RoiProcNum; i++) {
		nROIIndex = i + ROIIndex;

#ifdef FOR_DEBUG
		LOG_INF("nROIIndex:%d, i:%d\n", nROIIndex, i);
#endif

		if (g_rgn_w_buf[i] % 4 != 0) {
			LOG_INF("ROI_%d width(%d) must be multiple of 4\n",
				nROIIndex,
				g_rgn_w_buf[i]);
			PDA_Data->Status = -5;
			return -1;
		}

		if (g_rgn_w_buf[i] > 3280) {
			LOG_INF("ROI_%d width(%d) must be less than 3280\n",
				nROIIndex,
				g_rgn_w_buf[i]);
			PDA_Data->Status = -6;
			return -1;
		}

		if (g_rgn_x_buf[i] % 2 != 0) {
			LOG_INF("ROI_%d xoffset(%d) must be multiple of 2\n",
				nROIIndex,
				g_rgn_x_buf[i]);
			PDA_Data->Status = -7;
			return -1;
		}

		if (g_rgn_iw_buf[i] < 41) {
			LOG_INF("ROI_%d IW(%d) must be greater than 41\n",
				nROIIndex,
				g_rgn_iw_buf[i]);
			PDA_Data->Status = -8;
			return -1;
		}

		nTempVar = (1 << PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_BIN_FCTR);
		if (g_rgn_h_buf[i] % nTempVar != 0) {
			LOG_INF("ROI_%d height(%d) must be multiple of Binning number(%d)\n",
				i,
				g_rgn_h_buf[i],
				nTempVar);
			PDA_Data->Status = -9;
			return -1;
		}

		// ROI boundary must be on the boundary of patch
		nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PR_XNUM;
		if (g_rgn_x_buf[i] % nTempVar != 0) {
			LOG_INF("ROI_%d ROI boundary must be on the boundary of patch\n",
				nROIIndex);
			LOG_INF("ROI_%d_x: %d, PDA_PR_XNUM: %d\n",
				nROIIndex,
				g_rgn_x_buf[i],
				nTempVar);
			PDA_Data->Status = -10;
			return -1;
		}

		if (g_rgn_w_buf[i] % nTempVar != 0) {
			LOG_INF("ROI_%d ROI boundary must be on the boundary of patch\n",
				nROIIndex);
			LOG_INF("ROI_%d_w: %d, PDA_PR_XNUM: %d\n",
				nROIIndex,
				g_rgn_w_buf[i],
				nTempVar);
			PDA_Data->Status = -10;
			return -1;
		}

		nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PR_YNUM;
		if (g_rgn_y_buf[i] % nTempVar != 0) {
			LOG_INF("ROI_%d ROI boundary must be on the boundary of patch\n",
				nROIIndex);
			LOG_INF("ROI_%d_y: %d, PDA_PR_YNUM: %d\n",
				nROIIndex,
				g_rgn_y_buf[i],
				nTempVar);
			PDA_Data->Status = -10;
			return -1;
		}

		if (g_rgn_h_buf[i] % nTempVar != 0) {
			LOG_INF("ROI_%d ROI boundary must be on the boundary of patch\n",
				nROIIndex);
			LOG_INF("ROI_%d_h: %d, PDA_PR_YNUM: %d\n",
				nROIIndex,
				g_rgn_h_buf[i],
				nTempVar);
			PDA_Data->Status = -10;
			return -1;
		}

		// ROI can't exceed the image region
		nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_0.Bits.PDA_WIDTH;
		if (g_rgn_x_buf[i] > nTempVar || g_rgn_w_buf[i] > nTempVar - g_rgn_x_buf[i]) {
			LOG_INF("ROI_%d ROI exceed the image region\n", nROIIndex);
			PDA_Data->Status = -11;
			return -1;
		}

		nTempVar = PDA_Data->PDA_FrameSetting.PDA_CFG_0.Bits.PDA_HEIGHT;
		if (g_rgn_y_buf[i] > nTempVar || g_rgn_h_buf[i] > nTempVar - g_rgn_y_buf[i]) {
			LOG_INF("ROI_%d ROI exceed the image region\n", nROIIndex);
			PDA_Data->Status = -11;
			return -1;
		}

		// Register Range Limitation check
		if (g_rgn_x_buf[i] > 8191) {
			LOG_INF("ROI_X_%d (%d) out of range\n", nROIIndex, g_rgn_x_buf[i]);
			PDA_Data->Status = -19;
			return -1;
		}

		if (g_rgn_y_buf[i] > 8191) {
			LOG_INF("ROI_Y_%d (%d) out of range\n", nROIIndex, g_rgn_y_buf[i]);
			PDA_Data->Status = -20;
			return -1;
		}

		if (g_rgn_w_buf[i] < 20) {
			LOG_INF("ROI_W_%d (%d) out of range\n", nROIIndex, g_rgn_w_buf[i]);
			PDA_Data->Status = -21;
			return -1;
		}

		if (g_rgn_h_buf[i] < 4 || g_rgn_h_buf[i] > 4092) {
			LOG_INF("ROI_H_%d (%d) out of range\n", nROIIndex, g_rgn_h_buf[i]);
			PDA_Data->Status = -22;
			return -1;
		}

		if (g_rgn_iw_buf[i] > 3280) {
			LOG_INF("ROI_IW_%d (%d) out of range, w:%d, pat_width:%d\n",
				nROIIndex,
				g_rgn_iw_buf[i],
				g_rgn_w_buf[i],
				PDA_Data->PDA_FrameSetting.PDA_CFG_1.Bits.PDA_PAT_WIDTH);
			PDA_Data->Status = -23;
			return -1;
		}
	}

	return 0;
}

static int pda_validate_geometry(struct PDA_Data_t_v2 *data)
{
	unsigned int first, count, total, set;

	if (!pda_request_roi_valid(data)) {
		data->Status = -28;
		return -EINVAL;
	}
	for (first = 0; first < data->ROInumber; first += count) {
		count = min_t(unsigned int, PDA_MAXROI_PER_ROUND, data->ROInumber - first);
		if (ProcessROIData(data, count, first, MFALSE, 0)) {
			data->Status = -24;
			return -EINVAL;
		}
		if (CheckDesignLimitation(data, count, first))
			return -EINVAL;
	}
	for (set = 0; set < data->nNumerousROI; set++) {
		total = data->xnum[set] * data->ynum[set];
		for (first = 0; first < total; first += count) {
			count = min_t(unsigned int, PDA_MAXROI_PER_ROUND, total - first);
			if (ProcessROIData(data, count, first, MTRUE, set)) {
				data->Status = -24;
				return -EINVAL;
			}
			if (CheckDesignLimitation(data, count, first))
				return -EINVAL;
		}
	}
	/* Original PDA/DPDA/v2 clients all store one16-bit image sample per
	 * frame pixel. Table grid units are independent: do not infer its pitch.
	 */
	if ((u64)data->ImageSize <
	    (u64)data->PDA_FrameSetting.PDA_CFG_0.Bits.PDA_WIDTH *
		data->PDA_FrameSetting.PDA_CFG_0.Bits.PDA_HEIGHT * 2 ||
	    (data->ImageSize & 1) || !data->TableSize || (data->TableSize & 1)) {
		data->Status = -26;
		return -EINVAL;
	}

	return 0;
}

static void FillRegSettings(struct PDA_Data_t_v2 *pda_PdaConfig,
				unsigned int RoiProcNum,
				unsigned long OuputAddr,
				int PDA_Index)
{
	int RegIndex = 14;
	int ROI_MAX_INDEX = RoiProcNum-1;
	int pair = 0;

	// modify roi number register, change [11:6] bit to RoiProcNum
	pda_PdaConfig->PDA_FrameSetting.PDA_CFG_2.Bits.PDA_RGN_NUM = RoiProcNum;

	// roi number register setting
	PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_CFG_2_REG,
		pda_PdaConfig->PDA_FrameSetting.PDA_CFG_2.Raw);

	// 1024 ROI data sequentially fill to PDA_CFG[14] ~ PDA_CFG[126]
	for (pair = 0; pair <= ROI_MAX_INDEX; pair += 2) {
		PDA_WR32((PDA_devs[PDA_Index].m_pda_base + 0x004*(RegIndex++)),
			g_rgn_y_buf[pair]*65536 + g_rgn_x_buf[pair]);
		PDA_WR32((PDA_devs[PDA_Index].m_pda_base + 0x004*(RegIndex++)),
			g_rgn_h_buf[pair]*65536 + g_rgn_w_buf[pair]);
		if (pair == ROI_MAX_INDEX && pair%2 == 0) {
			PDA_WR32((PDA_devs[PDA_Index].m_pda_base + 0x004*(RegIndex++)),
				g_rgn_iw_buf[pair]);
		} else {
			PDA_WR32((PDA_devs[PDA_Index].m_pda_base + 0x004*(RegIndex++)),
				g_rgn_x_buf[pair+1]*65536 + g_rgn_iw_buf[pair]);
			PDA_WR32((PDA_devs[PDA_Index].m_pda_base + 0x004*(RegIndex++)),
				g_rgn_w_buf[pair+1]*65536 + g_rgn_y_buf[pair+1]);
			PDA_WR32((PDA_devs[PDA_Index].m_pda_base + 0x004*(RegIndex++)),
				g_rgn_iw_buf[pair+1]*65536 + g_rgn_h_buf[pair+1]);
		}
	}

	// output buffer address setting
	PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDAO_P1_BASE_ADDR_MSB_REG,
		(unsigned int)(OuputAddr >> 32));
	PDA_WR32(PDA_devs[PDA_Index].m_pda_base + PDA_PDAO_P1_BASE_ADDR_REG,
		(unsigned int)(OuputAddr));

#ifdef FOR_DEBUG
		LOG_INF("Fill Register Settings done\n");
#endif
}

static void LOGHWRegister(int i)
{
	LOG_INF("CFG_0/1/2/3/4/5/6: 0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_0_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_1_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_2_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_3_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_4_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_5_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_6_REG));

	LOG_INF("CFG_7/8/9/10/11/12/13: 0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_7_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_8_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_9_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_10_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_11_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_12_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_13_REG));

	LOG_INF("CFG_14/15/16/17/18/19/20: 0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_14_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_15_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_16_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_17_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_18_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_19_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_20_REG));

	LOG_INF("CFG_21/22/23/24/25/26/27/28: 0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_21_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_22_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_23_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_24_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_25_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_26_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_27_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_CFG_28_REG));

	LOG_INF("I_P1/TI_P1/I_P2/TI_P2/Out: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_REG));

	LOG_INF("SECURE/I_STRIDE/O_P1_XSIZE/TILE_STATUS: 0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_SECURE_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_STRIDE_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_XSIZE_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_TILE_STATUS_REG));

	LOG_INF("PDAI_P1_CON0/CON1/CON2/CON3/CON4: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON0_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON1_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON2_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON3_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_CON4_REG));

	LOG_INF("PDATI_P1_CON0/CON1/CON2/CON3/CON4: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON0_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON1_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON2_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON3_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_CON4_REG));

	LOG_INF("PDAI_P2_CON0/CON1/CON2/CON3/CON4: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON0_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON1_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON2_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON3_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_CON4_REG));

	LOG_INF("PDATI_P2_CON0/CON1/CON2/CON3/CON4: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON0_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON1_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON2_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON3_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_CON4_REG));

	LOG_INF("PDAO_P1_CON0/CON1/CON2/CON3/CON4: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON0_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON1_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON2_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON3_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_CON4_REG));

	LOG_INF("DMA_EN/DMA_RST/DMA_TOP/DCM_DIS/DCM_ST: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_DMA_EN_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_DMA_RST_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_DMA_TOP_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_DCM_DIS_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_DCM_ST_REG));

	LOG_INF("[ERR_STAT]I_P1/TI_P1/I_P2/TI_P2/O_P1: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_ERR_STAT_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_ERR_STAT_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_ERR_STAT_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_ERR_STAT_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_ERR_STAT_REG));

	LOG_INF("ERR_STAT_EN/ERR_STAT/TOP_CTL: 0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_ERR_STAT_EN_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_ERR_STAT_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_TOP_CTL_REG));

	LOG_INF("IRQ_TRIG/PDAO_DMA_EXISTED_ECO: 0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_IRQ_TRIG_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_DMA_EXISTED_ECO_REG));

	LOG_INF("[MSB]I_P1/TI_P1/I_P2/TI_P2/O_P1: 0x%x/0x%x/0x%x/0x%x/0x%x\n",
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_MSB_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_MSB_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_MSB_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_MSB_REG),
		PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_MSB_REG));
}

static void TF_dump_log(int hw_trigger_num)
{
	int i = 0;
	int sel_index = 0;

	unsigned int Debug_Sel[] = {0x00008120, 0x0000400e, 0x0000c000,
		0x11000000, 0x12000000, 0x13000000, 0x14000000, 0x15000000, 0x16000000,
		0x14000000, 0x14100000, 0x14200000, 0x14300000, 0x14400000, 0x14500000,
		0x21000000, 0x22000000, 0x23000000, 0x24000000, 0x25000000,
		0x24000000, 0x24100000, 0x24200000, 0x24300000, 0x24400000, 0x24500000,
		0x31000000, 0x32000000, 0x33000000, 0x34000000, 0x35000000,
		0x34000000, 0x34100000, 0x34200000, 0x34300000, 0x34400000, 0x34500000,
		0x41000000, 0x42000000, 0x43000000, 0x44000000, 0x45000000,
		0x44000000, 0x44100000, 0x44200000, 0x44300000, 0x44400000, 0x44500000,
		0x51000000, 0x52000000, 0x53000000, 0x54000000, 0x55000000,
		0x54000000, 0x54100000, 0x54200000, 0x54300000, 0x54400000, 0x54500000};
	int Length_Arr = sizeof(Debug_Sel)/sizeof(*Debug_Sel);

#ifdef SMI_LOG
	// SMI log
	mtk_smi_dbg_hang_detect("PDA device");
#endif

	// check debug data
	for (i = 0; i < hw_trigger_num; i++) {
		for (sel_index = 0; sel_index < Length_Arr; ++sel_index) {
			PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_DEBUG_SEL_REG,
				Debug_Sel[sel_index]);
			LOG_INF("PDA_%d DEBUG_SEL/DEBUG_DATA: 0x%x/0x%x\n",
				i, PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_DEBUG_SEL_REG),
				PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_DEBUG_DATA_REG));
		}
	}

	// check hw register setting
	for (i = 0; i < hw_trigger_num; i++) {
		LOG_INF("[outer] PDA_%d register LOG +++++\n", i);
		LOGHWRegister(i);
	}
}

static void TimeoutHandler(int hw_trigger_num)
{
	TF_dump_log(hw_trigger_num);
}

#ifdef PDA_TF_DUMP_71_3
int pda_tfault_callback(int port,
	dma_addr_t mva, void *data)
{
	int i = 0;

	LOG_INF("User:PDA_PDAI_P1_BASE_ADDR: 0x%lx\n", g_Address_LI);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("PDA%d:I_P1_MSB/I_P1: 0x%x/0x%x\n", i,
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_MSB_REG),
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_REG));
	}

	LOG_INF("User:PDA_PDATI_P1_BASE_ADDR: 0x%lx\n", g_Address_LT);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("PDA%d:TI_P1_MSB/TI_P1: 0x%x/0x%x\n", i,
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_MSB_REG),
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_REG));
	}

	LOG_INF("User:PDA_PDAI_P2_BASE_ADDR: 0x%lx\n", g_Address_RI);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("PDA%d:I_P2_MSB/I_P2: 0x%x/0x%x\n", i,
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_MSB_REG),
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_REG));
	}

	LOG_INF("User:PDA_PDATI_P2_BASE_ADDR: 0x%lx\n", g_Address_RT);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("PDA%d:TI_P2_MSB/TI_P2: 0x%x/0x%x\n", i,
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_MSB_REG),
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_REG));
	}

	LOG_INF("User:PDA_PDAO_P1_BASE_ADDR: 0x%lx\n", g_OutputBufferAddr);
	for (i = 0; i < g_PDA_quantity; i++) {
		LOG_INF("PDA%d:O_P1_MSB/O_P1: 0x%x/0x%x\n", i,
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_MSB_REG),
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_REG));
	}

	LOG_INF("ImageSize/TableSize/OutputSize/ROInumber: %d/%d/%d/%d\n",
		g_pda_Pdadata.ImageSize,
		g_pda_Pdadata.TableSize,
		g_pda_Pdadata.OutputSize,
		g_pda_Pdadata.ROInumber);

	LOG_INF("FD_L_Img/R_Img/L_Tbl/R_Tbl/Out: %d/%d/%d/%d/%d\n",
		g_pda_Pdadata.FD_L_Image,
		g_pda_Pdadata.FD_R_Image,
		g_pda_Pdadata.FD_L_Table,
		g_pda_Pdadata.FD_R_Table,
		g_pda_Pdadata.FD_Output);

	LOG_INF("Status/Timeout/nNumerousROI: %d/%d/%d\n",
		g_pda_Pdadata.Status,
		g_pda_Pdadata.Timeout,
		g_pda_Pdadata.nNumerousROI);

	LOG_INF("PDA_CFG_0/1/2/3/4/5/6: 0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_0.Raw,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_1,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_2,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_3,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_4,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_5,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_6);

	LOG_INF("PDA_CFG_7/8/9/10/11/12/13: 0x%x/0x%x/0x%x/0x%x/0x%x/0x%x/0x%x\n",
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_7,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_8,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_9,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_10,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_11,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_12,
		g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_13);

	for (i = 0; i < g_pda_Pdadata.ROInumber; ++i) {
		LOG_INF("rgn_x/y/h/w/iw[%d]: 0x%x/0x%x/0x%x/0x%x/0x%x\n", i,
			g_pda_Pdadata.rgn_x[i],
			g_pda_Pdadata.rgn_y[i],
			g_pda_Pdadata.rgn_h[i],
			g_pda_Pdadata.rgn_w[i],
			g_pda_Pdadata.rgn_iw[i]);
	}

	// dump data
	TF_dump_log(g_PDA_quantity);

	return 1;
}
#endif

static void pda_execute(int hw_trigger_num)
{
	int i;

#ifdef FOR_DEBUG
	LOG_INF("+\n");
#endif

	for (i = 0; i < hw_trigger_num; i++) {
		// PDA_TOP_CTL set 1'b1 to bit3, to load register from double buffer
		PDA_WR32(PDA_devs[i].m_pda_base  + PDA_PDA_TOP_CTL_REG, PDA_DOUBLE_BUFFER);

		// make sure all the pda setting take effect
		wmb();

		/* Publish possible DMA exposure before issuing the trigger. */
		pda_dma_active_mask |= BIT(i);
		// PDA_TOP_CTL set 1'b1 to bit1, to trigger sof
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_TOP_CTL_REG, PDA_TRIGGER);

		// make sure all the pda setting take effect
		wmb();

		// write 0 after trigger
		PDA_WR32(PDA_devs[i].m_pda_base + PDA_PDA_TOP_CTL_REG, PDA_CLEAR_REG);

#ifdef GET_PDA_TIME
		if (i == 0)
			ktime_get_real_ts64(&pda1_done_b);
		else
			ktime_get_real_ts64(&pda2_done_b);
#endif
	}

#ifdef FOR_DEBUG
	LOG_INF("-\n");
#endif
}

static int check_pda_status(int hw_trigger_num)
{
	int i = 0;

	for (i = 0; i < hw_trigger_num; i++) {
		if (READ_ONCE(PDA_devs[i].HWstatus) == 0) {
#ifdef FOR_DEBUG
			LOG_INF("PDA%d HWstatus = %d\n", i, READ_ONCE(PDA_devs[i].HWstatus));
#endif
			return 0;
		}
	}
	return 1;
}

static inline unsigned int pda_ms_to_jiffies(unsigned int ms)
{
	return ((ms * HZ + 512) >> 10);
}

static signed int pda_wait_irq(struct PDA_Data_t_v2 *pda_data, int hw_trigger_num)
{
	int ret = 0, i = 0;

	/* start to wait signal */
	ret = wait_event_interruptible_timeout(g_wait_queue_head,
						check_pda_status(hw_trigger_num),
						pda_ms_to_jiffies(pda_data->Timeout));

#ifdef GET_PDA_TIME
	ktime_get_real_ts64(&time_end);
#endif
	if (ret == 0) {
		TimeoutHandler(hw_trigger_num);

		// timeout error
		LOG_INF("wait_event_interruptible_timeout Fail\n");
		pda_data->Status = -2;
		return -1;
	} else if (ret < 0) {
		LOG_INF("Interrupted by a signal\n");
		pda_data->Status = -3;
		return -1;
	}

	// pda status
	pda_data->Status = 1;

	// update status to user
	for (i = 0; i < hw_trigger_num; i++) {
		if (READ_ONCE(PDA_devs[i].HWstatus) < 0) {
			pda_data->Status = READ_ONCE(PDA_devs[i].HWstatus);

			LOG_INF("PDA%d HW error (%d)", i, READ_ONCE(PDA_devs[i].HWstatus));

			LOG_INF("PDA%d PDA_PDAI_P1_ERR_STAT_REG = 0x%x",
				i, PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_ERR_STAT_REG));
			LOG_INF("PDA%d PDA_PDATI_P1_ERR_STAT_REG = 0x%x",
				i, PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_ERR_STAT_REG));
			LOG_INF("PDA%d PDA_PDAI_P2_ERR_STAT_REG = 0x%x",
				i, PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_ERR_STAT_REG));
			LOG_INF("PDA%d PDA_PDATI_P2_ERR_STAT_REG = 0x%x",
				i, PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_ERR_STAT_REG));
			LOG_INF("PDA%d PDA_PDAO_P1_ERR_STAT_REG = 0x%x",
				i, PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_ERR_STAT_REG));

			/* Common retirement requires the DMA-reset acknowledgement. */
		}
	}

	return pda_data->Status < 0 ? -EIO : ret;
}

static irqreturn_t pda_irqhandle(signed int Irq, void *DeviceId)
{
	unsigned int nPdaStatus = 0;
	int status = 1;

	// read pda status
	nPdaStatus = PDA_RD32(PDA_devs[0].m_pda_base + PDA_PDA_ERR_STAT_REG) &
		PDA_STATUS_REG;

	if (!nPdaStatus)
		return IRQ_NONE;

#ifdef FOR_DEBUG
	LOG_INF("PDA0 PDA_PDA_ERR_STAT_REG = 0x%x", nPdaStatus);
#endif

	// for WCL=1 case, write 1 to clear pda done status
	// PDA_WR32(PDA_devs[0].m_pda_base + PDA_PDA_ERR_STAT_REG, 0x00000001);


#ifdef GET_PDA_TIME
	ktime_get_real_ts64(&pda1_done_e);
#endif

#ifdef CHECK_IRQ_COUNT
	++g_PDA0_IRQCount;
	if (g_PDA0_IRQCount > g_reasonable_IRQCount) {
		status = -29;
	}
#endif

	WRITE_ONCE(PDA_devs[0].HWstatus, status);

	// wake up user space WAIT_IRQ flag
	wake_up_interruptible(&g_wait_queue_head);
	return IRQ_HANDLED;
}

static irqreturn_t pda2_irqhandle(signed int Irq, void *DeviceId)
{
	unsigned int nPdaStatus = 0;
	int status = 1;

	// read pda status
	nPdaStatus = PDA_RD32(PDA_devs[1].m_pda_base + PDA_PDA_ERR_STAT_REG) &
		PDA_STATUS_REG;

	if (!nPdaStatus)
		return IRQ_NONE;

#ifdef FOR_DEBUG
	LOG_INF("PDA1 PDA_PDA_ERR_STAT_REG = 0x%x", nPdaStatus);
#endif

	// for WCL=1 case, write 1 to clear pda done status
	// PDA_WR32(PDA_devs[1].m_pda_base + PDA_PDA_ERR_STAT_REG, 0x00000001);


#ifdef GET_PDA_TIME
	ktime_get_real_ts64(&pda2_done_e);
#endif

#ifdef CHECK_IRQ_COUNT
	++g_PDA1_IRQCount;
	if (g_PDA1_IRQCount > g_reasonable_IRQCount) {
		status = -29;
	}
#endif

	WRITE_ONCE(PDA_devs[1].HWstatus, status);

	// wake up user space WAIT_IRQ flag
	wake_up_interruptible(&g_wait_queue_head);
	return IRQ_HANDLED;
}

static int PDAProcessFunction(unsigned int nUserROINumber,
				unsigned int nROIcount,
				unsigned int isFixROI,
				int p_index)
{
	int i = 0;
	unsigned int nOneRoundProcROI = 0;
	int hw_trigger_num = 0;
	unsigned int nRemainder = 0, nFactor = 0;
	unsigned int nCurrentProcRoiIndex = 0;
	unsigned long nOutputAddr = 0;
	long nirqRet = 0;
	unsigned int CheckAddress = 0, CheckAddressMSB = 0;

	while (nROIcount > 0) {

		// read 0x3b4, avoid the impact of previous data
		// LOG_INF("PDA status before process = %d\n",
		//	PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDA_ERR_STAT_REG));
#ifdef FOR_DEBUG
		LOG_INF("nROIcount = %d\n", nROIcount);
#endif
		// reset HW status
		for (i = 0; i < g_PDA_quantity; i++)
			WRITE_ONCE(PDA_devs[i].HWstatus, 0);

		// reset local variable
		nOneRoundProcROI = 0;

		//reset hw trigger number
		hw_trigger_num = g_PDA_quantity;

		// assign strategy, used for multi-engine
		if (nROIcount >= (PDA_MAXROI_PER_ROUND * g_PDA_quantity)) {
			for (i = 0; i < g_PDA_quantity; i++) {
				g_CurrentProcRoiNum[i] = PDA_MAXROI_PER_ROUND;
				nOneRoundProcROI += g_CurrentProcRoiNum[i];
#ifdef FOR_DEBUG
				LOG_INF("g_CurrentProcRoiNum[%d] = %d\n",
					i, g_CurrentProcRoiNum[i]);
				LOG_INF("OneRoundProcROI = %d\n", nOneRoundProcROI);
#endif
			}
		} else {
			if (g_PDA_quantity == 0) {
				LOG_INF("Fail: g_PDA_quantity is zero\n");
				return -1;
			}
			nRemainder = nROIcount % g_PDA_quantity;
			nFactor = nROIcount / g_PDA_quantity;

			for (i = 0; i < g_PDA_quantity; i++) {
				g_CurrentProcRoiNum[i] = nFactor;
				if (nRemainder > 0) {
					g_CurrentProcRoiNum[i]++;
					nRemainder--;
				}
				nOneRoundProcROI += g_CurrentProcRoiNum[i];
#ifdef FOR_DEBUG
				LOG_INF("g_CurrentProcRoiNum[%d] = %d\n",
					i, g_CurrentProcRoiNum[i]);
				LOG_INF("OneRoundProcROI = %d\n", nOneRoundProcROI);
#endif
			}
		}

		// data preparing
		for (i = 0; i < g_PDA_quantity; i++) {
			// current process ROI index
			if (i == 0)
				nCurrentProcRoiIndex = (nUserROINumber - nROIcount);
			else if (i > 0)
				nCurrentProcRoiIndex += g_CurrentProcRoiNum[i - 1];
			else
				LOG_INF("Index is out of range, i = %d\n", i);
#ifdef FOR_DEBUG
			LOG_INF("nCurrentProcRoiIndex = %d\n",
				nCurrentProcRoiIndex);
#endif

			if (g_CurrentProcRoiNum[i] == 0) {
				hw_trigger_num = i;
#ifdef FOR_DEBUG
				LOG_INF("assign roi number is zero, no need to process\n");
#endif
				break;
			}

#ifdef FOR_DEBUG
			LOG_INF("i:%d, g_CurrentProcRoiNum:%d, nCurrentProcRoiIndex:%d\n",
				i,
				g_CurrentProcRoiNum[i],
				nCurrentProcRoiIndex);
#endif

			// calculate 1024 ROI data
			if (ProcessROIData(&g_pda_Pdadata,
					g_CurrentProcRoiNum[i],
					nCurrentProcRoiIndex,
					isFixROI,
					p_index) < 0) {
				LOG_INF("ProcessROIData Fail\n");
				g_pda_Pdadata.Status = -24;
				return -1;
			}

			if (CheckDesignLimitation(&g_pda_Pdadata,
					g_CurrentProcRoiNum[i],
					nCurrentProcRoiIndex) < 0) {
				LOG_INF("CheckDesignLimitation Fail\n");
				return -1;
			}

			// output address is equal to
			// total ROI number multiple by 1408
			nOutputAddr = g_OutputBufferAddr;
			nOutputAddr += nCurrentProcRoiIndex * OUT_BYTE_PER_ROI;

			if (g_OutputBufferOffset + (g_CurrentProcRoiNum[i]*OUT_BYTE_PER_ROI) >
				g_pda_Pdadata.OutputSize) {
				LOG_INF("fail, output buffer out of range\n");
				LOG_INF("Current output buffer addr: 0x%lx\n",
					nOutputAddr);
				LOG_INF("Base output buffer addr: 0x%lx\n",
					nOutputAddr);
				LOG_INF("PDA%d ROI process num: %d\n",
					i, g_CurrentProcRoiNum[i]);
				LOG_INF("Output buffer size: %d\n",
					g_pda_Pdadata.OutputSize);
				LOG_INF("Current process ROI index: %d\n",
					nCurrentProcRoiIndex);
				LOG_INF("nUserROINumber: %d\n",
					nUserROINumber);
				LOG_INF("nROIcount: %d\n", nROIcount);
				g_pda_Pdadata.Status = -30;
				return -1;
			}

#ifdef FOR_DEBUG
			LOG_INF("(nOutputAddr+g_OutputBufferOffset): 0x%lx\n",
				(nOutputAddr+g_OutputBufferOffset));
#endif

			FillRegSettings(&g_pda_Pdadata,
				g_CurrentProcRoiNum[i],
				(nOutputAddr+g_OutputBufferOffset),
				i);
		}

		// check input/output buffer address is valid
		for (i = 0; i < hw_trigger_num; i++) {
			CheckAddress =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_REG);
			CheckAddressMSB =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P1_BASE_ADDR_MSB_REG);
			if (CheckAddress == 0 && CheckAddressMSB == 0) {
				LOG_INF("PDA_%d PDA_PDAI_P1_BASE_ADDR is zero\n", i);
				g_pda_Pdadata.Status = -30;
				return -1;
			}

			CheckAddress =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_REG);
			CheckAddressMSB =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P1_BASE_ADDR_MSB_REG);
			if (CheckAddress == 0 && CheckAddressMSB == 0) {
				LOG_INF("PDA_%d PDA_PDATI_P1_BASE_ADDR is zero\n", i);
				g_pda_Pdadata.Status = -30;
				return -1;
			}

			CheckAddress =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_REG);
			CheckAddressMSB =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAI_P2_BASE_ADDR_MSB_REG);
			if (CheckAddress == 0 && CheckAddressMSB == 0) {
				LOG_INF("PDA_%d PDA_PDAI_P2_BASE_ADDR is zero\n", i);
				g_pda_Pdadata.Status = -30;
				return -1;
			}

			CheckAddress =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_REG);
			CheckAddressMSB =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDATI_P2_BASE_ADDR_MSB_REG);
			if (CheckAddress == 0 && CheckAddressMSB == 0) {
				LOG_INF("PDA_%d PDA_PDATI_P2_BASE_ADDR is zero\n", i);
				g_pda_Pdadata.Status = -30;
				return -1;
			}

			CheckAddress =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_REG);
			CheckAddressMSB =
			PDA_RD32(PDA_devs[i].m_pda_base + PDA_PDAO_P1_BASE_ADDR_MSB_REG);
			if (CheckAddress == 0 && CheckAddressMSB == 0) {
				LOG_INF("PDA_%d PDA_PDAO_P1_BASE_ADDR is zero\n", i);
				g_pda_Pdadata.Status = -30;
				return -1;
			}
		}

		// trigger PDA work
		pda_execute(hw_trigger_num);

		nirqRet = pda_wait_irq(&g_pda_Pdadata, hw_trigger_num);

		if (nirqRet < 0) {
			LOG_INF("pda_wait_irq Fail (%ld)\n", nirqRet);
			return nirqRet;
		}

		// update roi count which needed to process
		nROIcount -= nOneRoundProcROI;
	}
	return 1;
}

static long pda_ioctl_locked(struct file *a_pstFile,
			unsigned int a_u4Command,
			unsigned long a_u4Param)
{
	long nRet = 0;
	int nROIcount = 0;
	unsigned int nUserROINumber = 0;
	int i;
	int ret = 0;
	struct PDA_Init_Data Init_Data;

	if (g_u4EnableClockCount == 0 || g_PDA_quantity == 0) {
		LOG_INF("Cannot process without enable pda clock or no PDA support\n");
		return -1;
	}

	switch (a_u4Command) {
	case PDA_RESET:
		if (pda_dma_lease || pda_dma_active_mask) {
			nRet = pda_retire_dma();
		} else {
			for (i = 0; i < g_PDA_quantity; i++) {
				ret = pda_reset(i);
				if (!ret)
					ret = pda_nontransaction_reset(i);
				if (ret)
					nRet = ret;
			}
		}
		break;
	case PDA_GET_VERSION:

		if (copy_from_user(&Init_Data,
				   (void *)a_u4Param,
				   sizeof(struct PDA_Init_Data)) != 0) {
			LOG_INF("PDA_GET_VERSION copy_from_user failed\n");
			nRet = -EFAULT;
			break;
		}

		Init_Data.Kversion = KERNEL_VERSION;
		LOG_INF("kernel version: %d\n", Init_Data.Kversion);

		if (copy_to_user((void *)a_u4Param,
		    &Init_Data,
		    sizeof(struct PDA_Init_Data)) != 0) {
			LOG_INF("PDA_GET_VERSION copy_to_user failed\n");
			nRet = -EFAULT;
		}

		break;
	case PDA_ENQUE_WAITIRQ_V2:
		if (pda_dma_quarantined || pda_dma_active_mask || pda_dma_lease)
			return -EBUSY;

#ifdef GET_PDA_TIME
		ktime_get_real_ts64(&total_time_begin);
#endif


		// reset HW status
		for (i = 0; i < g_PDA_quantity; i++)
			WRITE_ONCE(PDA_devs[i].HWstatus, 0);

#ifdef CHECK_IRQ_COUNT
		// reset PDA0/PDA1 IRQ count
		g_PDA0_IRQCount = 0;
		g_PDA1_IRQCount = 0;
#ifdef FOR_DEBUG
		LOG_INF("PDA0_IRQCount = %d, PDA1_IRQCount = %d\n",
			g_PDA0_IRQCount, g_PDA1_IRQCount);
#endif
#endif

		// reset output buffer address offset
		g_OutputBufferOffset = 0;

		if (copy_from_user(&g_pda_Pdadata,
				   (void *)a_u4Param,
				   sizeof(struct PDA_Data_t_v2)) != 0) {
			LOG_INF("PDA_ENQUE_WAITIRQ copy_from_user failed\n");
			nRet = -EFAULT;
			break;
		}

		ret = !pda_request_roi_valid(&g_pda_Pdadata);
		if (ret) {
			g_pda_Pdadata.Status = -28;
			LOG_INF("ROI number out of range, ROInumber/nNumerousROI:%d/%d\n",
				g_pda_Pdadata.ROInumber,
				g_pda_Pdadata.nNumerousROI);
			LOG_INF("rgn:%d,fix:%d,xnum:%d,siz:%d,FD:%d,cfg0:0x%x,sta:%d/%d\n",
				g_pda_Pdadata.rgn_h[0],
				g_pda_Pdadata.fix_rgn_h[0],
				g_pda_Pdadata.xnum[0],
				g_pda_Pdadata.OutputSize,
				g_pda_Pdadata.FD_Output,
				g_pda_Pdadata.PDA_FrameSetting.PDA_CFG_0.Raw,
				g_pda_Pdadata.Status,
				g_pda_Pdadata.Timeout);
			goto EXIT_WITHOUT_FREE_IOVA;
		}

		ret = pda_validate_geometry(&g_pda_Pdadata);
		if (ret)
			goto EXIT_WITHOUT_FREE_IOVA;

		// MRAW PDA reset
		for (i = 0; i < g_PDA_quantity; i++) {
			ret = pda_nontransaction_reset(i);
			if (ret)
				return ret;
		}


		if (Get_Input_Addr_From_DMABUF(&g_pda_Pdadata) < 0) {
			g_pda_Pdadata.Status = -26;
			LOG_INF("Get_Input_Addr_From_DMABUF fail\n");
			goto INPUT_BUFFER_FREE_IOVA;
		}

			// output buffer mapping iova
		if (Get_Output_Addr_From_DMABUF(&g_pda_Pdadata) < 0) {
			g_pda_Pdadata.Status = -27;
			LOG_INF("Get_Output_Addr_From_DMABUF fail\n");
			goto EXIT;
		}

		ret = pda_begin_dma();
		if (ret) {
			nRet = ret;
			goto EXIT;
		}

		// PDA HW and DMA setting
		HWDMASettings(&g_pda_Pdadata);

		// ------------------------ PDA pre-process done -----------------

		// Process flexible roi
		nUserROINumber = g_pda_Pdadata.ROInumber;
#ifdef FOR_DEBUG
		LOG_INF("nUserROINumber = %d\n", nUserROINumber);
		LOG_INF("g_PDA_quantity = %d\n", g_PDA_quantity);
#endif

		if (nUserROINumber == 0) {
#ifdef FOR_DEBUG
			LOG_INF("no flexible roi needed\n");
#endif
			goto FIX_ROI;
		}

#ifdef CHECK_IRQ_COUNT
		// setting reasonable IRQ count
		g_reasonable_IRQCount = 1 +
			(int)((nUserROINumber-1)/(PDA_MAXROI_PER_ROUND*g_PDA_quantity));
		// reset PDA0/PDA1 IRQ count
		g_PDA0_IRQCount = 0;
		g_PDA1_IRQCount = 0;
#ifdef FOR_DEBUG
		LOG_INF("g_reasonable_IRQCount = %d\n", g_reasonable_IRQCount);
#endif
#endif

		// Init ROI count which needed to process
		nROIcount = nUserROINumber;

		if (PDAProcessFunction(nUserROINumber, nROIcount, MFALSE, 0) < 0) {
			LOG_INF("Flexible ROI, PDA process fail\n");
			goto EXIT;
		}

		g_OutputBufferOffset += nUserROINumber*OUT_BYTE_PER_ROI;
#ifdef FOR_DEBUG
		LOG_INF("g_OutputBufferOffset: %d\n", g_OutputBufferOffset);
#endif

FIX_ROI:
		// Process fix roi
#ifdef FOR_DEBUG
		LOG_INF("nNumerousROI:%d\n", g_pda_Pdadata.nNumerousROI);
#endif

		for (i = 0; i < g_pda_Pdadata.nNumerousROI; ++i) {
			nUserROINumber = g_pda_Pdadata.xnum[i]*g_pda_Pdadata.ynum[i];

#ifdef FOR_DEBUG
			LOG_INF("nUserROINumber = %d\n", nUserROINumber);
#endif

			if (nUserROINumber == 0 || nUserROINumber > 1024) {
				g_pda_Pdadata.Status = -28;
				LOG_INF("ROI number out of range,idx/ROI/x/ynum:%d/%d/%d/%d\n",
					i,
					nUserROINumber,
					g_pda_Pdadata.xnum[i],
					g_pda_Pdadata.ynum[i]);
				goto EXIT;
			}

#ifdef CHECK_IRQ_COUNT
			// setting reasonable IRQ count
			g_reasonable_IRQCount = 1 +
				(int)((nUserROINumber-1)/(PDA_MAXROI_PER_ROUND*g_PDA_quantity));
			// reset PDA0/PDA1 IRQ count
			g_PDA0_IRQCount = 0;
			g_PDA1_IRQCount = 0;
#ifdef FOR_DEBUG
			LOG_INF("g_reasonable_IRQCount = %d\n", g_reasonable_IRQCount);
#endif
#endif

			// Init ROI count which needed to process
			nROIcount = nUserROINumber;

			if (PDAProcessFunction(nUserROINumber, nROIcount, MTRUE, i) < 0) {
				LOG_INF("Fix ROI, PDA process fail\n");
				goto EXIT;
			}

			g_OutputBufferOffset += nUserROINumber*OUT_BYTE_PER_ROI;
#ifdef FOR_DEBUG
			LOG_INF("g_OutputBufferOffset: %d\n", g_OutputBufferOffset);
#endif
		}


//////////////////////////////////////////////////////////////////////////////
EXIT:
INPUT_BUFFER_FREE_IOVA:
		ret = pda_retire_dma();
		if (ret) {
			g_pda_Pdadata.Status = ret;
			nRet = ret;
		}

EXIT_WITHOUT_FREE_IOVA:
#ifdef FOR_DEBUG
		LOG_INF("Exit\n");
#endif

#ifdef GET_PDA_TIME
		// for compute pda process time
		ktime_get_real_ts64(&total_time_end);

		LOG_INF("PDA 1 execute time (%d)\n",
			(pda1_done_e.tv_nsec - pda1_done_b.tv_nsec)/1000);
		LOG_INF("PDA 2 execute time (%d)\n",
			(pda2_done_e.tv_nsec - pda2_done_b.tv_nsec)/1000);
		LOG_INF("SW wait time (%d)\n",
			(time_end.tv_nsec - pda1_done_b.tv_nsec)/1000);
		LOG_INF("kernel total cost time (%d)\n",
		(total_time_end.tv_nsec-total_time_begin.tv_nsec)/1000);
#endif

		if (copy_to_user((void *)a_u4Param,
		    &g_pda_Pdadata,
		    sizeof(struct PDA_Data_t_v2)) != 0) {
			LOG_INF("copy_to_user failed\n");
			nRet = -EFAULT;
		}
		break;
	default:
		LOG_INF("Unknown Cmd(%d)\n", a_u4Command);
		break;
	}
	return nRet;
}

static long PDA_Ioctl(struct file *file, unsigned int command, unsigned long argument)
{
	long ret;

	mutex_lock(&pda_operation_lock);
	if (!pda_online || !file || file->private_data != &pda_file_users)
		ret = -ENODEV;
	else
		ret = pda_ioctl_locked(file, command, argument);
	mutex_unlock(&pda_operation_lock);
	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long PDA_Ioctl_Compat(struct file *file, unsigned int command,
		unsigned long argument)
{
	return PDA_Ioctl(file, command, (unsigned long)compat_ptr(argument));
}
#endif

static int pda_open_locked(struct inode *a_pstInode, struct file *a_pstFile)
{
	int ret;

	ret = EnableClock(MTRUE);
	if (ret)
		return ret;
	LOG_INF("PDA open g_u4EnableClockCount: %d", g_u4EnableClockCount);

#ifdef PDA_MMQOS
	pda_mmqos_bw_set();
#endif

#ifdef CHECK_IRQ_COUNT
	g_reasonable_IRQCount = 0;
	g_PDA0_IRQCount = 0;
	g_PDA1_IRQCount = 0;
#ifdef FOR_DEBUG
	LOG_INF("IRQCount, Reasonable = %d, PDA0 = %d, PDA1 = %d\n",
		g_reasonable_IRQCount, g_PDA0_IRQCount, g_PDA1_IRQCount);
#endif
#endif

	return 0;
}

static int PDA_Open(struct inode *inode, struct file *file)
{
	int ret;

	mutex_lock(&pda_operation_lock);
	if (!pda_online || !file)
		ret = -ENODEV;
	else if (pda_file_users == UINT_MAX)
		ret = -EOVERFLOW;
	else {
		ret = pda_open_locked(inode, file);
		if (!ret) {
			pda_file_users++;
			file->private_data = &pda_file_users;
		}
	}
	mutex_unlock(&pda_operation_lock);
	return ret;
}

static int pda_release_locked(struct inode *inode, struct file *file)
{
	/* A retained request lease keeps PM/clocks on after the last file closes.
	 * Failure leaves the mapped buffers and module text pinned until recovery.
	 */
	if (pda_dma_lease || pda_dma_active_mask)
		pda_retire_dma();

#ifdef PDA_MMQOS
	if (!pda_dma_quarantined && g_u4EnableClockCount == 1)
		pda_mmqos_bw_reset();
#endif
	return EnableClock(MFALSE);
}

static int PDA_Release(struct inode *inode, struct file *file)
{
	int ret = 0;

	mutex_lock(&pda_operation_lock);
	if (!file || file->private_data != &pda_file_users || !pda_file_users) {
		ret = -EINVAL;
	} else {
		file->private_data = NULL;
		/* Removed backends have already retired or quarantined all ownership. */
		if (pda_online)
			ret = pda_release_locked(inode, file);
		pda_file_users--;
	}
	mutex_unlock(&pda_operation_lock);
	return ret;
}

/*****************************************************************************
 *
 ****************************************************************************/

static dev_t g_PDA_devno;
static struct cdev *g_pPDA_CharDrv;
static struct class *actuator_class;
static struct device *lens_device;

static const struct file_operations g_stPDA_fops = {
	.owner = THIS_MODULE,
	.open = PDA_Open,
	.release = PDA_Release,
	.unlocked_ioctl = PDA_Ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = PDA_Ioctl_Compat,
#endif
};

static inline int PDA_RegCharDev(void)
{
	int ret;

	ret = alloc_chrdev_region(&g_PDA_devno, 0, 1, PDA_DEV_NAME);
	if (ret)
		return ret;
	g_pPDA_CharDrv = cdev_alloc();
	if (!g_pPDA_CharDrv) {
		ret = -ENOMEM;
		goto unregister_number;
	}
	/* cdev_alloc already initialized the dynamic object's kobject. */
	g_pPDA_CharDrv->ops = &g_stPDA_fops;
	g_pPDA_CharDrv->owner = THIS_MODULE;
	ret = cdev_add(g_pPDA_CharDrv, g_PDA_devno, 1);
	if (ret) {
		kobject_put(&g_pPDA_CharDrv->kobj);
		goto clear_cdev;
	}
	actuator_class = class_create("PDAdrv");
	if (IS_ERR(actuator_class)) {
		ret = PTR_ERR(actuator_class);
		actuator_class = NULL;
		goto delete_cdev;
	}
	lens_device = device_create(actuator_class, NULL, g_PDA_devno, NULL,
				    PDA_DEV_NAME);
	if (IS_ERR(lens_device)) {
		ret = PTR_ERR(lens_device);
		lens_device = NULL;
		class_destroy(actuator_class);
		actuator_class = NULL;
		goto delete_cdev;
	}
	return 0;

delete_cdev:
	cdev_del(g_pPDA_CharDrv);
clear_cdev:
	g_pPDA_CharDrv = NULL;
unregister_number:
	unregister_chrdev_region(g_PDA_devno, 1);
	return ret;
}

static inline void PDA_UnRegCharDev(void)
{
	if (!g_pPDA_CharDrv)
		return;
	cdev_del(g_pPDA_CharDrv);
	g_pPDA_CharDrv = NULL;
	device_destroy(actuator_class, g_PDA_devno);
	lens_device = NULL;
	class_destroy(actuator_class);
	actuator_class = NULL;
	unregister_chrdev_region(g_PDA_devno, 1);
}

static int pda_maybe_publish(void)
{
	int ret;

	if (!pda_driver_ready || !pda_expected_mask ||
	    (pda_bound_mask & pda_expected_mask) != pda_expected_mask)
		return 0;
	if (pda_online)
		return 0;
	g_PDA_quantity = pda_expected_mask == 3 ? 2 : 1;
	ret = PDA_RegCharDev();
	if (ret) {
		g_PDA_quantity = 0;
		return ret;
	}
	pda_online = true;
	return 0;
}

static void pda_release_engine(unsigned int index)
{
	struct pda_engine_owner *owner = &pda_owners[index];

	if (owner->irq_requested) {
		free_irq(PDA_devs[index].irq, owner);
		owner->irq_requested = false;
	}
	if (owner->pm_enabled)
		pm_runtime_disable(owner->dev);
	if (!index) {
#ifdef PDA_MMQOS
		pda_mmqos_put();
#endif
		/* Borrowed syscon maps are owned by the global syscon cache. */
		pda_mraw_regmap = NULL;
		while (pda_owned_clocks) {
			pda_owned_clocks--;
			clk_put(pda_clk[pda_owned_clocks].CG_PDA_TOP_MUX);
			pda_clk[pda_owned_clocks].CG_PDA_TOP_MUX = NULL;
		}
		g_dev1 = NULL;
		pda_expected_mask = 0;
	} else {
		g_dev2 = NULL;
	}
	if (PDA_devs[index].m_pda_base)
		iounmap(PDA_devs[index].m_pda_base);
	memset(&PDA_devs[index], 0, sizeof(PDA_devs[index]));
	if (owner->dev)
		put_device(owner->dev);
	memset(owner, 0, sizeof(*owner));
}

static int pda_probe_engine(struct platform_device *pdev, unsigned int index)
{
	struct pda_engine_owner *owner = &pda_owners[index];
	struct device_node *node;
	int ret, i;

	mutex_lock(&pda_operation_lock);
	if (pda_teardown_quarantined || pda_file_users || owner->dev || pda_online) {
		ret = -EBUSY;
		goto out;
	}
	owner->dev = get_device(&pdev->dev);
	ret = dma_set_mask_and_coherent(owner->dev, DMA_BIT_MASK(34));
	if (ret)
		goto release;
	ret = pda_link_larbs(pdev);
	if (ret)
		goto release;
	PDA_devs[index].m_pda_base = of_iomap(pdev->dev.of_node, 0);
	if (!PDA_devs[index].m_pda_base) {
		ret = -ENOMEM;
		goto release;
	}
	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto release;
	if (!ret) {
		ret = -EINVAL;
		goto release;
	}
	PDA_devs[index].irq = ret;
	if (!index) {
		g_dev1 = owner->dev;
		pda_expected_mask = 1;
		node = of_find_compatible_node(NULL, NULL, "mediatek,camera-pda2");
		if (node) {
			if (of_device_is_available(node))
				pda_expected_mask |= BIT(1);
			of_node_put(node);
		}
		for (i = 0; i < PDA_CLK_NUM; i++) {
			struct clk *clock = clk_get(owner->dev, clk_names[i]);

			if (IS_ERR(clock)) {
				ret = PTR_ERR(clock);
				goto release;
			}
			pda_clk[i].CG_PDA_TOP_MUX = clock;
			pda_owned_clocks++;
		}
		node = of_find_compatible_node(NULL, NULL, "mediatek,mt6895-camsys_mraw");
		if (!node) {
			ret = -ENODEV;
			goto release;
		}
		pda_mraw_regmap = syscon_node_to_regmap(node);
		of_node_put(node);
		if (IS_ERR(pda_mraw_regmap)) {
			ret = PTR_ERR(pda_mraw_regmap);
			pda_mraw_regmap = NULL;
			goto release;
		}
#ifdef PDA_MMQOS
		ret = pda_mmqos_init();
		if (ret)
			goto release;
#endif
	} else {
		g_dev2 = owner->dev;
	}
	pm_runtime_enable(owner->dev);
	owner->pm_enabled = true;
	/* Keep IRQs disabled until the first successful PM/clock startup. */
	ret = request_irq(PDA_devs[index].irq,
			 index ? pda2_irqhandle : pda_irqhandle, IRQF_NO_AUTOEN,
			 dev_name(owner->dev), owner);
	if (ret)
		goto release;
	owner->irq_requested = true;
	pda_bound_mask |= BIT(index);
	ret = pda_maybe_publish();
	if (!ret)
		goto out;
	pda_bound_mask &= ~BIT(index);
release:
	pda_release_engine(index);
out:
	mutex_unlock(&pda_operation_lock);
	return ret;
}

static void pda_remove_engine(struct platform_device *pdev, unsigned int index)
{
	unsigned int i;

	mutex_lock(&pda_operation_lock);
	if (pda_owners[index].dev != &pdev->dev || pda_teardown_quarantined)
		goto out;
	pda_online = false;
	PDA_UnRegCharDev();
	if ((pda_dma_lease || pda_dma_active_mask) && pda_retire_dma()) {
		/* Do not release a device's manual MMIO/clock/DMA ownership after an
		 * uncertain stop. After detachment no file may access this backend.
		 * The retained module lease pins all state until reboot.
		 */
		pda_teardown_quarantined = true;
		for (i = 0; i < ARRAY_SIZE(pda_owners); i++) {
			if (pda_owners[i].irq_requested) {
				free_irq(PDA_devs[i].irq, &pda_owners[i]);
				pda_owners[i].irq_requested = false;
			}
		}
		dev_err(&pdev->dev, "DMA stop unacknowledged; retaining PDA resources until reboot\n");
		goto out;
	}
	/* Files are now detached; one physical PM/clock lease backs their count. */
	if (g_u4EnableClockCount) {
		g_u4EnableClockCount = 1;
		EnableClock(MFALSE);
	}
	g_PDA_quantity = 0;
	pda_bound_mask &= ~BIT(index);
	pda_release_engine(index);
out:
	mutex_unlock(&pda_operation_lock);
}

static int PDA_probe(struct platform_device *pdev)
{
	return pda_probe_engine(pdev, 0);
}

static int PDA2_probe(struct platform_device *pdev)
{
	return pda_probe_engine(pdev, 1);
}

static void PDA_remove(struct platform_device *pdev)
{
	pda_remove_engine(pdev, 0);
}

static void PDA2_remove(struct platform_device *pdev)
{
	pda_remove_engine(pdev, 1);
}

static int PDA_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	int ret;

	mutex_lock(&pda_operation_lock);
	ret = g_u4EnableClockCount || pda_teardown_quarantined ? -EBUSY : 0;
	mutex_unlock(&pda_operation_lock);
	return ret;
}

static int PDA_resume(struct platform_device *pdev)
{
	return 0;
}

//////////////////////////////////////// PDA driver //////////////////////////
#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gpda_of_device_id[] = {
	{.compatible = "mediatek,camera-pda",},
	{}
};
#endif

MODULE_DEVICE_TABLE(of, gpda_of_device_id);

static struct platform_driver PDADriver = {
	.probe = PDA_probe,
	.remove = PDA_remove,
	.suspend = PDA_suspend,
	.resume = PDA_resume,
	.driver = {
		   .name = PDA_DEV_NAME,
		   .owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = of_match_ptr(gpda_of_device_id),
#endif
		}
};

//////////////////////////////////////// PDA 2 driver ////////////////////////
#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gpda2_of_device_id[] = {
	{.compatible = "mediatek,camera-pda2",},
	{}
};
#endif

MODULE_DEVICE_TABLE(of, gpda2_of_device_id);

static struct platform_driver PDA2Driver = {
	.probe = PDA2_probe,
	.remove = PDA2_remove,
	.driver = {
		   .name = "camera-pda2",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(gpda2_of_device_id),
	}
};

static int __init camera_pda_init(void)
{
	int ret;

	ret = platform_driver_register(&PDADriver);
	if (ret)
		return ret;
	ret = platform_driver_register(&PDA2Driver);
	if (ret)
		goto unregister_primary;
	mutex_lock(&pda_operation_lock);
	pda_driver_ready = true;
	ret = pda_maybe_publish();
	if (ret)
		pda_driver_ready = false;
	mutex_unlock(&pda_operation_lock);
	if (!ret)
		return 0;
	platform_driver_unregister(&PDA2Driver);
unregister_primary:
	platform_driver_unregister(&PDADriver);
	return ret;
}

static void __exit camera_pda_exit(void)
{
	mutex_lock(&pda_operation_lock);
	pda_driver_ready = false;
	mutex_unlock(&pda_operation_lock);
	platform_driver_unregister(&PDA2Driver);
	platform_driver_unregister(&PDADriver);
}
/****************************************************************************
 *
 ****************************************************************************/
module_init(camera_pda_init);
module_exit(camera_pda_exit);
MODULE_DESCRIPTION("Camera PDA driver");
MODULE_AUTHOR("MM6SW3");
MODULE_LICENSE("GPL");

MODULE_IMPORT_NS(DMA_BUF);
