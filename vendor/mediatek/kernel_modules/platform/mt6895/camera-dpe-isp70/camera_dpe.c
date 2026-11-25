// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */
/**************************************************************
 * camera_DPE.c - Linux DPE Device Driver
 *
 * DESCRIPTION:
 *     This file provid the other drivers DPE relative functions
 *
 **************************************************************/


#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/types.h>
/* #include <asm/io.h> */
/* #include <asm/tcm.h> */
#include <linux/proc_fs.h> /* proc file use */
/*  */
#include <linux/slab.h>
#include <linux/spinlock.h>
/* #include <linux/io.h> */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/sched/clock.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/overflow.h>
#include <linux/suspend.h>
#include <linux/rtc.h>
// V4L2
#include <linux/mutex.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ioctl.h>
#define KERNEL_DMA_BUFFER
#ifdef KERNEL_DMA_BUFFER
#include <media/videobuf2-memops.h>
#include <linux/dma-direction.h>
#include <linux/dma-buf.h>
#endif
/*#include <linux/xlog.h>		 For xlog_printk(). */
/*  */
/*#include <mach/hardware.h>*/
/* #include <mach/mt6593_pll.h> */
#include "camera_dpe.h"
/*#include <mach/irqs.h>*/
/* #include <mach/mt_reg_base.h> */
/* #if IS_ENABLED(CONFIG_MTK_LEGACY) */
/* For clock mgr APIS. enable_clock()/disable_clock(). */
/* #include <mach/mt_clkmgr.h> */
/* #endif */
//!#include <mt-plat/sync_write.h> /* For mt65xx_reg_sync_writel(). */
/* For spm_enable_sodi()/spm_disable_sodi(). */
/* #include <mach/mt_spm_idle.h> */
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
//#include <cmdq_core.h>
//#include <cmdq_record.h>
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
//#include <linux/soc/mediatek/mtk-cmdq.h>
#include <soc/mediatek/smi.h>

//! for IOVA to PA
#include <linux/iommu.h>
#define CMdq_en
#define DPE_WR32_en
#define dpe_dump_read_en
#define SMI_CLK
#define CMDQ_COMMON
#define CHECK_SERVICE_IF_0 0
//!#define DPE_debug_log_en
//#define m4u_en
//!#define smi_en
//!#define WAKEUP_INIT


#ifdef m4u_en
#if IS_ENABLED(CONFIG_MTK_IOMMU_V2)
#include <mach/mt_iommu.h>
#include "mach/pseudo_m4u.h"
#else /* CONFIG_MTK_IOMMU_V2 */
//#include <m4u.h>
#endif /* CONFIG_MTK_IOMMU_V2 */
#endif

#ifdef smi_en
#include <smi_public.h>
#endif
#include "engine_request.h"
#ifdef KERNEL_DMA_BUFFER
#include "videobuf2-dma-contig.h"
#endif
/*#define DPE_PMQOS_EN*/
#if defined(DPE_PMQOS_EN) && defined(CONFIG_MTK_QOS_SUPPORT)
#include <linux/pm_qos.h>
#endif
/* Measure the kernel performance
 * #define __DPE_KERNEL_PERFORMANCE_MEASURE__
 */
#ifdef __DPE_KERNEL_PERFORMANCE_MEASURE__
#include <linux/met_drv.h>
#include <linux/mtk_ftrace.h>
#endif

//Another Performance Measure Usage
//#include <linux/ftrace_event.h>
//#include <linux/kallsyms.h>
//static unsigned long __read_mostly tracing_mark_write_addr;
//#define _kernel_trace_begin(name)                                              \
//	{                                                                      \
//		tracing_mark_write_addr =                                      \
//			kallsyms_lookup_name("tracing_mark_write");            \
//		event_trace_printk(tracing_mark_write_addr, "B|%d|%s\n",       \
//				   current->tgid, name);                       \
//	}
//#define _kernel_trace_end()                                                    \
//	{                                                                      \
//		event_trace_printk(tracing_mark_write_addr, "E\n");            \
//	}
// How to Use
//char strName[128];
// sprintf(strName, "TAG_K_WAKEUP (%d)",sof_count[_PASS1]);
// _kernel_trace_begin(strName);
// _kernel_trace_end();

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#if IS_ENABLED(CONFIG_COMPAT)
/* 64 bit */
#include <linux/compat.h>
#include <linux/fs.h>
#endif
/*  #include "smi_common.h" */
#include <linux/pm_wakeup.h>
/* CCF */
#if !IS_ENABLED(CONFIG_MTK_LEGACY) && IS_ENABLED(CONFIG_COMMON_CLK) /*CCF*/
#include <linux/clk.h>
struct DPE_CLK_STRUCT {
	struct clk *VCORE_GALS;
	struct clk *MAIN_GALS;
	struct clk *IMG_IPE;
	struct clk *IPE_DPE;
	struct clk *IPE_TOP;
	struct clk *IPE_SMI_LARB12;
};
struct DPE_CLK_STRUCT dpe_clk;
#endif

#ifndef MTRUE
#define MTRUE 1
#endif
#ifndef MFALSE
#define MFALSE 0
#endif
#define DPE_DEV_NAME "camera-dpe"
//#define EP_NO_CLKMGR
#define BYPASS_REG (0)
#define DUMMY_DPE (0)
#define UT_CASE
/*I can' test the situation in FPGA due to slow FPGA. */
#define MyTag "[DPE]"
#define IRQTag "KEEPER"
#define LOG_VRB(format, args...) pr_debug(MyTag format, ##args)
//#define DPE_DEBUG_USE
#ifdef DPE_DEBUG_USE
#define LOG_DBG(format, args...) pr_info(MyTag format, ##args)
#else
#define LOG_DBG(format, args...)
#endif
#define LOG_INF(format, args...) pr_info(MyTag format, ##args)
#define LOG_NOTICE(format, args...) pr_notice(MyTag format, ##args)
#define LOG_WRN(format, args...) pr_info(MyTag format, ##args)
#define LOG_ERR(format, args...) pr_info(MyTag format, ##args)
#define LOG_AST(format, args...) pr_info(MyTag format, ##args)
/**************************************************************
 *
 **************************************************************/
/* #define DPE_WR32(addr, data) iowrite32(data, addr) */
//#define DPE_WR32(addr, data) mt_reg_sync_writel(data, addr)
//#define DPE_RD32(addr) ioread32(addr)
#define DPE_WR32(addr, data) writel(data, addr)
#define DPE_RD32(addr) readl(addr)
#define DPE_MASKWR(addr, data, mask) \
	DPE_WR32(addr, ((DPE_RD32(addr) & ~(mask)) | data))
/**************************************************************
 *
 **************************************************************/
/* dynamic log level */
#define DPE_DBG_DBGLOG (0x00000001)
#define DPE_DBG_INFLOG (0x00000002)
#define DPE_DBG_INT (0x00000004)
#define DPE_DBG_READ_REG (0x00000008)
#define DPE_DBG_WRITE_REG (0x00000010)
#define DPE_DBG_TASKLET (0x00000020)
/*
 *    CAM interrupt status
 */
/* normal siganl : happens to be the same bit as register bit*/
/*#define DPE_INT_ST           (1<<0)*/
/*
 *   IRQ signal mask
 */
#define INT_ST_MASK_DPE (DPE_INT_ST)
#define CMDQ_REG_MASK 0xffffffff
#define DPE_START 0x1
#define DPE_ENABLE 0x1
#define MASK_15 0xF
#define ALIGN16(x) (((x)+MASK_15)&(~(MASK_15)))
#define MAX_NUM_TILE 4
#define TILE_WITH_NUM 3
//#define IOVA_TO_PA
#define IRQ_LOG
#if IS_ENABLED(CONFIG_MTK_IOMMU_V2)
static int DPE_MEM_USE_VIRTUL = 1;
#endif
/* static irqreturn_t DPE_Irq_CAM_A(signed int  Irq,void *DeviceId); */
static irqreturn_t ISP_Irq_DVP(signed int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_DVS(signed int Irq, void *DeviceId);
static void DPE_ScheduleWork(struct work_struct *data);
static void DVP_ScheduleWork(struct work_struct *data);
typedef irqreturn_t (*IRQ_CB)(signed int, void *);
struct ISR_TABLE {
	IRQ_CB isr_fp;
	unsigned int int_number;
	char device_name[16];
};
#if !IS_ENABLED(CONFIG_OF)
const struct ISR_TABLE DPE_IRQ_CB_TBL[DPE_IRQ_TYPE_AMOUNT] = {
	{ISP_Irq_DVP, DPE_IRQ_BIT_ID, "dvp"},
	{ISP_Irq_DVS, DPE_IRQ_BIT_ID, "dvs"},
};
#else
/* int number is got from kernel api */
const struct ISR_TABLE DPE_IRQ_CB_TBL[DPE_IRQ_TYPE_AMOUNT] = {
#if DUMMY_DPE
	{ISP_Irq_DVP, 0, "dvp-dummy"},
	{ISP_Irq_DVS, 0, "dvs-dummy"},
#else
	{ISP_Irq_DVP, 0, "dvp"},
	{ISP_Irq_DVS, 0, "dvs"},
#endif
};
#endif
/*
 */
/*  */
typedef void (*tasklet_cb)(unsigned long);
struct Tasklet_table {
	tasklet_cb tkt_cb;
	struct tasklet_struct *pDPE_tkt;
};
struct tasklet_struct Dpetkt[DPE_IRQ_TYPE_AMOUNT];
static void ISP_TaskletFunc_DVP(unsigned long data);
static void ISP_TaskletFunc_DVS(unsigned long data);
static struct Tasklet_table DPE_tasklet[DPE_IRQ_TYPE_AMOUNT] = {
	{ISP_TaskletFunc_DVP, &Dpetkt[DPE_IRQ_TYPE_INT_DVP_ST]},
	{ISP_TaskletFunc_DVS, &Dpetkt[DPE_IRQ_TYPE_INT_DVS_ST]},
};
static struct work_struct logWork;
static void logPrint(struct work_struct *data);
struct wakeup_source DPE_wake_lock;
static DEFINE_MUTEX(gDpeMutex);
static DEFINE_MUTEX(gDpeDequeMutex);
static DEFINE_MUTEX(gFDMutex);
static DEFINE_MUTEX(gDVSMutex);
#if IS_ENABLED(CONFIG_OF)
struct DPE_device {
	void __iomem *regs;
	struct device *dev;
	struct device		*larb12;
	int irq;
// V4L2
	struct v4l2_device v4l2_dev;
	struct mutex mutex;
	struct video_device vid_dpe_dev;
};
static struct DPE_device dpe_slots[2];
static struct DPE_device *DPE_devs = dpe_slots;
static DEFINE_MUTEX(dpe_probe_lock);
static bool dpe_ready;
static bool root_test_only;
module_param(root_test_only, bool, 0444);
MODULE_PARM_DESC(root_test_only, "Restrict test opens to CAP_SYS_ADMIN");
static bool wire_test_only;
module_param(wire_test_only, bool, 0444);
MODULE_PARM_DESC(wire_test_only, "Root-only V4L2 serializer echo; never submit depth jobs");
static bool probe_only;
module_param(probe_only, bool, 0444);
MODULE_PARM_DESC(probe_only, "Bind resources without publishing userspace nodes or enabling IRQs");
static int nr_DPE_devs;
/* Get HW modules' base address from device nodes */
#define DPE_DEV_NODE_IDX 0
#define IPESYS_DEV_MODE_IDX 1
#define ISP_DPE_BASE (DPE_devs[DPE_DEV_NODE_IDX].regs)
#define ISP_IPESYS_BASE (DPE_devs[IPESYS_DEV_MODE_IDX].regs)
#else
#define ISP_DPE_BASE (IPESYS_BASE + 0x100000)
#endif
#ifdef SMI_CLK
struct platform_device *DPE_pdev;
#endif
#ifdef KERNEL_DMA_BUFFER
struct device *gdev;
struct dma_buf *dbuf;
struct vb2_dc_buf {
	struct device			*dev;
	void				*vaddr;
	unsigned long			size;
	void				*cookie;
	dma_addr_t			dma_addr;
	unsigned long			attrs;
	enum dma_data_direction		dma_dir;
	struct sg_table			*dma_sgt;
	struct frame_vector		*vec;
	/* MMAP related */
	struct vb2_vmarea_handler	handler;
	refcount_t			refcount;
	struct sg_table			*sgt_base;
	const char *name;
	/* DMABUF related */
	struct dma_buf_attachment	*db_attach;
	struct dma_buf			*dma_buf;
	struct sg_table			*sgt;
};
static void *dpe_native_scratch;
static dma_addr_t dpe_native_scratch_dma;
struct vb2_dc_buf *kernel_dpebuf;
struct vb2_dc_buf *dpebuf;
dma_addr_t *g_dpewb_dvme_int_Buffer_pa;
dma_addr_t *g_dpewb_cost_int_Buffer_pa;
dma_addr_t *g_dpewb_asfrm_Buffer_pa;
dma_addr_t *g_dpewb_asfrmext_Buffer_pa;
dma_addr_t *g_dpewb_wmfhf_Buffer_pa;
#else
struct device *gdev;
#endif
static unsigned int g_u4EnableClockCount;
static unsigned int g_SuspendCnt;
/* maximum number for supporting user to do interrupt operation */
/* index 0 is for all the user that do not do register irq first */
#define IRQ_USER_NUM_MAX 32
bool g_DPE_PMState;
bool g_isDPELogEnable = MFALSE;
//for cmdq malibox
static struct cmdq_client *dpe_clt;
struct cmdq_base *dpe_clt_base;
u32 dvs_event_id;
u32 dvp_event_id;
enum DPE_FRAME_STATUS_ENUM {
	DPE_FRAME_STATUS_EMPTY,    /* 0 */
	DPE_FRAME_STATUS_ENQUE,    /* 1 */
	DPE_FRAME_STATUS_RUNNING,  /* 2 */
	DPE_FRAME_STATUS_FINISHED, /* 3 */
	DPE_FRAME_STATUS_TOTAL
};
enum DPE_REQUEST_STATE_ENUM {
	DPE_REQUEST_STATE_EMPTY,    /* 0 */
	DPE_REQUEST_STATE_PENDING,  /* 1 */
	DPE_REQUEST_STATE_RUNNING,  /* 2 */
	DPE_REQUEST_STATE_FINISHED, /* 3 */
	DPE_REQUEST_STATE_TOTAL
};
struct DPE_REQUEST_STRUCT {
	enum DPE_REQUEST_STATE_ENUM State;
	pid_t processID;       /* caller process ID */
	unsigned int callerID; /* caller thread ID */
	unsigned int enqueReqNum;   /* to judge it belongs to which frame package */
	unsigned int FrameWRIdx; /* Frame write Index */
	unsigned int RrameRDIdx; /* Frame read Index */
	enum DPE_FRAME_STATUS_ENUM
		DpeFrameStatus[_SUPPORT_MAX_DPE_FRAME_REQUEST_];
	struct DPE_Config DpeFrameConfig[_SUPPORT_MAX_DPE_FRAME_REQUEST_];
};
struct DPE_REQUEST_RING_STRUCT {
	unsigned int WriteIdx;     /* enque how many request  */
	unsigned int ReadIdx;      /* read which request index */
	unsigned int HWProcessIdx; /* HWWriteIdx */
	struct DPE_REQUEST_STRUCT
		DPEReq_Struct[_SUPPORT_MAX_DPE_REQUEST_RING_SIZE_];
};
struct DPE_CONFIG_STRUCT {
	struct DPE_Config DpeFrameConfig[_SUPPORT_MAX_DPE_FRAME_REQUEST_];
};
static struct DPE_REQUEST_RING_STRUCT g_DPE_ReqRing;
//static struct engine_requests dpe_reqs;
static struct engine_requests dpe_reqs_dvs;
static struct engine_requests dpe_reqs_dvp;
static struct DPE_Request kDpeReq;
#define PMD_ENTRIES_MAX	512
#define MMU_ION_BUF		BIT(24)
union mmu_table {
	u64		*entries;	/* Array of PTEs */
	/* Array of pages */
	struct page	**pages;
	/* Array of VAs */
	uintptr_t	*vas;
	/* Address of table */
	void		*addr;
	/* Page for table */
	unsigned long	page;
};
//static struct DPE_device *DPE_DVSreg;//!
struct tee_mmu {
	struct iommu_domain	*domain;
	dma_addr_t			dma_addr;
	/* ION case only */
	struct dma_buf			*dma_buf;
	struct dma_buf_attachment	*attach;
	struct sg_table			*sgt;
};
//DVS
struct tee_mmu	*SrcImg_Y_L_mmu;
struct tee_mmu	*SrcImg_Y_R_mmu;
struct tee_mmu	*ValidMap_L_mmu;
struct tee_mmu	*ValidMap_R_mmu;
struct tee_mmu	*OutBuf_OCC_mmu;
struct tee_mmu	*OutBuf_OCC_Ext_mmu;
struct tee_mmu	*SrcImg_Y_L_Pre_mmu;
struct tee_mmu	*SrcImg_Y_R_Pre_mmu;
struct tee_mmu	*InBuf_P4_L_mmu;
struct tee_mmu	*InBuf_P4_R_mmu;
//DVP
struct tee_mmu	*SrcImg_Y_mmu;
struct tee_mmu	*SrcImg_C_mmu;
struct tee_mmu	*InBuf_OCC_mmu;
struct tee_mmu	*OutBuf_CRM_mmu;
struct tee_mmu	*ASF_RD_mmu;
struct tee_mmu	*ASF_HF_mmu;
struct tee_mmu	*WMF_FILT_mmu;
struct tee_mmu	*InBuf_OCC_Ext_mmu;
struct tee_mmu	*ASF_RD_Ext_mmu;
struct tee_mmu	*ASF_HF_Ext_mmu;
unsigned int DPE_P4_EN;
unsigned int DVP_is16BitMode;
unsigned int get_dvs_iova[10];
unsigned int get_dvp_iova[10];
unsigned int DVS_only_en;
unsigned int DVS_Num;
unsigned int DVP_only_en;
unsigned int DVP_Num;
/**************************************************************
 *
 **************************************************************/
struct DPE_USER_INFO_STRUCT {
	pid_t Pid;
	pid_t Tid;
};
enum DPE_PROCESS_ID_ENUM {
	DPE_PROCESS_ID_NONE,
	DPE_PROCESS_ID_DPE,
	DPE_PROCESS_ID_AMOUNT
};
/**************************************************************
 *
 **************************************************************/
struct DPE_IRQ_INFO_STRUCT {
	unsigned int Status[DPE_IRQ_TYPE_AMOUNT];
	signed int DpeIrqCnt[IRQ_USER_NUM_MAX];
	pid_t ProcessID[IRQ_USER_NUM_MAX];
	unsigned int Mask[DPE_IRQ_TYPE_AMOUNT];
};
struct DPE_INFO_STRUCT {
	spinlock_t SpinLockDPERef;
	spinlock_t SpinLockDPE;
	spinlock_t SpinLockFD;//!
	spinlock_t SpinLockIrq[DPE_IRQ_TYPE_AMOUNT];
	wait_queue_head_t WaitQueueHead;
	struct work_struct ScheduleDpeWork;
	struct work_struct DVP_ScheduleDpeWork;
	struct workqueue_struct *wkqueue;
	unsigned int UserCount; /* User Count */
	unsigned int DebugMask; /* Debug Mask */
	signed int IrqNum;
	struct DPE_IRQ_INFO_STRUCT IrqInfo;
	signed int WriteReqIdx;
	signed int ReadReqIdx;
	pid_t ProcessID[_SUPPORT_MAX_DPE_FRAME_REQUEST_];
};
static struct DPE_INFO_STRUCT DPEInfo;
#if defined(DPE_PMQOS_EN) && defined(CONFIG_MTK_QOS_SUPPORT)
struct pm_qos_request dpe_pm_qos_request;
#endif
enum eLOG_TYPE {
	_LOG_DBG = 0,
	_LOG_INF = 1,
	_LOG_ERR = 2,
	_LOG_MAX = 3,
};
#define NORMAL_STR_LEN (512)
#define ERR_PAGE 2
#define DBG_PAGE 2
#define INF_PAGE 4
/* #define SV_LOG_STR_LEN NORMAL_STR_LEN */
#define LOG_PPNUM 2
static unsigned int m_CurrentPPB;
struct SV_LOG_STR {
	unsigned int _cnt[LOG_PPNUM][_LOG_MAX];
	/* char   _str[_LOG_MAX][SV_LOG_STR_LEN]; */
	char *_str[LOG_PPNUM][_LOG_MAX];
} *PSV_LOG_STR;
static void *pLog_kmalloc;
static struct SV_LOG_STR gSvLog[DPE_IRQ_TYPE_AMOUNT];
/*
 *   for irq used,keep log until IRQ_LOG_PRINTER being involked,
 *   limited:
 *   each log must shorter than 512 bytes
 *  total log length in each irq/logtype can't over 1024 bytes
 */
#ifdef IRQ_LOG
#define IRQ_LOG_KEEPER(irq, ppb, logT, fmt, ...) do {\
	char *ptr; \
	char *pDes;\
	int avaLen;\
	unsigned int *ptr2 = &gSvLog[irq]._cnt[ppb][logT];\
	unsigned int str_leng;\
	unsigned int logi;\
	int ret; \
	struct SV_LOG_STR *pSrc = &gSvLog[irq];\
	if (logT == _LOG_ERR) {\
		str_leng = NORMAL_STR_LEN*ERR_PAGE; \
	} else if (logT == _LOG_DBG) {\
		str_leng = NORMAL_STR_LEN*DBG_PAGE; \
	} else if (logT == _LOG_INF) {\
		str_leng = NORMAL_STR_LEN*INF_PAGE;\
	} else {\
		str_leng = 0;\
	} \
	ptr = pDes = (char *)\
		&(gSvLog[irq]._str[ppb][logT][gSvLog[irq]._cnt[ppb][logT]]);   \
	avaLen = str_leng - 1 - gSvLog[irq]._cnt[ppb][logT];\
	if (avaLen > 1) {\
		ret = snprintf((char *)(pDes), avaLen, fmt,\
			##__VA_ARGS__);   \
		if (ret < 0) { \
			LOG_ERR("snprintf fail(%d)\n", ret); \
		} \
		if ('\0' != gSvLog[irq]._str[ppb][logT][str_leng - 1]) {\
			LOG_ERR("log str over flow(%d)", irq);\
		} \
		while (*ptr++ != '\0') {        \
			(*ptr2)++;\
		}     \
	} else { \
		LOG_INF("(%d)(%d)log str avalible=0, print log\n", irq, logT);\
		ptr = pSrc->_str[ppb][logT];\
		if (pSrc->_cnt[ppb][logT] != 0) {\
			if (logT == _LOG_DBG) {\
				for (logi = 0; logi < DBG_PAGE; logi++) {\
					if (ptr[NORMAL_STR_LEN*(logi+1) - 1] !=\
									'\0') {\
						ptr[NORMAL_STR_LEN*(logi+1)\
								- 1] = '\0';\
						LOG_DBG("%s", &ptr[\
							NORMAL_STR_LEN*logi]);\
					} else {\
						LOG_DBG("%s",\
						&ptr[NORMAL_STR_LEN*logi]);\
						break;\
					} \
				} \
			} \
			else if (logT == _LOG_INF) {\
				for (logi = 0; logi < INF_PAGE; logi++) {\
					if (ptr[NORMAL_STR_LEN*(logi+1) - 1] !=\
									'\0') {\
						ptr[NORMAL_STR_LEN*(logi+1)\
								   - 1] = '\0';\
						LOG_INF("%s",		       \
						    &ptr[NORMAL_STR_LEN*logi]);\
					} else {\
						LOG_INF("%s",\
						&ptr[NORMAL_STR_LEN*logi]);\
						break;\
					} \
				} \
			} \
			else if (logT == _LOG_ERR) {\
				for (logi = 0; logi < ERR_PAGE; logi++) {\
					if (ptr[NORMAL_STR_LEN*(logi+1) - 1] !=\
									'\0') {\
						ptr[NORMAL_STR_LEN*(logi+1)\
								   - 1] = '\0';\
						LOG_INF("%s",\
						&ptr[NORMAL_STR_LEN*logi]);\
					} else {\
						LOG_INF("%s",\
						&ptr[NORMAL_STR_LEN*logi]);\
						break;\
					} \
				} \
			} \
			else {\
				LOG_INF("N.S.%d", logT);\
			} \
			ptr[0] = '\0';\
			pSrc->_cnt[ppb][logT] = 0;\
			avaLen = str_leng - 1;\
			ptr = pDes = (char *)\
			&(pSrc->_str[ppb][logT][pSrc->_cnt[ppb][logT]]);\
			ptr2 = &(pSrc->_cnt[ppb][logT]);\
		ret = snprintf((char *)(pDes), avaLen, fmt, ##__VA_ARGS__);  \
		if (ret < 0) { \
			LOG_ERR("snprintf fail(%d)\n", ret); \
		} \
			while (*ptr++ != '\0') {\
				(*ptr2)++;\
			} \
		} \
	} \
} while (0)
#else
#define IRQ_LOG_KEEPER(irq, ppb, logT, fmt, ...) \
	xlog_printk(ANDROID_LOG_DEBUG,\
"KEEPER", "[%s] " fmt, __func__, ##__VA_ARGS__)
#endif
#ifdef IRQ_LOG
#define IRQ_LOG_PRINTER(irq, ppb_in, logT_in) do {\
	struct SV_LOG_STR *pSrc = &gSvLog[irq];\
	char *ptr;\
	unsigned int i;\
	unsigned int ppb = 0;\
	unsigned int logT = 0;\
	if (ppb_in > 1) {\
		ppb = 1;\
	} else {\
		ppb = ppb_in;\
	} \
	if (logT_in > _LOG_ERR) {\
		logT = _LOG_ERR;\
	} else {\
		logT = logT_in;\
	} \
	ptr = pSrc->_str[ppb][logT];\
	if (pSrc->_cnt[ppb][logT] != 0) {\
		if (logT == _LOG_DBG) {\
			for (i = 0; i < DBG_PAGE; i++) {\
				if (ptr[NORMAL_STR_LEN*(i+1) - 1] != '\0') {\
					ptr[NORMAL_STR_LEN*(i+1) - 1] = '\0';\
					LOG_DBG("%s", &ptr[NORMAL_STR_LEN*i]);\
				} else {\
					LOG_DBG("%s", &ptr[NORMAL_STR_LEN*i]);\
					break;\
				} \
			} \
		} \
	else if (logT == _LOG_INF) {\
		for (i = 0; i < INF_PAGE; i++) {\
			if (ptr[NORMAL_STR_LEN*(i+1) - 1] != '\0') {\
				ptr[NORMAL_STR_LEN*(i+1) - 1] = '\0';\
				LOG_INF("%s", &ptr[NORMAL_STR_LEN*i]);\
			} else {\
				LOG_INF("%s", &ptr[NORMAL_STR_LEN*i]);\
				break;\
			} \
		} \
	} \
	else if (logT == _LOG_ERR) {\
		for (i = 0; i < ERR_PAGE; i++) {\
			if (ptr[NORMAL_STR_LEN*(i+1) - 1] != '\0') {\
				ptr[NORMAL_STR_LEN*(i+1) - 1] = '\0';\
				LOG_ERR("%s", &ptr[NORMAL_STR_LEN*i]);\
			} else {\
				LOG_ERR("%s", &ptr[NORMAL_STR_LEN*i]);\
				break;\
			} \
		} \
	} \
	else {\
		LOG_ERR("N.S.%d", logT);\
	} \
		ptr[0] = '\0';\
		pSrc->_cnt[ppb][logT] = 0;\
	} \
} while (0)
#else
#define IRQ_LOG_PRINTER(irq, ppb, logT)
#endif
#define IPESYS_REG_CG_CON               (ISP_IPESYS_BASE + 0x0)
#define IPESYS_REG_CG_SET               (ISP_IPESYS_BASE + 0x4)
#define IPESYS_REG_CG_CLR               (ISP_IPESYS_BASE + 0x8)
/* DPE unmapped base address macro for GCE to access */
#define DVS_CTRL00_HW                (DPE_BASE_HW)
#define DVS_CTRL01_HW                (DPE_BASE_HW + 0x004)
#define DVS_CTRL02_HW                (DPE_BASE_HW + 0x008)
#define DVS_CTRL03_HW                (DPE_BASE_HW + 0x00C)
//#define DVS_CTRL04_HW                (DPE_BASE_HW + 0x010)
//#define DVS_CTRL05_HW                (DPE_BASE_HW + 0x014)
#define DVS_CTRL06_HW                (DPE_BASE_HW + 0x018)
#define DVS_CTRL07_HW                (DPE_BASE_HW + 0x01C)
//#define DVS_OCC_PQ_0_HW              (DPE_BASE_HW + 0x030)
//#define DVS_OCC_PQ_1_HW              (DPE_BASE_HW + 0x034)
//#define DVS_OCC_ATPG_HW              (DPE_BASE_HW + 0x038)
#define DVS_IRQ_00_HW                (DPE_BASE_HW + 0x040)
#define DVS_CTRL_STATUS0_HW          (DPE_BASE_HW + 0x050)
//#define DVS_CTRL_STATUS1_HW          (DPE_BASE_HW + 0x054)
#define DVS_CTRL_STATUS2_HW          (DPE_BASE_HW + 0x058)
#define DVS_IRQ_STATUS_HW            (DPE_BASE_HW + 0x05C)
#define DVS_FRM_STATUS0_HW           (DPE_BASE_HW + 0x060)
#define DVS_FRM_STATUS1_HW           (DPE_BASE_HW + 0x064)
//#define DVS_FRM_STATUS2_HW           (DPE_BASE_HW + 0x068)
//#define DVS_FRM_STATUS3_HW           (DPE_BASE_HW + 0x06C)
#define DVS_CUR_STATUS_HW            (DPE_BASE_HW + 0x070)
#define DVS_SRC_CTRL_HW              (DPE_BASE_HW + 0x074)
#define DVS_CRC_CTRL_HW              (DPE_BASE_HW + 0x080)
#define DVS_CRC_IN_HW                (DPE_BASE_HW + 0x08C)
#define DVS_DRAM_STA0_HW             (DPE_BASE_HW + 0x090)
#define DVS_DRAM_STA1_HW             (DPE_BASE_HW + 0x094)
#define DVS_DRAM_ULT_HW              (DPE_BASE_HW + 0x098)
#define DVS_DRAM_PITCH_HW            (DPE_BASE_HW + 0x09C)
#define DVS_SRC_00_HW                (DPE_BASE_HW + 0x100)
#define DVS_SRC_01_HW                (DPE_BASE_HW + 0x104)
#define DVS_SRC_02_HW                (DPE_BASE_HW + 0x108)
#define DVS_SRC_03_HW                (DPE_BASE_HW + 0x10C)
#define DVS_SRC_04_HW                (DPE_BASE_HW + 0x110)
#define DVS_SRC_05_L_FRM0_HW         (DPE_BASE_HW + 0x114)
#define DVS_SRC_06_L_FRM1_HW         (DPE_BASE_HW + 0x118)
#define DVS_SRC_07_L_FRM2_HW         (DPE_BASE_HW + 0x11C)
#define DVS_SRC_08_L_FRM3_HW         (DPE_BASE_HW + 0x120)
#define DVS_SRC_09_R_FRM0_HW         (DPE_BASE_HW + 0x124)
#define DVS_SRC_10_R_FRM1_HW         (DPE_BASE_HW + 0x128)
#define DVS_SRC_11_R_FRM2_HW         (DPE_BASE_HW + 0x12C)
#define DVS_SRC_12_R_FRM3_HW         (DPE_BASE_HW + 0x130)
#define DVS_SRC_13_L_VMAP0_HW        (DPE_BASE_HW + 0x134)
#define DVS_SRC_14_L_VMAP1_HW        (DPE_BASE_HW + 0x138)
#define DVS_SRC_15_L_VMAP2_HW        (DPE_BASE_HW + 0x13C)
#define DVS_SRC_16_L_VMAP3_HW        (DPE_BASE_HW + 0x140)
#define DVS_SRC_17_R_VMAP0_HW        (DPE_BASE_HW + 0x144)
#define DVS_SRC_18_R_VMAP1_HW        (DPE_BASE_HW + 0x148)
#define DVS_SRC_19_R_VMAP2_HW        (DPE_BASE_HW + 0x14C)
#define DVS_SRC_20_R_VMAP3_HW        (DPE_BASE_HW + 0x150)
#define DVS_SRC_21_P4_L_DV_ADR_HW    (DPE_BASE_HW + 0x154)
//#define DVS_SRC_22_MEDV0_HW          (DPE_BASE_HW + 0x158)
//#define DVS_SRC_23_MEDV1_HW          (DPE_BASE_HW + 0x15C)
//#define DVS_SRC_24_MEDV2_HW          (DPE_BASE_HW + 0x160)
//#define DVS_SRC_25_MEDV3_HW          (DPE_BASE_HW + 0x164)
#define DVS_SRC_26_OCCDV0_HW         (DPE_BASE_HW + 0x168)
#define DVS_SRC_27_OCCDV1_HW         (DPE_BASE_HW + 0x16C)
#define DVS_SRC_28_OCCDV2_HW         (DPE_BASE_HW + 0x170)
#define DVS_SRC_29_OCCDV3_HW         (DPE_BASE_HW + 0x174)
//#define DVS_SRC_30_DCV_CONF0_HW      (DPE_BASE_HW + 0x178)
//#define DVS_SRC_31_DCV_CONF1_HW      (DPE_BASE_HW + 0x17C)
//#define DVS_SRC_32_DCV_CONF2_HW      (DPE_BASE_HW + 0x180)
//#define DVS_SRC_33_DCV_CONF3_HW      (DPE_BASE_HW + 0x184)
#define DVS_SRC_34_P4_R_DV_ADR_HW     (DPE_BASE_HW + 0x188)
//#define DVS_SRC_35_DCV_L_FRM1_HW     (DPE_BASE_HW + 0x18C)
//#define DVS_SRC_36_DCV_L_FRM2_HW     (DPE_BASE_HW + 0x190)
//#define DVS_SRC_37_DCV_L_FRM3_HW     (DPE_BASE_HW + 0x194)
//#define DVS_SRC_38_DCV_R_FRM0_HW     (DPE_BASE_HW + 0x198)
//#define DVS_SRC_39_DCV_R_FRM1_HW     (DPE_BASE_HW + 0x19C)
//#define DVS_SRC_40_DCV_R_FRM2_HW     (DPE_BASE_HW + 0x1A0)
//#define DVS_SRC_41_DCV_R_FRM3_HW     (DPE_BASE_HW + 0x1A4)
#define DVS_SRC_42_OCCDV_EXT0_HW     (DPE_BASE_HW + 0x1A8)
#define DVS_SRC_43_OCCDV_EXT1_HW     (DPE_BASE_HW + 0x1AC)
#define DVS_SRC_44_OCCDV_EXT2_HW     (DPE_BASE_HW + 0x1B0)
#define DVS_SRC_45_OCCDV_EXT3_HW     (DPE_BASE_HW + 0x1B4)
#define DVS_SRC_46_HW              (DPE_BASE_HW + 0x1B8)
#define DVS_CRC_OUT_0_HW             (DPE_BASE_HW + 0x1C0)
#define DVS_CRC_OUT_1_HW             (DPE_BASE_HW + 0x1C4)
#define DVS_CRC_OUT_2_HW             (DPE_BASE_HW + 0x1C8)
#define DVS_CRC_OUT_3_HW             (DPE_BASE_HW + 0x1CC)
#define DVS_DRAM_SEC_HW              (DPE_BASE_HW + 0x1D0)
//#define DVS_PD_SRC_00_L_FRM0_HW      (DPE_BASE_HW + 0x200)
//#define DVS_PD_SRC_01_L_FRM1_HW      (DPE_BASE_HW + 0x204)
//#define DVS_PD_SRC_02_L_FRM2_HW      (DPE_BASE_HW + 0x208)
//#define DVS_PD_SRC_03_L_FRM3_HW      (DPE_BASE_HW + 0x20C)
//#define DVS_PD_SRC_04_R_FRM0_HW      (DPE_BASE_HW + 0x210)
//#define DVS_PD_SRC_05_R_FRM1_HW      (DPE_BASE_HW + 0x214)
//#define DVS_PD_SRC_06_R_FRM2_HW      (DPE_BASE_HW + 0x218)
//#define DVS_PD_SRC_07_R_FRM3_HW      (DPE_BASE_HW + 0x21C)
//#define DVS_PD_SRC_08_OCCDV0_HW      (DPE_BASE_HW + 0x220)
//#define DVS_PD_SRC_09_OCCDV1_HW      (DPE_BASE_HW + 0x224)
//#define DVS_PD_SRC_10_OCCDV2_HW      (DPE_BASE_HW + 0x228)
//#define DVS_PD_SRC_11_OCCDV3_HW      (DPE_BASE_HW + 0x22C)
//#define DVS_PD_SRC_12_OCCDV_EXT0_HW  (DPE_BASE_HW + 0x230)
//#define DVS_PD_SRC_13_OCCDV_EXT1_HW  (DPE_BASE_HW + 0x234)
//#define DVS_PD_SRC_14_OCCDV_EXT2_HW  (DPE_BASE_HW + 0x238)
//#define DVS_PD_SRC_15_OCCDV_EXT3_HW  (DPE_BASE_HW + 0x23C)
//#define DVS_PD_SRC_16_DCV_CONF0_HW   (DPE_BASE_HW + 0x240)
//#define DVS_PD_SRC_17_DCV_CONF1_HW   (DPE_BASE_HW + 0x244)
//#define DVS_PD_SRC_18_DCV_CONF2_HW   (DPE_BASE_HW + 0x248)
//#define DVS_PD_SRC_19_DCV_CONF3_HW   (DPE_BASE_HW + 0x24C)
#define DVS_CTRL_RESERVED_HW         (DPE_BASE_HW + 0x2F8)
#define DVS_CTRL_ATPG_HW             (DPE_BASE_HW + 0x2FC)
#define DVS_ME_00_HW                 (DPE_BASE_HW + 0x300)
#define DVS_ME_01_HW                 (DPE_BASE_HW + 0x304)
#define DVS_ME_02_HW                 (DPE_BASE_HW + 0x308)
#define DVS_ME_03_HW                 (DPE_BASE_HW + 0x30C)
#define DVS_ME_04_HW                 (DPE_BASE_HW + 0x310)
//#define DVS_ME_05_HW                 (DPE_BASE_HW + 0x314)
#define DVS_ME_06_HW                 (DPE_BASE_HW + 0x318)
#define DVS_ME_07_HW                 (DPE_BASE_HW + 0x31C)
#define DVS_ME_08_HW                 (DPE_BASE_HW + 0x320)
#define DVS_ME_09_HW                 (DPE_BASE_HW + 0x324)
#define DVS_ME_10_HW                 (DPE_BASE_HW + 0x328)
#define DVS_ME_11_HW                 (DPE_BASE_HW + 0x32C)
#define DVS_ME_12_HW                 (DPE_BASE_HW + 0x330)
#define DVS_ME_13_HW                 (DPE_BASE_HW + 0x334)
#define DVS_ME_14_HW                 (DPE_BASE_HW + 0x338)
#define DVS_ME_15_HW                 (DPE_BASE_HW + 0x33C)
#define DVS_ME_16_HW                 (DPE_BASE_HW + 0x340)
#define DVS_ME_17_HW                 (DPE_BASE_HW + 0x344)
#define DVS_ME_18_HW                 (DPE_BASE_HW + 0x348)
#define DVS_ME_19_HW                 (DPE_BASE_HW + 0x34C)
#define DVS_ME_20_HW                 (DPE_BASE_HW + 0x350)
#define DVS_ME_21_HW                 (DPE_BASE_HW + 0x354)
#define DVS_ME_22_HW                 (DPE_BASE_HW + 0x358)
#define DVS_ME_23_HW                 (DPE_BASE_HW + 0x35C)
#define DVS_ME_24_HW                 (DPE_BASE_HW + 0x360)
#define DVS_ME_25_HW                 (DPE_BASE_HW + 0x364)
#define DVS_ME_26_HW                 (DPE_BASE_HW + 0x368)
#define DVS_ME_27_HW                 (DPE_BASE_HW + 0x36C)
#define DVS_ME_28_HW                 (DPE_BASE_HW + 0x370)
#define DVS_ME_29_HW                 (DPE_BASE_HW + 0x374)
#define DVS_ME_30_HW                 (DPE_BASE_HW + 0x378)
#define DVS_ME_31_HW                 (DPE_BASE_HW + 0x37C)
#define DVS_ME_32_HW                 (DPE_BASE_HW + 0x380)
#define DVS_ME_33_HW                 (DPE_BASE_HW + 0x384)
#define DVS_ME_34_HW                 (DPE_BASE_HW + 0x388)
#define DVS_ME_35_HW                 (DPE_BASE_HW + 0x38C)
#define DVS_ME_36_HW                 (DPE_BASE_HW + 0x390)
#define DVS_ME_37_HW                 (DPE_BASE_HW + 0x394)
#define DVS_ME_38_HW                 (DPE_BASE_HW + 0x398)
#define DVS_ME_39_HW                 (DPE_BASE_HW + 0x39C)
//#define DVS_STATUS_00_HW             (DPE_BASE_HW + 0x3E0)
//#define DVS_STATUS_01_HW             (DPE_BASE_HW + 0x3E4)
//#define DVS_STATUS_02_HW             (DPE_BASE_HW + 0x3E8)
//#define DVS_STATUS_03_HW             (DPE_BASE_HW + 0x3EC)
#define DVS_DEBUG_HW                 (DPE_BASE_HW + 0x3F4)
#define DVS_ME_RESERVED_HW           (DPE_BASE_HW + 0x3F8)
#define DVS_ME_ATPG_HW               (DPE_BASE_HW + 0x3FC)
#define DVS_ME_40_HW                 (DPE_BASE_HW + 0x400)
#define DVS_ME_41_HW                 (DPE_BASE_HW + 0x404)
#define DVS_OCC_PQ_0_HW              (DPE_BASE_HW + 0x3A0)
#define DVS_OCC_PQ_1_HW              (DPE_BASE_HW + 0x3A4)
#define DVS_OCC_PQ_2_HW              (DPE_BASE_HW + 0x3A8)
#define DVS_OCC_PQ_3_HW              (DPE_BASE_HW + 0x3AC)
#define DVS_OCC_PQ_4_HW              (DPE_BASE_HW + 0x3B0)
#define DVS_OCC_PQ_5_HW              (DPE_BASE_HW + 0x3B4)
#define DVS_OCC_PQ_6_HW              (DPE_BASE_HW + 0x3B8)
#define DVS_OCC_PQ_7_HW              (DPE_BASE_HW + 0x3BC)
#define DVS_OCC_PQ_8_HW              (DPE_BASE_HW + 0x3C0)
#define DVS_OCC_PQ_9_HW              (DPE_BASE_HW + 0x3C4)
#define DVS_OCC_PQ_10_HW             (DPE_BASE_HW + 0x3C8)
#define DVS_OCC_PQ_11_HW             (DPE_BASE_HW + 0x3CC)
#define DVS_OCC_ATPG_HW              (DPE_BASE_HW + 0x3D0)
#define DVP_CTRL00_HW                (DPE_BASE_HW + 0x800)
#define DVP_CTRL01_HW                (DPE_BASE_HW + 0x804)
#define DVP_CTRL02_HW                (DPE_BASE_HW + 0x808)
#define DVP_CTRL03_HW                (DPE_BASE_HW + 0x80C)
#define DVP_CTRL04_HW                (DPE_BASE_HW + 0x810)
//#define DVP_CTRL05_HW                (DPE_BASE_HW + 0x814)
//#define DVP_CTRL06_HW                (DPE_BASE_HW + 0x818)
#define DVP_CTRL07_HW                (DPE_BASE_HW + 0x81C)
#define DVP_IRQ_00_HW                (DPE_BASE_HW + 0x840)
#define DVP_IRQ_01_HW                (DPE_BASE_HW + 0x844)
#define DVP_CTRL_STATUS0_HW          (DPE_BASE_HW + 0x850)
#define DVP_CTRL_STATUS1_HW          (DPE_BASE_HW + 0x854)
#define DVP_CTRL_STATUS2_HW          (DPE_BASE_HW + 0x858)
#define DVP_IRQ_STATUS_HW            (DPE_BASE_HW + 0x85C)
#define DVP_FRM_STATUS0_HW           (DPE_BASE_HW + 0x860)
//#define DVP_FRM_STATUS1_HW           (DPE_BASE_HW + 0x864)
#define DVP_FRM_STATUS2_HW           (DPE_BASE_HW + 0x868)
//#define DVP_FRM_STATUS3_HW           (DPE_BASE_HW + 0x86C)
#define DVP_CUR_STATUS_HW            (DPE_BASE_HW + 0x870)
#define DVP_SRC_00_HW                (DPE_BASE_HW + 0x880)
#define DVP_SRC_01_HW                (DPE_BASE_HW + 0x884)
#define DVP_SRC_02_HW                (DPE_BASE_HW + 0x888)
#define DVP_SRC_03_HW                (DPE_BASE_HW + 0x88C)
#define DVP_SRC_04_HW                (DPE_BASE_HW + 0x890)
#define DVP_SRC_05_Y_FRM0_HW         (DPE_BASE_HW + 0x894)
#define DVP_SRC_06_Y_FRM1_HW         (DPE_BASE_HW + 0x898)
#define DVP_SRC_07_Y_FRM2_HW         (DPE_BASE_HW + 0x89C)
#define DVP_SRC_08_Y_FRM3_HW         (DPE_BASE_HW + 0x8A0)
#define DVP_SRC_09_C_FRM0_HW         (DPE_BASE_HW + 0x8A4)
#define DVP_SRC_10_C_FRM1_HW         (DPE_BASE_HW + 0x8A8)
#define DVP_SRC_11_C_FRM2_HW         (DPE_BASE_HW + 0x8AC)
#define DVP_SRC_12_C_FRM3_HW         (DPE_BASE_HW + 0x8B0)
#define DVP_SRC_13_OCCDV0_HW         (DPE_BASE_HW + 0x8B4)
#define DVP_SRC_14_OCCDV1_HW         (DPE_BASE_HW + 0x8B8)
#define DVP_SRC_15_OCCDV2_HW         (DPE_BASE_HW + 0x8BC)
#define DVP_SRC_16_OCCDV3_HW         (DPE_BASE_HW + 0x8C0)
#define DVP_SRC_17_CRM_HW            (DPE_BASE_HW + 0x8C4)
#define DVP_SRC_18_ASF_RMDV_HW       (DPE_BASE_HW + 0x8C8)
#define DVP_SRC_19_ASF_RDDV_HW       (DPE_BASE_HW + 0x8CC)
#define DVP_SRC_20_ASF_DV0_HW        (DPE_BASE_HW + 0x8D0)
#define DVP_SRC_21_ASF_DV1_HW        (DPE_BASE_HW + 0x8D4)
#define DVP_SRC_22_ASF_DV2_HW        (DPE_BASE_HW + 0x8D8)
#define DVP_SRC_23_ASF_DV3_HW        (DPE_BASE_HW + 0x8DC)
#define DVP_SRC_24_WMF_HFDV_HW       (DPE_BASE_HW + 0x8E0)
#define DVP_SRC_25_WMF_DV0_HW        (DPE_BASE_HW + 0x8E4)
#define DVP_SRC_26_WMF_DV1_HW        (DPE_BASE_HW + 0x8E8)
#define DVP_SRC_27_WMF_DV2_HW        (DPE_BASE_HW + 0x8EC)
#define DVP_SRC_28_WMF_DV3_HW        (DPE_BASE_HW + 0x8F0)
#define DVP_CORE_00_HW               (DPE_BASE_HW + 0x900)
#define DVP_CORE_01_HW               (DPE_BASE_HW + 0x904)
#define DVP_CORE_02_HW               (DPE_BASE_HW + 0x908)
#define DVP_CORE_03_HW               (DPE_BASE_HW + 0x90C)
#define DVP_CORE_04_HW               (DPE_BASE_HW + 0x910)
#define DVP_CORE_05_HW               (DPE_BASE_HW + 0x914)
#define DVP_CORE_06_HW               (DPE_BASE_HW + 0x918)
#define DVP_CORE_07_HW               (DPE_BASE_HW + 0x91C)
#define DVP_CORE_08_HW               (DPE_BASE_HW + 0x920)
#define DVP_CORE_09_HW               (DPE_BASE_HW + 0x924)
#define DVP_CORE_10_HW               (DPE_BASE_HW + 0x928)
#define DVP_CORE_11_HW               (DPE_BASE_HW + 0x92C)
#define DVP_CORE_12_HW               (DPE_BASE_HW + 0x930)
#define DVP_CORE_13_HW               (DPE_BASE_HW + 0x934)
#define DVP_CORE_14_HW               (DPE_BASE_HW + 0x938)
#define DVP_CORE_15_HW               (DPE_BASE_HW + 0x93C)
#define DVP_CORE_16_HW               (DPE_BASE_HW + 0x940)
//#define DVP_CORE_16_HW               (DPE_BASE_HW + 0x940)
//#define DVP_CORE_17_HW               (DPE_BASE_HW + 0x944)
//#define DVP_CORE_18_HW               (DPE_BASE_HW + 0x948)
//#define DVP_CORE_19_HW               (DPE_BASE_HW + 0x94C)
//#define DVP_CORE_20_HW               (DPE_BASE_HW + 0x950)
//#define DVP_CORE_21_HW               (DPE_BASE_HW + 0x954)
//#define DVP_CORE_22_HW               (DPE_BASE_HW + 0x958)
#define DVP_SRC_CTRL_HW              (DPE_BASE_HW + 0x9F4)
#define DVP_CTRL_RESERVED_HW         (DPE_BASE_HW + 0x9F8)
#define DVP_CTRL_ATPG_HW             (DPE_BASE_HW + 0x9FC)
#define DVP_CRC_OUT_0_HW             (DPE_BASE_HW + 0xA00)
#define DVP_CRC_OUT_1_HW             (DPE_BASE_HW + 0xA04)
#define DVP_CRC_OUT_2_HW             (DPE_BASE_HW + 0xA08)
#define DVP_CRC_CTRL_HW              (DPE_BASE_HW + 0xA60)
#define DVP_CRC_OUT_HW               (DPE_BASE_HW + 0xA64)
#define DVP_CRC_IN_HW                (DPE_BASE_HW + 0xA6C)
#define DVP_DRAM_STA_HW              (DPE_BASE_HW + 0xA70)
#define DVP_DRAM_ULT_HW              (DPE_BASE_HW + 0xA74)
#define DVP_DRAM_PITCH_HW            (DPE_BASE_HW + 0xA78)
#define DVP_CORE_CRC_IN_HW           (DPE_BASE_HW + 0xA8C)
#define DVP_EXT_SRC_13_OCCDV0_HW     (DPE_BASE_HW + 0xBB4)
#define DVP_EXT_SRC_14_OCCDV1_HW     (DPE_BASE_HW + 0xBB8)
#define DVP_EXT_SRC_15_OCCDV2_HW     (DPE_BASE_HW + 0xBBC)
#define DVP_EXT_SRC_16_OCCDV3_HW     (DPE_BASE_HW + 0xBC0)
#define DVP_EXT_SRC_18_ASF_RMDV_HW   (DPE_BASE_HW + 0xBC8)
#define DVP_EXT_SRC_19_ASF_RDDV_HW   (DPE_BASE_HW + 0xBCC)
#define DVP_EXT_SRC_20_ASF_DV0_HW    (DPE_BASE_HW + 0xBD0)
#define DVP_EXT_SRC_21_ASF_DV1_HW    (DPE_BASE_HW + 0xBD4)
#define DVP_EXT_SRC_22_ASF_DV2_HW    (DPE_BASE_HW + 0xBD8)
#define DVP_EXT_SRC_23_ASF_DV3_HW    (DPE_BASE_HW + 0xBDC)
/*SW Access Registers : using mapped base address from DTS*/
#define DVS_CTRL00_REG               (ISP_DPE_BASE)
#define DVS_CTRL01_REG               (ISP_DPE_BASE + 0x004)
#define DVS_CTRL02_REG               (ISP_DPE_BASE + 0x008)
#define DVS_CTRL03_REG               (ISP_DPE_BASE + 0x00C)
//#define DVS_CTRL04_REG               (ISP_DPE_BASE + 0x010)
//#define DVS_CTRL05_REG               (ISP_DPE_BASE + 0x014)
#define DVS_CTRL06_REG               (ISP_DPE_BASE + 0x018)
#define DVS_CTRL07_REG               (ISP_DPE_BASE + 0x01C)
//#define DVS_OCC_PQ_0_REG             (ISP_DPE_BASE + 0x030)
//#define DVS_OCC_PQ_1_REG             (ISP_DPE_BASE + 0x034)
//#define DVS_OCC_ATPG_REG             (ISP_DPE_BASE + 0x038)
#define DVS_IRQ_00_REG               (ISP_DPE_BASE + 0x040)
#define DVS_CTRL_STATUS0_REG         (ISP_DPE_BASE + 0x050)
//#define DVS_CTRL_STATUS1_REG         (ISP_DPE_BASE + 0x054)
#define DVS_CTRL_STATUS2_REG         (ISP_DPE_BASE + 0x058)
#define DVS_IRQ_STATUS_REG           (ISP_DPE_BASE + 0x05C)
#define DVS_FRM_STATUS0_REG          (ISP_DPE_BASE + 0x060)
#define DVS_FRM_STATUS1_REG          (ISP_DPE_BASE + 0x064)
//#define DVS_FRM_STATUS2_REG          (ISP_DPE_BASE + 0x068)
//#define DVS_FRM_STATUS3_REG          (ISP_DPE_BASE + 0x06C)
#define DVS_CUR_STATUS_REG           (ISP_DPE_BASE + 0x070)
#define DVS_SRC_CTRL_REG             (ISP_DPE_BASE + 0x074)
#define DVS_CRC_CTRL_REG             (ISP_DPE_BASE + 0x080)
#define DVS_CRC_IN_REG               (ISP_DPE_BASE + 0x08C)
#define DVS_DRAM_STA0_REG            (ISP_DPE_BASE + 0x090)
#define DVS_DRAM_STA1_REG            (ISP_DPE_BASE + 0x094)
#define DVS_DRAM_ULT_REG             (ISP_DPE_BASE + 0x098)
#define DVS_DRAM_PITCH_REG           (ISP_DPE_BASE + 0x09C)
#define DVS_SRC_00_REG               (ISP_DPE_BASE + 0x100)
#define DVS_SRC_01_REG               (ISP_DPE_BASE + 0x104)
#define DVS_SRC_02_REG               (ISP_DPE_BASE + 0x108)
#define DVS_SRC_03_REG               (ISP_DPE_BASE + 0x10C)
#define DVS_SRC_04_REG               (ISP_DPE_BASE + 0x110)
#define DVS_SRC_05_L_FRM0_REG        (ISP_DPE_BASE + 0x114)
#define DVS_SRC_06_L_FRM1_REG        (ISP_DPE_BASE + 0x118)
#define DVS_SRC_07_L_FRM2_REG        (ISP_DPE_BASE + 0x11C)
#define DVS_SRC_08_L_FRM3_REG        (ISP_DPE_BASE + 0x120)
#define DVS_SRC_09_R_FRM0_REG        (ISP_DPE_BASE + 0x124)
#define DVS_SRC_10_R_FRM1_REG        (ISP_DPE_BASE + 0x128)
#define DVS_SRC_11_R_FRM2_REG        (ISP_DPE_BASE + 0x12C)
#define DVS_SRC_12_R_FRM3_REG        (ISP_DPE_BASE + 0x130)
#define DVS_SRC_13_L_VMAP0_REG       (ISP_DPE_BASE + 0x134)
#define DVS_SRC_14_L_VMAP1_REG       (ISP_DPE_BASE + 0x138)
#define DVS_SRC_15_L_VMAP2_REG       (ISP_DPE_BASE + 0x13C)
#define DVS_SRC_16_L_VMAP3_REG       (ISP_DPE_BASE + 0x140)
#define DVS_SRC_17_R_VMAP0_REG       (ISP_DPE_BASE + 0x144)
#define DVS_SRC_18_R_VMAP1_REG       (ISP_DPE_BASE + 0x148)
#define DVS_SRC_19_R_VMAP2_REG       (ISP_DPE_BASE + 0x14C)
#define DVS_SRC_20_R_VMAP3_REG       (ISP_DPE_BASE + 0x150)
#define DVS_SRC_21_P4_L_DV_ADR_REG    (ISP_DPE_BASE + 0x154)
//#define DVS_SRC_22_MEDV0_REG         (ISP_DPE_BASE + 0x158)
//#define DVS_SRC_23_MEDV1_REG         (ISP_DPE_BASE + 0x15C)
//#define DVS_SRC_24_MEDV2_REG         (ISP_DPE_BASE + 0x160)
//#define DVS_SRC_25_MEDV3_REG         (ISP_DPE_BASE + 0x164)
#define DVS_SRC_26_OCCDV0_REG        (ISP_DPE_BASE + 0x168)
#define DVS_SRC_27_OCCDV1_REG        (ISP_DPE_BASE + 0x16C)
#define DVS_SRC_28_OCCDV2_REG        (ISP_DPE_BASE + 0x170)
#define DVS_SRC_29_OCCDV3_REG        (ISP_DPE_BASE + 0x174)
#define DVS_SRC_34_P4_R_DV_ADR_REG    (ISP_DPE_BASE + 0x188)
//#define DVS_SRC_35_DCV_L_FRM1_REG    (ISP_DPE_BASE + 0x18C)
//#define DVS_SRC_36_DCV_L_FRM2_REG    (ISP_DPE_BASE + 0x190)
//#define DVS_SRC_37_DCV_L_FRM3_REG    (ISP_DPE_BASE + 0x194)
//#define DVS_SRC_38_DCV_R_FRM0_REG    (ISP_DPE_BASE + 0x198)
//#define DVS_SRC_39_DCV_R_FRM1_REG    (ISP_DPE_BASE + 0x19C)
//#define DVS_SRC_40_DCV_R_FRM2_REG    (ISP_DPE_BASE + 0x1A0)
//#define DVS_SRC_41_DCV_R_FRM3_REG    (ISP_DPE_BASE + 0x1A4)
#define DVS_SRC_42_OCCDV_EXT0_REG    (ISP_DPE_BASE + 0x1A8)
#define DVS_SRC_43_OCCDV_EXT1_REG    (ISP_DPE_BASE + 0x1AC)
#define DVS_SRC_44_OCCDV_EXT2_REG    (ISP_DPE_BASE + 0x1B0)
#define DVS_SRC_45_OCCDV_EXT3_REG    (ISP_DPE_BASE + 0x1B4)
#define DVS_SRC_46_REG    (ISP_DPE_BASE + 0x1B8)
#define DVS_CRC_OUT_0_REG            (ISP_DPE_BASE + 0x1C0)
#define DVS_CRC_OUT_1_REG            (ISP_DPE_BASE + 0x1C4)
#define DVS_CRC_OUT_2_REG            (ISP_DPE_BASE + 0x1C8)
#define DVS_CRC_OUT_3_REG            (ISP_DPE_BASE + 0x1CC)
#define DVS_DRAM_SEC_REG             (ISP_DPE_BASE + 0x1D0)
//#define DVS_PD_SRC_00_L_FRM0_REG     (ISP_DPE_BASE + 0x200)
//#define DVS_PD_SRC_01_L_FRM1_REG     (ISP_DPE_BASE + 0x204)
//#define DVS_PD_SRC_02_L_FRM2_REG     (ISP_DPE_BASE + 0x208)
//#define DVS_PD_SRC_03_L_FRM3_REG     (ISP_DPE_BASE + 0x20C)
//#define DVS_PD_SRC_04_R_FRM0_REG     (ISP_DPE_BASE + 0x210)
//#define DVS_PD_SRC_05_R_FRM1_REG     (ISP_DPE_BASE + 0x214)
//#define DVS_PD_SRC_06_R_FRM2_REG     (ISP_DPE_BASE + 0x218)
//#define DVS_PD_SRC_07_R_FRM3_REG     (ISP_DPE_BASE + 0x21C)
//#define DVS_PD_SRC_08_OCCDV0_REG     (ISP_DPE_BASE + 0x220)
//#define DVS_PD_SRC_09_OCCDV1_REG     (ISP_DPE_BASE + 0x224)
//#define DVS_PD_SRC_10_OCCDV2_REG     (ISP_DPE_BASE + 0x228)
//#define DVS_PD_SRC_11_OCCDV3_REG     (ISP_DPE_BASE + 0x22C)
//#define DVS_PD_SRC_12_OCCDV_EXT0_REG (ISP_DPE_BASE + 0x230)
//#define DVS_PD_SRC_13_OCCDV_EXT1_REG (ISP_DPE_BASE + 0x234)
//#define DVS_PD_SRC_14_OCCDV_EXT2_REG (ISP_DPE_BASE + 0x238)
//#define DVS_PD_SRC_15_OCCDV_EXT3_REG (ISP_DPE_BASE + 0x23C)
//#define DVS_PD_SRC_16_DCV_CONF0_REG  (ISP_DPE_BASE + 0x240)
//#define DVS_PD_SRC_17_DCV_CONF1_REG  (ISP_DPE_BASE + 0x244)
//#define DVS_PD_SRC_18_DCV_CONF2_REG  (ISP_DPE_BASE + 0x248)
//#define DVS_PD_SRC_19_DCV_CONF3_REG  (ISP_DPE_BASE + 0x24C)
#define DVS_CTRL_RESERVED_REG        (ISP_DPE_BASE + 0x2F8)
#define DVS_CTRL_ATPG_REG            (ISP_DPE_BASE + 0x2FC)
#define DVS_ME_00_REG                (ISP_DPE_BASE + 0x300)
#define DVS_ME_01_REG                (ISP_DPE_BASE + 0x304)
#define DVS_ME_02_REG                (ISP_DPE_BASE + 0x308)
#define DVS_ME_03_REG                (ISP_DPE_BASE + 0x30C)
#define DVS_ME_04_REG                (ISP_DPE_BASE + 0x310)
//#define DVS_ME_05_REG                (ISP_DPE_BASE + 0x314)
#define DVS_ME_06_REG                (ISP_DPE_BASE + 0x318)
#define DVS_ME_07_REG                (ISP_DPE_BASE + 0x31C)
#define DVS_ME_08_REG                (ISP_DPE_BASE + 0x320)
#define DVS_ME_09_REG                (ISP_DPE_BASE + 0x324)
#define DVS_ME_10_REG                (ISP_DPE_BASE + 0x328)
#define DVS_ME_11_REG                (ISP_DPE_BASE + 0x32C)
#define DVS_ME_12_REG                (ISP_DPE_BASE + 0x330)
#define DVS_ME_13_REG                (ISP_DPE_BASE + 0x334)
#define DVS_ME_14_REG                (ISP_DPE_BASE + 0x338)
#define DVS_ME_15_REG                (ISP_DPE_BASE + 0x33C)
#define DVS_ME_16_REG                (ISP_DPE_BASE + 0x340)
#define DVS_ME_17_REG                (ISP_DPE_BASE + 0x344)
#define DVS_ME_18_REG                (ISP_DPE_BASE + 0x348)
#define DVS_ME_19_REG                (ISP_DPE_BASE + 0x34C)
#define DVS_ME_20_REG                (ISP_DPE_BASE + 0x350)
#define DVS_ME_21_REG                (ISP_DPE_BASE + 0x354)
#define DVS_ME_22_REG                (ISP_DPE_BASE + 0x358)
#define DVS_ME_23_REG                (ISP_DPE_BASE + 0x35C)
#define DVS_ME_24_REG                (ISP_DPE_BASE + 0x360)
#define DVS_ME_25_REG                (ISP_DPE_BASE + 0x364)
#define DVS_ME_26_REG                (ISP_DPE_BASE + 0x368)
#define DVS_ME_27_REG                (ISP_DPE_BASE + 0x36C)
#define DVS_ME_28_REG                (ISP_DPE_BASE + 0x370)
#define DVS_ME_29_REG                (ISP_DPE_BASE + 0x374)
#define DVS_ME_30_REG                (ISP_DPE_BASE + 0x378)
#define DVS_ME_31_REG                (ISP_DPE_BASE + 0x37C)
#define DVS_ME_32_REG                (ISP_DPE_BASE + 0x380)
#define DVS_ME_33_REG                (ISP_DPE_BASE + 0x384)
#define DVS_ME_34_REG                (ISP_DPE_BASE + 0x388)
#define DVS_ME_35_REG                (ISP_DPE_BASE + 0x38C)
#define DVS_ME_36_REG                (ISP_DPE_BASE + 0x390)
#define DVS_ME_37_REG                (ISP_DPE_BASE + 0x394)
#define DVS_ME_38_REG                (ISP_DPE_BASE + 0x398)
#define DVS_ME_39_REG                (ISP_DPE_BASE + 0x39C)
//#define DVS_STATUS_00_REG            (ISP_DPE_BASE + 0x3E0)
//#define DVS_STATUS_01_REG            (ISP_DPE_BASE + 0x3E4)
//#define DVS_STATUS_02_REG            (ISP_DPE_BASE + 0x3E8)
//#define DVS_STATUS_03_REG            (ISP_DPE_BASE + 0x3EC)
#define DVS_DEBUG_REG                (ISP_DPE_BASE + 0x3F4)
#define DVS_ME_RESERVED_REG          (ISP_DPE_BASE + 0x3F8)
#define DVS_ME_ATPG_REG              (ISP_DPE_BASE + 0x3FC)
#define DVS_ME_40_REG                (ISP_DPE_BASE + 0x400)
#define DVS_ME_41_REG                (ISP_DPE_BASE + 0x404)
#define DVS_OCC_PQ_0_REG             (ISP_DPE_BASE + 0x3A0)
#define DVS_OCC_PQ_1_REG             (ISP_DPE_BASE + 0x3A4)
#define DVS_OCC_PQ_2_REG             (ISP_DPE_BASE + 0x3A8)
#define DVS_OCC_PQ_3_REG             (ISP_DPE_BASE + 0x3AC)
#define DVS_OCC_PQ_4_REG             (ISP_DPE_BASE + 0x3B0)
#define DVS_OCC_PQ_5_REG             (ISP_DPE_BASE + 0x3B4)
#define DVS_OCC_PQ_6_REG             (ISP_DPE_BASE + 0x3B8)
#define DVS_OCC_PQ_7_REG             (ISP_DPE_BASE + 0x3BC)
#define DVS_OCC_PQ_8_REG             (ISP_DPE_BASE + 0x3C0)
#define DVS_OCC_PQ_9_REG             (ISP_DPE_BASE + 0x3C4)
#define DVS_OCC_PQ_10_REG            (ISP_DPE_BASE + 0x3C8)
#define DVS_OCC_PQ_11_REG            (ISP_DPE_BASE + 0x3CC)
#define DVS_OCC_ATPG_REG             (ISP_DPE_BASE + 0x3D0)
#define DVP_CTRL00_REG               (ISP_DPE_BASE + 0x800)
#define DVP_CTRL01_REG               (ISP_DPE_BASE + 0x804)
#define DVP_CTRL02_REG               (ISP_DPE_BASE + 0x808)
#define DVP_CTRL03_REG               (ISP_DPE_BASE + 0x80C)
#define DVP_CTRL04_REG               (ISP_DPE_BASE + 0x810)
//#define DVP_CTRL05_REG               (ISP_DPE_BASE + 0x814)
//#define DVP_CTRL06_REG               (ISP_DPE_BASE + 0x818)
#define DVP_CTRL07_REG               (ISP_DPE_BASE + 0x81C)
#define DVP_IRQ_00_REG               (ISP_DPE_BASE + 0x840)
#define DVP_IRQ_01_REG               (ISP_DPE_BASE + 0x844)
#define DVP_CTRL_STATUS0_REG         (ISP_DPE_BASE + 0x850)
#define DVP_CTRL_STATUS1_REG         (ISP_DPE_BASE + 0x854)
#define DVP_CTRL_STATUS2_REG         (ISP_DPE_BASE + 0x858)
#define DVP_IRQ_STATUS_REG           (ISP_DPE_BASE + 0x85C)
#define DVP_FRM_STATUS0_REG          (ISP_DPE_BASE + 0x860)
//#define DVP_FRM_STATUS1_REG          (ISP_DPE_BASE + 0x864)
#define DVP_FRM_STATUS2_REG          (ISP_DPE_BASE + 0x868)
//#define DVP_FRM_STATUS3_REG          (ISP_DPE_BASE + 0x86C)
#define DVP_CUR_STATUS_REG           (ISP_DPE_BASE + 0x870)
#define DVP_SRC_00_REG               (ISP_DPE_BASE + 0x880)
#define DVP_SRC_01_REG               (ISP_DPE_BASE + 0x884)
#define DVP_SRC_02_REG               (ISP_DPE_BASE + 0x888)
#define DVP_SRC_03_REG               (ISP_DPE_BASE + 0x88C)
#define DVP_SRC_04_REG               (ISP_DPE_BASE + 0x890)
#define DVP_SRC_05_Y_FRM0_REG        (ISP_DPE_BASE + 0x894)
#define DVP_SRC_06_Y_FRM1_REG        (ISP_DPE_BASE + 0x898)
#define DVP_SRC_07_Y_FRM2_REG        (ISP_DPE_BASE + 0x89C)
#define DVP_SRC_08_Y_FRM3_REG        (ISP_DPE_BASE + 0x8A0)
#define DVP_SRC_09_C_FRM0_REG        (ISP_DPE_BASE + 0x8A4)
#define DVP_SRC_10_C_FRM1_REG        (ISP_DPE_BASE + 0x8A8)
#define DVP_SRC_11_C_FRM2_REG        (ISP_DPE_BASE + 0x8AC)
#define DVP_SRC_12_C_FRM3_REG        (ISP_DPE_BASE + 0x8B0)
#define DVP_SRC_13_OCCDV0_REG        (ISP_DPE_BASE + 0x8B4)
#define DVP_SRC_14_OCCDV1_REG        (ISP_DPE_BASE + 0x8B8)
#define DVP_SRC_15_OCCDV2_REG        (ISP_DPE_BASE + 0x8BC)
#define DVP_SRC_16_OCCDV3_REG        (ISP_DPE_BASE + 0x8C0)
#define DVP_SRC_17_CRM_REG           (ISP_DPE_BASE + 0x8C4)
#define DVP_SRC_18_ASF_RMDV_REG      (ISP_DPE_BASE + 0x8C8)
#define DVP_SRC_19_ASF_RDDV_REG      (ISP_DPE_BASE + 0x8CC)
#define DVP_SRC_20_ASF_DV0_REG       (ISP_DPE_BASE + 0x8D0)
#define DVP_SRC_21_ASF_DV1_REG       (ISP_DPE_BASE + 0x8D4)
#define DVP_SRC_22_ASF_DV2_REG       (ISP_DPE_BASE + 0x8D8)
#define DVP_SRC_23_ASF_DV3_REG       (ISP_DPE_BASE + 0x8DC)
#define DVP_SRC_24_WMF_HFDV_REG      (ISP_DPE_BASE + 0x8E0)
#define DVP_SRC_25_WMF_DV0_REG       (ISP_DPE_BASE + 0x8E4)
#define DVP_SRC_26_WMF_DV1_REG       (ISP_DPE_BASE + 0x8E8)
#define DVP_SRC_27_WMF_DV2_REG       (ISP_DPE_BASE + 0x8EC)
#define DVP_SRC_28_WMF_DV3_REG       (ISP_DPE_BASE + 0x8F0)
#define DVP_CORE_00_REG              (ISP_DPE_BASE + 0x900)
#define DVP_CORE_01_REG              (ISP_DPE_BASE + 0x904)
#define DVP_CORE_02_REG              (ISP_DPE_BASE + 0x908)
#define DVP_CORE_03_REG              (ISP_DPE_BASE + 0x90C)
#define DVP_CORE_04_REG              (ISP_DPE_BASE + 0x910)
#define DVP_CORE_05_REG              (ISP_DPE_BASE + 0x914)
#define DVP_CORE_06_REG              (ISP_DPE_BASE + 0x918)
#define DVP_CORE_07_REG              (ISP_DPE_BASE + 0x91C)
#define DVP_CORE_08_REG              (ISP_DPE_BASE + 0x920)
#define DVP_CORE_09_REG              (ISP_DPE_BASE + 0x924)
#define DVP_CORE_10_REG              (ISP_DPE_BASE + 0x928)
#define DVP_CORE_11_REG              (ISP_DPE_BASE + 0x92C)
#define DVP_CORE_12_REG              (ISP_DPE_BASE + 0x930)
#define DVP_CORE_13_REG              (ISP_DPE_BASE + 0x934)
#define DVP_CORE_14_REG              (ISP_DPE_BASE + 0x938)
#define DVP_CORE_15_REG              (ISP_DPE_BASE + 0x93C)
#define DVP_CORE_16_REG              (ISP_DPE_BASE + 0x940)
//#define DVP_CORE_16_REG              (ISP_DPE_BASE + 0x940)
//#define DVP_CORE_17_REG              (ISP_DPE_BASE + 0x944)
//#define DVP_CORE_18_REG              (ISP_DPE_BASE + 0x948)
//#define DVP_CORE_19_REG              (ISP_DPE_BASE + 0x94C)
//#define DVP_CORE_20_REG              (ISP_DPE_BASE + 0x950)
//#define DVP_CORE_21_REG              (ISP_DPE_BASE + 0x954)
//#define DVP_CORE_22_REG              (ISP_DPE_BASE + 0x958)
#define DVP_SRC_CTRL_REG             (ISP_DPE_BASE + 0x9F4)
#define DVP_CTRL_RESERVED_REG        (ISP_DPE_BASE + 0x9F8)
#define DVP_CTRL_ATPG_REG            (ISP_DPE_BASE + 0x9FC)
#define DVP_CRC_OUT_0_REG            (ISP_DPE_BASE + 0xA00)
#define DVP_CRC_OUT_1_REG            (ISP_DPE_BASE + 0xA04)
#define DVP_CRC_OUT_2_REG            (ISP_DPE_BASE + 0xA08)
#define DVP_CRC_CTRL_REG             (ISP_DPE_BASE + 0xA60)
#define DVP_CRC_OUT_REG              (ISP_DPE_BASE + 0xA64)
#define DVP_CRC_IN_REG               (ISP_DPE_BASE + 0xA6C)
#define DVP_DRAM_STA_REG             (ISP_DPE_BASE + 0xA70)
#define DVP_DRAM_ULT_REG             (ISP_DPE_BASE + 0xA74)
#define DVP_DRAM_PITCH_REG           (ISP_DPE_BASE + 0xA78)
#define DVP_CORE_CRC_IN_REG          (ISP_DPE_BASE + 0xA8C)
#define DVP_EXT_SRC_13_OCCDV0_REG    (ISP_DPE_BASE + 0xBB4)
#define DVP_EXT_SRC_14_OCCDV1_REG    (ISP_DPE_BASE + 0xBB8)
#define DVP_EXT_SRC_15_OCCDV2_REG    (ISP_DPE_BASE + 0xBBC)
#define DVP_EXT_SRC_16_OCCDV3_REG    (ISP_DPE_BASE + 0xBC0)
#define DVP_EXT_SRC_18_ASF_RMDV_REG  (ISP_DPE_BASE + 0xBC8)
#define DVP_EXT_SRC_19_ASF_RDDV_REG  (ISP_DPE_BASE + 0xBCC)
#define DVP_EXT_SRC_20_ASF_DV0_REG   (ISP_DPE_BASE + 0xBD0)
#define DVP_EXT_SRC_21_ASF_DV1_REG   (ISP_DPE_BASE + 0xBD4)
#define DVP_EXT_SRC_22_ASF_DV2_REG   (ISP_DPE_BASE + 0xBD8)
#define DVP_EXT_SRC_23_ASF_DV3_REG   (ISP_DPE_BASE + 0xBDC)
#define DPE_MAX_REG_CNT              (0xBE0 >> 2)
/**************************************************************
 *
 **************************************************************/
static inline unsigned int DPE_MsToJiffies(unsigned int Ms)
{
	return ((Ms * HZ + 512) >> 10);
}
/**************************************************************
 *
 **************************************************************/
static inline unsigned int DPE_UsToJiffies(unsigned int Us)
{
	return (((Us / 1000) * HZ + 512) >> 10);
}
/**************************************************************
 *
 **************************************************************/
static inline unsigned int
DPE_GetIRQState(unsigned int type, unsigned int userNumber, unsigned int stus,
		enum DPE_PROCESS_ID_ENUM whichReq, int ProcessID)
{
	unsigned int ret = 0;
	unsigned int p;
	unsigned long flags;

	p = ProcessID % IRQ_USER_NUM_MAX;
	spin_lock_irqsave(&(DPEInfo.SpinLockIrq[type]), flags);
	if (stus & DPE_INT_ST) {
		ret = ((DPEInfo.IrqInfo.DpeIrqCnt[p] > 0) &&
		       (DPEInfo.IrqInfo.ProcessID[p] == ProcessID));
	} else {
		LOG_ERR("EWIRQ,type:%d,u:%d,stat:%d,wReq:%d,PID:0x%x\n",
			type, userNumber, stus, p, ProcessID);
	}
	spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[type]), flags);
	/*  */
	return ret;
}
/**************************************************************
 *
 **************************************************************/
static inline unsigned int DPE_JiffiesToMs(unsigned int Jiffies)
{
	return ((Jiffies * 1000) / HZ);
}
#define RegDump(start, end)                                                \
{                                                                          \
unsigned int i;                                                            \
for (i = start; i <= end; i += 0x10) {                                     \
	LOG_DBG("[0x%08X %08X],[0x%08X %08X],[0x%08X %08X],[0x%08X %08X]", \
		(unsigned int)(ISP_DPE_BASE + i),                          \
		(unsigned int)DPE_RD32(ISP_DPE_BASE + i),                  \
		(unsigned int)(ISP_DPE_BASE + i + 0x4),                    \
		(unsigned int)DPE_RD32(ISP_DPE_BASE + i +                  \
				       0x4),                               \
		(unsigned int)(ISP_DPE_BASE + i + 0x8),                    \
		(unsigned int)DPE_RD32(ISP_DPE_BASE + i +                  \
				       0x8),                               \
		(unsigned int)(ISP_DPE_BASE + i + 0xc),                    \
		(unsigned int)DPE_RD32(ISP_DPE_BASE + i +                  \
				       0xc));                              \
}                                                                          \
}
/* Native hardware descriptors; ownership is private to one queued request. */
struct dpe_map_desc {
	size_t address, fd, offset;
	unsigned int stage, conditional;
};
#define DPE_MAP_DESC(n, st, cond) { offsetof(struct DPE_Config, n), \
	offsetof(struct DPE_Config, DPE_DMapSettings.n ## _fd), \
	offsetof(struct DPE_Config, DPE_DMapSettings.n ## _Ofs), st, cond }
static const struct dpe_map_desc dpe_map_descs[] = {
	DPE_MAP_DESC(Dpe_InBuf_SrcImg_Y_L, 0, 0),
	DPE_MAP_DESC(Dpe_InBuf_SrcImg_Y_R, 0, 0),
	DPE_MAP_DESC(Dpe_InBuf_ValidMap_L, 0, 0),
	DPE_MAP_DESC(Dpe_InBuf_ValidMap_R, 0, 0),
	DPE_MAP_DESC(Dpe_OutBuf_OCC, 0, 0),
	DPE_MAP_DESC(Dpe_OutBuf_OCC_Ext, 0, 0),
	DPE_MAP_DESC(Dpe_InBuf_SrcImg_Y_L_Pre, 0, 1),
	DPE_MAP_DESC(Dpe_InBuf_SrcImg_Y_R_Pre, 0, 1),
	DPE_MAP_DESC(Dpe_InBuf_P4_L_DV, 0, 1),
	DPE_MAP_DESC(Dpe_InBuf_P4_R_DV, 0, 1),
	DPE_MAP_DESC(Dpe_InBuf_SrcImg_Y, 1, 0),
	DPE_MAP_DESC(Dpe_InBuf_SrcImg_C, 1, 0),
	DPE_MAP_DESC(Dpe_InBuf_OCC, 1, 0),
	DPE_MAP_DESC(Dpe_OutBuf_CRM, 1, 0),
	DPE_MAP_DESC(Dpe_OutBuf_ASF_RD, 1, 0),
	DPE_MAP_DESC(Dpe_OutBuf_ASF_HF, 1, 0),
	DPE_MAP_DESC(Dpe_OutBuf_WMF_FILT, 1, 0),
	DPE_MAP_DESC(Dpe_InBuf_OCC_Ext, 1, 2),
	DPE_MAP_DESC(Dpe_OutBuf_ASF_RD_Ext, 1, 2),
	DPE_MAP_DESC(Dpe_OutBuf_ASF_HF_Ext, 1, 2),
};
#undef DPE_MAP_DESC
static const size_t dpe_address_offsets[] = {
	offsetof(struct DPE_Config, Dpe_InBuf_SrcImg_Y_L),
	offsetof(struct DPE_Config, Dpe_InBuf_SrcImg_Y_R),
	offsetof(struct DPE_Config, Dpe_InBuf_SrcImg_Y),
	offsetof(struct DPE_Config, Dpe_InBuf_SrcImg_C),
	offsetof(struct DPE_Config, Dpe_InBuf_ValidMap_L),
	offsetof(struct DPE_Config, Dpe_InBuf_ValidMap_R),
	offsetof(struct DPE_Config, Dpe_OutBuf_CONF),
	offsetof(struct DPE_Config, Dpe_OutBuf_OCC),
	offsetof(struct DPE_Config, Dpe_OutBuf_OCC_Ext),
	offsetof(struct DPE_Config, Dpe_InBuf_OCC),
	offsetof(struct DPE_Config, Dpe_InBuf_OCC_Ext),
	offsetof(struct DPE_Config, Dpe_OutBuf_CRM),
	offsetof(struct DPE_Config, Dpe_OutBuf_ASF_RM),
	offsetof(struct DPE_Config, Dpe_OutBuf_ASF_RM_Ext),
	offsetof(struct DPE_Config, Dpe_OutBuf_ASF_RD),
	offsetof(struct DPE_Config, Dpe_OutBuf_ASF_RD_Ext),
	offsetof(struct DPE_Config, Dpe_OutBuf_ASF_HF),
	offsetof(struct DPE_Config, Dpe_OutBuf_ASF_HF_Ext),
	offsetof(struct DPE_Config, Dpe_OutBuf_WMF_HF),
	offsetof(struct DPE_Config, Dpe_OutBuf_WMF_FILT),
	offsetof(struct DPE_Config, DVP_SRC_18_ASF_RMDV),
	offsetof(struct DPE_Config, DVP_SRC_24_WMF_HFDV),
	offsetof(struct DPE_Config, DVP_EXT_SRC_18_ASF_RMDV),
	offsetof(struct DPE_Config, Dpe_InBuf_SrcImg_Y_L_Pre),
	offsetof(struct DPE_Config, Dpe_InBuf_SrcImg_Y_R_Pre),
	offsetof(struct DPE_Config, Dpe_InBuf_P4_L_DV),
	offsetof(struct DPE_Config, Dpe_InBuf_P4_R_DV),
};
struct dpe_request_owner {
	unsigned int count, expanded;
	unsigned int first[MAX_FRAMES_PER_REQUEST];
	dma_addr_t client_addresses[MAX_FRAMES_PER_REQUEST][ARRAY_SIZE(dpe_address_offsets)];
	struct DPE_Config configs[MAX_FRAMES_PER_REQUEST];
	struct tee_mmu maps[MAX_FRAMES_PER_REQUEST][ARRAY_SIZE(dpe_map_descs)];
};
static void dpe_unmap_one(struct tee_mmu *m)
{
	if (m->sgt) dma_buf_unmap_attachment_unlocked(m->attach, m->sgt, DMA_BIDIRECTIONAL);
	if (m->attach) dma_buf_detach(m->dma_buf, m->attach);
	if (m->dma_buf) dma_buf_put(m->dma_buf);
	memset(m, 0, sizeof(*m));
}
static int dpe_map_one(struct tee_mmu *m, int fd, unsigned int offset,
		       dma_addr_t *address)
{
	struct scatterlist *sg;
	u64 base, end, next;
	unsigned int i;
	int ret;

	/* Zero is the native absent-fd sentinel; preserve direct-address clients. */
	if (fd <= 0) return 0;
	m->dma_buf = dma_buf_get(fd);
	if (IS_ERR(m->dma_buf)) { ret = PTR_ERR(m->dma_buf); m->dma_buf = NULL; return ret; }
	if (offset >= m->dma_buf->size) { ret = -EINVAL; goto fail; }
	m->attach = dma_buf_attach(m->dma_buf, gdev);
	if (IS_ERR(m->attach)) { ret = PTR_ERR(m->attach); m->attach = NULL; goto fail; }
	m->sgt = dma_buf_map_attachment_unlocked(m->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(m->sgt)) { ret = PTR_ERR(m->sgt); m->sgt = NULL; goto fail; }
	if (!m->sgt || !m->sgt->sgl || !m->sgt->nents) { ret = -EINVAL; goto fail; }
	base = sg_dma_address(m->sgt->sgl);
	next = base;
	for_each_sg(m->sgt->sgl, sg, m->sgt->nents, i) {
		if (sg_dma_address(sg) != next ||
		    check_add_overflow(next, (u64)sg_dma_len(sg), &end) ||
		    end > DMA_BIT_MASK(34) + 1ULL) { ret = -ERANGE; goto fail; }
		next = end;
	}
	if (next - base < m->dma_buf->size ||
	    check_add_overflow(base, (u64)offset, &end)) { ret = -ERANGE; goto fail; }
	*address = end;
	return 0;
fail:
	dpe_unmap_one(m);
	return ret;
}
static void dpe_free_owner(struct dpe_request_owner *o)
{
	unsigned int i, j;
	if (!o) return;
	for (i = o->count; i; i--)
		for (j = ARRAY_SIZE(dpe_map_descs); j; j--)
			dpe_unmap_one(&o->maps[i - 1][j - 1]);
	kfree(o);
}
static void dpe_release_request(struct frame *frames)
{
	struct dpe_request_owner *o = frames[0].owner;
	frames[0].owner = NULL;
	dpe_free_owner(o);
}
static int dpe_map_config(struct dpe_request_owner *o, unsigned int index)
{
	struct DPE_Config *c = &o->configs[index];
	unsigned int j;
	for (j = 0; j < ARRAY_SIZE(dpe_map_descs); j++) {
		const struct dpe_map_desc *d = &dpe_map_descs[j];
		unsigned int fd, offset;
		int ret;
		if ((!d->stage && c->Dpe_engineSelect == MODE_DVP_ONLY) ||
		    (d->stage && c->Dpe_engineSelect == MODE_DVS_ONLY)) continue;
		if (d->conditional == 1 && !(c->Dpe_DVSSettings.TuningBuf_ME.DVS_ME_28 & 0x400)) continue;
		if (d->conditional == 2 &&
		    !((c->Dpe_is16BitMode == 1 && !c->Dpe_DVPSettings.SubModule_EN.asf_conf_en) ||
		      (c->Dpe_is16BitMode == 0 && c->Dpe_DVPSettings.SubModule_EN.asf_conf_en))) continue;
		fd = *(unsigned int *)((char *)c + d->fd);
		offset = *(unsigned int *)((char *)c + d->offset);
		ret = dpe_map_one(&o->maps[index][j], (int)fd, offset,
			(dma_addr_t *)((char *)c + d->address));
		if (ret) return ret;
	}
	return 0;
}
static int dpe_count_frames(const struct DPE_Request *req, unsigned int *total)
{
	unsigned int i, n, count = 0;
	if (!req || !req->m_pDpeConfig || !req->m_ReqNum ||
	    req->m_ReqNum > MAX_FRAMES_PER_REQUEST) return -EINVAL;
	for (i = 0; i < req->m_ReqNum; i++) {
		const struct DPE_Config *c = &req->m_pDpeConfig[i];
		if (c->Dpe_engineSelect < MODE_DVS_DVP_BOTH ||
		    c->Dpe_engineSelect > MODE_DVP_ONLY ||
		    c->Dpe_engineSelect != req->m_pDpeConfig[0].Dpe_engineSelect) return -EINVAL;
		n = c->Dpe_DVSSettings.is_pd_mode ? c->Dpe_DVSSettings.pd_frame_num : 1;
		if (!n || n > MAX_FRAMES_PER_REQUEST - count) return -E2BIG;
		count += n;
	}
	*total = count;
	return 0;
}
/* Preserve the native ISP70 PD tile offsets and 64-pixel search range. The
 * original code's search index was always zero; do not change native tuning. */
static void dpe_make_tile(struct DPE_Config *dst, const struct DPE_Config *src,
			 unsigned int tile, unsigned int count)
{
	unsigned int search = tile == 0 ? 0 : tile == count - 1 ? 128 : 64;
	unsigned int offset = tile * src->Dpe_DVSSettings.occ_width - search;
	unsigned int input = src->Dpe_DVSSettings.pd_st_x + offset;
	bool p4 = src->Dpe_DVSSettings.TuningBuf_ME.DVS_ME_28 & 0x400;
	*dst = *src;
	dst->Dpe_DVSSettings.input_offset = offset;
	dst->Dpe_DVSSettings.occ_start_x = search;
#define TILE_INPUT(n) do { if (src->n) dst->n = src->n + input; } while (0)
	TILE_INPUT(Dpe_InBuf_SrcImg_Y_L);
	TILE_INPUT(Dpe_InBuf_SrcImg_Y_R);
	TILE_INPUT(Dpe_InBuf_ValidMap_L);
	TILE_INPUT(Dpe_InBuf_ValidMap_R);
	if (p4) {
		TILE_INPUT(Dpe_InBuf_SrcImg_Y_L_Pre);
		TILE_INPUT(Dpe_InBuf_SrcImg_Y_R_Pre);
	}
#undef TILE_INPUT
	if (src->Dpe_OutBuf_OCC)
		dst->Dpe_OutBuf_OCC += tile * src->Dpe_DVSSettings.occ_width;
	if (src->Dpe_OutBuf_OCC_Ext)
		dst->Dpe_OutBuf_OCC_Ext += tile * src->Dpe_DVSSettings.occ_width;
}
signed int dpe_enque_cb(struct frame *frames, void *data)
{
	struct DPE_Request *req = data;
	struct dpe_request_owner *o;
	unsigned int total, i, j, t, f = 0, n;
	int ret = dpe_count_frames(req, &total);
	if (ret) return ret;
	if (!frames || frames[0].owner) return -EBUSY;
	for (f = 0; f < total; f++) if (!frames[f].data) return -EINVAL;
	o = kzalloc(sizeof(*o), GFP_KERNEL);
	if (!o) return -ENOMEM;
	o->count = req->m_ReqNum;
	o->expanded = total;
	memcpy(o->configs, req->m_pDpeConfig, o->count * sizeof(*o->configs));
	f = 0;
	for (i = 0; i < o->count; i++) {
		for (j = 0; j < ARRAY_SIZE(dpe_address_offsets); j++)
			o->client_addresses[i][j] = *(dma_addr_t *)((char *)&o->configs[i] + dpe_address_offsets[j]);
		ret = dpe_map_config(o, i);
		if (ret) goto fail;
		n = o->configs[i].Dpe_DVSSettings.is_pd_mode ?
			o->configs[i].Dpe_DVSSettings.pd_frame_num : 1;
		o->first[i] = f;
		for (t = 0; t < n; t++, f++) {
			if (o->configs[i].Dpe_DVSSettings.is_pd_mode)
				dpe_make_tile(frames[f].data, &o->configs[i], t, n);
			else
				memcpy(frames[f].data, &o->configs[i], sizeof(struct DPE_Config));
		}
	}
	frames[0].owner = o;
	return total;
fail:
	dpe_free_owner(o);
	return ret;
}
static int dpe_deque_cb_common(struct frame *frames, void *data, bool native_wire)
{
	struct DPE_Request *req = data;
	struct dpe_request_owner *o;
	unsigned int i, j;
	if (!frames || !req || !req->m_pDpeConfig) return -EINVAL;
	o = frames[0].owner;
	if (!o || req->m_ReqNum < o->count) return -ENOSPC;
	for (i = 0; i < o->count; i++) {
		memcpy(&req->m_pDpeConfig[i], frames[o->first[i]].data,
			sizeof(struct DPE_Config));
		/* A native V4L2 client supplied FDs plus32bit descriptors, not our
		 * resolved34bit IOVAs. Keep those client-owned values unchanged. */
		if (native_wire)
			for (j = 0; j < ARRAY_SIZE(dpe_address_offsets); j++)
				*(dma_addr_t *)((char *)&req->m_pDpeConfig[i] + dpe_address_offsets[j]) =
					o->client_addresses[i][j];
	}
	req->m_ReqNum = o->count;
	dpe_release_request(frames);
	return 0;
}
signed int dpe_deque_cb(struct frame *frames, void *data)
{
	return dpe_deque_cb_common(frames, data, false);
}
static int dpe_deque_wire_cb(struct frame *frames, void *data)
{
	return dpe_deque_cb_common(frames, data, true);
}
void DPE_Config_DVS(struct DPE_Config *pDpeConfig,
	struct DPE_Kernel_Config *pConfigToKernel)
{
	unsigned int frmWidth = pDpeConfig->Dpe_DVSSettings.frm_width;
	unsigned int frmHeight = pDpeConfig->Dpe_DVSSettings.frm_height;
	unsigned int L_engStartX = pDpeConfig->Dpe_DVSSettings.l_eng_start_x;
	unsigned int R_engStartX = pDpeConfig->Dpe_DVSSettings.r_eng_start_x;
	unsigned int engStartY = pDpeConfig->Dpe_DVSSettings.eng_start_y;
	unsigned int engWidth = pDpeConfig->Dpe_DVSSettings.eng_width;
	unsigned int engHeight = pDpeConfig->Dpe_DVSSettings.eng_height;
	unsigned int occWidth = pDpeConfig->Dpe_DVSSettings.occ_width;
	unsigned int occStartX = pDpeConfig->Dpe_DVSSettings.occ_start_x;
	unsigned int DVS_OUT_ADJ_En = pDpeConfig->Dpe_DVSSettings.out_adj_en;
	unsigned int DVS_OUT_ADJ_Dv_En = pDpeConfig->Dpe_DVSSettings.out_adj_dv_en;
	unsigned int DVS_OUT_ADJ_Dv_WIDTH = pDpeConfig->Dpe_DVSSettings.out_adj_dv_width;
	unsigned int DVS_OUT_ADJ_Dv_HIGHT = pDpeConfig->Dpe_DVSSettings.out_adj_dv_high;

	// pitch / 16
	unsigned int pitch = pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch >> 4;
	unsigned int full_tile_width = pDpeConfig->Dpe_DVSSettings.dram_out_pitch >> 4;
	mutex_lock(&gFDMutex);
	DPE_P4_EN = ((pDpeConfig->Dpe_DVSSettings.TuningBuf_ME.DVS_ME_28 & 0x400)>>10);
	mutex_unlock(&gFDMutex);
	//if (DVS_OUT_ADJ_En == 0)
	//	DVS_OUT_ADJ_Dv_WIDTH = occWidth;

	LOG_INF(
	"DVS param: frm w/h(%d/%d), engStart X_L/X_R/Y(%d/%d/%d), eng w/h(%d/%d), occ w(%d), occ startX(%d), pitch(%d), main eye(%d), 16bit mode(%d), sbf/conf/occ_en(%d/%d/%d), Dpe_InBuf_SrcImg_Y_L: (0x%0llx), Dpe_InBuf_SrcImg_Y_R(0x%0llx), Dpe_InBuf_ValidMap_L(0x%0llx), Dpe_InBuf_ValidMap_R(0x%0llx), Dpe_OutBuf_CONF(0x%0llx), Dpe_OutBuf_OCC(0x%0llx)\n",
	frmWidth, frmHeight, L_engStartX, R_engStartX, engStartY,
	engWidth, engHeight, occWidth, occStartX, pitch,
	pDpeConfig->Dpe_DVSSettings.mainEyeSel, pDpeConfig->Dpe_is16BitMode,
	pDpeConfig->Dpe_DVSSettings.SubModule_EN.sbf_en,
	pDpeConfig->Dpe_DVSSettings.SubModule_EN.conf_en,
	pDpeConfig->Dpe_DVSSettings.SubModule_EN.occ_en,
	pDpeConfig->Dpe_InBuf_SrcImg_Y_L, pDpeConfig->Dpe_InBuf_SrcImg_Y_R,
	pDpeConfig->Dpe_InBuf_ValidMap_L, pDpeConfig->Dpe_InBuf_ValidMap_R,
	pDpeConfig->Dpe_OutBuf_CONF, pDpeConfig->Dpe_OutBuf_OCC);
	#ifdef DPE_debug_log_en
	LOG_INF(
	"Dpe_InBuf_SrcImg_Y_L: (0x%lx), Dpe_InBuf_SrcImg_Y_R(0x%lx), Dpe_InBuf_ValidMap_L(0x%lx), Dpe_InBuf_ValidMap_R(0x%lx), Dpe_OutBuf_CONF(0x%lx), Dpe_OutBuf_OCC(0x%lx)\n",
	pDpeConfig->Dpe_InBuf_SrcImg_Y_L,
	pDpeConfig->Dpe_InBuf_SrcImg_Y_R,
	pDpeConfig->Dpe_InBuf_ValidMap_L, pDpeConfig->Dpe_InBuf_ValidMap_R,
	pDpeConfig->Dpe_OutBuf_CONF, pDpeConfig->Dpe_OutBuf_OCC);
	#endif
	if ((frmWidth % 16 != 0))
		LOG_ERR("frame width is not 16 byte align w(%d)\n", frmWidth);
	if ((frmHeight % 2 != 0))
		LOG_ERR("frame height is not 2 byte align h(%d)\n", frmHeight);
	if ((occWidth % 16 != 0))
		LOG_ERR("occ width is not 16 byte align w(%d)\n", occWidth);
	if (L_engStartX < R_engStartX) {
		LOG_ERR("L_engStartX(%d) < R_engStartX(%d)\n",
		L_engStartX, R_engStartX);
	}
if (pDpeConfig->Dpe_engineSelect == MODE_DVS_DVP_BOTH) {
	pConfigToKernel->DVS_CTRL00 =
((pDpeConfig->Dpe_DVSSettings.Iteration.y_IterTimes & 0x3) << 0) |
((pDpeConfig->Dpe_DVSSettings.Iteration.y_IterStartDirect_0 & 0x1) << 2) |
((pDpeConfig->Dpe_DVSSettings.Iteration.y_IterStartDirect_1 & 0x1) << 3) |
((pDpeConfig->Dpe_DVSSettings.Iteration.x_IterStartDirect_0 & 0x1) << 8) |
((pDpeConfig->Dpe_DVSSettings.Iteration.x_IterStartDirect_1 & 0x1) << 9) |
(0xD << 10) |
(0x0 << 15) |
((pDpeConfig->Dpe_DVSSettings.SubModule_EN.sbf_en & 0x1) << 16) |
((pDpeConfig->Dpe_DVSSettings.SubModule_EN.occ_en & 0x1) << 18) |
((0x0 & 0x3) << 20) | // c_dvp_cur_frm
((0x0 & 0x3) << 22) | // c_dvs_cur_frm
((0x1 & 0x3) << 24) | // c_dvs_prev_frm //!ISP7
((0x3 & 0x3) << 27) | // c_dvp_trig_en, c_dvs_trig_en
((0x2 & 0x3) << 29)	| // c_dpe_fw_trig, c_dpe_fw_trig_en
((0x1 & 0x1) << 31); // c_dvs_en
} else if (pDpeConfig->Dpe_engineSelect == MODE_DVS_ONLY) {
	pConfigToKernel->DVS_CTRL00 =
((pDpeConfig->Dpe_DVSSettings.Iteration.y_IterTimes & 0x3) << 0) |
((pDpeConfig->Dpe_DVSSettings.Iteration.y_IterStartDirect_0 & 0x1) << 2) |
((pDpeConfig->Dpe_DVSSettings.Iteration.y_IterStartDirect_1 & 0x1) << 3) |
((pDpeConfig->Dpe_DVSSettings.Iteration.x_IterStartDirect_0 & 0x1) << 8) |
((pDpeConfig->Dpe_DVSSettings.Iteration.x_IterStartDirect_1 & 0x1) << 9) |
(0xD << 10) |
(0x0 << 15) |
((pDpeConfig->Dpe_DVSSettings.SubModule_EN.sbf_en & 0x1) << 16) |
((pDpeConfig->Dpe_DVSSettings.SubModule_EN.occ_en & 0x1) << 18) |
((0x0 & 0x3) << 20) | // c_dvp_cur_frm
((0x0 & 0x3) << 22) | // c_dvs_cur_frm
((0x1 & 0x3) << 24) | // c_dvs_prev_frm //!ISP7 Temporal Stable ?V P4 on WTA
((0x2 & 0x3) << 27) | // c_dvp_trig_en, c_dvs_trig_en
((0x2 & 0x3) << 29) | // c_dpe_fw_trig, c_dpe_fw_trig_en
((0x1 & 0x1) << 31); // c_dvs_en
}
	//!ISP7 Tile mode
	if (pDpeConfig->Dpe_DVSSettings.is_pd_mode) {
		pConfigToKernel->DVS_DRAM_PITCH =
		((pitch) & 0x3FF) | ((pitch & 0x3FF) << 10) |
		((pDpeConfig->Dpe_DVSSettings.dram_out_pitch_en & 0x01) << 31) |
		((full_tile_width & 0x3FF) << 20); //!ISP7
		pConfigToKernel->DVS_SRC_00 =
		((pDpeConfig->Dpe_is16BitMode & 0x1) << 0) | //!c_dv16b_mode = 1 ;ISP7 only
		((pDpeConfig->Dpe_DVSSettings.mainEyeSel & 0x1) << 1) |
		((0x0 & 0x1) << 8) | // c_vmap_in_pxl
		((0x0 & 0x1) << 9); // c_vmap_all_vld
	} else {
		pConfigToKernel->DVS_DRAM_PITCH =
		((pitch) & 0x3FF) | ((pitch & 0x3FF) << 10) |
		((pDpeConfig->Dpe_DVSSettings.dram_out_pitch_en & 0x01) << 31) |
		((full_tile_width & 0x3FF) << 20); //!ISP7

		pConfigToKernel->DVS_SRC_00 =
		((pDpeConfig->Dpe_is16BitMode & 0x1) << 0) | //!c_dv16b_mode = 1 ;ISP7 only
		((pDpeConfig->Dpe_DVSSettings.mainEyeSel & 0x1) << 1) |
		((0x0 & 0x1) << 8) | // c_vmap_in_pxl
		((0x0 & 0x1) << 9); // c_vmap_all_vld
	}

	pConfigToKernel->DVS_SRC_01 =
	((frmHeight & 0x7FF) << 0) | ((frmWidth & 0x7FF) << 12);
	pConfigToKernel->DVS_SRC_02 =
	((engHeight & 0x7FF) << 0) | ((engWidth & 0x7FF) << 12);
	pConfigToKernel->DVS_SRC_03 =
	((R_engStartX & 0x7FF) << 0) |
	((L_engStartX & 0x7FF) << 12) |
	((engStartY & 0xFF) << 24);
	pConfigToKernel->DVS_SRC_04 =
	((occWidth & 0x7FF) << 0) | ((occStartX & 0x7FF) << 12);
	//LOG_INF("DVS_SRC_04 =(0x%lx)", pConfigToKernel->DVS_SRC_04);
	if (pDpeConfig->Dpe_InBuf_SrcImg_Y_L != 0x0) {
		pConfigToKernel->DVS_SRC_05_L_FRM0 =
		pDpeConfig->Dpe_InBuf_SrcImg_Y_L;
	} else
		LOG_ERR("No Left Src Image Y!\n");

	//LOG_INF("DVS_SRC_05_L_FRM0 =(0x%lx)",
	//pConfigToKernel->DVS_SRC_05_L_FRM0);

	if (pDpeConfig->Dpe_InBuf_SrcImg_Y_R != 0x0) {
		pConfigToKernel->DVS_SRC_09_R_FRM0 =
		pDpeConfig->Dpe_InBuf_SrcImg_Y_R;
	}	else
		LOG_ERR("No Right Src Image Y!\n");

	//LOG_INF("DVS_SRC_09_R_FRM0 =(0x%lx)",
	//pConfigToKernel->DVS_SRC_09_R_FRM0);
	mutex_lock(&gFDMutex);
	if (DPE_P4_EN == 1) {
		LOG_INF("dpe_p4_enable == 1\n");
		if (pDpeConfig->Dpe_InBuf_SrcImg_Y_L_Pre != 0x0) {
			pConfigToKernel->DVS_SRC_06_L_FRM1 =
			pDpeConfig->Dpe_InBuf_SrcImg_Y_L_Pre;
		} else
			LOG_ERR("No Pre Left Src Image Y!\n");

	if (pDpeConfig->Dpe_InBuf_SrcImg_Y_R_Pre != 0x0) {
		pConfigToKernel->DVS_SRC_10_R_FRM1 =
		pDpeConfig->Dpe_InBuf_SrcImg_Y_R_Pre;
	} else
		LOG_ERR("No Right Src Image Y!\n");

	if (pDpeConfig->Dpe_InBuf_P4_L_DV != 0x0) {
		pConfigToKernel->DVS_SRC_21_P4_L_DV_ADR =
		pDpeConfig->Dpe_InBuf_P4_L_DV;
	} else
		LOG_ERR("No DVS DVS_SRC_21_P4_L_DV Buffer!\n");

	if (pDpeConfig->Dpe_InBuf_P4_R_DV != 0x0) {
		pConfigToKernel->DVS_SRC_34_P4_R_DV_ADR =
		pDpeConfig->Dpe_InBuf_P4_R_DV;
	} else
		LOG_ERR("No DVS DVS_SRC_34_P4_R_DV Buffer!\n");

	#ifdef KERNEL_DMA_BUFFER
	pConfigToKernel->DVS_SRC_21_P4_L_DV_ADR =
	//((uintptr_t)g_dpewb_dvme_int_Buffer_pa & 0xffffffff);
	pDpeConfig->Dpe_InBuf_P4_L_DV;
	#else
	if (pDpeConfig->DVS_SRC_21_P4_L_DV_ADR != 0x0) {
		pConfigToKernel->DVS_SRC_21_P4_L_DV_ADR =
		pDpeConfig->DVS_SRC_21_P4_L_DV_ADR;
	} else
		LOG_ERR("No DVS DVS_SRC_21_INTER_MEDV Buffer!\n");

	LOG_INF("DVS_SRC_21_P4_L_DV_ADR =(0x%lx)",
	pConfigToKernel->DVS_SRC_21_P4_L_DV_ADR);
	#endif


	#ifdef KERNEL_DMA_BUFFER
	pConfigToKernel->DVS_SRC_34_P4_R_DV_ADR =
	//((uintptr_t)g_dpewb_cost_int_Buffer_pa & 0xffffffff);
	pDpeConfig->Dpe_InBuf_P4_R_DV;
	#else
	if (pDpeConfig->DVS_SRC_34_P4_R_DV_ADR != 0x0) {
		pConfigToKernel->DVS_SRC_34_P4_R_DV_ADR =
		pDpeConfig->DVS_SRC_34_P4_R_DV_ADR;
	} else
		LOG_ERR("No DVS DVS_SRC_34_DCV_L_FRM0 Buffer!\n");
	#endif
	LOG_INF("DVS_SRC_34_P4_R_DV_ADR =(0x%llx)",
	pConfigToKernel->DVS_SRC_34_P4_R_DV_ADR);
	}
	mutex_unlock(&gFDMutex);

	if (pDpeConfig->Dpe_InBuf_ValidMap_L != 0x0) {
		pConfigToKernel->DVS_SRC_13_L_VMAP0 =
		pDpeConfig->Dpe_InBuf_ValidMap_L;
		//LOG_INF("DVS_SRC_13_L_VMAP0 =(0x%lx)\n",
		//pConfigToKernel->DVS_SRC_13_L_VMAP0);
	} else
		LOG_ERR("No Left Valid Map!\n");
	if (pDpeConfig->Dpe_InBuf_ValidMap_R != 0x0) {
		pConfigToKernel->DVS_SRC_17_R_VMAP0 =
		pDpeConfig->Dpe_InBuf_ValidMap_R;
		//LOG_INF("DVS_SRC_17_R_VMAP0 =(0x%lx)\n",
		//pConfigToKernel->DVS_SRC_17_R_VMAP0);
	} else
		LOG_ERR("No Right Valid Map!\n");

	//LOG_INF("ADJ_Dv_HIGHT =(0x%lx), ADJ_Dv_WIDTH=(0x%lx), ADJ_Dv_En=(0x%lx)\n",
	//DVS_OUT_ADJ_Dv_HIGHT, DVS_OUT_ADJ_Dv_WIDTH, DVS_OUT_ADJ_Dv_En);

	//!ISP7 DVS NN Down-Sample
	pConfigToKernel->DVS_SRC_46 =
	((DVS_OUT_ADJ_En & 0x1) << 24)	|
	((DVS_OUT_ADJ_Dv_En & 0x1) << 23)	|
	((DVS_OUT_ADJ_Dv_WIDTH & 0x7FF) << 12) |
	((DVS_OUT_ADJ_Dv_HIGHT & 0x7FF) << 0);
	//!
	//LOG_INF("DVS_SRC_46 =(0x%lx)", pConfigToKernel->DVS_SRC_46);

	pConfigToKernel->DVS_CTRL_ATPG = 0x80000000;
	if (pDpeConfig->Dpe_OutBuf_OCC != 0x0) {
		pConfigToKernel->DVS_SRC_26_OCCDV0 =
		pDpeConfig->Dpe_OutBuf_OCC;
		//LOG_INF("DVS_SRC_26_OCCDV0 =(0x%lx)\n",
		//pConfigToKernel->DVS_SRC_26_OCCDV0);
	} else
		LOG_ERR("No DVS OCC Output Buffer!\n");

	if (pDpeConfig->Dpe_OutBuf_OCC_Ext != 0x0) {
		pConfigToKernel->DVS_SRC_42_OCCDV_EXT0 =
		pDpeConfig->Dpe_OutBuf_OCC_Ext;
		//LOG_INF("DVS_SRC_42_OCCDV_EXT0 =(0x%lx)",
		//pConfigToKernel->DVS_SRC_42_OCCDV_EXT0);
	} else
		LOG_ERR("No DVS Ext Output Buffer!\n");

	static_assert(offsetof(struct DPE_Kernel_Config, DVS_ME_00) +
		sizeof(pDpeConfig->Dpe_DVSSettings.TuningBuf_ME) <= sizeof(*pConfigToKernel));
	memcpy((char *)pConfigToKernel + offsetof(struct DPE_Kernel_Config, DVS_ME_00),
	&pDpeConfig->Dpe_DVSSettings.TuningBuf_ME,
	sizeof(pDpeConfig->Dpe_DVSSettings.TuningBuf_ME));
	static_assert(offsetof(struct DPE_Kernel_Config, DVS_OCC_PQ_0) +
		sizeof(pDpeConfig->Dpe_DVSSettings.TuningBuf_OCC) <= sizeof(*pConfigToKernel));
	memcpy((char *)pConfigToKernel + offsetof(struct DPE_Kernel_Config, DVS_OCC_PQ_0),
	&pDpeConfig->Dpe_DVSSettings.TuningBuf_OCC,
	sizeof(pDpeConfig->Dpe_DVSSettings.TuningBuf_OCC));

}
void DPE_Config_DVP(struct DPE_Config *pDpeConfig,
	struct DPE_Kernel_Config *pConfigToKernel)
{
	unsigned int frmWidth, frmHeight, engWidth, engHeight;
	unsigned int occWidth, occHeight;
	unsigned int engStartX, engStartY, occStartX, occStartY, pitch;
	unsigned int engStart_offset_Y, engStart_offset_C;
	unsigned int DVP_ASF_DV16b_EN = pDpeConfig->Dpe_is16BitMode;
	//unsigned int DVP_ASF_DV16b_EN = 0;
	unsigned int DVP_ASF_CONF_EN = pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_conf_en;
	unsigned int DVP_ASF_DEPTH_MODE = pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_depth_mode;

	engStartX = pDpeConfig->Dpe_DVPSettings.eng_start_x;
	engStartY = pDpeConfig->Dpe_DVPSettings.eng_start_y;
	frmWidth = pDpeConfig->Dpe_DVPSettings.frm_width - engStartX;
	frmHeight = pDpeConfig->Dpe_DVPSettings.frm_height - (engStartY * 2);
	engWidth = pDpeConfig->Dpe_DVPSettings.frm_width - (engStartX * 2);
	engHeight = frmHeight;
	occStartX = engStartX;
	occStartY = 0;
	occWidth = engWidth;
	occHeight = engHeight;

	//DPE_is16BitMode = pDpeConfig->Dpe_is16BitMode;
	// pitch = frame width / 16
	pitch = ALIGN16(pDpeConfig->Dpe_DVPSettings.frm_width) >> 4;
	engStart_offset_Y = engStartY * (pitch << 4);
	engStart_offset_C = engStart_offset_Y >> 1;
	LOG_INF(
	"DVP param: frm w/h(%d/%d), engstart X/Y(%d/%d), eng w/h(%d/%d), occ w/h(%d/%d), occstart X/Y(%d/%d), pitch(%d), eng start offset Y/C(0x%x/0x%x), Dpe_InBuf_SrcImg_Y: (0x%0llx), Dpe_InBuf_SrcImg_C(0x%0llx), Dpe_InBuf_OCC(0x%0llx), Dpe_OutBuf_CRM(0x%0llx), Dpe_OutBuf_ASF_RD(0x%0llx), Dpe_OutBuf_ASF_HF(0x%0llx), DVP_SRC_18_ASF_RMDV(0x%lx)\n",
	frmWidth, frmHeight, engStartX, engStartY, engWidth, engHeight,
	occWidth, occHeight, occStartX, occStartY, pitch,
	engStart_offset_Y, engStart_offset_C,
	pDpeConfig->Dpe_InBuf_SrcImg_Y, pDpeConfig->Dpe_InBuf_SrcImg_C,
	pDpeConfig->Dpe_InBuf_OCC, pDpeConfig->Dpe_OutBuf_CRM,
	pDpeConfig->Dpe_OutBuf_ASF_RD, pDpeConfig->Dpe_OutBuf_ASF_HF,
	((uintptr_t)g_dpewb_asfrm_Buffer_pa & 0xfffffffff));
	if ((occWidth % 16 != 0))
		LOG_ERR("occ width is not 16 byte align w (%d)\n", occWidth);

	if (DVP_ASF_DV16b_EN &&
		pDpeConfig->Dpe_DVPSettings.SubModule_EN.wmf_hf_en)
		LOG_ERR("WMF should not enable in 16 bit mode\n");

	if (DVP_ASF_DV16b_EN && DVP_ASF_CONF_EN)
		LOG_ERR("ASF CONF should not enable in 16 bit mode\n");


	// If hf rounds is odd, nb_rounds can't use.
	if (pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_hf_rounds % 2)
		pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_nb_rounds = 0;

	pConfigToKernel->DVP_CTRL04 =
	(pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_crm_en << 0) |
	(pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_rm_en << 1) |
	(pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_rd_en << 2) |
	(pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_hf_en << 3) |
	(pDpeConfig->Dpe_DVPSettings.SubModule_EN.wmf_hf_en << 4) |
	(pDpeConfig->Dpe_DVPSettings.SubModule_EN.wmf_filt_en << 5) |
	((pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_hf_rounds & (0xF)) << 8) |
	((pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_nb_rounds & (0x3)) << 12) |
	((pDpeConfig->Dpe_DVPSettings.SubModule_EN.wmf_filt_rounds & (0x3)) << 16) |
	(DVP_ASF_DV16b_EN << 18) | //!ISP7.0
	(pDpeConfig->Dpe_DVPSettings.SubModule_EN.asf_recursive_en << 19) |
	(DVP_ASF_CONF_EN << 21)|//!ISP7.0
	(DVP_ASF_DEPTH_MODE << 22);

	pConfigToKernel->DVP_DRAM_PITCH =
	((pitch & 0x7F) << 4) | ((pitch & 0x7F) << 20);
	pConfigToKernel->DVP_SRC_00 =
	((engHeight & 0x3FF) << 0) | ((engWidth & 0x3FF) << 10) |
	((pDpeConfig->Dpe_DVPSettings.Y_only & 0x1) << 29) |
	((pDpeConfig->Dpe_DVPSettings.mainEyeSel & 0x1) << 30);
	pConfigToKernel->DVP_SRC_01 =
	((frmHeight & 0x3FF) << 0) | ((frmWidth & 0x3FF) << 10);
	pConfigToKernel->DVP_SRC_02 =
	((engStartX & 0x3FF) << 0);
	// | ((engStartY & 0x3FF) << 10); for Simulation
	pConfigToKernel->DVP_SRC_03 =
	((occHeight & 0x3FF) << 0) | ((occWidth & 0x3FF) << 10);
	pConfigToKernel->DVP_SRC_04 =
	((occStartX & 0x3FF) << 0);
	// | ((occStartY & 0x3FF) << 10); for Simulation
	if (pDpeConfig->Dpe_DVPSettings.mainEyeSel == RIGHT) {
		if (pDpeConfig->Dpe_InBuf_SrcImg_Y != 0x0) {
			pConfigToKernel->DVP_SRC_05_Y_FRM0 =
			pDpeConfig->Dpe_InBuf_SrcImg_Y +
			(engStart_offset_Y);
		} else
			LOG_ERR("No DVP Right Src Image Y!\n");

	} else if (pDpeConfig->Dpe_DVPSettings.mainEyeSel == LEFT) {
		if (pDpeConfig->Dpe_InBuf_SrcImg_Y != 0x0) {
			pConfigToKernel->DVP_SRC_05_Y_FRM0 =
			pDpeConfig->Dpe_InBuf_SrcImg_Y +
			(engStart_offset_Y);
		} else
			LOG_ERR("No DVP Left Src Image Y!\n");
	}

	#ifdef DPE_debug_log_en
	LOG_INF("DVP_SRC_05_Y_FRM0 = %lx\n",
	pConfigToKernel->DVP_SRC_05_Y_FRM0);
	#endif

	if (pDpeConfig->Dpe_InBuf_SrcImg_C != 0x0) {
		pConfigToKernel->DVP_SRC_09_C_FRM0 =
		pDpeConfig->Dpe_InBuf_SrcImg_C +
		(engStart_offset_C);
	} else
		LOG_ERR("No Src Image C!\n");

	#ifdef DPE_debug_log_en
	LOG_INF("DVP_SRC_09_C_FRM0 = %lx\n",
	pConfigToKernel->DVP_SRC_09_C_FRM0);
	#endif

	if (pDpeConfig->Dpe_InBuf_OCC != 0x0) {
		pConfigToKernel->DVP_SRC_13_OCCDV0 =
		pDpeConfig->Dpe_InBuf_OCC;
	} else
		LOG_ERR("No DVP OCC In!\n");

	#ifdef DPE_debug_log_en
	LOG_INF("DVP_SRC_13_OCCDV0 = %lx\n",
	pConfigToKernel->DVP_SRC_13_OCCDV0);
	#endif

	if (pDpeConfig->Dpe_OutBuf_CRM != 0x0) {
		pConfigToKernel->DVP_SRC_17_CRM =
		pDpeConfig->Dpe_OutBuf_CRM;
	} else
		LOG_ERR("No CRM Output Buffer!\n");

	#ifdef DPE_debug_log_en
	LOG_INF("DVP_SRC_17_CRM = %lx\n",
	pConfigToKernel->DVP_SRC_17_CRM);
	#endif

	#ifdef KERNEL_DMA_BUFFER
	//LOG_INF("get kernel asf buffer\n");
	pConfigToKernel->DVP_SRC_18_ASF_RMDV =
	((dma_addr_t)g_dpewb_asfrm_Buffer_pa & 0xfffffffff);
	#else
	if (pDpeConfig->DVP_SRC_18_ASF_RMDV != 0x0) {
		pConfigToKernel->DVP_SRC_18_ASF_RMDV =
		pDpeConfig->DVP_SRC_18_ASF_RMDV;
	} else
		LOG_ERR("No DVS DVP_SRC_18_ASF_RMDV Buffer!\n");
	#endif

	#ifdef DPE_debug_log_en
	LOG_INF("DVP_SRC_18_ASF_RMDV = %lx\n",
	pConfigToKernel->DVP_SRC_18_ASF_RMDV);
	#endif

	if (pDpeConfig->Dpe_OutBuf_ASF_RD != 0x0) {
		pConfigToKernel->DVP_SRC_19_ASF_RDDV =
		pDpeConfig->Dpe_OutBuf_ASF_RD;
	} else
		LOG_ERR("No ASF_RD Output Buffer!\n");
	#ifdef DPE_debug_log_en
	LOG_INF("DVP_SRC_19_ASF_RDDV = %lx\n",
	pConfigToKernel->DVP_SRC_19_ASF_RDDV);
	#endif

	if (pDpeConfig->Dpe_OutBuf_ASF_HF != 0x0) {
		pConfigToKernel->DVP_SRC_20_ASF_DV0 =
		pDpeConfig->Dpe_OutBuf_ASF_HF;
	} else
		LOG_ERR("No ASF Output Buffer!\n");

	#ifdef DPE_debug_log_en
	LOG_INF("DVP_SRC_20_ASF_DV0 = %lx\n",
	pConfigToKernel->DVP_SRC_20_ASF_DV0);
	#endif

	if (pDpeConfig->Dpe_is16BitMode == 0) { //for WMF
		#ifdef KERNEL_DMA_BUFFER
		pConfigToKernel->DVP_SRC_24_WMF_HFDV =
		((dma_addr_t)g_dpewb_wmfhf_Buffer_pa & 0xfffffffff);
		#else
		if (pDpeConfig->DVP_SRC_24_WMF_HFDV != 0x0) {
			pConfigToKernel->DVP_SRC_24_WMF_HFDV =
			pDpeConfig->DVP_SRC_24_WMF_HFDV;
		} else
			LOG_INF("No DVS DVP_SRC_24_WMF_HFDV Buffer!\n");
		#endif

		#ifdef DPE_debug_log_en
		LOG_INF("DVP_SRC_24_WMF_HFDV = %lx\n",
		pConfigToKernel->DVP_SRC_24_WMF_HFDV);
		#endif

		if (pDpeConfig->Dpe_OutBuf_WMF_FILT != 0x0) {
			pConfigToKernel->DVP_SRC_25_WMF_DV0 =
			(dma_addr_t)pDpeConfig->Dpe_OutBuf_WMF_FILT;
		} else
			LOG_INF("No WMF Output Buffer!\n");
			spin_lock(&(DPEInfo.SpinLockFD));
			DVP_is16BitMode = 0;
			spin_unlock(&(DPEInfo.SpinLockFD));
	}
		#ifdef DPE_debug_log_en
		LOG_INF("DVP_SRC_25_WMF_DV0 = %lx\n",
		pConfigToKernel->DVP_SRC_25_WMF_DV0);
		#endif

		if (((pDpeConfig->Dpe_is16BitMode == 1) && (DVP_ASF_CONF_EN == 0)) ||
				((pDpeConfig->Dpe_is16BitMode == 0) && (DVP_ASF_CONF_EN == 1))) {
			spin_lock(&(DPEInfo.SpinLockFD));
			DVP_is16BitMode = 1;
			spin_unlock(&(DPEInfo.SpinLockFD));
			if (pDpeConfig->Dpe_InBuf_OCC_Ext != 0x0) {
				pConfigToKernel->DVP_EXT_SRC_13_OCCDV0 =
				(dma_addr_t)pDpeConfig->Dpe_InBuf_OCC_Ext;
			} else
				LOG_INF("No DVP Ext OCC Input Buffer!\n");

			LOG_INF("DVP_EXT_SRC_13_OCCDV0 = %llx\n",
			pConfigToKernel->DVP_EXT_SRC_13_OCCDV0);

			#ifdef KERNEL_DMA_BUFFER
			LOG_INF("get kernel asf ext buffer\n");
			pConfigToKernel->DVP_EXT_SRC_18_ASF_RMDV =
			((dma_addr_t)g_dpewb_asfrmext_Buffer_pa & 0xfffffffff);
			#else
			if (pDpeConfig->DVP_EXT_SRC_18_ASF_RMDV != 0x0) {
				pConfigToKernel->DVP_EXT_SRC_18_ASF_RMDV =
				pDpeConfig->DVP_EXT_SRC_18_ASF_RMDV;
			} else
				LOG_INF("No DVS DVP_EXT_SRC_18_ASF_RMDV Buffer!\n");
			#endif

			LOG_INF("DVP_EXT_SRC_18_ASF_RMDV = %llx\n",
			pConfigToKernel->DVP_EXT_SRC_18_ASF_RMDV);

			if (pDpeConfig->Dpe_OutBuf_ASF_RD_Ext != 0x0) {
				pConfigToKernel->DVP_EXT_SRC_19_ASF_RDDV =
				pDpeConfig->Dpe_OutBuf_ASF_RD_Ext;
				LOG_INF("set Dpe_OutBuf_ASF_RD_Ext!\n");
			} else
				LOG_INF("No ASF_RD_EXT Output Buffer!\n");

			LOG_INF("DVP_EXT_SRC_19_ASF_RDDV = %llx\n",
			pConfigToKernel->DVP_EXT_SRC_19_ASF_RDDV);

			if (pDpeConfig->Dpe_OutBuf_ASF_HF_Ext != 0x0) {
				pConfigToKernel->DVP_EXT_SRC_20_ASF_DV0 =
				(dma_addr_t)pDpeConfig->Dpe_OutBuf_ASF_HF_Ext;
				LOG_INF("set Dpe_OutBuf_ASF_HF_Ext!\n");
			} else
				LOG_INF("No DVP Ext ASF Output Buffer!\n");

			LOG_INF("DVP_EXT_SRC_20_ASF_DV0 = %llx\n",
			pConfigToKernel->DVP_EXT_SRC_20_ASF_DV0);
}
	pConfigToKernel->DVP_CTRL_ATPG = 0x80000000;
	static_assert(offsetof(struct DPE_Kernel_Config, DVP_CORE_00) +
		sizeof(pDpeConfig->Dpe_DVPSettings.TuningBuf_CORE) <= sizeof(*pConfigToKernel));
	memcpy((char *)pConfigToKernel + offsetof(struct DPE_Kernel_Config, DVP_CORE_00),
	&pDpeConfig->Dpe_DVPSettings.TuningBuf_CORE,
	sizeof(pDpeConfig->Dpe_DVPSettings.TuningBuf_CORE));
}

void DPE_DumpUserSpaceReg(struct DPE_Kernel_Config *pDpeConfig)
{
	LOG_INF("=====DPE Config From User Space========\n");
	LOG_INF("DVS_CTRL00 = 0x%08X\n", pDpeConfig->DVS_CTRL00);
	LOG_INF("DVS_CTRL01 = 0x%08X\n", pDpeConfig->DVS_CTRL01);
	LOG_INF("DVS_CTRL02 = 0x%08X\n", pDpeConfig->DVS_CTRL02);
	LOG_INF("DVS_CTRL03 = 0x%08X\n", pDpeConfig->DVS_CTRL03);
	LOG_INF("DVS_CTRL06 = 0x%08X\n", pDpeConfig->DVS_CTRL06);
	LOG_INF("DVS_CTRL07 = 0x%08X\n", pDpeConfig->DVS_CTRL07);
	LOG_INF("DVS_IRQ_00 = 0x%08X\n", pDpeConfig->DVS_IRQ_00);
	LOG_INF("DVS_SRC_CTRL = 0x%08X\n", pDpeConfig->DVS_SRC_CTRL);
	LOG_INF("DVS_CRC_CTRL = 0x%08X\n", pDpeConfig->DVS_CRC_CTRL);
	LOG_INF("DVS_DRAM_ULT = 0x%08X\n", pDpeConfig->DVS_DRAM_ULT);
	LOG_INF("DVS_DRAM_PITCH = 0x%08X\n", pDpeConfig->DVS_DRAM_PITCH);
	LOG_INF("DVS_SRC_00 = 0x%08X\n", pDpeConfig->DVS_SRC_00);
	LOG_INF("DVS_SRC_01 = 0x%08X\n", pDpeConfig->DVS_SRC_01);
	LOG_INF("DVS_SRC_02 = 0x%08X\n", pDpeConfig->DVS_SRC_02);
	LOG_INF("DVS_SRC_03 = 0x%08X\n", pDpeConfig->DVS_SRC_03);
	LOG_INF("DVS_SRC_04 = 0x%08X\n", pDpeConfig->DVS_SRC_04);
	LOG_INF("DVS_SRC_05 = 0x%08llX\n", pDpeConfig->DVS_SRC_05_L_FRM0);
	LOG_INF("DVS_SRC_06 = 0x%08llX\n", pDpeConfig->DVS_SRC_06_L_FRM1);
	LOG_INF("DVS_SRC_07 = 0x%08llX\n", pDpeConfig->DVS_SRC_07_L_FRM2);
	LOG_INF("DVS_SRC_08 = 0x%08llX\n", pDpeConfig->DVS_SRC_08_L_FRM3);
	LOG_INF("DVS_SRC_09 = 0x%08llX\n", pDpeConfig->DVS_SRC_09_R_FRM0);
	LOG_INF("DVS_SRC_10 = 0x%08llX\n", pDpeConfig->DVS_SRC_10_R_FRM1);
	LOG_INF("DVS_SRC_11 = 0x%08llX\n", pDpeConfig->DVS_SRC_11_R_FRM2);
	LOG_INF("DVS_SRC_12 = 0x%08llX\n", pDpeConfig->DVS_SRC_12_R_FRM3);
	LOG_INF("DVS_SRC_13 = 0x%08llX\n", pDpeConfig->DVS_SRC_13_L_VMAP0);
	LOG_INF("DVS_SRC_14 = 0x%08llX\n", pDpeConfig->DVS_SRC_14_L_VMAP1);
	LOG_INF("DVS_SRC_15 = 0x%08llX\n", pDpeConfig->DVS_SRC_15_L_VMAP2);
	LOG_INF("DVS_SRC_16 = 0x%08llX\n", pDpeConfig->DVS_SRC_16_L_VMAP3);
	LOG_INF("DVS_SRC_17 = 0x%08llX\n", pDpeConfig->DVS_SRC_17_R_VMAP0);
	LOG_INF("DVS_SRC_18 = 0x%08llX\n", pDpeConfig->DVS_SRC_18_R_VMAP1);
	LOG_INF("DVS_SRC_19 = 0x%08llX\n", pDpeConfig->DVS_SRC_19_R_VMAP2);
	LOG_INF("DVS_SRC_20 = 0x%08llX\n", pDpeConfig->DVS_SRC_20_R_VMAP3);
	LOG_INF("DVS_SRC_21 = 0x%08llX\n", pDpeConfig->DVS_SRC_21_P4_L_DV_ADR);
	LOG_INF("DVS_SRC_26 = 0x%08llX\n", pDpeConfig->DVS_SRC_26_OCCDV0);
	LOG_INF("DVS_SRC_27 = 0x%08llX\n", pDpeConfig->DVS_SRC_27_OCCDV1);
	LOG_INF("DVS_SRC_28 = 0x%08llX\n", pDpeConfig->DVS_SRC_28_OCCDV2);
	LOG_INF("DVS_SRC_29 = 0x%08llX\n", pDpeConfig->DVS_SRC_29_OCCDV3);
	LOG_INF("DVS_SRC_34 = 0x%08llX\n", pDpeConfig->DVS_SRC_34_P4_R_DV_ADR);
	LOG_INF("DVS_SRC_42 = 0x%08llX\n", pDpeConfig->DVS_SRC_42_OCCDV_EXT0);
	LOG_INF("DVS_SRC_43 = 0x%08llX\n", pDpeConfig->DVS_SRC_43_OCCDV_EXT1);
	LOG_INF("DVS_SRC_44 = 0x%08llX\n", pDpeConfig->DVS_SRC_44_OCCDV_EXT2);
	LOG_INF("DVS_SRC_45 = 0x%08llX\n", pDpeConfig->DVS_SRC_45_OCCDV_EXT3);
	LOG_INF("DVS_SRC_46 = 0x%08X\n", pDpeConfig->DVS_SRC_46);
	LOG_INF("DVS_DRAM_SEC = 0x%08X\n", pDpeConfig->DVS_DRAM_SEC);
	LOG_INF("DVS_CTRL_ATPG = 0x%08X\n", pDpeConfig->DVS_CTRL_ATPG);
	LOG_INF("=====DPE Config end========\n");

	LOG_INF("DVS_PD_SRC_00 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_00_L_FRM0);
	LOG_INF("DVS_PD_SRC_01 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_01_L_FRM1);
	LOG_INF("DVS_PD_SRC_02 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_02_L_FRM2);
	LOG_INF("DVS_PD_SRC_03 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_03_L_FRM3);
	LOG_INF("DVS_PD_SRC_04 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_04_R_FRM0);
	LOG_INF("DVS_PD_SRC_05 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_05_R_FRM1);
	LOG_INF("DVS_PD_SRC_06 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_06_R_FRM2);
	LOG_INF("DVS_PD_SRC_07 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_07_R_FRM3);
	LOG_INF("DVS_PD_SRC_08 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_08_OCCDV0);
	LOG_INF("DVS_PD_SRC_09 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_09_OCCDV1);
	LOG_INF("DVS_PD_SRC_10 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_10_OCCDV2);
	LOG_INF("DVS_PD_SRC_11 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_11_OCCDV3);
	LOG_INF("DVS_PD_SRC_12 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_12_OCCDV_EXT0);
	LOG_INF("DVS_PD_SRC_13 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_13_OCCDV_EXT1);
	LOG_INF("DVS_PD_SRC_14 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_14_OCCDV_EXT2);
	LOG_INF("DVS_PD_SRC_15 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_15_OCCDV_EXT3);
	LOG_INF("DVS_PD_SRC_16 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_16_DCV_CONF0);
	LOG_INF("DVS_PD_SRC_17 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_17_DCV_CONF1);
	LOG_INF("DVS_PD_SRC_18 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_18_DCV_CONF2);
	LOG_INF("DVS_PD_SRC_19 = 0x%08X\n",
	pDpeConfig->DVS_PD_SRC_19_DCV_CONF3);
	LOG_INF("DVS_ME_00 = 0x%08X\n", pDpeConfig->DVS_ME_00);
	LOG_INF("DVS_ME_01 = 0x%08X\n", pDpeConfig->DVS_ME_01);
	LOG_INF("DVS_ME_02 = 0x%08X\n", pDpeConfig->DVS_ME_02);
	LOG_INF("DVS_ME_03 = 0x%08X\n", pDpeConfig->DVS_ME_03);
	LOG_INF("DVS_ME_04 = 0x%08X\n", pDpeConfig->DVS_ME_04);
	LOG_INF("DVS_ME_06 = 0x%08X\n", pDpeConfig->DVS_ME_06);
	LOG_INF("DVS_ME_07 = 0x%08X\n", pDpeConfig->DVS_ME_07);
	LOG_INF("DVS_ME_08 = 0x%08X\n", pDpeConfig->DVS_ME_08);
	LOG_INF("DVS_ME_09 = 0x%08X\n", pDpeConfig->DVS_ME_09);
	LOG_INF("DVS_ME_10 = 0x%08X\n", pDpeConfig->DVS_ME_10);
	LOG_INF("DVS_ME_11 = 0x%08X\n", pDpeConfig->DVS_ME_11);
	LOG_INF("DVS_ME_12 = 0x%08X\n", pDpeConfig->DVS_ME_12);
	LOG_INF("DVS_ME_13 = 0x%08X\n", pDpeConfig->DVS_ME_13);
	LOG_INF("DVS_ME_14 = 0x%08X\n", pDpeConfig->DVS_ME_14);
	LOG_INF("DVS_ME_15 = 0x%08X\n", pDpeConfig->DVS_ME_15);
	LOG_INF("DVS_ME_16 = 0x%08X\n", pDpeConfig->DVS_ME_16);
	LOG_INF("DVS_ME_17 = 0x%08X\n", pDpeConfig->DVS_ME_17);
	LOG_INF("DVS_ME_18 = 0x%08X\n", pDpeConfig->DVS_ME_18);
	LOG_INF("DVS_ME_19 = 0x%08X\n", pDpeConfig->DVS_ME_19);
	LOG_INF("DVS_ME_20 = 0x%08X\n", pDpeConfig->DVS_ME_20);
	LOG_INF("DVS_ME_21 = 0x%08X\n", pDpeConfig->DVS_ME_21);
	LOG_INF("DVS_ME_22 = 0x%08X\n", pDpeConfig->DVS_ME_22);
	LOG_INF("DVS_ME_23 = 0x%08X\n", pDpeConfig->DVS_ME_23);
	LOG_INF("DVS_ME_24 = 0x%08X\n", pDpeConfig->DVS_ME_24);
	LOG_INF("DVS_ME_25 = 0x%08X\n", pDpeConfig->DVS_ME_25);
	LOG_INF("DVS_ME_26 = 0x%08X\n", pDpeConfig->DVS_ME_26);
	LOG_INF("DVS_ME_27 = 0x%08X\n", pDpeConfig->DVS_ME_27);
	LOG_INF("DVS_ME_28 = 0x%08X\n", pDpeConfig->DVS_ME_28);
	LOG_INF("DVS_ME_29 = 0x%08X\n", pDpeConfig->DVS_ME_29);
	LOG_INF("DVS_ME_30 = 0x%08X\n", pDpeConfig->DVS_ME_30);
	LOG_INF("DVS_ME_31 = 0x%08X\n", pDpeConfig->DVS_ME_31);
	LOG_INF("DVS_ME_32 = 0x%08X\n", pDpeConfig->DVS_ME_32);
	LOG_INF("DVS_ME_33 = 0x%08X\n", pDpeConfig->DVS_ME_33);
	LOG_INF("DVS_ME_34 = 0x%08X\n", pDpeConfig->DVS_ME_34);
	LOG_INF("DVS_ME_35 = 0x%08X\n", pDpeConfig->DVS_ME_35);
	LOG_INF("DVS_ME_36 = 0x%08X\n", pDpeConfig->DVS_ME_36);
	LOG_INF("DVS_ME_37 = 0x%08X\n", pDpeConfig->DVS_ME_37);
	LOG_INF("DVS_ME_38 = 0x%08X\n", pDpeConfig->DVS_ME_38);
	LOG_INF("DVS_ME_39 = 0x%08X\n", pDpeConfig->DVS_ME_39);
	LOG_INF("DVS_ME_37 = 0x%08X\n", pDpeConfig->DVS_ME_37);
	LOG_INF("DVS_ME_38 = 0x%08X\n", pDpeConfig->DVS_ME_38);
	LOG_INF("DVS_ME_39 = 0x%08X\n", pDpeConfig->DVS_ME_39);

	LOG_INF("DVS_DEBUG = 0x%08X\n", pDpeConfig->DVS_DEBUG);
	LOG_INF("DVS_ME_RESERVED = 0x%08X\n", pDpeConfig->DVS_ME_RESERVED);
	LOG_INF("DVS_ME_ATPG = 0x%08X\n", pDpeConfig->DVS_ME_ATPG);
	LOG_INF("DVS_ME_40 = 0x%08X\n", pDpeConfig->DVS_ME_40);
	LOG_INF("DVS_ME_41 = 0x%08X\n", pDpeConfig->DVS_ME_41);

	LOG_INF("DVS_OCC_PQ_0 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_0);
	LOG_INF("DVS_OCC_PQ_1 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_1);
	LOG_INF("DVS_OCC_PQ_2 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_2);
	LOG_INF("DVS_OCC_PQ_3 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_3);
	LOG_INF("DVS_OCC_PQ_4 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_4);
	LOG_INF("DVS_OCC_PQ_5 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_5);
	LOG_INF("DVS_OCC_PQ_6 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_6);
	LOG_INF("DVS_OCC_PQ_7 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_7);
	LOG_INF("DVS_OCC_PQ_8 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_8);
	LOG_INF("DVS_OCC_PQ_9 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_9);
	LOG_INF("DVS_OCC_PQ_10 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_10);
	LOG_INF("DVS_OCC_PQ_11 = 0x%08X\n", pDpeConfig->DVS_OCC_PQ_11);
	LOG_INF("DVS_OCC_ATPG = 0x%08X\n", pDpeConfig->DVS_OCC_ATPG);

	LOG_INF("DVP_CTRL00 = 0x%08X\n", pDpeConfig->DVP_CTRL00);
	LOG_INF("DVP_CTRL01 = 0x%08X\n", pDpeConfig->DVP_CTRL01);
	LOG_INF("DVP_CTRL02 = 0x%08X\n", pDpeConfig->DVP_CTRL02);
	LOG_INF("DVP_CTRL03 = 0x%08X\n", pDpeConfig->DVP_CTRL03);
	LOG_INF("DVP_CTRL04 = 0x%08X\n", pDpeConfig->DVP_CTRL04);
	LOG_INF("DVP_CTRL07 = 0x%08X\n", pDpeConfig->DVP_CTRL07);
	LOG_INF("DVP_IRQ_00 = 0x%08X\n", pDpeConfig->DVP_IRQ_00);
	LOG_INF("DVP_IRQ_01 = 0x%08X\n", pDpeConfig->DVP_IRQ_01);
	LOG_INF("DVP_SRC_00 = 0x%08X\n", pDpeConfig->DVP_SRC_00);
	LOG_INF("DVP_SRC_01 = 0x%08X\n", pDpeConfig->DVP_SRC_01);
	LOG_INF("DVP_SRC_02 = 0x%08X\n", pDpeConfig->DVP_SRC_02);
	LOG_INF("DVP_SRC_03 = 0x%08X\n", pDpeConfig->DVP_SRC_03);
	LOG_INF("DVP_SRC_04 = 0x%08X\n", pDpeConfig->DVP_SRC_04);
	LOG_INF("DVP_SRC_05 = 0x%08llX\n", pDpeConfig->DVP_SRC_05_Y_FRM0);
	LOG_INF("DVP_SRC_06 = 0x%08llX\n", pDpeConfig->DVP_SRC_06_Y_FRM1);
	LOG_INF("DVP_SRC_07 = 0x%08llX\n", pDpeConfig->DVP_SRC_07_Y_FRM2);
	LOG_INF("DVP_SRC_08 = 0x%08llX\n", pDpeConfig->DVP_SRC_08_Y_FRM3);
	LOG_INF("DVP_SRC_09 = 0x%08llX\n", pDpeConfig->DVP_SRC_09_C_FRM0);
	LOG_INF("DVP_SRC_10 = 0x%08llX\n", pDpeConfig->DVP_SRC_10_C_FRM1);
	LOG_INF("DVP_SRC_11 = 0x%08llX\n", pDpeConfig->DVP_SRC_11_C_FRM2);
	LOG_INF("DVP_SRC_12 = 0x%08llX\n", pDpeConfig->DVP_SRC_12_C_FRM3);
	LOG_INF("DVP_SRC_13 = 0x%08llX\n", pDpeConfig->DVP_SRC_13_OCCDV0);
	LOG_INF("DVP_SRC_14 = 0x%08llX\n", pDpeConfig->DVP_SRC_14_OCCDV1);
	LOG_INF("DVP_SRC_15 = 0x%08llX\n", pDpeConfig->DVP_SRC_15_OCCDV2);
	LOG_INF("DVP_SRC_16 = 0x%08llX\n", pDpeConfig->DVP_SRC_16_OCCDV3);
	LOG_INF("DVP_SRC_17 = 0x%08llX\n", pDpeConfig->DVP_SRC_17_CRM);
	LOG_INF("DVP_SRC_18 = 0x%08llX\n", pDpeConfig->DVP_SRC_18_ASF_RMDV);
	LOG_INF("DVP_SRC_19 = 0x%08llX\n", pDpeConfig->DVP_SRC_19_ASF_RDDV);
	LOG_INF("DVP_SRC_20 = 0x%08llX\n", pDpeConfig->DVP_SRC_20_ASF_DV0);
	LOG_INF("DVP_SRC_21 = 0x%08llX\n", pDpeConfig->DVP_SRC_21_ASF_DV1);
	LOG_INF("DVP_SRC_22 = 0x%08llX\n", pDpeConfig->DVP_SRC_22_ASF_DV2);
	LOG_INF("DVP_SRC_23 = 0x%08llX\n", pDpeConfig->DVP_SRC_23_ASF_DV3);
	LOG_INF("DVP_SRC_24 = 0x%08llX\n", pDpeConfig->DVP_SRC_24_WMF_HFDV);
	LOG_INF("DVP_SRC_25 = 0x%08llX\n", pDpeConfig->DVP_SRC_25_WMF_DV0);
	LOG_INF("DVP_SRC_26 = 0x%08llX\n", pDpeConfig->DVP_SRC_26_WMF_DV1);
	LOG_INF("DVP_SRC_27 = 0x%08llX\n", pDpeConfig->DVP_SRC_27_WMF_DV2);
	LOG_INF("DVP_SRC_28 = 0x%08llX\n", pDpeConfig->DVP_SRC_28_WMF_DV3);
	LOG_INF("DVP_CORE_00 = 0x%08X\n", pDpeConfig->DVP_CORE_00);
	LOG_INF("DVP_CORE_01 = 0x%08X\n", pDpeConfig->DVP_CORE_01);
	LOG_INF("DVP_CORE_02 = 0x%08X\n", pDpeConfig->DVP_CORE_02);
	LOG_INF("DVP_CORE_03 = 0x%08X\n", pDpeConfig->DVP_CORE_03);
	LOG_INF("DVP_CORE_04 = 0x%08X\n", pDpeConfig->DVP_CORE_04);
	LOG_INF("DVP_CORE_05 = 0x%08X\n", pDpeConfig->DVP_CORE_05);
	LOG_INF("DVP_CORE_06 = 0x%08X\n", pDpeConfig->DVP_CORE_06);
	LOG_INF("DVP_CORE_07 = 0x%08X\n", pDpeConfig->DVP_CORE_07);
	LOG_INF("DVP_CORE_08 = 0x%08X\n", pDpeConfig->DVP_CORE_08);
	LOG_INF("DVP_CORE_09 = 0x%08X\n", pDpeConfig->DVP_CORE_09);
	LOG_INF("DVP_CORE_10 = 0x%08X\n", pDpeConfig->DVP_CORE_10);
	LOG_INF("DVP_CORE_11 = 0x%08X\n", pDpeConfig->DVP_CORE_11);
	LOG_INF("DVP_CORE_12 = 0x%08X\n", pDpeConfig->DVP_CORE_12);
	LOG_INF("DVP_CORE_13 = 0x%08X\n", pDpeConfig->DVP_CORE_13);
	LOG_INF("DVP_CORE_14 = 0x%08X\n", pDpeConfig->DVP_CORE_14);
	LOG_INF("DVP_CORE_15 = 0x%08X\n", pDpeConfig->DVP_CORE_15);
	LOG_INF("DVP_CORE_16 = 0x%08X\n", pDpeConfig->DVP_CORE_16);
	LOG_INF("DVP_SRC_CTRL = 0x%08X\n", pDpeConfig->DVP_SRC_CTRL);
	LOG_INF("DVP_CRC_CTRL = 0x%08X\n", pDpeConfig->DVP_CRC_CTRL);
	LOG_INF("DVP_DRAM_ULT = 0x%08X\n", pDpeConfig->DVP_DRAM_ULT);
	LOG_INF("DVP_DRAM_PITCH = 0x%08X\n", pDpeConfig->DVP_DRAM_PITCH);
	LOG_INF("DVP_EXT_SRC_13_OCCDV0 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_13_OCCDV0);
	LOG_INF("DVP_EXT_SRC_14_OCCDV1 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_14_OCCDV1);
	LOG_INF("DVP_EXT_SRC_15_OCCDV2 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_15_OCCDV2);
	LOG_INF("DVP_EXT_SRC_16_OCCDV3 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_16_OCCDV3);
	LOG_INF("DVP_EXT_SRC_18_ASF_RMDV = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_18_ASF_RMDV);
	LOG_INF("DVP_EXT_SRC_19_ASF_RDDV = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_19_ASF_RDDV);
	LOG_INF("DVP_EXT_SRC_20_ASF_DV0 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_20_ASF_DV0);
	LOG_INF("DVP_EXT_SRC_21_ASF_DV1 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_21_ASF_DV1);
	LOG_INF("DVP_EXT_SRC_22_ASF_DV2 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_22_ASF_DV2);
	LOG_INF("DVP_EXT_SRC_23_ASF_DV3 = 0x%08llX\n",
		pDpeConfig->DVP_EXT_SRC_23_ASF_DV3);
	LOG_INF("DVP_CORE_00 = 0x%08X\n", pDpeConfig->DVP_CORE_00);
	LOG_INF("DVP_CORE_01 = 0x%08X\n", pDpeConfig->DVP_CORE_01);
	LOG_INF("DVP_CORE_02 = 0x%08X\n", pDpeConfig->DVP_CORE_02);
	LOG_INF("DVP_CORE_03 = 0x%08X\n", pDpeConfig->DVP_CORE_03);
	LOG_INF("DVP_CORE_04 = 0x%08X\n", pDpeConfig->DVP_CORE_04);
	LOG_INF("DVP_CORE_05 = 0x%08X\n", pDpeConfig->DVP_CORE_05);
	LOG_INF("DVP_CORE_06 = 0x%08X\n", pDpeConfig->DVP_CORE_06);
	LOG_INF("DVP_CORE_07 = 0x%08X\n", pDpeConfig->DVP_CORE_07);
	LOG_INF("DVP_CORE_08 = 0x%08X\n", pDpeConfig->DVP_CORE_08);
	LOG_INF("DVP_CORE_09 = 0x%08X\n", pDpeConfig->DVP_CORE_09);
	LOG_INF("DVP_CORE_10 = 0x%08X\n", pDpeConfig->DVP_CORE_10);
	LOG_INF("DVP_CORE_11 = 0x%08X\n", pDpeConfig->DVP_CORE_11);
	LOG_INF("DVP_CORE_12 = 0x%08X\n", pDpeConfig->DVP_CORE_12);
	LOG_INF("DVP_CORE_13 = 0x%08X\n", pDpeConfig->DVP_CORE_13);
	LOG_INF("DVP_CORE_14 = 0x%08X\n", pDpeConfig->DVP_CORE_14);
	LOG_INF("DVP_CORE_15 = 0x%08X\n", pDpeConfig->DVP_CORE_15);
}
static DEFINE_MUTEX(dpe_users_lock);
static bool dpe_closing = true;
static bool dpe_faulted;
static atomic_t dpe_pending_packets = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(dpe_packet_wait);
static void dpe_packet_done(void)
{
	if (atomic_dec_and_test(&dpe_pending_packets)) wake_up_all(&dpe_packet_wait);
}
static void dpe_record_fault(void)
{
	WRITE_ONCE(dpe_faulted, true);
	wake_up_interruptible(&DPEInfo.WaitQueueHead);
}
void cmdq_cb_destroy(struct cmdq_cb_data data)
{
	if (data.err && !READ_ONCE(dpe_closing)) dpe_record_fault();
	cmdq_pkt_destroy((struct cmdq_pkt *)data.data);
	dpe_packet_done();
}
struct sg_table *dpe_dma_buf_map_attachment_unlocked(struct dma_buf_attachment *attach,
					enum dma_data_direction direction)
{
	struct sg_table *sg_table;

	might_sleep();
	if (WARN_ON(!attach || !attach->dmabuf))
		return ERR_PTR(-EINVAL);
	sg_table = dma_buf_map_attachment_unlocked(attach, direction);
	if (!sg_table)
		sg_table = ERR_PTR(-ENOMEM);
	return sg_table;
}

signed int CmdqDPEHW(struct frame *frame)
{
	struct DPE_Kernel_Config *pDpeConfig;
	struct DPE_Kernel_Config DpeConfig = {0};
	int ret;
	struct DPE_Config *pDpeUserConfig;

	#ifdef CMdq_en
	struct cmdq_pkt *handle;
	#endif
	//struct cmdqRecStruct *handle;
	//uint64_t engineFlag = (uint64_t)(1LL << CMDQ_ENG_DPE);
#if defined(DPE_PMQOS_EN) && defined(CONFIG_MTK_QOS_SUPPORT)
	unsigned int w_imgi, h_imgi, w_mvio, h_mvio, w_bvo, h_bvo;
	unsigned int dma_bandwidth, trig_num;
#endif

	if (wire_test_only) return -EOPNOTSUPP;
	if (READ_ONCE(dpe_closing) || READ_ONCE(dpe_faulted)) return -ESHUTDOWN;
	if (frame == NULL || frame->data == NULL)
		return -1;

	LOG_DBG("%s request sent to CMDQ driver", __func__);
	pDpeUserConfig = (struct DPE_Config *) frame->data;
	pDpeConfig = &DpeConfig;
/************** Pass User info to DPE_Kernel_Config **************/

	if (pDpeUserConfig->Dpe_engineSelect == MODE_DVS_DVP_BOTH) {
		DPE_Config_DVS(pDpeUserConfig, pDpeConfig);
		DPE_Config_DVP(pDpeUserConfig, pDpeConfig);
		pDpeConfig->DPE_MODE = 0;
	} else if (pDpeUserConfig->Dpe_engineSelect == MODE_DVS_ONLY) {
		DPE_Config_DVS(pDpeUserConfig, pDpeConfig);
		pDpeConfig->DPE_MODE = 1;
	} else if (pDpeUserConfig->Dpe_engineSelect == MODE_DVP_ONLY) {
		DPE_Config_DVP(pDpeUserConfig, pDpeConfig);
		pDpeConfig->DPE_MODE = 2;
	}

	if (g_isDPELogEnable)
		DPE_DumpUserSpaceReg(pDpeConfig);
	//cmdqRecCreate(CMDQ_SCENARIO_ISP_DPE, &handle);
	//cmdqRecSetEngine(handle, engineFlag);
	//cmdq_pkt_cl_create(&handle, dpe_clt);
#ifdef CMdq_en
	handle = cmdq_pkt_create(dpe_clt);
	if (IS_ERR_OR_NULL(handle)) { dpe_record_fault(); return handle ? PTR_ERR(handle) : -ENOMEM; }
#endif
#define CMDQWR(REG) \
	cmdq_pkt_write(handle, dpe_clt_base, \
	REG ##_HW, pDpeConfig->REG, CMDQ_REG_MASK)
#define CMDQWR_DPE_DRAM_ADDR(REG) \
	cmdq_pkt_write(handle, dpe_clt_base, \
	REG ##_HW, (pDpeConfig->REG)>>4, CMDQ_REG_MASK)
//LOG_INF("pDpeConfig->DPE_MODE = %x\n", pDpeConfig->DPE_MODE);
if (pDpeConfig->DPE_MODE != 2) {
	if (pDpeConfig->DPE_MODE == 1) {
		/* DVS Only Mode*/
		/* dvp_en = 1 */
		#ifdef CMdq_en
		cmdq_pkt_write(handle, dpe_clt_base,
		DVP_CTRL00_HW, 0x80000000, 0x80000000);
		#endif
	}
	/* mask trigger bit */
	#ifdef CMdq_en
	cmdq_pkt_write(handle, dpe_clt_base,
	DVS_CTRL00_HW, pDpeConfig->DVS_CTRL00, 0xDBF5FC00);
	cmdq_pkt_write(handle, dpe_clt_base,
	DVS_CTRL02_HW, 0x70310001, CMDQ_REG_MASK);
	#endif


	//LOG_INF("mask trigger bit\n");
	/* cmdq_pkt_write(handle, dpe_clt_base, */
	/* DVS_CTRL07_HW, 0x0000FF1F, CMDQ_REG_MASK); */
	/* cmdq_pkt_write(handle, dpe_clt_base, */
	/* DVS_SRC_CTRL_HW, 0x00000040, CMDQ_REG_MASK); */
	/* DVS Frame Done IRQ */
	if (pDpeConfig->DPE_MODE == 1) { // MODE_DVS_ONLY
		//LOG_INF("MODE_DVS_ONLY\n");
		#ifdef CMdq_en
		cmdq_pkt_write(handle, dpe_clt_base,
		DVS_IRQ_00_HW, 0x00000E00, 0x00000F00);
		#endif
	} else { // MODE_DVS_DVP_BOTH
		#ifdef CMdq_en		 // MODE_DVS_DVP_BOTH
		cmdq_pkt_write(handle, dpe_clt_base,
		DVS_IRQ_00_HW, 0x00000F00, 0x00000F00);
		#endif
	}
	//LOG_INF("star CMDQWR\n");
	#ifdef CMdq_en
	CMDQWR(DVS_DRAM_PITCH);
	CMDQWR(DVS_SRC_00);
	CMDQWR(DVS_SRC_01);
	CMDQWR(DVS_SRC_02);
	CMDQWR(DVS_SRC_03);
	CMDQWR(DVS_SRC_04);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_05_L_FRM0);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_06_L_FRM1);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_07_L_FRM2);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_08_L_FRM3);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_09_R_FRM0);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_10_R_FRM1);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_11_R_FRM2);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_12_R_FRM3);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_13_L_VMAP0);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_14_L_VMAP1);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_15_L_VMAP2);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_16_L_VMAP3);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_17_R_VMAP0);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_18_R_VMAP1);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_19_R_VMAP2);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_20_R_VMAP3);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_21_P4_L_DV_ADR);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_26_OCCDV0);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_27_OCCDV1);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_28_OCCDV2);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_29_OCCDV3);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_34_P4_R_DV_ADR);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_42_OCCDV_EXT0);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_43_OCCDV_EXT1);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_44_OCCDV_EXT2);
	CMDQWR_DPE_DRAM_ADDR(DVS_SRC_45_OCCDV_EXT3);
	CMDQWR(DVS_SRC_46);
	CMDQWR(DVS_DRAM_SEC);
	CMDQWR(DVS_CTRL_ATPG);

	if (pDpeUserConfig->Dpe_DVSSettings.is_pd_mode) {
		//CMDQWR_DPE_DRAM_ADDR(DVS_PD_SRC_04_R_FRM0);
		//CMDQWR_DPE_DRAM_ADDR(DVS_PD_SRC_08_OCCDV0);
		//CMDQWR_DPE_DRAM_ADDR(DVS_PD_SRC_16_DCV_CONF0);
		//if (pDpeUserConfig->DVP_ASF_CONF_EN != 0) //!ISP7
		//CMDQWR_DPE_DRAM_ADDR(DVS_PD_SRC_12_OCCDV_EXT0);
	}
	CMDQWR(DVS_ME_00);
	CMDQWR(DVS_ME_01);
	CMDQWR(DVS_ME_02);
	CMDQWR(DVS_ME_03);
	CMDQWR(DVS_ME_04);
	CMDQWR(DVS_ME_06);
	CMDQWR(DVS_ME_07);
	CMDQWR(DVS_ME_08);
	CMDQWR(DVS_ME_09);
	CMDQWR(DVS_ME_10);
	CMDQWR(DVS_ME_11);
	CMDQWR(DVS_ME_12);
	CMDQWR(DVS_ME_13);
	CMDQWR(DVS_ME_14);
	CMDQWR(DVS_ME_15);
	CMDQWR(DVS_ME_16);
	CMDQWR(DVS_ME_17);
	CMDQWR(DVS_ME_18);
	CMDQWR(DVS_ME_19);
	CMDQWR(DVS_ME_20);
	CMDQWR(DVS_ME_21);
	CMDQWR(DVS_ME_22);
	CMDQWR(DVS_ME_23);
	CMDQWR(DVS_ME_24);
	CMDQWR(DVS_ME_25);
	CMDQWR(DVS_ME_26);
	CMDQWR(DVS_ME_27);
	CMDQWR(DVS_ME_28);
	CMDQWR(DVS_ME_29);
	CMDQWR(DVS_ME_30);
	CMDQWR(DVS_ME_31);
	CMDQWR(DVS_ME_32);
	CMDQWR(DVS_ME_33);
	CMDQWR(DVS_ME_34);
	CMDQWR(DVS_ME_35);
	CMDQWR(DVS_ME_36);
	CMDQWR(DVS_ME_37);
	CMDQWR(DVS_ME_38);
	CMDQWR(DVS_ME_39);
	CMDQWR(DVS_DEBUG);
	CMDQWR(DVS_ME_RESERVED);
	CMDQWR(DVS_ME_ATPG);
	CMDQWR(DVS_ME_40);
	CMDQWR(DVS_ME_41);
	CMDQWR(DVS_OCC_PQ_0);
	CMDQWR(DVS_OCC_PQ_1);
	CMDQWR(DVS_OCC_PQ_2);
	CMDQWR(DVS_OCC_PQ_3);
	CMDQWR(DVS_OCC_PQ_4);
	CMDQWR(DVS_OCC_PQ_5);
	CMDQWR(DVS_OCC_PQ_6);
	CMDQWR(DVS_OCC_PQ_7);
	CMDQWR(DVS_OCC_PQ_8);
	CMDQWR(DVS_OCC_PQ_9);
	CMDQWR(DVS_OCC_PQ_10);
	CMDQWR(DVS_OCC_PQ_11);
	CMDQWR(DVS_OCC_ATPG);
	#endif
/* CRC EN, CRC SEL =0x0 */
/* cmdq_pkt_write(handle, dpe_clt_base, */
/* DVS_CRC_CTRL_HW, 0x00000001, 0x00000F01); */
/* CRC CLEAR  = 1 */
/* cmdq_pkt_write(handle, dpe_clt_base, */
/* DVS_CRC_CTRL_HW, 0x00000010, 0x00000010); */
/* CRC CLEAR  = 0 */
/* cmdq_pkt_write(handle, dpe_clt_base, */
/* DVS_CRC_CTRL_HW, 0x00000000, 0x00000010); */
}
	/*================= DVP Settings ==================*/
if (pDpeConfig->DPE_MODE != 1) {
	if (pDpeConfig->DPE_MODE == 2) {
		/* DVP Only Mode*/
		/* dvs_en = 1, DPE FW Tri En = 1, */
		/* dvp_trig_en = 1, dvs_trig_en = 0  */
		#ifdef CMdq_en
		cmdq_pkt_write(handle, dpe_clt_base,
		DVS_CTRL00_HW, 0xC8000000, 0xD8300000);
		cmdq_pkt_write(handle, dpe_clt_base,
		DVS_CTRL02_HW, 0x70310001, CMDQ_REG_MASK);
		#endif
	}
	#ifdef CMdq_en
	cmdq_pkt_write(handle, dpe_clt_base,
	DVP_CTRL00_HW, 0x80000080, CMDQ_REG_MASK);
	cmdq_pkt_write(handle, dpe_clt_base,
	DVP_CTRL02_HW, 0x70310001, CMDQ_REG_MASK);
	/* cmdq_pkt_write(handle, dpe_clt_base, */
	/* DVP_CTRL07_HW, 0x00000707, CMDQ_REG_MASK); */
	/* DVP Frame Done IRQ */
	cmdq_pkt_write(handle, dpe_clt_base,
	DVP_IRQ_00_HW, 0x00000E00, 0x00000F00);
	//LOG_INF("star CMDQWR DVP Settings\n");


	CMDQWR(DVP_DRAM_PITCH);
	CMDQWR(DVP_SRC_00);
	CMDQWR(DVP_SRC_01);
	CMDQWR(DVP_SRC_02);
	CMDQWR(DVP_SRC_03);
	CMDQWR(DVP_SRC_04);
	CMDQWR(DVP_CTRL04);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_05_Y_FRM0);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_06_Y_FRM1);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_07_Y_FRM2);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_08_Y_FRM3);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_09_C_FRM0);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_10_C_FRM1);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_11_C_FRM2);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_12_C_FRM3);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_13_OCCDV0);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_14_OCCDV1);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_15_OCCDV2);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_16_OCCDV3);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_17_CRM);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_18_ASF_RMDV);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_19_ASF_RDDV);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_20_ASF_DV0);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_21_ASF_DV1);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_22_ASF_DV2);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_23_ASF_DV3);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_24_WMF_HFDV);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_25_WMF_DV0);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_26_WMF_DV1);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_27_WMF_DV2);
	CMDQWR_DPE_DRAM_ADDR(DVP_SRC_28_WMF_DV3);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_13_OCCDV0);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_14_OCCDV1);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_15_OCCDV2);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_16_OCCDV3);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_18_ASF_RMDV);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_19_ASF_RDDV);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_20_ASF_DV0);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_21_ASF_DV1);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_22_ASF_DV2);
	CMDQWR_DPE_DRAM_ADDR(DVP_EXT_SRC_23_ASF_DV3);
	CMDQWR(DVP_CORE_00);
	CMDQWR(DVP_CORE_01);
	CMDQWR(DVP_CORE_02);
	CMDQWR(DVP_CORE_03);
	CMDQWR(DVP_CORE_04);
	CMDQWR(DVP_CORE_05);
	CMDQWR(DVP_CORE_06);
	CMDQWR(DVP_CORE_07);
	CMDQWR(DVP_CORE_08);
	CMDQWR(DVP_CORE_09);
	CMDQWR(DVP_CORE_10);
	CMDQWR(DVP_CORE_11);
	CMDQWR(DVP_CORE_12);
	CMDQWR(DVP_CORE_13);
	CMDQWR(DVP_CORE_14);
	CMDQWR(DVP_CORE_15);
	CMDQWR(DVP_CORE_16);
	#endif
/*CRC EN, CRC SEL = 0x1000*/
/* cmdq_pkt_write(handle, dpe_clt_base, */
/* DVP_CRC_CTRL_HW, 0x00000801, 0x00000F01); */
/* CRC CLEAR  = 1 */
/* cmdq_pkt_write(handle, dpe_clt_base, */
/* DVP_CRC_CTRL_HW, 0x00000010, 0x00000010); */
/* CRC CLEAR  = 0 */
/* cmdq_pkt_write(handle, dpe_clt_base, */
/* DVP_CRC_CTRL_HW, 0x00000000, 0x00000010); */
}

/* DPE FW Tri = 1*/
#ifdef CMdq_en
cmdq_pkt_write(handle, dpe_clt_base, DVS_CTRL00_HW, 0x20000000, 0x20000000);
//LOG_INF("DPE FW Tri = %x\n", pDpeConfig->DVS_CTRL00);
	if (pDpeConfig->DPE_MODE == 1) /* DVS ONLY MODE */
		cmdq_pkt_wfe(handle, dvs_event_id);
	else
		cmdq_pkt_wfe(handle, dvp_event_id);
cmdq_pkt_write(handle, dpe_clt_base, DVS_CTRL00_HW, 0x00000000, 0x20000000);
#endif
#if defined(DPE_PMQOS_EN) && defined(CONFIG_MTK_QOS_SUPPORT)
	//LOG_INF("DPE_PMQOS_EN =1\n");
	trig_num = (pDpeConfig->DPE_CTRL & 0x00000F00) >> 8;
	w_imgi = pDpeConfig->DPE_SIZE & 0x000001FF;
	h_imgi = (pDpeConfig->DPE_SIZE & 0x01FF0000) >> 16;
	w_mvio = ((w_imgi + 1) >> 1) - 1;
	w_mvio = ((w_mvio / 7) << 4) + (((((w_mvio % 7) + 1) * 18) + 7) >> 3);
	h_mvio = (h_imgi + 1) >> 1;
	w_bvo =  (w_imgi + 1) >> 1;
	h_bvo =  (h_imgi + 1) >> 1;
	dma_bandwidth = ((w_imgi * h_imgi) * 2 + (w_mvio * h_mvio) * 2 * 16 +
			(w_bvo * h_bvo)) * trig_num * 30 / 1000000;
	cmdq_task_update_property(handle, &dma_bandwidth, sizeof(unsigned int));
#endif
	/* non-blocking API, Please  use cmdqRecFlushAsync() */
	//cmdq_task_flush_async_destroy(handle);
	/* flush and destroy in cmdq */
//LOG_INF("cmd_pkt start\n");
#ifdef CMdq_en
	atomic_inc(&dpe_pending_packets);
	ret = cmdq_pkt_flush_threaded(handle, cmdq_cb_destroy, handle);
	/* This deployed EXT provider queues the threaded callback only for a
	 * nonnegative submit result. Failed submission retains packet ownership. */
	if (ret < 0) {
		dpe_record_fault();
		cmdq_pkt_destroy(handle);
		dpe_packet_done();
		return ret;
	}
#endif
//LOG_INF("cmd_pkt end\n");
	return 0;
}
signed int dpe_feedback(struct frame *frame)
{
	struct DPE_Config *pDpeConfig;

	pDpeConfig = (struct DPE_Config *) frame->data;
	/* TODO: read statistics and write to the frame data */
	// pDpeConfig->DVS_IRQ_STATUS = DPE_RD32(DVS_IRQ_STATUS_REG);
	return 0;
}
static const struct engine_ops dpe_ops = {
	.req_enque_cb = dpe_enque_cb,
	.req_deque_cb = dpe_deque_cb,
	.req_deque_wire_cb = dpe_deque_wire_cb,
	.frame_handler = CmdqDPEHW,
	.req_feedback_cb = dpe_feedback,
	.req_release_cb = dpe_release_request,
};
#if defined(DPE_PMQOS_EN) && defined(CONFIG_MTK_QOS_SUPPORT)
void cmdq_pm_qos_start(struct TaskStruct *task, struct TaskStruct *task_list[],
								u32 size)
{
	unsigned int dma_bandwidth;

	dma_bandwidth = *(unsigned int *) task->prop_addr;
	pm_qos_update_request(&dpe_pm_qos_request, dma_bandwidth);
	LOG_INF("+ PMQOS Bandwidth : %d MB/sec\n", dma_bandwidth);
}
void cmdq_pm_qos_stop(struct TaskStruct *task, struct TaskStruct *task_list[],
								u32 size)
{
	pm_qos_update_request(&dpe_pm_qos_request, 0);
	LOG_DBG("- PMQOS Bandwidth : %d\n", 0);
}
#endif
unsigned int Compute_Para(struct DPE_Config *pDpeConfig,
	unsigned int tile_occ_width)
{
	unsigned int w_width; //!full_tile_width
	unsigned int tile_num = MAX_NUM_TILE;
	unsigned int egn_st_x = pDpeConfig->Dpe_DVSSettings.l_eng_start_x;

	w_width = (tile_num*tile_occ_width)+(2*egn_st_x);
	while (w_width > pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch) {
		tile_num = tile_num - 1;
		w_width = (tile_num*tile_occ_width)+(2*egn_st_x);
	}
	if (tile_num > 0) {
		pDpeConfig->Dpe_DVSSettings.pd_frame_num = tile_num;
		return w_width;
	}
	pDpeConfig->Dpe_DVSSettings.pd_frame_num = 1;
	return 0;
}
void Get_Tile_Info(struct DPE_Config *pDpeConfig)
{
	unsigned int tile_occ_width[TILE_WITH_NUM] = {640, 512, 384};
	unsigned int w_width[TILE_WITH_NUM] = {0};
	unsigned int tile_num[TILE_WITH_NUM] = {0};
	unsigned int idx = 0, i = 0;
	unsigned int max_width = 0, interval = 0, st_x = 0;
	unsigned int engStart_x_L, engStart_x_R, frmHeight;
	unsigned int engWidth;

	engStart_x_L = pDpeConfig->Dpe_DVSSettings.l_eng_start_x;
	engStart_x_R = pDpeConfig->Dpe_DVSSettings.r_eng_start_x;
	frmHeight = pDpeConfig->Dpe_DVSSettings.frm_height;
	engWidth = pDpeConfig->Dpe_DVSSettings.eng_width;
#if IS_ENABLED(CONFIG_MTK_LEGACY)
	if (pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch <
			(tile_occ_width[TILE_WITH_NUM-1]+(2*engStart_x_L))) {
		LOG_ERR("Frame size [%d] is smaller than 384\n",
		pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch);
		pDpeConfig->Dpe_DVSSettings.pd_frame_num = 1;
	} else {
		for (i = 0; i < TILE_WITH_NUM; i++) {
			w_width[i] =
			Compute_Para(pDpeConfig, tile_occ_width[i]);
			tile_num[i] =
			pDpeConfig->Dpe_DVSSettings.pd_frame_num;
			if (w_width[i] > max_width) {
				max_width = w_width[i];
				idx = i;
			}
		}

			interval = (pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch - w_width[idx])/2;
			st_x = ((interval%16) == 0) ? (interval) : ((interval/16)*16);
			//pDpeConfig->Dpe_DVSSettings.eng_start_y = 0;
			pDpeConfig->Dpe_DVSSettings.frm_width =
			tile_occ_width[idx] + (2*engStart_x_L);
			pDpeConfig->Dpe_DVSSettings.eng_width =
			pDpeConfig->Dpe_DVSSettings.frm_width -
			engStart_x_L - engStart_x_R;
			pDpeConfig->Dpe_DVSSettings.eng_height
			= frmHeight -
			(2*pDpeConfig->Dpe_DVSSettings.eng_start_y);
			pDpeConfig->Dpe_DVSSettings.occ_width = tile_occ_width[idx];
			pDpeConfig->Dpe_DVSSettings.occ_start_x = engStart_x_L;
			pDpeConfig->Dpe_DVSSettings.pd_frame_num = tile_num[idx];
#if defined(UT_CASE)
		pDpeConfig->Dpe_DVSSettings.pd_st_x = 0;
#else
		pDpeConfig->Dpe_DVSSettings.pd_st_x = st_x;
#endif
	}
#else
	//!ISP7 tile mode
	if (pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch <
			(tile_occ_width[TILE_WITH_NUM-1]+(2*engStart_x_L))) {
		LOG_ERR("Frame size [%d] is smaller than 384\n",
		pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch);
		pDpeConfig->Dpe_DVSSettings.pd_frame_num = 1;
	} else {//!ISP7 tile mode
		for (i = 0; i < TILE_WITH_NUM; i++) {
			w_width[i] =
			Compute_Para(pDpeConfig, tile_occ_width[i]);
			LOG_INF("a w_width[%d] = %d\n", i, w_width[i]);
			tile_num[i] =
			pDpeConfig->Dpe_DVSSettings.pd_frame_num;
			LOG_INF("a tile_num[%d] = %d\n", i, tile_num[i]);
			if (w_width[i] > max_width) {
				max_width = w_width[i];
				idx = i;
			}
		}

		interval = (pDpeConfig->Dpe_DVSSettings.dram_pxl_pitch - w_width[idx])/2;
		st_x = ((interval%16) == 0) ? (interval) : ((interval/16)*16);
		pDpeConfig->Dpe_DVSSettings.frm_width = engWidth; //!ISP7
		//!pDpeConfig->Dpe_DVSSettings.eng_width =
		//!pDpeConfig->Dpe_DVSSettings.frm_width -
		//!engStart_x_L - engStart_x_R;
		pDpeConfig->Dpe_DVSSettings.eng_height
		= frmHeight -
		(2*pDpeConfig->Dpe_DVSSettings.eng_start_y);
		pDpeConfig->Dpe_DVSSettings.occ_width = tile_occ_width[idx];
		pDpeConfig->Dpe_DVSSettings.occ_start_x = engStart_x_L;
		pDpeConfig->Dpe_DVSSettings.pd_frame_num = tile_num[idx];


		LOG_INF("a occ_width = %d,pd_frame_num  = %d\n",
		pDpeConfig->Dpe_DVSSettings.occ_width,
		pDpeConfig->Dpe_DVSSettings.pd_frame_num);
	if ((pDpeConfig->Dpe_DVSSettings.dram_out_pitch_en) == 0 &&
			(pDpeConfig->Dpe_DVSSettings.occ_width > 640))
		LOG_INF("Dram_out_pitch_en not turn on, but occwidth over 640\n");
	if ((pDpeConfig->Dpe_DVSSettings.dram_out_pitch_en == 1) &&
			(pDpeConfig->Dpe_DVSSettings.occ_width < 640))
		LOG_INF("Dram_out_pitch_en turn on, but occwidth is smaller than 640\n");

#if defined(UT_CASE)
		pDpeConfig->Dpe_DVSSettings.pd_st_x = 0;
#else
		pDpeConfig->Dpe_DVSSettings.pd_st_x = st_x;
#endif
	}
#endif
}
/*
 *
 */
static signed int DPE_DumpReg(void)
#if BYPASS_REG
{
	return 0;
}
#else
{
	signed int Ret = 0;
	/*  */
	LOG_INF("- E.");
	spin_lock(&(DPEInfo.SpinLockDPE));
	if (g_u4EnableClockCount == 0) {
		spin_unlock(&(DPEInfo.SpinLockDPE));
		return 0;
	}
	spin_unlock(&(DPEInfo.SpinLockDPE));
//#if 1
	/*  */
	LOG_INF("DPE Config Info\n");
	/* DPE Config0 */
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL00_HW),
		(unsigned int)DPE_RD32(DVS_CTRL00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL01_HW),
		(unsigned int)DPE_RD32(DVS_CTRL01_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL02_HW),
		(unsigned int)DPE_RD32(DVS_CTRL02_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL03_HW),
		(unsigned int)DPE_RD32(DVS_CTRL03_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL06_HW),
		(unsigned int)DPE_RD32(DVS_CTRL06_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL07_HW),
		(unsigned int)DPE_RD32(DVS_CTRL07_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_IRQ_00_HW),
		(unsigned int)DPE_RD32(DVS_IRQ_00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL_STATUS0_HW),
		(unsigned int)DPE_RD32(DVS_CTRL_STATUS0_REG));
//	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL_STATUS1_HW),
//		(unsigned int)DPE_RD32(DVS_CTRL_STATUS1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL_STATUS2_HW),
		(unsigned int)DPE_RD32(DVS_CTRL_STATUS2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_IRQ_STATUS_HW),
		(unsigned int)DPE_RD32(DVS_IRQ_STATUS_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_FRM_STATUS0_HW),
		(unsigned int)DPE_RD32(DVS_FRM_STATUS0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_FRM_STATUS1_HW),
		(unsigned int)DPE_RD32(DVS_FRM_STATUS1_REG));
//	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_FRM_STATUS2_HW),
//		(unsigned int)DPE_RD32(DVS_FRM_STATUS2_REG));
//	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_FRM_STATUS3_HW),
//		(unsigned int)DPE_RD32(DVS_FRM_STATUS3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CUR_STATUS_HW),
		(unsigned int)DPE_RD32(DVS_CUR_STATUS_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_CTRL_HW),
		(unsigned int)DPE_RD32(DVS_SRC_CTRL_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CRC_CTRL_HW),
		(unsigned int)DPE_RD32(DVS_CRC_CTRL_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CRC_IN_HW),
		(unsigned int)DPE_RD32(DVS_CRC_IN_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_DRAM_STA0_HW),
		(unsigned int)DPE_RD32(DVS_DRAM_STA0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_DRAM_STA1_HW),
		(unsigned int)DPE_RD32(DVS_DRAM_STA1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_DRAM_ULT_HW),
		(unsigned int)DPE_RD32(DVS_DRAM_ULT_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_DRAM_PITCH_HW),
		(unsigned int)DPE_RD32(DVS_DRAM_PITCH_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_00_HW),
		(unsigned int)DPE_RD32(DVS_SRC_00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_01_HW),
		(unsigned int)DPE_RD32(DVS_SRC_01_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_02_HW),
		(unsigned int)DPE_RD32(DVS_SRC_02_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_03_HW),
		(unsigned int)DPE_RD32(DVS_SRC_03_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_04_HW),
		(unsigned int)DPE_RD32(DVS_SRC_04_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_05_L_FRM0_HW),
		(unsigned int)DPE_RD32(DVS_SRC_05_L_FRM0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_06_L_FRM1_HW),
		(unsigned int)DPE_RD32(DVS_SRC_06_L_FRM1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_07_L_FRM2_HW),
		(unsigned int)DPE_RD32(DVS_SRC_07_L_FRM2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_08_L_FRM3_HW),
		(unsigned int)DPE_RD32(DVS_SRC_08_L_FRM3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_09_R_FRM0_HW),
		(unsigned int)DPE_RD32(DVS_SRC_09_R_FRM0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_10_R_FRM1_HW),
		(unsigned int)DPE_RD32(DVS_SRC_10_R_FRM1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_11_R_FRM2_HW),
		(unsigned int)DPE_RD32(DVS_SRC_11_R_FRM2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_12_R_FRM3_HW),
		(unsigned int)DPE_RD32(DVS_SRC_12_R_FRM3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_13_L_VMAP0_HW),
		(unsigned int)DPE_RD32(DVS_SRC_13_L_VMAP0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_14_L_VMAP1_HW),
		(unsigned int)DPE_RD32(DVS_SRC_14_L_VMAP1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_15_L_VMAP2_HW),
		(unsigned int)DPE_RD32(DVS_SRC_15_L_VMAP2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_16_L_VMAP3_HW),
		(unsigned int)DPE_RD32(DVS_SRC_16_L_VMAP3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_17_R_VMAP0_HW),
		(unsigned int)DPE_RD32(DVS_SRC_17_R_VMAP0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_18_R_VMAP1_HW),
		(unsigned int)DPE_RD32(DVS_SRC_18_R_VMAP1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_19_R_VMAP2_HW),
		(unsigned int)DPE_RD32(DVS_SRC_19_R_VMAP2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_20_R_VMAP3_HW),
		(unsigned int)DPE_RD32(DVS_SRC_20_R_VMAP3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_21_P4_L_DV_ADR_HW),
		(unsigned int)DPE_RD32(DVS_SRC_21_P4_L_DV_ADR_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_26_OCCDV0_HW),
		(unsigned int)DPE_RD32(DVS_SRC_26_OCCDV0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_27_OCCDV1_HW),
		(unsigned int)DPE_RD32(DVS_SRC_27_OCCDV1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_28_OCCDV2_HW),
		(unsigned int)DPE_RD32(DVS_SRC_28_OCCDV2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_29_OCCDV3_HW),
		(unsigned int)DPE_RD32(DVS_SRC_29_OCCDV3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_34_P4_R_DV_ADR_HW),
		(unsigned int)DPE_RD32(DVS_SRC_34_P4_R_DV_ADR_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_42_OCCDV_EXT0_HW),
		(unsigned int)DPE_RD32(DVS_SRC_42_OCCDV_EXT0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_43_OCCDV_EXT1_HW),
		(unsigned int)DPE_RD32(DVS_SRC_43_OCCDV_EXT1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_44_OCCDV_EXT2_HW),
		(unsigned int)DPE_RD32(DVS_SRC_44_OCCDV_EXT2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_45_OCCDV_EXT3_HW),
		(unsigned int)DPE_RD32(DVS_SRC_45_OCCDV_EXT3_REG));
//!
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_SRC_46_HW),
		(unsigned int)DPE_RD32(DVS_SRC_46_REG));
//!
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CRC_OUT_0_HW),
		(unsigned int)DPE_RD32(DVS_CRC_OUT_0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CRC_OUT_1_HW),
		(unsigned int)DPE_RD32(DVS_CRC_OUT_1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CRC_OUT_2_HW),
		(unsigned int)DPE_RD32(DVS_CRC_OUT_2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CRC_OUT_3_HW),
		(unsigned int)DPE_RD32(DVS_CRC_OUT_3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_DRAM_SEC_HW),
		(unsigned int)DPE_RD32(DVS_DRAM_SEC_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL_RESERVED_HW),
		(unsigned int)DPE_RD32(DVS_CTRL_RESERVED_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_CTRL_ATPG_HW),
		(unsigned int)DPE_RD32(DVS_CTRL_ATPG_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_00_HW),
		(unsigned int)DPE_RD32(DVS_ME_00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_01_HW),
		(unsigned int)DPE_RD32(DVS_ME_01_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_02_HW),
		(unsigned int)DPE_RD32(DVS_ME_02_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_03_HW),
		(unsigned int)DPE_RD32(DVS_ME_03_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_04_HW),
		(unsigned int)DPE_RD32(DVS_ME_04_REG));
	//LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_05_HW),
	//	(unsigned int)DPE_RD32(DVS_ME_05_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_06_HW),
		(unsigned int)DPE_RD32(DVS_ME_06_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_07_HW),
		(unsigned int)DPE_RD32(DVS_ME_07_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_08_HW),
		(unsigned int)DPE_RD32(DVS_ME_08_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_09_HW),
		(unsigned int)DPE_RD32(DVS_ME_09_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_10_HW),
		(unsigned int)DPE_RD32(DVS_ME_10_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_11_HW),
		(unsigned int)DPE_RD32(DVS_ME_11_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_12_HW),
		(unsigned int)DPE_RD32(DVS_ME_12_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_13_HW),
		(unsigned int)DPE_RD32(DVS_ME_13_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_14_HW),
		(unsigned int)DPE_RD32(DVS_ME_14_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_15_HW),
		(unsigned int)DPE_RD32(DVS_ME_15_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_16_HW),
		(unsigned int)DPE_RD32(DVS_ME_16_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_17_HW),
		(unsigned int)DPE_RD32(DVS_ME_17_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_18_HW),
		(unsigned int)DPE_RD32(DVS_ME_18_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_19_HW),
		(unsigned int)DPE_RD32(DVS_ME_19_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_20_HW),
		(unsigned int)DPE_RD32(DVS_ME_20_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_21_HW),
		(unsigned int)DPE_RD32(DVS_ME_21_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_22_HW),
		(unsigned int)DPE_RD32(DVS_ME_22_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_23_HW),
		(unsigned int)DPE_RD32(DVS_ME_23_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_24_HW),
		(unsigned int)DPE_RD32(DVS_ME_24_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_25_HW),
		(unsigned int)DPE_RD32(DVS_ME_25_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_26_HW),
		(unsigned int)DPE_RD32(DVS_ME_26_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_27_HW),
		(unsigned int)DPE_RD32(DVS_ME_27_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_28_HW),
		(unsigned int)DPE_RD32(DVS_ME_28_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_29_HW),
		(unsigned int)DPE_RD32(DVS_ME_29_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_30_HW),
		(unsigned int)DPE_RD32(DVS_ME_30_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_31_HW),
		(unsigned int)DPE_RD32(DVS_ME_31_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_32_HW),
		(unsigned int)DPE_RD32(DVS_ME_32_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_33_HW),
		(unsigned int)DPE_RD32(DVS_ME_33_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_34_HW),
		(unsigned int)DPE_RD32(DVS_ME_34_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_35_HW),
		(unsigned int)DPE_RD32(DVS_ME_35_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_36_HW),
		(unsigned int)DPE_RD32(DVS_ME_36_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_37_HW),
		(unsigned int)DPE_RD32(DVS_ME_37_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_38_HW),
		(unsigned int)DPE_RD32(DVS_ME_38_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_39_HW),
		(unsigned int)DPE_RD32(DVS_ME_39_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_DEBUG_HW),
		(unsigned int)DPE_RD32(DVS_DEBUG_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_RESERVED_HW),
		(unsigned int)DPE_RD32(DVS_ME_RESERVED_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_ATPG_HW),
		(unsigned int)DPE_RD32(DVS_ME_ATPG_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_40_HW),
		(unsigned int)DPE_RD32(DVS_ME_40_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_ME_41_HW),
		(unsigned int)DPE_RD32(DVS_ME_41_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_0_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_1_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_2_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_3_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_4_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_4_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_5_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_5_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_6_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_6_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_7_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_7_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_8_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_8_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_9_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_9_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_10_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_10_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_PQ_11_HW),
		(unsigned int)DPE_RD32(DVS_OCC_PQ_11_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVS_OCC_ATPG_HW),
		(unsigned int)DPE_RD32(DVS_OCC_ATPG_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL00_HW),
		(unsigned int)DPE_RD32(DVP_CTRL00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL01_HW),
		(unsigned int)DPE_RD32(DVP_CTRL01_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL02_HW),
		(unsigned int)DPE_RD32(DVP_CTRL02_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL03_HW),
		(unsigned int)DPE_RD32(DVP_CTRL03_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL04_HW),
		(unsigned int)DPE_RD32(DVP_CTRL04_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL07_HW),
		(unsigned int)DPE_RD32(DVP_CTRL07_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_IRQ_00_HW),
		(unsigned int)DPE_RD32(DVP_IRQ_00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_IRQ_01_HW),
		(unsigned int)DPE_RD32(DVP_IRQ_01_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_STATUS0_HW),
		(unsigned int)DPE_RD32(DVP_CTRL_STATUS0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_STATUS1_HW),
		(unsigned int)DPE_RD32(DVP_CTRL_STATUS1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_STATUS2_HW),
		(unsigned int)DPE_RD32(DVP_CTRL_STATUS2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_IRQ_STATUS_HW),
		(unsigned int)DPE_RD32(DVP_IRQ_STATUS_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_FRM_STATUS0_HW),
		(unsigned int)DPE_RD32(DVP_FRM_STATUS0_REG));
//	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_FRM_STATUS1_HW),
//		(unsigned int)DPE_RD32(DVP_FRM_STATUS1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_FRM_STATUS2_HW),
		(unsigned int)DPE_RD32(DVP_FRM_STATUS2_REG));
//	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_FRM_STATUS3_HW),
//		(unsigned int)DPE_RD32(DVP_FRM_STATUS3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CUR_STATUS_HW),
		(unsigned int)DPE_RD32(DVP_CUR_STATUS_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_00_HW),
		(unsigned int)DPE_RD32(DVP_SRC_00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_01_HW),
		(unsigned int)DPE_RD32(DVP_SRC_01_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_02_HW),
		(unsigned int)DPE_RD32(DVP_SRC_02_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_03_HW),
		(unsigned int)DPE_RD32(DVP_SRC_03_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_04_HW),
		(unsigned int)DPE_RD32(DVP_SRC_04_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_05_Y_FRM0_HW),
		(unsigned int)DPE_RD32(DVP_SRC_05_Y_FRM0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_06_Y_FRM1_HW),
		(unsigned int)DPE_RD32(DVP_SRC_06_Y_FRM1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_07_Y_FRM2_HW),
		(unsigned int)DPE_RD32(DVP_SRC_07_Y_FRM2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_08_Y_FRM3_HW),
		(unsigned int)DPE_RD32(DVP_SRC_08_Y_FRM3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_09_C_FRM0_HW),
		(unsigned int)DPE_RD32(DVP_SRC_09_C_FRM0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_10_C_FRM1_HW),
		(unsigned int)DPE_RD32(DVP_SRC_10_C_FRM1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_11_C_FRM2_HW),
		(unsigned int)DPE_RD32(DVP_SRC_11_C_FRM2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_12_C_FRM3_HW),
		(unsigned int)DPE_RD32(DVP_SRC_12_C_FRM3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_13_OCCDV0_HW),
		(unsigned int)DPE_RD32(DVP_SRC_13_OCCDV0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_14_OCCDV1_HW),
		(unsigned int)DPE_RD32(DVP_SRC_14_OCCDV1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_15_OCCDV2_HW),
		(unsigned int)DPE_RD32(DVP_SRC_15_OCCDV2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_16_OCCDV3_HW),
		(unsigned int)DPE_RD32(DVP_SRC_16_OCCDV3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_17_CRM_HW),
		(unsigned int)DPE_RD32(DVP_SRC_17_CRM_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_18_ASF_RMDV_HW),
		(unsigned int)DPE_RD32(DVP_SRC_18_ASF_RMDV_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_19_ASF_RDDV_HW),
		(unsigned int)DPE_RD32(DVP_SRC_19_ASF_RDDV_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_20_ASF_DV0_HW),
		(unsigned int)DPE_RD32(DVP_SRC_20_ASF_DV0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_21_ASF_DV1_HW),
		(unsigned int)DPE_RD32(DVP_SRC_21_ASF_DV1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_22_ASF_DV2_HW),
		(unsigned int)DPE_RD32(DVP_SRC_22_ASF_DV2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_23_ASF_DV3_HW),
		(unsigned int)DPE_RD32(DVP_SRC_23_ASF_DV3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_24_WMF_HFDV_HW),
		(unsigned int)DPE_RD32(DVP_SRC_24_WMF_HFDV_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_25_WMF_DV0_HW),
		(unsigned int)DPE_RD32(DVP_SRC_25_WMF_DV0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_26_WMF_DV1_HW),
		(unsigned int)DPE_RD32(DVP_SRC_26_WMF_DV1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_27_WMF_DV2_HW),
		(unsigned int)DPE_RD32(DVP_SRC_27_WMF_DV2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_28_WMF_DV3_HW),
		(unsigned int)DPE_RD32(DVP_SRC_28_WMF_DV3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_00_HW),
		(unsigned int)DPE_RD32(DVP_CORE_00_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_01_HW),
		(unsigned int)DPE_RD32(DVP_CORE_01_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_02_HW),
		(unsigned int)DPE_RD32(DVP_CORE_02_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_03_HW),
		(unsigned int)DPE_RD32(DVP_CORE_03_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_04_HW),
		(unsigned int)DPE_RD32(DVP_CORE_04_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_05_HW),
		(unsigned int)DPE_RD32(DVP_CORE_05_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_06_HW),
		(unsigned int)DPE_RD32(DVP_CORE_06_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_07_HW),
		(unsigned int)DPE_RD32(DVP_CORE_07_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_08_HW),
		(unsigned int)DPE_RD32(DVP_CORE_08_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_09_HW),
		(unsigned int)DPE_RD32(DVP_CORE_09_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_10_HW),
		(unsigned int)DPE_RD32(DVP_CORE_10_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_11_HW),
		(unsigned int)DPE_RD32(DVP_CORE_11_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_12_HW),
		(unsigned int)DPE_RD32(DVP_CORE_12_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_13_HW),
		(unsigned int)DPE_RD32(DVP_CORE_13_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_14_HW),
		(unsigned int)DPE_RD32(DVP_CORE_14_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_15_HW),
		(unsigned int)DPE_RD32(DVP_CORE_15_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_16_HW),
		(unsigned int)DPE_RD32(DVP_CORE_16_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_SRC_CTRL_HW),
		(unsigned int)DPE_RD32(DVP_SRC_CTRL_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_RESERVED_HW),
		(unsigned int)DPE_RD32(DVP_CTRL_RESERVED_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_ATPG_HW),
		(unsigned int)DPE_RD32(DVP_CTRL_ATPG_REG));

	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CRC_OUT_0_HW),
		(unsigned int)DPE_RD32(DVP_CRC_OUT_0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CRC_OUT_1_HW),
		(unsigned int)DPE_RD32(DVP_CRC_OUT_1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CRC_OUT_2_HW),
		(unsigned int)DPE_RD32(DVP_CRC_OUT_2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CRC_CTRL_HW),
		(unsigned int)DPE_RD32(DVP_CRC_CTRL_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CRC_OUT_HW),
		(unsigned int)DPE_RD32(DVP_CRC_OUT_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CRC_IN_HW),
		(unsigned int)DPE_RD32(DVP_CRC_IN_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_DRAM_STA_HW),
		(unsigned int)DPE_RD32(DVP_DRAM_STA_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_DRAM_ULT_HW),
		(unsigned int)DPE_RD32(DVP_DRAM_ULT_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_DRAM_PITCH_HW),
		(unsigned int)DPE_RD32(DVP_DRAM_PITCH_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_CORE_CRC_IN_HW),
		(unsigned int)DPE_RD32(DVP_CORE_CRC_IN_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_13_OCCDV0_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_13_OCCDV0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_14_OCCDV1_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_14_OCCDV1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_15_OCCDV2_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_15_OCCDV2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_16_OCCDV3_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_16_OCCDV3_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_18_ASF_RMDV_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_18_ASF_RMDV_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_19_ASF_RDDV_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_19_ASF_RDDV_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_20_ASF_DV0_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_20_ASF_DV0_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_21_ASF_DV1_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_21_ASF_DV1_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_22_ASF_DV2_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_22_ASF_DV2_REG));
	LOG_INF("[0x%08X %08X]\n", (unsigned int)(DVP_EXT_SRC_23_ASF_DV3_HW),
		(unsigned int)DPE_RD32(DVP_EXT_SRC_23_ASF_DV3_REG));
//#endif
	LOG_INF("- X.");
	/*  */
	return Ret;
}
#endif
#if !IS_ENABLED(CONFIG_MTK_LEGACY) && IS_ENABLED(CONFIG_COMMON_CLK) /*CCF*/
static inline int DPE_Prepare_Enable_ccf_clock(void)
{
	struct clk *clocks[] = { dpe_clk.VCORE_GALS, dpe_clk.MAIN_GALS,
		dpe_clk.IMG_IPE, dpe_clk.IPE_DPE, dpe_clk.IPE_SMI_LARB12,
		dpe_clk.IPE_TOP };
	int ret;
	unsigned int i;

	ret = pm_runtime_resume_and_get(gdev);
	if (ret < 0) return ret;
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		ret = clk_prepare_enable(clocks[i]);
		if (ret) {
			while (i) clk_disable_unprepare(clocks[--i]);
			pm_runtime_put_sync(gdev);
			return ret;
		}
	}
	return 0;
}
static inline void DPE_Disable_Unprepare_ccf_clock(void)
{
	clk_disable_unprepare(dpe_clk.IPE_TOP);
	clk_disable_unprepare(dpe_clk.IPE_SMI_LARB12);
	clk_disable_unprepare(dpe_clk.IPE_DPE);
	clk_disable_unprepare(dpe_clk.IMG_IPE);
	clk_disable_unprepare(dpe_clk.MAIN_GALS);
	clk_disable_unprepare(dpe_clk.VCORE_GALS);
	pm_runtime_put_sync(gdev);
}
#endif
static DEFINE_MUTEX(dpe_power_lock);
static int DPE_EnableClock(bool enable)
{
	int ret = 0;
	mutex_lock(&dpe_power_lock);
	if (enable) {
		if (!g_u4EnableClockCount) {
			ret = DPE_Prepare_Enable_ccf_clock();
			if (ret) goto out;
		}
		spin_lock(&DPEInfo.SpinLockDPE);
		g_u4EnableClockCount++;
		spin_unlock(&DPEInfo.SpinLockDPE);
	} else {
		if (!g_u4EnableClockCount) { ret = -EINVAL; goto out; }
		spin_lock(&DPEInfo.SpinLockDPE);
		g_u4EnableClockCount--;
		spin_unlock(&DPEInfo.SpinLockDPE);
		if (!g_u4EnableClockCount) DPE_Disable_Unprepare_ccf_clock();
	}
out:
	mutex_unlock(&dpe_power_lock);
	return ret;
}
/*******************************************************************************
 *
 ******************************************************************************/
static inline void DPE_Reset(void)
{
	LOG_DBG("- E.");
	LOG_DBG(" DPE Reset start!\n");
	spin_lock(&(DPEInfo.SpinLockDPERef));
	if (DPEInfo.UserCount > 1) {
		spin_unlock(&(DPEInfo.SpinLockDPERef));
		LOG_DBG("Curr UserCount(%d) users exist", DPEInfo.UserCount);
	} else {
		spin_unlock(&(DPEInfo.SpinLockDPERef));
		/* Reset DPE flow */
		#ifdef CMdq_en
		DPE_MASKWR(DVS_CTRL01_REG, 0x70000000, 0x70000000);
		DPE_MASKWR(DVP_CTRL01_REG, 0x70000000, 0x70000000);
		DPE_MASKWR(DVS_CTRL01_REG, 0x00000000, 0x70000000);
		DPE_MASKWR(DVP_CTRL01_REG, 0x00000000, 0x70000000);
		#endif
		LOG_DBG(" DPE Reset end!\n");
	}
}
/*******************************************************************************
 *
 ******************************************************************************/
static signed int DPE_ReadReg(struct DPE_REG_IO_STRUCT *pRegIo)
{
	unsigned int i;
	signed int Ret = 0;
	/*  */
	struct DPE_REG_STRUCT reg;
	/* unsigned int* pData = (unsigned int*)pRegIo->Data; */
	struct DPE_REG_STRUCT *pData = (struct DPE_REG_STRUCT *) pRegIo->pData;

	if ((pRegIo->pData == NULL) ||
		(pRegIo->Count == 0) ||
		(pRegIo->Count > DPE_MAX_REG_CNT)) {
		LOG_ERR("ERROR: pRegIo->pData is NULL or Count:%d\n",
			pRegIo->Count);
		Ret = -EFAULT;
		goto EXIT;
	}
	for (i = 0; i < pRegIo->Count; i++) {
		if (get_user(reg.Addr, (unsigned int *) &pData->Addr) != 0) {
			LOG_ERR("get_user failed");
			Ret = -EFAULT;
			goto EXIT;
		}
		/* pData++; */
		/*  */
		if ((ISP_DPE_BASE + reg.Addr >= ISP_DPE_BASE)
		    && (ISP_DPE_BASE + reg.Addr <
						(ISP_DPE_BASE + DPE_REG_RANGE))
			&& ((reg.Addr & 0x3) == 0)) {
			reg.Val = DPE_RD32(ISP_DPE_BASE + reg.Addr);
		} else {
			LOG_ERR(
			"Wrong address(0x%p)", (ISP_DPE_BASE + reg.Addr));
			reg.Val = 0;
		}
		/*  */
		if (put_user(reg.Val, (unsigned int *) &(pData->Val)) != 0) {
			LOG_ERR("put_user failed");
			Ret = -EFAULT;
			goto EXIT;
		}
		pData++;
		/*  */
	}
	/*  */
EXIT:
	return Ret;
}
/*******************************************************************************
 *
 ******************************************************************************/
static signed int DPE_WriteRegToHw(struct DPE_REG_STRUCT *pReg,
							unsigned int Count)
{
	signed int Ret = 0;
	unsigned int i;
	bool dbgWriteReg;

	spin_lock(&(DPEInfo.SpinLockDPE));
	dbgWriteReg = DPEInfo.DebugMask & DPE_DBG_WRITE_REG;
	spin_unlock(&(DPEInfo.SpinLockDPE));
	/*  */
	if (dbgWriteReg)
		LOG_DBG("- E.\n");
	/*  */
	for (i = 0; i < Count; i++) {
		if (dbgWriteReg) {
			LOG_DBG("Addr(0x%lx), Val(0x%x)\n",
				(unsigned long)(ISP_DPE_BASE + pReg[i].Addr),
				(unsigned int) (pReg[i].Val));
		}
		if (((ISP_DPE_BASE + pReg[i].Addr) <
						(ISP_DPE_BASE + DPE_REG_RANGE))
			&& ((pReg[i].Addr & 0x3) == 0)) {
			#ifdef DPE_WR32_en
			DPE_WR32(ISP_DPE_BASE + pReg[i].Addr, pReg[i].Val);
			#endif
		} else {
			LOG_ERR("wrong address(0x%lx)\n",
				(unsigned long)(ISP_DPE_BASE + pReg[i].Addr));
		}
	}
	/*  */
	return Ret;
}
/*******************************************************************************
 *
 ******************************************************************************/
static signed int DPE_WriteReg(struct DPE_REG_IO_STRUCT *pRegIo)
{
	signed int Ret = 0;
	/*
	 *  signed int TimeVd = 0;
	 *  signed int TimeExpdone = 0;
	 *  signed int TimeTasklet = 0;
	 */
	/* unsigned char* pData = NULL; */
	struct DPE_REG_STRUCT *pData = NULL;
	/*  */
	if (DPEInfo.DebugMask & DPE_DBG_WRITE_REG)
		LOG_DBG("Data(0x%p), Count(%d)\n", (pRegIo->pData),
							(pRegIo->Count));
	/*  */
	if ((pRegIo->pData == NULL) ||
		(pRegIo->Count == 0) ||
		(pRegIo->Count > DPE_MAX_REG_CNT)) {
		LOG_ERR("ERROR: pRegIo->pData is NULL or Count:%d\n",
			pRegIo->Count);
		Ret = -EFAULT;
		goto EXIT;
	}
	pData = kmalloc((pRegIo->Count) *
		sizeof(struct DPE_REG_STRUCT),
		GFP_KERNEL); /* Use GFP_KERNEL instead of GFP_ATOMIC */
if (pData == NULL) {
	LOG_INF("ERROR: kmalloc failed, (process, pid, tgid)=(%s, %d, %d)\n",
		current->comm,
		current->pid,
		current->tgid);
		Ret = -ENOMEM;
		goto EXIT;
}
	if (copy_from_user(pData,
		(void __user *)(pRegIo->pData),
		pRegIo->Count * sizeof(struct DPE_REG_STRUCT)) != 0) {
		LOG_ERR("copy_from_user failed\n");
		Ret = -EFAULT;
		goto EXIT;
	}
	/*  */
	Ret = DPE_WriteRegToHw(pData, pRegIo->Count);
	/*  */
EXIT:
	if (pData != NULL) {
		kfree(pData);
		pData = NULL;
	}
	return Ret;
}
/*******************************************************************************
 *
 ******************************************************************************/
static signed int DPE_WaitIrq(struct DPE_WAIT_IRQ_STRUCT *WaitIrq)
{
	signed int Ret = 0;
	signed int Timeout = WaitIrq->Timeout;
	enum DPE_PROCESS_ID_ENUM whichReq = DPE_PROCESS_ID_NONE;
	/*unsigned int i;*/
	unsigned long flags; /* old: unsigned int flags;*/
	unsigned int irqStatus;
	/*int cnt = 0;*/
	//struct timeval time_getrequest;
	struct timespec64 time_getrequest;
	//unsigned long long sec = 0;
	//unsigned long usec = 0;
	unsigned int p;
	/* do_gettimeofday(&time_getrequest); */
	//sec = cpu_clock(0);	/* ns */
	//do_div(sec, 1000);	/* usec */
	//usec = do_div(sec, 1000000);	/* sec and usec */
	//time_getrequest.tv_usec = usec;
	//time_getrequest.tv_sec = sec;
	ktime_get_ts64(&time_getrequest);
	/* Debug interrupt */
	if (DPEInfo.DebugMask & DPE_DBG_INT) {
		if (WaitIrq->Status & DPEInfo.IrqInfo.Mask[WaitIrq->Type]) {
			if (WaitIrq->UserKey > 0) {
				LOG_DBG(
				"+WaitIrq clr(%d), Type(%d), Stat(0x%08X), Timeout(%d),usr(%d), ProcID(%d)\n",
				WaitIrq->Clear, WaitIrq->Type, WaitIrq->Status,
				WaitIrq->Timeout, WaitIrq->UserKey,
				WaitIrq->ProcessID);
			}
		}
	}
	/* 1. wait type update */
	if (WaitIrq->Clear == DPE_IRQ_CLEAR_STATUS) {
		spin_lock_irqsave(&(DPEInfo.SpinLockIrq[WaitIrq->Type]), flags);
		/* LOG_DBG(
		 * "WARNING: Clear(%d), Type(%d): IrqStatus(0x%08X) has been
		 * cleared" ,WaitIrq->EventInfo.Clear,WaitIrq->Type,
		 * DPEInfo.IrqInfo.Status[WaitIrq->Type]);
		 * DPEInfo.IrqInfo.Status[WaitIrq->Type][
		 * WaitIrq->EventInfo.UserKey] &= (~WaitIrq->EventInfo.Status);
		 */
		DPEInfo.IrqInfo.Status[WaitIrq->Type] &= (~WaitIrq->Status);
		spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[WaitIrq->Type]),
									flags);
		return Ret;
	}
	if (WaitIrq->Clear == DPE_IRQ_CLEAR_WAIT) {
		spin_lock_irqsave(&(DPEInfo.SpinLockIrq[WaitIrq->Type]), flags);
		if (DPEInfo.IrqInfo.Status[WaitIrq->Type] & WaitIrq->Status)
			DPEInfo.IrqInfo.Status[WaitIrq->Type] &=
							(~WaitIrq->Status);
		spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[WaitIrq->Type]),
									flags);
	} else if (WaitIrq->Clear == DPE_IRQ_CLEAR_ALL) {
		spin_lock_irqsave(&(DPEInfo.SpinLockIrq[WaitIrq->Type]), flags);
		DPEInfo.IrqInfo.Status[WaitIrq->Type] = 0;
		spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[WaitIrq->Type]),
									flags);
	}
	/* DPE_IRQ_WAIT_CLEAR ==> do nothing */
	/* Store irqinfo status in here to redeuce time of spin_lock_irqsave */
	spin_lock_irqsave(&(DPEInfo.SpinLockIrq[WaitIrq->Type]), flags);
	irqStatus = DPEInfo.IrqInfo.Status[WaitIrq->Type];
	spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[WaitIrq->Type]), flags);
	if (WaitIrq->Status & DPE_INT_ST) {
		whichReq = DPE_PROCESS_ID_DPE;
	} else {
		LOG_ERR(
		"No Such Stats can be waited!! irq Type/User/Sts/Pid(0x%x/%d/0x%x/%d)\n",
			WaitIrq->Type, WaitIrq->UserKey, WaitIrq->Status,
			WaitIrq->ProcessID);
	}
	/* 2. start to wait signal */
	Timeout = wait_event_interruptible_timeout(DPEInfo.WaitQueueHead,
						   READ_ONCE(dpe_faulted) || DPE_GetIRQState(
							WaitIrq->Type,
							WaitIrq->UserKey,
							WaitIrq->Status,
							whichReq,
							WaitIrq->ProcessID),
						   DPE_MsToJiffies(
							WaitIrq->Timeout));
	if (READ_ONCE(dpe_faulted)) return -EIO;
	p = WaitIrq->ProcessID % IRQ_USER_NUM_MAX;
	/* check if user is interrupted by system signal */
	if ((Timeout != 0) &&
		(!DPE_GetIRQState(WaitIrq->Type, WaitIrq->UserKey,
			WaitIrq->Status, whichReq, WaitIrq->ProcessID))) {
		LOG_ERR(
		"interrupted by system, timeout(%d),irq Type/User/Sts/whichReq/Pid(0x%x/%d/0x%x/%d/%d)\n",
		Timeout, WaitIrq->Type, WaitIrq->UserKey, WaitIrq->Status,
						whichReq, WaitIrq->ProcessID);
		Ret = -ERESTARTSYS;	/* actually it should be -ERESTARTSYS */
		goto EXIT;
	}
	/* timeout */
	if (Timeout == 0) {
		spin_lock_irqsave(&(DPEInfo.SpinLockIrq[WaitIrq->Type]),
									flags);
		irqStatus = DPEInfo.IrqInfo.Status[WaitIrq->Type];
		spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[WaitIrq->Type]),
									flags);
		LOG_ERR(
		"WaitIrq Timeout:Tout(%d) clr(%d) Type(%d) IrqStat(0x%08X) WaitStat(0x%08X) usrKey(%d)\n",
		     WaitIrq->Timeout, WaitIrq->Clear, WaitIrq->Type, irqStatus,
			WaitIrq->Status, WaitIrq->UserKey);
		LOG_ERR(
		"WaitIrq Timeout:whichReq(%d),ProcID(%d) DpeIrqCnt[%d](0x%08X) WriteReq(0x%08X) ReadReq(0x%08X)\n",
			whichReq, WaitIrq->ProcessID,
			p, DPEInfo.IrqInfo.DpeIrqCnt[p],
			DPEInfo.WriteReqIdx, DPEInfo.ReadReqIdx);
		if (WaitIrq->bDumpReg) {
			DPE_DumpReg();
			dpe_request_dump(&dpe_reqs_dvs);
		}
		Ret = -EFAULT;
		goto EXIT;
	} else {
#ifdef __DPE_KERNEL_PERFORMANCE_MEASURE__
		mt_kernel_trace_begin("DPE WaitIrq");
#endif
		spin_lock_irqsave(&(DPEInfo.SpinLockIrq[WaitIrq->Type]), flags);
		irqStatus = DPEInfo.IrqInfo.Status[WaitIrq->Type];
		spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[WaitIrq->Type]),
									flags);
		if (WaitIrq->Clear == DPE_IRQ_WAIT_CLEAR) {
			spin_lock_irqsave(&(DPEInfo.SpinLockIrq[WaitIrq->Type]),
									flags);
			if (WaitIrq->Status & DPE_INT_ST) {
				DPEInfo.IrqInfo.DpeIrqCnt[p]--;
				if (DPEInfo.IrqInfo.DpeIrqCnt[p] == 0)
					DPEInfo.IrqInfo.Status[WaitIrq->Type] &=
							(~WaitIrq->Status);
			} else {
				LOG_ERR(
				"DPE_IRQ_WAIT_CLEAR Error, Type(%d), WaitStatus(0x%08X)",
					WaitIrq->Type, WaitIrq->Status);
			}
			spin_unlock_irqrestore(
				&(DPEInfo.SpinLockIrq[WaitIrq->Type]), flags);
		}
#ifdef __DPE_KERNEL_PERFORMANCE_MEASURE__
		mt_kernel_trace_end();
#endif
	}
EXIT:
	return Ret;
}
/*******************************************************************************
 *
 ******************************************************************************/
/* Only outer headers contain ABI-sized pointers. Nested native payloads
 * remain user pointers and are copied by the existing request handlers. */
static int dpe_get_reg_header(struct DPE_REG_IO_STRUCT *out, void __user *arg)
{
#if IS_ENABLED(CONFIG_COMPAT)
	if (in_compat_syscall()) {
		struct compat_DPE_REG_IO_STRUCT v;
		if (copy_from_user(&v, arg, sizeof(v))) return -EFAULT;
		out->pData = compat_ptr(v.pData);
		out->Count = v.Count;
		return 0;
	}
#endif
	return copy_from_user(out, arg, sizeof(*out)) ? -EFAULT : 0;
}
static int dpe_get_req_header(struct DPE_Request *out, void __user *arg)
{
#if IS_ENABLED(CONFIG_COMPAT)
	if (in_compat_syscall()) {
		struct compat_DPE_Request v;
		if (copy_from_user(&v, arg, sizeof(v))) return -EFAULT;
		out->m_pDpeConfig = compat_ptr(v.m_pDpeConfig);
		out->m_ReqNum = v.m_ReqNum;
		return 0;
	}
#endif
	return copy_from_user(out, arg, sizeof(*out)) ? -EFAULT : 0;
}
static int dpe_put_req_header(void __user *arg, const struct DPE_Request *in)
{
#if IS_ENABLED(CONFIG_COMPAT)
	if (in_compat_syscall()) {
		struct compat_DPE_Request v = {
			.m_ReqNum = in->m_ReqNum,
			.m_pDpeConfig = ptr_to_compat(in->m_pDpeConfig),
		};
		return copy_to_user(arg, &v, sizeof(v)) ? -EFAULT : 0;
	}
#endif
	return copy_to_user(arg, in, sizeof(*in)) ? -EFAULT : 0;
}

static long DPE_ioctl(struct file *pFile, unsigned int Cmd, unsigned long Param)
{
	signed int Ret = 0;
	/*unsigned int pid = 0;*/
	struct DPE_REG_IO_STRUCT RegIo;
	struct DPE_WAIT_IRQ_STRUCT IrqInfo;
	struct DPE_CLEAR_IRQ_STRUCT ClearIrq;
	struct DPE_Config dpe_DpeConfig;
	struct DPE_Request dpe_DpeReq;
	struct DPE_Config *ioctl_configs __free(kfree) = NULL;
	signed int enqnum;
	struct DPE_USER_INFO_STRUCT *pUserInfo;
	int enqueNum;
	int dequeNum;
	unsigned long flags;
	int req_temp;
	/* old: unsigned int flags;*//* FIX to avoid build warning */
	/*  */
	if (READ_ONCE(dpe_faulted)) return -EIO;
	if (pFile->private_data == NULL) {
		LOG_WRN("private_data NULL,(process, pid, tgid)=(%s, %d, %d)",
			current->comm,
				current->pid, current->tgid);
		return -EFAULT;
	}
	/*  */
	pUserInfo = (struct DPE_USER_INFO_STRUCT *) (pFile->private_data);
	/*  */
	switch (Cmd) {
	case DPE_RESET:
		{
			spin_lock(&(DPEInfo.SpinLockDPE));
			DPE_Reset();
			spin_unlock(&(DPEInfo.SpinLockDPE));
			break;
		}
		/*  */
	case DPE_DUMP_REG:
		{
			Ret = DPE_DumpReg();
			break;
		}
	case DPE_DUMP_ISR_LOG:
		{
			unsigned int currentPPB = m_CurrentPPB;

			spin_lock_irqsave(
				&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
									flags);
			m_CurrentPPB = (m_CurrentPPB + 1) % LOG_PPNUM;
			spin_unlock_irqrestore(
				&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
									flags);
			IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVP_ST, currentPPB,
								_LOG_INF);
			IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVP_ST, currentPPB,
								_LOG_ERR);
			break;
		}
	case DPE_READ_REGISTER:
		{
			if (dpe_get_reg_header(&RegIo, (void __user *)Param) == 0) {
				Ret = DPE_ReadReg(&RegIo);
			} else {
				LOG_ERR(
				"DPE_READ_REGISTER copy_from_user failed");
				Ret = -EFAULT;
			}
			break;
		}
	case DPE_WRITE_REGISTER:
		{
			if (dpe_get_reg_header(&RegIo, (void __user *)Param) == 0) {
				Ret = DPE_WriteReg(&RegIo);
			} else {
				LOG_ERR(
				"DPE_WRITE_REGISTER copy_from_user failed");
				Ret = -EFAULT;
			}
			break;
		}
	case DPE_WAIT_IRQ:
		{
			if (copy_from_user(&IrqInfo, (void *)Param,
				sizeof(struct DPE_WAIT_IRQ_STRUCT)) == 0) {
				/*  */
				if ((IrqInfo.Type >= DPE_IRQ_TYPE_AMOUNT) ||
							(IrqInfo.Type < 0)) {
					Ret = -EFAULT;
					LOG_ERR("invalid type(%d)",
								IrqInfo.Type);
					goto EXIT;
				}
				if ((IrqInfo.UserKey >= IRQ_USER_NUM_MAX) ||
							(IrqInfo.UserKey < 0)) {
					LOG_ERR(
					"invalid userKey(%d), max(%d), force userkey = 0\n",
						IrqInfo.UserKey,
						IRQ_USER_NUM_MAX);
						IrqInfo.UserKey = 0;
				}
				LOG_INF(
				"IRQ clear(%d), type(%d), userKey(%d), timeout(%d), status(0x%x)\n",
					IrqInfo.Clear, IrqInfo.Type,
					IrqInfo.UserKey, IrqInfo.Timeout,
					IrqInfo.Status);
				IrqInfo.ProcessID = pUserInfo->Pid;
				Ret = DPE_WaitIrq(&IrqInfo);
				if (copy_to_user((void *)Param, &IrqInfo,
				sizeof(struct DPE_WAIT_IRQ_STRUCT)) != 0) {
					LOG_ERR("copy_to_user failed\n");
					Ret = -EFAULT;
				}
			} else {
				LOG_ERR("DPE_WAIT_IRQ copy_from_user failed");
				Ret = -EFAULT;
			}
			break;
		}
	case DPE_CLEAR_IRQ:
		{
			if (copy_from_user(&ClearIrq, (void *)Param,
				sizeof(struct DPE_CLEAR_IRQ_STRUCT)) == 0) {
				LOG_DBG("DPE_CLEAR_IRQ Type(%d)",
								ClearIrq.Type);
				if ((ClearIrq.Type >= DPE_IRQ_TYPE_AMOUNT) ||
							(ClearIrq.Type < 0)) {
					Ret = -EFAULT;
					LOG_ERR("invalid type(%d)",
								ClearIrq.Type);
					goto EXIT;
				}
				/*  */
				if ((ClearIrq.UserKey >= IRQ_USER_NUM_MAX)
				    || (ClearIrq.UserKey < 0)) {
					LOG_ERR("errUserEnum(%d)",
							ClearIrq.UserKey);
					Ret = -EFAULT;
					goto EXIT;
				}
				LOG_DBG(
				"DPE_CLEAR_IRQ:Type(%d),Status(0x%08X),IrqStatus(0x%08X)\n",
					ClearIrq.Type, ClearIrq.Status,
					DPEInfo.IrqInfo.Status[ClearIrq.Type]);
				spin_lock_irqsave(
				&(DPEInfo.SpinLockIrq[ClearIrq.Type]), flags);
				DPEInfo.IrqInfo.Status[ClearIrq.Type] &=
							(~ClearIrq.Status);
				spin_unlock_irqrestore(
				&(DPEInfo.SpinLockIrq[ClearIrq.Type]), flags);
			} else {
				LOG_ERR(
				"DPE_CLEAR_IRQ copy_from_user failed\n");
				Ret = -EFAULT;
			}
			break;
		}
	case DPE_ENQNUE_NUM:
		{
			if (copy_from_user(&enqueNum, (void *)Param,
							sizeof(int)) == 0) {
				if (DPE_REQUEST_STATE_EMPTY ==
				    g_DPE_ReqRing.DPEReq_Struct[
							g_DPE_ReqRing.WriteIdx].
				    State) {
					spin_lock_irqsave(
					&(DPEInfo.SpinLockIrq[
						DPE_IRQ_TYPE_INT_DVP_ST]),
									flags);
					g_DPE_ReqRing.DPEReq_Struct[
						g_DPE_ReqRing.WriteIdx].
							processID =
								pUserInfo->Pid;
					g_DPE_ReqRing.DPEReq_Struct[
						g_DPE_ReqRing.WriteIdx].
							enqueReqNum = enqueNum;
					spin_unlock_irqrestore(
					&(DPEInfo.SpinLockIrq[
					DPE_IRQ_TYPE_INT_DVP_ST]), flags);
					if (enqueNum >
					_SUPPORT_MAX_DPE_FRAME_REQUEST_) {
						LOG_ERR(
						"DPE Enque Num is bigger than enqueNum:%d\n",
						     enqueNum);
					}
					LOG_DBG(
					"DPE_ENQNUE_NUM:%d\n", enqueNum);
				} else {
					LOG_ERR(
					"WFME Enque request state is not empty:%d, writeIdx:%d, readIdx:%d\n",
					     g_DPE_ReqRing.DPEReq_Struct[
						g_DPE_ReqRing.WriteIdx].State,
						g_DPE_ReqRing.WriteIdx,
						g_DPE_ReqRing.ReadIdx);
				}
			} else {
				LOG_ERR(
				"DPE_EQNUE_NUM copy_from_user failed\n");
				Ret = -EFAULT;
			}
			break;
		}
		/* struct DPE_Config */
	case DPE_ENQUE:
		{
			if (copy_from_user(&dpe_DpeConfig, (void *)Param,
					sizeof(struct DPE_Config)) == 0) {
				/* LOG_DBG(
				 * "DPE_CLEAR_IRQ:Type(%d),Status(0x%08X),
				 * IrqStatus(0x%08X)",
				 * ClearIrq.Type, ClearIrq.Status,
				 * DPEInfo.IrqInfo.Status[ClearIrq.Type]);
				 */
				spin_lock_irqsave(
				&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
						  flags);
				if ((DPE_REQUEST_STATE_EMPTY ==
				    g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].State)
				    && (g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].FrameWRIdx <
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].enqueReqNum)) {
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].DpeFrameStatus[
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].FrameWRIdx] =
					DPE_FRAME_STATUS_ENQUE;
					memcpy(&g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].DpeFrameConfig[
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].FrameWRIdx++],
					&dpe_DpeConfig,
					sizeof(struct DPE_Config));
					if (g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].FrameWRIdx ==
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.WriteIdx].enqueReqNum) {
						g_DPE_ReqRing.DPEReq_Struct[
						g_DPE_ReqRing.WriteIdx].State =
						DPE_REQUEST_STATE_PENDING;
						g_DPE_ReqRing.WriteIdx =
						    (g_DPE_ReqRing.WriteIdx +
						     1) %
					_SUPPORT_MAX_DPE_REQUEST_RING_SIZE_;
						LOG_DBG("DPE enque done!!\n");
					} else {
						LOG_DBG("DPE enque frame!!\n");
					}
				} else {
					LOG_ERR(
					"No Buffer! WriteIdx(%d), Stat(%d), FrameWRIdx(%d), enqueReqNum(%d)\n",
					     g_DPE_ReqRing.WriteIdx,
					     g_DPE_ReqRing.DPEReq_Struct[
						g_DPE_ReqRing.WriteIdx].State,
					     g_DPE_ReqRing.DPEReq_Struct[
						g_DPE_ReqRing.WriteIdx].
								FrameWRIdx,
					     g_DPE_ReqRing.DPEReq_Struct[
							g_DPE_ReqRing.WriteIdx].
								enqueReqNum);
				}
				spin_unlock_irqrestore(
				&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
						       flags);
				LOG_ERR("ConfigDPE Not Support\n");
			} else {
				LOG_ERR("DPE_ENQUE copy_from_user failed\n");
				Ret = -EFAULT;
			}
			break;
		}
	case DPE_ENQUE_REQ:
		{
			if (dpe_get_req_header(&dpe_DpeReq, (void __user *)Param) == 0) {
				if (!dpe_DpeReq.m_ReqNum || dpe_DpeReq.m_ReqNum > MAX_FRAMES_PER_REQUEST ||
				    !dpe_DpeReq.m_pDpeConfig) { Ret = -EINVAL; goto EXIT; }
				ioctl_configs = kcalloc(dpe_DpeReq.m_ReqNum, sizeof(*ioctl_configs), GFP_KERNEL);
				if (!ioctl_configs) { Ret = -ENOMEM; goto EXIT; }

				LOG_DBG("DPE_ENQNUE_NUM:%d, pid:%d\n",
					dpe_DpeReq.m_ReqNum, pUserInfo->Pid);
				if (dpe_DpeReq.m_ReqNum >
					_SUPPORT_MAX_DPE_FRAME_REQUEST_) {
					LOG_ERR(
					"DPE Enque Num is bigger than enqueNum:%d\n",
						dpe_DpeReq.m_ReqNum);
					Ret = -EFAULT;
					goto EXIT;
				}
				if (copy_from_user
				    (ioctl_configs,
				     (void *)dpe_DpeReq.m_pDpeConfig,
				     dpe_DpeReq.m_ReqNum *
					sizeof(struct DPE_Config)) != 0) {
					LOG_ERR(
					"copy DPEConfig from request fail!!\n");
					Ret = -EFAULT;
					goto EXIT;
				}
				//!mutex_lock(&gDpeMutex);
				mutex_lock(&gDVSMutex);

				enqnum = -EINVAL;
				kDpeReq.m_ReqNum = dpe_DpeReq.m_ReqNum;
				kDpeReq.m_pDpeConfig =
					ioctl_configs;
				LOG_INF("[DPE ioctl DPE ENQUE REQ] Dpe_engineSelect = %d\n",
				kDpeReq.m_pDpeConfig->Dpe_engineSelect);

				if (kDpeReq.m_pDpeConfig->Dpe_engineSelect == MODE_DVS_ONLY) {
					enqnum = dpe_enque_request(&dpe_reqs_dvs,
					kDpeReq.m_ReqNum, &kDpeReq, pUserInfo->Pid);
				}
				if ((kDpeReq.m_pDpeConfig->Dpe_engineSelect == MODE_DVP_ONLY) ||
					(kDpeReq.m_pDpeConfig->Dpe_engineSelect ==
					MODE_DVS_DVP_BOTH)) {
					enqnum = dpe_enque_request(&dpe_reqs_dvp,
					kDpeReq.m_ReqNum, &kDpeReq, pUserInfo->Pid);
				}

				if (enqnum < 0) { Ret = enqnum; mutex_unlock(&gDVSMutex); goto EXIT; }
				LOG_DBG("Config DPE Request!!\n");
				/* Use a workqueue to set CMDQ to prevent
				 * HW CMDQ request consuming speed from being
				 * faster than SW frame-queue update speed.
				 */
				if (kDpeReq.m_pDpeConfig->Dpe_engineSelect == MODE_DVS_ONLY) {
					req_temp = dpe_request_running(&dpe_reqs_dvs);
					//LOG_INF("[DPE ioctl]dpe_request_running star = %d\n",
					//req_temp);
					if (!req_temp) {
						//if (!dpe_request_running(&dpe_reqs)) {
						LOG_INF("[dvs]direct request_handler\n");
						dpe_request_handler(&dpe_reqs_dvs,
						&(DPEInfo.SpinLockIrq[
						DPE_IRQ_TYPE_INT_DVP_ST]));
					}
				}

				if ((kDpeReq.m_pDpeConfig->Dpe_engineSelect == MODE_DVP_ONLY) ||
					(kDpeReq.m_pDpeConfig->Dpe_engineSelect ==
					MODE_DVS_DVP_BOTH)) {
					req_temp = dpe_request_running(&dpe_reqs_dvp);
					LOG_INF("[DPE ioctl]dpe_request_running star = %d\n",
					req_temp);
					if (!req_temp) {
						LOG_INF("[DPE ioctl DVP]direct request_handler\n");
						dpe_request_handler(&dpe_reqs_dvp,
						&(DPEInfo.SpinLockIrq[
						DPE_IRQ_TYPE_INT_DVP_ST]));
					}
				}
				mutex_unlock(&gDVSMutex);
				//mutex_unlock(&gDpeMutex);
			} else {
				LOG_ERR(
				"DPE_ENQUE_REQ copy_from_user failed\n");
				Ret = -EFAULT;
			}
			break;
		}
	case DPE_DEQUE_NUM:
		{
			if (DPE_REQUEST_STATE_FINISHED ==
			    g_DPE_ReqRing.DPEReq_Struct[
				g_DPE_ReqRing.ReadIdx].State) {
				dequeNum =
				    g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].enqueReqNum;
				LOG_DBG("DPE_DEQUE_NUM(%d)\n", dequeNum);
			} else {
				dequeNum = 0;
				LOG_ERR(
				"DEQUE_NUM:No Buffer: ReadIdx(%d) State(%d) RrameRDIdx(%d) enqueReqNum(%d)\n",
				     g_DPE_ReqRing.ReadIdx,
				     g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].State,
				     g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx,
				     g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].enqueReqNum);
			}
			if (copy_to_user((void *)Param, &dequeNum,
						sizeof(unsigned int)) != 0) {
				LOG_ERR("DPE_DEQUE_NUM copy_to_user failed\n");
				Ret = -EFAULT;
			}
			break;
		}
	case DPE_DEQUE:
		{
			spin_lock_irqsave(
			&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]), flags);
			if ((DPE_REQUEST_STATE_FINISHED ==
			     g_DPE_ReqRing.DPEReq_Struct[
				 g_DPE_ReqRing.ReadIdx].State)
			    && (g_DPE_ReqRing.DPEReq_Struct[
				g_DPE_ReqRing.ReadIdx].RrameRDIdx <
				g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].enqueReqNum)) {
				if (DPE_FRAME_STATUS_FINISHED ==
				    g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].DpeFrameStatus[
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx]) {
					memcpy(&dpe_DpeConfig,
					&g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].DpeFrameConfig[
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx],
					sizeof(struct DPE_Config));
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].DpeFrameStatus[
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx++] =
					DPE_FRAME_STATUS_EMPTY;
				}
				if (g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx ==
				    g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].enqueReqNum) {
					g_DPE_ReqRing.DPEReq_Struct[
						g_DPE_ReqRing.ReadIdx].State =
						DPE_REQUEST_STATE_EMPTY;
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].FrameWRIdx = 0;
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx = 0;
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].enqueReqNum = 0;
					g_DPE_ReqRing.ReadIdx =
					(g_DPE_ReqRing.ReadIdx + 1) %
					_SUPPORT_MAX_DPE_REQUEST_RING_SIZE_;
					LOG_DBG("DPE ReadIdx(%d)\n",
							g_DPE_ReqRing.ReadIdx);
				}
				spin_unlock_irqrestore(
				&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
						       flags);
				if (copy_to_user
				    ((void *)Param,
				     &g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].DpeFrameConfig[
					g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx],
					sizeof(struct DPE_Config)) != 0) {
					LOG_ERR(
					"DPE_DEQUE copy_to_user fail\n");
					Ret = -EFAULT;
				}
			} else {
				spin_unlock_irqrestore(
				&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
						       flags);
				LOG_ERR("DPE_DEQUE No Buf: (%d)(%d)(%d)(%d)\n",
				    g_DPE_ReqRing.ReadIdx,
				    g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].State,
				    g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].RrameRDIdx,
				    g_DPE_ReqRing.DPEReq_Struct[
					g_DPE_ReqRing.ReadIdx].enqueReqNum);
			}
			break;
		}
	case DPE_DEQUE_REQ:
		{
			if (dpe_get_req_header(&dpe_DpeReq, (void __user *)Param) == 0) {
				if (!dpe_DpeReq.m_ReqNum || dpe_DpeReq.m_ReqNum > MAX_FRAMES_PER_REQUEST ||
				    !dpe_DpeReq.m_pDpeConfig) { Ret = -EINVAL; goto EXIT; }
				ioctl_configs = kcalloc(dpe_DpeReq.m_ReqNum, sizeof(*ioctl_configs), GFP_KERNEL);
				if (!ioctl_configs) { Ret = -ENOMEM; goto EXIT; }

				if (copy_from_user(ioctl_configs, dpe_DpeReq.m_pDpeConfig,
					dpe_DpeReq.m_ReqNum * sizeof(*ioctl_configs))) { Ret = -EFAULT; goto EXIT; }
				//mutex_lock(&gDpeDequeMutex);
				mutex_lock(&gDVSMutex);

				Ret = -EINVAL;
				kDpeReq.m_ReqNum = dpe_DpeReq.m_ReqNum;
				kDpeReq.m_pDpeConfig =
					ioctl_configs;

				if (kDpeReq.m_pDpeConfig->Dpe_engineSelect == MODE_DVS_ONLY)
					Ret = dpe_deque_request(&dpe_reqs_dvs,
					&kDpeReq.m_ReqNum, &kDpeReq);


				if ((kDpeReq.m_pDpeConfig->Dpe_engineSelect == MODE_DVP_ONLY) ||
					(kDpeReq.m_pDpeConfig->Dpe_engineSelect == 0))
					Ret = dpe_deque_request(&dpe_reqs_dvp,
					&kDpeReq.m_ReqNum, &kDpeReq);



				if (Ret) { mutex_unlock(&gDVSMutex); goto EXIT; }
				dequeNum = kDpeReq.m_ReqNum;
				dpe_DpeReq.m_ReqNum = dequeNum;

				//mutex_unlock(&gDpeDequeMutex);
				mutex_unlock(&gDVSMutex);
				if (dpe_DpeReq.m_pDpeConfig == NULL) {
					LOG_ERR("NULL ptr:DpeReq.m_pDpeConfig");
					Ret = -EFAULT;
					goto EXIT;
				}
				if (copy_to_user
					((void *)dpe_DpeReq.m_pDpeConfig,
				  &ioctl_configs[0],
				  dequeNum *
					sizeof(struct DPE_Config)) != 0) {
					LOG_ERR
					("DPE_DEQUE_REQ frmcfg failed\n");
					Ret = -EFAULT;
				}
				if (dpe_put_req_header((void __user *)Param, &dpe_DpeReq) != 0) {
					LOG_ERR("DPE_DEQUE_REQ DpeReq fail\n");
					Ret = -EFAULT;
				}
			} else {
				LOG_ERR("DPE_CMD_DPE_DEQUE_REQ failed\n");
				Ret = -EFAULT;
			}
			break;
		}
	default:
		{
			LOG_ERR("Unknown Cmd(%d)", Cmd);
			LOG_ERR("Cmd(%d),Dir(%d),Typ(%d),Nr(%d),Size(%d)\n",
				Cmd, _IOC_DIR(Cmd),
				_IOC_TYPE(Cmd), _IOC_NR(Cmd), _IOC_SIZE(Cmd));
			Ret = -EPERM;
			break;
		}
	}
	/*  */
EXIT:
	if (Ret != 0) {
		LOG_ERR("Fail Cmd(%d), Pid(%d), (proc, pid, tgid)=(%s, %d, %d)",
			Cmd,
			pUserInfo->Pid,
			current->comm,
			current->pid,
			current->tgid);
	}
	/*  */
	return Ret;
}
#if IS_ENABLED(CONFIG_COMPAT)
/*******************************************************************************
 *
 ******************************************************************************/
static long DPE_ioctl_compat(struct file *filp, unsigned int cmd,
                              unsigned long arg)
{
	switch (cmd) {
	case COMPAT_DPE_READ_REGISTER: cmd = DPE_READ_REGISTER; break;
	case COMPAT_DPE_WRITE_REGISTER: cmd = DPE_WRITE_REGISTER; break;
	case COMPAT_DPE_ENQUE_REQ: cmd = DPE_ENQUE_REQ; break;
	case COMPAT_DPE_DEQUE_REQ: cmd = DPE_DEQUE_REQ; break;
	default: break;
	}
	return DPE_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif
/*******************************************************************************
 *
 ******************************************************************************/
static int dpe_start_request_pair(void)
{
	int ret;
	ret = dpe_register_requests(&dpe_reqs_dvs, sizeof(struct DPE_Config));
	if (ret) return -ENOMEM;
	dpe_set_engine_ops(&dpe_reqs_dvs, &dpe_ops);
	ret = dpe_register_requests(&dpe_reqs_dvp, sizeof(struct DPE_Config));
	if (ret) {
		dpe_unregister_requests(&dpe_reqs_dvs);
		return -ENOMEM;
	}
	dpe_set_engine_ops(&dpe_reqs_dvp, &dpe_ops);
	return 0;
}
static int dpe_stop_request_pair(void)
{
	WRITE_ONCE(dpe_closing, true);
	/* Stop CPU IRQ consumers and producers before stopping/resetting hardware.
	 * GCE hardware event delivery does not depend on CPU IRQ enable state. */
	disable_irq(dpe_slots[0].irq);
	disable_irq(dpe_slots[1].irq);
	cancel_work_sync(&DPEInfo.ScheduleDpeWork);
	cancel_work_sync(&DPEInfo.DVP_ScheduleDpeWork);
	cmdq_mbox_stop(dpe_clt);
	DPE_Reset();
	if (!wait_event_timeout(dpe_packet_wait,
		!atomic_read(&dpe_pending_packets), msecs_to_jiffies(2000))) {
		/* Retain maps, request storage and power if callback drain cannot be
		 * established. Reboot is required; never free possibly-live DMA. */
		WRITE_ONCE(dpe_faulted, true);
		return -ETIMEDOUT;
	}
	return 0;
}
static signed int DPE_open(struct inode *pInode, struct file *pFile)
{
	signed int Ret = 0;
	unsigned int i, j;
	/*int q = 0, p = 0;*/
	struct DPE_USER_INFO_STRUCT *pUserInfo;

	if (!smp_load_acquire(&dpe_ready)) return -EAGAIN;
	if ((wire_test_only || root_test_only) && !capable(CAP_SYS_ADMIN)) return -EACCES;
	mutex_lock(&dpe_users_lock);
	if (READ_ONCE(dpe_faulted)) { mutex_unlock(&dpe_users_lock); return -EIO; }

	LOG_INF("- E. UserCount: %d.", DPEInfo.UserCount);

	/*  */
	spin_lock(&(DPEInfo.SpinLockDPERef));
	pFile->private_data = NULL;
	pFile->private_data = kmalloc(sizeof(struct DPE_USER_INFO_STRUCT),
								GFP_ATOMIC);
	if (pFile->private_data == NULL) {
		LOG_DBG("kmalloc failed, (proc, pid, tgid)=(%s, %d, %d)",
								current->comm,
						current->pid, current->tgid);
		Ret = -ENOMEM;
		spin_unlock(&DPEInfo.SpinLockDPERef);
		goto EXIT;
	} else {
		pUserInfo = (struct DPE_USER_INFO_STRUCT *) pFile->private_data;
		pUserInfo->Pid = DPEInfo.UserCount;
		pUserInfo->Tid = current->tgid;
	}
	/*  */
	if (DPEInfo.UserCount > 0) {
		DPEInfo.UserCount++;
		spin_unlock(&(DPEInfo.SpinLockDPERef));
		LOG_DBG("Cur Usr(%d), (proc, pid, tgid)=(%s, %d, %d), exist",
			DPEInfo.UserCount, current->comm, current->pid,
								current->tgid);
		goto EXIT;
	} else {
		DPEInfo.UserCount++;
		spin_unlock(&(DPEInfo.SpinLockDPERef));
		/* do wait queue head init when re-enter in camera */
		/*  */
		for (i = 0; i < _SUPPORT_MAX_DPE_REQUEST_RING_SIZE_; i++) {
			/* DPE */
			g_DPE_ReqRing.DPEReq_Struct[i].processID = 0x0;
			g_DPE_ReqRing.DPEReq_Struct[i].callerID = 0x0;
			g_DPE_ReqRing.DPEReq_Struct[i].enqueReqNum = 0x0;
			/* g_DPE_ReqRing.DPEReq_Struct[i].enqueIdx = 0x0; */
			g_DPE_ReqRing.DPEReq_Struct[i].State =
				DPE_REQUEST_STATE_EMPTY;
			g_DPE_ReqRing.DPEReq_Struct[i].FrameWRIdx = 0x0;
			g_DPE_ReqRing.DPEReq_Struct[i].RrameRDIdx = 0x0;
			for (j = 0; j < _SUPPORT_MAX_DPE_FRAME_REQUEST_; j++) {
				g_DPE_ReqRing.DPEReq_Struct[i].DpeFrameStatus[
					j] = DPE_FRAME_STATUS_EMPTY;
			}
		}
		g_DPE_ReqRing.WriteIdx = 0x0;
		g_DPE_ReqRing.ReadIdx = 0x0;
		g_DPE_ReqRing.HWProcessIdx = 0x0;
		for (i = 0; i < DPE_IRQ_TYPE_AMOUNT; i++)
			DPEInfo.IrqInfo.Status[i] = 0;
		for (i = 0; i < _SUPPORT_MAX_DPE_FRAME_REQUEST_; i++)
			DPEInfo.ProcessID[i] = 0;
		DPEInfo.WriteReqIdx = 0;
		DPEInfo.ReadReqIdx = 0;
		/* DPEInfo.IrqInfo.DpeIrqCnt = 0; */
		for (i = 0; i < IRQ_USER_NUM_MAX; i++)
			DPEInfo.IrqInfo.DpeIrqCnt[i] = 0;
		/*  */
		Ret = dpe_start_request_pair();
		if (Ret) {
			spin_lock(&DPEInfo.SpinLockDPERef);
			DPEInfo.UserCount--;
			spin_unlock(&DPEInfo.SpinLockDPERef);
			kfree(pFile->private_data);
			pFile->private_data = NULL;
			goto EXIT;
		}
		//
		LOG_DBG("Cur Usr(%d), (proc, pid, tgid)=(%s, %d, %d), 1st user",
			DPEInfo.UserCount, current->comm, current->pid,
								current->tgid);
	}

	spin_lock(&(DPEInfo.SpinLockFD));
	DVP_is16BitMode = 0;
	DVS_only_en = 0;
	DVS_Num = 0;
	DVP_only_en = 0;
	DVP_Num = 0;

	for (i = 0 ; i < 10 ; i++) {
		get_dvs_iova[i] = 0;
		get_dvp_iova[i] = 0;
	}
	spin_unlock(&(DPEInfo.SpinLockFD));

	/* Enable clock */
	Ret = DPE_EnableClock(MTRUE);
	if (Ret) {
		dpe_unregister_requests(&dpe_reqs_dvs);
		dpe_unregister_requests(&dpe_reqs_dvp);
		spin_lock(&DPEInfo.SpinLockDPERef);
		DPEInfo.UserCount--;
		spin_unlock(&DPEInfo.SpinLockDPERef);
		kfree(pFile->private_data);
		pFile->private_data = NULL;
		goto EXIT;
	}
	cmdq_mbox_enable(dpe_clt->chan);
	WRITE_ONCE(dpe_closing, false);
	enable_irq(dpe_slots[0].irq);
	enable_irq(dpe_slots[1].irq);
	g_SuspendCnt = 0;
	LOG_INF("DPE open g_u4EnableClockCount: %d", g_u4EnableClockCount);
	/*  */
/*#define KERNEL_LOG*/
#ifdef KERNEL_LOG
    /* In EP, Add DPE_DBG_WRITE_REG for debug. Should remove it after EP */
	DPEInfo.DebugMask = (DPE_DBG_INT | DPE_DBG_DBGLOG | DPE_DBG_WRITE_REG);
#endif
EXIT:
	LOG_INF("- X. Ret: %d. UserCount: %d.", Ret, DPEInfo.UserCount);
	mutex_unlock(&dpe_users_lock);
	return Ret;
}
/******************************************************************************
 *
 ******************************************************************************/
static signed int DPE_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	mutex_lock(&dpe_users_lock);
	if (!file->private_data) goto out;
	kfree(file->private_data);
	file->private_data = NULL;
	spin_lock(&DPEInfo.SpinLockDPERef);
	if (WARN_ON_ONCE(!DPEInfo.UserCount)) {
		spin_unlock(&DPEInfo.SpinLockDPERef);
		ret = -EINVAL;
		goto out;
	}
	DPEInfo.UserCount--;
	if (DPEInfo.UserCount) {
		spin_unlock(&DPEInfo.SpinLockDPERef);
		goto out;
	}
	spin_unlock(&DPEInfo.SpinLockDPERef);
	ret = dpe_stop_request_pair();
	if (ret) goto out;
	dpe_unregister_requests(&dpe_reqs_dvs);
	dpe_unregister_requests(&dpe_reqs_dvp);
	cmdq_mbox_disable(dpe_clt->chan);
	ret = DPE_EnableClock(MFALSE);
out:
	mutex_unlock(&dpe_users_lock);
	return ret;
}
/*******************************************************************************/
static dev_t DPEDevNo;
static struct cdev *pDPECharDrv;
static struct class *pDPEClass;
static const struct file_operations DPEFileOper = {
	.owner = THIS_MODULE,
	.open = DPE_open,
	.release = DPE_release,
	/* .flush   = mt_DPE_flush, */
	/* .mmap = DPE_mmap, */
	.unlocked_ioctl = DPE_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = DPE_ioctl_compat,
#endif
};
/*******************************************************************************
 *
 ******************************************************************************/
static inline void DPE_UnregCharDev(void)
{
	LOG_DBG("- E.");
	/*  */
	/* Release char driver */
	if (pDPECharDrv != NULL) {
		cdev_del(pDPECharDrv);
		pDPECharDrv = NULL;
	}
	/*  */
	unregister_chrdev_region(DPEDevNo, 1);
}
/*******************************************************************************
 *
 ******************************************************************************/
static inline signed int DPE_RegCharDev(void)
{
	signed int Ret = 0;
	/*  */
	LOG_DBG("- E.");
	/*  */
	Ret = alloc_chrdev_region(&DPEDevNo, 0, 1, DPE_DEV_NAME);
	if (Ret < 0) {
		LOG_ERR("alloc_chrdev_region failed, %d", Ret);
		return Ret;
	}
	/* Allocate driver */
	pDPECharDrv = cdev_alloc();
	if (pDPECharDrv == NULL) {
		LOG_ERR("cdev_alloc failed");
		Ret = -ENOMEM;
		goto EXIT;
	}
	/* Attath file operation. */
	pDPECharDrv->ops = &DPEFileOper;
	/*  */
	pDPECharDrv->owner = THIS_MODULE;
	/* Add to system */
	Ret = cdev_add(pDPECharDrv, DPEDevNo, 1);
	if (Ret < 0) {
		LOG_ERR("Attath file operation failed, %d", Ret);
		goto EXIT;
	}
	/*  */
EXIT:
	if (Ret < 0) {
		if (pDPECharDrv) {
			kobject_put(&pDPECharDrv->kobj);
			pDPECharDrv = NULL;
		}
		unregister_chrdev_region(DPEDevNo, 1);
	}
	/*  */
	LOG_DBG("- X.");
	return Ret;
}
/*******************************************************************************
 * V4L2
 ******************************************************************************/
static int dpe_fop_open(struct file *filp)
{
	return DPE_open(NULL, filp);
}
static int dpe_fop_release(struct file *file)
{
	DPE_release(NULL, file);
	return 0;
}
unsigned int dpe_fop_poll(struct file *file, poll_table *wait)
{
	//struct DPE_Kernel_Config *pDpeConfig;
	//struct DPE_Kernel_Config DpeConfig;
	struct DPE_USER_INFO_STRUCT *pUserInfo;
	unsigned int buf_rdy;
	unsigned long flags;
	unsigned int p;


	#ifdef DPE_debug_log_en
	LOG_INF("DPE Poll\n");
	#endif
	//DPE_DumpUserSpaceReg(pDpeConfig);
	//LOG_INF("DPE_DumpReg star dpe_fop_poll!\n");
	//DPE_DumpReg();
	if (READ_ONCE(dpe_faulted)) return EPOLLERR;
	pUserInfo = (struct DPE_USER_INFO_STRUCT *) (file->private_data);
	poll_wait(file, &DPEInfo.WaitQueueHead, wait);
	buf_rdy = DPE_GetIRQState(DPE_IRQ_TYPE_INT_DVP_ST,
				0x0,
				DPE_INT_ST, DPE_PROCESS_ID_DPE,
				pUserInfo->Pid);
	p = pUserInfo->Pid % IRQ_USER_NUM_MAX;
	LOG_INF("buf_rdy = %d\n", buf_rdy);
	if (buf_rdy) {
		spin_lock_irqsave
		(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]), flags);
		DPEInfo.IrqInfo.DpeIrqCnt[p]--;
if (DPEInfo.IrqInfo.DpeIrqCnt[p] == 0)
	DPEInfo.IrqInfo.Status[DPE_IRQ_TYPE_INT_DVP_ST] &= (~DPE_INT_ST);
spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]), flags);
		return POLLIN | POLLRDNORM;
	} else
		return 0;
}
static const struct v4l2_file_operations vid_fops = {
	.owner          = THIS_MODULE,
	.open           = dpe_fop_open,
	.release        = dpe_fop_release,
	.poll           = dpe_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	/* TODO */
	//.mmap           = vb2_fop_mmap,
	//.read           = vb2_fop_read,
	//.write          = vb2_fop_write,
};
#include "dpe_native_wire.h"

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	/*struct video_device *vdev = video_devdata(file);*/
	int ret;
	struct DPE_USER_INFO_STRUCT *pUserInfo;
	struct DPE_Request ureq;
	struct DPE_Request kreq;
	/* size of cfgs = 3 owing to call stact limitation*/
	struct DPE_Config *cfgs __free(kfree) = NULL;
	struct DPE_Config *pcfgs;
	//unsigned long flags;
	unsigned int f;
	int temp_req;
	//unsigned int p_cnt;
	//pid_t ProcessID;

	//int tmep_cnt;
	#ifdef DPE_debug_log_en
	LOG_INF("[%s]buf address/len = 0x%lx/0x%x\n",
		__func__, p->m.userptr,  p->length);
	#endif
	pUserInfo = (struct DPE_USER_INFO_STRUCT *) (file->private_data);
	if (READ_ONCE(dpe_faulted)) return -EIO;
	ret = dpe_get_req_header(&ureq, (void __user *)p->m.userptr);
	if (ret) goto EXIT;
	if (!ureq.m_ReqNum || ureq.m_ReqNum > 3 || !ureq.m_pDpeConfig) {
		ret = -EINVAL; goto EXIT;
	}
	cfgs = kcalloc(3, sizeof(*cfgs), GFP_KERNEL);
	if (!cfgs) { ret = -ENOMEM; goto EXIT; }
	ret = dpe_get_v4l2_configs(cfgs, ureq.m_pDpeConfig, ureq.m_ReqNum);
	if (ret) goto EXIT;
	if (wire_test_only)
		return dpe_put_v4l2_configs(ureq.m_pDpeConfig, cfgs, ureq.m_ReqNum);
	for (f = 0; f < ureq.m_ReqNum; f++) {
		if (cfgs[f].Dpe_DVSSettings.is_pd_mode) {
			pcfgs = &cfgs[f];
			Get_Tile_Info(pcfgs);
		/* Expansion validated by the shared enqueue callback. */
		}
	}
	kreq.m_pDpeConfig = cfgs;
	kreq.m_ReqNum = ureq.m_ReqNum;
	#ifdef DPE_debug_log_en
	LOG_INF("[vidioc qbuf] Dpe engineSelect = %d\n",
	cfgs[0].Dpe_engineSelect);
	#endif

	//kreq.m_ReqNum = ureq.m_ReqNum;
	//mutex_lock(&gDpeMutex);	/* Protect the Multi Process */
	ret = -EINVAL;
	mutex_lock(&gDVSMutex);
	//spin_lock_irqsave(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
	//		       flags);
	if (cfgs[0].Dpe_engineSelect == MODE_DVS_ONLY)
		ret = dpe_enque_request(&dpe_reqs_dvs, kreq.m_ReqNum, &kreq, pUserInfo->Pid);


	if ((cfgs[0].Dpe_engineSelect == MODE_DVP_ONLY) ||
			(cfgs[0].Dpe_engineSelect == MODE_DVS_DVP_BOTH))
		ret = dpe_enque_request(&dpe_reqs_dvp, kreq.m_ReqNum, &kreq, pUserInfo->Pid);


	//spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
	//		       flags);
	if (ret < 0) { mutex_unlock(&gDVSMutex); goto EXIT; }
	ret = 0;
	/* Use a workqueue to set CMDQ to prevent HW CMDQ request
	 *  consuming speed from being faster than SW frame-queue update speed.
	 */

	if (cfgs[0].Dpe_engineSelect == MODE_DVS_ONLY) {
		temp_req = dpe_request_running(&dpe_reqs_dvs);
		#ifdef DPE_debug_log_en
		LOG_INF("[vidioc qbuf]dpe_request_running stat = %d\n", temp_req);
		#endif
		if (!temp_req) {
			//if (!dpe_request_running(&dpe_reqs)) {
			//LOG_INF("[vidioc_qbuf]direct request_handler\n");
			dpe_request_handler(&dpe_reqs_dvs,
			&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));
		}
	}
	if ((cfgs[0].Dpe_engineSelect == MODE_DVP_ONLY) ||
		(cfgs[0].Dpe_engineSelect == MODE_DVS_DVP_BOTH)) {
		temp_req = dpe_request_running(&dpe_reqs_dvp);
		#ifdef DPE_debug_log_en
		LOG_INF("[vidioc qbuf]dpe_request_running stat = %d\n", temp_req);
		#endif
		if (!temp_req) {
			//if (!dpe_request_running(&dpe_reqs)) {
			//LOG_INF("[vidioc_qbuf]direct request_handler\n");
			dpe_request_handler(&dpe_reqs_dvp,
			&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));
		}
	}
	//mutex_unlock(&gDpeMutex);
	mutex_unlock(&gDVSMutex);

EXIT:
	return ret;
}
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	/*struct video_device *vdev = video_devdata(file);*/
	signed int Ret = 0;
	struct DPE_Request ureq;
	struct DPE_Request kreq;
	/* size of cfgs = 3 owing to call stact limitation*/
	struct DPE_Config *cfgs __free(kfree) = NULL;
	//unsigned long flags;
	//unsigned int m_real_ReqNum;

	//struct DPE_Config *pDpeConfig;
	#ifdef DPE_debug_log_en
	LOG_INF("DPE_DumpReg  star\n");
	DPE_DumpReg();//!test
	LOG_INF("DPE_DumpReg end\n");
	#endif
	//LOG_INF("[%s]buf address/len = 0x%lx/0x%x, ureq =0x%x\n",
	//__func__, p->m.userptr,  p->length, sizeof(ureq));

	if (READ_ONCE(dpe_faulted)) return -EIO;
	Ret = dpe_get_req_header(&ureq, (void __user *)p->m.userptr);
	if (Ret) goto EXIT;
	if (!ureq.m_ReqNum || ureq.m_ReqNum > 3 || !ureq.m_pDpeConfig) {
		Ret = -EINVAL; goto EXIT;
	}
	cfgs = kcalloc(3, sizeof(*cfgs), GFP_KERNEL);
	if (!cfgs) { Ret = -ENOMEM; goto EXIT; }
	Ret = dpe_get_v4l2_configs(cfgs, ureq.m_pDpeConfig, ureq.m_ReqNum);
	if (Ret) goto EXIT;

	if (Ret == 0) {

		//!mutex_lock(&gDpeDequeMutex);
		Ret = -EINVAL;
		mutex_lock(&gDVSMutex);
		//spin_lock_irqsave(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
		//									flags);
		kreq.m_pDpeConfig = cfgs;
		kreq.m_ReqNum = ureq.m_ReqNum;

		#ifdef DPE_debug_log_en
		LOG_INF("[vidioc dqbuf] Dpe_engineSelect = %d\n",
		cfgs[0].Dpe_engineSelect);
		#endif
		if (cfgs[0].Dpe_engineSelect == MODE_DVS_ONLY)
			Ret = dpe_deque_request_wire(&dpe_reqs_dvs, &kreq.m_ReqNum, &kreq);


		if ((cfgs[0].Dpe_engineSelect == MODE_DVP_ONLY) ||
				(cfgs[0].Dpe_engineSelect == MODE_DVS_DVP_BOTH)) {
			Ret = dpe_deque_request_wire(&dpe_reqs_dvp, &kreq.m_ReqNum, &kreq);
		}

		//spin_unlock_irqrestore(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]),
		//		flags);
		//!mutex_unlock(&gDpeDequeMutex);
		mutex_unlock(&gDVSMutex);
		if (Ret) goto EXIT;
		if (kreq.m_ReqNum > 3) { Ret = -EOVERFLOW; goto EXIT; }
		ureq.m_ReqNum = kreq.m_ReqNum;
		if (ureq.m_pDpeConfig == NULL) {
			LOG_ERR("NULL user pointer");
			Ret = -EFAULT;
			goto EXIT;
		}
		if ((Ret = dpe_put_v4l2_configs(ureq.m_pDpeConfig,
		    kreq.m_pDpeConfig, kreq.m_ReqNum)) != 0) {
			LOG_ERR
			    ("DPE_DEQUE_REQ native config copy failed\n");
			goto EXIT;
		}

		if (dpe_put_req_header((void __user *)p->m.userptr, &ureq) != 0) {
			LOG_ERR("DPE_DEQUE_REQ copy_to_user failed\n");
			Ret = -EFAULT;
		}
	} else {
		LOG_ERR("DPE_CMD_DPE_DEQUE_REQ copy_from_user failed\n");
		Ret = -EFAULT;
	}

	LOG_DBG("[%s]buf address/len = 0x%lx/0x%x\n",
		__func__, p->m.userptr,  p->length);
EXIT:
	return Ret;
}
static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct DPE_device *dev = video_drvdata(file);
	/*struct video_device *vdev = video_devdata(file);*/
	strscpy(cap->driver, "dpe", sizeof(cap->driver));
	strscpy(cap->card, "dpe", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", dev->v4l2_dev.name);
	LOG_DBG("[%s]\n", __func__);
cap->device_caps =
	V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}
int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	/*struct DPE_device *dev = video_drvdata(file);*/
	f->fmt.pix.width       = 777;
	f->fmt.pix.height      = 555;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	f->fmt.pix.field       = V4L2_FIELD_INTERLACED;
	f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height;
	LOG_ERR("[%s] sizeimage(%d)\n", __func__, f->fmt.pix.sizeimage);
	return 0;
}
static const struct v4l2_ioctl_ops vid_ioctl_ops = {
	.vidioc_querycap		   = vidioc_querycap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_qbuf			   = vidioc_qbuf,
	.vidioc_dqbuf			   = vidioc_dqbuf,
	//.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	//.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	//.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	//.vidioc_querybuf		   = vb2_ioctl_querybuf,
	//.vidioc_expbuf			   = vb2_ioctl_expbuf,
	//.vidioc_streamon		   = vb2_ioctl_streamon,
	//.vidioc_streamoff		   = vb2_ioctl_streamoff,
};
/* Native callbacks reference global state. Successful binds are retained until
 * reboot, as for the other bring-up engines; manual unbind is suppressed. */
static void dpe_put_larb(void *data)
{
	put_device(data);
}
static void dpe_delete_link(void *data)
{
	device_link_del(data);
}
static void dpe_disable_pm(void *data)
{
	pm_runtime_disable(data);
}
static void dpe_destroy_mailbox(void *data)
{
	cmdq_mbox_destroy(data);
}
static int dpe_get_larb(struct device *dev)
{
	struct device_node *node;
	struct platform_device *pdev;
	struct device_link *link;
	int ret;

	node = of_parse_phandle(dev->of_node, "mediatek,larb", 0);
	if (!node)
		return -EINVAL;
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, dpe_put_larb, &pdev->dev);
	if (ret)
		return ret;
	if (!pdev->dev.driver)
		return -EPROBE_DEFER;
	link = device_link_add(dev, &pdev->dev,
		DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);
	if (!link)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, dpe_delete_link, link);
	if (!ret)
		DPE_devs[0].larb12 = &pdev->dev;
	return ret;
}
static int DPE_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct DPE_device *d;
	struct resource *res;
	struct video_device *vfd;
	struct device *char_dev;
	int idx, ret;
	unsigned int i;
	bool cdev_ready = false, class_ready = false, node_ready = false;
	bool v4l2_ready = false;
	struct clk **clocks[] = { &dpe_clk.VCORE_GALS, &dpe_clk.MAIN_GALS,
		&dpe_clk.IMG_IPE, &dpe_clk.IPE_DPE,
		&dpe_clk.IPE_SMI_LARB12, &dpe_clk.IPE_TOP };
	const char *names[] = { "VCORE_GALS", "MAIN_GALS", "IMG_IPE",
		"IPE_DPE", "IPE_SMI_LARB12", "IPE_TOP" };

	if (of_device_is_compatible(dev->of_node, "mediatek,dvs"))
		idx = 0;
	else if (of_device_is_compatible(dev->of_node, "mediatek,dvp"))
		idx = 1;
	else
		return -ENODEV;
	mutex_lock(&dpe_probe_lock);
	if (dpe_slots[idx].dev) { ret = -EBUSY; goto unlock; }
	/* Native global index zero is DVS; never depend on DT enumeration order. */
	if (idx && !dpe_slots[0].dev) { ret = -EPROBE_DEFER; goto unlock; }
	d = &dpe_slots[idx];
	d->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) { ret = -ENODEV; goto fail; }
	/* Native DT intentionally overlaps the DVS and DVP windows by 0x800.
	 * Preserve of_iomap's shared-window contract, adding managed lifetime;
	 * devm_platform_ioremap_resource would reject the second native alias. */
	d->regs = devm_ioremap(dev, res->start, resource_size(res));
	if (!d->regs) { ret = -ENOMEM; goto fail; }
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret) goto fail;
	dma_set_max_seg_size(dev, UINT_MAX);
	d->irq = platform_get_irq(pdev, 0);
	if (d->irq < 0) { ret = d->irq; goto fail; }
	ret = of_property_read_u32(dev->of_node,
		idx ? "dvp_done_async_shot" : "dvs_done_async_shot",
		idx ? &dvp_event_id : &dvs_event_id);
	if (ret) goto fail;
	/* All locks, log buffers and tasklets were initialized in module init.
	 * Keep both lines masked until the complete pair is ready. */
	ret = devm_request_irq(dev, d->irq,
		idx ? ISP_Irq_DVP : ISP_Irq_DVS, IRQF_NO_AUTOEN,
		dev_name(dev), d);
	if (ret) goto fail;
	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, dpe_disable_pm, dev);
	if (ret) goto fail;
	if (!idx) {
		dpe_clt = cmdq_mbox_create(dev, 0);
		if (IS_ERR_OR_NULL(dpe_clt)) {
			ret = dpe_clt ? PTR_ERR(dpe_clt) : -ENODEV;
			dpe_clt = NULL;
			goto fail;
		}
		ret = devm_add_action_or_reset(dev, dpe_destroy_mailbox, dpe_clt);
		if (ret) { dpe_clt = NULL; goto fail; }
		nr_DPE_devs = 1;
		__module_get(THIS_MODULE);
		ret = 0;
		goto unlock;
	}
	ret = dpe_get_larb(dev);
	if (ret) goto fail;
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		*clocks[i] = devm_clk_get(dev, names[i]);
		if (IS_ERR(*clocks[i])) { ret = PTR_ERR(*clocks[i]); goto fail; }
	}
	gdev = dev;
	dpe_native_scratch = dmam_alloc_coherent(dev, WB_TOTAL_SIZE,
		&dpe_native_scratch_dma, GFP_KERNEL);
	if (!dpe_native_scratch) { ret = -ENOMEM; goto fail; }
	g_dpewb_dvme_int_Buffer_pa = (dma_addr_t *)(uintptr_t)dpe_native_scratch_dma;
	g_dpewb_cost_int_Buffer_pa = (dma_addr_t *)(uintptr_t)
		(dpe_native_scratch_dma + WB_INT_MEDV_SIZE);
	g_dpewb_asfrm_Buffer_pa = (dma_addr_t *)(uintptr_t)
		(dpe_native_scratch_dma + WB_INT_MEDV_SIZE + WB_DCV_L_SIZE);
	g_dpewb_asfrmext_Buffer_pa = (dma_addr_t *)(uintptr_t)
		(dpe_native_scratch_dma + WB_INT_MEDV_SIZE + WB_DCV_L_SIZE + WB_ASFRM_SIZE);
	g_dpewb_wmfhf_Buffer_pa = (dma_addr_t *)(uintptr_t)
		(dpe_native_scratch_dma + WB_INT_MEDV_SIZE + WB_DCV_L_SIZE +
		 WB_ASFRM_SIZE + WB_ASFRMExt_SIZE);
	if (probe_only) {
		nr_DPE_devs = 2;
		__module_get(THIS_MODULE);
		dev_info(dev, "Native ISP70 DPE probe-only pair ready; clients and IRQs disabled\n");
		ret = 0;
		goto unlock;
	}
	/* Prepare V4L2 state before either userspace entry point is published. */
	strscpy(d->v4l2_dev.name, "dpe-000", sizeof(d->v4l2_dev.name));
	ret = v4l2_device_register(dev, &d->v4l2_dev);
	if (ret) goto fail;
	v4l2_ready = true;
	mutex_init(&d->mutex);
	vfd = &d->vid_dpe_dev;
	strscpy(vfd->name, "dpe-vid", sizeof(vfd->name));
	vfd->fops = &vid_fops;
	vfd->ioctl_ops = &vid_ioctl_ops;
	vfd->release = video_device_release_empty;
	vfd->v4l2_dev = &d->v4l2_dev;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	vfd->lock = &d->mutex;
	video_set_drvdata(vfd, d);
	ret = DPE_RegCharDev();
	if (ret) goto fail;
	cdev_ready = true;
	pDPEClass = class_create("DPEdrv");
	if (IS_ERR(pDPEClass)) { ret = PTR_ERR(pDPEClass); pDPEClass = NULL; goto fail; }
	class_ready = true;
	char_dev = device_create(pDPEClass, NULL, DPEDevNo, NULL, DPE_DEV_NAME);
	if (IS_ERR(char_dev)) { ret = PTR_ERR(char_dev); goto fail; }
	node_ready = true;
	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret) goto fail;
	/* No fallible operations after video publication. Open gates prevent any
	 * access through the earlier char node while this setup is incomplete. */
	nr_DPE_devs = 2;
	g_DPE_PMState = 0;
	__module_get(THIS_MODULE);
	smp_store_release(&dpe_ready, true);
	dev_info(dev, "Native ISP70 DPE pair ready\n");
	ret = 0;
	goto unlock;
fail:
	if (node_ready) device_destroy(pDPEClass, DPEDevNo);
	if (class_ready) { class_destroy(pDPEClass); pDPEClass = NULL; }
	if (cdev_ready) DPE_UnregCharDev();
	if (v4l2_ready) v4l2_device_unregister(&d->v4l2_dev);
	if (idx) {
		DPE_devs[0].larb12 = NULL;
		gdev = NULL;
		dpe_native_scratch = NULL;
		dpe_native_scratch_dma = 0;
		g_dpewb_dvme_int_Buffer_pa = NULL;
		g_dpewb_cost_int_Buffer_pa = NULL;
		g_dpewb_asfrm_Buffer_pa = NULL;
		g_dpewb_asfrmext_Buffer_pa = NULL;
		g_dpewb_wmfhf_Buffer_pa = NULL;
	}
	/* devres releases MMIO/IRQ/DMA/clock refs/PM/link in reverse order after
	 * this failed probe returns. No callbacks can run: IRQ lines stay masked. */
	memset(d, 0, sizeof(*d));
unlock:
	mutex_unlock(&dpe_probe_lock);
	return ret;
}
static void DPE_remove(struct platform_device *pdev)
{
	/* Successful binds retain a module reference and suppress user unbind. */
	WARN_ON_ONCE(READ_ONCE(dpe_ready));
}
/*******************************************************************************/
static signed int DPE_suspend(struct platform_device *pDev, pm_message_t Mesg)
{

	return 0;
}
/*******************************************************************************
 *
 ******************************************************************************/
static signed int DPE_resume(struct platform_device *pDev)
{

	return 0;
}
/*---------------------------------------------------------------------------*/
#if IS_ENABLED(CONFIG_PM)
/*---------------------------------------------------------------------------*/
static int dpe_suspend_pm_event(struct notifier_block *notifier,
			unsigned long event, void *unused)
{
	if (!smp_load_acquire(&dpe_ready)) return NOTIFY_DONE;
	if (event == PM_SUSPEND_PREPARE &&
	    (READ_ONCE(DPEInfo.UserCount) || READ_ONCE(g_u4EnableClockCount) ||
	     atomic_read(&dpe_pending_packets))) return NOTIFY_BAD;
	return NOTIFY_DONE;
}

int DPE_pm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);
	LOG_DBG("calling %s()\n", __func__);
	return DPE_suspend(pdev, PMSG_SUSPEND);
}
int DPE_pm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);
	LOG_DBG("calling %s()\n", __func__);
	return DPE_resume(pdev);
}
#if !IS_ENABLED(CONFIG_OF)
/*extern void mt_irq_set_sens(unsigned int irq, unsigned int sens);*/
/*extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);*/
#endif
int DPE_pm_restore_noirq(struct device *device)
{
	pr_debug("calling %s()\n", __func__);
#if !IS_ENABLED(CONFIG_OF)
/*	mt_irq_set_sens(DPE_IRQ_BIT_ID, MT_LEVEL_SENSITIVE);*/
/*	mt_irq_set_polarity(DPE_IRQ_BIT_ID, MT_POLARITY_LOW);*/
#endif
	return 0;
}
/*---------------------------------------------------------------------------*/
#else				/*CONFIG_PM */
/*---------------------------------------------------------------------------*/
#define DPE_pm_suspend NULL
#define DPE_pm_resume  NULL
#define DPE_pm_restore_noirq NULL
/*---------------------------------------------------------------------------*/
#endif				/*CONFIG_PM */
/*---------------------------------------------------------------------------*/
#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id DPE_of_ids[] = {
	//{.compatible = "mediatek,ipesys_config",},
	{.compatible = "mediatek,dvp",},
	{.compatible = "mediatek,dvs",},
	{}
};
#endif
const struct dev_pm_ops DPE_pm_ops = {
	.suspend = DPE_pm_suspend,
	.resume = DPE_pm_resume,
	.freeze = DPE_pm_suspend,
	.thaw = DPE_pm_resume,
	.poweroff = DPE_pm_suspend,
	.restore = DPE_pm_resume,
	.restore_noirq = DPE_pm_restore_noirq,
};
/*******************************************************************************
 *
 ******************************************************************************/
static struct platform_driver DPEDriver = {
	.probe = DPE_probe,
	.remove = DPE_remove,
	.suspend = DPE_suspend,
	.resume = DPE_resume,
	.driver = {
		   .name = DPE_DEV_NAME,
		   .suppress_bind_attrs = true,
		   .owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = DPE_of_ids,
#endif
#if IS_ENABLED(CONFIG_PM)
		   .pm = &DPE_pm_ops,
#endif
		}
};

#if IS_ENABLED(CONFIG_PM)
static struct notifier_block dpe_suspend_pm_notifier_func = {
	.notifier_call = dpe_suspend_pm_event,
	.priority = 0,
};
#endif

static int dpe_dump_read(struct seq_file *m, void *v)
{
/* fix unexpected close clock issue */
#ifdef dpe_dump_read_en
	int i, j;

	if (DPEInfo.UserCount <= 0)
		return 0;
	seq_puts(m, "\n============ dpe dump register============\n");
	seq_puts(m, "DPE Config Info\n");
	for (i = 0x2C; i < 0x8C; i = i + 4) {
		seq_printf(m, "[0x%08X %08X]\n",
				(unsigned int)(DPE_BASE_HW + i),
			   (unsigned int)DPE_RD32(ISP_DPE_BASE + i));
	}
	seq_puts(m, "DPE Debug Info\n");
	for (i = 0x120; i < 0x148; i = i + 4) {
		seq_printf(m, "[0x%08X %08X]\n",
				(unsigned int)(DPE_BASE_HW + i),
			   (unsigned int)DPE_RD32(ISP_DPE_BASE + i));
	}
	seq_puts(m, "DPE Config Info\n");
	for (i = 0x230; i < 0x2D8; i = i + 4) {
		seq_printf(m, "[0x%08X %08X]\n",
				(unsigned int)(DPE_BASE_HW + i),
			   (unsigned int)DPE_RD32(ISP_DPE_BASE + i));
	}
	seq_puts(m, "DPE Debug Info\n");
	for (i = 0x2F4; i < 0x30C; i = i + 4) {
		seq_printf(m, "[0x%08X %08X]\n",
				(unsigned int)(DPE_BASE_HW + i),
			   (unsigned int)DPE_RD32(ISP_DPE_BASE + i));
	}
	seq_puts(m, "\n");
	seq_printf(m, "Dpe Clock Count:%d\n", g_u4EnableClockCount);
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_IRQ_STATUS_HW),
		   (unsigned int)DPE_RD32(DVS_IRQ_STATUS_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_CTRL_STATUS0_HW),
		   (unsigned int)DPE_RD32(DVS_CTRL_STATUS0_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_CTRL_STATUS2_HW),
		   (unsigned int)DPE_RD32(DVS_CTRL_STATUS2_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_CUR_STATUS_HW),
		   (unsigned int)DPE_RD32(DVS_CUR_STATUS_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_FRM_STATUS0_HW),
		   (unsigned int)DPE_RD32(DVS_FRM_STATUS0_REG));
//	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_FRM_STATUS2_HW),
//		   (unsigned int)DPE_RD32(DVS_FRM_STATUS2_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_DRAM_STA0_HW),
		   (unsigned int)DPE_RD32(DVS_DRAM_STA0_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVS_DRAM_STA1_HW),
		   (unsigned int)DPE_RD32(DVS_DRAM_STA1_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVP_IRQ_STATUS_HW),
		   (unsigned int)DPE_RD32(DVP_IRQ_STATUS_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_STATUS0_HW),
		   (unsigned int)DPE_RD32(DVP_CTRL_STATUS0_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_STATUS1_HW),
		   (unsigned int)DPE_RD32(DVP_CTRL_STATUS1_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVP_CTRL_STATUS2_HW),
		   (unsigned int)DPE_RD32(DVP_CTRL_STATUS2_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVP_CUR_STATUS_HW),
		   (unsigned int)DPE_RD32(DVP_CUR_STATUS_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVP_FRM_STATUS0_HW),
		   (unsigned int)DPE_RD32(DVP_FRM_STATUS0_REG));
	seq_printf(m, "[0x%08X %08X]\n", (unsigned int)(DVP_FRM_STATUS2_HW),
		   (unsigned int)DPE_RD32(DVP_FRM_STATUS2_REG));
	seq_printf(m, "DPE:HWProcessIdx:%d, WriteIdx:%d, ReadIdx:%d\n",
		   g_DPE_ReqRing.HWProcessIdx, g_DPE_ReqRing.WriteIdx,
		   g_DPE_ReqRing.ReadIdx);
	for (i = 0; i < _SUPPORT_MAX_DPE_REQUEST_RING_SIZE_; i++) {
		seq_printf(m,
			   "DPE:State:%d, processID:0x%08X, callerID:0x%08X, enqueReqNum:%d, FrameWRIdx:%d, RrameRDIdx:%d\n",
			   g_DPE_ReqRing.DPEReq_Struct[i].State,
			   g_DPE_ReqRing.DPEReq_Struct[i].processID,
			   g_DPE_ReqRing.DPEReq_Struct[i].callerID,
			   g_DPE_ReqRing.DPEReq_Struct[i].enqueReqNum,
			   g_DPE_ReqRing.DPEReq_Struct[i].FrameWRIdx,
			   g_DPE_ReqRing.DPEReq_Struct[i].RrameRDIdx);
		for (j = 0; j < _SUPPORT_MAX_DPE_FRAME_REQUEST_;) {
			seq_printf(m,
				   "DPE:FrmStat[%d]:%d, FrmStat[%d]:%d\n",
				   j,
			g_DPE_ReqRing.DPEReq_Struct[i].DpeFrameStatus[j],
				j + 1,
			g_DPE_ReqRing.DPEReq_Struct[i].DpeFrameStatus[j + 1]);
				j = j + 2;
		}
	}
	seq_puts(m, "\n============ dpe dump debug ============\n");
#endif
	return 0;
}
static int proc_dpe_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, dpe_dump_read, NULL);
}
static const struct file_operations dpe_dump_proc_fops = {
	.owner = THIS_MODULE,
	.open = proc_dpe_dump_open,
	.read = seq_read,
};
static int dpe_reg_read(struct seq_file *m, void *v)
{
/* fix unexpected close clock issue */
#ifdef dpe_dump_read_en
	unsigned int i;

	if (DPEInfo.UserCount <= 0)
		return 0;
	seq_puts(m, "======== read dpe register ========\n");
	for (i = 0x000; i <= 0x1CC; i = i + 4) {
		seq_printf(m, "[0x%08X 0x%08X]\n",
				(unsigned int)(DPE_BASE_HW + i),
				(unsigned int)DPE_RD32(ISP_DPE_BASE + i));
	}
	for (i = 0x800; i <= 0xBDC; i = i + 4) {
		seq_printf(m, "[0x%08X 0x%08X]\n",
				(unsigned int)(DPE_BASE_HW + i),
				(unsigned int)DPE_RD32(ISP_DPE_BASE + i));
	}
#endif
	return 0;
}
static ssize_t dpe_reg_write(struct file *file, const char __user *buffer,
						size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;
	/*char *pEnd;*/
	char addrSzBuf[24];
	char valSzBuf[24];
	char *pszTmp;
	int addr = 0, val = 0;

	if (DPEInfo.UserCount <= 0)
		return 0;
	spin_lock(&(DPEInfo.SpinLockDPE));
	if (g_u4EnableClockCount == 0) {
		spin_unlock(&(DPEInfo.SpinLockDPE));
		return 0;
	}
	spin_unlock(&(DPEInfo.SpinLockDPE));
	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';
	addrSzBuf[23] = '\0';
	valSzBuf[23] = '\0';
	if (sscanf(desc, "%23s %23s", addrSzBuf, valSzBuf) == 2) {
		pszTmp = strstr(addrSzBuf, "0x");
		if (pszTmp == NULL) {
			/*if (1 != sscanf(addrSzBuf, "%d", &addr))*/
#ifdef dpe_dump_read_en
			if (kstrtoint(addrSzBuf, 0, &addr) != 0)
				LOG_ERR("scan decimal addr is wrong !!:%s",
								addrSzBuf);
#else
				LOG_ERR("hex address only:%s", addrSzBuf);
#endif
		} else {
			if (strlen(addrSzBuf) > 2) {
				if (sscanf(addrSzBuf + 2, "%x", &addr) != 1)
					LOG_ERR("error hex addr:%s", addrSzBuf);
			} else {
				LOG_INF("DPE Write Addr Error!!:%s", addrSzBuf);
			}
		}
		pszTmp = strstr(valSzBuf, "0x");
		if (pszTmp == NULL) {
			/*if (1 != sscanf(valSzBuf, "%d", &val))*/
#ifdef dpe_dump_read_en
			if (kstrtoint(valSzBuf, 0, &val) != 0)
				LOG_ERR("scan decimal value is wrong !!:%s",
								valSzBuf);
#else
				LOG_ERR("HEX address only :%s", valSzBuf);
#endif
		} else {
			if (strlen(valSzBuf) > 2) {
				if (sscanf(valSzBuf + 2, "%x", &val) != 1)
					LOG_ERR("error hex val:%s", valSzBuf);
			} else {
				LOG_INF("DPE Write Value Error!!:%s\n",
								valSzBuf);
			}
		}
		if ((addr >= DPE_BASE_HW) && (addr <= DVP_CTRL_ATPG_HW)
			&& ((addr & 0x3) == 0)) {
			LOG_INF("Write Request - addr:0x%x, value:0x%x\n", addr,
									val);
			#ifdef DPE_WR32_en
			DPE_WR32((ISP_DPE_BASE + (addr - DPE_BASE_HW)), val);
			#endif
		} else {
			LOG_INF
			    ("write add out-of-range addr:0x%x, value:0x%x\n",
			     addr, val);
		}
	} else if (sscanf(desc, "%23s", addrSzBuf) == 1) {
		pszTmp = strstr(addrSzBuf, "0x");
		if (pszTmp == NULL) {
			/*if (1 != sscanf(addrSzBuf, "%d", &addr))*/
#ifdef dpe_dump_read_en
			if (kstrtoint(addrSzBuf, 0, &addr) != 0)
				LOG_ERR("scan decimal addr is wrong !!:%s",
								addrSzBuf);
#else
				LOG_ERR("HEX address only :%s", addrSzBuf);
#endif
		} else {
			if (strlen(addrSzBuf) > 2) {
				if (sscanf(addrSzBuf + 2, "%x", &addr) != 1)
					LOG_ERR("hex addr err:%s", addrSzBuf);
			} else {
				LOG_INF("DPE Read Addr Error!!:%s", addrSzBuf);
			}
		}
		if ((addr >= DPE_BASE_HW) && (addr <= DVP_CTRL_ATPG_HW)
			&& ((addr & 0x3) == 0)) {
			val = DPE_RD32((ISP_DPE_BASE + (addr - DPE_BASE_HW)));
			LOG_INF("Read Request - addr:0x%x,value:0x%x\n", addr,
									val);
		} else {
			LOG_INF
			    ("Read-Addr out-of-range!! addr:0x%x, value:0x%x\n",
			     addr, val);
		}
	}
	return count;
}
static int proc_dpe_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, dpe_reg_read, NULL);
}
static const struct file_operations dpe_reg_proc_fops = {
	.owner = THIS_MODULE,
	.open = proc_dpe_reg_open,
	.read = seq_read,
	.write = dpe_reg_write,
};
/**************************************************************
 *
 **************************************************************/
#if IS_ENABLED(CONFIG_MTK_M4U)
#if IS_ENABLED(CONFIG_MTK_IOMMU_V2)
enum mtk_iommu_callback_ret_t DPE_M4U_TranslationFault_callback(int port,
	unsigned long mva, void *data)
#else
enum m4u_callback_ret_t DPE_M4U_TranslationFault_callback(int port,
	unsigned int mva, void *data)
#endif
{
	pr_info("[ISP_M4U]fault call port=%d, mva=0x%lx", port, mva);
	switch (port) {
	default:
		DPE_DumpReg();
	break;
	}
#if IS_ENABLED(CONFIG_MTK_IOMMU_V2)
	return MTK_IOMMU_CALLBACK_HANDLED;
#else
	return M4U_CALLBACK_HANDLED;
#endif
}
#endif
/*******************************************************************************
 *
 ******************************************************************************/
int32_t DPE_ClockOnCallback(uint64_t engineFlag)
{
	/* LOG_DBG("DPE_ClockOnCallback"); */
	/* LOG_DBG("+CmdqEn:%d", g_u4EnableClockCount); */
	/* DPE_EnableClock(MTRUE); */
	return 0;
}
int32_t DPE_DumpCallback(uint64_t engineFlag, int level)
{
	LOG_DBG("DPE DumpCallback");
	DPE_DumpReg();
	dpe_request_dump(&dpe_reqs_dvs);
	return 0;
}
int32_t DPE_ResetCallback(uint64_t engineFlag)
{
	LOG_DBG("DPE ResetCallback");
	DPE_Reset();
	return 0;
}
int32_t DPE_ClockOffCallback(uint64_t engineFlag)
{
	/* LOG_DBG("DPE_ClockOffCallback"); */
	/* DPE_EnableClock(MFALSE); */
	/* LOG_DBG("-CmdqEn:%d", g_u4EnableClockCount); */
	return 0;
}
static signed int __init DPE_Init(void)
{
	signed int Ret = 0, j;
	void *tmp;
#if CHECK_SERVICE_IF_0
	struct device_node *node = NULL;

	/* FIX-ME: linux-3.10 procfs API changed */
	/* use proc_create */
	struct proc_dir_entry *isp_dpe_dir;
#endif
	int i;
	/*  */
	LOG_INF("- E. DPE Init");
	/*  */
#if CHECK_SERVICE_IF_0
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,DPE");
	if (!node) {
		LOG_ERR("find mediatek,DPE node failed!!!\n");
		return -ENODEV;
	}
	ISP_DPE_BASE = of_iomap(node, 0);
	if (!ISP_DPE_BASE) {
		LOG_ERR("unable to map ISP_DPE_BASE registers!!!\n");
		return -ENODEV;
	}
	LOG_DBG("ISP_DPE_BASE: %lx\n", ISP_DPE_BASE);

	isp_dpe_dir = proc_mkdir("dpe", NULL);
	if (!isp_dpe_dir) {
		LOG_ERR("[%s]: fail to mkdir /proc/dpe\n", __func__);
		return 0;
	}
	proc_entry = proc_create("dpe_dump", 0444, isp_dpe_dir,
							&dpe_dump_proc_fops);
	proc_entry = proc_create("dpe_reg", 0644, isp_dpe_dir,
							&dpe_reg_proc_fops);
#endif
	/* isr log */
	if (PAGE_SIZE <
	    ((DPE_IRQ_TYPE_AMOUNT * NORMAL_STR_LEN *
		((DBG_PAGE + INF_PAGE + ERR_PAGE) + 1)) * LOG_PPNUM)) {
		i = 0;
		while (i <
		       ((DPE_IRQ_TYPE_AMOUNT * NORMAL_STR_LEN *
			 ((DBG_PAGE + INF_PAGE + ERR_PAGE) + 1)) * LOG_PPNUM)) {
			i += PAGE_SIZE;
		}
	} else {
		i = PAGE_SIZE;
	}
	pLog_kmalloc = kmalloc(i, GFP_KERNEL);
	if (pLog_kmalloc == NULL) {
		LOG_ERR("log mem not enough\n");
		return -ENOMEM;
	}
	memset(pLog_kmalloc, 0x00, i);
	tmp = pLog_kmalloc;
	for (i = 0; i < LOG_PPNUM; i++) {
		for (j = 0; j < DPE_IRQ_TYPE_AMOUNT; j++) {
			gSvLog[j]._str[i][_LOG_DBG] = (char *)tmp;
			tmp = (void *)((char *)tmp +
						(NORMAL_STR_LEN * DBG_PAGE));
			gSvLog[j]._str[i][_LOG_INF] = (char *)tmp;
			tmp = (void *)((char *)tmp +
						(NORMAL_STR_LEN * INF_PAGE));
			gSvLog[j]._str[i][_LOG_ERR] = (char *)tmp;
			tmp = (void *)((char *)tmp +
						(NORMAL_STR_LEN * ERR_PAGE));
		}
		/* tmp = (void*) ((unsigned int)tmp + NORMAL_STR_LEN); */
		tmp = (void *)((char *)tmp + NORMAL_STR_LEN);	/* overflow */
	}
	spin_lock_init(&DPEInfo.SpinLockDPERef);
	spin_lock_init(&DPEInfo.SpinLockDPE);
	spin_lock_init(&DPEInfo.SpinLockFD);
	for (i = 0; i < DPE_IRQ_TYPE_AMOUNT; i++) {
		spin_lock_init(&DPEInfo.SpinLockIrq[i]);
		tasklet_init(DPE_tasklet[i].pDPE_tkt, DPE_tasklet[i].tkt_cb, 0);
	}
	init_waitqueue_head(&DPEInfo.WaitQueueHead);
	INIT_WORK(&DPEInfo.ScheduleDpeWork, DPE_ScheduleWork);
	INIT_WORK(&DPEInfo.DVP_ScheduleDpeWork, DVP_ScheduleWork);
	INIT_WORK(&logWork, logPrint);
	seqlock_init(&dpe_reqs_dvs.seqlock);
	dpe_reqs_dvs.state_lock = &DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST];
	dpe_reqs_dvp.state_lock = dpe_reqs_dvs.state_lock;
	seqlock_init(&dpe_reqs_dvp.seqlock);
	DPEInfo.IrqInfo.Mask[DPE_IRQ_TYPE_INT_DVP_ST] = INT_ST_MASK_DPE;
	DPEInfo.wkqueue = alloc_ordered_workqueue("DPE-CMDQ-WQ", 0);
	if (!DPEInfo.wkqueue) { kfree(pLog_kmalloc); return -ENOMEM; }
	Ret = register_pm_notifier(&dpe_suspend_pm_notifier_func);
	if (Ret) goto free_workqueue;
	Ret = platform_driver_register(&DPEDriver);
	if (!Ret) return 0;
	unregister_pm_notifier(&dpe_suspend_pm_notifier_func);
free_workqueue:
	destroy_workqueue(DPEInfo.wkqueue);
	DPEInfo.wkqueue = NULL;
	kfree(pLog_kmalloc);
	pLog_kmalloc = NULL;
	return Ret;
}
/*******************************************************************************/
static void __exit DPE_Exit(void)
{
	int i;
	/* Only reachable when neither hardware instance bound/pinned this module. */
	platform_driver_unregister(&DPEDriver);
	unregister_pm_notifier(&dpe_suspend_pm_notifier_func);
	destroy_workqueue(DPEInfo.wkqueue);
	cancel_work_sync(&logWork);
	for (i = 0; i < DPE_IRQ_TYPE_AMOUNT; i++)
		tasklet_kill(DPE_tasklet[i].pDPE_tkt);
	kfree(pLog_kmalloc);
}
/*******************************************************************************/
void DPE_ScheduleWork(struct work_struct *data)
{

	if (DPE_DBG_DBGLOG & DPEInfo.DebugMask)
		LOG_INF("- E.");

	dpe_request_handler(&dpe_reqs_dvs,
			&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));

	LOG_INF("[%s] start", __func__);
	if (!dpe_request_running(&dpe_reqs_dvs))
		LOG_INF("[%s]no more requests", __func__);

}

void DVP_ScheduleWork(struct work_struct *data)
{

	if (DPE_DBG_DBGLOG & DPEInfo.DebugMask)
		LOG_INF("- E.");

	dpe_request_handler(&dpe_reqs_dvp,
			&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));

	LOG_INF("[%s] start", __func__);
	if (!dpe_request_running(&dpe_reqs_dvp))
		LOG_INF("[%s]no more requests", __func__);

}
static irqreturn_t ISP_Irq_DVP(signed int Irq, void *DeviceId)
{
	unsigned int DvsStatus, DvpStatus;
	bool bResulst = MFALSE;
	bool isDvpDone = MFALSE;
	pid_t ProcessID;
	unsigned int p = 0;

	DvsStatus = DPE_RD32(DVS_CTRL_STATUS0_REG);	/* DVS Status */
	DvpStatus = DPE_RD32(DVP_CTRL_STATUS0_REG);	/* DVP Status */

	if ((DvsStatus == 0) || (DvpStatus == 0))
		LOG_INF("DPE Read status fail, IRQ, DvsStatus: 0x%08x, DvpStatus: 0x%08x\n",
		DvsStatus, DvpStatus);

	#ifdef DPE_debug_log_en
	 LOG_INF("DVP IRQ, DvsStatus: 0x%08x, DvpStatus: 0x%08x\n",
	 DvsStatus, DvpStatus);
	#endif
	/* DPE done status may rise later, so can't use done status now  */
	/* if (DPE_INT_ST == (DPE_INT_ST & DvpStatus)) { */

		DPE_WR32(DVP_IRQ_00_REG, 0x040000F0); /* Clear DVP IRQ */
		DPE_WR32(DVP_IRQ_00_REG, 0x04000F00);

		isDvpDone = MTRUE;
	/* } */
	spin_lock(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));
	if (isDvpDone == MTRUE) {
		/* Update the frame status. */
#ifdef __DPE_KERNEL_PERFORMANCE_MEASURE__
		mt_kernel_trace_begin("dpe_irq");
#endif
		if (dpe_update_request(&dpe_reqs_dvp, &ProcessID) == 0)
			bResulst = MTRUE;
		if (bResulst == MTRUE) {
			#if REQUEST_REGULATION == REQUEST_BASE_REGULATION
			/* schedule_work(&DPEInfo.ScheduleDpeWork); */
			queue_work(DPEInfo.wkqueue, &DPEInfo.DVP_ScheduleDpeWork);
			#endif
			p = ProcessID % IRQ_USER_NUM_MAX;
			DPEInfo.IrqInfo.Status[DPE_IRQ_TYPE_INT_DVP_ST] |=
				DPE_INT_ST;
			DPEInfo.IrqInfo.ProcessID[p] =
				ProcessID;
			DPEInfo.IrqInfo.DpeIrqCnt[p]++;
			DPEInfo.ProcessID[DPEInfo.WriteReqIdx] = ProcessID;
			DPEInfo.WriteReqIdx =
				(DPEInfo.WriteReqIdx + 1) %
				_SUPPORT_MAX_DPE_FRAME_REQUEST_;
		}
#ifdef __DPE_KERNEL_PERFORMANCE_MEASURE__
		mt_kernel_trace_end();
#endif
		/* Config the Next frame */
	}

	spin_unlock(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));
	if (bResulst == MTRUE)
		wake_up_interruptible(&DPEInfo.WaitQueueHead);
	/* dump log, use tasklet */
	IRQ_LOG_KEEPER(
		DPE_IRQ_TYPE_INT_DVP_ST,
		m_CurrentPPB,
		_LOG_INF,
		"DVP_IRQ:%d,0x%x:0x%x,0x%x:0x%x,Result:%d\n",
		Irq,
		DVS_CTRL_STATUS0_HW,
		DvsStatus,
		DVP_CTRL_STATUS0_HW,
		DvpStatus,
		bResulst);
	IRQ_LOG_KEEPER(
		DPE_IRQ_TYPE_INT_DVP_ST,
		m_CurrentPPB,
		_LOG_INF,
		"IrqCnt[%d]:0x%x,WReq:0x%x,RReq:0x%x\n",
		p, DPEInfo.IrqInfo.DpeIrqCnt[p],
		DPEInfo.WriteReqIdx,
		DPEInfo.ReadReqIdx);
	#if (REQUEST_REGULATION == FRAME_BASE_REGULATION)
	/* schedule_work(&DPEInfo.ScheduleDpeWork); */
	queue_work(DPEInfo.wkqueue, &DPEInfo.DVP_ScheduleDpeWork);
	#endif
	if (isDvpDone == MTRUE)

		schedule_work(&logWork);

		//tasklet_schedule(DPE_tasklet[DPE_IRQ_TYPE_INT_DVP_ST].pDPE_tkt);

	return IRQ_HANDLED;
}
static irqreturn_t ISP_Irq_DVS(signed int Irq, void *DeviceId)
{
	unsigned int DvsStatus, DvpStatus;
	bool bResulst = MFALSE;
	bool isDvsDone = MFALSE;
	pid_t ProcessID;
	unsigned int p = 0;

	DvsStatus = DPE_RD32(DVS_CTRL_STATUS0_REG);	/* DVS Status */
	DvpStatus = DPE_RD32(DVP_CTRL_STATUS0_REG);	/* DVP Status */

	if ((DvsStatus == 0) || (DvpStatus == 0))
		LOG_INF("DPE Read status fail, IRQ, DvsStatus: 0x%08x, DvpStatus: 0x%08x\n",
		DvsStatus, DvpStatus);

	#ifdef DPE_debug_log_en
	LOG_INF("DVS IRQ, DvsStatus: 0x%08x, DvpStatus: 0x%08x\n",
	DvsStatus, DvpStatus);
	#endif

	/* DPE done status may rise later, so can't use done status now  */
	/* if (DPE_INT_ST == (DPE_INT_ST & DvsStatus)) { */

		DPE_WR32(DVS_IRQ_00_REG, 0x040000F0); /* Clear DVS IRQ */
		DPE_WR32(DVS_IRQ_00_REG, 0x04000F00);

		isDvsDone = MTRUE;
	/* } */
	spin_lock(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));
	if (isDvsDone == MTRUE) {
		/* Update the frame status. */
#ifdef __DPE_KERNEL_PERFORMANCE_MEASURE__
		mt_kernel_trace_begin("dpe_irq");
#endif
		if (dpe_update_request(&dpe_reqs_dvs, &ProcessID) == 0)
			bResulst = MTRUE;
		if (bResulst == MTRUE) {
			#if REQUEST_REGULATION == REQUEST_BASE_REGULATION
			/* schedule_work(&DPEInfo.ScheduleDpeWork); */
			queue_work(DPEInfo.wkqueue, &DPEInfo.ScheduleDpeWork);
			#endif
			p = ProcessID % IRQ_USER_NUM_MAX;
			DPEInfo.IrqInfo.Status[DPE_IRQ_TYPE_INT_DVP_ST] |=
				DPE_INT_ST;
			DPEInfo.IrqInfo.ProcessID[p] =
				ProcessID;
			DPEInfo.IrqInfo.DpeIrqCnt[p]++;
			DPEInfo.ProcessID[DPEInfo.WriteReqIdx] = ProcessID;
			DPEInfo.WriteReqIdx =
				(DPEInfo.WriteReqIdx + 1) %
				_SUPPORT_MAX_DPE_FRAME_REQUEST_;
		}
#ifdef __DPE_KERNEL_PERFORMANCE_MEASURE__
		mt_kernel_trace_end();
#endif
		/* Config the Next frame */
	}
	spin_unlock(&(DPEInfo.SpinLockIrq[DPE_IRQ_TYPE_INT_DVP_ST]));
	if (bResulst == MTRUE)
		wake_up_interruptible(&DPEInfo.WaitQueueHead);
	/* dump log, use tasklet */
	IRQ_LOG_KEEPER(
		DPE_IRQ_TYPE_INT_DVS_ST,
		m_CurrentPPB,
		_LOG_INF,
		"DVS_IRQ:%d,0x%x:0x%x,0x%x:0x%x,Result:%d\n",
		Irq,
		DVS_CTRL_STATUS0_HW,
		DvsStatus,
		DVP_CTRL_STATUS0_HW,
		DvpStatus,
		bResulst);
	IRQ_LOG_KEEPER(
		DPE_IRQ_TYPE_INT_DVS_ST,
		m_CurrentPPB,
		_LOG_INF,
		"IrqCnt[%d]:0x%x,WReq:0x%x,RReq:0x%x\n",
		p, DPEInfo.IrqInfo.DpeIrqCnt[p],
		DPEInfo.WriteReqIdx,
		DPEInfo.ReadReqIdx);
	#if (REQUEST_REGULATION == FRAME_BASE_REGULATION)
	/* schedule_work(&DPEInfo.ScheduleDpeWork); */
	queue_work(DPEInfo.wkqueue, &DPEInfo.ScheduleDpeWork);
	#endif
	if (isDvsDone == MTRUE)
//#if 1
		schedule_work(&logWork);
//#else
//tasklet_schedule(DPE_tasklet[DPE_IRQ_TYPE_INT_DVP_ST].pDPE_tkt);
//#endif
	return IRQ_HANDLED;
}
static void ISP_TaskletFunc_DVP(unsigned long data)
{
	IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVP_ST, m_CurrentPPB, _LOG_DBG);
	IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVP_ST, m_CurrentPPB, _LOG_INF);
	IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVP_ST, m_CurrentPPB, _LOG_ERR);
}
static void ISP_TaskletFunc_DVS(unsigned long data)
{
	IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVS_ST, m_CurrentPPB, _LOG_DBG);
	IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVS_ST, m_CurrentPPB, _LOG_INF);
	IRQ_LOG_PRINTER(DPE_IRQ_TYPE_INT_DVS_ST, m_CurrentPPB, _LOG_ERR);
}
static void logPrint(struct work_struct *data)
{
	unsigned long arg = 0;

	ISP_TaskletFunc_DVP(arg);
	ISP_TaskletFunc_DVS(arg);
}
/******************************************************************************
 *
 ******************************************************************************/
module_init(DPE_Init);
module_exit(DPE_Exit);
MODULE_DESCRIPTION("Camera DPE driver");
MODULE_AUTHOR("MM3SW2");
MODULE_LICENSE("GPL");

MODULE_IMPORT_NS(DMA_BUF);
