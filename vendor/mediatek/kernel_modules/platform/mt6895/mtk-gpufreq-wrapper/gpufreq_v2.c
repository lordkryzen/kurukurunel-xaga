// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 */

/**
 * @file    gpufreq_v2.c
 * @brief   GPU-DVFS Driver Common Wrapper
 */

/**
 * ===============================================
 * Include
 * ===============================================
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/export.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>

#include <gpufreq_v2.h>
#include <gpufreq_ipi.h>
#include <gpufreq_mssv.h>
#include <gpufreq_debug.h>
#include <gpueb_ipi.h>
#include <gpueb_reserved_mem.h>
#include <gpueb_debug.h>
#include <gpueb_common.h>
#include <ghpm_wrapper.h>
#include "gpufreq_legacy_wire.h"
#include "gpufreq_legacy_status.h"
#include "gpufreq_legacy_tables.h"
#include "gpufreq_firmware.h"
#include "gpufreq_mt6895_a16_wire.h"
#include "thermal_gpu_provider.h"
#include <mtk_gpu_utility.h>

#if IS_ENABLED(CONFIG_MTK_PBM)
#include <mtk_pbm_gpu_cb.h>
#endif
#if IS_ENABLED(CONFIG_DEVAPC_ARCH_MULTI)
#include <linux/soc/mediatek/devapc_public.h>
#endif

/**
 * ===============================================
 * Local Function Declaration
 * ===============================================
 */
static int gpufreq_wrapper_pdrv_probe(struct platform_device *pdev);
static void gpufreq_wrapper_pdrv_remove(struct platform_device *pdev);
static int gpufreq_shared_memory_init(void);
static int gpufreq_gpueb_init(void);
static void gpufreq_init_external_callback(void);
static int gpufreq_ipi_to_gpueb(struct gpufreq_ipi_data data);
static int gpufreq_validate_target(unsigned int *target);
static void gpufreq_dump_external_status(char *log_buf, int *log_len, int log_size);
static void gpufreq_dump_internal_status(char *log_buf, int *log_len, int log_size);
static void gpufreq_dump_shared_status(char *log_buf, int *log_len, int log_size);
static void gpufreq_abort(void);

/**
 * ===============================================
 * Local Variable Definition
 * ===============================================
 */
static const struct of_device_id g_gpufreq_wrapper_of_match[] = {
	{ .compatible = "mediatek,gpufreq_wrapper" },
	{ /* sentinel */ }
};
static struct platform_driver g_gpufreq_wrapper_pdrv = {
	.probe = gpufreq_wrapper_pdrv_probe,
	.remove = gpufreq_wrapper_pdrv_remove,
	.driver = {
		.name = "gpufreq_wrapper",
		.suppress_bind_attrs = true,
		.owner = THIS_MODULE,
		.of_match_table = g_gpufreq_wrapper_of_match,
	},
};

static struct gpufreq_platform_fp *gpufreq_fp;
static struct gpuppm_platform_fp *gpuppm_fp;
static struct gpufreq_ipi_data g_recv_msg;
static struct gpufreq_shared_status *g_shared_status;
/* Never cast firmware's legacy region as the native shared-status structure. */
static void __iomem *g_legacy_status;
static struct gpufreq_legacy_memory g_legacy_memory;
static struct gpufreq_legacy_ipi g_legacy_reply;
static bool g_legacy_firmware;
static struct gpufreq_opp_info g_legacy_working[2][GPUFREQ_MAX_OPP_NUM];
static struct gpufreq_opp_info g_legacy_signed[2][GPUFREQ_MAX_OPP_NUM];
static u32 g_legacy_signed_count[2];
static struct gpufreq_legacy_debug_opp g_legacy_debug_opp[2];
static struct gpufreq_legacy_opp g_legacy_table_scratch[GPUFREQ_MAX_OPP_NUM];
static bool g_legacy_tables_ready;
static DEFINE_MUTEX(gpufreq_callback_lock);
static bool g_legacy_callbacks_pinned;
static phys_addr_t g_shared_mem_pa;
static int g_ipi_channel;
static unsigned int g_gpufreq_wrapper_ready;
static unsigned int g_gpufreq_platform_ready;
static unsigned int g_dual_buck;
static unsigned int g_gpueb_support;
static unsigned int g_gpufreq_bringup;
static unsigned int g_ipi_magic;
static unsigned int g_shared_mem_size;
static unsigned int g_abort_state;
static raw_spinlock_t gpufreq_ipi_lock;
static raw_spinlock_t gpufreq_power_lock;

static bool g_a16_firmware;
static void __iomem *g_a16_status;
static bool g_a16_tables_ready;
static int g_a16_fault;
static unsigned char g_a16_snapshot[A16_SHARED_SIZE];
static struct gpufreq_opp_info g_a16_working[2][A16_OPP_CAPACITY];
static u32 a16_read(unsigned int offset)
{
    if (!g_a16_status || offset > A16_SHARED_SIZE - sizeof(u32) || (offset & 3))
        return 0;
    return readl(g_a16_status + offset);
}

/* Executed under the common polling-mailbox lock. Never use legacy opcodes. */
static int gpufreq_a16_transaction(struct gpufreq_ipi_data request)
{
    int ret;
    lockdep_assert_held(&gpufreq_ipi_lock);
    if (g_a16_fault) return -ENOTCONN;
    switch (request.cmd_id) {
    case CMD_INIT_SHARED_MEM: break;
    case CMD_GET_FREQ_BY_IDX:
    case CMD_GET_POWER_BY_IDX:
    case CMD_COMMIT:
        if ((request.target != TARGET_GPU && request.target != TARGET_STACK) ||
            request.u.oppidx < 0 || request.u.oppidx >=
            (request.target == TARGET_GPU ? 6 : 40)) return -ERANGE;
        break;
    case CMD_DUAL_COMMIT:
        if (request.u.dual_commit.gpu_oppidx < 0 || request.u.dual_commit.gpu_oppidx >= 6 ||
            request.u.dual_commit.stack_oppidx < 0 || request.u.dual_commit.stack_oppidx >= 40)
            return -ERANGE;
        break;
    case CMD_GET_OPPIDX_BY_FREQ:
    case CMD_GET_LEAKAGE_POWER:
        if (request.target != TARGET_GPU && request.target != TARGET_STACK) return -EINVAL;
        break;
    case CMD_SET_LIMIT:
    case CMD_SWITCH_LIMIT:
        if (request.target != TARGET_STACK || request.u.set_limit.limiter >= LIMIT_NUM)
            return -EOPNOTSUPP;
        break;
    case CMD_POWER_CONTROL:
    case CMD_ACTIVE_SLEEP_CONTROL:
    case CMD_PDCA_CONFIG:
        if (request.u.power_state > 1) return -EINVAL;
        break;
    default: return -EOPNOTSUPP;
    }
    request.magic = g_ipi_magic;
    memset(&g_recv_msg, 0, sizeof(g_recv_msg));
    ret = mtk_ipi_send_compl(get_gpueb_ipidev(), g_ipi_channel, IPI_SEND_POLLING,
        &request, sizeof(request) / sizeof(u32), IPI_TIMEOUT_MS);
    if (ret != IPI_ACTION_DONE) {
        /* Completion is uncertain: no retry, fallback power sequence or debug SMC. */
        g_a16_fault = ret < 0 ? ret : -EIO;
        return g_a16_fault;
    }
    return 0;
}

static int gpufreq_a16_gpueb_init(void)
{
    struct gpufreq_ipi_data request = {};
    void __iomem *magic;
    unsigned long flags;
    u32 domain, row, col, value;
    int ret;
    BUILD_BUG_ON(sizeof(request) != 32);
    BUILD_BUG_ON(offsetof(struct gpufreq_ipi_data, u) != 16);
    BUILD_BUG_ON(sizeof(struct gpufreq_opp_info) != A16_ROW_BYTES);
    if (!g_a16_status || g_shared_mem_size != A16_SHARED_SIZE || !g_dual_buck)
        return -EINVAL;
    g_ipi_channel = gpueb_get_send_PIN_ID_by_name("IPI_ID_GPUFREQ");
    if (g_ipi_channel < 0) return g_ipi_channel;
    ret = mtk_ipi_register(get_gpueb_ipidev(), g_ipi_channel, NULL, NULL, &g_recv_msg);
    if (ret != IPI_ACTION_DONE) return ret < 0 ? ret : -EIO;
    magic = gpueb_get_gpr_addr(GPUEB_SRAM_GPR13);
    if (!magic) { ret = -ENODEV; goto fail; }
    g_ipi_magic = readl(magic);
    if (!g_ipi_magic) { ret = -ENODATA; goto fail; }
    writel(0, magic);
    request.cmd_id = CMD_INIT_SHARED_MEM;
    request.u.shared_mem.base = g_shared_mem_pa;
    request.u.shared_mem.size = g_shared_mem_size;
    raw_spin_lock_irqsave(&gpufreq_ipi_lock, flags);
    ret = gpufreq_a16_transaction(request);
    raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, flags);
    if (ret) goto fail;
    memcpy_fromio(g_a16_snapshot, g_a16_status, sizeof(g_a16_snapshot));
    ret = a16_validate_snapshot(g_a16_snapshot, sizeof(g_a16_snapshot));
    if (ret) goto fail;
    for (domain = 0; domain < 2; domain++) {
        for (row = 0; row < (domain ? 40 : 6); row++) {
            u32 *output = (u32 *)&g_a16_working[domain][row];
            for (col = 0; col < 6; col++) {
                ret = a16_u32(g_a16_snapshot, sizeof(g_a16_snapshot),
                    (domain ? A16_WORKING_STACK : A16_WORKING_GPU) + row * A16_ROW_BYTES + col * 4,
                    &value);
                if (ret) goto fail;
                output[col] = value;
            }
        }
        /* Independently verify synchronous response union before publishing ready. */
        memset(&request, 0, sizeof(request));
        request.cmd_id = CMD_GET_FREQ_BY_IDX;
        request.target = domain ? TARGET_STACK : TARGET_GPU;
        request.u.oppidx = 0;
        raw_spin_lock_irqsave(&gpufreq_ipi_lock, flags);
        ret = gpufreq_a16_transaction(request);
        value = g_recv_msg.u.freq;
        raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, flags);
        if (ret) goto fail;
        if (value != g_a16_working[domain][0].freq) { ret = -EPROTO; goto fail; }
    }
    smp_store_release(&g_a16_tables_ready, true);
    GPUFREQ_LOGI("MT6895 A16 wire validated: GPU 6 / STACK 40 working OPPs");
    return 0;
fail:
    mtk_ipi_unregister(get_gpueb_ipidev(), g_ipi_channel);
    return ret;
}


/**
 * ===============================================
 * External Function Callback
 * ===============================================
 */
#define legacy_status_read(field) \
	readl(g_legacy_status + offsetof(struct gpufreq_legacy_status_prefix, field))

static int gpufreq_legacy_transaction(const struct gpufreq_ipi_data *request)
{
	struct gpufreq_legacy_ipi message;
	struct gpufreq_ipi_data reply;
	int ret;

	/* Caller holds the existing polling-IPI serialization lock. */
	lockdep_assert_held(&gpufreq_ipi_lock);
	ret = gpufreq_legacy_encode(request, &g_legacy_memory, &message);
	if (ret)
		return ret;
	memset(&g_legacy_reply, 0, sizeof(g_legacy_reply));
	ret = mtk_ipi_send_compl(get_gpueb_ipidev(), g_ipi_channel,
		IPI_SEND_POLLING, &message, sizeof(message) / sizeof(u32), IPI_TIMEOUT_MS);
	if (ret != IPI_ACTION_DONE)
		return ret;
	ret = gpufreq_legacy_decode(request, &message, &g_legacy_reply, &reply);
	if (ret)
		return ret;
	g_recv_msg = reply;
	return 0;
}

static int gpufreq_legacy_debug_query(u32 command, u32 target)
{
	struct gpufreq_legacy_ipi request = {.command = command, .target = target};
	int ret;

	lockdep_assert_held(&gpufreq_ipi_lock);
	if (command < 8 || command > 12 || (target != TARGET_GPU && target != TARGET_STACK))
		return -EINVAL;
	memset(&g_legacy_reply, 0, sizeof(g_legacy_reply));
	ret = mtk_ipi_send_compl(get_gpueb_ipidev(), g_ipi_channel, IPI_SEND_POLLING,
		&request, sizeof(request) / sizeof(u32), IPI_TIMEOUT_MS);
	if (ret)
		return ret;
	if (g_legacy_reply.command && g_legacy_reply.command != command)
		return -EPROTO;
	/* Stock status/debug-info refreshes publish shared memory and only
	 * check transport completion; unlike table fetches, no result payload.
	 */
	return command <= 9 ? 0 : g_legacy_reply.payload.value;
}

/* Debug payload belongs to the firmware and is shared between both targets.
 * Copy it while holding the same mailbox lock as the query; never return a
 * pointer into firmware scratch, or substitute a requested clock for a meter.
 */
static int gpufreq_legacy_debug_snapshot(u32 target,
	struct gpufreq_legacy_debug_opp *output)
{
	struct gpufreq_legacy_debug_opp snapshot;
	unsigned long flags;
	int ret;

	if (!output)
		return -EINVAL;
	if (!smp_load_acquire(&g_gpufreq_wrapper_ready))
		return -EAGAIN;
	raw_spin_lock_irqsave(&gpufreq_ipi_lock, flags);
	ret = gpufreq_legacy_debug_query(8, target);
	if (!ret)
		memcpy_fromio(&snapshot, g_legacy_status + sizeof(struct gpufreq_legacy_status_prefix),
			sizeof(snapshot));
	raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, flags);
	if (!ret)
		*output = snapshot;
	return ret;
}

/* Initial, private snapshots become immutable after the complete set passes.
 * The firmware retains ownership of live voltage/OPP policy and debug scratch.
 */
static int gpufreq_legacy_load_tables(void)
{
	unsigned long flags;
	u32 target, index, count, kind;
	int ret = 0;

	for (target = TARGET_GPU; target <= (g_dual_buck ? TARGET_STACK : TARGET_GPU); target++) {
		struct gpufreq_legacy_debug_opp *info = &g_legacy_debug_opp[target - TARGET_GPU];

		index = target - TARGET_GPU;
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, flags);
		ret = gpufreq_legacy_debug_query(8, target);
		if (!ret) {
			memcpy_fromio(info, g_legacy_status + sizeof(struct gpufreq_legacy_status_prefix),
				sizeof(*info));
			count = target == TARGET_STACK ? legacy_status_read(opp_num_stack) :
				legacy_status_read(opp_num_gpu);
			if (info->working_count <= 0 || (u32)info->working_count != count ||
			    info->signed_count < info->working_count ||
			    info->signed_count > GPUFREQ_MAX_OPP_NUM)
				ret = -EPROTO;
		}
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, flags);
		if (ret)
			return ret;
		for (kind = 0; kind < 2; kind++) {
			struct gpufreq_opp_info *table = kind ? g_legacy_signed[index] :
				g_legacy_working[index];

			/* Like the stock debug frontend outside explicit test mode,
			 * CMD_GET_SIGNED_TABLE exposes only working_count rows.
			 * signed_count includes test-only rows which are not written
			 * to scratch in normal mode (40 exposed vs 42 nominal).
			 */
			count = info->working_count;
			if (count > (g_legacy_memory.size - 0x400) / sizeof(struct gpufreq_legacy_opp))
				return -ERANGE;
			raw_spin_lock_irqsave(&gpufreq_ipi_lock, flags);
			ret = gpufreq_legacy_debug_query(kind ? 11 : 10, target);
			if (!ret) {
				memcpy_fromio(g_legacy_table_scratch, g_legacy_status + 0x400,
					count * sizeof(struct gpufreq_legacy_opp));
				ret = gpufreq_legacy_decode_table(g_legacy_table_scratch, count,
					g_legacy_memory.size - 0x400, table);
			}
			raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, flags);
			if (ret)
				return ret;
			if (kind)
				g_legacy_signed_count[index] = count;
		}
	}
	smp_store_release(&g_legacy_tables_ready, true);
	return 0;
}

static int gpufreq_legacy_gpueb_init(void)
{
	struct gpufreq_ipi_data request = {.cmd_id = CMD_INIT_SHARED_MEM};
	struct gpufreq_legacy_status_prefix status;
	unsigned long flags;
	int ret;

	g_ipi_channel = gpueb_get_send_PIN_ID_by_name("IPI_ID_GPUFREQ");
	if (g_ipi_channel < 0)
		return g_ipi_channel;
	ret = mtk_ipi_register(get_gpueb_ipidev(), g_ipi_channel,
		NULL, NULL, &g_legacy_reply);
	if (ret != IPI_ACTION_DONE)
		return ret;
	raw_spin_lock_irqsave(&gpufreq_ipi_lock, flags);
	ret = gpufreq_legacy_transaction(&request);
	if (!ret)
		ret = g_recv_msg.u.return_value;
	raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, flags);
	if (!ret) {
		memcpy_fromio(&status, g_legacy_status, sizeof(status));
		ret = gpufreq_legacy_validate_status(&status, GPUFREQ_MAX_OPP_NUM, g_dual_buck);
	}
	if (!ret)
		ret = gpufreq_legacy_load_tables();
	if (ret)
		mtk_ipi_unregister(get_gpueb_ipidev(), g_ipi_channel);
	return ret;
}

unsigned long (*ged_get_last_commit_idx_fp)(void);
EXPORT_SYMBOL(ged_get_last_commit_idx_fp);
unsigned long (*ged_get_last_commit_top_idx_fp)(void);
EXPORT_SYMBOL(ged_get_last_commit_top_idx_fp);
unsigned long (*ged_get_last_commit_stack_idx_fp)(void);
EXPORT_SYMBOL(ged_get_last_commit_stack_idx_fp);
void (*ged_notify_gpu_fix_opp_fp)(int gpu_opp, int stack_opp);
EXPORT_SYMBOL(ged_notify_gpu_fix_opp_fp);
void (*ged_notify_gpu_fix_freq_volt_fp)(unsigned int fgpu, unsigned int vgpu,
	unsigned int fstack, unsigned int vtstack);
EXPORT_SYMBOL(ged_notify_gpu_fix_freq_volt_fp);
void (*ged_notify_dcs_fix_opp_fp)(int gpu_opp, int stack_opp);
EXPORT_SYMBOL(ged_notify_dcs_fix_opp_fp);
#if IS_ENABLED(CONFIG_DEVAPC_ARCH_MULTI)
static bool gpufreq_devapc_vio_callback(void);
static bool gpufreq_bus_tracker_vio_callback(int slave_type);
struct devapc_power_callbacks devapc_cb_gpu = {
	.type = DEVAPC_TYPE_GPU,
	.query_power = gpufreq_devapc_vio_callback,
};
struct devapc_power_callbacks devapc_cb_gpu1 = {
	.type = DEVAPC_TYPE_GPU1,
	.query_power = gpufreq_devapc_vio_callback,
};
struct devapc_excep_callbacks bus_tracker_cb = {
	.type = DEVAPC_TYPE_GPU1,
	.handle_excep = gpufreq_bus_tracker_vio_callback,
};
#endif /* CONFIG_DEVAPC_ARCH_MULTI */

/**
 * ===============================================
 * External Function Definition
 * ===============================================
 */
/***********************************************************************************
 * Function Name      : gpufreq_bringup
 * Returns            : status - Current status of bring-up
 * Description        : Check GPUFREQ bring-up status
 *                      If it's bring-up status, GPUFREQ is out-of-function
 ***********************************************************************************/
unsigned int gpufreq_bringup(void)
{
	return g_gpufreq_bringup;
}
EXPORT_SYMBOL(gpufreq_bringup);

/***********************************************************************************
 * Function Name      : gpufreq_wrapper_ready
 * Returns            : status - Status of wrapper driver probe
 * Description        : Check GPUFREQ wrapper probe status
 ***********************************************************************************/
unsigned int gpufreq_wrapper_ready(void)
{
	return g_gpufreq_wrapper_ready;
}
EXPORT_SYMBOL(gpufreq_wrapper_ready);

enum gpufreq_firmware_abi gpufreq_get_firmware_abi(void)
{
	if (!smp_load_acquire(&g_gpufreq_wrapper_ready))
		return GPUFREQ_FIRMWARE_UNAVAILABLE;
	if (!g_gpueb_support)
		return GPUFREQ_FIRMWARE_AP;
	if (g_a16_firmware) return GPUFREQ_FIRMWARE_MT6895_A16;
	return g_legacy_firmware ? GPUFREQ_FIRMWARE_LEGACY : GPUFREQ_FIRMWARE_NATIVE;
}
EXPORT_SYMBOL_GPL(gpufreq_get_firmware_abi);

/***********************************************************************************
 * Function Name      : gpufreq_driver_ready
 * Returns            : status - Status of wrapper and platform driver probe
 * Description        : Check GPUFREQ wrapper and platform probe status
 ***********************************************************************************/
unsigned int gpufreq_driver_ready(void)
{
	return g_gpufreq_wrapper_ready && g_gpufreq_platform_ready;
}
EXPORT_SYMBOL(gpufreq_driver_ready);

/***********************************************************************************
 * Function Name      : gpufreq_power_ctrl_enable
 * Inputs             : -
 * Outputs            : -
 * Returns            : status - Current status of power control
 * Description        : Check whether power control of GPU HW is enable
 *                      If it's disabled, GPU HW is always power-on
 ***********************************************************************************/
unsigned int gpufreq_power_ctrl_enable(void)
{
	if (g_a16_firmware) return a16_read(0xcc);
	unsigned int power_control = false;

	if (g_legacy_firmware)
		return legacy_status_read(power_control);

	if (g_shared_status)
		power_control = g_shared_status->power_control;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return power_control;
}
EXPORT_SYMBOL(gpufreq_power_ctrl_enable);

/***********************************************************************************
 * Function Name      : gpufreq_active_sleep_ctrl_enable
 * Inputs             : -
 * Outputs            : -
 * Returns            : status - Current status of active-idle control
 * Description        : Check whether active-idle control of GPU HW is enable
 *                      If it's disabled, GPU HW is always active
 ***********************************************************************************/
unsigned int gpufreq_active_sleep_ctrl_enable(void)
{
	if (g_a16_firmware) return a16_read(0xd0);
	unsigned int active_sleep_control = false;

	if (g_legacy_firmware)
		return false;

	if (g_shared_status)
		active_sleep_control = g_shared_status->active_sleep_control;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return active_sleep_control;
}
EXPORT_SYMBOL(gpufreq_active_sleep_ctrl_enable);

/***********************************************************************************
 * Function Name      : gpufreq_get_power_state
 * Inputs             : -
 * Outputs            : -
 * Returns            : state - Current status of GPU power
 * Description        : Get current power state
 ***********************************************************************************/
unsigned int gpufreq_get_power_state(void)
{
	if (g_a16_firmware) return (s32)a16_read(0x1c) > 0 ? GPU_PWR_ON : GPU_PWR_OFF;
	enum gpufreq_power_state power_state = GPU_PWR_OFF;
	int power_count = 0, active_count = 0;

	if (g_legacy_firmware)
		return (s32)legacy_status_read(power_count) > 0 ? GPU_PWR_ON : GPU_PWR_OFF;

	if (g_shared_status) {
		power_count = g_shared_status->power_count;
		power_state = power_count > 0 ? GPU_PWR_ON : GPU_PWR_OFF;
	} else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return power_state;
}
EXPORT_SYMBOL(gpufreq_get_power_state);

/***********************************************************************************
 * Function Name      : gpufreq_get_dvfs_state
 * Inputs             : -
 * Outputs            : -
 * Returns            : state - Current status of GPU DVFS
 * Description        : Get current DVFS state
 *                      If it isn't DVFS_FREE, then DVFS is fixed in some state
 ***********************************************************************************/
unsigned int gpufreq_get_dvfs_state(void)
{
	if (g_a16_firmware) return a16_read(0xd4);
	enum gpufreq_dvfs_state dvfs_state = DVFS_DISABLE;

	if (g_legacy_firmware) {
		unsigned int state;

		/* An incompatible fixed state conservatively disables native DVFS. */
		if (gpufreq_legacy_dvfs_state(legacy_status_read(dvfs_state), &state))
			return DVFS_DISABLE;
		return state;
	}

	if (g_shared_status)
		dvfs_state = g_shared_status->dvfs_state;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return dvfs_state;
}
EXPORT_SYMBOL(gpufreq_get_dvfs_state);

/***********************************************************************************
 * Function Name      : gpufreq_get_shader_present
 * Inputs             : -
 * Outputs            : -
 * Returns            : shader_present - Current shader cores
 * Description        : Get GPU shader cores
 *                      This is for Mali GPU DDK to control power domain of shader cores
 ***********************************************************************************/
unsigned int gpufreq_get_shader_present(void)
{
	if (g_a16_firmware) return a16_read(0xd8);
	unsigned int shader_present = 0;

	if (g_legacy_firmware)
		return legacy_status_read(shader_present);

	if (g_shared_status)
		shader_present = g_shared_status->shader_present;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return shader_present;
}
EXPORT_SYMBOL(gpufreq_get_shader_present);

/***********************************************************************************
 * Function Name      : gpufreq_get_segment_id
 * Inputs             : -
 * Outputs            : -
 * Returns            : segment_id - Segment of GPU
 * Description        : Get GPU segment ID
 ***********************************************************************************/
unsigned int gpufreq_get_segment_id(void)
{
	if (g_a16_firmware) return a16_read(0x114);
	unsigned int segment_id = 0;

	if (g_legacy_firmware) {
		if (!smp_load_acquire(&g_legacy_tables_ready))
			return 0;
		return g_legacy_debug_opp[TARGET_GPU - TARGET_GPU].segment;
	}

	if (g_shared_status)
		segment_id = g_shared_status->segment_id;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return segment_id;
}
EXPORT_SYMBOL(gpufreq_get_segment_id);

/***********************************************************************************
 * Function Name      : gpufreq_get_checker_result
 * Description        : Get checker result
 ***********************************************************************************/
unsigned int gpufreq_get_checker_result(void)
{
	if (g_a16_firmware) return a16_read(0x13c);
	unsigned int result = 0;

	if (g_shared_status)
		result = g_shared_status->checker_result;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return result;
}
EXPORT_SYMBOL(gpufreq_get_checker_result);

/***********************************************************************************
 * Function Name      : gpufreq_dump_infra_status
 * Inputs             : -
 * Outputs            : -
 * Returns            : -
 * Description        : Dump GPU related infra status
 ***********************************************************************************/
void gpufreq_dump_infra_status(void)
{
	if (g_a16_firmware) return; /* Generic GPR dump exceeds this SoC resource. */
	gpueb_dump_status(NULL, NULL, 0);
	gpufreq_dump_shared_status(NULL, NULL, 0);
	gpufreq_dump_external_status(NULL, NULL, 0);
	gpufreq_dump_internal_status(NULL, NULL, 0);
}
EXPORT_SYMBOL(gpufreq_dump_infra_status);

/***********************************************************************************
 * Function Name      : gpufreq_dump_infra_status_logbuffer
 * Inputs             : -
 * Outputs            : -
 * Returns            : -
 * Description        : Dump GPU related infra status and write to log buffer
 ***********************************************************************************/
void gpufreq_dump_infra_status_logbuffer(char *log_buf, int *log_len, int log_size)
{
	if (g_a16_firmware) return; /* Generic GPR dump exceeds this SoC resource. */
	gpueb_dump_status(log_buf, log_len, log_size);
	gpufreq_dump_shared_status(log_buf, log_len, log_size);
	gpufreq_dump_external_status(log_buf, log_len, log_size);
	gpufreq_dump_internal_status(log_buf, log_len, log_size);
}
EXPORT_SYMBOL(gpufreq_dump_infra_status_logbuffer);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_freq
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : freq   - Current freq of given target
 * Description        : Query current frequency of the target
 ***********************************************************************************/
unsigned int gpufreq_get_cur_freq(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x44 + (target == TARGET_STACK ? 4 : 0));
    }
	unsigned int freq = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(cur_fstack) :
			legacy_status_read(cur_fgpu);

	if (target == TARGET_STACK && g_shared_status)
		freq = g_shared_status->cur_fstack;
	else if (target == TARGET_GPU && g_shared_status)
		freq = g_shared_status->cur_fgpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return freq;
}
EXPORT_SYMBOL(gpufreq_get_cur_freq);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_out_freq
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : freq   - Current out freq of given target
 * Description        : Query current out frequency of the target
 ***********************************************************************************/
unsigned int gpufreq_get_cur_out_freq(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x4c + (target == TARGET_STACK ? 4 : 0));
    }
	unsigned int freq = 0;

	/* No legacy PRBC/clock-output metric; zero means unavailable. */
	if (g_legacy_firmware)
		return 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (target == TARGET_STACK && g_shared_status)
		freq = g_shared_status->cur_out_fstack;
	else if (target == TARGET_GPU && g_shared_status)
		freq = g_shared_status->cur_out_fgpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return freq;
}
EXPORT_SYMBOL(gpufreq_get_cur_out_freq);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_avg_freq
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : freq   - Current avg freq of given target
 * Description        : Query current avg frequency of the target
 ***********************************************************************************/
unsigned int gpufreq_get_cur_avg_freq(enum gpufreq_target target)
{
    if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(target == TARGET_STACK ? 0x26a0 : 0x4c);
    }
	unsigned int freq = 0;

	/* No legacy PRBC/clock-output metric; zero means unavailable. */
	if (g_legacy_firmware)
		return 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (target == TARGET_STACK && g_shared_status)
		freq = g_shared_status->prbc_info.avg_freq;
	else if (target == TARGET_GPU && g_shared_status)
		freq = g_shared_status->cur_out_fgpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return freq;
}
EXPORT_SYMBOL(gpufreq_get_cur_avg_freq);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_volt
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : volt   - Current volt of given target
 * Description        : Query current voltage of the target
 ***********************************************************************************/
unsigned int gpufreq_get_cur_volt(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x5c + (target == TARGET_STACK ? 4 : 0));
    }
	unsigned int volt = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(cur_vstack) :
			legacy_status_read(cur_vgpu);

	if (target == TARGET_STACK && g_shared_status)
		volt = g_shared_status->cur_vstack;
	else if (target == TARGET_GPU && g_shared_status)
		volt = g_shared_status->cur_vgpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return volt;
}
EXPORT_SYMBOL(gpufreq_get_cur_volt);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_vsram
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : vsram  - Current vsram volt of given target
 * Description        : Query current vsram voltage of the target
 ***********************************************************************************/
unsigned int gpufreq_get_cur_vsram(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x64 + (target == TARGET_STACK ? 4 : 0));
    }
	unsigned int vsram = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(cur_vsram_stack) :
			legacy_status_read(cur_vsram_gpu);

	if (target == TARGET_STACK && g_shared_status)
		vsram = g_shared_status->cur_vsram_stack;
	else if (target == TARGET_GPU && g_shared_status)
		vsram = g_shared_status->cur_vsram_gpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return vsram;
}
EXPORT_SYMBOL(gpufreq_get_cur_vsram);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_power
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : power  - Current power of given target
 * Description        : Query current power of the target
 ***********************************************************************************/
unsigned int gpufreq_get_cur_power(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x7c + (target == TARGET_STACK ? 4 : 0));
    }
	unsigned int power = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(cur_power_stack) :
			legacy_status_read(cur_power_gpu);

	if (target == TARGET_STACK && g_shared_status)
		power = g_shared_status->cur_power_stack;
	else if (target == TARGET_GPU && g_shared_status)
		power = g_shared_status->cur_power_gpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return power;
}
EXPORT_SYMBOL(gpufreq_get_cur_power);

/***********************************************************************************
 * Function Name      : gpufreq_get_max_power
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : power  - Max power of given target in working table
 * Description        : Query maximum power of the target
 ***********************************************************************************/
unsigned int gpufreq_get_max_power(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x84 + (target == TARGET_STACK ? 4 : 0));
    }
	unsigned int power = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(max_power_stack) :
			legacy_status_read(max_power_gpu);

	if (target == TARGET_STACK && g_shared_status)
		power = g_shared_status->max_power_stack;
	else if (target == TARGET_GPU && g_shared_status)
		power = g_shared_status->max_power_gpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return power;
}
EXPORT_SYMBOL(gpufreq_get_max_power);

/***********************************************************************************
 * Function Name      : gpufreq_get_min_power
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : power  - Min power of given target in working table
 * Description        : Query minimum power of the target
 ***********************************************************************************/
unsigned int gpufreq_get_min_power(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x8c + (target == TARGET_STACK ? 4 : 0));
    }
	unsigned int power = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(min_power_stack) :
			legacy_status_read(min_power_gpu);

	if (target == TARGET_STACK && g_shared_status)
		power = g_shared_status->min_power_stack;
	else if (target == TARGET_GPU && g_shared_status)
		power = g_shared_status->min_power_gpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return power;
}
EXPORT_SYMBOL(gpufreq_get_min_power);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_oppidx
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : oppidx - Current working OPP index of given target
 * Description        : Query current working OPP index of the target
 ***********************************************************************************/
int gpufreq_get_cur_oppidx(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0x4 + (target == TARGET_STACK ? 4 : 0));
    }
	int oppidx = -1;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(cur_oppidx_stack) :
			legacy_status_read(cur_oppidx_gpu);

	if (target == TARGET_STACK && g_shared_status)
		oppidx = g_shared_status->cur_oppidx_stack;
	else if (target == TARGET_GPU && g_shared_status)
		oppidx = g_shared_status->cur_oppidx_gpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return oppidx;
}
EXPORT_SYMBOL(gpufreq_get_cur_oppidx);

/***********************************************************************************
 * Function Name      : gpufreq_get_opp_num
 * Inputs             : target  - Target of GPU DVFS (GPU, STACK, DEFAULT)
 * Outputs            : -
 * Returns            : opp_num - # of OPP index in working table of given target
 * Description        : Query number of OPP index in working table of the target
 ***********************************************************************************/
int gpufreq_get_opp_num(enum gpufreq_target target)
{
	if (g_a16_firmware) {
        if (gpufreq_validate_target(&target)) return 0;
        return a16_read(0xc + (target == TARGET_STACK ? 4 : 0));
    }
	int opp_num = -1;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware)
		return target == TARGET_STACK ? legacy_status_read(opp_num_stack) :
			legacy_status_read(opp_num_gpu);

	if (target == TARGET_STACK && g_shared_status)
		opp_num = g_shared_status->opp_num_stack;
	else if (target == TARGET_GPU && g_shared_status)
		opp_num = g_shared_status->opp_num_gpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return opp_num;
}
EXPORT_SYMBOL(gpufreq_get_opp_num);

/***********************************************************************************
 * Function Name      : gpufreq_get_freq_by_idx
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      oppidx - Worknig OPP index of the target
 * Outputs            : -
 * Returns            : freq   - Freq of given target at given working OPP index
 * Description        : Query freq of the target by working OPP index
 ***********************************************************************************/
unsigned int gpufreq_get_freq_by_idx(enum gpufreq_target target, int oppidx)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	unsigned int freq = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_GET_FREQ_BY_IDX;
		send_msg.target = target;
		send_msg.u.oppidx = oppidx;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			freq = g_recv_msg.u.freq;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (target == TARGET_STACK && gpufreq_fp && gpufreq_fp->get_fstack_by_idx)
		freq = gpufreq_fp->get_fstack_by_idx(oppidx);
	else if (target == TARGET_GPU && gpufreq_fp && gpufreq_fp->get_fgpu_by_idx)
		freq = gpufreq_fp->get_fgpu_by_idx(oppidx);
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");

done:
	return freq;
}
EXPORT_SYMBOL(gpufreq_get_freq_by_idx);

/***********************************************************************************
 * Function Name      : gpufreq_get_power_by_idx
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      oppidx - Worknig OPP index of the target
 * Outputs            : -
 * Returns            : power  - Power of given target at given working OPP index
 * Description        : Query volt of the target by working OPP index
 ***********************************************************************************/
unsigned int gpufreq_get_power_by_idx(enum gpufreq_target target, int oppidx)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	unsigned int power = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_GET_POWER_BY_IDX;
		send_msg.target = target;
		send_msg.u.oppidx = oppidx;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			power = g_recv_msg.u.power;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (target == TARGET_STACK && gpufreq_fp && gpufreq_fp->get_pstack_by_idx)
		power = gpufreq_fp->get_pstack_by_idx(oppidx);
	else if (target == TARGET_GPU && gpufreq_fp && gpufreq_fp->get_pgpu_by_idx)
		power = gpufreq_fp->get_pgpu_by_idx(oppidx);
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");

done:
	return power;
}
EXPORT_SYMBOL(gpufreq_get_power_by_idx);

/***********************************************************************************
 * Function Name      : gpufreq_get_oppidx_by_freq
 * Inputs             : target - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      freq   - Freq of the target
 * Outputs            : -
 * Returns            : oppidx - Working OPP index of given target that has given freq
 * Description        : Query working OPP index of the target that has the frequency
 ***********************************************************************************/
int gpufreq_get_oppidx_by_freq(enum gpufreq_target target, unsigned int freq)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int oppidx = -1;

	if (gpufreq_validate_target(&target))
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_GET_OPPIDX_BY_FREQ;
		send_msg.target = target;
		send_msg.u.freq = freq;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			oppidx = g_recv_msg.u.oppidx;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (target == TARGET_STACK && gpufreq_fp && gpufreq_fp->get_idx_by_fstack)
		oppidx = gpufreq_fp->get_idx_by_fstack(freq);
	else if (target == TARGET_GPU && gpufreq_fp && gpufreq_fp->get_idx_by_fgpu)
		oppidx = gpufreq_fp->get_idx_by_fgpu(freq);
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");

done:
	return oppidx;
}
EXPORT_SYMBOL(gpufreq_get_oppidx_by_freq);

/***********************************************************************************
 * Function Name      : gpufreq_get_leakage_power
 * Inputs             : target    - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      volt      - Voltage of the target
 * Outputs            : -
 * Returns            : p_leakage - Leakage power of given target computed by given volt
 * Description        : Compute leakage power of the target by the voltage
 ***********************************************************************************/
unsigned int gpufreq_get_leakage_power(enum gpufreq_target target, unsigned int volt)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	unsigned int p_leakage = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_GET_LEAKAGE_POWER;
		send_msg.target = target;
		send_msg.u.volt = volt;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			p_leakage = g_recv_msg.u.power;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (target == TARGET_STACK && gpufreq_fp && gpufreq_fp->get_lkg_pstack)
		p_leakage = gpufreq_fp->get_lkg_pstack(volt, 30);
	else if (target == TARGET_GPU && gpufreq_fp && gpufreq_fp->get_lkg_pgpu)
		p_leakage = gpufreq_fp->get_lkg_pgpu(volt, 30);
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");

done:
	return p_leakage;
}
EXPORT_SYMBOL(gpufreq_get_leakage_power);

/***********************************************************************************
 * Function Name      : gpufreq_get_dynamic_power
 * Inputs             : target    - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      freq      - Frequency of the target
 *                      volt      - Voltage of the target
 * Outputs            : -
 * Returns            : p_dynamic - Dynamic power of given target computed by given freq
 *                                  and volt
 * Description        : Compute dynamic power of the target by the frequency and voltage
 ***********************************************************************************/
unsigned int gpufreq_get_dynamic_power(enum gpufreq_target target,
	unsigned int freq, unsigned int volt)
{
	unsigned int p_dynamic = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	/* implement on AP */
	if (target == TARGET_STACK && gpufreq_fp && gpufreq_fp->get_dyn_pstack)
		p_dynamic = gpufreq_fp->get_dyn_pstack(freq, volt);
	else if (target == TARGET_GPU && gpufreq_fp && gpufreq_fp->get_dyn_pgpu)
		p_dynamic = gpufreq_fp->get_dyn_pgpu(freq, volt);
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");

done:
	return p_dynamic;
}
EXPORT_SYMBOL(gpufreq_get_dynamic_power);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_temperature
 * Inputs             : -
 * Outputs            : -
 * Returns            : temper - Current temperature of GPU
 * Description        : Query current temperature of GPU
 ***********************************************************************************/
int gpufreq_get_cur_temperature(void)
{
	if (g_a16_firmware) return (s32)a16_read(0x30);
	int temper = -274;

	/* Legacy debug temperature_compensation is a policy flag, not Celsius. */
	if (g_legacy_firmware)
		return temper;

	if (g_shared_status)
		temper = g_shared_status->temperature;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

	return temper;
}
EXPORT_SYMBOL(gpufreq_get_cur_temperature);

/***********************************************************************************
 * Function Name      : gpufreq_power_control
 * Inputs             : power          - Target power state
 * Outputs            : -
 * Returns            : power_count    - Success
 *                      GPUFREQ_EINVAL - Failure
 *                      GPUFREQ_ENOENT - Null implementation
 * Description        : Control power state of whole MFG system
 ***********************************************************************************/
int gpufreq_power_control(enum gpufreq_power_state power)
{
	unsigned long power_flags;

	unsigned long ipi_flags;	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	GPUFREQ_TRACE_START("power=%d", power);

	if (!gpufreq_power_ctrl_enable()) {
		GPUFREQ_LOGD("power control is disabled");
		ret = GPUFREQ_SUCCESS;
		goto done;
	}

	raw_spin_lock_irqsave(&gpufreq_power_lock, power_flags);

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_POWER_CONTROL;
		send_msg.u.power_state = power;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done_unlock;
	}

	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->power_control)
		ret = gpufreq_fp->power_control(power);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
	}

done_unlock:
	if (unlikely(ret < 0))
		GPUFREQ_LOGE("fail to control power state: %s (%d)",
			power ? "GPU_PWR_ON" : "GPU_PWR_OFF", ret);

	raw_spin_unlock_irqrestore(&gpufreq_power_lock, power_flags);

done:
	GPUFREQ_TRACE_END();

	return ret;
}
EXPORT_SYMBOL(gpufreq_power_control);

/***********************************************************************************
 * Function Name      : gpufreq_active_sleep_control
 * Inputs             : power          - Target power state
 * Outputs            : -
 * Returns            : active_count   - Success
 *                      GPUFREQ_EINVAL - Failure
 *                      GPUFREQ_ENOENT - Null implementation
 * Description        : Control runtime active-idle state of GPU
 ***********************************************************************************/
int gpufreq_active_sleep_control(enum gpufreq_power_state power)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	if (g_legacy_firmware)
		return -EOPNOTSUPP;

	GPUFREQ_TRACE_START("power=%d", power);

	if (!gpufreq_active_sleep_ctrl_enable()) {
		GPUFREQ_LOGD("active-idle control is disabled");
		ret = GPUFREQ_SUCCESS;
		goto done;
	}

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_ACTIVE_SLEEP_CONTROL;
		send_msg.u.power_state = power;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->active_sleep_control) {
		ret = gpufreq_fp->active_sleep_control(power);
	} else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
	}

done:
	if (unlikely(ret < 0))
		GPUFREQ_LOGE("fail to control active-idle state: %s (%d)",
			power ? "ACTIVE" : "IDLE", ret);

	GPUFREQ_TRACE_END();

	return ret;
}
EXPORT_SYMBOL(gpufreq_active_sleep_control);

/***********************************************************************************
 * Function Name      : gpufreq_commit
 * Inputs             : target          - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      oppidx          - Working OPP index of the target
 * Outputs            : -
 * Returns            : GPUFREQ_SUCCESS - Success
 *                      GPUFREQ_EINVAL  - Failure
 *                      GPUFREQ_ENOENT  - Null implementation
 * Description        : Commit DVFS request to the target with working OPP index
 *                      It will be constrained by the limit recorded in GPU PPM
 ***********************************************************************************/
int gpufreq_commit(enum gpufreq_target target, int oppidx)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	GPUFREQ_TRACE_START("target=%d, oppidx=%d", target, oppidx);

	ret = gpufreq_validate_target(&target);
	if (ret)
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_COMMIT;
		send_msg.target = target;
		send_msg.u.oppidx = oppidx;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (gpuppm_fp && gpuppm_fp->limited_commit)
		ret = gpuppm_fp->limited_commit(target, oppidx);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpuppm platform function pointer (ENOENT)");
	}

done:
	if (unlikely(ret))
		GPUFREQ_LOGE("fail to commit %s OPP index: %d (%d)",
			target == TARGET_STACK ? "STACK" : "GPU", oppidx, ret);

	GPUFREQ_TRACE_END();

	return ret;
}
EXPORT_SYMBOL(gpufreq_commit);

/***********************************************************************************
 * Function Name      : gpufreq_dual_commit
 * Inputs             : gpu_oppidx      - Working OPP index of GPU
 *                      stack_oppidx    - Working OPP index of STACK
 * Outputs            : -
 * Returns            : GPUFREQ_SUCCESS - Success
 *                      GPUFREQ_EINVAL  - Failure
 *                      GPUFREQ_ENOENT  - Null implementation
 * Description        : Commit DVFS request to both GPU & STACK with working OPP index
 *                      It will be constrained by the limit recorded in GPU PPM
 ***********************************************************************************/
int gpufreq_dual_commit(int gpu_oppidx, int stack_oppidx)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	if (g_legacy_firmware)
		return -EOPNOTSUPP;

	GPUFREQ_TRACE_START("gpu_oppidx=%d, stack_oppidx=%d", gpu_oppidx, stack_oppidx);

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_DUAL_COMMIT;
		send_msg.u.dual_commit.gpu_oppidx = gpu_oppidx;
		send_msg.u.dual_commit.stack_oppidx = stack_oppidx;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->update_temperature)
		gpufreq_fp->update_temperature();
	if (gpuppm_fp && gpuppm_fp->limited_dual_commit)
		ret = gpuppm_fp->limited_dual_commit(gpu_oppidx, stack_oppidx);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpuppm platform function pointer (ENOENT)");
	}

done:
	if (unlikely(ret))
		GPUFREQ_LOGE("fail to commit GPU/STACK OPP index: %d/%d (%d)",
			gpu_oppidx, stack_oppidx, ret);

	GPUFREQ_TRACE_END();

	return ret;
}
EXPORT_SYMBOL(gpufreq_dual_commit);

/***********************************************************************************
 * Function Name      : gpufreq_set_limit
 * Inputs             : target          - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      limiter         - Pre-defined user that set limit to GPU DVFS
 *                      ceiling_info    - Upper limit info (oppidx, freq, volt, power, ...)
 *                      floor_info      - Lower limit info (oppidx, freq, volt, power, ...)
 * Outputs            : -
 * Returns            : GPUFREQ_SUCCESS - Success
 *                      GPUFREQ_EINVAL  - Failure
 *                      GPUFREQ_ENOENT  - Null implementation
 * Description        : Set ceiling and floor limit to GPU DVFS by specified limiter
 *                      It will immediately trigger DVFS if current OPP violates limit
 ***********************************************************************************/
int gpufreq_set_limit(enum gpufreq_target target,
	enum gpuppm_limiter limiter, int ceiling_info, int floor_info)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	GPUFREQ_TRACE_START("target=%d, limiter=%d, ceiling_info=%d, floor_info=%d",
		target, limiter, ceiling_info, floor_info);

	ret = gpufreq_validate_target(&target);
	if (ret)
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_SET_LIMIT;
		send_msg.target = target;
		send_msg.u.set_limit.limiter = limiter;
		send_msg.u.set_limit.ceiling_info = ceiling_info;
		send_msg.u.set_limit.floor_info = floor_info;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (gpuppm_fp && gpuppm_fp->set_limit)
		ret = gpuppm_fp->set_limit(target, limiter, ceiling_info, floor_info, true);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpuppm platform function pointer (ENOENT)");
	}

done:
	if (unlikely(ret))
		GPUFREQ_LOGE("fail to set %s limiter: %d, ceiling_info: %d, floor_info: %d (%d)",
			target == TARGET_STACK ? "STACK" : "GPU",
			limiter, ceiling_info, floor_info, ret);

	GPUFREQ_TRACE_END();

	return ret;
}
EXPORT_SYMBOL(gpufreq_set_limit);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_limit_idx
 * Inputs             : target    - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      limit     - Type of limit (GPUPPM_CEILING, GPUPPM_FLOOR)
 * Outputs            : -
 * Returns            : limit_idx - Working OPP index of ceiling or floor
 * Description        : Query current working OPP index of ceiling or floor limit
 ***********************************************************************************/
int gpufreq_get_cur_limit_idx(enum gpufreq_target target, enum gpuppm_limit_type limit)
{
    if (g_a16_firmware) {
        if (gpufreq_validate_target(&target) || target != TARGET_STACK ||
            limit < 0 || limit >= GPUPPM_INVALID) return -EOPNOTSUPP;
        return a16_read(limit == GPUPPM_CEILING ? 0xb4 : 0xb8);
    }
	int limit_idx = -1;

	if (gpufreq_validate_target(&target))
		goto done;

	if (limit >= GPUPPM_INVALID || limit < 0) {
		GPUFREQ_LOGE("invalid limit target: %d (EINVAL)", limit);
		goto done;
	}

	if (g_legacy_firmware) {
		if (target == TARGET_STACK)
			return limit == GPUPPM_CEILING ? legacy_status_read(cur_ceiling_stack) :
				legacy_status_read(cur_floor_stack);
		return limit == GPUPPM_CEILING ? legacy_status_read(cur_ceiling_gpu) :
			legacy_status_read(cur_floor_gpu);
	}

	if (g_shared_status) {
		if (limit == GPUPPM_CEILING)
			limit_idx = g_shared_status->cur_ceiling;
		else if (limit == GPUPPM_FLOOR)
			limit_idx = g_shared_status->cur_floor;
	} else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return limit_idx;
}
EXPORT_SYMBOL(gpufreq_get_cur_limit_idx);

/***********************************************************************************
 * Function Name      : gpufreq_get_cur_limiter
 * Inputs             : target  - Target of GPU DVFS (GPU, STACK, DEFAULT)
 *                      limit   - Type of limit (GPUPPM_CEILING, GPUPPM_FLOOR)
 * Outputs            : -
 * Returns            : limiter - Pre-defined user that set limit to GPU DVFS
 * Description        : Query which user decide current ceiling and floor OPP index
 ***********************************************************************************/
unsigned int gpufreq_get_cur_limiter(enum gpufreq_target target, enum gpuppm_limit_type limit)
{
    if (g_a16_firmware) {
        if (gpufreq_validate_target(&target) || target != TARGET_STACK ||
            limit < 0 || limit >= GPUPPM_INVALID) return LIMIT_NUM;
        return a16_read(limit == GPUPPM_CEILING ? 0xbc : 0xc0);
    }
	unsigned int limiter = 0;

	if (gpufreq_validate_target(&target))
		goto done;

	if (limit >= GPUPPM_INVALID || limit < 0) {
		GPUFREQ_LOGE("invalid limit target: %d (EINVAL)", limit);
		goto done;
	}

	if (g_legacy_firmware) {
		unsigned int raw;

		if (target == TARGET_STACK)
			raw = limit == GPUPPM_CEILING ? legacy_status_read(cur_c_limiter_stack) :
				legacy_status_read(cur_f_limiter_stack);
		else
			raw = limit == GPUPPM_CEILING ? legacy_status_read(cur_c_limiter_gpu) :
				legacy_status_read(cur_f_limiter_gpu);
		if (gpufreq_native_limiter(raw, &limiter))
			return LIMIT_NUM;
		return limiter;
	}

	if (g_shared_status) {
		if (limit == GPUPPM_CEILING)
			limiter = g_shared_status->cur_c_limiter;
		else if (limit == GPUPPM_FLOOR)
			limiter = g_shared_status->cur_f_limiter;
	} else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return limiter;
}
EXPORT_SYMBOL(gpufreq_get_cur_limiter);

/***********************************************************************************
 * Function Name      : gpufreq_get_core_mask_table
 * Inputs             : -
 * Outputs            : -
 * Returns            : core_mask_table  - Pointer to array of core mask structure
 * Description        : Get core mask table
 *                      This is for DCS to scale the number of shader cores
 ***********************************************************************************/
struct gpufreq_core_mask_info *gpufreq_get_core_mask_table(void)
{
	struct gpufreq_core_mask_info *core_mask_table = NULL;

	/* implement only on AP */
	if (gpufreq_fp && gpufreq_fp->get_core_mask_table)
		core_mask_table = gpufreq_fp->get_core_mask_table();
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");

	return core_mask_table;
}
EXPORT_SYMBOL(gpufreq_get_core_mask_table);

/***********************************************************************************
 * Function Name      : gpufreq_get_core_num
 * Inputs             : -
 * Outputs            : -
 * Returns            : core_num  - # of GPU Shader cores
 * Description        : Get number of GPU shader cores
 *                      This is for DCS to scale the number of shader cores
 ***********************************************************************************/
unsigned int gpufreq_get_core_num(void)
{
	unsigned int core_num = 0;

	/* implement only on AP */
	if (gpufreq_fp && gpufreq_fp->get_core_num)
		core_num = gpufreq_fp->get_core_num();
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");

	return core_num;
}
EXPORT_SYMBOL(gpufreq_get_core_num);

/***********************************************************************************
 * Function Name      : gpufreq_pdca_config
 * Inputs             : power          - Target power state
 * Outputs            : -
 * Returns            : -
 * Description        : Manually control PDCA setting
 ***********************************************************************************/
void gpufreq_pdca_config(enum gpufreq_power_state power)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_PDCA_CONFIG;
		send_msg.u.power = power;

		ret = gpufreq_ipi_to_gpueb(send_msg);
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
	/* implement on AP */
	} else {
		if (gpufreq_fp && gpufreq_fp->pdca_config)
			gpufreq_fp->pdca_config(power);
		else
			GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
	}
}
EXPORT_SYMBOL(gpufreq_pdca_config);

/***********************************************************************************
 * Function Name      : gpufreq_fake_mtcmos_control
 * Inputs             : power          - Target power state
 * Outputs            : -
 * Returns            : -
 * Description        : Fake PWR_CON value of SPM MFG register
 ***********************************************************************************/
void gpufreq_fake_mtcmos_control(enum gpufreq_power_state power)
{
	if (power == GPU_PWR_ON)
		gpufreq_set_mfgsys_config(CONFIG_FAKE_MTCMOS_CTRL, FEAT_ENABLE);
	else if (power == GPU_PWR_OFF)
		gpufreq_set_mfgsys_config(CONFIG_FAKE_MTCMOS_CTRL, FEAT_DISABLE);
}
EXPORT_SYMBOL(gpufreq_fake_mtcmos_control);

/***********************************************************************************
 * Function Name      : gpufreq_update_debug_opp_info
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
int gpufreq_update_debug_opp_info(void)
{
	if (g_a16_firmware) return -EOPNOTSUPP;
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	if (g_legacy_firmware) {
		struct gpufreq_legacy_debug_opp snapshot;

		/* Validate/refresh only the firmware-owned snapshot. The native
		 * debug module must not interpret the legacy shared layout.
		 */
		ret = gpufreq_legacy_debug_snapshot(TARGET_GPU, &snapshot);
		if (!ret && g_dual_buck)
			ret = gpufreq_legacy_debug_snapshot(TARGET_STACK, &snapshot);
		return ret;
	}

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_UPDATE_DEBUG_OPP_INFO;

		ret = gpufreq_ipi_to_gpueb(send_msg);
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
	/* implement on AP */
	} else {
		if (gpufreq_fp && gpufreq_fp->update_debug_opp_info)
			gpufreq_fp->update_debug_opp_info();
		else {
			ret = GPUFREQ_ENOENT;
			GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
		}
	}

	return ret;
}

/***********************************************************************************
 * Function Name      : gpufreq_get_working_table
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
const struct gpufreq_opp_info *gpufreq_get_working_table(enum gpufreq_target target)
{
    if (g_a16_firmware) {
        if (gpufreq_validate_target(&target) || !smp_load_acquire(&g_a16_tables_ready))
            return NULL;
        return g_a16_working[target - TARGET_GPU];
    }
	const struct gpufreq_opp_info *opp_table = NULL;

	if (gpufreq_validate_target(&target))
		goto done;

	if (g_legacy_firmware) {
		if (!smp_load_acquire(&g_legacy_tables_ready))
			return NULL;
		return g_legacy_working[target - TARGET_GPU];
	}

	if (target == TARGET_STACK && g_shared_status)
		opp_table = g_shared_status->working_table_stack;
	else if (target == TARGET_GPU && g_shared_status)
		opp_table = g_shared_status->working_table_gpu;
	else
		GPUFREQ_LOGE("null gpufreq shared memory (ENOENT)");

done:
	return opp_table;
}
EXPORT_SYMBOL(gpufreq_get_working_table);

/***********************************************************************************
 * Function Name      : gpufreq_switch_limit
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
int gpufreq_switch_limit(enum gpufreq_target target,
	enum gpuppm_limiter limiter, int c_enable, int f_enable)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	ret = gpufreq_validate_target(&target);
	if (ret)
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_SWITCH_LIMIT;
		send_msg.target = target;
		send_msg.u.set_limit.limiter = limiter;
		send_msg.u.set_limit.ceiling_info = c_enable;
		send_msg.u.set_limit.floor_info = f_enable;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (gpuppm_fp && gpuppm_fp->switch_limit)
		ret = gpuppm_fp->switch_limit(target, limiter, c_enable, f_enable, true);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpuppm platform function pointer (ENOENT)");
	}

done:
	if (unlikely(ret))
		GPUFREQ_LOGE("fail to switch %s limiter: %d, c_enable: %d, f_enable: %d (%d)",
			target == TARGET_STACK ? "STACK" : "GPU",
			limiter, c_enable, f_enable, ret);

	GPUFREQ_TRACE_END();

	return ret;
}

/***********************************************************************************
 * Function Name      : gpufreq_fix_target_oppidx
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
int gpufreq_fix_target_oppidx(enum gpufreq_target target, int oppidx)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	GPUFREQ_TRACE_START("target=%d, oppidx=%d", target, oppidx);

	ret = gpufreq_validate_target(&target);
	if (ret)
		goto done;

	/* notify DCS Policy */
	if (ged_notify_dcs_fix_opp_fp)
		ged_notify_dcs_fix_opp_fp(oppidx, oppidx);

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_FIX_TARGET_OPPIDX;
		send_msg.target = target;
		send_msg.u.oppidx = oppidx;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (target == TARGET_STACK && gpufreq_fp && gpufreq_fp->fix_target_oppidx_stack)
		ret = gpufreq_fp->fix_target_oppidx_stack(oppidx);
	else if (target == TARGET_GPU && gpufreq_fp && gpufreq_fp->fix_target_oppidx_gpu)
		ret = gpufreq_fp->fix_target_oppidx_gpu(oppidx);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
	}

done:
	/* notify DVFS Policy */
	if (likely(!ret) && ged_notify_gpu_fix_opp_fp)
		ged_notify_gpu_fix_opp_fp(oppidx, oppidx);

	if (unlikely(ret))
		GPUFREQ_LOGE("fail to fix %s OPP index: %d (%d)",
			target == TARGET_STACK ? "STACK" : "GPU", oppidx, ret);

	GPUFREQ_TRACE_END();

	return ret;
}

/***********************************************************************************
 * Function Name      : gpufreq_fix_dual_target_oppidx
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
int gpufreq_fix_dual_target_oppidx(int gpu_oppidx, int stack_oppidx)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	if (g_legacy_firmware)
		return -EOPNOTSUPP;

	GPUFREQ_TRACE_START("gpu_oppidx=%d, stack_oppidx=%d", gpu_oppidx, stack_oppidx);

	/* notify DCS Policy */
	if (ged_notify_dcs_fix_opp_fp)
		ged_notify_dcs_fix_opp_fp(gpu_oppidx, stack_oppidx);

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_FIX_DUAL_TARGET_OPPIDX;
		send_msg.u.dual_commit.gpu_oppidx = gpu_oppidx;
		send_msg.u.dual_commit.stack_oppidx = stack_oppidx;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->fix_target_oppidx_dual)
		ret = gpufreq_fp->fix_target_oppidx_dual(gpu_oppidx, stack_oppidx);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
	}

done:
	/* notify DVFS Policy */
	if (likely(!ret) && ged_notify_gpu_fix_opp_fp)
		ged_notify_gpu_fix_opp_fp(gpu_oppidx, stack_oppidx);

	if (unlikely(ret))
		GPUFREQ_LOGE("fail to fix GPU/STACK OPP index: %d/%d (%d)",
			gpu_oppidx, stack_oppidx, ret);

	GPUFREQ_TRACE_END();

	return ret;
}

/***********************************************************************************
 * Function Name      : gpufreq_fix_custom_freq_volt
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
int gpufreq_fix_custom_freq_volt(enum gpufreq_target target,
	unsigned int freq, unsigned int volt)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	GPUFREQ_TRACE_START("target=%d, freq=%d, volt=%d", target, freq, volt);

	ret = gpufreq_validate_target(&target);
	if (ret)
		goto done;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_FIX_CUSTOM_FREQ_VOLT;
		send_msg.target = target;
		send_msg.u.custom.freq = freq;
		send_msg.u.custom.volt = volt;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (target == TARGET_STACK && gpufreq_fp && gpufreq_fp->fix_custom_freq_volt_stack)
		ret = gpufreq_fp->fix_custom_freq_volt_stack(freq, volt);
	else if (target == TARGET_GPU && gpufreq_fp && gpufreq_fp->fix_custom_freq_volt_gpu)
		ret = gpufreq_fp->fix_custom_freq_volt_gpu(freq, volt);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
	}

done:
	/* notify DVFS Policy */
	if (likely(!ret) && ged_notify_gpu_fix_freq_volt_fp)
		ged_notify_gpu_fix_freq_volt_fp(freq, volt, freq, volt);

	if (unlikely(ret))
		GPUFREQ_LOGE("fail to fix %s F(%d)/V(%d) (%d)",
			target == TARGET_STACK ? "STACK" : "GPU", freq, volt, ret);

	GPUFREQ_TRACE_END();

	return ret;
}

/***********************************************************************************
 * Function Name      : gpufreq_fix_dual_custom_freq_volt
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
int gpufreq_fix_dual_custom_freq_volt(unsigned int fgpu, unsigned int vgpu,
	unsigned int fstack, unsigned int vstack)
{
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	if (g_legacy_firmware)
		return -EOPNOTSUPP;

	GPUFREQ_TRACE_START("fgpu=%d, vgpu=%d, fstack=%d, vstack=%d", fgpu, vgpu, fstack, vstack);

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_FIX_DUAL_CUSTOM_FREQ_VOLT;
		send_msg.u.dual_custom.fgpu = fgpu;
		send_msg.u.dual_custom.vgpu = vgpu;
		send_msg.u.dual_custom.fstack = fstack;
		send_msg.u.dual_custom.vstack = vstack;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
		goto done;
	}

	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->fix_custom_freq_volt_dual)
		ret = gpufreq_fp->fix_custom_freq_volt_dual(fgpu, vgpu, fstack, vstack);
	else {
		ret = GPUFREQ_ENOENT;
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
	}

done:
	/* notify DVFS Policy */
	if (likely(!ret) && ged_notify_gpu_fix_freq_volt_fp)
		ged_notify_gpu_fix_freq_volt_fp(fgpu, vgpu, fstack, vstack);

	if (unlikely(ret))
		GPUFREQ_LOGE("fail to fix GPU F(%d)/V(%d), STACK F(%d)/V(%d) (%d)",
			fgpu, vgpu, fstack, vstack, ret);

	GPUFREQ_TRACE_END();

	return ret;
}

/***********************************************************************************
 * Function Name      : gpufreq_set_mfgsys_config
 * Description        : Only for GPUFREQ internal debug purpose
 ***********************************************************************************/
int gpufreq_set_mfgsys_config(enum gpufreq_config_target target, enum gpufreq_config_value val)
{
	if (g_a16_firmware) return -EOPNOTSUPP;
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	if (g_legacy_firmware)
		return -EOPNOTSUPP;

	/* implement on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready && target < CONFIG_AP_IMPL_BOUNDARY) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_SET_MFGSYS_CONFIG;
		send_msg.u.mfg_cfg.target = target;
		send_msg.u.mfg_cfg.val = val;

		ret = gpufreq_ipi_to_gpueb(send_msg);
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
	/* implement on AP */
	} else {
		if (gpufreq_fp && gpufreq_fp->set_mfgsys_config)
			gpufreq_fp->set_mfgsys_config(target, val);
		else {
			ret = GPUFREQ_ENOENT;
			GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
		}
	}

	return ret;
}
EXPORT_SYMBOL(gpufreq_set_mfgsys_config);

/***********************************************************************************
 * Function Name      : gpufreq_mssv_commit
 * Description        : Only for GPUFREQ MSSV test purpose
 ***********************************************************************************/
int gpufreq_mssv_commit(unsigned int target, unsigned int val)
{
#if GPUFREQ_MSSV_TEST_MODE
	unsigned long ipi_flags;
	struct gpufreq_ipi_data send_msg = {};
	int ret = GPUFREQ_SUCCESS;

	/* implement only on EB */
	if (g_gpueb_support && g_gpufreq_wrapper_ready) {
		raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
		send_msg.cmd_id = CMD_MSSV_COMMIT;
		send_msg.u.mssv.target = target;
		send_msg.u.mssv.val = val;

		if (!gpufreq_ipi_to_gpueb(send_msg))
			ret = g_recv_msg.u.return_value;
		else
			ret = GPUFREQ_EINVAL;
		raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);
	/* implement on AP */
	} else {
		if (gpufreq_fp && gpufreq_fp->mssv_commit)
			ret = gpufreq_fp->mssv_commit(target, val);
		else {
			ret = GPUFREQ_ENOENT;
			GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
		}
	}

	return ret;
#else
	GPUFREQ_UNREFERENCED(target);
	GPUFREQ_UNREFERENCED(val);

	return GPUFREQ_EINVAL;
#endif /* GPUFREQ_MSSV_TEST_MODE */
}

/***********************************************************************************
 * Function Name      : gpufreq_ipi_to_gpueb
 * Description        : Only for GPUFREQ internal IPI purpose
 ***********************************************************************************/
static int gpufreq_ipi_to_gpueb(struct gpufreq_ipi_data data)
{
	if (g_a16_firmware) return gpufreq_a16_transaction(data);
	int ret = GPUFREQ_SUCCESS;

	if (data.cmd_id < 0 || data.cmd_id >= CMD_NUM) {
		GPUFREQ_LOGE("invalid gpufreq IPI command: %d (EINVAL)", data.cmd_id);
		ret = GPUFREQ_EINVAL;
		goto done;
	}

	if (g_legacy_firmware)
		return gpufreq_legacy_transaction(&data);

	/* apply IPI magic */
	data.magic = g_ipi_magic;

	GPUFREQ_LOGD("channel: %d send IPI command: %s (%d)",
		g_ipi_channel, GPUFREQ_IPI_CMD_STRING(data.cmd_id), data.cmd_id);

	ret = mtk_ipi_send_compl_to_gpueb(g_ipi_channel, IPI_SEND_POLLING,
		(void *)&data, GPUFREQ_IPI_DATA_LEN, IPI_TIMEOUT_MS);
	if (unlikely(ret != IPI_ACTION_DONE)) {
		GPUFREQ_LOGE("[ABORT] fail to send IPI channel: %d, command: %s (%d)",
			g_ipi_channel, GPUFREQ_IPI_CMD_STRING(data.cmd_id), ret);
		gpufreq_abort();
		goto done;
	}

	ret = GPUFREQ_SUCCESS;

	GPUFREQ_LOGD("channel: %d receive IPI command: %s (%d)",
		g_ipi_channel, GPUFREQ_IPI_CMD_STRING(g_recv_msg.cmd_id), g_recv_msg.cmd_id);

done:
	return ret;
}

/***********************************************************************************
 * Function Name      : gpufreq_validate_target
 * Description        : Validate gpufreq target and re-assign default target
 ***********************************************************************************/
static int gpufreq_validate_target(unsigned int *target)
{
	if (*target >= TARGET_INVALID || (*target == TARGET_STACK && !g_dual_buck)) {
		GPUFREQ_LOGE("invalid OPP target: %d (EINVAL)", *target);
		return GPUFREQ_EINVAL;
	}

	if (*target == TARGET_DEFAULT) {
		if (g_dual_buck)
			*target = TARGET_STACK;
		else
			*target = TARGET_GPU;
	}

	return GPUFREQ_SUCCESS;
}

/***********************************************************************************
 * Function Name      : gpufreq_dump_external_status
 * Description        : Dump MFGSYS external debug info
 ***********************************************************************************/
static void gpufreq_dump_external_status(char *log_buf, int *log_len, int log_size)
{
	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->dump_external_status)
		gpufreq_fp->dump_external_status(log_buf, log_len, log_size);
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
}

/***********************************************************************************
 * Function Name      : gpufreq_dump_internal_status
 * Description        : Dump MFGSYS internal debug info, protected by power lock
 ***********************************************************************************/
static void gpufreq_dump_internal_status(char *log_buf, int *log_len, int log_size)
{
	unsigned long power_flags;
	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->dump_internal_status) {
		if (g_abort_state)
			gpufreq_fp->dump_internal_status(log_buf, log_len, log_size);
		else {
			raw_spin_lock_irqsave(&gpufreq_power_lock, power_flags);
			gpufreq_fp->dump_internal_status(log_buf, log_len, log_size);
			raw_spin_unlock_irqrestore(&gpufreq_power_lock, power_flags);
		}
	} else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
}

/***********************************************************************************
 * Function Name      : gpufreq_dump_shared_status
 * Description        : Dump info from shared memory
 ***********************************************************************************/
static void gpufreq_dump_shared_status(char *log_buf, int *log_len, int log_size)
{
	/* implement on AP */
	if (gpufreq_fp && gpufreq_fp->dump_shared_status)
		gpufreq_fp->dump_shared_status(log_buf, log_len, log_size);
	else
		GPUFREQ_LOGE("null gpufreq platform function pointer (ENOENT)");
}

/***********************************************************************************
 * Function Name      : gpufreq_abort
 * Description        : Trigger exception when fatal error and dump infra status
 ***********************************************************************************/
static void gpufreq_abort(void)
{
	g_abort_state = true;
	gpufreq_dump_infra_status();

#if GPUFREQ_FORCE_WDT_ENABLE
	gpueb_trigger_wdt("GPUFREQ");
#else
	BUG_ON(1);
#endif /* GPUFREQ_FORCE_WDT_ENABLE */
}

#if IS_ENABLED(CONFIG_DEVAPC_ARCH_MULTI)
static bool gpufreq_devapc_vio_callback(void)
{
	gpufreq_set_mfgsys_config(CONFIG_DEVAPC_HANDLE, CONFIG_VAL_IGNORE);

	return GPU_PWR_ON;
}
#endif /* CONFIG_DEVAPC_ARCH_MULTI */

#if IS_ENABLED(CONFIG_DEVAPC_ARCH_MULTI)
static bool gpufreq_bus_tracker_vio_callback(int slave_type)
{
	if (gpufreq_fp && gpufreq_fp->bus_tracker_vio_handler)
		return gpufreq_fp->bus_tracker_vio_handler();

	return false;
}
#endif /* CONFIG_DEVAPC_ARCH_MULTI */

static void gpufreq_init_external_callback(void)
{
#if IS_ENABLED(CONFIG_MTK_PBM)
	struct pbm_gpu_callback_table pbm_cb = {
		.get_max_pb = gpufreq_get_max_power,
		.get_min_pb = gpufreq_get_min_power,
		.get_cur_pb = gpufreq_get_cur_power,
		.get_cur_vol = gpufreq_get_cur_volt,
		.set_limit = gpufreq_set_limit,
	};
#endif /* CONFIG_MTK_PBM */

	/* These legacy HAL/PBM/devapc registries expose raw function pointers,
	 * with no owner lease or unregister-and-drain API. Retain this module
	 * once callbacks escape; a reboot is required to replace the provider.
	 * Merely clearing the pointers at remove cannot drain existing callers.
	 */
	if (g_legacy_firmware) {
		mutex_lock(&gpufreq_callback_lock);
		if (!g_legacy_callbacks_pinned) {
			__module_get(THIS_MODULE);
			g_legacy_callbacks_pinned = true;
		}
		mutex_unlock(&gpufreq_callback_lock);
	}

	/* hook GPU HAL callback */
	mtk_get_gpu_limit_index_fp = gpufreq_get_cur_limit_idx;
	mtk_get_gpu_limiter_fp = gpufreq_get_cur_limiter;
	mtk_get_gpu_cur_freq_fp = gpufreq_get_cur_freq;
	mtk_get_gpu_cur_oppidx_fp = gpufreq_get_cur_oppidx;

	/* register PBM callback */
#if IS_ENABLED(CONFIG_MTK_PBM)
	register_pbm_gpu_notify(&pbm_cb);
#endif /* CONFIG_MTK_PBM */

#if IS_ENABLED(CONFIG_DEVAPC_ARCH_MULTI)
	register_devapc_power_callback(&devapc_cb_gpu);
	register_devapc_power_callback(&devapc_cb_gpu1);
	register_devapc_exception_callback(&bus_tracker_cb);
#endif /* CONFIG_DEVAPC_ARCH_MULTI */
}

void gpufreq_register_gpufreq_fp(struct gpufreq_platform_fp *platform_fp)
{
	if (!platform_fp) {
		GPUFREQ_LOGE("null gpufreq platform function pointer (EINVAL)");
		return;
	}

	gpufreq_fp = platform_fp;

	/* init shared status on AP side */
	if (!g_legacy_firmware && !g_a16_firmware && gpufreq_fp && gpufreq_fp->set_shared_status)
		gpufreq_fp->set_shared_status(g_shared_status);
}
EXPORT_SYMBOL(gpufreq_register_gpufreq_fp);

void gpufreq_register_gpuppm_fp(struct gpuppm_platform_fp *platform_fp)
{
	if (!platform_fp) {
		GPUFREQ_LOGE("null gpuppm platform function pointer (EINVAL)");
		return;
	}

	gpuppm_fp = platform_fp;

	/* init shared status on AP side */
	if (!g_legacy_firmware && !g_a16_firmware && gpuppm_fp && gpuppm_fp->set_shared_status)
		gpuppm_fp->set_shared_status(g_shared_status);

	/* all platform & gpuppm impl is ready after gpuppm func pointer registration */
	/* register external callback function at here */
	gpufreq_init_external_callback();

	/* init gpufreq debug */
	if (!g_legacy_firmware && !g_a16_firmware)
		gpufreq_debug_init(g_dual_buck, g_gpueb_support, g_shared_status);

	g_gpufreq_platform_ready = true;
}
EXPORT_SYMBOL(gpufreq_register_gpuppm_fp);

static int gpufreq_shared_memory_init(void)
{
	struct device_node *of_gpueb = NULL;
	int ret = GPUFREQ_SUCCESS;
	phys_addr_t gpueb_mem_pa = 0, gpueb_mem_va = 0, gpueb_mem_size = 0;
	phys_addr_t gpufreq_mem_pa = 0, gpufreq_mem_va = 0;
	unsigned int gpufreq_mem_size = 0;

	/* get pre-allocated gpueb shared memory from dts */
	of_gpueb = of_find_compatible_node(NULL, NULL, "mediatek,gpueb");
	if (unlikely(!of_gpueb)) {
		GPUFREQ_LOGE("fail to find gpueb of_node");
		ret = GPUFREQ_ENOENT;
		goto done;
	}

	if (of_property_present(of_gpueb, "gpueb_mem_table") ||
	    of_property_present(of_gpueb, "gpueb_mem_name_table")) {
		struct gpufreq_legacy_memory memory;
		struct gpufreq_legacy_ipi probe;
		struct gpufreq_ipi_data init = {.cmd_id = CMD_INIT_SHARED_MEM};
		phys_addr_t size;

		if (!g_gpueb_support ||
		    !of_property_present(of_gpueb, "gpueb_mem_table") ||
		    !of_property_present(of_gpueb, "gpueb_mem_name_table") ||
		    of_property_present(of_gpueb, "gpueb-mem-table") ||
		    of_property_present(of_gpueb, "gpueb-mem-name-table")) {
			ret = -EINVAL;
			goto legacy_done;
		}
		gpufreq_mem_va = gpueb_get_reserve_mem_virt_by_name("MEM_ID_GPUFREQ");
		memory.base = gpueb_get_reserve_mem_phys_by_name("MEM_ID_GPUFREQ");
		size = gpueb_get_reserve_mem_size_by_name("MEM_ID_GPUFREQ");
		if (!gpufreq_mem_va || !memory.base || !size) {
			ret = -EPROBE_DEFER;
			goto legacy_done;
		}
		if (size > U32_MAX) {
			ret = -EOVERFLOW;
			goto legacy_done;
		}
		memory.size = size;
		ret = gpufreq_legacy_encode(&init, &memory, &probe);
		if (ret)
			goto legacy_done;
		g_legacy_memory = memory;
		g_legacy_status = (void __iomem *)(uintptr_t)gpufreq_mem_va;
		g_legacy_firmware = true;
legacy_done:
		of_node_put(of_gpueb);
		return ret;
	}

    if (g_a16_firmware) {
        phys_addr_t size = gpueb_get_reserve_mem_size_by_name("MEM_ID_GPUFREQ");
        g_shared_mem_pa = gpueb_get_reserve_mem_phys_by_name("MEM_ID_GPUFREQ");
        g_a16_status = (void __iomem *)(uintptr_t)
            gpueb_get_reserve_mem_virt_by_name("MEM_ID_GPUFREQ");
        if (!g_a16_status || !g_shared_mem_pa) ret = -EPROBE_DEFER;
        else if (size != A16_SHARED_SIZE) ret = -EINVAL;
        else g_shared_mem_size = size;
        of_node_put(of_gpueb);
        return ret;
    }
	of_property_read_u64(of_gpueb, "gpueb_mem_addr", &gpueb_mem_pa);
	if (unlikely(!gpueb_mem_pa)) {
		GPUFREQ_LOGE("fail to get gpueb reserved memory physical address");
		ret = GPUFREQ_ENOENT;
		goto done;
	}

	of_property_read_u64(of_gpueb, "gpueb_mem_size", &gpueb_mem_size);
	if (unlikely(!gpueb_mem_size)) {
		GPUFREQ_LOGE("fail to get gpueb reserved memory size");
		ret = GPUFREQ_ENOENT;
		goto done;
	}

	/* Transfer physical addr to virtual addr */
	gpueb_mem_va = (phys_addr_t)(size_t)ioremap_wc(gpueb_mem_pa, gpueb_mem_size);
	if (unlikely(!gpueb_mem_va)) {
		GPUFREQ_LOGE("fail to get gpueb reserved memory virtual address");
		ret = GPUFREQ_ENOENT;
		goto done;
	}

	/* init gpufreq shared memory from gpueb shared memory */
	of_property_read_u32_index(of_gpueb,
		"gpueb-mem-table", GPUFREQ_MEM_TABLE_IDX, &gpufreq_mem_size);
	if (unlikely(!gpufreq_mem_size)) {
		GPUFREQ_LOGE("fail to get gpufreq reserved memory size");
		ret = GPUFREQ_ENOENT;
		goto done;
	}
	/* gpufreq shared memory start from the beginning of gpueb's */
	gpufreq_mem_pa = gpueb_mem_pa;
	gpufreq_mem_va = gpueb_mem_va;

	g_shared_status = (struct gpufreq_shared_status *)(uintptr_t)gpufreq_mem_va;
	g_shared_mem_pa = gpufreq_mem_pa;
	g_shared_mem_size = gpufreq_mem_size;

	/* init to 0 */
	memset((void *)gpufreq_mem_va, 0, gpufreq_mem_size);

	GPUFREQ_LOGI("shared status memory: 0x%llx (phy_addr: 0x%llx, size: %u, used: %lu)",
		(unsigned long long)g_shared_status, g_shared_mem_pa,
		g_shared_mem_size, sizeof(struct gpufreq_shared_status));

done:
	of_node_put(of_gpueb);
	return ret;
}

static int gpufreq_gpueb_init(void)
{
	if (g_a16_firmware) return gpufreq_a16_gpueb_init();
	unsigned long ipi_flags;
	int ret = GPUFREQ_SUCCESS;
	struct gpufreq_ipi_data send_msg = {};
	void __iomem *gpueb_gpr_addr = NULL;

	if (g_legacy_firmware)
		return gpufreq_legacy_gpueb_init();

	/* power on GPUEB */
	ret = gpueb_ctrl(GHPM_ON, MFG1_OFF, SUSPEND_POWER_ON);
	if (unlikely(ret)) {
		GPUFREQ_LOGE("fail to power on GPUEB (%d)", ret);
		gpufreq_abort();
		goto done;
	}

	/* init ipi channel */
	g_ipi_channel = gpueb_get_send_PIN_ID_by_name("IPI_ID_GPUFREQ");
	if (unlikely(g_ipi_channel < 0)) {
		GPUFREQ_LOGE("fail to get gpufreq ipi channel id");
		ret = GPUFREQ_ENOENT;
		goto done;
	}
	mtk_ipi_register(get_gpueb_ipidev(), g_ipi_channel, NULL, NULL, (void *)&g_recv_msg);

	/* init ipi magic number */
	gpueb_gpr_addr = gpueb_get_gpr_addr(GPUEB_SRAM_GPR13);
	g_ipi_magic = readl(gpueb_gpr_addr);
	if (unlikely(!g_ipi_magic)) {
		GPUFREQ_LOGE("fail to init ipi magic number");
		gpufreq_abort();
		ret = GPUFREQ_EINVAL;
		goto done;
	}
	writel(0x0, gpueb_gpr_addr);

	/* init shared status on EB side */
	raw_spin_lock_irqsave(&gpufreq_ipi_lock, ipi_flags);
	send_msg.magic = g_ipi_magic;
	send_msg.cmd_id = CMD_INIT_SHARED_MEM;
	send_msg.u.shared_mem.base = g_shared_mem_pa;
	send_msg.u.shared_mem.size = g_shared_mem_size;
	ret = gpufreq_ipi_to_gpueb(send_msg);
	if (unlikely(ret))
		GPUFREQ_LOGE("fail to init gpufreq shared memory");
	raw_spin_unlock_irqrestore(&gpufreq_ipi_lock, ipi_flags);

	/* power off GPUEB */
	ret = gpueb_ctrl(GHPM_OFF, MFG1_OFF, SUSPEND_POWER_OFF);
	if (unlikely(ret)) {
		GPUFREQ_LOGE("fail to power off GPUEB (%d)", ret);
		gpufreq_abort();
		goto done;
	}

done:
	return ret;
}

static int gpufreq_thermal_max_frequency(unsigned int *frequency)
{
	unsigned int value;

	if (!frequency)
		return -EINVAL;
	if (!smp_load_acquire(&g_gpufreq_wrapper_ready))
		return -EAGAIN;
	value = gpufreq_get_freq_by_idx(TARGET_DEFAULT, 0);
	if ((s32)value <= 0)
		return -ENODATA;
	*frequency = value;
	return 0;
}

static const struct thermal_gpu_frequency_provider gpufreq_thermal_provider = {
	.owner = THIS_MODULE,
	.get_max_frequency = gpufreq_thermal_max_frequency,
};

static int gpufreq_wrapper_pdrv_probe(struct platform_device *pdev)
{
	struct device_node *of_wrapper = pdev->dev.of_node;
	int ret = GPUFREQ_SUCCESS;

	GPUFREQ_LOGI("start to probe gpufreq wrapper driver");

	if (unlikely(!of_wrapper)) {
		GPUFREQ_LOGE("fail to find gpufreq wrapper of_node, treat as bringup");
		g_gpufreq_bringup = true;
		goto done;
	}

	g_a16_firmware = of_property_read_bool(of_wrapper, "mediatek,mt6895-a16-firmware");
	of_property_read_u32(of_wrapper, "dual-buck", &g_dual_buck);
	of_property_read_u32(of_wrapper, "gpueb-support", &g_gpueb_support);
	of_property_read_u32(of_wrapper, "gpufreq-bringup", &g_gpufreq_bringup);

	/* init spinlock */
	raw_spin_lock_init(&gpufreq_ipi_lock);
	raw_spin_lock_init(&gpufreq_power_lock);

	/* keep probe successful but do nothing when bringup */
	if (g_gpufreq_bringup) {
		GPUFREQ_LOGI("skip gpufreq wrapper driver probe when bringup");
		goto done;
	}

	/* ensure probe order */
	if (!g_ghpm_ready) {
		GPUFREQ_LOGE("ghpm has not been probed, defer gpufreq wrapper driver probe");
		ret = -EPROBE_DEFER;
		goto done;
	}

	/* init shared memory */
	ret = gpufreq_shared_memory_init();
	if (unlikely(ret)) {
		GPUFREQ_LOGE("fail to init gpufreq shared memory (%d)", ret);
		goto done;
	}

	/* init gpueb ipi setting */
	if (g_gpueb_support) {
		ret = gpufreq_gpueb_init();
		if (unlikely(ret)) {
			GPUFREQ_LOGE("fail to init gpueb ipi (%d)", ret);
			goto done;
		}
	}

	GPUFREQ_LOGI("gpufreq wrapper driver probe done, dual_buck: %s, gpueb_mode: %s",
		g_dual_buck ? "true" : "false",
		g_gpueb_support ? "on" : "off");

	smp_store_release(&g_gpufreq_wrapper_ready, true);
	ret = thermal_register_gpu_frequency_provider(&gpufreq_thermal_provider);
	if (ret) {
		smp_store_release(&g_gpufreq_wrapper_ready, false);
		smp_store_release(&g_legacy_tables_ready, false);
		if (g_gpueb_support)
			mtk_ipi_unregister(get_gpueb_ipidev(), g_ipi_channel);
	}

done:
	return ret;
}

/* Firmware provider clients retain their module symbol dependencies.
 * Sysfs unbind is suppressed: runtime client hot-unplug is not supported.
 */
static void gpufreq_wrapper_pdrv_remove(struct platform_device *pdev)
{
	thermal_unregister_gpu_frequency_provider(&gpufreq_thermal_provider);
	smp_store_release(&g_gpufreq_wrapper_ready, false);
	if (g_gpueb_support && g_ipi_channel >= 0)
		mtk_ipi_unregister(get_gpueb_ipidev(), g_ipi_channel);
	smp_store_release(&g_legacy_tables_ready, false);
	smp_store_release(&g_a16_tables_ready, false);
	g_a16_status = NULL;
	g_legacy_status = NULL;
	g_legacy_firmware = false;
	g_shared_status = NULL;
}

static int __init gpufreq_wrapper_init(void)
{
	int ret = GPUFREQ_SUCCESS;

	GPUFREQ_LOGI("start to init gpufreq wrapper driver");

	/* register platform driver */
	ret = platform_driver_register(&g_gpufreq_wrapper_pdrv);
	if (unlikely(ret)) {
		GPUFREQ_LOGE("fail to register gpufreq wrapper driver (%d)", ret);
		goto done;
	}

	GPUFREQ_LOGI("gpufreq wrapper driver init done");

done:
	return ret;
}

static void __exit gpufreq_wrapper_exit(void)
{
	platform_driver_unregister(&g_gpufreq_wrapper_pdrv);
}

module_init(gpufreq_wrapper_init);
module_exit(gpufreq_wrapper_exit);

MODULE_DEVICE_TABLE(of, g_gpufreq_wrapper_of_match);
MODULE_DESCRIPTION("MediaTek GPU-DVFS wrapper driver");
MODULE_LICENSE("GPL");
