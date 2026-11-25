/* SPDX-License-Identifier: GPL-2.0 */
/* MT6895 legacy firmware ABI; field order derives from Xiaomi's gpufreq_ipi.h. */
#ifndef GPUFREQ_LEGACY_STATUS_H
#define GPUFREQ_LEGACY_STATUS_H
#include <linux/build_bug.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/stddef.h>
struct gpufreq_legacy_status_prefix {
	int cur_oppidx_gpu;
	int cur_oppidx_stack;
	int opp_num_gpu;
	int opp_num_stack;
	unsigned int cur_fgpu;
	unsigned int cur_fstack;
	unsigned int cur_vgpu;
	unsigned int cur_vstack;
	unsigned int cur_vsram_gpu;
	unsigned int cur_vsram_stack;
	unsigned int cur_power_gpu;
	unsigned int cur_power_stack;
	unsigned int max_power_gpu;
	unsigned int max_power_stack;
	unsigned int min_power_gpu;
	unsigned int min_power_stack;
	unsigned int cur_ceiling_gpu;
	unsigned int cur_floor_gpu;
	unsigned int cur_ceiling_stack;
	unsigned int cur_floor_stack;
	unsigned int cur_c_limiter_gpu;
	unsigned int cur_f_limiter_gpu;
	unsigned int cur_c_limiter_stack;
	unsigned int cur_f_limiter_stack;
	unsigned int power_control;
	unsigned int dvfs_state;
	unsigned int shader_present;
	int power_count;
	unsigned int aging_enable;
	unsigned int avs_enable;
	unsigned int sb_version;
	unsigned int ptp_version;
};

static_assert(sizeof(struct gpufreq_legacy_status_prefix) == 128);

/* A fresh CPU-local snapshot is supplied by the MMIO-owning transport. */
static inline int gpufreq_legacy_validate_status(const struct gpufreq_legacy_status_prefix *status,
	unsigned int maximum_opp_count, bool dual_buck)
{
	if (!status || !maximum_opp_count)
		return -EINVAL;
	if (status->opp_num_gpu <= 0 || (unsigned int)status->opp_num_gpu > maximum_opp_count ||
	    status->cur_oppidx_gpu < 0 || status->cur_oppidx_gpu >= status->opp_num_gpu ||
	    !status->cur_fgpu || !status->cur_vgpu || !status->cur_vsram_gpu ||
	    status->power_count < 0)
		return -ENODATA;
	if (dual_buck && (status->opp_num_stack <= 0 || (unsigned int)status->opp_num_stack > maximum_opp_count ||
	    status->cur_oppidx_stack < 0 || status->cur_oppidx_stack >= status->opp_num_stack ||
	    !status->cur_fstack || !status->cur_vstack || !status->cur_vsram_stack))
		return -ENODATA;
	if (status->cur_ceiling_gpu >= (unsigned int)status->opp_num_gpu ||
	    status->cur_floor_gpu >= (unsigned int)status->opp_num_gpu ||
	    (dual_buck && (status->cur_ceiling_stack >= (unsigned int)status->opp_num_stack ||
	     status->cur_floor_stack >= (unsigned int)status->opp_num_stack)))
		return -ERANGE;
	return 0;
}

/* Only identical semantics cross this boundary. Legacy DEBUG_KEEP/AVS_KEEP
 * must not masquerade as native FIX_OPP/FIX_FREQ_VOLT despite equal bit values.
 * Keep the raw legacy mask available for diagnostics in the owning backend.
 */
static inline int gpufreq_legacy_dvfs_state(unsigned int legacy,
	unsigned int *native)
{
	const unsigned int common = (1U << 0) | (1U << 1) | (1U << 4);
	unsigned int result = 0;

	if (!native)
		return -EINVAL;
	if (legacy & ~common)
		return -EOPNOTSUPP;
	if (legacy & (1U << 0))
		result |= DVFS_DISABLE;
	if (legacy & (1U << 1))
		result |= DVFS_POWEROFF;
	if (legacy & (1U << 4))
		result |= DVFS_AGING_KEEP;
	*native = result;
	return 0;
}
#endif
