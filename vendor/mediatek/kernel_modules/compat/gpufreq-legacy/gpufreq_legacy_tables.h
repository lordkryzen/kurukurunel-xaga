/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GPUFREQ_LEGACY_TABLES_H
#define GPUFREQ_LEGACY_TABLES_H
#include <linux/build_bug.h>
#include <linux/errno.h>
#include <linux/types.h>

struct gpufreq_legacy_opp {
	u32 frequency, voltage, sram_voltage, post_divider, aging_delta, power;
};
struct gpufreq_legacy_debug_opp {
	s32 current_index;
	u32 frequency, voltage, sram_voltage;
	u32 measured_frequency, pll_frequency, regulator_voltage, regulator_sram_voltage;
	s32 buck_count, mtcmos_count, cg_count, power_count;
	u32 segment;
	s32 working_count, signed_count;
	u32 dvfs_state, shader_mask, aging_enabled, avs_enabled, gpm_enabled;
	u32 sb_version, ptp_version, temperature_compensation;
};
static_assert(sizeof(struct gpufreq_legacy_opp) == 24);
static_assert(sizeof(struct gpufreq_legacy_debug_opp) == 92);

/* Validate every row before publishing any output. Frequencies remain kHz,
 * voltages mV*100 and power mW. The only margin reported by this firmware is
 * its aging delta: preserve that value explicitly; do not calculate/apply a
 * newer AVS/aging policy on the AP or reinterpret the adjusted voltage.
 */
static inline int gpufreq_legacy_decode_table(const struct gpufreq_legacy_opp *source,
	u32 count, u32 available_bytes, struct gpufreq_opp_info *output)
{
	u32 i;

	if (!source || !output || !count)
		return -EINVAL;
	if (count > GPUFREQ_MAX_OPP_NUM || count > available_bytes / sizeof(*source))
		return -ERANGE;
	for (i = 0; i < count; i++) {
		if (!source[i].frequency || !source[i].voltage || !source[i].sram_voltage ||
		    source[i].post_divider > POSDIV_POWER_16 ||
		    source[i].aging_delta > source[i].voltage ||
		    (i && source[i].frequency > source[i - 1].frequency))
			return -EPROTO;
	}
	for (i = 0; i < count; i++) {
		output[i].freq = source[i].frequency;
		output[i].volt = source[i].voltage;
		output[i].vsram = source[i].sram_voltage;
		output[i].posdiv = source[i].post_divider;
		output[i].margin = source[i].aging_delta;
		/* Zero power is valid in captured xaga firmware GPU tables. */
		output[i].power = source[i].power;
	}
	return 0;
}
#endif
