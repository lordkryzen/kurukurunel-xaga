#include <linux/types.h>
/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GPUFREQ_FIRMWARE_H
#define GPUFREQ_FIRMWARE_H
/* Firmware protocol generation, not GPU DDK or hardware generation. */
enum gpufreq_firmware_abi {
	GPUFREQ_FIRMWARE_UNAVAILABLE,
	GPUFREQ_FIRMWARE_LEGACY,
	GPUFREQ_FIRMWARE_NATIVE,
	GPUFREQ_FIRMWARE_AP,
};
enum gpufreq_firmware_abi gpufreq_get_firmware_abi(void);
/* Distinct wire profile; original public enum and its symbol CRC stay fixed. */
#define GPUFREQ_FIRMWARE_MT6895_A16 ((enum gpufreq_firmware_abi)4)
static inline bool gpufreq_separate_mt6895_domains(void)
{
    enum gpufreq_firmware_abi abi = gpufreq_get_firmware_abi();
    return abi == GPUFREQ_FIRMWARE_LEGACY || abi == GPUFREQ_FIRMWARE_MT6895_A16;
}
#endif
