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
#endif
