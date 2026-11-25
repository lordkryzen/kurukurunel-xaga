/* SPDX-License-Identifier: GPL-2.0 */
#ifndef THERMAL_GPU_PROVIDER_H
#define THERMAL_GPU_PROVIDER_H
#include <linux/module.h>

/* Frequencies use the native GPUFREQ kHz unit. Errors mean unavailable. */
struct thermal_gpu_frequency_provider {
	struct module *owner;
	int (*get_max_frequency)(unsigned int *frequency);
};
int thermal_register_gpu_frequency_provider(const struct thermal_gpu_frequency_provider *provider);
void thermal_unregister_gpu_frequency_provider(const struct thermal_gpu_frequency_provider *provider);
#endif
