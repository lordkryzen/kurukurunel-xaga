/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GPUEB_LIFECYCLE_H
#define GPUEB_LIFECYCLE_H
int gpueb_ipi_exit(void);
void gpueb_reserved_mem_exit(void);
void gpueb_timesync_exit(void);
void gpueb_logger_exit(void);
void gpueb_debug_exit(void);
void gpueb_hw_voter_dbg_exit(void);
bool gpueb_legacy_profile(void);
void ghpm_debug_exit(void);
#endif
