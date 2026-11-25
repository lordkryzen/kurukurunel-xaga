/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __LPM_TRACE_H__
#define __LPM_TRACE_H__

#include <lpm_plat_common.h>

void lpm_trace_release(void);
int lpm_trace_parsing(struct device_node *parent);
int lpm_trace_instance_get(int type, struct LPM_PLAT_TRACE *ins);

#endif
