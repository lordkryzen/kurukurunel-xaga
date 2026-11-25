/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __TCPC_OPERATION_LIFETIME_H__
#define __TCPC_OPERATION_LIFETIME_H__
#include <linux/srcu.h>
#include <linux/gfp_types.h>
struct tcpc_device;
struct tcpc_ops;
/* Caller retains the TCPC reference. Stop is not callable inside a lease. */
int tcpc_operation_get(struct tcpc_device *tcpc, struct tcpc_ops **ops);
void tcpc_operation_put(struct tcpc_device *tcpc, int index);
void tcpc_operation_stop(struct tcpc_device *tcpc);
/* Policy buffers survive device_del while a caller owns a TCPC ref. */
void *tcpc_owned_zalloc(struct tcpc_device *tcpc, size_t size, gfp_t flags);
void tcpc_owned_free(struct tcpc_device *tcpc, void *data);
#endif
