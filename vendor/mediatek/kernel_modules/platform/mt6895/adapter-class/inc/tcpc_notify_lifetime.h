/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __TCPC_NOTIFY_LIFETIME_H__
#define __TCPC_NOTIFY_LIFETIME_H__

struct tcpc_device;
/* Caller owns tcpc through return; not callable from its notification worker. */
void tcpc_notify_queue_stop(struct tcpc_device *tcpc);

#endif
