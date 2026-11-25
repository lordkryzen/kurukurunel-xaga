/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ADSP_MBOX_LIFECYCLE_H
#define _ADSP_MBOX_LIFECYCLE_H
struct mtk_mbox_pin_recv;
int adsp_mbox_bind_core(struct mtk_mbox_pin_recv *pin, unsigned int core_id);
#endif
