/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ADSP_IPI_QUEUE_LIFECYCLE_H
#define _ADSP_IPI_QUEUE_LIFECYCLE_H

/* Linux-only startup/shutdown; original firmware-facing headers stay exact. */
int dsp_ipi_queue_start(void);
int dsp_ipi_queue_stop(void);

#endif
