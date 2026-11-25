/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _AUDIO_IPI_LIFECYCLE_H
#define _AUDIO_IPI_LIFECYCLE_H

int audio_ipi_dma_start(void);
void audio_ipi_dma_stop(void);

int audio_messenger_ipi_start(void);
void audio_messenger_ipi_stop(void);

#endif
