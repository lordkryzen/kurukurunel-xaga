/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ADSP_IPI_LIFETIME_H
#define _ADSP_IPI_LIFETIME_H

/* Linux callback ownership only; no firmware packet or retained header changes. */
typedef void (*adsp_ipi_callback_t)(int, void *, unsigned int);
struct adsp_ipi_handler;
int adsp_ipi_mutation_check(void);
adsp_ipi_callback_t adsp_ipi_handler_callback(struct adsp_ipi_handler *entry);
struct adsp_ipi_handler *adsp_ipi_handler_get(unsigned int id,
	void (*match)(int, void *, unsigned int));
void adsp_ipi_handler_put(struct adsp_ipi_handler *entry);
void adsp_ipi_handler_invoke(struct adsp_ipi_handler *entry, unsigned int id,
	void *buf, unsigned int len);

int adsp_ipi_queue_send(unsigned int core, unsigned int id, void *buf,
	unsigned int len, unsigned int wait_ms);
int adsp_ipi_queue_recv(unsigned int core, unsigned int id, void *buf,
	unsigned int len, adsp_ipi_callback_t callback);

#endif
