/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_TRANSPORT_LIFETIME_H
#define MTK_TRANSPORT_LIFETIME_H

/* The caller first destroys its channels/endpoints; outstanding endpoint
 * references retain the device object until their final release.
 */
struct mtk_rpmsg_device;
void mtk_rpmsg_destroy_device(struct mtk_rpmsg_device *mdev);

/* Registration remains masked until the owner explicitly enables its IRQ. */
struct platform_device;
struct mtk_mbox_device;
int mtk_mbox_probe_disabled(struct platform_device *pdev,
	struct mtk_mbox_device *mbdev, unsigned int mbox);

/* Caller has quiesced all users and freed mailbox IRQs before this call. */
struct mtk_ipi_device;
int mtk_ipi_device_destroy_quiesced(struct mtk_ipi_device *ipidev);
#endif
