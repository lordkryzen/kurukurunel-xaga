/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_MTK_CCD_CLIENT_H
#define _LINUX_MTK_CCD_CLIENT_H

#include <linux/rpmsg/mtk_ccd_rpmsg.h>

/* A successful mtk_create_client_msgdevice() returns an owned device reference.
 * Destroy consumes that reference even if daemon teardown already removed it.
 * Callers serialize access to their client pointer against destroy.
 */
int mtk_ccd_create_client_endpoint(struct mtk_rpmsg_device *mdev,
				 rpmsg_rx_cb_t cb, void *priv,
				 struct rpmsg_channel_info info);
int mtk_ccd_send_client(struct mtk_rpmsg_device *mdev, void *data, int len);
int mtk_ccd_destroy_client(struct mtk_rpmsg_device *mdev);

#endif
