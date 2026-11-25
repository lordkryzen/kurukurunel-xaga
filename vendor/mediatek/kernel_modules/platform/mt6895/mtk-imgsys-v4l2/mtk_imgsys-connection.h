/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_IMGSYS_CONNECTION_H_
#define _MTK_IMGSYS_CONNECTION_H_
#include <linux/bitops.h>
struct mtk_imgsys_dev;
enum imgsys_connection_resource {
	IMGSYS_CONN_REGULATOR = BIT(0),
	IMGSYS_CONN_MODULES = BIT(1),
	IMGSYS_CONN_POWER = BIT(2),
	IMGSYS_CONN_HCP = BIT(3),
	IMGSYS_CONN_POOL = BIT(4),
	IMGSYS_CONN_QUEUE = BIT(5),
	IMGSYS_CONN_RUNNER = BIT(6),
	IMGSYS_CONN_CMDQ = BIT(7),
	IMGSYS_CONN_INIT_HANDLER = BIT(8),
	IMGSYS_CONN_FRAME_HANDLER = BIT(9),
	IMGSYS_CONN_TOKEN_HANDLER = BIT(10),
};
int mtk_imgsys_power_get(struct mtk_imgsys_dev *imgsys_dev);
#endif
