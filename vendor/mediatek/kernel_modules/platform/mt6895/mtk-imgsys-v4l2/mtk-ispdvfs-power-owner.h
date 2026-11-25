/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_ISPDVFS_POWER_OWNER_H
#define MTK_ISPDVFS_POWER_OWNER_H
struct regulator;
int mtk_ispdvfs_regulator_enable(struct regulator *reg);
int mtk_ispdvfs_regulator_disable(struct regulator *reg);
#endif
