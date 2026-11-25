/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_EMIMPU_OWNER_H
#define MTK_EMIMPU_OWNER_H

struct emi_mpu;
struct emimpu_owner;

int mtk_emimpu_owner_publish(struct emi_mpu *mpu);
void mtk_emimpu_owner_withdraw(struct emi_mpu *mpu);
struct emimpu_owner *mtk_emimpu_owner_get(void);
void mtk_emimpu_owner_put(struct emimpu_owner *owner);
struct emi_mpu *mtk_emimpu_owner_device(struct emimpu_owner *owner);

#endif
