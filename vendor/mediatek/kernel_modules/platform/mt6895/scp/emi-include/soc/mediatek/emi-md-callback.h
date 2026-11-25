/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_EMIMPU_MD_CALLBACK_H
#define MTK_EMIMPU_MD_CALLBACK_H

struct module;
typedef void (*emimpu_md_callback_t)(const char *message);

/* Registration persists across MPU probe/unbind. Writer calls may sleep.
 * Every caller must unregister before its callback data is destroyed, and
 * before module exit returns. Neither writer API may run inside the callback:
 * a concurrent unregister holds the writer mutex while waiting for dispatch.
 * A NULL owner is reserved for built-in callbacks. Notifications retain the
 * caller's execution context, including hard IRQ constraints.
 */
int mtk_emimpu_md_callback_register(emimpu_md_callback_t callback,
		struct module *owner);
/* Registration is pending until the owning modem probe has succeeded. */
int mtk_emimpu_md_callback_activate(struct module *owner);
int mtk_emimpu_md_callback_unregister(emimpu_md_callback_t callback,
		struct module *owner);
int mtk_emimpu_md_callback_unregister_owner(struct module *owner);
void mtk_emimpu_md_callback_notify(const char *message);

#endif
