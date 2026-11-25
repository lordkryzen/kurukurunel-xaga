/* Private, scalar ABI between native and modern header translation units. */
#include <linux/types.h>
#include <linux/notifier.h>
phys_addr_t native_bridge_memory(unsigned int id, unsigned int kind);
int native_bridge_feature(unsigned int id, bool enable);
unsigned int native_bridge_ready(unsigned int id);
int native_bridge_notify(struct notifier_block *nb, bool enable);
