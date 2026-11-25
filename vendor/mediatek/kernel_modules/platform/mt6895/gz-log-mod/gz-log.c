// SPDX-License-Identifier: GPL-2.0
/* Native MTK shared-log v1, with owned reserved-ring and reader lifetime.
 * xaga's preloader owns the existing reserved ring. Never ADD/RM that ring.
 * Boards without that firmware-owned region are explicitly unsupported here.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/panic_notifier.h>
#include <linux/sched/clock.h>
#include <linux/math64.h>
#include <gz-trusty/trusty.h>
#include <gz-trusty/smcall.h>

struct log_rb { u32 alloc, put, sz; char data[]; };
struct gz_log_state {
 struct kref refs;
 struct device *parent;
 struct log_rb *ring;
 size_t mapped_bytes;
 u32 ring_bytes, get;
 struct mutex lock;
 wait_queue_head_t wait;
 struct proc_dir_entry *stream, *full;
 struct notifier_block call_nb, panic_nb;
 bool call_registered, panic_registered, stopping;
};
struct gz_reader { struct gz_log_state *s; bool full; u32 get; };
static bool gz_ring_size_valid(size_t bytes, u32 sz)
{
 return bytes > sizeof(struct log_rb) && sz && is_power_of_2(sz) &&
        sz <= bytes - sizeof(struct log_rb) && sz < (1U << 31);
}
static int gz_ring_window(u32 put, u32 alloc, u32 size, u32 *get)
{
 if ((u32)(alloc - *get) > size)
  *get = alloc - size;
 return (u32)(put - *get) <= size ? 0 : -EAGAIN;
}
static void gz_log_free(struct kref *ref)
{
 struct gz_log_state *s = container_of(ref, struct gz_log_state, refs);
 if (s->ring)
  memunmap(s->ring);
 put_device(s->parent);
 kfree(s);
}
static int gz_log_notify(struct notifier_block *nb, unsigned long event, void *data)
{
 struct gz_log_state *s = container_of(nb, struct gz_log_state, call_nb);
 if (event == TRUSTY_CALL_RETURNED && !READ_ONCE(s->stopping))
  wake_up_interruptible(&s->wait);
 return NOTIFY_OK;
}
static int gz_log_panic(struct notifier_block *nb, unsigned long event, void *data)
{
 struct gz_log_state *s = container_of(nb, struct gz_log_state, panic_nb);
 wake_up_interruptible(&s->wait);
 return NOTIFY_OK;
}
static int gz_open_common(struct inode *inode, struct file *file, bool full)
{
 struct gz_log_state *s = pde_data(inode);
 struct gz_reader *r;
 int ret;
 if (!try_module_get(THIS_MODULE))
  return -ENODEV;
 r = kzalloc(sizeof(*r), GFP_KERNEL);
 if (!r) { ret = -ENOMEM; goto put_module; }
 mutex_lock(&s->lock);
 if (s->stopping) { ret = -ENODEV; goto unlock; }
 kref_get(&s->refs);
 r->s = s;
 r->full = full;
 r->get = READ_ONCE(s->ring->put);
 r->get -= min(r->get, s->ring_bytes);
 mutex_unlock(&s->lock);
 ret = nonseekable_open(inode, file);
 if (ret) { kref_put(&s->refs, gz_log_free); kfree(r); goto put_module; }
 file->private_data = r;
 return 0;
unlock:
 mutex_unlock(&s->lock);
 kfree(r);
put_module:
 module_put(THIS_MODULE);
 return ret;
}
static int gz_open(struct inode *i, struct file *f) { return gz_open_common(i, f, false); }
static int gz_full_open(struct inode *i, struct file *f) { return gz_open_common(i, f, true); }
static int gz_release(struct inode *inode, struct file *file)
{
 struct gz_reader *r = file->private_data;
 kref_put(&r->s->refs, gz_log_free);
 kfree(r);
 module_put(THIS_MODULE);
 return 0;
}
static ssize_t gz_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
 struct gz_reader *r = file->private_data;
 struct gz_log_state *s = r->s;
 u32 put, alloc, get;
 size_t copied = 0;
 int ret;
 char chunk[256];
 if (!count) return 0;
 ret = mutex_lock_interruptible(&s->lock);
 if (ret) return ret;
 if (s->stopping) { ret = -ENODEV; goto out; }
 if (READ_ONCE(s->ring->sz) != s->ring_bytes) { ret = -EIO; goto out; }
 put = READ_ONCE(s->ring->put);
 rmb();
 alloc = READ_ONCE(s->ring->alloc);
 get = r->full ? r->get : s->get;
 ret = gz_ring_window(put, alloc, s->ring_bytes, &get);
 if (ret) goto out;
 while (get != put && copied < count) {
  size_t n = min_t(size_t, (u32)(put - get), min(count - copied, sizeof(chunk)));
  size_t i;
  for (i = 0; i < n; i++)
   chunk[i] = READ_ONCE(s->ring->data[(get + i) & (s->ring_bytes - 1)]);
  rmb();
  /* Discard any batch overwritten while the producer advanced. */
  alloc = READ_ONCE(s->ring->alloc);
  if ((u32)(alloc - get) > s->ring_bytes) { ret = -EAGAIN; break; }
  if (copy_to_user(buf + copied, chunk, n)) { ret = -EFAULT; break; }
  get += n;
  copied += n;
 }
 if (r->full) r->get = get; else s->get = get;
 if (copied) { *pos += copied; ret = copied; }
 else if (get == put) ret = 0;
out:
 mutex_unlock(&s->lock);
 return ret;
}
static __poll_t gz_poll(struct file *file, poll_table *wait)
{
 struct gz_reader *r = file->private_data;
 struct gz_log_state *s = r->s;
 __poll_t mask = 0;
 poll_wait(file, &s->wait, wait);
 mutex_lock(&s->lock);
 if (s->stopping) mask = EPOLLHUP | EPOLLERR;
 else if (READ_ONCE(s->ring->sz) != s->ring_bytes) mask = EPOLLERR;
 else if ((r->full ? r->get : s->get) != READ_ONCE(s->ring->put)) mask = EPOLLIN | EPOLLRDNORM;
 mutex_unlock(&s->lock);
 return mask;
}
static const struct proc_ops stream_ops = { .proc_open = gz_open, .proc_read = gz_read, .proc_poll = gz_poll, .proc_release = gz_release };
static const struct proc_ops full_ops = { .proc_open = gz_full_open, .proc_read = gz_read, .proc_release = gz_release };
static void gz_retire(struct gz_log_state *s)
{
 mutex_lock(&s->lock);
 WRITE_ONCE(s->stopping, true);
 mutex_unlock(&s->lock);
 wake_up_interruptible(&s->wait);
 proc_remove(s->full);
 proc_remove(s->stream);
 if (s->panic_registered)
  atomic_notifier_chain_unregister(&panic_notifier_list, &s->panic_nb);
 if (s->call_registered)
  gz_trusty_call_notifier_unregister(s->parent, &s->call_nb);
 /* Core15 does not export __wake_up_pollfree. Preserve the exact native
  * wait.c POLLFREE protocol through its exported __wake_up primitive. */
 if (waitqueue_active(&s->wait)) {
  __wake_up(&s->wait, TASK_NORMAL, 0, poll_to_key(EPOLLHUP | POLLFREE));
  WARN_ON_ONCE(waitqueue_active(&s->wait));
 }
 synchronize_rcu();
 kref_put(&s->refs, gz_log_free);
}
static int gz_map_reserved(struct gz_log_state *s)
{
 struct device_node *node;
 struct reserved_mem *mem;
 node = of_find_compatible_node(NULL, NULL, "mediatek,gz-log");
 if (!node) return -EOPNOTSUPP;
 mem = of_reserved_mem_lookup(node);
 of_node_put(node);
 if (!mem || mem->size <= sizeof(struct log_rb)) return -EINVAL;
 s->mapped_bytes = mem->size;
 s->ring = memremap(mem->base, mem->size, MEMREMAP_WB);
 if (!s->ring) return -ENOMEM;
 s->ring_bytes = READ_ONCE(s->ring->sz);
 if (!gz_ring_size_valid(s->mapped_bytes, s->ring_bytes)) return -EINVAL;
 return 0;
}
static int gz_align_time(struct platform_device *pdev)
{
 u64 mhz = div_u64(arch_timer_get_cntfrq(), 1000000);
 u64 diff = __arch_counter_get_cntvct() - div_u64(mhz * sched_clock(), 1000);
 int ret = gz_trusty_fast_call32(pdev->dev.parent,
  MTEE_SMCNR(MT_SMCF_FC_KTIME_ALIGN, pdev->dev.parent), diff >> 32, (u32)diff, 0);
 return ret < 0 ? ret : 0;
}
static int gz_probe(struct platform_device *pdev)
{
 struct gz_log_state *s;
 int ret;
 if (!pdev->dev.parent) return -ENODEV;
 ret = gz_trusty_std_call32(pdev->dev.parent,
  MTEE_SMCNR(SMCF_SC_SHARED_LOG_VERSION, pdev->dev.parent), 1, 0, 0);
 if (ret != 1) return ret < 0 ? ret : -EPROTONOSUPPORT;
 s = kzalloc(sizeof(*s), GFP_KERNEL);
 if (!s) return -ENOMEM;
 kref_init(&s->refs);
 mutex_init(&s->lock);
 init_waitqueue_head(&s->wait);
 s->parent = get_device(pdev->dev.parent);
 ret = gz_map_reserved(s);
 if (ret) goto fail;
 ret = gz_align_time(pdev);
 if (ret) goto fail;
 s->call_nb.notifier_call = gz_log_notify;
 s->panic_nb.notifier_call = gz_log_panic;
 ret = gz_trusty_call_notifier_register(s->parent, &s->call_nb);
 if (ret) goto fail;
 s->call_registered = true;
 ret = atomic_notifier_chain_register(&panic_notifier_list, &s->panic_nb);
 if (ret) goto fail;
 s->panic_registered = true;
 s->stream = proc_create_data("gz_log", 0440, NULL, &stream_ops, s);
 if (!s->stream) { ret = -ENOMEM; goto fail; }
 s->full = proc_create_data("gz_log_full", 0440, NULL, &full_ops, s);
 if (!s->full) { ret = -ENOMEM; goto fail; }
 platform_set_drvdata(pdev, s);
 dev_info(&pdev->dev, "native firmware-owned log ring ready: %u/%zu bytes\n", s->ring_bytes, s->mapped_bytes);
 return 0;
fail:
 gz_retire(s);
 return dev_err_probe(&pdev->dev, ret, "native log initialization failed\n");
}
static void gz_remove(struct platform_device *pdev)
{
 struct gz_log_state *s = platform_get_drvdata(pdev);
 platform_set_drvdata(pdev, NULL);
 gz_retire(s);
}
static const struct of_device_id gz_match[] = { { .compatible = "android,trusty-gz-log-v1" }, {} };
MODULE_DEVICE_TABLE(of, gz_match);
static struct platform_driver gz_driver = { .probe = gz_probe, .remove = gz_remove,
 .resume = gz_align_time, .driver = { .name = "trusty-gz-log", .of_match_table = gz_match } };
module_platform_driver(gz_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MTK native v1 firmware-owned reserved log reader");
