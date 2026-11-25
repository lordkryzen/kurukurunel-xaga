// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015-2019, MICROTRUST Incorporated
 * Copyright (C) 2015 Google, Inc.
 *
 */

#define IMSG_TAG "[tz_log]"
#include <imsg_log.h>

#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/panic_notifier.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <asm/page.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <teei_client_main.h>
#include "tz_log.h"
#include "nt_smc_call.h"
#include "notify_queue.h"
#include "switch_queue.h"
#include "utdriver_macro.h"
#include "log_perf.h"

#define TEE_LOG_TYPE	0x10

struct tz_log_state *g_tz_log_state;
static struct completion teei_log_comp;

int init_tlog_comp_fn(void)
{
	init_completion(&teei_log_comp);

	return 0;
}

void teei_notify_log_fn(void)
{
	if (!completion_done(&teei_log_comp))
		complete(&teei_log_comp);
}

/* sz is shared with firmware: validate each snapshot before using its mask. */
static bool tz_log_size_valid(u32 size, size_t header)
{
	return size && is_power_of_2(size) && size <= TZ_LOG_SIZE - header;
}

static int __tz_driver_read_logs(struct tz_log_state *s, char *buffer,
				u32 get, unsigned int cnt)
{
	struct log_rb *log = s->log;
	u32 size = READ_ONCE(log->sz);
	unsigned int i;

	if (!tz_log_size_valid(size, sizeof(*log)) || cnt > size)
		return -EINVAL;
	for (i = 0; i < cnt; i++)
		buffer[i] = READ_ONCE(log->data[get++ & (size - 1)]);
	return cnt;
}

int tz_driver_read_logs(char *buffer, unsigned long count)
{
	struct tz_log_state *s = g_tz_log_state;
	struct log_rb *log;
	u32 size, get, put, alloc, available;
	int read_chars;

	if (!s || !s->log || (!buffer && count))
		return -EINVAL;
	log = s->log;
	size = READ_ONCE(log->sz);
	if (!tz_log_size_valid(size, sizeof(*log)))
		return -EINVAL;
	get = s->read_get;
	put = READ_ONCE(log->put);
	rmb();
	alloc = READ_ONCE(log->alloc);
	if ((u32)(alloc - put) > size)
		return -EOVERFLOW;
	if ((u32)(alloc - get) > size)
		get = alloc - size;
	available = put - get;
	if (available > size)
		return -EOVERFLOW;
	read_chars = __tz_driver_read_logs(s, buffer, get,
			min_t(unsigned long, available, count));
	if (read_chars < 0)
		return read_chars;
	rmb();
	alloc = READ_ONCE(log->alloc);
	if ((u32)(alloc - get) > size) {
		s->read_get = alloc - size;
		return -EOVERFLOW;
	}
	s->read_get = get + read_chars;
	return read_chars;
}

static int log_read_line(struct tz_log_state *s, u32 put, u32 get)
{
	struct log_rb *log = s->log;
	u32 size = READ_ONCE(log->sz);
	u32 available = put - get;
	size_t i = 0, limit;
	char c = '\0';

	if (!tz_log_size_valid(size, sizeof(*log)) || available > size)
		return -EINVAL;
	limit = min_t(size_t, available, sizeof(s->line_buffer) - 1);
	while (i < limit && c != '\n')
		s->line_buffer[i++] = c = READ_ONCE(log->data[get++ & (size - 1)]);
	s->line_buffer[i] = '\0';
	return i;
}

static void tz_driver_dump_logs(struct tz_log_state *s)
{
	struct log_rb *log;
	struct boot_log_rb *boot_log;
	u32 size, get, put, alloc, budget;
	int read_chars;
	static DEFINE_RATELIMIT_STATE(_rs,
				TZ_LOG_RATELIMIT_INTERVAL,
				TZ_LOG_RATELIMIT_BURST);

	if (!s || !s->log || !s->boot_log)
		return;
	log = s->log;
	boot_log = s->boot_log;
	size = READ_ONCE(log->sz);
	if (!tz_log_size_valid(size, sizeof(*log)))
		return;
	get = s->get;
	/* A continuously active producer must not monopolize this worker. */
	budget = size;
	while (budget && (put = READ_ONCE(log->put)) != get) {
		rmb();
		alloc = READ_ONCE(log->alloc);
		if ((u32)(alloc - put) > size)
			break;
		if ((u32)(alloc - get) > size)
			get = alloc - size;
		if (put == get)
			break;
		read_chars = log_read_line(s, put, get);
		if (read_chars <= 0)
			break;
		rmb();
		alloc = READ_ONCE(log->alloc);
		/* Discard a line overwritten while it was being copied. */
		if ((u32)(alloc - get) > size) {
			get = alloc - size;
			break;
		}
		IMSG_PRINTK("[TZ_LOG] %s", s->line_buffer);
		{
			u32 boot_size = READ_ONCE(boot_log->sz);
			u32 boot_put = READ_ONCE(boot_log->put);

			if (tz_log_size_valid(boot_size, sizeof(*boot_log)) &&
			    boot_put <= boot_size && (u32)read_chars <= boot_size - boot_put) {
				memcpy(&boot_log->data[boot_put], s->line_buffer, read_chars);
				WRITE_ONCE(boot_log->put, boot_put + read_chars);
			}
		}
		__ratelimit(&_rs);
		get += read_chars;
		budget -= min_t(u32, budget, read_chars);
#if ENABLE_LOG_PERF == 1
		measure_log_perf(IMSG_TAG, read_chars,
					*(uint32_t *)(s->line_buffer));
#endif
	}
	s->get = get;
}

int teei_log_fn(void *work)
{
	int retVal = 0;
#if IS_ENABLED(CONFIG_MICROTRUST_TZ_LOG)
	struct tz_log_state *s;

	s = g_tz_log_state;
#endif

	while (1) {
		if (switch_input_index == switch_output_index) {
			retVal = wait_for_completion_interruptible(
							&teei_log_comp);
			if (retVal != 0)
				continue;
		}

#if IS_ENABLED(CONFIG_MICROTRUST_TZ_LOG)
		msleep(20);
		tz_driver_dump_logs(s);
#endif

	}

	return NOTIFY_OK;
}

static int tz_log_panic_notify(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct tz_log_state *s;

	/*
	 * Don't grab the spin lock to hold up the panic notifier, even
	 * though this is racy.
	 */
	s = container_of(nb, struct tz_log_state, panic_notifier);
	IMSG_INFO("tz log panic notifier\n");
	tz_driver_dump_logs(s);
	return NOTIFY_OK;
}

#ifdef ENABLED_TEEI_BOOT_LOG
static struct tz_log_state *get_tz_log_state(void)
{
	struct tz_driver_state *drv_state = get_tz_drv_state();
	struct platform_device *pdev = drv_state->tz_log_pdev;

	return platform_get_drvdata(pdev);
}

static void *boot_log_seq_start(struct seq_file *f, loff_t *pos)
{
	struct tz_log_state *s = get_tz_log_state();
	struct boot_log_rb *log = s->boot_log;

	if (!tz_log_size_valid(READ_ONCE(log->sz), sizeof(*log)) ||
	    READ_ONCE(log->put) > READ_ONCE(log->sz) ||
	    READ_ONCE(log->get) > READ_ONCE(log->put) ||
	    *pos >= READ_ONCE(log->put))
		return NULL;

	return (void *)log;
}

static void *boot_log_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
	struct boot_log_rb *log = v;

	if (!tz_log_size_valid(READ_ONCE(log->sz), sizeof(*log)) ||
	    READ_ONCE(log->put) > READ_ONCE(log->sz) ||
	    READ_ONCE(log->get) > READ_ONCE(log->put) ||
	    *pos >= READ_ONCE(log->put))
		return NULL;

	*pos = log->get;
	return v;
}

static void boot_log_seq_stop(struct seq_file *f, void *v)
{
}

static int boot_log_read_line(struct boot_log_rb *log, char *out)
{
	u32 size = READ_ONCE(log->sz);
	u32 put = READ_ONCE(log->put), get = READ_ONCE(log->get);
	size_t i = 0, limit;
	char c = '\0';

	if (!tz_log_size_valid(size, sizeof(*log)) || put > size || get > put)
		return -EINVAL;
	limit = min_t(size_t, put - get, TZ_LINE_BUFFER_SIZE - 1);
	while (i < limit && c != '\n')
		out[i++] = c = log->data[get++ & (size - 1)];
	out[i] = '\0';
	return i;
}

static int boot_log_seq_show(struct seq_file *f, void *v)
{
	struct boot_log_rb *log = v;
	char line_buffer[TZ_LINE_BUFFER_SIZE];
	int read_chars;

	read_chars = boot_log_read_line(log, line_buffer);

	if (read_chars < 0)
		return read_chars;
	if (read_chars) {
		seq_printf(f, "%s", line_buffer);
		log->get += read_chars;
	}

	return 0;
}

static const struct seq_operations boot_log_seq_ops = {
	.start = boot_log_seq_start,
	.next = boot_log_seq_next,
	.stop = boot_log_seq_stop,
	.show = boot_log_seq_show
};

static int boot_log_open(struct inode *inode, struct file *file)
{
	struct tz_log_state *s = get_tz_log_state();
	struct boot_log_rb *log = s->boot_log;

	log->get = 0;

	return seq_open(file, &boot_log_seq_ops);
};

static const struct file_operations boot_log_fops = {
	.open = boot_log_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

static struct dentry *root_entry;
static int tz_log_debugfs_init(void)
{
	root_entry = debugfs_create_dir("tz_log", NULL);
	if (!root_entry) {
		IMSG_WARN("Can not create tz_log debugfs\n");
		return -1;
	}

	debugfs_create_file("boot_log", 0444, root_entry, NULL, &boot_log_fops);

	return 0;
}


#endif

int teei_change_log_status(unsigned long new_status)
{
	struct completion *wait_completion = NULL;
	int retVal = 0;

	teei_cpus_read_lock();

	wait_completion = kmalloc(sizeof(struct completion), GFP_KERNEL);
	if (wait_completion == NULL) {
		IMSG_ERROR("TEEI: Failed to alloc completion[%s]\n", __func__);
		teei_cpus_read_unlock();
		return -ENOMEM;
	}

	init_completion(wait_completion);

	retVal = add_work_entry(SMC_CALL_TYPE, N_INVOKE_T_NQ, 0, 0, 0);
	if (retVal != 0) {
		IMSG_ERROR("TEEI: Failed to add_work_entry[%s]\n", __func__);
		kfree(wait_completion);
		teei_cpus_read_unlock();
		return retVal;
	}

	retVal = add_nq_entry(TEEI_MODIFY_TEE_CONFIG, TEE_LOG_TYPE,
				(unsigned long long)(wait_completion),
				new_status, 0, 0);
	if (retVal != 0) {
		IMSG_ERROR("TEEI: Failed to add one nq to n_t_buffer\n");
		kfree(wait_completion);
		teei_cpus_read_unlock();
		return retVal;
	}

	teei_notify_switch_fn();

	wait_for_completion(wait_completion);
	kfree(wait_completion);
	teei_cpus_read_unlock();

	return 0;
}

int tz_log_probe(struct platform_device *pdev)
{
	struct tz_log_state *s;
	int result;

	IMSG_DEBUG("%s\n", __func__);

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		result = -ENOMEM;
		goto error_alloc_state;
	}

	g_tz_log_state = s;

	spin_lock_init(&s->lock);
	s->dev = &pdev->dev;
	s->get = 0;
	s->read_get = 0;
	s->log_pages = alloc_pages(GFP_KERNEL | __GFP_ZERO | GFP_DMA32,
				   get_order(TZ_LOG_SIZE));
	if (!s->log_pages) {
		result = -ENOMEM;
		goto error_alloc_log;
	}
	s->log = page_address(s->log_pages);

	s->boot_log_pages = alloc_pages(GFP_KERNEL | __GFP_ZERO | GFP_DMA32,
				   get_order(TZ_LOG_SIZE));
	if (!s->boot_log_pages) {
		result = -ENOMEM;
		goto error_alloc_boot_log;
	}
	s->boot_log = page_address(s->boot_log_pages);

	s->boot_log->put = 0;
	s->boot_log->sz = rounddown_pow_of_two(
				TZ_LOG_SIZE - sizeof(struct boot_log_rb));

	s->panic_notifier.notifier_call = tz_log_panic_notify;
	result = atomic_notifier_chain_register(&panic_notifier_list,
						&s->panic_notifier);
	if (result < 0) {
		IMSG_ERROR("failed to register panic notifier\n");
		goto error_panic_notifier;
	}
	platform_set_drvdata(pdev, s);

#ifdef ENABLED_TEEI_BOOT_LOG
	tz_log_debugfs_init();
#endif

	return 0;

error_panic_notifier:
	__free_pages(s->boot_log_pages, get_order(TZ_LOG_SIZE));
error_alloc_boot_log:
	__free_pages(s->log_pages, get_order(TZ_LOG_SIZE));
error_alloc_log:
	kfree(s);
	g_tz_log_state = NULL;
error_alloc_state:
	return result;
}

int tz_log_remove(struct platform_device *pdev)
{
	struct tz_log_state *s = platform_get_drvdata(pdev);

	IMSG_DEBUG("%s\n", __func__);

	platform_set_drvdata(pdev, NULL);
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &s->panic_notifier);

	__free_pages(s->log_pages, get_order(TZ_LOG_SIZE));
	__free_pages(s->boot_log_pages, get_order(TZ_LOG_SIZE));
	kfree(s);
	g_tz_log_state = NULL;

	return 0;
}

