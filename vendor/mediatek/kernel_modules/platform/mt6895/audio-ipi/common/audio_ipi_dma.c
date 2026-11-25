// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2016 MediaTek Inc.

#include <audio_ipi_dma.h>
#include <audio_ipi_lifecycle.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/interrupt.h>

#include <linux/string.h>
#include <linux/io.h>
#include <linux/genalloc.h>

#include <linux/vmalloc.h>

#include <linux/delay.h>
#include <linux/uaccess.h>      /* needed by copy_to_user */

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
#include <adsp_helper.h>
#endif

#include <audio_log.h>
#include <audio_assert.h>
#include <audio_ipi_platform.h>

#include <audio_task.h>
#include <audio_controller_msg_id.h>
#include <audio_messenger_ipi.h>

#include <audio_ringbuf.h>
#include <linux/wait.h>




/*
 * =============================================================================
 *                     MACRO
 * =============================================================================
 */

#define MAX_SIZE_OF_ONE_FRAME (16) /* 32-bits * 4ch */
#define DO_BYTE_ALIGN(value, mask) (((value) + mask) & (~mask))


/* hal read */
#define MAX_DSP_MSG_NUM_IN_QUEUE (256)
#define MAX_DSP_DMA_WRITE_SIZE   (0x10000)


/*
 * =============================================================================
 *                     log
 * =============================================================================
 */

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[IPI][DMA] %s(), " fmt "\n", __func__

//#define DUMP_DMA_DEBUG
#if defined(DUMP_DMA_DEBUG)
#define DMA_DUMP_MAX_SIZE (512)
static char *g_dump_buf[DMA_DUMP_MAX_SIZE];
#endif

/*
 * =============================================================================
 *                     struct def
 * =============================================================================
 */

/* DMA */
struct audio_region_t {
	uint32_t offset;        /* ex: 0x1000 */
	uint32_t size;          /* ex: 0x100 */
	uint8_t  resv_128[120];

	uint32_t read_idx;      /* ex: 0x0 ~ 0xFF */
	uint8_t  resv_256[124];

	uint32_t write_idx;     /* ex: 0x0 ~ 0xFF */
	uint8_t  resv_384[124];
};


struct audio_ipi_dma_t {
	struct aud_ptr_t base_phy;
	struct aud_ptr_t base_vir;

	uint32_t size;
	uint8_t  resv_128[108];

	struct audio_region_t region[TASK_SCENE_SIZE][NUM_AUDIO_IPI_DMA_PATH];
	uint32_t checksum;

	/* the beginning of pool data: regions allocated from pool_offset */
	uint32_t pool_offset;
};



/* queue */
struct hal_dma_queue_t {
	struct ipi_msg_t msg[MAX_DSP_MSG_NUM_IN_QUEUE];

	uint32_t size;
	uint32_t idx_r;
	uint32_t idx_w;

	spinlock_t queue_lock;
	wait_queue_head_t queue_wq;

	struct audio_ringbuf_t dma_data;
	uint8_t *tmp_buf_d2k;
	uint8_t *tmp_buf_k2h;
};



/*
 * =============================================================================
 *                     global var
 * =============================================================================
 */

static struct audio_ipi_dma_t *g_dma[NUM_OPENDSP_TYPE];
static struct gen_pool *g_dma_pool[NUM_OPENDSP_TYPE];

static bool g_dsp_init_flag[NUM_OPENDSP_TYPE];
static bool g_region_reg_flag[TASK_SCENE_SIZE];

static struct hal_dma_queue_t g_hal_dma_queue;
static DEFINE_MUTEX(audio_dma_lifecycle_lock);
static bool audio_dma_started;
static DEFINE_MUTEX(audio_dma_init_lock);
static DEFINE_MUTEX(audio_region_lock);

/* audio dsp cache limitation:*/
/* byte align alignment: scp cm4_a(32), cm4_b(32), audio dsp(128) */
static uint8_t g_cache_alilgn_order[NUM_OPENDSP_TYPE];
static uint8_t g_cache_alilgn_mask[NUM_OPENDSP_TYPE];
/*
 * =============================================================================
 *                     utilities
 * =============================================================================
 */

inline phys_addr_t offset_to_phy_addr(const uint32_t offset,
				      const uint32_t dsp_id)
{
	if (!g_dma[dsp_id])
		return 0;
	return g_dma[dsp_id]->base_phy.addr_val + offset;
}

inline uint8_t *offset_to_vir_addr(const uint32_t offset,
				   const uint32_t dsp_id)
{
	if (!g_dma[dsp_id])
		return 0;
	return g_dma[dsp_id]->base_vir.addr + offset;
}

inline uint32_t phy_addr_to_offset(const phys_addr_t phy_addr,
				   const uint32_t dsp_id)
{
	if (!g_dma[dsp_id])
		return 0;
	return phy_addr - g_dma[dsp_id]->base_phy.addr_val;
}

inline uint32_t vir_addr_to_offset(const uint8_t *vir_addr,
				   const uint32_t dsp_id)
{
	if (!g_dma[dsp_id])
		return 0;
	return vir_addr - g_dma[dsp_id]->base_vir.addr;
}

inline uint64_t phy_addr_to_vir_addr_val(const phys_addr_t phy_addr,
		const uint32_t dsp_id)
{
	if (!g_dma[dsp_id])
		return 0;
	return g_dma[dsp_id]->base_vir.addr_val +
	       phy_addr_to_offset(phy_addr, dsp_id);
}


inline uint8_t *dma_vir_base(const uint32_t dsp_id)
{
	if (!g_dma[dsp_id]) {
		pr_notice("g_dma is NULL!!!, return.");
		return NULL;
	}
	ipi_dbg("g_dma->base_vir.addr=%p", g_dma[dsp_id]->base_vir.addr);
	return g_dma[dsp_id]->base_vir.addr;
}


#define DUMP_REGION(LOG_F, description, p_region, count) \
	do { \
		if (p_region && description) { \
			LOG_F( \
			       "%s, offset: 0x%x, size: 0x%x, " \
			       "read_idx: 0x%x, write_idx: 0x%x, " \
			       "count: %u", \
			       description, \
			       (p_region)->offset, \
			       (p_region)->size, \
			       (p_region)->read_idx, \
			       (p_region)->write_idx, \
			       count); \
		} else { \
			pr_notice("%uL, %p %p", \
				  __LINE__, p_region, description); \
		} \
	} while (0)


/*
 * =============================================================================
 *                     init
 * =============================================================================
 */

static int hal_dma_init_msg_queue(struct hal_dma_queue_t *msg_queue,
				  const uint32_t size);
static int hal_dma_deinit_msg_queue(struct hal_dma_queue_t *msg_queue);

int init_audio_ipi_dma_by_dsp(const uint32_t dsp_id)
{
	uint32_t size = 0, mem_id = 0;
	struct audio_ipi_dma_t *dma;
	struct gen_pool *pool;
	phys_addr_t physical, end;
	int ret;

	if (dsp_id >= NUM_OPENDSP_TYPE || !is_audio_dsp_support(dsp_id))
		return -ENODEV;
	if (g_dma[dsp_id] || g_dma_pool[dsp_id])
		return -EALREADY;
#if !defined(MTK_AUDIO_CM4_DMA_SUPPORT)
	if (dsp_id == AUDIO_OPENDSP_USE_CM4_A || dsp_id == AUDIO_OPENDSP_USE_CM4_B)
		return -ENODEV;
#endif
	ret = get_reserve_mem_size(dsp_id, &mem_id, &size);
	if (ret)
		return ret;
	/* Validate the entire shared header before even its first field is written. */
	if (mem_id == 0xFFFFFFFF || size < sizeof(*dma))
		return -EINVAL;
	dma = get_reserve_mem_virt(dsp_id, mem_id);
	if (!dma)
		return -ENOMEM;
	physical = get_reserve_mem_phys(dsp_id, mem_id);
	if (!physical)
		return -EINVAL;
	if (check_add_overflow(physical, (phys_addr_t)size, &end))
		return -EOVERFLOW;
	g_cache_alilgn_order[dsp_id] = get_cache_aligned_order(dsp_id);
	g_cache_alilgn_mask[dsp_id] = get_cache_aligned_mask(dsp_id);
	if (!g_cache_alilgn_order[dsp_id] || !g_cache_alilgn_mask[dsp_id])
		return -EINVAL;

	/* Keep the exact original firmware header and pool-offset convention. */
	memset_io(dma, 0, size);
	dma->base_phy.addr_val = physical;
	dma->base_vir.addr = (void *)dma;
	dma->size = size;
	dma->checksum = (uint8_t *)&dma->checksum - (uint8_t *)dma;
	dma->pool_offset = (uint8_t *)&dma->pool_offset - (uint8_t *)dma;
	dma->pool_offset = DO_BYTE_ALIGN(dma->pool_offset, g_cache_alilgn_mask[dsp_id]);
	if (dma->pool_offset >= dma->size)
		return -EINVAL;
	pool = gen_pool_create(g_cache_alilgn_order[dsp_id], -1);
	if (!pool)
		return -ENOMEM;
	ret = gen_pool_add_virt(pool, dma->base_vir.addr_val + dma->pool_offset,
		dma->base_phy.addr_val + dma->pool_offset,
		(size_t)(dma->size - dma->pool_offset), -1);
	if (ret) {
		gen_pool_destroy(pool);
		return ret;
	}
	g_dma_pool[dsp_id] = pool;
	g_dma[dsp_id] = dma;
	return 0;
}

int deinit_audio_ipi_dma_by_dsp(const uint32_t dsp_id)
{
	unsigned int task;

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -ENODEV;
	if (!g_dma[dsp_id])
		return 0;
	/* A shared task-flag table does not make another DSP's region ours to free. */
	for (task = 0; task < TASK_SCENE_SIZE; task++)
		if (audio_get_dsp_id(task) == dsp_id && g_region_reg_flag[task])
			audio_ipi_dma_free_region(task);
	if (g_dma_pool[dsp_id]) {
		gen_pool_destroy(g_dma_pool[dsp_id]);
		g_dma_pool[dsp_id] = NULL;
	}
	g_dma[dsp_id] = NULL;
	g_dsp_init_flag[dsp_id] = false;
	return 0;
}

int audio_ipi_dma_start(void)
{
	unsigned int dsp;
	uint32_t maximum = 0;
	int ret = 0;

	mutex_lock(&audio_dma_lifecycle_lock);
	if (audio_dma_started)
		goto out;
	for (dsp = 0; dsp < NUM_OPENDSP_TYPE; dsp++) {
		if (!is_audio_dsp_support(dsp))
			continue;
		ret = init_audio_ipi_dma_by_dsp(dsp);
		if (ret)
			goto fail;
		if (g_dma[dsp]->size > maximum)
			maximum = g_dma[dsp]->size;
	}
	/* One HAL consumer queue, sized once for all retained DSP pools. */
	if (maximum) {
		ret = hal_dma_init_msg_queue(&g_hal_dma_queue, maximum);
		if (ret)
			goto fail;
	}
	audio_dma_started = true;
	goto out;
fail:
	for (dsp = NUM_OPENDSP_TYPE; dsp > 0; dsp--)
		deinit_audio_ipi_dma_by_dsp(dsp - 1);
out:
	mutex_unlock(&audio_dma_lifecycle_lock);
	return ret;
}

void audio_ipi_dma_stop(void)
{
	unsigned int dsp;

	/* Dispatcher, task queues and user file operations must already be quiesced. */
	mutex_lock(&audio_dma_lifecycle_lock);
	for (dsp = NUM_OPENDSP_TYPE; dsp > 0; dsp--)
		deinit_audio_ipi_dma_by_dsp(dsp - 1);
	hal_dma_deinit_msg_queue(&g_hal_dma_queue);
	audio_dma_started = false;
	mutex_unlock(&audio_dma_lifecycle_lock);
}

void init_audio_ipi_dma(void)
{
	WARN_ON(audio_ipi_dma_start());
}

void deinit_audio_ipi_dma(void)
{
	audio_ipi_dma_stop();
}

int audio_ipi_dma_init_dsp(const uint32_t dsp_id)
{
	struct ipi_msg_t ipi_msg;
	int ret = 0;
	uint8_t task = TASK_SCENE_INVALID;
	uint64_t payload[2];

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id(%u) invalid!!!", dsp_id);
		return -ENODEV;
	}
	if (!g_dma[dsp_id]) {
		pr_info("g_dma[%u] is NULL!!! return -EFAULT", dsp_id);
		return -EFAULT;
	}

	task = get_audio_controller_task(dsp_id);
	if (task == TASK_SCENE_INVALID) {
		pr_info("task(%d) invalid!!!", task);
		return -ENODEV;
	}

	WRITE_ONCE(g_dsp_init_flag[dsp_id], false);

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (is_audio_use_adsp(dsp_id)) {
		ret = adsp_register_feature(AUDIO_CONTROLLER_FEATURE_ID);
		if (ret)
			return ret;
	}
#endif
	payload[0] = g_dma[dsp_id]->base_phy.addr_val;
	payload[1] = (uint64_t)g_dma[dsp_id]->size;

	ret = audio_send_ipi_msg(
		      &ipi_msg,
		      task,
		      AUDIO_IPI_LAYER_TO_DSP,
		      AUDIO_IPI_PAYLOAD,
		      AUDIO_IPI_MSG_DIRECT_SEND,
		      AUD_CTL_MSG_A2D_DMA_INIT,
		      sizeof(payload),
		      0,
		      &payload);

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (is_audio_use_adsp(dsp_id))
		adsp_deregister_feature(AUDIO_CONTROLLER_FEATURE_ID);
#endif
	WRITE_ONCE(g_dsp_init_flag[dsp_id], ret == 0);
	pr_info("dsp_id: %u, task: %d, ret: %d", dsp_id, task, ret);
	return ret;
}


static int audio_dma_ensure_initialized(uint32_t dsp_id)
{
	int ret = 0;

	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	mutex_lock(&audio_dma_init_lock);
	if (!READ_ONCE(g_dsp_init_flag[dsp_id]))
		ret = audio_ipi_dma_init_dsp(dsp_id);
	mutex_unlock(&audio_dma_init_lock);
	return ret;
}

void *get_audio_ipi_dma_vir_addr(uint32_t dsp_id, phys_addr_t physical)
{
	phys_addr_t offset;

	if (dsp_id >= NUM_OPENDSP_TYPE || !g_dma[dsp_id] || !physical ||
	    physical < g_dma[dsp_id]->base_phy.addr_val)
		return NULL;
	offset = physical - g_dma[dsp_id]->base_phy.addr_val;
	if (offset >= g_dma[dsp_id]->size)
		return NULL;
	return g_dma[dsp_id]->base_vir.addr + offset;
}



/*
 * =============================================================================
 *                     alloc/free
 * =============================================================================
 */

int audio_ipi_dma_alloc(const uint8_t task, phys_addr_t *phy_addr,
	void **virt_addr, const uint32_t size)
{
	uint32_t dsp_id;
	unsigned long address;
	phys_addr_t physical;
	int ret;

	if (phy_addr)
		*phy_addr = 0;
	if (virt_addr)
		*virt_addr = NULL;
	if (!phy_addr || !virt_addr || !size)
		return -EINVAL;
	if (task >= TASK_SCENE_SIZE)
		return -EOVERFLOW;
	dsp_id = audio_get_dsp_id(task);
	if (dsp_id >= NUM_OPENDSP_TYPE || !g_dma[dsp_id])
		return -ENODEV;
	if (!g_dma_pool[dsp_id])
		return -ENOMEM;
	ret = audio_dma_ensure_initialized(dsp_id);
	if (ret)
		return ret;
	address = gen_pool_alloc(g_dma_pool[dsp_id], size);
	if (!address)
		return -ENOMEM;
	physical = gen_pool_virt_to_phys(g_dma_pool[dsp_id], address);
	if (!physical) {
		gen_pool_free(g_dma_pool[dsp_id], address, size);
		return -EFAULT;
	}
	/* The pool already returned a kernel virtual address; do not reconstruct it
	 * from a truncated/shared physical-base subtraction.
	 */
	*phy_addr = physical;
	*virt_addr = (void *)address;
	return 0;
}


int audio_ipi_dma_free(const uint8_t task,
		       phys_addr_t phy_addr, const uint32_t size)
{
	uint32_t dsp_id = audio_get_dsp_id(task);

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id(%u) invalid!!!", dsp_id);
		return -ENODEV;
	}
	if (g_dma[dsp_id] == NULL) {
		pr_info("g_dma[%u] is NULL!!!", dsp_id);
		return -ENODEV;
	}
	if (g_dma_pool[dsp_id] == NULL) {
		pr_info("dma_pool[%u] is NULL!!!", dsp_id);
		return -ENOMEM;
	}
	if (phy_addr == 0 || size == 0) {
		pr_info("arg err, %llu, %u", phy_addr, size);
		return -EINVAL;
	}


	gen_pool_free(g_dma_pool[dsp_id],
		      phy_addr_to_vir_addr_val(phy_addr, dsp_id),
		      size);

	pr_info("task %d, size 0x%x, (%zu/%zu)",
		task,
		size,
		gen_pool_avail(g_dma_pool[dsp_id]),
		gen_pool_size(g_dma_pool[dsp_id]));

	return 0;
}



/*
 * =============================================================================
 *                     alloc/free region
 * =============================================================================
 */

int audio_ipi_dma_alloc_region(const uint8_t task,
	const uint32_t ap_to_dsp_size, const uint32_t dsp_to_ap_size)
{
	uint32_t sizes[NUM_AUDIO_IPI_DMA_PATH] = {ap_to_dsp_size, dsp_to_ap_size};
	unsigned long addresses[NUM_AUDIO_IPI_DMA_PATH] = {0};
	uint32_t offsets[NUM_AUDIO_IPI_DMA_PATH] = {0};
	struct ipi_msg_t ipi_msg;
	uint32_t dsp_id;
	uint8_t controller;
	unsigned int i;
	int ret;

	if (task >= TASK_SCENE_SIZE)
		return -EOVERFLOW;
	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	dsp_id = audio_get_dsp_id(task);
	if (dsp_id >= NUM_OPENDSP_TYPE || dsp_id == AUDIO_OPENDSP_USE_CM4_B || !g_dma[dsp_id])
		return -ENODEV;
	if (!g_dma_pool[dsp_id])
		return -ENOMEM;
	controller = get_audio_controller_task(dsp_id);
	if (controller == TASK_SCENE_INVALID)
		return -ENODEV;
	mutex_lock(&audio_region_lock);
	if (g_region_reg_flag[task]) {
		ret = -EEXIST;
		goto out;
	}
	ret = audio_dma_ensure_initialized(dsp_id);
	if (ret)
		goto out;
	/* Stage both extents privately before publishing either shared descriptor. */
	for (i = 0; i < NUM_AUDIO_IPI_DMA_PATH; i++) {
		phys_addr_t physical, offset;

		if (!sizes[i])
			continue;
		addresses[i] = gen_pool_alloc(g_dma_pool[dsp_id], sizes[i]);
		if (!addresses[i]) {
			ret = -ENOMEM;
			goto rollback;
		}
		physical = gen_pool_virt_to_phys(g_dma_pool[dsp_id], addresses[i]);
		if (!physical || physical < g_dma[dsp_id]->base_phy.addr_val) {
			ret = -EFAULT;
			goto rollback;
		}
		offset = physical - g_dma[dsp_id]->base_phy.addr_val;
		if (offset < g_dma[dsp_id]->pool_offset || offset >= g_dma[dsp_id]->size ||
		    sizes[i] > g_dma[dsp_id]->size - offset) {
			ret = -ERANGE;
			goto rollback;
		}
		offsets[i] = offset;
	}
	for (i = 0; i < NUM_AUDIO_IPI_DMA_PATH; i++) {
		struct audio_region_t *region = &g_dma[dsp_id]->region[task][i];

		region->offset = offsets[i];
		region->size = sizes[i];
		region->read_idx = 0;
		region->write_idx = 0;
	}
	g_region_reg_flag[task] = true;
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (is_audio_use_adsp(dsp_id)) {
		ret = adsp_register_feature(AUDIO_CONTROLLER_FEATURE_ID);
		if (ret)
			goto out; /* Shared state is exposed: retain its extents. */
	}
#endif
	ret = audio_send_ipi_msg(&ipi_msg, controller, AUDIO_IPI_LAYER_TO_DSP,
		AUDIO_IPI_MSG_ONLY, AUDIO_IPI_MSG_DIRECT_SEND,
		AUD_CTL_MSG_A2D_DMA_UPDATE_REGION, task, 0, NULL);
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (is_audio_use_adsp(dsp_id))
		adsp_deregister_feature(AUDIO_CONTROLLER_FEATURE_ID);
#endif
	/* Failed/uncertain publication is not permission to recycle exposed memory. */
	goto out;
rollback:
	for (i = 0; i < NUM_AUDIO_IPI_DMA_PATH; i++)
		if (addresses[i])
			gen_pool_free(g_dma_pool[dsp_id], addresses[i], sizes[i]);
out:
	mutex_unlock(&audio_region_lock);
	return ret;
}


int audio_ipi_dma_free_region(const uint8_t task)
{
	struct audio_region_t *region = NULL;

	phys_addr_t phy_value = 0;

	struct ipi_msg_t ipi_msg;

	int ret = 0;

	int i = 0;

	uint8_t task_ctrl = TASK_SCENE_INVALID;
	uint32_t dsp_id;

	if (task >= TASK_SCENE_SIZE)
		return -EOVERFLOW;
	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	dsp_id = audio_get_dsp_id(task);

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id(%u) invalid!!!", dsp_id);
		return -ENODEV;
	}
	if (g_dma[dsp_id] == NULL) {
		pr_info("g_dma[%u] is NULL!!!", dsp_id);
		return -ENODEV;
	}
	if (g_dma_pool[dsp_id] == NULL) {
		pr_info("dma_pool[%u] is NULL!!!", dsp_id);
		return -ENOMEM;
	}
	if (task >= TASK_SCENE_SIZE) {
		pr_info("task: %d", task);
		return -EOVERFLOW;
	}
	task_ctrl = get_audio_controller_task(dsp_id);
	if (task_ctrl == TASK_SCENE_INVALID) {
		pr_info("ipi dma is not supported in dsp_id(%u), ctrl = %d",
			dsp_id, task_ctrl);
		return -ENODEV;
	}

	mutex_lock(&audio_region_lock);
	if (g_region_reg_flag[task] == false) {
		pr_info("task: %d already unregister", task);
		mutex_unlock(&audio_region_lock);
		return -ENODEV;
	}
	g_region_reg_flag[task] = false;

	for (i = 0; i < NUM_AUDIO_IPI_DMA_PATH; i++) {
		region = &g_dma[dsp_id]->region[task][i];

		if (region->read_idx != region->write_idx) {
			pr_notice("region[%d][%d]: %u != %u",
				  task, i, region->read_idx, region->write_idx);
		}
		if (region->size == 0) {
			AUD_ASSERT(region->offset == 0);
			continue;
		}
		phy_value = offset_to_phy_addr(region->offset, dsp_id);
		ipi_dbg("task %d, region[%d] sz 0x%x, offset 0x%x, phy_value 0x%x",
			task, i, region->size, region->offset, phy_value);

		gen_pool_free(g_dma_pool[dsp_id],
			      phy_addr_to_vir_addr_val(phy_value, dsp_id),
			      region->size);

		region->offset = 0;
		region->size = 0;
		region->read_idx = 0;
		region->write_idx = 0;
	}

	if (ret == 0) {
		pr_info("task %d, a2d sz 0x%x, offset 0x%x, d2a sz 0x%x, offset 0x%x, (%zu/%zu)",
			task,
			g_dma[dsp_id]->region[task][0].size,
			g_dma[dsp_id]->region[task][0].offset,
			g_dma[dsp_id]->region[task][1].size,
			g_dma[dsp_id]->region[task][1].offset,
			gen_pool_avail(g_dma_pool[dsp_id]),
			gen_pool_size(g_dma_pool[dsp_id]));

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
		if (is_audio_use_adsp(dsp_id))
			adsp_register_feature(AUDIO_CONTROLLER_FEATURE_ID);
#endif
		audio_send_ipi_msg(
			&ipi_msg,
			task_ctrl,
			AUDIO_IPI_LAYER_TO_DSP,
			AUDIO_IPI_MSG_ONLY,
			AUDIO_IPI_MSG_DIRECT_SEND,
			AUD_CTL_MSG_A2D_DMA_UPDATE_REGION,
			task,
			0,
			NULL);
	}
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (is_audio_use_adsp(dsp_id))
		adsp_deregister_feature(AUDIO_CONTROLLER_FEATURE_ID);
#endif
	mutex_unlock(&audio_region_lock);
	return 0;
}


int audio_ipi_dma_free_region_all_task(uint32_t dsp_id)
{
	uint8_t task = 0;
	int ret_itor = 0;
	int ret = 0;

	for (task = 0 ; task < TASK_SCENE_SIZE; task++) {
		if (audio_get_dsp_id(task) == dsp_id) {
			if (g_region_reg_flag[task]) {
				ret_itor = audio_ipi_dma_free_region(task);
				if (ret_itor != 0)
					ret = ret_itor;
			}
		}
	}

	return ret;
}



/*
 * =============================================================================
 *                     region
 * =============================================================================
 */

static uint32_t audio_region_data_count(struct audio_region_t *region)
{
	uint32_t count = 0;

	if (!region)
		return 0;

	if (region->size == 0) {
		pr_notice("size fail");
		return 0;
	}

	if (region->read_idx >= region->size) {
		DUMP_REGION(pr_notice, "read_idx fail", region, count);
		region->read_idx %= region->size;
	} else if (region->write_idx >= region->size) {
		DUMP_REGION(pr_notice, "write_idx fail", region, count);
		region->write_idx %= region->size;
	}

	if (region->write_idx >= region->read_idx)
		count = region->write_idx - region->read_idx;
	else
		count = region->size - (region->read_idx - region->write_idx);

	return count;
}


static uint32_t audio_region_free_space(struct audio_region_t *region)
{
	uint32_t count = 0;

	if (!region)
		return 0;

	count = region->size - audio_region_data_count(region);

	if (count >= MAX_SIZE_OF_ONE_FRAME)
		count -= MAX_SIZE_OF_ONE_FRAME;
	else
		count = 0;

	return count;
}


static int audio_region_write_from_linear(uint32_t dsp_id,
		struct audio_region_t *region,
		const void *linear_buf,
		uint32_t count)
{
	uint32_t count_align = DO_BYTE_ALIGN(count,
					     g_cache_alilgn_mask[dsp_id]);

	uint32_t free_space = 0;
	uint8_t *base = NULL;
	uint32_t w2e = 0;
#if defined(DUMP_DMA_DEBUG)
	uint32_t dump_size = DMA_DUMP_MAX_SIZE;
	uint8_t *dump_start_idx = NULL;

	memset(g_dump_buf, '\0', DMA_DUMP_MAX_SIZE);
#endif

	if (!region) {
		pr_info("region is NULL!!! return -EFAULT");
		return -EFAULT;
	}
	if (!linear_buf) {
		pr_info("linear_buf is NULL!!! return -EFAULT");
		return -EFAULT;
	}
	if (!dma_vir_base(dsp_id)) {
		pr_info("dma_vir_base(%d) is NULL!!! return -EFAULT",
			dsp_id);
		return -EFAULT;
	}

	if (region->size == 0) {
		DUMP_REGION(pr_notice, "size fail", region, count);
		return -ENODEV;
	}

	if (region->read_idx >= region->size) {
		DUMP_REGION(pr_notice, "read_idx fail", region, count);
		region->read_idx %= region->size;
	}
	if (region->write_idx >= region->size) {
		DUMP_REGION(pr_notice, "write_idx fail", region, count);
		region->write_idx %= region->size;
	}
	DUMP_REGION(ipi_dbg, "in", region, count);

	free_space = audio_region_free_space(region);
	if (free_space < count_align) {
		DUMP_REGION(pr_notice, "free_space < count_align", region,
			    count);
		return -EOVERFLOW;
	}

	base = dma_vir_base(dsp_id) + region->offset;
#if defined(DUMP_DMA_DEBUG)
	dump_start_idx = base + region->write_idx;
#endif
	if (region->read_idx <= region->write_idx) {
		w2e = region->size - region->write_idx;
		if (count_align <= w2e) {
			memcpy(base + region->write_idx, linear_buf, count);
			region->write_idx += count_align;
			if (region->write_idx == region->size)
				region->write_idx = 0;
		} else {
			memcpy(base + region->write_idx, linear_buf, w2e);
			memcpy(base, (uint8_t *)linear_buf + w2e, count - w2e);
			region->write_idx = count_align - w2e;
		}
	} else {
		memcpy(base + region->write_idx, linear_buf, count);
		region->write_idx += count_align;
	}
	DUMP_REGION(ipi_dbg, "out", region, count);
#if defined(DUMP_DMA_DEBUG)
	if (count < DMA_DUMP_MAX_SIZE)
		dump_size = count;

	if (count_align <= w2e) {
		memcpy(g_dump_buf, dump_start_idx, dump_size);
		pr_info("count(%u), dump size(%u), buf(%p)(%s)"
			, count, dump_size, dump_start_idx, g_dump_buf);
	} else {
		if (dump_size <= w2e) {
			memcpy(g_dump_buf, dump_start_idx, dump_size);
			pr_info("count(%u), dump size(%u), buf(%p)(%s)"
				, count, dump_size, dump_start_idx, g_dump_buf);
		} else {
			memcpy(g_dump_buf, dump_start_idx, w2e);
			memcpy(g_dump_buf, base, dump_size - w2e);
			pr_info(
				"count(%u), dump size(%u), buf1(%p), buf2(%p), buf1+2(%s)"
				, count, dump_size, dump_start_idx, base,
				g_dump_buf);
		}
	}
#endif
	return 0;
}


static int audio_region_read_to_linear(uint32_t dsp_id,
				       void *linear_buf,
				       struct audio_region_t *region,
				       uint32_t count)
{
	uint32_t count_align = DO_BYTE_ALIGN(count,
					     g_cache_alilgn_mask[dsp_id]);

	uint32_t available_count = 0;
	uint8_t *base = NULL;
	uint32_t r2e = 0;

	if (!region)
		return -EFAULT;
	if (!linear_buf)
		return -EFAULT;
	if (!dma_vir_base(dsp_id))
		return -EFAULT;

	if (region->size == 0) {
		DUMP_REGION(pr_notice, "size fail", region, count);
		return -ENODEV;
	}

	if (region->read_idx >= region->size) {
		DUMP_REGION(pr_notice, "read_idx fail", region, count);
		region->read_idx %= region->size;
	}
	if (region->write_idx >= region->size) {
		DUMP_REGION(pr_notice, "write_idx fail", region, count);
		region->write_idx %= region->size;
	}


	DUMP_REGION(ipi_dbg, "in", region, count);

	available_count = audio_region_data_count(region);
	if (count_align > available_count) {
		DUMP_REGION(pr_notice, "count_align > available_count",
			    region, count);
		return -ENOMEM;
	}

	base = dma_vir_base(dsp_id) + region->offset;

	if (region->read_idx <= region->write_idx) {
		memcpy(linear_buf, base + region->read_idx, count);
		region->read_idx += count_align;
	} else {
		r2e = region->size - region->read_idx;
		if (r2e >= count_align) {
			memcpy(linear_buf, base + region->read_idx, count);
			region->read_idx += count_align;
			if (region->read_idx == region->size)
				region->read_idx = 0;
		} else {
			memcpy(linear_buf, base + region->read_idx, r2e);
			memcpy((uint8_t *)linear_buf + r2e, base, count - r2e);
			region->read_idx = count_align - r2e;
		}
	}

	DUMP_REGION(ipi_dbg, "out", region, count);

	return 0;
}


static int audio_region_drop(uint32_t dsp_id,
			     struct audio_region_t *region,
			     uint32_t count)
{
	uint32_t count_align = DO_BYTE_ALIGN(count,
					     g_cache_alilgn_mask[dsp_id]);
	uint32_t available_count = 0;
	uint8_t *base = NULL;
	uint32_t r2e = 0;

	if (!region)
		return -EFAULT;
	if (!dma_vir_base(dsp_id))
		return -EFAULT;

	if (region->size == 0) {
		DUMP_REGION(pr_notice, "size fail", region, count);
		return -ENODEV;
	}

	if (region->read_idx >= region->size) {
		DUMP_REGION(pr_notice, "read_idx fail", region, count);
		region->read_idx %= region->size;
	}
	if (region->write_idx >= region->size) {
		DUMP_REGION(pr_notice, "write_idx fail", region, count);
		region->write_idx %= region->size;
	}


	DUMP_REGION(ipi_dbg, "in", region, count);

	available_count = audio_region_data_count(region);
	if (count_align > available_count) {
		DUMP_REGION(pr_notice, "count_align > available_count",
			    region, count);
		return -ENOMEM;
	}

	base = dma_vir_base(dsp_id) + region->offset;

	if (region->read_idx <= region->write_idx)
		region->read_idx += count_align;
	else {
		r2e = region->size - region->read_idx;
		if (r2e >= count_align) {
			region->read_idx += count_align;
			if (region->read_idx == region->size)
				region->read_idx = 0;
		} else
			region->read_idx = count_align - r2e;
	}

	DUMP_REGION(ipi_dbg, "out", region, count);

	return 0;
}


int audio_ipi_dma_write_region(const uint8_t task,
			       const void *data_buf,
			       uint32_t data_size,
			       uint32_t *write_idx)
{
	struct audio_region_t *region = NULL;

	int ret = 0;
	uint32_t dsp_id = audio_get_dsp_id(task);

	if (task >= TASK_SCENE_SIZE) {
		pr_info("task: %d", task);
		return -EOVERFLOW;
	}

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id(%u) invalid!!!", dsp_id);
		return -ENODEV;
	}
	if (!g_dma[dsp_id]) {
		pr_info("g_dma[%u] is NULL!!! return -EFAULT", dsp_id);
		return -EFAULT;
	}
	if (!data_buf || !write_idx) {
		pr_info("buf: %p or idx: %p is NULL!!!", data_buf, write_idx);
		return -EFAULT;
	}
	if (data_size == 0) {
		pr_info("task: %d, data_size = 0", task);
		return -ENODATA;
	}

	region = &g_dma[dsp_id]->region[task][AUDIO_IPI_DMA_AP_TO_DSP];
	DUMP_REGION(ipi_dbg, "region", region, data_size);

	/* keep the data index before write */
	*write_idx = region->write_idx;

	/* write data */
	ret = audio_region_write_from_linear(dsp_id,
					     region, data_buf, data_size);

	return ret;
}


int audio_ipi_dma_read_region(const uint8_t task,
			      void *data_buf,
			      uint32_t data_size,
			      uint32_t read_idx)
{
	struct audio_region_t *region = NULL;

	int ret = 0;
	uint32_t dsp_id = audio_get_dsp_id(task);

	if (task >= TASK_SCENE_SIZE) {
		pr_info("task: %d", task);
		return -EOVERFLOW;
	}
	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id(%u) invalid!!!", dsp_id);
		return -ENODEV;
	}
	if (!g_dma[dsp_id]) {
		pr_info("g_dma[%u] is NULL!!! return -EFAULT", dsp_id);
		return -EFAULT;
	}
	if (!data_buf) {
		pr_info("buf: %p is NULL!!!", data_buf);
		return -EFAULT;
	}
	if (data_size == 0) {
		pr_info("task: %d, data_size = 0", task);
		return -ENODATA;
	}

	region = &g_dma[dsp_id]->region[task][AUDIO_IPI_DMA_DSP_TO_AP];
	DUMP_REGION(ipi_dbg, "region", region, data_size);

	/* check read index */
	if (read_idx != region->read_idx) {
		pr_debug("read_idx 0x%x != region->read_idx 0x%x!!",
			 read_idx, region->read_idx);
		region->read_idx = read_idx;
	}

	/* read data */
	ret = audio_region_read_to_linear(dsp_id,
					  data_buf, region, data_size);

	return ret;
}


int audio_ipi_dma_drop_region(const uint8_t task,
			      uint32_t drop_size,
			      uint32_t read_idx)

{
	struct audio_region_t *region = NULL;

	int ret = 0;
	uint32_t dsp_id = audio_get_dsp_id(task);

	if (task >= TASK_SCENE_SIZE)
		return -EOVERFLOW;
	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -ENODEV;
	if (!g_dma[dsp_id])
		return -EFAULT;
	if (drop_size == 0)
		return -ENODATA;

	region = &g_dma[dsp_id]->region[task][AUDIO_IPI_DMA_DSP_TO_AP];
	DUMP_REGION(ipi_dbg, "region", region, drop_size);

	/* check read index */
	if (read_idx != region->read_idx) {
		pr_debug("read_idx 0x%x != region->read_idx 0x%x!!",
			 read_idx, region->read_idx);
		region->read_idx = read_idx;
	}

	/* drop data */
	ret = audio_region_drop(dsp_id, region, drop_size);

	return ret;
}



/*
 * =============================================================================
 *                     DSP -> DMA -> HAL
 * =============================================================================
 */

inline bool hal_dma_check_idx_msg_valid(
	const struct hal_dma_queue_t *msg_queue,
	const uint32_t idx_msg)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	return (idx_msg < msg_queue->size) ? true : false;
}


inline bool hal_dma_check_queue_empty(const struct hal_dma_queue_t *msg_queue)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	return (msg_queue->idx_r == msg_queue->idx_w);
}


inline bool hal_dma_check_queue_to_be_full(
	const struct hal_dma_queue_t *msg_queue)
{
	uint32_t idx_w_to_be = 0;

	if (msg_queue == NULL)
		return false;

	idx_w_to_be = msg_queue->idx_w + 1;
	if (idx_w_to_be == msg_queue->size)
		idx_w_to_be = 0;

	return (idx_w_to_be == msg_queue->idx_r) ? true : false;
}


inline uint32_t hal_dma_get_num_msg_in_queue(
	const struct hal_dma_queue_t *msg_queue)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return 0;
	}

	return (msg_queue->idx_w >= msg_queue->idx_r) ?
	       (msg_queue->idx_w - msg_queue->idx_r) :
	       ((msg_queue->size - msg_queue->idx_r) + msg_queue->idx_w);
}


static int hal_dma_push(
	struct hal_dma_queue_t *msg_queue,
	struct ipi_msg_t *p_ipi_msg,
	uint32_t *p_idx_msg)
{
	uint32_t data_size = 0;
	uint32_t free_space = 0;
	int retval = 0;

	if (msg_queue == NULL || p_ipi_msg == NULL || p_idx_msg == NULL)
		return -EFAULT;

	/* get data from DMA ASAP s.t. adsp could fill more data */
	data_size = p_ipi_msg->dma_info.data_size;
	if ((data_size + sizeof(struct ipi_msg_t)) > MAX_DSP_DMA_WRITE_SIZE) {
		pr_info("task: %d, msg_id: 0x%x, data overflow, data_size %u, drop it",
			p_ipi_msg->task_scene, p_ipi_msg->msg_id, data_size);
		audio_ipi_dma_drop_region(p_ipi_msg->task_scene,
					  data_size,
					  p_ipi_msg->dma_info.rw_idx);
		WARN_ON(1); /* enlarge tmp_buf_d2k!! no realloc in ISR() */
		return -EOVERFLOW;
	}
	retval = audio_ipi_dma_read_region(
			 p_ipi_msg->task_scene,
			 msg_queue->tmp_buf_d2k,
			 data_size,
			 p_ipi_msg->dma_info.rw_idx);
	if (retval != 0)
		return retval;

	/* copy data & push msg to queue */
	free_space = audio_ringbuf_free_space(&msg_queue->dma_data);
	if (hal_dma_check_queue_to_be_full(msg_queue) == true ||
	    hal_dma_check_idx_msg_valid(msg_queue, msg_queue->idx_w) == false ||
	    data_size > free_space) {
		pr_info_once("task: %d, msg_id: 0x%x, queue overflow, idx_r: %u, idx_w: %u, drop it",
			     p_ipi_msg->task_scene, p_ipi_msg->msg_id,
			     msg_queue->idx_r, msg_queue->idx_w);
		return -EOVERFLOW;
	}

	*p_idx_msg = msg_queue->idx_w;
	msg_queue->idx_w++;
	if (msg_queue->idx_w == msg_queue->size)
		msg_queue->idx_w = 0;

	memcpy((void *)&msg_queue->msg[*p_idx_msg],
	       p_ipi_msg,
	       sizeof(struct ipi_msg_t));

	audio_ringbuf_copy_from_linear_impl(
		&msg_queue->dma_data,
		msg_queue->tmp_buf_d2k,
		data_size);

	return 0;
}


static int hal_dma_pop(struct hal_dma_queue_t *msg_queue)
{
	struct ipi_msg_t *p_ipi_msg = NULL;

	if (msg_queue == NULL) {
		pr_info("NULL!! msg_queue: %p", msg_queue);
		return -EFAULT;
	}

	/* check queue empty */
	if (hal_dma_check_queue_empty(msg_queue) == true) {
		pr_info("queue is empty, idx_r: %u, idx_w: %u",
			msg_queue->idx_r,
			msg_queue->idx_w);
		return -1;
	}

	/* pop */
	p_ipi_msg = &msg_queue->msg[msg_queue->idx_r];
	msg_queue->idx_r++;
	if (msg_queue->idx_r == msg_queue->size)
		msg_queue->idx_r = 0;


	ipi_dbg("task: %d, msg_id: 0x%x, idx_r: %u, idx_w: %u, queue(%u/%u)",
		p_ipi_msg->task_scene,
		p_ipi_msg->msg_id,
		msg_queue->idx_r,
		msg_queue->idx_w,
		hal_dma_get_num_msg_in_queue(msg_queue),
		msg_queue->size);

	return 0;
}


static int hal_dma_front(
	struct hal_dma_queue_t *msg_queue,
	struct ipi_msg_t **pp_ipi_msg,
	uint32_t *p_idx_msg)
{
	uint32_t data_size = 0;

	if (msg_queue == NULL || pp_ipi_msg == NULL || p_idx_msg == NULL) {
		pr_info("NULL!! msg_queue: %p, pp_ipi_msg: %p, p_idx_msg: %p",
			msg_queue, pp_ipi_msg, p_idx_msg);
		return -EFAULT;
	}

	*pp_ipi_msg = NULL;
	*p_idx_msg = 0xFFFFFFFF;

	/* check queue empty */
	if (hal_dma_check_queue_empty(msg_queue) == true) {
		pr_info("queue empty, idx_r: %u, idx_w: %u",
			msg_queue->idx_r, msg_queue->idx_w);
		return -ENOMEM;
	}

	/* front */
	if (hal_dma_check_idx_msg_valid(msg_queue, msg_queue->idx_r) == false) {
		pr_info("idx_r %u is invalid!! return",
			msg_queue->idx_r);
		return -1;
	}
	*p_idx_msg = msg_queue->idx_r;
	*pp_ipi_msg = &msg_queue->msg[*p_idx_msg];

	data_size = (*pp_ipi_msg)->dma_info.data_size;
	if ((data_size + sizeof(struct ipi_msg_t)) > MAX_DSP_DMA_WRITE_SIZE) {
		pr_notice("%u + %zu > %u!!",
			  data_size, sizeof(struct ipi_msg_t),
			  MAX_DSP_DMA_WRITE_SIZE);
		audio_ringbuf_drop_data(&msg_queue->dma_data, data_size);
		hal_dma_pop(msg_queue);
		return -EOVERFLOW;
	}

	memcpy(msg_queue->tmp_buf_k2h, *pp_ipi_msg, sizeof(struct ipi_msg_t));

	audio_ringbuf_copy_to_linear(
		msg_queue->tmp_buf_k2h + sizeof(struct ipi_msg_t),
		&msg_queue->dma_data,
		data_size);

	return 0;
}


static int hal_dma_init_msg_queue(struct hal_dma_queue_t *msg_queue,
				  const uint32_t size)
{
	char *ring;
	uint8_t *d2k, *k2h;
	uint32_t ring_size = size / 2;

	if (!msg_queue || !ring_size)
		return -EINVAL;
	if (msg_queue->dma_data.base || msg_queue->tmp_buf_d2k || msg_queue->tmp_buf_k2h)
		return -EALREADY;
	ring = vmalloc(ring_size);
	if (!ring)
		return -ENOMEM;
	d2k = vmalloc(MAX_DSP_DMA_WRITE_SIZE);
	if (!d2k) {
		vfree(ring);
		return -ENOMEM;
	}
	k2h = vmalloc(MAX_DSP_DMA_WRITE_SIZE);
	if (!k2h) {
		vfree(d2k);
		vfree(ring);
		return -ENOMEM;
	}
	memset(msg_queue->msg, 0, sizeof(msg_queue->msg));
	msg_queue->size = MAX_DSP_MSG_NUM_IN_QUEUE;
	msg_queue->idx_r = 0;
	msg_queue->idx_w = 0;
	spin_lock_init(&msg_queue->queue_lock);
	init_waitqueue_head(&msg_queue->queue_wq);
	msg_queue->dma_data.size = ring_size;
	msg_queue->dma_data.base = ring;
	msg_queue->dma_data.read = ring;
	msg_queue->dma_data.write = ring;
	msg_queue->tmp_buf_d2k = d2k;
	msg_queue->tmp_buf_k2h = k2h;
	return 0;
}

static int hal_dma_deinit_msg_queue(struct hal_dma_queue_t *msg_queue)
{
	if (!msg_queue)
		return -EINVAL;
	vfree(msg_queue->dma_data.base);
	vfree(msg_queue->tmp_buf_d2k);
	vfree(msg_queue->tmp_buf_k2h);
	memset(&msg_queue->dma_data, 0, sizeof(msg_queue->dma_data));
	msg_queue->tmp_buf_d2k = NULL;
	msg_queue->tmp_buf_k2h = NULL;
	msg_queue->size = 0;
	msg_queue->idx_r = 0;
	msg_queue->idx_w = 0;
	return 0;
}


static int hal_dma_get_queue_msg(
	struct hal_dma_queue_t *msg_queue,
	struct ipi_msg_t **pp_ipi_msg,
	uint32_t *p_idx_msg)
{
	bool is_empty = true;

	unsigned long flags = 0;
	int retval = 0;

	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	is_empty = hal_dma_check_queue_empty(msg_queue);
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);

	/* wait until message is pushed to queue */
	if (is_empty == true) {
		retval = wait_event_interruptible(
				 msg_queue->queue_wq,
				 !hal_dma_check_queue_empty(msg_queue));
		if (retval == -ERESTARTSYS) {
			pr_info("-ERESTARTSYS");
			retval = -EINTR;
		}
	}

	if (hal_dma_check_queue_empty(msg_queue) == false) {
		spin_lock_irqsave(&msg_queue->queue_lock, flags);
		retval = hal_dma_front(msg_queue, pp_ipi_msg, p_idx_msg);
		spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	}

	return retval;
}


int audio_ipi_dma_msg_to_hal(struct ipi_msg_t *p_ipi_msg)
{
	struct hal_dma_queue_t *msg_queue = &g_hal_dma_queue;

	uint32_t idx_msg = 0;

	unsigned long flags = 0;
	int retval = 0;

	if (p_ipi_msg == NULL || msg_queue == NULL)
		return -EFAULT;

	if (p_ipi_msg->data_type != AUDIO_IPI_DMA ||
	    p_ipi_msg->target_layer != AUDIO_IPI_LAYER_TO_HAL ||
	    p_ipi_msg->dma_info.data_size == 0)
		return -EFAULT;

	/* push message to queue */
	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	retval = hal_dma_push(msg_queue, p_ipi_msg, &idx_msg);
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	if (retval != 0)
		return retval;

	/* notify queue thread to process it */
	dsb(SY);
	wake_up_interruptible(&msg_queue->queue_wq);

	return 0;
}


size_t audio_ipi_dma_msg_read(void __user *buf, size_t count)
{
	struct hal_dma_queue_t *msg_queue = &g_hal_dma_queue;
	struct ipi_msg_t *p_ipi_msg = NULL;
	uint32_t idx_msg = 0;

	size_t copy_size = 0;

	unsigned long flags = 0;
	int retval = 0;

	if (buf == NULL || count == 0 || msg_queue == NULL) {
		pr_info("arg!! %p %zu %p, return", buf, count, msg_queue);
		return 0;
	}


	/* wait until element pushed */
	retval = hal_dma_get_queue_msg(msg_queue, &p_ipi_msg, &idx_msg);
	if (retval != 0) {
		pr_info("hal_dma_get_queue_msg retval %d", retval);
		return 0;
	}
	p_ipi_msg = &msg_queue->msg[idx_msg];

#ifdef DEBUG_IPI
	DUMP_IPI_MSG("dma kernel -> hal", p_ipi_msg);
#endif

	/* copy data */
	copy_size = sizeof(struct ipi_msg_t) +
		    p_ipi_msg->dma_info.data_size;
	if (count < copy_size) {
		pr_notice("%zu < %zu + %u!! drop",
			  count,
			  sizeof(struct ipi_msg_t),
			  p_ipi_msg->dma_info.data_size);
		copy_size = 0;
	} else {
		retval = copy_to_user(
				 buf,
				 msg_queue->tmp_buf_k2h,
				 copy_size);
		if (retval != 0)
			pr_info("hal_dma_get_queue_msg retval %d", retval);
	}


	/* pop message from queue */
	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	hal_dma_pop(msg_queue);
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);

	return copy_size;
}



