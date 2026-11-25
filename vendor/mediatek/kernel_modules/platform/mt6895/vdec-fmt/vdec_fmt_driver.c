// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/file.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include "vdec_fmt_driver.h"
#include "vdec_fmt_dmabuf.h"
#include "vdec_fmt_pm.h"
#include "vdec_fmt_utils.h"
#include "iommu_debug.h"

static struct mtk_vdec_fmt *fmt_mtkdev;
static DEFINE_MUTEX(fmt_native_ioctl_lock);

int fmt_dbg_level;

module_param(fmt_dbg_level, int, 0644);

static int fmt_check_reg_base(struct mtk_vdec_fmt *fmt, u64 addr, u64 length)
{
	int i;

	if (addr >= MAP_PA_BASE_1GB)
		return -EINVAL;

	for (i = 0; i < (int)FMT_MAP_HW_REG_NUM; i++)
		if (addr >= (u64)fmt->map_base[i].base &&
			addr + length <= (u64)fmt->map_base[i].base + fmt->map_base[i].len)
			return i;
	fmt_err("addr %llx length %llx not found!", addr, length);

	return -EINVAL;
}

static int fmt_lock(u32 id, bool sec)
{
	unsigned int suspend_block_cnt = 0;
	int ret = -1;
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;
	enum fmt_gce_status gce_status = GCE_NONE;

	if (id >= fmt->gce_th_num)
		return ret;

	while (fmt->is_entering_suspend == 1) {
		suspend_block_cnt++;
		if (suspend_block_cnt > SUSPEND_TIMEOUT_CNT) {
			fmt_debug(0, "blocked by suspend");
			suspend_block_cnt = 0;
		}
		usleep_range(10000, 20000);
	}

	//if (sec != false)
	//	gce_status = GCE_SECURE;
	//else
	// FMT only support sec in TZMP flow, always use normal GCE
	gce_status = GCE_NORMAL;

	fmt_debug(1, "id %d sec %d gce_status %d cur gce_status %d",
				id, sec,
				gce_status,
				fmt->gce_status[id]);

	// only need to trylock semaphore when gce status change
	// NONE -> NORMAL/ NONE -> SECURE / NORMAL <-> SECURE
	if (gce_status != fmt->gce_status[id])
		ret = down_trylock(&fmt->fmt_sem[id]);
	else
		ret = 0;

	if (ret == 0)
		fmt->gce_status[id] = gce_status;

	return ret;
}

static void fmt_unlock(u32 id)
{
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	if (id >= fmt->gce_th_num)
		return;

	fmt_debug(1, "id %d cur gce_status %d", id, fmt->gce_status[id]);

	up(&fmt->fmt_sem[id]);

	fmt->gce_status[id] = GCE_NONE;
}

static int fmt_set_gce_task(struct cmdq_pkt *pkt_ptr, u32 id,
	struct dmabuf_info iinfo, struct dmabuf_info oinfo)
{
	int i;
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	mutex_lock(&fmt->mux_task);
	for (i = 0; i < FMT_INST_MAX; i++) {
		if (fmt->gce_task[i].used == 0 && fmt->gce_task[i].pkt_ptr == NULL) {
			fmt->gce_task[i].used = 1;
			fmt->gce_task[i].pkt_ptr = pkt_ptr;
			fmt->gce_task[i].identifier = id;
			fmt->gce_task[i].iinfo = iinfo;
			fmt->gce_task[i].oinfo = oinfo;
			mutex_unlock(&fmt->mux_task);
			fmt_debug(1, "Set identifier from %d to %d taskid %d pkt_ptr %p",
				fmt->gce_task[i].identifier, id, i, pkt_ptr);

			return i;
		}
	}
	mutex_unlock(&fmt->mux_task);

	return -1;
}

static void fmt_clear_gce_task(unsigned int taskid)
{
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	mutex_lock(&fmt->mux_task);
	if (fmt->gce_task[taskid].used == 1 && fmt->gce_task[taskid].pkt_ptr != NULL) {
		fmt_debug(1, "taskid %d pkt_ptr %p, set id %d to %d",
			taskid, fmt->gce_task[taskid].pkt_ptr, fmt->gce_task[taskid].identifier, 0);
		fmt->gce_task[taskid].used = 0;
		fmt->gce_task[taskid].pkt_ptr = NULL;
		fmt->gce_task[taskid].identifier = 0;
		fmt_dmabuf_free_iova(fmt->gce_task[taskid].iinfo.dbuf,
		fmt->gce_task[taskid].iinfo.attach, fmt->gce_task[taskid].iinfo.sgt);
		fmt_dmabuf_free_iova(fmt->gce_task[taskid].oinfo.dbuf,
		fmt->gce_task[taskid].oinfo.attach, fmt->gce_task[taskid].oinfo.sgt);
		fmt_dmabuf_put(fmt->gce_task[taskid].iinfo.dbuf);
		fmt_dmabuf_put(fmt->gce_task[taskid].oinfo.dbuf);
	}
	mutex_unlock(&fmt->mux_task);
}

void cmdq_usage_cb_func(u32 thd_id)
{
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	fmt_debug(0, "thd:%d", thd_id);
	queue_work(fmt->cmdq_cb_workqueue, &fmt->cmdq_cb_work);
}

static void fmt_active_resource_time_check(struct work_struct *work)
{
 /* Completion and file release own packets and DMA attachments. A callback
  * cannot destroy them based on elapsed wall time while userspace waits. */
 (void)work;
}

static int fmt_set_gce_cmd(struct cmdq_pkt *pkt,
	unsigned char cmd, u64 addr, u64 data, u32 mask, u32 gpr, unsigned int idx,
	struct dmabuf_info *iinfo, struct dmabuf_info *oinfo, struct dmabufmap map[],
	bool secure)
{
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;
	int type;
	u64 iova = 0;

 if (cmd >= CMD_MAX) return -EINVAL;
 if ((cmd == CMD_READ || cmd == CMD_WRITE || cmd == CMD_POLL_REG || cmd == CMD_WRITE_FD) &&
     ((addr & 3) || fmt_check_reg_base(fmt, addr, 4) < 0)) return -EINVAL;
 if (cmd == CMD_WRITE_RDMA && addr > CPR_IDX_FMT_RDMA_PIPE_IDX) return -EINVAL;
 if (cmd == CMD_WRITE_FD_RDMA && addr > CPR_IDX_FMT_RDMA_AFBC_PAYLOAD_OST) return -EINVAL;
 if ((cmd == CMD_WAIT_EVENT || cmd == CMD_SET_EVENT || cmd == CMD_CLEAR_EVENT) &&
     (data > SYNC_TOKEN_PREBUILT_VFMT_LOCK || (cmd == CMD_CLEAR_EVENT && data >= GCE_EVENT_MAX) ||
      (cmd == CMD_SET_EVENT && data < SYNC_TOKEN_PREBUILT_VFMT_WAIT))) return -EINVAL;

	switch (cmd) {
	case CMD_READ:
		fmt_debug(3, "CMD_READ addr 0x%llx", addr);
		if (fmt_check_reg_base(fmt, addr, 4) >= 0)
			return cmdq_pkt_read_addr(pkt, addr, CMDQ_THR_SPR_IDX1);
		else
			fmt_err("CMD_READ wrong addr: 0x%llx", addr);
	break;
	case CMD_WRITE:
		fmt_debug(3, "CMD_WRITE addr 0x%llx data 0x%llx mask 0x%x", addr, data, mask);
		if (fmt_check_reg_base(fmt, addr, 4) >= 0)
			return cmdq_pkt_write(pkt, fmt->clt_base, addr, data, mask);
		else
			fmt_err("CMD_WRITE wrong addr: 0x%llx 0x%llx 0x%x",
				addr, data, mask);
	break;
	case CMD_WRITE_RDMA:
		fmt_debug(3, "CMD_WRITE_RDMA cpridx %llu data 0x%llx offset 0x%x",
				addr, data, mask);
		if (addr == CPR_IDX_FMT_RDMA_PIPE_IDX) {
			fmt_debug(3, "cmdq_pkt_assign_command idx %d cpr %llu value 0x%llx",
					idx, addr, data);
			return cmdq_pkt_assign_command(pkt,
					CMDQ_CPR_PREBUILT_PIPE(CMDQ_PREBUILT_VFMT),
					idx);
		} else if (addr <= CPR_IDX_FMT_RDMA_AFBC_PAYLOAD_OST) {
			fmt_debug(3, "cmdq_pkt_assign_command idx %d cpr %llu value 0x%llx",
					idx, addr, data);
			return cmdq_pkt_assign_command(pkt,
					CMDQ_CPR_PREBUILT(CMDQ_PREBUILT_VFMT, idx, addr),
					data);
		} else {
			fmt_err("invalid GCE cpr index %llu", addr);
		}
	break;
	case CMD_POLL_REG:
		fmt_debug(3, "CMD_POLL_REG addr 0x%llx data 0x%llx mask 0x%x", addr, data, mask);
		if (fmt_check_reg_base(fmt, addr, 4) >= 0)
			return cmdq_pkt_poll_addr(pkt, data, addr, mask, gpr);
		else
			fmt_err("CMD_POLL_REG wrong addr: 0x%llx 0x%llx 0x%x",
				addr, data, mask);
	break;
	case CMD_WAIT_EVENT:
		if (data < GCE_EVENT_MAX) { // fmt hw event
			fmt_debug(3, "CMD_WAIT_EVENT eid %d", fmt->gce_codec_eid[data]);
			return cmdq_pkt_wfe(pkt, fmt->gce_codec_eid[data]);
		} else if (data >= SYNC_TOKEN_PREBUILT_VFMT_WAIT // rdma sw wa sync event
					&& data <= SYNC_TOKEN_PREBUILT_VFMT_LOCK) {
			switch (data) {
			case SYNC_TOKEN_PREBUILT_VFMT_WAIT:
				fmt_debug(3, "cmdq_pkt_wfe %d",
					CMDQ_TOKEN_PREBUILT_VFMT_WAIT);
				return cmdq_pkt_wfe(pkt, CMDQ_TOKEN_PREBUILT_VFMT_WAIT);
			break;
			case SYNC_TOKEN_PREBUILT_VFMT_SET:
				fmt_debug(3, "cmdq_pkt_wfe %d",
					CMDQ_TOKEN_PREBUILT_VFMT_SET);
				return cmdq_pkt_wfe(pkt, CMDQ_TOKEN_PREBUILT_VFMT_SET);
			break;
			case SYNC_TOKEN_PREBUILT_VFMT_LOCK:
				fmt_debug(3, "cmdq_pkt_wfe %d",
					CMDQ_TOKEN_PREBUILT_VFMT_LOCK);
				return cmdq_pkt_wfe(pkt, CMDQ_TOKEN_PREBUILT_VFMT_LOCK);
			break;
			default:
			break;
			}
			fmt_debug(3, "CMD_WAIT_EVENT %llu", data);
		} else
			fmt_err("CMD_WAIT_EVENT got wrong eid %llu",
				data);
	break;
	case CMD_SET_EVENT:
		if (data >= SYNC_TOKEN_PREBUILT_VFMT_WAIT // rdma sw wa sync event
			&& data <= SYNC_TOKEN_PREBUILT_VFMT_LOCK) {
			switch (data) {
			case SYNC_TOKEN_PREBUILT_VFMT_WAIT:
				fmt_debug(3, "cmdq_pkt_set_event %d",
					CMDQ_TOKEN_PREBUILT_VFMT_WAIT);
				return cmdq_pkt_set_event(pkt, CMDQ_TOKEN_PREBUILT_VFMT_WAIT);
			break;
			case SYNC_TOKEN_PREBUILT_VFMT_SET:
				fmt_debug(3, "cmdq_pkt_set_event %d",
					CMDQ_TOKEN_PREBUILT_VFMT_SET);
				return cmdq_pkt_set_event(pkt, CMDQ_TOKEN_PREBUILT_VFMT_SET);
			break;
			case SYNC_TOKEN_PREBUILT_VFMT_LOCK:
				fmt_debug(3, "cmdq_pkt_set_event %d",
					CMDQ_TOKEN_PREBUILT_VFMT_LOCK);
				return cmdq_pkt_set_event(pkt, CMDQ_TOKEN_PREBUILT_VFMT_LOCK);
			break;
			default:
			break;
			}
			fmt_debug(3, "CMD_SET_EVENT %llu", data);
		} else
			fmt_err("CMD_WAIT_EVENT got wrong eid %llu",
				data);
	break;
	case CMD_CLEAR_EVENT:
		if (data < GCE_EVENT_MAX) {
			fmt_debug(3, "CMD_CLEAR_EVENT eid %d", fmt->gce_codec_eid[data]);
			return cmdq_pkt_clear_event(pkt, fmt->gce_codec_eid[data]);
		} else
			fmt_err("got wrong eid %llu",
				data);
	break;
	case CMD_WRITE_FD:
		/* Native FMT uses fd=0 for an absent chroma/third plane: emit
		 * zero, without acquiring or releasing another plane owner. */
		fmt_debug(3, "CMD_WRITE_FD addr 0x%llx fd 0x%llx offset 0x%x", addr, data, mask);
		type = fmt_check_reg_base(fmt, addr, 4);
		if (type >= 0 && type < FMT_MAP_HW_REG_NUM) {
			if (type % 2 == 0) {
				iova = data ? fmt_translate_fd(data, mask, map, fmt->dev,
					&iinfo->dbuf, &iinfo->attach, &iinfo->sgt, false) : 0;
				if (data && (!iova || iinfo->attach == NULL || iinfo->sgt == NULL))
					return -1;
				if (addr - fmt->map_base[type].base >= 0xf30) { // rdma 34bit
					fmt_debug(3, "cmdq_pkt_write base: 0x%lx addr:0x%llx value:0x%llx",
						fmt->map_base[type].base, addr, (iova >> 32));
					return cmdq_pkt_write(pkt, fmt->clt_base, addr,
						(iova >> 32), ~0);
				} else {
					fmt_debug(3, "cmdq_pkt_write base: 0x%lx addr:0x%llx value:0x%llx",
							fmt->map_base[type].base, addr, iova);
					return cmdq_pkt_write(pkt, fmt->clt_base, addr,
						iova, ~0);
				}
			} else {
				iova = data ? fmt_translate_fd(data, mask, map, fmt->dev,
						&oinfo->dbuf, &oinfo->attach, &oinfo->sgt, !secure) : 0;
				if (data && (!iova || oinfo->attach == NULL || oinfo->sgt == NULL))
					return -1;
				if (addr - fmt->map_base[type].base >= 0xf34) { // wrot 34bit
					fmt_debug(3, "cmdq_pkt_write base: 0x%lx addr:0x%llx value:0x%llx",
						fmt->map_base[type].base, addr, (iova >> 32));
					return cmdq_pkt_write(pkt, fmt->clt_base, addr,
						(iova >> 32), ~0);
				} else {
					fmt_debug(3, "cmdq_pkt_write base: 0x%lx addr:0x%llx value:0x%llx",
							fmt->map_base[type].base, addr, iova);
					return cmdq_pkt_write(pkt, fmt->clt_base, addr,
						iova, ~0);
				}
			}
		} else
			fmt_err("CMD_WRITE_FD wrong addr: 0x%llx 0x%llx 0x%x",
				addr, data, mask);
	break;
	case CMD_WRITE_FD_RDMA:
		fmt_debug(3, "CMD_WRITE_FD_RDMA cpridx %llu fd 0x%llx offset 0x%x", addr, data, mask);
		iova = data ? fmt_translate_fd(data, mask, map, fmt->dev,
					&iinfo->dbuf, &iinfo->attach, &iinfo->sgt, false) : 0;
		if (data && (!iova || iinfo->attach == NULL || iinfo->sgt == NULL))
			return -1;
		if (addr >= CPR_IDX_FMT_RDMA_SRC_OFFSET_0
			&& addr <= CPR_IDX_FMT_RDMA_UFO_DEC_LENGTH_BASE_C) {
			fmt_debug(3, "cmdq_pkt_assign_command idx %d cpr %llu value 0x%llx",
						idx, addr, iova);
			return cmdq_pkt_assign_command(pkt,
					CMDQ_CPR_PREBUILT(CMDQ_PREBUILT_VFMT, idx, addr),
					iova);
		} else if (addr >= CPR_IDX_FMT_RDMA_SRC_BASE_0_MSB
					&& addr <= CPR_IDX_FMT_RDMA_SRC_OFFSET_2_MSB){ // 34bit msb
			fmt_debug(3, "cmdq_pkt_assign_command idx %d cpr %llu value 0x%llx",
						idx, addr, (iova >> 32));
			return cmdq_pkt_assign_command(pkt,
					CMDQ_CPR_PREBUILT(CMDQ_PREBUILT_VFMT, idx, addr),
					(iova >> 32));
		} else {
			fmt_err("invalid GCE cpr index %llu", addr);
		}
	break;
	default:
		fmt_err("unknown GCE cmd %d", cmd);
	break;
	}
	return 0;
}

#define ADDR0(offset) \
	(fmt->map_base[0+idx*2].base + offset)
#define ADDR1(offset) \
	(fmt->map_base[1+idx*2].base + offset)
#define REG0(offset) \
	FMT_GET32(fmt->map_base[0+idx*2].va + offset)
#define REG1(offset) \
	FMT_GET32(fmt->map_base[1+idx*2].va + offset)
static void fmt_dump_addr_reg(void)
{
	int idx;
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	if (fmt->dtsInfo.RDMA_needWA)
		cmdq_util_prebuilt_dump(1, CMDQ_TOKEN_PREBUILT_VFMT_WAIT);

	if (!fmt->dtsInfo.RDMA_needWA) {
		for (idx = 0; idx < fmt->dtsInfo.pipeNum; idx++) {
			fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
			idx, ADDR0(0xF00), REG0(0xF00), ADDR0(0xF08), REG0(0xF08),
			ADDR0(0xF10), REG0(0xF10));
			fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
			idx, ADDR0(0xF20), REG0(0xF20), ADDR0(0xF28), REG0(0xF28),
			ADDR0(0xF54), REG0(0xF54));
		}
	}


	for (idx = 0; idx < fmt->dtsInfo.pipeNum; idx++) {
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x0), REG0(0x0), ADDR0(0x4), REG0(0x4),
		ADDR0(0x20), REG0(0x20), ADDR0(0x24), REG0(0x24));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x28), REG0(0x28), ADDR0(0x30), REG0(0x30),
		ADDR0(0x38), REG0(0x38), ADDR0(0x60), REG0(0x60));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x68), REG0(0x68), ADDR0(0x70), REG0(0x70),
		ADDR0(0x78), REG0(0x78), ADDR0(0x90), REG0(0x90),
		ADDR0(0x98), REG0(0x98));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x240), REG0(0x240), ADDR0(0x244), REG0(0x244),
		ADDR0(0x248), REG0(0x248), ADDR0(0x250), REG0(0x250));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x254), REG0(0x254), ADDR0(0x258), REG0(0x258),
		ADDR0(0x260), REG0(0x260), ADDR0(0x264), REG0(0x264));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x268), REG0(0x268), ADDR0(0x270), REG0(0x270),
		ADDR0(0x274), REG0(0x274), ADDR0(0x278), REG0(0x278));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x280), REG0(0x280), ADDR0(0x284), REG0(0x284),
		ADDR0(0x288), REG0(0x288), ADDR0(0x290), REG0(0x290));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x2A0), REG0(0x2A0), ADDR0(0x2A8), REG0(0x2A8),
		ADDR0(0x2B8), REG0(0x2B8));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x2C0), REG0(0x2C0), ADDR0(0x2C4), REG0(0x2C4),
		ADDR0(0x2C8), REG0(0x2C8));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x400), REG0(0x400), ADDR0(0x408), REG0(0x408),
		ADDR0(0x410), REG0(0x410), ADDR0(0x418), REG0(0x418));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x420), REG0(0x420), ADDR0(0x428), REG0(0x428),
		ADDR0(0x430), REG0(0x430), ADDR0(0x438), REG0(0x438));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x440), REG0(0x440), ADDR0(0x448), REG0(0x448),
		ADDR0(0x450), REG0(0x450), ADDR0(0x458), REG0(0x458));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x460), REG0(0x460), ADDR0(0x468), REG0(0x468),
		ADDR0(0x470), REG0(0x470), ADDR0(0x478), REG0(0x478));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x480), REG0(0x480), ADDR0(0x488), REG0(0x488),
		ADDR0(0x490), REG0(0x490), ADDR0(0x498), REG0(0x498));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x4A0), REG0(0x4A0), ADDR0(0x4A8), REG0(0x4A8),
		ADDR0(0x4B0), REG0(0x4B0), ADDR0(0x4B8), REG0(0x4B8));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR0(0x4C0), REG0(0x4C0), ADDR0(0x4C8), REG0(0x4C8),
		ADDR0(0x4D0), REG0(0x4D0), ADDR0(0x4D8), REG0(0x4D8),
		ADDR0(0x4E0), REG0(0x4E0));
		fmt_debug(0, "RDMA%d(0x%lx) 0x%x",
		idx, ADDR0(0xF54), REG0(0xF54));

		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x0), REG1(0x0), ADDR1(0x4), REG1(0x4),
		ADDR1(0x8), REG1(0x8), ADDR1(0xC), REG1(0xC));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x10), REG1(0x10), ADDR1(0x14), REG1(0x14),
		ADDR1(0x18), REG1(0x18), ADDR1(0x1C), REG1(0x1C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x20), REG1(0x20), ADDR1(0x24), REG1(0x24),
		ADDR1(0x28), REG1(0x28), ADDR1(0x2C), REG1(0x2C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x30), REG1(0x30), ADDR1(0x34), REG1(0x34),
		ADDR1(0x38), REG1(0x38), ADDR1(0x3C), REG1(0x3C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x40), REG1(0x40), ADDR1(0x44), REG1(0x44),
		ADDR1(0x48), REG1(0x48));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x50), REG1(0x50), ADDR1(0x54), REG1(0x54),
		ADDR1(0x58), REG1(0x58), ADDR1(0x5C), REG1(0x5C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx)) 0x%x",
		idx, ADDR1(0x68), REG1(0x68), ADDR1(0x6C), REG1(0x6C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x70), REG1(0x70), ADDR1(0x74), REG1(0x74),
		ADDR1(0x78), REG1(0x78), ADDR1(0x7C), REG1(0x7C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0x80), REG1(0x80), ADDR1(0x84), REG1(0x84),
		ADDR1(0x88), REG1(0x88), ADDR1(0x8C), REG1(0x8C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0xD0), REG1(0xD0), ADDR1(0xD4), REG1(0xD4),
		ADDR1(0xD8), REG1(0xD8), ADDR1(0xDC), REG1(0xDC));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0xE0), REG1(0xE0), ADDR1(0xE4), REG1(0xE4),
		ADDR1(0xE8), REG1(0xE8), ADDR1(0xEC), REG1(0xEC),
		ADDR1(0xF0), REG1(0xF0));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0xF00), REG1(0xF00), ADDR1(0xF04), REG1(0xF04),
		ADDR1(0xF08), REG1(0xF08), ADDR1(0xF0C), REG1(0xF0C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0xF10), REG1(0xF10), ADDR1(0xF14), REG1(0xF14),
		ADDR1(0xF18), REG1(0xF18), ADDR1(0xF1C), REG1(0xF1C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0xF20), REG1(0xF20), ADDR1(0xF24), REG1(0xF24),
		ADDR1(0xF28), REG1(0xF28), ADDR1(0xF2C), REG1(0xF2C));
		fmt_debug(0, "WROT%d(0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x (0x%lx) 0x%x",
		idx, ADDR1(0xF30), REG1(0xF30), ADDR1(0xF34), REG1(0xF34),
		ADDR1(0xF38), REG1(0xF38), ADDR1(0xF3C), REG1(0xF3C));
	}
}

static void fmt_gce_flush_callback(struct cmdq_cb_data data)
{
	struct cmdq_pkt *pkt_ptr;
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	pkt_ptr = (struct cmdq_pkt *)data.data;

	if (data.err < 0) {
		fmt_err("pkt_ptr %p aee_cb %p", pkt_ptr, pkt_ptr->aee_cb);
		atomic_set(&fmt->fmt_error, 1);
		fmt_dump_addr_reg();
	}
}

static int fmt_gce_timeout_aee(struct cmdq_cb_data data)
{
	struct cmdq_pkt *pkt_ptr;

	pkt_ptr = (struct cmdq_pkt *)data.data;

	if (pkt_ptr->err_data.wfe_timeout) {
		fmt_err("wfe %d timeout", pkt_ptr->err_data.event);
		return CMDQ_NO_AEE;
	} else {
		fmt_err("not wfe %d timeout", pkt_ptr->err_data.event);
		return CMDQ_AEE_WARN;
	}
}

static int fmt_native_finish_task(unsigned int taskid, struct file *owner, bool wait)
{
 struct mtk_vdec_fmt *fmt = fmt_mtkdev;
 struct gce_cmdq_task *task;
 unsigned int id, i;
 int ret = 0;
 if (taskid >= FMT_INST_MAX) return -EINVAL;
 task = &fmt->gce_task[taskid];
 if (!task->used || !task->pkt_ptr || task->owner != owner) return -EINVAL;
 id = task->identifier;
 if (id >= fmt->gce_th_num) return -EINVAL;
 if (wait) ret = cmdq_pkt_wait_complete(task->pkt_ptr);
 /* wait_complete synchronizes completion/error callback before destruction. */
 cmdq_pkt_destroy(task->pkt_ptr);
 fmt_clear_gce_task(taskid);
 task->owner = NULL;
 atomic_dec(&fmt->gce_job_cnt[id]);
 if (!atomic_read(&fmt->gce_job_cnt[0]) && !atomic_read(&fmt->gce_job_cnt[1])) {
  for (i = 0; i < fmt->gce_th_num; i++) fmt_end_dvfs_emi_bw(fmt, i);
  fmt_clock_off(fmt);
 }
 if (!atomic_read(&fmt->gce_job_cnt[id])) fmt_unlock(id);
 return ret;
}

static int fmt_native_submit(struct gce_cmdq_obj *buff, struct file *owner)
{
 struct mtk_vdec_fmt *fmt = fmt_mtkdev;
 struct cmdq_pkt *pkt = NULL;
 struct dmabuf_info in = {0}, out = {0};
 struct dmabufmap map[FMT_FD_RESERVE];
 struct gce_cmds *cmds;
 unsigned int id = buff->identifier, i;
 int ret, taskid;
 bool acquired = false, powered = false;
 if (id >= fmt->gce_th_num || buff->secure ||
     !buff->pmqos_param.pixel_size) return -EOPNOTSUPP;
 if (atomic_read(&fmt->fmt_error)) return -EIO;
 if (READ_ONCE(fmt->is_entering_suspend)) return -EBUSY;
 if (!fmt->clt_fmt[id]) return -ENODEV;
 if (!access_ok(buff->taskid, sizeof(u32))) return -EFAULT;
 cmds = fmt->gce_cmds[id];
 if (copy_from_user(cmds, u64_to_user_ptr(buff->cmds_user_ptr), sizeof(*cmds))) return -EFAULT;
 if (!cmds->cmd_cnt || cmds->cmd_cnt > FMT_CMDQ_CMD_MAX) return -EINVAL;
 for (i = 0; i < FMT_FD_RESERVE; i++) {map[i].fd = -1; map[i].iova = 0;}
 ret = fmt_lock(id, false);
 if (ret) return -EBUSY;
 acquired = !atomic_read(&fmt->gce_job_cnt[id]);
 pkt = cmdq_pkt_create(fmt->clt_fmt[id]);
 if (IS_ERR_OR_NULL(pkt)) {ret = pkt ? PTR_ERR(pkt) : -ENOMEM; pkt = NULL; goto fail;}
 for (i = 0; i < cmds->cmd_cnt; i++) {
  ret = fmt_set_gce_cmd(pkt, cmds->cmd[i], cmds->addr[i], cmds->data[i],
   cmds->mask[i], fmt->gce_gpr[id], id, &in, &out, map, false);
  if (ret) goto fail;
 }
 if (!atomic_read(&fmt->gce_job_cnt[0]) && !atomic_read(&fmt->gce_job_cnt[1])) {
  ret = fmt_clock_on(fmt);
  if (ret) goto fail;
  powered = true;
 }
 taskid = fmt_set_gce_task(pkt, id, in, out);
 if (taskid < 0) {ret = -ENOSPC; goto fail;}
 fmt->gce_task[taskid].owner = owner;
 memcpy(&fmt->gce_task[taskid].cmdq_buff, buff, sizeof(*buff));
 atomic_inc(&fmt->gce_job_cnt[id]);
 fmt_start_dvfs_emi_bw(fmt, buff->pmqos_param, id);
 pkt->aee_cb = fmt_gce_timeout_aee;
 ret = cmdq_pkt_flush_async(pkt, fmt_gce_flush_callback, pkt);
 if (ret) {fmt_native_finish_task(taskid, owner, false); return ret;}
 if (copy_to_user(buff->taskid, &taskid, sizeof(taskid))) {
  fmt_native_finish_task(taskid, owner, true); return -EFAULT;
 }
 return 0;
fail:
 if (powered) fmt_clock_off(fmt);
 if (pkt) cmdq_pkt_destroy(pkt);
 fmt_dmabuf_free_iova(in.dbuf, in.attach, in.sgt);
 fmt_dmabuf_free_iova(out.dbuf, out.attach, out.sgt);
 fmt_dmabuf_put(in.dbuf); fmt_dmabuf_put(out.dbuf);
 if (acquired) fmt_unlock(id);
 return ret;
}

static int fmt_gce_cmd_flush(unsigned long arg, struct file *owner)
{
 struct gce_cmdq_obj buff;
 if (copy_from_user(&buff, (void __user *)arg, sizeof(buff))) return -EFAULT;
 return fmt_native_submit(&buff, owner);
}

static int fmt_gce_wait_callback(unsigned long arg, struct file *owner)
{
 unsigned int id;
 if (copy_from_user(&id, (void __user *)arg, sizeof(id))) return -EFAULT;
 return fmt_native_finish_task(id, owner, true);
}

static int fmt_get_platform_dts(unsigned long arg)
{
	struct dts_info *dts;
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	dts = &fmt->dtsInfo;

	if (copy_to_user((void *)arg, dts, sizeof(*dts))) {
		fmt_err("Copy dts to user space failed\n");
		return -EFAULT;
	}

	return 0;
}

static int vdec_fmt_open(struct inode *inode, struct file *file)
{
	fmt_debug(1, "tid:%d", current->pid);

	return 0;
}

static int vdec_fmt_release(struct inode *inode, struct file *file)
{
 unsigned int i;
 (void)inode;
 mutex_lock(&fmt_native_ioctl_lock);
 if (fmt_mtkdev && fmt_mtkdev->native_ready)
  for (i = 0; i < FMT_INST_MAX; i++)
   if (fmt_mtkdev->gce_task[i].used && fmt_mtkdev->gce_task[i].owner == file)
    fmt_native_finish_task(i, file, true);
 mutex_unlock(&fmt_native_ioctl_lock);
 return 0;
}

static long vdec_fmt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -ENOTTY;
	mutex_lock(&fmt_native_ioctl_lock);
	if (!fmt_mtkdev || !fmt_mtkdev->native_ready) {
		mutex_unlock(&fmt_native_ioctl_lock);
		return -ENODEV;
	}

	fmt_debug(1, "pid:%d tid:%d", current->group_leader->pid, current->pid);

	switch (cmd) {
	case FMT_GCE_SET_CMD_FLUSH:
		ret = fmt_gce_cmd_flush(arg, file);
		break;
	case FMT_GCE_WAIT_CALLBACK:
		ret = fmt_gce_wait_callback(arg, file);
		break;
	case FMT_GET_PLATFORM_DTS:
		ret = fmt_get_platform_dts(arg);
		break;
	default:
		fmt_err("Unknown cmd");
		break;
	}

	mutex_unlock(&fmt_native_ioctl_lock);
	return ret;
}

#ifdef MTK_VDEC_FMT_DVT_VERIFICATION
void vdec_fmt_vma_open(struct vm_area_struct *vma)
{
	fmt_debug(1, "vdec fmt VMA open, virt %lx, phys %lx",
			vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void vdec_fmt_vma_close(struct vm_area_struct *vma)
{
	fmt_debug(1, "vdec fmt VMA close, virt %lx, phys %lx",
			vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static const struct vm_operations_struct
vdec_fmt_remap_vm_ops = {
	.open = vdec_fmt_vma_open,
	.close = vdec_fmt_vma_close,
};

static int vdec_fmt_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long length;
	unsigned long pfn;

	length = vma->vm_end - vma->vm_start;
	pfn = vma->vm_pgoff<<PAGE_SHIFT;

	if (fmt_check_reg_base(fmt_mtkdev, pfn, length) != 0) {
		fmt_err("illegal mmap pfn 0x%lx, len 0x%lx",
			pfn, length);
		return -EAGAIN;
	}
	fmt_debug(1, "start 0x%lx, end 0x%lx, pgoff 0x%lx",
			(unsigned long)vma->vm_start,
			(unsigned long)vma->vm_end,
			(unsigned long)vma->vm_pgoff);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	vma->vm_ops = &vdec_fmt_remap_vm_ops;
	vdec_fmt_vma_open(vma);

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_COMPAT)
static int compat_get_gce_cmdq_obj_data(
		unsigned long arg,
		struct gce_cmdq_obj *data)
{
	long ret = -1;
	struct compat_gce_cmdq_obj data32;

	ret = (long)copy_from_user(&data32, compat_ptr(arg),
		(unsigned long)sizeof(struct compat_gce_cmdq_obj));
	if (ret != 0L) {
		fmt_err("Copy data from user failed!\n");
		return -EINVAL;
	}

	data->cmds_user_ptr = data32.cmds_user_ptr;
	data->identifier = data32.identifier;
	data->secure = data32.secure;
	data->taskid = compat_ptr(data32.taskid);
	data->pmqos_param.tv_sec = data32.pmqos_param.tv_sec;
	data->pmqos_param.tv_usec = data32.pmqos_param.tv_usec;
	data->pmqos_param.pixel_size = data32.pmqos_param.pixel_size;
	data->pmqos_param.rdma_datasize = data32.pmqos_param.rdma_datasize;
	data->pmqos_param.wdma_datasize = data32.pmqos_param.wdma_datasize;

	return 0;

}
static long compat_vdec_fmt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int err = 0;

	fmt_debug(1, "pid:%d tid:%d", current->group_leader->pid, current->pid);
	switch (cmd) {
	case COMPAT_FMT_GCE_SET_CMD_FLUSH:
	{
		struct gce_cmdq_obj data;

		err = compat_get_gce_cmdq_obj_data(arg, &data);
		if (err != 0)
			return err;
		mutex_lock(&fmt_native_ioctl_lock);
		if (!fmt_mtkdev || !fmt_mtkdev->native_ready) err = -ENODEV;
		else err = fmt_native_submit(&data, file);
		mutex_unlock(&fmt_native_ioctl_lock);
		return err;
	}
	case FMT_GCE_WAIT_CALLBACK:
	case FMT_GET_PLATFORM_DTS:
		return file->f_op->unlocked_ioctl(file, cmd, arg);
	default:
		fmt_err("Unknown cmd");
		return -EFAULT;
	}
}
#endif

static const struct file_operations vdec_fmt_fops = {
	.owner = THIS_MODULE,
	.open = vdec_fmt_open,
	.release = vdec_fmt_release,
	.unlocked_ioctl = vdec_fmt_ioctl,
#ifdef MTK_VDEC_FMT_DVT_VERIFICATION
	.mmap = vdec_fmt_mmap,
#endif
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_vdec_fmt_ioctl,
#endif
};

static int vdec_fmt_suspend(struct device *pDev)
{
	struct mtk_vdec_fmt *fmt = fmt_mtkdev;

	if (!fmt || !fmt->native_ready) return 0;
	fmt_debug(1, "+");

	if (atomic_read(&fmt->gce_job_cnt[0]) > 0 ||
		atomic_read(&fmt->gce_job_cnt[1]) > 0) {
		fmt_debug(0, "waiting %d %d",
			atomic_read(&fmt->gce_job_cnt[0]),
			atomic_read(&fmt->gce_job_cnt[1]));
		return -EBUSY;
	}
	fmt_debug(1, "-");
	return 0;
}

static int vdec_fmt_resume(struct device *pDev)
{
	fmt_debug(1, "resume done");
	return 0;
}

static int vdec_fmt_suspend_notifier(struct notifier_block *nb,
				unsigned long action, void *data)
{
	int wait_cnt = 0;
	struct mtk_vdec_fmt *fmt = container_of(nb, struct mtk_vdec_fmt, pm_notifier);

	fmt_debug(1, "action %lu", action);
	switch (action) {
	case PM_SUSPEND_PREPARE:
		fmt->is_entering_suspend = 1;
		while (atomic_read(&fmt->gce_job_cnt[0]) > 0 ||
				atomic_read(&fmt->gce_job_cnt[1]) > 0) {
			wait_cnt++;
			if (wait_cnt > 5) {
				fmt_debug(0, "waiting %d %d",
				atomic_read(&fmt->gce_job_cnt[0]),
				atomic_read(&fmt->gce_job_cnt[1]));
				/* Current task is still not finished, don't
				 * care, will check again in real suspend
				 */
				return NOTIFY_DONE;
			}
			usleep_range(10000, 20000);
		}
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		fmt->is_entering_suspend = 0;
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

int fmt_translation_fault_callback(int port, u64 mva, void *data)
{
	fmt_debug(0, "TF callback, port:%d, mva:0x%llx %p", port, mva, data);
	fmt_dump_addr_reg();
	mtk_smi_dbg_hang_detect("debug");
	return 0;
}

void fmt_translation_fault_callback_setting(struct mtk_vdec_fmt *fmt)
{
	int i;

	for (i = 0; i < FMT_PORT_NUM; i++) {
		if (fmt->fmt_m4u_ports[i] != 0)
			mtk_iommu_register_fault_callback(fmt->fmt_m4u_ports[i],
			(mtk_iommu_fault_callback_t)fmt_translation_fault_callback,
			fmt, false);
	}
}

static void fmt_native_put_larb(void *p) {put_device(p);}

static int vdec_fmt_probe(struct platform_device *pdev)
{
 static const char *events[GCE_EVENT_MAX] = {
  "rdma0-sw-rst-done-eng", "rdma0-tile-done", "wdma0-sw-rst-done-eng", "wdma0-tile-done",
  "rdma1-sw-rst-done-eng", "rdma1-tile-done", "wdma1-sw-rst-done-eng", "wdma1-tile-done"
 };
 struct device *dev = &pdev->dev;
 struct mtk_vdec_fmt *fmt;
 struct device_node *node;
 struct platform_device *larb;
 struct resource *res;
 unsigned int i;
 u32 wa = 0;
 int ret;
 if (fmt_mtkdev) return -EBUSY;
 fmt = devm_kzalloc(dev, sizeof(*fmt), GFP_KERNEL);
 if (!fmt) return -ENOMEM;
 fmt->dev = dev;
 ret = of_property_read_string(dev->of_node, "mediatek,fmtname", &fmt->fmtname);
 if (ret) return ret;
 if (strcmp(fmt->fmtname, VDEC_FMT_DEVNAME)) return -EINVAL;
 ret = of_property_read_u32(dev->of_node, "mediatek,fmt-gce-th-num", &fmt->gce_th_num);
 if (ret || !fmt->gce_th_num || fmt->gce_th_num > FMT_CORE_NUM) return -EINVAL;
 fmt->dtsInfo.pipeNum = fmt->gce_th_num;
 ret = of_property_read_u32(dev->of_node, "mediatek,fmt-rdma-swwa", &wa);
 if (ret && of_find_property(dev->of_node, "mediatek,fmt-rdma-swwa", NULL)) return ret;
 fmt->dtsInfo.RDMA_needWA = !!wa;
 for (i = 0; i < FMT_MAP_HW_REG_NUM; i++) {
  res = platform_get_resource(pdev, IORESOURCE_MEM, i);
  if (!res) break;
  fmt->map_base[i].base = res->start;
  fmt->map_base[i].len = resource_size(res);
  fmt->map_base[i].va = (unsigned long)devm_ioremap(dev, res->start, resource_size(res));
  if (!fmt->map_base[i].va) return -ENOMEM;
 }
 if (i < fmt->gce_th_num * 2) return -EINVAL;
 fmt->dtsInfo.RDMA_baseAddr = fmt->map_base[0].base;
 fmt->dtsInfo.WROT_baseAddr = fmt->map_base[1].base;
 for (i = 0; i < GCE_EVENT_MAX; i++) {
  ret = of_property_read_u16(dev->of_node, events[i], &fmt->gce_codec_eid[i]);
  if (ret) return ret;
 }
 ret = of_property_read_u32(dev->of_node, "gce-gpr", &fmt->gce_gpr[0]);
 if (ret) return ret;
 for (i = 1; i < fmt->gce_th_num; i++) {
  ret = of_property_read_u32_index(dev->of_node, "gce-gpr", i, &fmt->gce_gpr[i]);
  if (ret) fmt->gce_gpr[i] = fmt->gce_gpr[0]; /* Native DT has one shared scratch register. */
 }
 for (i = 0; i < FMT_PORT_NUM; i++) {
  ret = of_property_read_u32_index(dev->of_node, "m4u-ports", i, &wa);
  if (ret) return ret;
  fmt->fmt_m4u_ports[i] = wa;
 }
 ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
 if (ret) return ret;
 node = of_parse_phandle(dev->of_node, "mediatek,larbs", 0);
 if (!node) return -EINVAL;
 larb = of_find_device_by_node(node); of_node_put(node);
 if (!larb) return -EPROBE_DEFER;
 ret = devm_add_action_or_reset(dev, fmt_native_put_larb, &larb->dev);
 if (ret) return ret;
 if (!device_is_bound(&larb->dev)) return -EPROBE_DEFER;
 fmt->fmtLarb = &larb->dev;
 fmt->clk_VDEC = devm_clk_get(dev, "MT_CG_VDEC");
 if (IS_ERR(fmt->clk_VDEC)) return PTR_ERR(fmt->clk_VDEC);
 fmt->clk_MINI_MDP = devm_clk_get(dev, "MT_CG_MINI_MDP");
 if (IS_ERR(fmt->clk_MINI_MDP)) return PTR_ERR(fmt->clk_MINI_MDP);
 ret = fmt_native_prepare_qos(fmt);
 if (ret) return ret;
 fmt->clt_base = cmdq_register_device(dev);
 if (!fmt->clt_base) {ret = -ENOMEM; goto qos;}
 for (i = 0; i < fmt->gce_th_num; i++) {
  fmt->gce_cmds[i] = devm_kzalloc(dev, sizeof(struct gce_cmds), GFP_KERNEL);
  fmt->mux_gce_th[i] = devm_kzalloc(dev, sizeof(struct mutex), GFP_KERNEL);
  fmt->mux_cmdq_pkt[i] = devm_kzalloc(dev, sizeof(struct mutex), GFP_KERNEL);
  if (!fmt->gce_cmds[i] || !fmt->mux_gce_th[i] || !fmt->mux_cmdq_pkt[i]) {ret = -ENOMEM; goto clients;}
  mutex_init(fmt->mux_gce_th[i]); mutex_init(fmt->mux_cmdq_pkt[i]);
  sema_init(&fmt->fmt_sem[i], 1);
  fmt->clt_fmt[i] = cmdq_mbox_create(dev, i);
  if (IS_ERR_OR_NULL(fmt->clt_fmt[i])) {
   ret = fmt->clt_fmt[i] ? PTR_ERR(fmt->clt_fmt[i]) : -ENODEV;
   fmt->clt_fmt[i] = NULL; goto clients;
  }
 }
 mutex_init(&fmt->mux_fmt); mutex_init(&fmt->mux_task); mutex_init(&fmt->mux_active_time);
 fmt->cmdq_cb_workqueue = alloc_ordered_workqueue(VDEC_FMT_DEVNAME, WQ_MEM_RECLAIM | WQ_FREEZABLE);
 if (!fmt->cmdq_cb_workqueue) {ret = -ENOMEM; goto clients;}
 INIT_WORK(&fmt->cmdq_cb_work, fmt_active_resource_time_check);
 fmt->native_larb_link = device_link_add(dev, fmt->fmtLarb, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
 if (!fmt->native_larb_link) {ret = -ENOMEM; goto work;}
 pm_runtime_enable(dev);
 platform_set_drvdata(pdev, fmt);
 fmt->pm_notifier.notifier_call = vdec_fmt_suspend_notifier;
 /* Callbacks are registered only after all their state exists. */
 WRITE_ONCE(fmt_mtkdev, fmt);
 ret = register_pm_notifier(&fmt->pm_notifier);
 if (ret) goto global;
 for (i = 0; i < FMT_PORT_NUM; i++) {
  ret = mtk_iommu_register_fault_callback(fmt->fmt_m4u_ports[i],
   (mtk_iommu_fault_callback_t)fmt_translation_fault_callback, fmt, false);
  if (ret) goto callbacks;
  fmt->native_fault_count++;
 }
 ret = alloc_chrdev_region(&fmt->fmt_devno, 0, 1, fmt->fmtname);
 if (ret) goto callbacks;
 fmt->fmt_cdev = cdev_alloc();
 if (!fmt->fmt_cdev) {ret = -ENOMEM; goto devno;}
 fmt->fmt_cdev->owner = THIS_MODULE; fmt->fmt_cdev->ops = &vdec_fmt_fops;
 ret = cdev_add(fmt->fmt_cdev, fmt->fmt_devno, 1);
 if (ret) {kobject_put(&fmt->fmt_cdev->kobj); goto devno;}
 fmt->fmt_class = class_create(fmt->fmtname);
 if (IS_ERR(fmt->fmt_class)) {ret = PTR_ERR(fmt->fmt_class); goto cdev;}
 fmt->fmt_device = device_create(fmt->fmt_class, NULL, fmt->fmt_devno, NULL, "%s", fmt->fmtname);
 if (IS_ERR(fmt->fmt_device)) {ret = PTR_ERR(fmt->fmt_device); goto class;}
 /* Qualified callback owner remains resident; runtime unbind is disabled. */
 __module_get(THIS_MODULE);
 WRITE_ONCE(fmt->native_ready, true);
 cmdq_get_usage_cb(fmt->clt_fmt[0]->chan, cmdq_usage_cb_func);
 fmt_debug(0, "initialization completed pipes=%u rdma_wa=%u", fmt->gce_th_num, fmt->dtsInfo.RDMA_needWA);
 return 0;
class: class_destroy(fmt->fmt_class);
cdev: cdev_del(fmt->fmt_cdev);
devno: unregister_chrdev_region(fmt->fmt_devno, 1);
callbacks:
 while (fmt->native_fault_count) mtk_iommu_unregister_fault_callback(fmt->fmt_m4u_ports[--fmt->native_fault_count], false);
 unregister_pm_notifier(&fmt->pm_notifier);
global:
 WRITE_ONCE(fmt_mtkdev, NULL); platform_set_drvdata(pdev, NULL); pm_runtime_disable(dev);
 device_link_del(fmt->native_larb_link);
work: destroy_workqueue(fmt->cmdq_cb_workqueue);
clients:
 for (i = 0; i < FMT_CORE_NUM; i++) if (fmt->clt_fmt[i]) cmdq_mbox_destroy(fmt->clt_fmt[i]);
qos: fmt_native_unprepare_qos(fmt);
 return ret;
}

static const struct of_device_id vdec_fmt_match[] = {
	{.compatible = "mediatek-vdec-fmt",},
	{}
};


static void vdec_fmt_remove(struct platform_device *pdev)
{
 /* Successful native callback owners are pinned; no runtime unload route. */
 WARN_ON(platform_get_drvdata(pdev));
}

static const struct dev_pm_ops vdec_fmt_pm_ops = {
	.suspend = vdec_fmt_suspend,
	.resume = vdec_fmt_resume,
};

static struct platform_driver vdec_fmt_driver = {
	.probe  = vdec_fmt_probe,
	.remove = vdec_fmt_remove,
	.driver = {
		.name   = VDEC_FMT_DEVNAME,
		.owner  = THIS_MODULE,
		.pm = &vdec_fmt_pm_ops,
		.of_match_table = vdec_fmt_match,
		.suppress_bind_attrs = true,
	},
};

static int __init fmt_init(void)
{
 int ret = fmt_sync_device_init();
 if (ret) return ret;
 ret = platform_driver_register(&vdec_fmt_driver);
 if (ret) fmt_sync_device_exit();
 return ret;
}
static void fmt_exit(void)
{
 platform_driver_unregister(&vdec_fmt_driver);
 fmt_sync_device_exit();
}

subsys_initcall(fmt_init);
module_exit(fmt_exit);


MODULE_LICENSE("GPL");
