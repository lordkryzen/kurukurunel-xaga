// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Yu-Chang Wang <Yu-Chang.Wang@mediatek.com>
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "clk-fhctl.h"
#include "clk-fhctl-pll.h"
#include "clk-fhctl-util.h"
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>
#include <include/gpueb_ipi.h>

#define FHCTL_TARGET FHCTL_GPUEB
#define FHCTL_IPI_TIMEOUT_MS 10
#define IPI_TIMEOUT_LONG_MS 3000

struct match {
	char *name;
	struct fh_hdlr *hdlr;
	int (*init)(struct pll_dts *array
			, struct match *match);
};
struct id_map {
	char *name;
	int base;
};
struct hdlr_data_v1 {
	struct pll_dts *array;
	spinlock_t *lock;
	struct fh_pll_domain *domain;
	u8 tr_id;
	void __iomem *reg_tr;
	struct id_map *map;
};
struct freqhopping_ioctl {
	unsigned int pll_id;
	struct freqhopping_ssc {
		unsigned int idx_pattern; /* idx_pattern: Deprecated Field */
		unsigned int dt;
		unsigned int df;
		unsigned int upbnd;
		unsigned int lowbnd;
		unsigned int dds; /* dds: Deprecated Field */
	} ssc_setting;    /* used only when user-define */
	int result;
};
struct fhctl_ipi_data {
	unsigned int cmd;
	union {
		struct freqhopping_ioctl fh_ctl;
		unsigned int args[8];
	} u;
};
enum FH_DEVCTL_CMD_ID {
	FH_DCTL_CMD_SSC_ENABLE = 0x1004,
	FH_DCTL_CMD_SSC_DISABLE = 0x1005,
	FH_DCTL_CMD_GENERAL_DFS = 0x1006,
	FH_DCTL_CMD_ARM_DFS = 0x1007,
	FH_DCTL_CMD_SSC_TBL_CONFIG = 0x100A,
	FH_DCTL_CMD_PLL_PAUSE = 0x100E,
	FH_DCTL_CMD_MAX,
	FH_DBG_CMD_TR_BEGIN64_LOW = 0x2001,
	FH_DBG_CMD_TR_BEGIN64_HIGH = 0x2002,
	FH_DBG_CMD_TR_END64_LOW = 0x2003,
	FH_DBG_CMD_TR_END64_HIGH = 0x2004,
	FH_DBG_CMD_TR_ID = 0x2005,
	FH_DBG_CMD_TR_VAL = 0x2006,
	FH_DBG_CMD_TR_BEGIN32 = 0x2007,
	FH_DBG_CMD_TR_END32 = 0x2008,
};
#define FHCTL_D_LEN (sizeof(struct fhctl_ipi_data)/\
	sizeof(unsigned int))
static unsigned int ack_data;
static DEFINE_SPINLOCK(fh_backend_lock);
static int channel_id;

static int id_to_gpueb(struct id_map *map, char *domain, int fh_id)
{
	if (!map)
		return fh_id;

	while (map->name != NULL) {
		if (strcmp(map->name,
					domain) == 0) {
			return (map->base + fh_id);
		}
		map++;
	}
	return fh_id;
}
/* Native debug command reads the firmware transaction counter only.
 * No DFS, SSC or register-control command is accepted here. */
int fhctl_gpueb_query(u32 *reply)
{
 struct fhctl_ipi_data request = {};
 unsigned long flags;
 int ret;
 if (!reply || !get_gpueb_ipidev())
  return -EINVAL;
 request.cmd = FH_DBG_CMD_TR_ID;
 spin_lock_irqsave(&fh_backend_lock, flags);
 ret = mtk_ipi_send_compl(get_gpueb_ipidev(), channel_id, IPI_SEND_POLLING,
                          &request, FHCTL_D_LEN, 100);
 if (!ret)
  *reply = READ_ONCE(ack_data);
 spin_unlock_irqrestore(&fh_backend_lock, flags);
 return ret;
}

static int gpueb_hopping_v1(void *priv_data, char *domain_name, int fh_id,
                             unsigned int new_dds, int postdiv)
{
 struct hdlr_data_v1 *d = priv_data;
 struct fhctl_ipi_data request = {};
 struct fh_pll_regs *regs;
 struct fh_pll_data *data;
 unsigned long flags;
 u32 readback, con_pcw, divider_before;
 int ret;

 if (!d || !d->array || !d->domain || !d->lock || !domain_name ||
     strcmp(domain_name, d->array->domain) || fh_id != d->array->fh_id)
  return -EINVAL;
 if (!fhctl_clock_ops_allowed())
  return -EACCES;
 data = &d->domain->data[fh_id];
 regs = &d->domain->regs[fh_id];
 if (!new_dds || (new_dds & ~data->dds_mask) || !regs->reg_con_pcw ||
     (postdiv != 9999 && (postdiv <= 0 || postdiv > 128 || (postdiv & (postdiv - 1)))))
  return -ERANGE;
 spin_lock_irqsave(d->lock, flags);
 if (!fhctl_clock_ops_allowed()) {
  ret = -EIO;
  goto unlock;
 }
 divider_before = readl(regs->reg_con_pcw) & (7U << 24);
 /* Native xaga firmware was exercised with keep-divider9999. A CCF
  * divider change is unsupported here BEFORE sending; its existing
  * native register path remains available via -ENOENT. */
 if (postdiv != 9999 && (1U << (divider_before >> 24)) != (u32)postdiv) {
  ret = -ENOENT;
  goto unlock;
 }
 request.cmd = FH_DCTL_CMD_GENERAL_DFS;
 request.u.args[0] = id_to_gpueb(d->map, domain_name, fh_id);
 request.u.args[1] = new_dds;
 request.u.args[2] = 9999;
 /* Nonzero changing transaction byte; native firmware echoes it. */
 d->tr_id++;
 if (!d->tr_id)
  d->tr_id++;
 request.u.args[7] = d->tr_id;
 WRITE_ONCE(ack_data, 0);
 ret = mtk_ipi_send_compl(get_gpueb_ipidev(), channel_id, IPI_SEND_POLLING,
                          &request, FHCTL_D_LEN, FHCTL_IPI_TIMEOUT_MS);
 if (ret) {
  ret = ret < 0 ? ret : -EIO;
  goto fault;
 }
 con_pcw = readl(regs->reg_con_pcw);
 readback = con_pcw & data->dds_mask;
 if (READ_ONCE(ack_data) != d->tr_id || readback != new_dds ||
     ((con_pcw & (7U << 24)) != divider_before)) {
  ret = -EIO;
  goto fault;
 }
 ret = 0;
 goto unlock;
fault:
 /* No secondary debug sends/AEE or direct register fallback after an
  * uncertain transaction. Keep state pinned and reject further control. */
 fhctl_record_fault(ret);
 pr_err_ratelimited("FHCTL gpueb transaction fault ret=%d pll=%s ack=%u expected=%u\n",
                    ret, d->array->pll_name, READ_ONCE(ack_data), d->tr_id);
unlock:
 spin_unlock_irqrestore(d->lock, flags);
 return ret;
}

static int gpueb_reapply_current(void *priv_data)
{
 struct hdlr_data_v1 *d = priv_data;
 u32 current_dds;
 if (!d || !d->array || !d->domain)
  return -EINVAL;
 current_dds = readl(d->domain->regs[d->array->fh_id].reg_con_pcw) &
           d->domain->data[d->array->fh_id].dds_mask;
 return gpueb_hopping_v1(d, d->array->domain, d->array->fh_id, current_dds, 9999);
}

static int gpueb_ssc_enable_v1(void *priv_data,
		char *domain_name, int fh_id, int rate)
{
	struct freqhopping_ioctl fh_ctl = {};
	struct fhctl_ipi_data ipi_data;
	int ret;
	struct hdlr_data_v1 *d = (struct hdlr_data_v1 *)priv_data;
	spinlock_t *lock = d->lock;
	struct pll_dts *array = d->array;
	struct fh_pll_data *data = d->domain->data;
	unsigned long flags = 0;

	if (true) /* SSC awaits separate tuning/ack qualification. */
		return -EACCES;

	spin_lock_irqsave(lock, flags);

	FHDBG("rate<%d>\n", rate);

	fh_ctl.pll_id = id_to_gpueb(d->map, domain_name, fh_id);
	fh_ctl.result = 0;
	fh_ctl.ssc_setting.dt = data->dt_val;
	fh_ctl.ssc_setting.df = data->df_val;
	fh_ctl.ssc_setting.upbnd = 0;
	fh_ctl.ssc_setting.lowbnd = rate;

	memset(&ipi_data, 0, sizeof(struct fhctl_ipi_data));
	memcpy(&ipi_data.u.fh_ctl, &fh_ctl,
					sizeof(struct freqhopping_ioctl));

	ipi_data.cmd = FH_DCTL_CMD_SSC_ENABLE;
	ret = mtk_ipi_send_compl(get_gpueb_ipidev(), channel_id,
			IPI_SEND_POLLING, &ipi_data,
			FHCTL_D_LEN, IPI_TIMEOUT_LONG_MS);
	FHDBG("ret<%d>\n", ret);

	array->ssc_rate = rate;

	spin_unlock_irqrestore(lock, flags);
	return 0;
}

static int gpueb_ssc_disable_v1(void *priv_data,
		char *domain_name, int fh_id)
{
	struct freqhopping_ioctl fh_ctl = {};
	struct fhctl_ipi_data ipi_data;
	int ret;
	struct hdlr_data_v1 *d = (struct hdlr_data_v1 *)priv_data;
	spinlock_t *lock = d->lock;
	struct pll_dts *array = d->array;
	unsigned long flags = 0;

	if (true) /* SSC awaits separate tuning/ack qualification. */
		return -EACCES;

	spin_lock_irqsave(lock, flags);

	FHDBG("\n");

	fh_ctl.pll_id = id_to_gpueb(d->map, domain_name, fh_id);
	fh_ctl.result = 0;

	memset(&ipi_data, 0, sizeof(struct fhctl_ipi_data));
	memcpy(&ipi_data.u.fh_ctl, &fh_ctl,
					sizeof(struct freqhopping_ioctl));

	ipi_data.cmd = FH_DCTL_CMD_SSC_DISABLE;
	ret = mtk_ipi_send_compl(get_gpueb_ipidev(), channel_id,
			IPI_SEND_POLLING, &ipi_data,
			FHCTL_D_LEN, IPI_TIMEOUT_LONG_MS);
	FHDBG("ret<%d>\n", ret);

	array->ssc_rate = 0;

	spin_unlock_irqrestore(lock, flags);
	return 0;
}

static int gpueb_init_v1(struct pll_dts *array, struct match *match)
{
 static bool ipi_inited;
 
 struct hdlr_data_v1 *priv_data, *match_data;
 struct fh_hdlr *hdlr;
 struct fh_pll_domain *domain;
 int ret;

 if (!get_gpueb_ipidev())
  return -EPROBE_DEFER;
 if (!ipi_inited) {
  channel_id = gpueb_get_send_PIN_ID_by_name("IPI_ID_FHCTL");
  if (channel_id < 0)
   return -EPROBE_DEFER;
 }
 priv_data = kzalloc(sizeof(*priv_data), GFP_KERNEL);
 if (!priv_data)
  return -ENOMEM;
 hdlr = kzalloc(sizeof(*hdlr), GFP_KERNEL);
 if (!hdlr) {
  kfree(priv_data);
  return -ENOMEM;
 }
 /* Domain tables are module-global; never permit unmap/unload after they
  * acquire addresses or after IPI may publish a receiver pointer. */
 fhctl_pin_transport();
 init_fh_domain(array->domain, array->comp, array->fhctl_base, array->apmixed_base);
 domain = get_fh_domain(array->domain);
 if (!domain) {
  ret = -EINVAL;
  goto free_private;
 }
 if (!ipi_inited) {
  ret = mtk_ipi_register(get_gpueb_ipidev(), channel_id, NULL, NULL, &ack_data);
  if (ret)
   goto free_private;
  ipi_inited = true;
 }
 priv_data->array = array;
 priv_data->lock = &fh_backend_lock;
 priv_data->domain = domain;
 match_data = match->hdlr->data;
 if (match_data->reg_tr)
  priv_data->reg_tr = array->fhctl_base + (unsigned long)match_data->reg_tr;
 priv_data->map = match_data->map;
 hdlr->data = priv_data;
 hdlr->ops = match->hdlr->ops;
 array->hdlr = hdlr;
 if (fhctl_clock_ops_allowed() && array->ssc_rate)
  return hdlr->ops->ssc_enable(hdlr->data, array->domain, array->fh_id, array->ssc_rate);
 return 0;
free_private:
 /* The registered receiver points only to static ack_data, never these
  * private objects. Code/static receiver stay pinned on partial failure. */
 kfree(hdlr);
 kfree(priv_data);
 return ret;
}

static struct fh_operation gpueb_ops_v1 = {
 .reapply_current = gpueb_reapply_current,
	.hopping = gpueb_hopping_v1,
	.ssc_enable = gpueb_ssc_enable_v1,
	.ssc_disable = gpueb_ssc_disable_v1,
};

/*mt6983 begin*/
static struct id_map map_6983[] = {
	{"gpu0", 1000},
	{"gpu2", 3000},
	{"gpu3", 4000},
	{}
};

struct hdlr_data_v1 hdlr_data_6983_gpueb = {
	.reg_tr = NULL,
	.map = map_6983,
};
static struct fh_hdlr gpueb_hdlr_6983 = {
	.ops = &gpueb_ops_v1,
	.data = &hdlr_data_6983_gpueb,
};
static struct match mt6983_match = {
	.name = "mediatek,mt6983-fhctl",
	.hdlr = &gpueb_hdlr_6983,
	.init = &gpueb_init_v1,
};
/*mt6983 end*/

/*mt6895 begin*/
static struct match mt6895_match = {
	.name = "mediatek,mt6895-fhctl",
	.hdlr = &gpueb_hdlr_6983,
	.init = &gpueb_init_v1,
};
/*mt6895 end*/

/*mt6879 begin*/
struct hdlr_data_v1 hdlr_data_6879_gpueb = {
	.reg_tr = NULL,
	.map = NULL,
};
static struct fh_hdlr gpueb_hdlr_6879 = {
	.ops = &gpueb_ops_v1,
	.data = &hdlr_data_6879_gpueb,
};
static struct match mt6879_match = {
	.name = "mediatek,mt6879-fhctl",
	.hdlr = &gpueb_hdlr_6879,
	.init = &gpueb_init_v1,
};
/*mt6879 end*/

/*mt6855 begin*/
struct hdlr_data_v1 hdlr_data_6855_gpueb = {
	.reg_tr = NULL,
	.map = NULL,
};
static struct fh_hdlr gpueb_hdlr_6855 = {
	.ops = &gpueb_ops_v1,
	.data = &hdlr_data_6855_gpueb,
};
static struct match mt6855_match = {
	.name = "mediatek,mt6855-fhctl",
	.hdlr = &gpueb_hdlr_6855,
	.init = &gpueb_init_v1,
};
/*mt6855 end*/

static struct match *matches[] = {
	&mt6855_match,
	&mt6879_match,
	&mt6895_match,
	&mt6983_match,
	NULL,
};

int fhctl_gpueb_init(struct pll_dts *array)
{
	int i;
	int num_pll;
	struct match **match;

	FHDBG("\n");
	match = matches;
	num_pll = array->num_pll;

	/* find match by compatible */
	while (*match != NULL) {
		char *comp = (*match)->name;
		char *target = array->comp;

		if (strcmp(comp,
					target) == 0) {
			break;
		}
		match++;
	}

	if (*match == NULL) {
		FHDBG("no match!\n");
		return -1;
	}

	/* init flow for every pll */
	for (i = 0; i < num_pll ; i++, array++) {
		char *method = array->method;

		if (strcmp(method,
					FHCTL_TARGET) == 0) {
			int ret = (*match)->init(array, *match);
			if (ret)
				return ret;
		}
	}

	FHDBG("\n");
	return 0;
}
