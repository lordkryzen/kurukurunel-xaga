/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include "include/if_miwill.h"
#include "include/miwill_proto.h"
#include "include/miwill_debug.h"

#define TIME_OUT (HZ)
#define DUPLI_TABLE_LEN 4
#define MIWILL_SEQ_NUM 65536



struct red_priv {
	atomic_t sent_packets;
	unsigned long bitmap[DUPLI_TABLE_LEN][BITS_TO_LONGS(MIWILL_SEQ_NUM)];
	unsigned long timer_number;
	spinlock_t lock;
	struct timer_list timer;
};

static inline struct red_priv *get_red_priv(struct miwill *miw)
{
	return miw->mode_priv;
}

bool is_duplicate(struct miwill *miw, unsigned long sequence)
{
	struct red_priv *priv = get_red_priv(miw);
	unsigned int i;
	bool duplicate = false;

	spin_lock_bh(&priv->lock);
	for (i = 0; i < DUPLI_TABLE_LEN; i++)
		duplicate |= test_bit(sequence, priv->bitmap[i]);
	if (!duplicate)
		set_bit(sequence, priv->bitmap[(-priv->timer_number) & 3]);
	spin_unlock_bh(&priv->lock);
	return duplicate;
}

static bool red_transmit(struct miwill *miw, struct sk_buff *skb)
{
	struct miwill_port *port;
	struct sk_buff *skb2;
	struct MIWILL_ethhdr *miw_ethhdr;
	struct red_priv* data = get_red_priv(miw);
	int ret = 0;

#ifdef DEBUG_PACKET
	miwill_cmn_dbg("%s", __func__);
	skb_dump(KERN_ERR,skb,true);
#endif
	skb = miwill_add_head(skb, miw);
	if(unlikely(!skb)){
		miwill_cmn_err("%s, miwill_add_head", __func__);
		return false;
	}

	miw_ethhdr = (struct MIWILL_ethhdr *)(skb->data);
	miw_ethhdr->header.mode = REDUD_MODE;
	miw_ethhdr->header.option.redud.sub_type = 0x00;
	miw_ethhdr->header.option.redud.seq_num = htons((u16)atomic_inc_return(&data->sent_packets));

	spin_lock_bh(&miw->lock);
	list_for_each_entry(port, &miw->port_list, list) {
		if (unlikely(!port))
			goto drop;

		if(!miwill_port_txable(port))
			continue;

		skb2 = skb_copy(skb, GFP_ATOMIC);
		if (unlikely(!skb2))
			continue;

		ret = miwill_dev_queue_xmit(miw, port, skb2);
		if (ret) {
			miwill_cmn_dbg("%s, miwill_dev_queue_xmit %s, ret:%d",
				__func__, port->dev ? port->dev->name : "port->dev is NULL", ret);
			continue;
		}
	}
	spin_unlock_bh(&miw->lock);
#ifdef DEBUG_PACKET
	miwill_cmn_dbg("%s sent_packets:%ld", __func__, atomic_read(&data->sent_packets));
#endif
	
	dev_kfree_skb(skb);
	return true;
drop:
	spin_unlock_bh(&miw->lock);
	dev_kfree_skb_any(skb);
	return false;
}

rx_handler_result_t red_receive(struct miwill *miw,struct miwill_port *port,struct sk_buff *skb)
{
	struct MIWILL_ethhdr *miw_ethhdr;
	unsigned long cur_packets;

#ifdef DEBUG_PACKET
	miwill_cmn_dbg("%s before", __func__);
	skb_dump(KERN_ERR,skb,true);
#endif
	miw_ethhdr = (struct MIWILL_ethhdr *)(skb->data-ETH_HLEN);
	cur_packets = ntohs(miw_ethhdr->header.option.redud.seq_num);
	if(is_duplicate(miw, cur_packets%MIWILL_SEQ_NUM)) {
		dev_kfree_skb(skb);
		return RX_HANDLER_CONSUMED;
	}
#ifdef DEBUG_PACKET
	miwill_cmn_dbg("%s dmac "MAC_FMT" ", __func__, MAC_ARG(miw_ethhdr->h_dest));
	miwill_cmn_dbg("%s smac "MAC_FMT" ", __func__, MAC_ARG(miw_ethhdr->h_source));
	miwill_cmn_dbg("%s seq %04x", __func__, ntohs(miw_ethhdr->header.option.redud.seq_num));
#endif
	skb->protocol = miw_ethhdr->header.encap_proto;
	memmove(skb->data - ETH_ALEN, miw_ethhdr, 2 * ETH_ALEN);
	skb_pull_rcsum(skb, MIWILL_HEAD_LEN);
	skb->mac_header += MIWILL_HEAD_LEN;
	skb_reset_network_header(skb);
	skb_reset_transport_header(skb);
	skb_reset_mac_len(skb);
#ifdef DEBUG_PACKET
	miwill_cmn_dbg("%s after", __func__);
	skb_dump(KERN_ERR,skb,true);
#endif
	return RX_HANDLER_ANOTHER;
}

static void timer_callback(struct timer_list *timer)
{
	struct red_priv *priv = from_timer(priv, timer, timer);

	spin_lock_bh(&priv->lock);
	bitmap_zero(priv->bitmap[(~priv->timer_number) & 3], MIWILL_SEQ_NUM);
	priv->timer_number++;
	spin_unlock_bh(&priv->lock);
	mod_timer(timer, jiffies + TIME_OUT / DUPLI_TABLE_LEN);
}

static void red_exit(struct miwill *miw)
{
	struct red_priv *priv = get_red_priv(miw);

	timer_shutdown_sync(&priv->timer);
	kvfree(priv);
	miw->mode_priv = NULL;
}

static int red_init(struct miwill *miw);
static const struct miwill_mode_ops red_mode_ops = {
	.init			= red_init,
	.exit			= red_exit,
	.receive		= red_receive,
	.transmit		= red_transmit,
	.port_enter = miwill_modeop_port_enter,
};

static const struct miwill_mode red_mode = {
	.kind = "redudancy",
	.owner = THIS_MODULE,
	.ops = &red_mode_ops,

};

static int red_init(struct miwill *miw)
{
	struct red_priv *priv = kvzalloc(sizeof(*priv), GFP_KERNEL);

	if (!priv)
		return -ENOMEM;
	spin_lock_init(&priv->lock);
	atomic_set(&priv->sent_packets, 0);
	miw->mode_priv = priv;
	timer_setup(&priv->timer, timer_callback, 0);
	mod_timer(&priv->timer, jiffies + TIME_OUT / DUPLI_TABLE_LEN);
	return 0;
}

static int __init miwill_red_init_module(void)
{
	return miwill_mode_register(&red_mode);
}

static void __exit miwill_red_cleanup_module(void)
{
	miwill_mode_unregister(&red_mode);
}

module_init(miwill_red_init_module);
module_exit(miwill_red_cleanup_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Minjun Xi <ximinjun@xiaomi.com> ");
MODULE_DESCRIPTION("redudancy mode support for MiWiLL protocol");
