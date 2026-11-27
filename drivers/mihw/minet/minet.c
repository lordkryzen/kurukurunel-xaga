// SPDX-License-Identifier: GPL-2.0-only
#include <linux/hashtable.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/module.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>
#include <net/addrconf.h>
#include <net/genetlink.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/netfilter/nf_socket.h>
#include <net/sock.h>
#include "minet_wire.h"

#define MINET_UIDS 64
#define MINET_FLOWS 256
static char *family_name = "minet_genl";
module_param(family_name, charp, 0444);
static bool include_local;
module_param(include_local, bool, 0444);

struct tracked_flow {
	struct hlist_node hash;
	struct list_head list;
	u32 uid;
	struct minet_flow wire;
};
static DEFINE_HASHTABLE(flows, 8);
static LIST_HEAD(flow_list);
static DEFINE_SPINLOCK(flow_lock);
static u32 watched_uids[MINET_UIDS];
static unsigned int flow_count;
static unsigned int active_apps;
static struct genl_family minet_family;
static struct workqueue_struct *report_queue;
static struct delayed_work report_work;
static struct proc_dir_entry *stat_proc;
static atomic64_t packets = ATOMIC64_INIT(0);
static atomic64_t overflow = ATOMIC64_INIT(0);
static atomic64_t reports = ATOMIC64_INIT(0);
static atomic64_t report_errors = ATOMIC64_INIT(0);

static bool uid_watched(u32 uid)
{
	int i;

	for (i = 0; i < MINET_UIDS; i++)
		if (watched_uids[i] == uid)
			return true;
	return false;
}

static void remove_flows(u32 uid)
{
	struct tracked_flow *flow, *next;

	list_for_each_entry_safe(flow, next, &flow_list, list) {
		if (uid && flow->uid != uid)
			continue;
		hash_del(&flow->hash);
		list_del(&flow->list);
		flow_count--;
		kfree(flow);
	}
}

static int configure_apps(struct sk_buff *skb, struct genl_info *info)
{
	const struct minet_command *command;
	u32 new_uids[MINET_UIDS];
	unsigned int i, j, length;
	int ret = 0;

	if (!info->attrs[1])
		return -EINVAL;
	length = nla_len(info->attrs[1]);
	command = nla_data(info->attrs[1]);
	if (length < sizeof(*command) || command->version != 1 ||
	    command->operation < 1 || command->operation > 3 ||
	    command->count > MINET_UIDS ||
	    length != sizeof(*command) + command->count * sizeof(u32))
		return -EINVAL;
	for (i = 0; i < command->count; i++)
		if (!command->uids[i] || !uid_valid(make_kuid(&init_user_ns, command->uids[i])))
			return -EINVAL;
	spin_lock_bh(&flow_lock);
	memcpy(new_uids, watched_uids, sizeof(new_uids));
	if (command->operation == 3) {
		memset(watched_uids, 0, sizeof(watched_uids));
		remove_flows(0);
		active_apps = 0;
		goto out;
	}
	for (i = 0; i < command->count; i++) {
		int available = -1;

		for (j = 0; j < MINET_UIDS; j++) {
			if (new_uids[j] == command->uids[i])
				break;
			if (!new_uids[j] && available < 0)
				available = j;
		}
		if (command->operation == 2) {
			if (j < MINET_UIDS)
				new_uids[j] = 0;
		} else if (j == MINET_UIDS) {
			if (available < 0) {
				ret = -ENOSPC;
				goto out;
			}
			new_uids[available] = command->uids[i];
		}
	}
	memcpy(watched_uids, new_uids, sizeof(new_uids));
	active_apps = 0;
	for (i = 0; i < MINET_UIDS; i++)
		active_apps += !!watched_uids[i];
	if (command->operation == 2)
		for (i = 0; i < command->count; i++)
			remove_flows(command->uids[i]);
out:
	spin_unlock_bh(&flow_lock);
	return ret;
}

static const struct nla_policy minet_policy[2] = {
	[1] = { .type = NLA_BINARY, .len = sizeof(struct minet_command) + MINET_UIDS * 4 },
};
static const struct genl_ops minet_ops[] = {
	{ .cmd = 1, .flags = GENL_ADMIN_PERM, .policy = minet_policy,
	  .maxattr = 1, .doit = configure_apps },
};
static const struct genl_multicast_group minet_groups[] = {
	{ .name = "minet_event", .flags = GENL_MCAST_CAP_NET_ADMIN },
};
static struct genl_family minet_family = {
	.name = "minet_genl", .version = 1, .maxattr = 1,
	.module = THIS_MODULE, .ops = minet_ops, .n_ops = ARRAY_SIZE(minet_ops),
	.mcgrps = minet_groups, .n_mcgrps = ARRAY_SIZE(minet_groups),
};

static void publish_flows(struct work_struct *work)
{
	LIST_HEAD(snapshot);
	struct tracked_flow *flow, *next;
	struct minet_report *report;
	struct sk_buff *skb;
	void *header;
	int ret;

	report = kzalloc(struct_size(report, flows, MINET_FLOWS), GFP_KERNEL);
	if (!report)
		goto requeue;
	spin_lock_bh(&flow_lock);
	list_splice_init(&flow_list, &snapshot);
	list_for_each_entry(flow, &snapshot, list)
		hash_del(&flow->hash);
	flow_count = 0;
	spin_unlock_bh(&flow_lock);
	while (!list_empty(&snapshot)) {
		flow = list_first_entry(&snapshot, struct tracked_flow, list);
		memset(report, 0, sizeof(*report));
		report->version = 1;
		report->uid = flow->uid;
		list_for_each_entry_safe(flow, next, &snapshot, list) {
			if (flow->uid != report->uid)
				continue;
			report->flows[report->count++] = flow->wire;
			list_del(&flow->list);
			kfree(flow);
		}
		skb = genlmsg_new(nla_total_size(struct_size(report, flows, report->count)), GFP_KERNEL);
		if (!skb) {
			atomic64_inc(&report_errors);
			continue;
		}
		header = genlmsg_put(skb, 0, 0, &minet_family, 0, 2);
		if (!header || nla_put(skb, 1, struct_size(report, flows, report->count), report)) {
			kfree_skb(skb);
			atomic64_inc(&report_errors);
			continue;
		}
		genlmsg_end(skb, header);
		ret = genlmsg_multicast(&minet_family, skb, 0, 0, GFP_KERNEL);
		if (ret < 0 && ret != -ESRCH)
			atomic64_inc(&report_errors);
		else
			atomic64_inc(&reports);
	}
	kfree(report);
requeue:
	queue_delayed_work(report_queue, &report_work, 5 * HZ);
}

static bool packet_tuple(struct sk_buff *skb, u8 family, bool incoming,
			 struct minet_flow *flow)
{
	struct { __be16 source, dest; } port_buffer, *ports;
	unsigned int offset;
	int protocol, network_offset = skb_network_offset(skb);

	if (network_offset < 0)
		return false;
	if (family == NFPROTO_IPV4) {
		struct iphdr ip_buffer, *ip;
		__be32 remote;

		ip = skb_header_pointer(skb, network_offset, sizeof(ip_buffer), &ip_buffer);
		if (!ip || ip->version != 4 || ip->ihl < 5 || ip_is_fragment(ip))
			return false;
		protocol = ip->protocol;
		offset = network_offset + ip->ihl * 4;
		remote = incoming ? ip->saddr : ip->daddr;
		if (!include_local && (ipv4_is_loopback(remote) || ipv4_is_multicast(remote) ||
		    ipv4_is_lbcast(remote) || ipv4_is_zeronet(remote) ||
		    ipv4_is_private_10(remote) || ipv4_is_private_172(remote) ||
		    ipv4_is_private_192(remote) || ipv4_is_linklocal_169(remote)))
			return false;
		flow->family = AF_INET;
		memcpy(flow->local_address, incoming ? &ip->daddr : &ip->saddr, 4);
		memcpy(flow->remote_address, &remote, 4);
	} else {
		struct ipv6hdr ip_buffer, *ip;
		const struct in6_addr *remote;
		unsigned short fragment_offset = 0;

		ip = skb_header_pointer(skb, network_offset, sizeof(ip_buffer), &ip_buffer);
		if (!ip || ip->version != 6)
			return false;
		offset = 0;
		protocol = ipv6_find_hdr(skb, &offset, -1, &fragment_offset, NULL);
		if (protocol < 0 || fragment_offset)
			return false;
		remote = incoming ? &ip->saddr : &ip->daddr;
		if (!include_local && (ipv6_addr_type(remote) &
		    (IPV6_ADDR_MULTICAST | IPV6_ADDR_LOOPBACK | IPV6_ADDR_LINKLOCAL)))
			return false;
		flow->family = AF_INET6;
		memcpy(flow->local_address, incoming ? &ip->daddr : &ip->saddr, 16);
		memcpy(flow->remote_address, remote, 16);
	}
	if (protocol != IPPROTO_TCP && protocol != IPPROTO_UDP)
		return false;
	ports = skb_header_pointer(skb, offset, sizeof(port_buffer), &port_buffer);
	if (!ports)
		return false;
	flow->protocol = protocol;
	flow->local_port = ntohs(incoming ? ports->dest : ports->source);
	flow->remote_port = ntohs(incoming ? ports->source : ports->dest);
	return true;
}

static unsigned int observe_packet(void *unused, struct sk_buff *skb,
				   const struct nf_hook_state *state)
{
	struct minet_flow wire = {};
	struct tracked_flow *flow;
	struct sock *sk;
	u32 uid, key;
	bool incoming = state->hook == NF_INET_LOCAL_IN;

	if (!READ_ONCE(active_apps))
		return NF_ACCEPT;
	if (incoming) {
		sk = state->pf == NFPROTO_IPV4 ?
			nf_sk_lookup_slow_v4(state->net, skb, state->in) :
			nf_sk_lookup_slow_v6(state->net, skb, state->in);
	} else {
		sk = skb_to_full_sk(skb);
		if (sk)
			sock_hold(sk);
	}
	if (!sk)
		return NF_ACCEPT;
	if (!sk_fullsock(sk)) {
		sock_gen_put(sk);
		return NF_ACCEPT;
	}
	uid = __kuid_val(sock_i_uid(sk));
	sock_gen_put(sk);
	if (!uid)
		return NF_ACCEPT;
	spin_lock_bh(&flow_lock);
	if (!uid_watched(uid)) {
		spin_unlock_bh(&flow_lock);
		return NF_ACCEPT;
	}
	spin_unlock_bh(&flow_lock);
	if (!packet_tuple(skb, state->pf, incoming, &wire))
		return NF_ACCEPT;
	key = uid ^ wire.protocol ^ wire.local_port;
	spin_lock_bh(&flow_lock);
	if (!uid_watched(uid))
		goto out;
	hash_for_each_possible(flows, flow, hash, key)
		if (flow->uid == uid && !memcmp(&flow->wire, &wire, offsetof(struct minet_flow, rx_packets)))
			goto count;
	if (flow_count == MINET_FLOWS) {
		atomic64_inc(&overflow);
		goto out;
	}
	flow = kzalloc(sizeof(*flow), GFP_ATOMIC);
	if (!flow) {
		atomic64_inc(&overflow);
		goto out;
	}
	flow->uid = uid;
	flow->wire = wire;
	hash_add(flows, &flow->hash, key);
	list_add_tail(&flow->list, &flow_list);
	flow_count++;
count:
	if (incoming) {
		flow->wire.rx_packets++;
		flow->wire.rx_bytes += skb->len;
	} else {
		flow->wire.tx_packets++;
		flow->wire.tx_bytes += skb->len;
	}
	atomic64_inc(&packets);
out:
	spin_unlock_bh(&flow_lock);
	return NF_ACCEPT;
}

static const struct nf_hook_ops minet_hooks[] = {
	{ .hook = observe_packet, .pf = NFPROTO_IPV4, .hooknum = NF_INET_LOCAL_IN, .priority = NF_IP_PRI_LAST },
	{ .hook = observe_packet, .pf = NFPROTO_IPV4, .hooknum = NF_INET_LOCAL_OUT, .priority = NF_IP_PRI_LAST },
	{ .hook = observe_packet, .pf = NFPROTO_IPV6, .hooknum = NF_INET_LOCAL_IN, .priority = NF_IP6_PRI_LAST },
	{ .hook = observe_packet, .pf = NFPROTO_IPV6, .hooknum = NF_INET_LOCAL_OUT, .priority = NF_IP6_PRI_LAST },
};
static int stat_show(struct seq_file *file, void *unused)
{
	seq_printf(file, "packets: %lld\nreports: %lld\nreport_errors: %lld\noverflow: %lld\n",
		atomic64_read(&packets), atomic64_read(&reports),
		atomic64_read(&report_errors), atomic64_read(&overflow));
	return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE(stat);

static int __init minet_init(void)
{
	int ret;

	static_assert(sizeof(struct minet_flow) == 56);
	static_assert(sizeof(struct minet_report) == 12);
	if (!family_name || !*family_name || strlen(family_name) >= GENL_NAMSIZ)
		return -EINVAL;
	strscpy(minet_family.name, family_name, sizeof(minet_family.name));
	report_queue = alloc_workqueue("minet", WQ_UNBOUND | WQ_FREEZABLE, 1);
	if (!report_queue)
		return -ENOMEM;
	INIT_DEFERRABLE_WORK(&report_work, publish_flows);
	ret = genl_register_family(&minet_family);
	if (ret)
		goto destroy_queue;
	ret = nf_register_net_hooks(&init_net, minet_hooks, ARRAY_SIZE(minet_hooks));
	if (ret)
		goto unregister_family;
	stat_proc = proc_create("minet_stat", 0444, NULL, &stat_proc_ops);
	if (!stat_proc) {
		ret = -ENOMEM;
		goto remove_hooks;
	}
	queue_delayed_work(report_queue, &report_work, 5 * HZ);
	return 0;
remove_hooks:
	nf_unregister_net_hooks(&init_net, minet_hooks, ARRAY_SIZE(minet_hooks));
unregister_family:
	genl_unregister_family(&minet_family);
destroy_queue:
	destroy_workqueue(report_queue);
	return ret;
}

static void __exit minet_exit(void)
{
	proc_remove(stat_proc);
	nf_unregister_net_hooks(&init_net, minet_hooks, ARRAY_SIZE(minet_hooks));
	cancel_delayed_work_sync(&report_work);
	genl_unregister_family(&minet_family);
	destroy_workqueue(report_queue);
	remove_flows(0);
}
module_init(minet_init);
module_exit(minet_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Per-UID IP flow statistics for Xiaomi Minet");
