// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/cgroup.h>
#include <linux/cred.h>
#include <linux/freezer.h>
#include <linux/netlink.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/pid.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/signal.h>
#include <linux/timekeeping.h>
#include <linux/user_namespace.h>
#include <linux/tracepoint.h>
#include <net/net_namespace.h>
#include <net/netfilter/nf_socket.h>
#include <net/sock.h>
#include <trace/hooks/binder.h>
#include <trace/hooks/signal.h>
#include "binder_internal.h"
#include "millet_wire.h"

static unsigned int netlink_port = 30;
module_param(netlink_port, uint, 0444);
static unsigned int frozen_uid_min = 10000;
module_param(frozen_uid_min, uint, 0600);
static unsigned int test_uid;
module_param(test_uid, uint, 0444);
static unsigned long binder_warn_ahead_space = 128 * 1024;
module_param(binder_warn_ahead_space, ulong, 0600);

static int (*unfreeze_binder_proc)(struct binder_proc *proc);
static DEFINE_SPINLOCK(policy_lock);
static u32 core_uids[256], black_pids[10], black_uids[10];
static bool freezebinder_enabled;
static atomic64_t policy_filtered = ATOMIC64_INIT(0);
static atomic64_t policy_unfrozen = ATOMIC64_INIT(0);

static bool table_has(const u32 *table, size_t count, u32 value)
{
 size_t i;
 for (i = 0; i < count; i++) if (table[i] == value) return true;
 return false;
}
static int table_change(u32 *table, size_t count, u32 value, bool add)
{
 size_t i, empty = count;
 for (i = 0; i < count; i++) {
  if (table[i] == value) { if (!add) table[i] = 0; return 0; }
  if (!table[i] && empty == count) empty = i;
 }
 if (!add) return 0;
 if (empty == count) return -ENOSPC;
 table[empty] = value;
 return 0;
}
static bool caller_is_important(u32 uid, u32 pid, u32 target_uid)
{
 unsigned long flags;
 bool allow;
 u32 appid = uid % 100000;
 spin_lock_irqsave(&policy_lock, flags);
 if (!uid || !freezebinder_enabled || table_has(core_uids, ARRAY_SIZE(core_uids), target_uid))
  allow = true;
 else if (table_has(black_pids, ARRAY_SIZE(black_pids), pid))
  allow = false;
 else if (appid >= READ_ONCE(frozen_uid_min))
  allow = table_has(core_uids, ARRAY_SIZE(core_uids), uid);
 else
  allow = !table_has(black_uids, ARRAY_SIZE(black_uids), appid);
 spin_unlock_irqrestore(&policy_lock, flags);
 return allow;
}

static struct sock *millet_socket;
static struct proc_dir_entry *millet_proc;
static u32 peers[MILLET_CHANNELS];
static u32 watched_uids[64];
static bool watched_armed[64];
static u64 watched_generation[64];
static u64 watch_generation;
static DEFINE_SPINLOCK(watched_lock);
static struct hlist_head *binder_processes;
static struct mutex *binder_processes_lock;
static atomic64_t sent = ATOMIC64_INIT(0);
static atomic64_t dropped = ATOMIC64_INIT(0);
static atomic64_t rejected = ATOMIC64_INIT(0);
static atomic64_t notifications[MILLET_CHANNELS];
static atomic64_t packet_v4 = ATOMIC64_INIT(0);
static atomic64_t packet_v6 = ATOMIC64_INIT(0);
static atomic64_t lookup_misses = ATOMIC64_INIT(0);
static u32 last_packet_uid;
static u32 rejected_channel, rejected_kind, rejected_command, rejected_uid;
static u32 rejected_length;

static int stats_get(char *buf, const struct kernel_param *parameter)
{
	unsigned long flags;
	unsigned int i, core_count = 0, pid_count = 0, uid_count = 0;

	spin_lock_irqsave(&policy_lock, flags);
	for (i = 0; i < ARRAY_SIZE(core_uids); i++) core_count += !!core_uids[i];
	for (i = 0; i < ARRAY_SIZE(black_pids); i++) pid_count += !!black_pids[i];
	for (i = 0; i < ARRAY_SIZE(black_uids); i++) uid_count += !!black_uids[i];
	spin_unlock_irqrestore(&policy_lock, flags);
	return scnprintf(buf, PAGE_SIZE,
		"sent=%lld dropped=%lld rejected=%lld signal=%lld binder=%lld state=%lld packet=%lld handshake=%lld ipv4=%lld ipv6=%lld lookup_miss=%lld last_uid=%u rejected_channel=%u rejected_kind=%u rejected_command=%u rejected_uid=%u rejected_length=%u core_uids=%u black_pids=%u black_uids=%u freezebinder=%u supported=%u filtered=%lld unfrozen=%lld\n",
		atomic64_read(&sent), atomic64_read(&dropped), atomic64_read(&rejected),
		atomic64_read(&notifications[1]), atomic64_read(&notifications[2]),
		atomic64_read(&notifications[3]), atomic64_read(&notifications[5]),
		atomic64_read(&notifications[6]), atomic64_read(&packet_v4),
		atomic64_read(&packet_v6), atomic64_read(&lookup_misses),
		READ_ONCE(last_packet_uid), READ_ONCE(rejected_channel),
		READ_ONCE(rejected_kind), READ_ONCE(rejected_command),
		READ_ONCE(rejected_uid), READ_ONCE(rejected_length), core_count, pid_count, uid_count, READ_ONCE(freezebinder_enabled), !!unfreeze_binder_proc, atomic64_read(&policy_filtered), atomic64_read(&policy_unfrozen));
}

static const struct kernel_param_ops stats_ops = { .get = stats_get };
module_param_cb(stats, &stats_ops, NULL, 0444);

static int millet_version_show(struct seq_file *file, void *unused)
{
	seq_puts(file, "1\n");
	return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE(millet_version);

static int millet_stat_show(struct seq_file *file, void *unused)
{
	seq_printf(file, "sent: %lld\ndropped: %lld\nrejected: %lld\n",
		   atomic64_read(&sent), atomic64_read(&dropped), atomic64_read(&rejected));
	return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE(millet_stat);

static bool channel_supported(u32 channel)
{
	return channel > 0 && channel < MILLET_CHANNELS &&
	       channel != MILLET_RESERVED;
}

static bool task_is_frozen(struct task_struct *task)
{
	return task && (freezing(task) || (READ_ONCE(task->__state) & TASK_FROZEN) ||
			cgroup_task_frozen(task));
}

static u32 task_uid_value(struct task_struct *task)
{
	return __kuid_val(task_uid(task));
}

static int send_event(struct millet_event *event)
{
	struct sk_buff *skb;
	struct nlmsghdr *header;
	struct timespec64 now;
	u32 peer, monitor;
	int ret;

	if (!channel_supported(event->channel))
		return -EINVAL;
	switch (event->channel) {
	case MILLET_BINDER_STAT:
		monitor = MILLET_BINDER;
		break;
	case MILLET_HANDSHAKE:
		monitor = MILLET_SIGNAL;
		break;
	default:
		monitor = event->channel;
	}
	peer = READ_ONCE(peers[monitor]);
	if (!peer)
		return -ENOTCONN;
	skb = nlmsg_new(sizeof(*event), GFP_ATOMIC);
	if (!skb) {
		atomic64_inc(&dropped);
		return -ENOMEM;
	}
	header = nlmsg_put(skb, 0, 0, 0, sizeof(*event), 0);
	if (!header) {
		kfree_skb(skb);
		atomic64_inc(&dropped);
		return -EMSGSIZE;
	}
	event->monitor = monitor;
	event->source = MILLET_KERNEL_PORT;
	event->destination = MILLET_USER_PORT;
	ktime_get_ts64(&now);
	event->seconds = now.tv_sec;
	event->nanoseconds = now.tv_nsec;
	memcpy(nlmsg_data(header), event, sizeof(*event));
	ret = netlink_unicast(millet_socket, skb, peer, MSG_DONTWAIT);
	if (ret < 0) {
		atomic64_inc(&dropped);
		if (ret == -ECONNREFUSED)
			cmpxchg(&peers[monitor], peer, 0);
	} else {
		atomic64_inc(&sent);
		atomic64_inc(&notifications[event->channel]);
	}
	return ret;
}

static void binder_preset(void *unused, struct hlist_head *head,
			  struct mutex *lock, struct binder_proc *proc)
{
	WRITE_ONCE(binder_processes_lock, lock);
	smp_store_release(&binder_processes, head);
}

static void binder_notify(struct binder_proc *target, struct binder_proc *caller,
			  struct binder_thread *thread,
			  struct binder_transaction_data *transaction, u64 reason)
{
	struct millet_event event = {
		.channel = MILLET_BINDER,
		.kind = MILLET_EVENT,
		.binder_event = reason,
	};
	struct task_struct *task;

	if (!target || !caller || !READ_ONCE(peers[MILLET_BINDER]))
		return;
	task = target->tsk;
	if (!task || task_uid_value(task) < frozen_uid_min ||
	    (test_uid && task_uid_value(task) != test_uid) ||
	    task_tgid_nr(task) == task_tgid_nr(caller->tsk) || !task_is_frozen(task))
		return;
	event.uid = task_uid_value(task);
	event.caller_uid = task_uid_value(caller->tsk);
	event.caller_pid = task_tgid_nr(caller->tsk);
	event.caller_tid = thread ? thread->pid : task_pid_nr(current);
	event.target_pid = task_tgid_nr(task);
	event.oneway = !!(transaction->flags & TF_ONE_WAY);
	event.code = transaction->code;
	if (READ_ONCE(freezebinder_enabled) && reason != 0) {
		bool allow = caller_is_important(event.caller_uid, event.caller_pid, event.uid);

		if (reason == 2 && !event.oneway && allow) {
			int ret = unfreeze_binder_proc(target);

			if (ret >= 0) {
				event.binder_event = 4;
				if (ret) atomic64_inc(&policy_unfrozen);
			}
		}
		if (!allow && event.binder_event != 4) {
			atomic64_inc(&policy_filtered);
			if (event.oneway) return;
			event.binder_event += 111;
		}
	}
	send_event(&event);
}

static void binder_trans(void *unused, struct binder_proc *target,
			 struct binder_proc *caller, struct binder_thread *thread,
			 struct binder_transaction_data *transaction)
{
	size_t size, free;

	binder_notify(target, caller, thread, transaction, 2);
	if (!(transaction->flags & TF_ONE_WAY) || !target)
		return;
	if (check_add_overflow((size_t)transaction->data_size,
			       (size_t)transaction->offsets_size, &size) ||
	    check_add_overflow(size, sizeof(struct binder_buffer), &size) ||
	    check_mul_overflow(size, (size_t)3, &size))
		return;
	free = binder_alloc_get_free_async_space(&target->alloc);
	if (free < size || free < READ_ONCE(binder_warn_ahead_space))
		binder_notify(target, caller, thread, transaction, 0);
}

static void binder_reply(void *unused, struct binder_proc *target,
			 struct binder_proc *caller, struct binder_thread *thread,
			 struct binder_transaction_data *transaction)
{
	binder_notify(target, caller, thread, transaction, 1);
}

static void binder_wait(void *unused, bool do_proc_work,
			struct binder_thread *thread, struct binder_proc *proc)
{
	struct binder_transaction *transaction;
	struct task_struct *target = NULL;
	struct millet_event event = {
		.channel = MILLET_BINDER,
		.kind = MILLET_EVENT,
		.binder_event = 3,
	};

	if (!READ_ONCE(peers[MILLET_BINDER]) || !thread)
		return;
	/* The hook runs with proc->inner_lock held; it pins this stack entry. */
	transaction = thread->transaction_stack;
	if (!transaction)
		return;
	spin_lock(&transaction->lock);
	if (transaction->to_proc && transaction->to_proc->tsk &&
	    transaction->to_proc != proc && !(transaction->flags & TF_ONE_WAY)) {
		target = transaction->to_proc->tsk;
		get_task_struct(target);
		event.code = transaction->code;
	}
	spin_unlock(&transaction->lock);
	if (!target)
		return;
	if (task_is_frozen(target) && task_uid_value(target) >= frozen_uid_min &&
	    (!test_uid || task_uid_value(target) == test_uid)) {
		event.uid = task_uid_value(target);
		event.target_pid = task_tgid_nr(target);
		event.caller_uid = task_uid_value(current);
		event.caller_pid = task_tgid_nr(current);
		event.caller_tid = task_pid_nr(current);
		if (READ_ONCE(freezebinder_enabled) &&
		    !caller_is_important(event.caller_uid, event.caller_pid, event.uid)) {
			event.binder_event += 111;
			atomic64_inc(&policy_filtered);
		}
		send_event(&event);
	}
	put_task_struct(target);
}

static bool binder_thread_busy(struct binder_thread *thread)
{
	struct binder_transaction *transaction;
	bool busy;

	if (!list_empty(&thread->todo))
		return true;
	transaction = thread->transaction_stack;
	if (!transaction)
		return false;
	spin_lock(&transaction->lock);
	busy = transaction->to_thread == thread;
	spin_unlock(&transaction->lock);
	return busy;
}

static int query_binder_uid(u32 uid)
{
	struct hlist_head *head = smp_load_acquire(&binder_processes);
	struct mutex *lock = READ_ONCE(binder_processes_lock);
	struct binder_proc *proc;
	struct rb_node *node;
	struct millet_event event = {
		.channel = MILLET_BINDER_STAT,
		.kind = MILLET_EVENT,
		.uid = uid,
		/* The legacy state response uses this slot for the requester PID. */
		.reserved_pointer1 = task_tgid_nr(current),
	};
	bool busy = false;

	if (!head || !lock)
		return -EAGAIN;
	mutex_lock(lock);
	hlist_for_each_entry(proc, head, proc_node) {
		if (!proc->tsk || task_uid_value(proc->tsk) != uid)
			continue;
		spin_lock(&proc->inner_lock);
		busy = !list_empty(&proc->todo);
		for (node = rb_first(&proc->threads); node && !busy; node = rb_next(node)) {
			struct binder_thread *thread = rb_entry(node, struct binder_thread, rb_node);
			busy = binder_thread_busy(thread);
		}
		spin_unlock(&proc->inner_lock);
		if (busy)
			break;
	}
	mutex_unlock(lock);
	event.caller_uid = busy;
	return send_event(&event);
}

static void signal_sent(void *unused, int signal, struct task_struct *killer,
			struct task_struct *target)
{
	struct millet_event event = { .channel = MILLET_SIGNAL, .kind = MILLET_EVENT };

	if ((signal != SIGQUIT && signal != SIGABRT && signal != SIGKILL && signal != SIGTERM) ||
	    !target || !READ_ONCE(peers[MILLET_SIGNAL]) || !task_is_frozen(target))
		return;
	if (test_uid && task_uid_value(target) != test_uid)
		return;
	event.uid = task_uid_value(target);
	event.caller_uid = task_tgid_nr(target);
	send_event(&event);
}

static int extended_packet_command(u32 command, u32 uid)
{
 unsigned long flags;
 bool add = !(command & 1);
 int ret;
 struct millet_event event = { .channel = MILLET_PACKET, .kind = MILLET_EVENT, .uid = 111 };

 if (command < 100 || command > 105 || !uid || uid > INT_MAX)
  return -EINVAL;
 if (command <= 101 && uid == 111) {
  if (!unfreeze_binder_proc) return -EOPNOTSUPP;
  spin_lock_irqsave(&policy_lock, flags);
  freezebinder_enabled = add;
  spin_unlock_irqrestore(&policy_lock, flags);
  return add ? send_event(&event) : 0;
 }
 if (command != 102 && command != 103 && !uid_valid(make_kuid(&init_user_ns, uid)))
  return -EINVAL;
 spin_lock_irqsave(&policy_lock, flags);
 if (command <= 101)
  ret = table_change(core_uids, ARRAY_SIZE(core_uids), uid, add);
 else if (command <= 103)
  ret = table_change(black_pids, ARRAY_SIZE(black_pids), uid, add);
 else
  ret = table_change(black_uids, ARRAY_SIZE(black_uids), uid, add);
 spin_unlock_irqrestore(&policy_lock, flags);
 return ret;
}

static int packet_command(u32 command, u32 uid)
{
	unsigned long flags;
	int i, empty = -1, ret = 0;

	if (command >= 100)
		return extended_packet_command(command, uid);
	if (command < 1 || command > 3 || (command != 3 && !uid))
		return -EINVAL;
	spin_lock_irqsave(&watched_lock, flags);
	if (command == 3) {
		memset(watched_uids, 0, sizeof(watched_uids));
		memset(watched_armed, 0, sizeof(watched_armed));
		for (i = 0; i < ARRAY_SIZE(watched_generation); i++)
			watched_generation[i] = ++watch_generation;
		goto done;
	}
	for (i = 0; i < ARRAY_SIZE(watched_uids); i++) {
		if (watched_uids[i] == uid) {
			watched_generation[i] = ++watch_generation;
			watched_armed[i] = command == 1;
			if (command == 2)
				watched_uids[i] = 0;
			goto done;
		}
		if (!watched_uids[i] && empty < 0)
			empty = i;
	}
	if (command == 1) {
		if (empty < 0)
			ret = -ENOSPC;
		else {
			watched_uids[empty] = uid;
			watched_armed[empty] = true;
			watched_generation[empty] = ++watch_generation;
		}
	}
done:
	spin_unlock_irqrestore(&watched_lock, flags);
	return ret;
}

static unsigned int packet_received(void *unused, struct sk_buff *skb,
				    const struct nf_hook_state *state)
{
	struct millet_event event = { .channel = MILLET_PACKET, .kind = MILLET_EVENT };
	struct sock *sk;
	u32 uid;
	u64 generation = 0;
	unsigned long flags;
	int i, ret, claimed = -1;
	bool watched = false;

	if (!READ_ONCE(peers[MILLET_PACKET]))
		return NF_ACCEPT;
	for (i = 0; i < ARRAY_SIZE(watched_uids); i++)
		watched |= READ_ONCE(watched_uids[i]) != 0;
	if (!watched)
		return NF_ACCEPT;
	atomic64_inc(state->pf == NFPROTO_IPV4 ? &packet_v4 : &packet_v6);
	sk = state->pf == NFPROTO_IPV4 ?
		nf_sk_lookup_slow_v4(state->net, skb, state->in) :
		nf_sk_lookup_slow_v6(state->net, skb, state->in);
	if (!sk) {
		atomic64_inc(&lookup_misses);
		return NF_ACCEPT;
	}
	if (!sk_fullsock(sk)) {
		sock_gen_put(sk);
		return NF_ACCEPT;
	}
	uid = __kuid_val(sock_i_uid(sk));
	WRITE_ONCE(last_packet_uid, uid);
	sock_gen_put(sk);
	if (!uid)
		return NF_ACCEPT;
	spin_lock_irqsave(&watched_lock, flags);
	for (i = 0; i < ARRAY_SIZE(watched_uids); i++) {
		if (watched_uids[i] != uid || !watched_armed[i])
			continue;
		claimed = i;
		generation = watched_generation[i];
		watched_armed[i] = false;
		break;
	}
	spin_unlock_irqrestore(&watched_lock, flags);
	if (claimed < 0)
		return NF_ACCEPT;
	event.uid = uid;
	ret = send_event(&event);
	spin_lock_irqsave(&watched_lock, flags);
	if (watched_uids[claimed] == uid && watched_generation[claimed] == generation) {
		if (ret < 0)
			watched_armed[claimed] = true;
		else
			watched_uids[claimed] = 0;
	}
	spin_unlock_irqrestore(&watched_lock, flags);
	return NF_ACCEPT;
}

static struct nf_hook_ops packet_hooks[] = {
	{ .hook = packet_received, .pf = NFPROTO_IPV4,
	  .hooknum = NF_INET_LOCAL_IN, .priority = NF_IP_PRI_LAST },
	{ .hook = packet_received, .pf = NFPROTO_IPV6,
	  .hooknum = NF_INET_LOCAL_IN, .priority = NF_IP6_PRI_LAST },
};

static void receive_command(struct sk_buff *skb)
{
	struct millet_command command = {};
	struct nlmsghdr *header;
	struct millet_event event = { .kind = MILLET_LOOPBACK };
	u32 uid = __kuid_val(NETLINK_CB(skb).creds.uid);
	u32 peer = NETLINK_CB(skb).portid;
	int remaining, ret;

	if ((uid != 0 && uid != 1000) || !peer) {
		atomic64_inc(&rejected);
		return;
	}
	nlmsg_for_each_msg(header, nlmsg_hdr(skb), skb->len, remaining) {
		if (nlmsg_len(header) != sizeof(command))
			goto reject;
		memcpy(&command, nlmsg_data(header), sizeof(command));
		memset(&event, 0, sizeof(event));
		if (!channel_supported(command.channel) || command.source != MILLET_USER_PORT ||
		    command.destination != MILLET_KERNEL_PORT)
			goto reject;
		if (command.kind == MILLET_LOOPBACK) {
			WRITE_ONCE(peers[command.channel], peer);
			memset(&event, 0, sizeof(event));
			event.kind = MILLET_LOOPBACK;
			event.channel = command.channel;
			send_event(&event);
			if (command.channel == MILLET_PACKET && READ_ONCE(freezebinder_enabled)) {
				event.kind = MILLET_EVENT;
				event.uid = 111;
				send_event(&event);
			}
			continue;
		}
		/* Commands come from the framework's own socket, not the monitor socket. */
		if (command.kind != MILLET_COMMAND)
			goto reject;
		switch (command.channel) {
		case MILLET_BINDER:
		case MILLET_BINDER_STAT:
			ret = query_binder_uid(command.command);
			break;
		case MILLET_PACKET:
			ret = packet_command(command.command, command.uid);
			break;
		case MILLET_HANDSHAKE:
			event.channel = MILLET_HANDSHAKE;
			event.kind = MILLET_EVENT;
			ret = send_event(&event);
			break;
		default:
			ret = -EOPNOTSUPP;
		}
		if (ret < 0) {
			WRITE_ONCE(rejected_channel, command.channel);
			WRITE_ONCE(rejected_kind, command.kind);
			WRITE_ONCE(rejected_command, command.command);
			WRITE_ONCE(rejected_uid, uid);
			WRITE_ONCE(rejected_length, nlmsg_len(header));
			atomic64_inc(&rejected);
		}
	}
	if (remaining)
		atomic64_inc(&rejected);
	return;
reject:
	WRITE_ONCE(rejected_channel, command.channel);
	WRITE_ONCE(rejected_kind, command.kind);
	WRITE_ONCE(rejected_command, command.command);
	WRITE_ONCE(rejected_uid, uid);
	WRITE_ONCE(rejected_length, nlmsg_len(header));
	atomic64_inc(&rejected);
}

static int __init millet_init(void)
{
	struct netlink_kernel_cfg cfg = { .input = receive_command };
	int ret;

	static_assert(sizeof(struct millet_command) == 80);
	static_assert(sizeof(struct millet_event) == 144);
	static_assert(offsetof(struct millet_event, caller_uid) == 120);
	if (netlink_port < 23 || netlink_port >= MAX_LINKS)
		return -EINVAL;
	unfreeze_binder_proc = (void *)__symbol_get("binder_proc_unfreeze");
	millet_socket = netlink_kernel_create(&init_net, netlink_port, &cfg);
	if (!millet_socket) {
		if (unfreeze_binder_proc) __symbol_put("binder_proc_unfreeze");
		return -EADDRINUSE;
	}
	ret = register_trace_android_vh_binder_preset(binder_preset, NULL);
	if (ret)
		goto close_socket;
	ret = register_trace_android_vh_binder_trans(binder_trans, NULL);
	if (ret)
		goto remove_preset;
	ret = register_trace_android_vh_binder_reply(binder_reply, NULL);
	if (ret)
		goto remove_trans;
	ret = register_trace_android_vh_binder_wait_for_work(binder_wait, NULL);
	if (ret)
		goto remove_reply;
	ret = register_trace_android_vh_do_send_sig_info(signal_sent, NULL);
	if (ret)
		goto remove_wait;
	ret = nf_register_net_hooks(&init_net, packet_hooks, ARRAY_SIZE(packet_hooks));
	if (ret)
		goto remove_signal;
	millet_proc = proc_mkdir(netlink_port == 30 ? "millet" : "millet_compat", NULL);
	if (millet_proc &&
	    proc_create("version", 0444, millet_proc, &millet_version_proc_ops) &&
	    proc_create("millet_stat", 0444, millet_proc, &millet_stat_proc_ops))
		return 0;
	proc_remove(millet_proc);
	nf_unregister_net_hooks(&init_net, packet_hooks, ARRAY_SIZE(packet_hooks));
	ret = -ENOMEM;
remove_signal:
	unregister_trace_android_vh_do_send_sig_info(signal_sent, NULL);
remove_wait:
	unregister_trace_android_vh_binder_wait_for_work(binder_wait, NULL);
remove_reply:
	unregister_trace_android_vh_binder_reply(binder_reply, NULL);
remove_trans:
	unregister_trace_android_vh_binder_trans(binder_trans, NULL);
remove_preset:
	unregister_trace_android_vh_binder_preset(binder_preset, NULL);
	tracepoint_synchronize_unregister();
close_socket:
	netlink_kernel_release(millet_socket);
	if (unfreeze_binder_proc) __symbol_put("binder_proc_unfreeze");
	return ret;
}

static void __exit millet_exit(void)
{
	proc_remove(millet_proc);
	nf_unregister_net_hooks(&init_net, packet_hooks, ARRAY_SIZE(packet_hooks));
	unregister_trace_android_vh_do_send_sig_info(signal_sent, NULL);
	unregister_trace_android_vh_binder_wait_for_work(binder_wait, NULL);
	unregister_trace_android_vh_binder_reply(binder_reply, NULL);
	unregister_trace_android_vh_binder_trans(binder_trans, NULL);
	unregister_trace_android_vh_binder_preset(binder_preset, NULL);
	tracepoint_synchronize_unregister();
	netlink_kernel_release(millet_socket);
	if (unfreeze_binder_proc) __symbol_put("binder_proc_unfreeze");
}

module_init(millet_init);
module_exit(millet_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Binder, signal and packet notifications for the Millet protocol");
