/* drivers/misc/usid_stat.c
 *
 * Copyright (C) 2015 SAMSUNG, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *******************************************************************************
 *                                  HISTORY                                    *
 *******************************************************************************
 * ver   who                                         what                      *
 * ---- -------------------------------------------- ------------------------- *
 * 1.0   Junho Jang <vincent.jang@samsung.com>       <2015>                    *
 *                                                   Initial Release           *
 * ---- -------------------------------------------- ------------------------- *
 * 1.1   Hunsup Jung <hunsup.jung@samsung.com>       <2017.05.16>              *
 *                                                   Remove unnecessary code   *
 * ---- -------------------------------------------- ------------------------- *
 * 1.2   Hunsup Jung <hunsup.jung@samsung.com>       <2017.06.12>              *
 *                                                   Just release version 1.2  *
 * ---- -------------------------------------------- ------------------------- *
 * 1.3   Junho Jang <vincent.jang@samsung.com>       <2018>                    *
 *                                                   refactoring           *
 * ---- -------------------------------------------- ------------------------- *
 * 1.4   Junho Jang <vincent.jang@samsung.com>       <2020>                    *
 *                                                   add per-usid usage statistics *
 * ---- -------------------------------------------- ------------------------- *
 */

#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/power/irq_history.h>
#include <linux/security.h>

#include <linux/usid_stat.h>

#ifdef CONFIG_ENERGY_MONITOR
#include <linux/sort.h>
#include <linux/power/energy_monitor.h>
#endif
#ifdef CONFIG_SLEEP_MONITOR
#include <linux/power/sleep_monitor.h>
#endif /* CONFIG_SLEEP_MONITOR */

#define USID_STAT_PREFIX	"usid_stat: "

struct usid_stat {
	struct list_head list;
	pid_t pid;
	char comm[TASK_COMM_LEN];

	u32 usid;
	u32 sid;
	uid_t uid;

	atomic_t total_rcv;
	atomic_t total_snd;
	atomic_t total_rcv_count;
	atomic_t total_snd_count;
	atomic_t current_rcv;
	atomic_t current_snd;
	atomic_t current_rcv_count;
	atomic_t current_snd_count;
	atomic_t wakeup_count;
	atomic_t activity;
	atomic_t total_activity;
	ktime_t first_rcv;
	ktime_t first_snd;
	ktime_t last_transmit;
	int suspend_count;
#ifdef CONFIG_ENERGY_MONITOR
	atomic_t emon_rcv;
	atomic_t emon_snd;
	atomic_t emon_rcv_count;
	atomic_t emon_snd_count;
	atomic_t batr_rcv;
	atomic_t batr_snd;
	atomic_t batr_rcv_count;
	atomic_t batr_snd_count;
#endif
};

static DEFINE_SPINLOCK(usid_lock);
static LIST_HEAD(usid_list);
static ktime_t resume_time;
static int special_usid;

#ifdef CONFIG_ENERGY_MONITOR
static int usid_stat_emon_is_whitelist(struct usid_stat *entry)
{
	/* hard code irq name for now, need to get from device tree */
	if (entry->usid < 5000) {
		if (strncmp(entry->comm, "gpsslogd", 8) == 0 ||
				 strncmp(entry->comm, "lhd", 3) == 0)
			return 1;
	}

	return 0;
}

static int usid_stat_emon_cmp_func(const void *a, const void *b)
{
	struct usid_stat_traffic *pa = (struct usid_stat_traffic *)(a);
	struct usid_stat_traffic *pb = (struct usid_stat_traffic *)(b);
	return ((pb->snd + pb->rcv) - (pa->snd + pb->rcv));
}

void usid_stat_get_traffic_emon(int type,
	struct usid_stat_traffic *tcp_traffic, size_t n)
{
	struct usid_stat *entry;
	unsigned long flags;
	int i = 0;
	unsigned int snd_bytes, rcv_bytes, total_bytes;
	unsigned int snd_count, rcv_count;

	memset(tcp_traffic, 0, sizeof(struct usid_stat_traffic) * n);

	i = 0;
	spin_lock_irqsave(&usid_lock, flags);
	list_for_each_entry(entry, &usid_list, list) {
		if (usid_stat_emon_is_whitelist(entry))
			continue;

		snd_bytes = (unsigned int)(atomic_read(&entry->emon_snd) + INT_MIN);
		rcv_bytes = (unsigned int)(atomic_read(&entry->emon_rcv) + INT_MIN);
		total_bytes = snd_bytes + rcv_bytes;

		if (!total_bytes)
			continue;

		snd_count = (unsigned int)(atomic_read(&entry->emon_snd_count) + INT_MIN);
		rcv_count = (unsigned int)(atomic_read(&entry->emon_rcv_count) + INT_MIN);

		if (i < n) {
			memcpy(&tcp_traffic[i].comm, &entry->comm, TASK_COMM_LEN);
			tcp_traffic[i].usid = entry->usid;
			tcp_traffic[i].uid = entry->uid;
			tcp_traffic[i].sid = entry->sid;
			tcp_traffic[i].snd = snd_bytes;
			tcp_traffic[i].rcv = rcv_bytes;
			tcp_traffic[i].snd_count= snd_count;
			tcp_traffic[i].rcv_count= rcv_count;
			i++;
			if (i == n)
				sort(&tcp_traffic[0],
					n,
					sizeof(struct usid_stat_traffic),
					usid_stat_emon_cmp_func, NULL);
		} else {
			if (total_bytes > (tcp_traffic[n-1].snd + tcp_traffic[n-1].rcv)) {
				memcpy(&tcp_traffic[n-1].comm, &entry->comm, TASK_COMM_LEN);
				tcp_traffic[n-1].usid = entry->usid;
				tcp_traffic[n-1].uid = entry->uid;
				tcp_traffic[n-1].sid = entry->sid;
				tcp_traffic[n-1].snd = snd_bytes;
				tcp_traffic[n-1].rcv = rcv_bytes;
				tcp_traffic[n-1].snd_count= snd_count;
				tcp_traffic[n-1].rcv_count= rcv_count;

				sort(&tcp_traffic[0],
					n,
					sizeof(struct usid_stat_traffic),
					usid_stat_emon_cmp_func, NULL);
			}
		}

		if (type != ENERGY_MON_TYPE_DUMP) {
			atomic_set(&entry->emon_rcv, INT_MIN);
			atomic_set(&entry->emon_snd, INT_MIN);
			atomic_set(&entry->emon_rcv_count, INT_MIN);
			atomic_set(&entry->emon_snd_count, INT_MIN);
		}
	}
	spin_unlock_irqrestore(&usid_lock, flags);

	if (i < n && i != 0)
		sort(&tcp_traffic[0],
			n,
			sizeof(struct usid_stat_traffic),
			usid_stat_emon_cmp_func, NULL);
}

void usid_stat_get_traffic_batr(
	struct usid_stat_traffic *tcp_traffic, size_t n)
{
	struct usid_stat *entry;
	unsigned long flags;
	int i = 0;
	unsigned int snd_bytes, rcv_bytes, total_bytes;
	unsigned int snd_count, rcv_count;

	memset(tcp_traffic, 0, sizeof(struct usid_stat_traffic) * n);

	i = 0;
	spin_lock_irqsave(&usid_lock, flags);
	list_for_each_entry(entry, &usid_list, list) {
		if (usid_stat_emon_is_whitelist(entry))
			continue;

		snd_bytes = (unsigned int)(atomic_read(&entry->batr_snd) + INT_MIN);
		rcv_bytes = (unsigned int)(atomic_read(&entry->batr_rcv) + INT_MIN);
		total_bytes = snd_bytes + rcv_bytes;

		if (!total_bytes)
			continue;

		snd_count = (unsigned int)(atomic_read(&entry->batr_snd_count) + INT_MIN);
		rcv_count = (unsigned int)(atomic_read(&entry->batr_rcv_count) + INT_MIN);

		if (i < n) {
			memcpy(&tcp_traffic[i].comm, &entry->comm, TASK_COMM_LEN);
			tcp_traffic[i].usid = entry->usid;
			tcp_traffic[i].uid = entry->uid;
			tcp_traffic[i].sid = entry->sid;
			tcp_traffic[i].snd = snd_bytes;
			tcp_traffic[i].rcv = rcv_bytes;
			tcp_traffic[i].snd_count= snd_count;
			tcp_traffic[i].rcv_count= rcv_count;
			i++;
			if (i == n)
				sort(&tcp_traffic[0],
					n,
					sizeof(struct usid_stat_traffic),
					usid_stat_emon_cmp_func, NULL);
		} else {
			if (total_bytes > (tcp_traffic[n-1].snd + tcp_traffic[n-1].rcv)) {
				memcpy(&tcp_traffic[n-1].comm, &entry->comm, TASK_COMM_LEN);
				tcp_traffic[n-1].usid = entry->usid;
				tcp_traffic[n-1].uid = entry->uid;
				tcp_traffic[n-1].sid = entry->sid;
				tcp_traffic[n-1].snd = snd_bytes;
				tcp_traffic[n-1].rcv = rcv_bytes;
				tcp_traffic[n-1].snd_count= snd_count;
				tcp_traffic[n-1].rcv_count= rcv_count;

				sort(&tcp_traffic[0],
					n,
					sizeof(struct usid_stat_traffic),
					usid_stat_emon_cmp_func, NULL);
			}
		}

		atomic_set(&entry->batr_rcv, INT_MIN);
		atomic_set(&entry->batr_snd, INT_MIN);
		atomic_set(&entry->batr_rcv_count, INT_MIN);
		atomic_set(&entry->batr_snd_count, INT_MIN);
	}
	spin_unlock_irqrestore(&usid_lock, flags);

	if (i < n && i != 0)
		sort(&tcp_traffic[0],
			n,
			sizeof(struct usid_stat_traffic),
			usid_stat_emon_cmp_func, NULL);
}

#endif

static u32 find_usid(uid_t uid, u32 sid)
{
	u32 usid;
	unsigned n;
	char *ctx = NULL;
	int err;

	if (uid >= 5000) {
		usid = uid + sid;

		if (special_usid == -1) {
			err = security_secid_to_secctx(sid, &ctx, &n);
			if (!err) {
				if (!strcmp(ctx, "User")) {
					special_usid = usid;
					pr_info(USID_STAT_PREFIX"%s: found special_usid: %d\n",
						__func__, special_usid);
				}
				security_release_secctx(ctx, n);
			}
		}
	} else
		usid = uid;

	return usid;
}

static struct usid_stat *find_usid_stat(pid_t pid, u32 usid)
{
	unsigned long flags;
	struct usid_stat *entry;
	struct task_struct *tsk;

	if (usid >= 5000 && usid != special_usid) {
		spin_lock_irqsave(&usid_lock, flags);
		list_for_each_entry(entry, &usid_list, list) {
			if (entry->usid == usid) {
				spin_unlock_irqrestore(&usid_lock, flags);
				return entry;
			}
		}
		spin_unlock_irqrestore(&usid_lock, flags);
	} else {
		rcu_read_lock();
		tsk = find_task_by_vpid(pid);
		if (!tsk) {
			rcu_read_unlock();
			return ERR_PTR(-ESRCH);
		}
		rcu_read_unlock();

		spin_lock_irqsave(&usid_lock, flags);
		list_for_each_entry(entry, &usid_list, list) {
			if (!strcmp(tsk->comm, entry->comm)) {
				entry->pid = pid;
				spin_unlock_irqrestore(&usid_lock, flags);
				return entry;
			}
		}
		spin_unlock_irqrestore(&usid_lock, flags);
	}
	return NULL;
}

/* Create a new entry for tracking the specified usid. */
static struct usid_stat *create_stat(pid_t pid, u32 usid, uid_t uid, u32 sid)
{
	unsigned long flags;
	struct task_struct *tsk;
	struct usid_stat *new_usid;

	/* Create the pid stat struct and append it to the list. */
	new_usid = kmalloc(sizeof(struct usid_stat), GFP_KERNEL);
	if (new_usid == NULL)
		return NULL;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		rcu_read_unlock();
		kfree(new_usid);
		return ERR_PTR(-ESRCH);
	}
	rcu_read_unlock();
	memcpy(new_usid->comm, tsk->comm, TASK_COMM_LEN);

	new_usid->pid = pid;
	new_usid->usid = usid;
	new_usid->uid = uid;
	new_usid->sid = sid;

	/* Counters start at INT_MIN, so we can track 4GB of traffic. */
	atomic_set(&new_usid->total_rcv, INT_MIN);
	atomic_set(&new_usid->total_snd, INT_MIN);
	atomic_set(&new_usid->total_rcv_count, INT_MIN);
	atomic_set(&new_usid->total_snd_count, INT_MIN);

	atomic_set(&new_usid->current_rcv, INT_MIN);
	atomic_set(&new_usid->current_snd, INT_MIN);
	atomic_set(&new_usid->current_rcv_count, INT_MIN);
	atomic_set(&new_usid->current_snd_count, INT_MIN);

	atomic_set(&new_usid->wakeup_count, INT_MIN);
	atomic_set(&new_usid->activity, INT_MIN);
	atomic_set(&new_usid->total_activity, INT_MIN);

#ifdef CONFIG_ENERGY_MONITOR
	atomic_set(&new_usid->emon_rcv, INT_MIN);
	atomic_set(&new_usid->emon_snd, INT_MIN);
	atomic_set(&new_usid->emon_rcv_count, INT_MIN);
	atomic_set(&new_usid->emon_snd_count, INT_MIN);
	atomic_set(&new_usid->batr_rcv, INT_MIN);
	atomic_set(&new_usid->batr_snd, INT_MIN);
	atomic_set(&new_usid->batr_rcv_count, INT_MIN);
	atomic_set(&new_usid->batr_snd_count, INT_MIN);
#endif

	spin_lock_irqsave(&usid_lock, flags);
	list_add_tail(&new_usid->list, &usid_list);
	spin_unlock_irqrestore(&usid_lock, flags);

	return new_usid;
}

int usid_stat_tcp_snd(pid_t pid, int size)
{
	struct usid_stat *entry;
	ktime_t now;
	struct task_struct *tsk;
	uid_t uid;
	u32 usid, sid;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		rcu_read_unlock();
		pr_err(USID_STAT_PREFIX"No task(%d)\n", pid);
		return -ESRCH;
	}
	rcu_read_unlock();

	uid = from_kuid_munged(current_user_ns(), task_uid(tsk));
	security_task_getsecid(tsk, &sid);
	usid = find_usid(uid, sid);

	entry = find_usid_stat(pid, usid);
	if (entry == NULL) {
		entry = create_stat(pid, usid, uid, sid);

		if (entry == NULL) {
			pr_err(USID_STAT_PREFIX"failed to allocate memory\n");
			return -ENOMEM;
		}
	}

	now = ktime_get();
	entry->last_transmit = now;
	if (entry->first_snd.tv64 < resume_time.tv64)
		entry->first_snd = now;

	atomic_add(size, &entry->total_snd);
	atomic_inc(&entry->total_snd_count);
	atomic_add(size, &entry->current_snd);
	atomic_inc(&entry->current_snd_count);

	atomic_set(&entry->activity, INT_MIN + 1);
	entry->suspend_count = suspend_stats.success;

#ifdef CONFIG_ENERGY_MONITOR
	atomic_add(size, &entry->emon_snd);
	atomic_inc(&entry->emon_snd_count);
	atomic_add(size, &entry->batr_snd);
	atomic_inc(&entry->batr_snd_count);
#endif

	return 0;
}

int usid_stat_tcp_rcv(pid_t pid, int size)
{
	struct usid_stat *entry;
	ktime_t now;
	struct task_struct *tsk;
	uid_t uid;
	u32 usid, sid;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		rcu_read_unlock();
		pr_err(USID_STAT_PREFIX"No task(%d)\n", pid);
		return -ESRCH;
	}
	rcu_read_unlock();

	uid = from_kuid_munged(current_user_ns(), task_uid(tsk));
	security_task_getsecid(tsk, &sid);
	usid = find_usid(uid, sid);

	entry = find_usid_stat(pid, usid);
	if (entry == NULL) {
		entry = create_stat(pid, usid, uid, sid);

		if (entry == NULL) {
			pr_err(USID_STAT_PREFIX"failed to allocate memory\n");
			return -ENOMEM;
		}
	}

	now = ktime_get();
	entry->last_transmit = now;
	if (entry->first_rcv.tv64 < resume_time.tv64)
		entry->first_rcv = now;

	atomic_add(size, &entry->total_rcv);
	atomic_inc(&entry->total_rcv_count);
	atomic_add(size, &entry->current_rcv);
	atomic_inc(&entry->current_rcv_count);

	atomic_set(&entry->activity, INT_MIN + 1);
	entry->suspend_count = suspend_stats.success;

#ifdef CONFIG_ENERGY_MONITOR
	atomic_add(size, &entry->emon_rcv);
	atomic_inc(&entry->emon_rcv_count);
	atomic_add(size, &entry->batr_rcv);
	atomic_inc(&entry->batr_rcv_count);
#endif

	return 0;
}

static int usid_stat_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	unsigned int snd_bytes, rcv_bytes;
	unsigned int snd_count, rcv_count;
	unsigned int total_snd_bytes, total_rcv_bytes;
	unsigned int total_snd_count, total_rcv_count;
	unsigned int transmit_count, total_transmit_count;
	unsigned int wakeup_count, activity, total_activity;
	struct usid_stat *entry;
	char *ctx = NULL;
	unsigned n;
	int err;

	seq_printf(m, "name                                             transmit_count "
				"snd_count snd_bytes rcv_count rcv_bytes "
				"activity wakeup_count total_activity total_transmit_count "
				"total_snd_count total_snd_bytes "
				"total_rcv_count total_rcv_bytes "
				"last_transmit suspend_count\n");
	spin_lock_irqsave(&usid_lock, flags);
	list_for_each_entry(entry, &usid_list, list) {
		snd_bytes = (unsigned int)(atomic_read(&entry->current_snd) + INT_MIN);
		rcv_bytes = (unsigned int)(atomic_read(&entry->current_rcv) + INT_MIN);
		snd_count = (unsigned int)(atomic_read(&entry->current_snd_count) + INT_MIN);
		rcv_count = (unsigned int)(atomic_read(&entry->current_rcv_count) + INT_MIN);
		transmit_count = snd_count + rcv_count;

		activity = (unsigned int)(atomic_read(&entry->activity) + INT_MIN);
		wakeup_count = (unsigned int)(atomic_read(&entry->wakeup_count) + INT_MIN);

		total_snd_bytes = (unsigned int)(atomic_read(&entry->total_snd) + INT_MIN);
		total_rcv_bytes = (unsigned int)(atomic_read(&entry->total_rcv) + INT_MIN);
		total_snd_count = (unsigned int)(atomic_read(&entry->total_snd_count) + INT_MIN);
		total_rcv_count = (unsigned int)(atomic_read(&entry->total_rcv_count) + INT_MIN);
		total_activity = (unsigned int)(atomic_read(&entry->total_activity) + INT_MIN) + activity;
		total_transmit_count = total_snd_count + total_rcv_count;

		if (entry->usid >= 5000 && entry->usid != special_usid) {
			err = security_secid_to_secctx(entry->usid - 5001, &ctx, &n);
			if (!err) {
				seq_printf(m, "%-48s %14u "
						"%9u %9u %9u %9u "
						"%8u %12u %14u %20u "
						"%15u %15u "
						"%15u %15u "
						"%13lld %13d\n",
						ctx, transmit_count,
						snd_count, snd_bytes, rcv_count, rcv_bytes,
						activity, wakeup_count, total_activity, total_transmit_count,
						total_snd_count, total_snd_bytes,
						total_rcv_count, total_rcv_bytes,
						ktime_to_ms(entry->last_transmit),
						entry->suspend_count);
				security_release_secctx(ctx, n);
			}
		} else {
			seq_printf(m, "%-48s %14u "
					"%9u %9u %9u %9u "
					"%8u %12u %14u %20u "
					"%15u %15u "
					"%15u %15u "
					"%13lld %13d\n",
					entry->comm, transmit_count,
					snd_count, snd_bytes, rcv_count, rcv_bytes,
					activity, wakeup_count, total_activity, total_transmit_count,
					total_snd_count, total_snd_bytes,
					total_rcv_count, total_rcv_bytes,
					ktime_to_ms(entry->last_transmit),
					entry->suspend_count);
		}
	}
	spin_unlock_irqrestore(&usid_lock, flags);

	return 0;
}

static int usid_stat_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, usid_stat_show, NULL);
}

static const struct file_operations usid_stat_fops = {
	.open       = usid_stat_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int usid_stat_is_network_wakeup(void)
{
	int ret = 0;
	struct irq_history last;

	get_last_irq_history(&last);

	/* hard code irq name for now, need to get from device tree */
	if (!strncmp(last.name, "CP", IRQ_NAME_LENGTH) ||
		!strncmp(last.name, "WIFI", IRQ_NAME_LENGTH))
		ret = 1;

	return ret;
}

static int usid_stat_pm_suspend_prepare_cb(ktime_t ktime)
{
	struct usid_stat *wakeup_entry = NULL;
	struct usid_stat *entry;
	unsigned long flags;
	unsigned int snd_bytes, rcv_bytes;
	unsigned int snd_count, rcv_count, transmit_count;
	unsigned int entry_cnt = 0;
	int nw_wakeup;

	nw_wakeup = usid_stat_is_network_wakeup();

	spin_lock_irqsave(&usid_lock, flags);
	list_for_each_entry(entry, &usid_list, list) {
		if (entry->last_transmit.tv64 > ktime.tv64) {
			snd_count = (unsigned int)(atomic_read(&entry->current_snd_count) + INT_MIN);

			if (!wakeup_entry)
				wakeup_entry = entry;
			else if ((snd_count == 0 ||
					entry->first_rcv.tv64 < entry->first_snd.tv64) &&
					wakeup_entry->first_rcv.tv64 > entry->first_rcv.tv64) {
				wakeup_entry = entry;
			}

			entry_cnt++;
			atomic_inc(&entry->total_activity);
			snd_bytes = (unsigned int)(atomic_read(&entry->current_snd) + INT_MIN);
			rcv_bytes = (unsigned int)(atomic_read(&entry->current_rcv) + INT_MIN);
			rcv_count = (unsigned int)(atomic_read(&entry->current_rcv_count) + INT_MIN);
			transmit_count = snd_count + rcv_count;

			if (entry->usid >= 5000 && entry->usid != special_usid) {
				pr_info(USID_STAT_PREFIX"%4d: %2d %-16u "
						"%6u "
						"%6u %6u %10u %6u %10u "
						"%10lld "
						"%10lld %10lld "
						"%10lld "
						"%10lld\n",
						suspend_stats.success, entry_cnt, entry->usid,
						atomic_read(&entry->total_activity) + INT_MIN,
						transmit_count, snd_count, snd_bytes, rcv_count, rcv_bytes,
						ktime_to_ms(ktime),
						ktime_to_ms(entry->first_snd), ktime_to_ms(entry->first_rcv),
						ktime_to_ms(entry->last_transmit),
						ktime_to_ms(ktime_sub(entry->last_transmit, ktime)));		
			} else {
				pr_info(USID_STAT_PREFIX"%4d: %2d %-16s "
						"%6u "
						"%6u %6u %10u %6u %10u "
						"%10lld "
						"%10lld %10lld "
						"%10lld "
						"%10lld\n",
						suspend_stats.success, entry_cnt, entry->comm,
						atomic_read(&entry->total_activity) + INT_MIN,
						transmit_count, snd_count, snd_bytes, rcv_count, rcv_bytes,
						ktime_to_ms(ktime),
						ktime_to_ms(entry->first_snd), ktime_to_ms(entry->first_rcv),
						ktime_to_ms(entry->last_transmit),
						ktime_to_ms(ktime_sub(entry->last_transmit, ktime)));
			}
		}
	}
	if (wakeup_entry && nw_wakeup) {
		if (wakeup_entry->usid >= 5000 && wakeup_entry->usid != special_usid)
			pr_info(USID_STAT_PREFIX"%16u\n", wakeup_entry->usid);
		else
			pr_info(USID_STAT_PREFIX"%16s\n", wakeup_entry->comm);
		atomic_inc(&wakeup_entry->wakeup_count);
	}
	spin_unlock_irqrestore(&usid_lock, flags);

	return 0;
}

static int usid_stat_pm_post_suspend_cb(ktime_t ktime)
{
	unsigned long flags;
	struct usid_stat *entry;

	spin_lock_irqsave(&usid_lock, flags);
	list_for_each_entry(entry, &usid_list, list) {
		if (entry->last_transmit.tv64 > ktime.tv64) {
			/* Reset counter, so we can track  traffic during next post suspend. */
			atomic_set(&entry->current_rcv, INT_MIN);
			atomic_set(&entry->current_snd, INT_MIN);
			atomic_set(&entry->current_rcv_count, INT_MIN);
			atomic_set(&entry->current_snd_count, INT_MIN);
			atomic_set(&entry->activity, INT_MIN);
		}
	}
	spin_unlock_irqrestore(&usid_lock, flags);

	return 0;
}

static int usid_stat_pm_notifier(struct notifier_block *nb,
		unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		usid_stat_pm_suspend_prepare_cb(resume_time);
		break;
	case PM_POST_SUSPEND:
		usid_stat_pm_post_suspend_cb(resume_time);
		resume_time =  ktime_get();
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block usid_stat_notifier_block = {
	.notifier_call = usid_stat_pm_notifier,
};

#ifdef CONFIG_SLEEP_MONITOR
static int usid_stat_sleep_monitor_read_cb(void *priv,
		unsigned int *raw_val, int check_level, int caller_type)
{
	int mask = 0;
	unsigned long flags;
	unsigned int total_snd_count = 0, total_rcv_count = 0;
	unsigned int total_transmit_count = 0, total_act_cnt = 0;
	struct usid_stat *entry;
	ktime_t *resume_time = (ktime_t *)priv;

	if (caller_type != SLEEP_MONITOR_CALL_SUSPEND) {
		*raw_val = 0;
		return 0;
	}

	spin_lock_irqsave(&usid_lock, flags);
	list_for_each_entry(entry, &usid_list, list) {
		if (entry->last_transmit.tv64 > resume_time->tv64) {
			total_act_cnt += (unsigned int) (atomic_read(&entry->activity) + INT_MIN);
			total_snd_count = (unsigned int) (atomic_read(&entry->current_snd_count) + INT_MIN);
			total_rcv_count = (unsigned int) (atomic_read(&entry->current_rcv_count) + INT_MIN);
			total_transmit_count += total_snd_count + total_rcv_count;
		}
	}
	spin_unlock_irqrestore(&usid_lock, flags);

	mask = 0xffffffff;
	*raw_val = (total_transmit_count > mask) ? mask : total_transmit_count;

	if (total_act_cnt == 0)
		return DEVICE_POWER_OFF;
	else
		return (total_act_cnt > DEVICE_UNKNOWN) ? DEVICE_UNKNOWN : total_act_cnt;
}

static struct sleep_monitor_ops usid_stat_sleep_monitor_ops = {
	.read_cb_func = usid_stat_sleep_monitor_read_cb,
};
#endif

static int __init usid_stat_init(void)
{
	struct dentry *root;

	root = debugfs_create_dir("usid_stat", NULL);
	if (!root) {
		pr_err(USID_STAT_PREFIX"failed to create sap_pid_stat debugfs directory\n");
		return -ENOMEM;
	}

	/* Make interface to read the tcp traffic statistic */
	if (!debugfs_create_file("stat", 0660, root, NULL, &usid_stat_fops))
		goto error_debugfs;

	if (register_pm_notifier(&usid_stat_notifier_block))
		goto error_debugfs;

	special_usid = -1;

#ifdef CONFIG_SLEEP_MONITOR
	sleep_monitor_register_ops(&resume_time,
			&usid_stat_sleep_monitor_ops,
			SLEEP_MONITOR_TCP);
#endif

	return 0;

error_debugfs:
    debugfs_remove_recursive(root);

	return -1;
}

late_initcall(usid_stat_init);
