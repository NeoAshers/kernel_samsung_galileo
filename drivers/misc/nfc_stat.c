/* drivers/misc/nfc_stat.c
 *
 * Copyright (C) 2020 SAMSUNG, Inc.
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
 * 1.0   Junho Jang <vincent.jang@samsung.com>       <2020>                    *
 *                                                   Initial Release           *
 * ---- -------------------------------------------- ------------------------- *
 */

#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/security.h>

#include <linux/nfc_stat.h>

#ifdef CONFIG_ENERGY_MONITOR
#include <linux/sort.h>
#include <linux/power/energy_monitor.h>
#endif

#define NFC_STAT_PREFIX	"nfc_stat: "

struct nfc_subsystem {
	ktime_t total_time;
	ktime_t max_time;
	ktime_t max_time_stamp;
	ktime_t last_time;

	unsigned long active_count;
	unsigned long relax_count;
	bool			active;
};

struct se_subsystem {
	ktime_t total_time;
	ktime_t max_time;
	ktime_t max_time_stamp;
	ktime_t last_time;

	unsigned long active_count;
	unsigned long relax_count;
	bool			active;
};

struct nfc_stat {
	struct list_head list;
	pid_t pid;
	char comm[TASK_COMM_LEN];

	u32 usid;
	u32 sid;
	uid_t uid;

	/* nfc subsystem is not currently supported
	 * struct nfc_subsystem nfc;
	*/
	struct se_subsystem se;
};

static DEFINE_SPINLOCK(nfc_lock);
static LIST_HEAD(nfc_list);
static ktime_t resume_time;
static int special_usid;

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
					pr_info(NFC_STAT_PREFIX"%s: found special_usid: %d\n",
						__func__, special_usid);
				}
				security_release_secctx(ctx, n);
			}
		}
	} else
		usid = uid;

	return usid;
}

static struct nfc_stat *find_usid_stat(pid_t pid, u32 usid)
{
	unsigned long flags;
	struct nfc_stat *entry;
	struct task_struct *tsk;

	if (usid >= 5000 && usid != special_usid) {
		spin_lock_irqsave(&nfc_lock, flags);
		list_for_each_entry(entry, &nfc_list, list) {
			if (entry->usid == usid) {
				spin_unlock_irqrestore(&nfc_lock, flags);
				return entry;
			}
		}
		spin_unlock_irqrestore(&nfc_lock, flags);
	} else {
		rcu_read_lock();
		tsk = find_task_by_vpid(pid);
		if (!tsk) {
			rcu_read_unlock();
			return ERR_PTR(-ESRCH);
		}
		rcu_read_unlock();

		spin_lock_irqsave(&nfc_lock, flags);
		list_for_each_entry(entry, &nfc_list, list) {
			if (!strcmp(tsk->comm, entry->comm)) {
				entry->pid = pid;
				spin_unlock_irqrestore(&nfc_lock, flags);
				return entry;
			}
		}
		spin_unlock_irqrestore(&nfc_lock, flags);
	}
	return NULL;
}

/* Create a new entry for tracking the specified usid. */
static struct nfc_stat *create_stat(pid_t pid, u32 usid, uid_t uid, u32 sid)
{
	unsigned long flags;
	struct task_struct *tsk;
	struct nfc_stat *new_usid;

	/* Create the pid stat struct and append it to the list. */
	new_usid = kzalloc(sizeof(struct nfc_stat), GFP_KERNEL);
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

	spin_lock_irqsave(&nfc_lock, flags);
	list_add_tail(&new_usid->list, &nfc_list);
	spin_unlock_irqrestore(&nfc_lock, flags);

	return new_usid;
}

int nfc_stat_se_activate(pid_t pid)
{
	struct nfc_stat *entry;
	struct task_struct *tsk;
	uid_t uid;
	u32 usid, sid;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		rcu_read_unlock();
		pr_err(NFC_STAT_PREFIX"No task(%d)\n", pid);
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
			pr_err(NFC_STAT_PREFIX"failed to allocate memory\n");
			return -ENOMEM;
		}
	}

	entry->se.active = true;
	entry->se.active_count++;
	entry->se.last_time = ktime_get();

	pr_info(NFC_STAT_PREFIX"%s: %s\n", __func__, current->comm);

	return 0;
}

int nfc_stat_se_deactivate(pid_t pid)
{
	struct nfc_stat *entry;
	struct task_struct *tsk;
	uid_t uid;
	u32 usid, sid;
	ktime_t duration;
	ktime_t now;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		rcu_read_unlock();
		pr_err(NFC_STAT_PREFIX"No task(%d)\n", pid);
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
			pr_err(NFC_STAT_PREFIX"failed to allocate memory\n");
			return -ENOMEM;
		}
	}

	entry->se.relax_count++;

	now = ktime_get();
	if (!entry->se.active) {
		entry->se.last_time = now;
		pr_err(NFC_STAT_PREFIX"%s: %s(%lu,%lu)\n", __func__,
			current->comm, entry->se.active_count, entry->se.relax_count);
		return 0;
	}

	entry->se.active = false;

	duration = ktime_sub(now, entry->se.last_time);
	entry->se.total_time = ktime_add(entry->se.total_time, duration);
	if (ktime_to_ns(duration) > ktime_to_ns(entry->se.max_time)) {
		entry->se.max_time = duration;
		entry->se.max_time_stamp = now;
	}
	entry->se.last_time = now;

	pr_info(NFC_STAT_PREFIX"%s: %s\n", __func__, current->comm);

	return 0;
}

static int nfc_stat_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	struct nfc_stat *entry;
	char *ctx = NULL;
	unsigned n;
	int err;
	ktime_t total_time;
	ktime_t max_time;
	ktime_t max_time_stamp;
	unsigned long active_count;
	unsigned long relax_count;
	ktime_t active_time;

	seq_printf(m, "name                                             "
			"active_count  relax_count   "
			"active_since  total_time    "
			"max_time      max_time_stamp  "
			"last_change\n");
	spin_lock_irqsave(&nfc_lock, flags);
	list_for_each_entry(entry, &nfc_list, list) {
		total_time = entry->se.total_time;
		max_time = entry->se.max_time;
		max_time_stamp = entry->se.max_time_stamp;
		active_count = entry->se.active_count;
		relax_count = entry->se.relax_count;

		if (entry->se.active) {
			ktime_t now = ktime_get();

			active_time = ktime_sub(now, entry->se.last_time);
			total_time = ktime_add(total_time, active_time);
			if (active_time.tv64 > max_time.tv64) {
				max_time = active_time;
				max_time_stamp = now;
			}
		} else {
			active_time = ktime_set(0, 0);
		}

		if (entry->usid >= 5000 && entry->usid != special_usid) {
			err = security_secid_to_secctx(entry->usid - 5001, &ctx, &n);
			if (!err) {
				seq_printf(m, "%-48s "
						"%-12lu  %-12lu  "
						"%-12lld  %-12lld  "
						"%-12lld  %-14lld  "
						"%-12lld\n",
						ctx,
						active_count, relax_count,
						ktime_to_ms(active_time), ktime_to_ms(total_time),
						ktime_to_ms(max_time), ktime_to_ms(max_time_stamp),
						ktime_to_ms(entry->se.last_time));
				security_release_secctx(ctx, n);
			}
		} else {
			seq_printf(m, "%-48s "
					"%-12lu  %-12lu  "
					"%-12lld  %-12lld  "
					"%-12lld  %-14lld  "
					"%-12lld\n",
					entry->comm,
					active_count, relax_count,
					ktime_to_ms(active_time), ktime_to_ms(total_time),
					ktime_to_ms(max_time), ktime_to_ms(max_time_stamp),
					ktime_to_ms(entry->se.last_time));
		}
	}
	spin_unlock_irqrestore(&nfc_lock, flags);

	return 0;
}

static int nfc_stat_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, nfc_stat_show, NULL);
}

static const struct file_operations nfc_stat_fops = {
	.open       = nfc_stat_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int nfc_stat_pm_suspend_prepare_cb(ktime_t ktime)
{
	return 0;
}

static int nfc_stat_pm_post_suspend_cb(ktime_t ktime)
{
	return 0;
}

static int nfc_stat_pm_notifier(struct notifier_block *nb,
		unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		nfc_stat_pm_suspend_prepare_cb(resume_time);
		break;
	case PM_POST_SUSPEND:
		nfc_stat_pm_post_suspend_cb(resume_time);
		resume_time = ktime_get();
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block nfc_stat_notifier_block = {
	.notifier_call = nfc_stat_pm_notifier,
};

static int __init nfc_stat_init(void)
{
	struct dentry *root;

	root = debugfs_create_dir("nfc_stat", NULL);
	if (!root) {
		pr_err(NFC_STAT_PREFIX"failed to create nfc_stat debugfs directory\n");
		return -ENOMEM;
	}

	/* Make interface to read the tcp traffic statistic */
	if (!debugfs_create_file("stat", 0660, root, NULL, &nfc_stat_fops))
		goto error_debugfs;

	if (register_pm_notifier(&nfc_stat_notifier_block))
		goto error_debugfs;

	special_usid = -1;

	return 0;

error_debugfs:
	debugfs_remove_recursive(root);

	return -1;
}

late_initcall(nfc_stat_init);
