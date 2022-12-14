/*
 * To log which wakeup source prevent AP sleep.
 *
 * Copyright (C) 2015 SAMSUNG, Inc.
 * Sanghyeon Lee <sirano06.lee@samsung.com>
 * Hunsup Jung <hunsup.jung@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/suspend.h>
#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/security.h>

#include <linux/power/slave_wakelock.h>
#ifdef CONFIG_SLEEP_MONITOR
#include <linux/power/sleep_monitor.h>
#endif
#ifdef CONFIG_ENERGY_MONITOR
#include <linux/power/energy_monitor.h>
#endif

#define NONLOCK		0
#define MAINLOCK	BIT(0)
#define DIMLOCK		BIT(1)
#define DISPLOCK	BIT(2)

#define INTERNAL_LOCK_BASE 100000
static const char internal_lock[30][20] = {
	"", /* 100000 */
	"i_alpm", "i_bat", "i_batfull", "i_booting", "i_dump",     /* 100001~100005 */
	"i_hdmi", "i_ode", "i_popup", "i_sounddock", "i_time",      /* 100006~100010 */
	"i_usb", "i_poweroff", "i_suspend", "i_cooldown", "i_lowbat",    /* 100011~100015 */
	"i_cradle", "i_earjack", "i_powerkey", "i_pm", "i_hallic",      /* 100016~100020 */
};

struct slave_wakelock {
	struct list_head entry;
	spinlock_t lock;

	unsigned int pid;
	u32 usid;
	char name[SLWL_NAME_LENGTH];
	u8 lock_flag;

	ktime_t max_time;
	ktime_t max_time_stamp;
	ktime_t last_time;
	ktime_t lock_time;
	ktime_t last_prev_time;
	ktime_t total_prev_time;

#ifdef CONFIG_SLEEP_MONITOR
	ktime_t slp_mon_time;
#endif
#ifdef CONFIG_ENERGY_MONITOR
	ktime_t energy_mon_time;
	ktime_t energy_mon_dump_time;
	ktime_t energy_mon_check_time;
#endif

	unsigned long active_count;
};

static LIST_HEAD(slwl_list);
static struct dentry *slave_wakelock_dentry;
static int special_usid;

#ifdef CONFIG_ENERGY_MONITOR
static ktime_t booting_time;

static int slave_wakelock_emon_is_whitelist(
	struct slave_wakelock *iter)
{
	if (iter->usid >= 5000) {
		if (strncmp(iter->name, "bixby-wakeup-s",
			sizeof("bixby-wakeup-s")) == 0)
			return 1;
	}

	return 0;
}

/* Initial energy_monitor info */
static void init_energy_mon_time(void)
{
	struct slave_wakelock *iter;
	ktime_t ktime_zero = ktime_set(0, 0);

	list_for_each_entry_rcu(iter, &slwl_list, entry) {
		iter->energy_mon_time = ktime_zero;
		iter->energy_mon_dump_time = ktime_zero;
		iter->energy_mon_check_time = ktime_get();
	}
}

/* Insert energy_mon_time */
static int insert_energy_mon_time(int type,
	struct emon_slave_wakelock *slwl, struct slave_wakelock *iter, int size)
{
	ktime_t e_prev_time;
	ktime_t ktime_zero = ktime_set(0, 0);
	int i = 0, j = 0;

	if (type != ENERGY_MON_TYPE_DUMP)
		e_prev_time = iter->energy_mon_time;
	else
		e_prev_time = iter->energy_mon_dump_time;

	for (i = 0; i < size; i++) {
		/* Insertion in empty space */
		if (ktime_to_ms(slwl[i].prevent_time) == ktime_to_ms(ktime_zero)) {
			slwl[i].usid = iter->usid;
			slwl[i].special_usid = special_usid;
			strncpy(slwl[i].slwl_name, iter->name, SLWL_NAME_LENGTH);
			slwl[i].slwl_name[SLWL_NAME_LENGTH - 1] = 0;
			slwl[i].prevent_time = e_prev_time;
			break;
		}

		/* Insertion in order */
		if (ktime_to_ms(e_prev_time) > ktime_to_ms(slwl[i].prevent_time)) {
			for (j = SLWL_ARRAY_SIZE - 1; j > i; j--) {
				if (ktime_to_ms(slwl[j - 1].prevent_time) == ktime_to_ms(ktime_zero))
					continue;
				slwl[j].usid = slwl[j - 1].usid;
				slwl[j].special_usid = special_usid;
				strncpy(slwl[j].slwl_name, slwl[j - 1].slwl_name, SLWL_NAME_LENGTH);
				slwl[j].slwl_name[SLWL_NAME_LENGTH - 1] = 0;
				slwl[j].prevent_time = slwl[j - 1].prevent_time;
			}
			slwl[i].usid = iter->usid;
			slwl[i].special_usid = special_usid;
			strncpy(slwl[i].slwl_name, iter->name, SLWL_NAME_LENGTH);
			slwl[i].slwl_name[SLWL_NAME_LENGTH - 1] = 0;
			slwl[i].prevent_time = e_prev_time;
			break;
		}
	}

	return 0;
}

/* Sort energy_mon_time */
static int sort_energy_mon_time(int type,
		struct emon_slave_wakelock *slwl, int size)
{
	struct slave_wakelock *iter;
	int num = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slwl_list, entry) {
		if (type != ENERGY_MON_TYPE_DUMP && ktime_to_ms(iter->energy_mon_time) <= 0)
			continue;
		if (type == ENERGY_MON_TYPE_DUMP && ktime_to_ms(iter->energy_mon_dump_time) <= 0)
			continue;
		if (slave_wakelock_emon_is_whitelist(iter))
			continue;

		insert_energy_mon_time(type, slwl, iter, size);
		num++;
	}
	rcu_read_unlock();

	return num;
}

/* Energy monitor call this function to get slave wakelock has largest energy_mon_time */
void slave_wakelock_get_large_prevent_time(int type,
		struct emon_slave_wakelock *slwl, int size)
{
	struct slave_wakelock *iter;
	ktime_t active_time = ktime_set(0, 0);
	int num = 0;

	memset(slwl, 0, sizeof(*slwl) * size);

	list_for_each_entry_rcu(iter, &slwl_list, entry) {
		if (iter->lock_flag == NONLOCK)
			continue;

		if (ktime_to_ms(iter->lock_time) > ktime_to_ms(iter->energy_mon_check_time))
			active_time = ktime_sub(ktime_get(), iter->lock_time);
		else
			active_time = ktime_sub(ktime_get(), iter->energy_mon_check_time);

		if (type != ENERGY_MON_TYPE_DUMP)
			iter->energy_mon_time = ktime_add(iter->energy_mon_time, active_time);
		else
			iter->energy_mon_dump_time = ktime_add(iter->energy_mon_time, active_time);
	}

	num = sort_energy_mon_time(type, slwl, size);

	if (type != ENERGY_MON_TYPE_DUMP)
		init_energy_mon_time();
}
#endif

#ifdef CONFIG_SLEEP_MONITOR
static int s_slwl_num;
static int s_slwl_idx[SLWL_ARRAY_SIZE];
static char s_slwl_name[SLWL_ARRAY_SIZE][SLWL_NAME_LENGTH];
static ktime_t s_slwl_prev_time[SLWL_ARRAY_SIZE];

/* Initial sleep_monitor info */
static void init_slp_mon_time(void)
{
	struct slave_wakelock *iter;
	ktime_t ktime_zero = ktime_set(0, 0);
	int i = 0;

	for(i = 0; i < SLWL_ARRAY_SIZE; i++) {
		memset(s_slwl_name[i], 0, SLWL_ARRAY_SIZE);
		s_slwl_prev_time[i] = ktime_set(0, 0);
		s_slwl_idx[i] = 0;
	}
	s_slwl_num = 0;

	list_for_each_entry_rcu(iter, &slwl_list, entry)
		iter->slp_mon_time = ktime_zero;
}

/* Sort slp_mon_time */
static void slp_mon_sort_time(void)
{
	struct slave_wakelock *iter;
	int i = 0, j = 0, idx = 0;
	ktime_t ktime_zero = ktime_set(0, 0);

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slwl_list, entry) {

		if (ktime_to_ms(iter->slp_mon_time) <= 0) {
			idx++;
			continue;
		}

		for (i = 0; i < SLWL_ARRAY_SIZE; i++) {
			/* Insertion in empty space */
			if (ktime_to_ms(s_slwl_prev_time[i]) == ktime_to_ms(ktime_zero)) {
				s_slwl_idx[i] = idx;
				strncpy(s_slwl_name[i], iter->name, SLWL_NAME_LENGTH);
				s_slwl_name[i][SLWL_NAME_LENGTH - 1] = 0;
				s_slwl_prev_time[i] = iter->slp_mon_time;
				s_slwl_num++;
				break;
			}

			/* Insertion in order */
			if (ktime_to_ms(iter->slp_mon_time) > ktime_to_ms(s_slwl_prev_time[i])) {
				for (j = SLWL_ARRAY_SIZE - 1; j > i; j--) {
					if (ktime_to_ms(s_slwl_prev_time[j - 1]) == ktime_to_ms(ktime_zero))
						continue;
					s_slwl_idx[j] = s_slwl_idx[j - 1];
					strncpy(s_slwl_name[j], s_slwl_name[j - 1], SLWL_NAME_LENGTH);
					s_slwl_prev_time[j] = s_slwl_prev_time[j - 1];
				}
				s_slwl_idx[i] = idx;
				s_slwl_prev_time[i] = iter->slp_mon_time;
				strncpy(s_slwl_name[i], iter->name, SLWL_NAME_LENGTH);
				if (s_slwl_num < 4)
					s_slwl_num++;
				break;
			}
		}
		idx++;
	}
	rcu_read_unlock();

	pr_cont("[SLWL] slave_wakelock_list");
	for (i = 0; i < s_slwl_num; i++)
		pr_cont("%s(%lld)/", s_slwl_name[i], ktime_to_ms(s_slwl_prev_time[i]));
	pr_info("");
}

/* Sleep monitor call this function to get slave wakelock has largest slp_mon_time */
static int slp_mon_slwl_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slave_wakelock *iter;
	int i = 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND)
		slp_mon_sort_time();

	if (s_slwl_num < 1)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (s_slwl_idx[0] == i) {
				*raw_val |= i << SLWL_IDX_BIT;
				if (ktime_to_ms(iter->slp_mon_time) > SLWL_PREVENT_TIME_MAX)
					iter->slp_mon_time = ktime_set(0, SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->slp_mon_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (s_slwl_num == 1)
		init_slp_mon_time();

	return 1;
}

static int slp_mon_slwl1_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slave_wakelock *iter;
	int i = 0;

	if (s_slwl_num < 2)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (s_slwl_idx[1] == i) {
				*raw_val |= i << SLWL_IDX_BIT;
				if (ktime_to_ms(iter->slp_mon_time) > SLWL_PREVENT_TIME_MAX)
					iter->slp_mon_time = ktime_set(0, SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->slp_mon_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (s_slwl_num == 2)
		init_slp_mon_time();

	return 1;
}

static int slp_mon_slwl2_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slave_wakelock *iter;
	int i = 0;

	if (s_slwl_num < 3)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (s_slwl_idx[2] == i) {
				*raw_val |= i << SLWL_IDX_BIT;
				if (ktime_to_ms(iter->slp_mon_time) > SLWL_PREVENT_TIME_MAX)
					iter->slp_mon_time = ktime_set(0, SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->slp_mon_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (s_slwl_num == 3)
		init_slp_mon_time();

	return 1;
}

static int slp_mon_slwl3_cb(void *priv, unsigned int *raw_val,
								int check_level, int caller_type)
{
	struct slave_wakelock *iter;
	int i = 0;

	if (s_slwl_num < 4)
		return 0;

	if (caller_type == SLEEP_MONITOR_CALL_SUSPEND) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (s_slwl_idx[3] == i) {
				*raw_val |= i << SLWL_IDX_BIT;
				if (ktime_to_ms(iter->slp_mon_time) > SLWL_PREVENT_TIME_MAX)
					iter->slp_mon_time = ktime_set(0, SLWL_PREVENT_TIME_MAX * 1000);
				*raw_val |= ktime_to_ms(iter->slp_mon_time);
				break;
			}
			i++;
		}
		rcu_read_unlock();
	}

	if (s_slwl_num == 4)
		init_slp_mon_time();

	return 1;
}

static struct sleep_monitor_ops slp_mon_slwl_dev = {
	.read_cb_func = slp_mon_slwl_cb,
};

static struct sleep_monitor_ops slp_mon_slwl_dev1 = {
	.read_cb_func = slp_mon_slwl1_cb,
};

static struct sleep_monitor_ops slp_mon_slwl_dev2 = {
	.read_cb_func = slp_mon_slwl2_cb,
};

static struct sleep_monitor_ops slp_mon_slwl_dev3 = {
	.read_cb_func = slp_mon_slwl3_cb,
};

/* Print slave wake lock list for sleep monitor */
static ssize_t slp_mon_read_slwl_list(struct file *file,
        char __user *buffer, size_t count, loff_t *ppos)
{
	struct slave_wakelock *iter;
	char *buf = NULL;
	ssize_t ret = 0;
	int i = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, PAGE_SIZE);

	if (*ppos == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%08x]%s", special_key,
						get_type_marker(SLEEP_MONITOR_CALL_SLWL_LIST));
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%s/", iter->name);
			i++;
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret,"\n");
		rcu_read_unlock();
	}

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret)) {
			kfree(buf);
			return -EFAULT;
		}
		*ppos += ret;
	}

	kfree(buf);
	return ret;
}

static int slp_mon_slwl_notifier(struct notifier_block *nb,
								unsigned long event, void *dummy)
{
	switch (event) {
		case PM_POST_SUSPEND:
			init_slp_mon_time();
			break;
		default:
			break;
	}
	return 0;
}

static struct notifier_block slp_mon_slwl_notifier_ops = {
	.notifier_call =	slp_mon_slwl_notifier,
};

static const struct file_operations slp_mon_slwl_list_ops = {
	.read =	slp_mon_read_slwl_list,
};

static void register_sleep_monitor_cb(void)
{
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev, SLEEP_MONITOR_SLWL);
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev1, SLEEP_MONITOR_SLWL1);
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev2, SLEEP_MONITOR_SLWL2);
	sleep_monitor_register_ops(NULL, &slp_mon_slwl_dev3, SLEEP_MONITOR_SLWL3);

	if (slp_mon_d) {
		struct dentry *d = debugfs_create_file("slp_mon_slwl", S_IRUSR,
								slp_mon_d, NULL, &slp_mon_slwl_list_ops);
		if (!d)
			pr_err("SLWL: %s: debugfs_create_file, error\n", __func__);
	} else
		pr_err("SLWL: %s: dentry is not defined\n", __func__);

	if (register_pm_notifier(&slp_mon_slwl_notifier_ops))
		pr_err("SLWL: %s: error register_pm_notifier\n", __func__);
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
					pr_info("[SLWL] %s: found special_usid: %d\n",
						__func__, special_usid);
				}
				security_release_secctx(ctx, n);
			}
		}
	} else
		usid = uid;

	return usid;
}

/* Initial slave wakelock list */
static void init_slave_wakelock(struct slave_wakelock *slwl)
{
	spin_lock_init(&slwl->lock);

	slwl->pid = 0;
	slwl->usid = 0;
	memset(slwl->name, 0, SLWL_NAME_LENGTH);
	slwl->lock_flag = 0;

	slwl->max_time = ktime_set(0, 0);
	slwl->max_time_stamp = ktime_set(0, 0);
	slwl->last_time = ktime_set(0, 0);
	slwl->lock_time = ktime_set(0, 0);
	slwl->last_prev_time = ktime_set(0, 0);
	slwl->total_prev_time = ktime_set(0, 0);

#ifdef CONFIG_SLEEP_MONITOR
	slwl->slp_mon_time = ktime_set(0, 0);
#endif
#ifdef CONFIG_ENERGY_MONITOR
	slwl->energy_mon_time = ktime_set(0, 0);
	slwl->energy_mon_dump_time = ktime_set(0, 0);
	slwl->energy_mon_check_time = booting_time;
#endif

	slwl->active_count= 0;
}

/* If lock_flag is 0, reset flag and time */
static int update_slave_wakelock(
	const char *name, unsigned int pid, u32 usid, u8 lock_flag)
{
	struct slave_wakelock *iter;
	ktime_t ktime_zero = ktime_set(0, 0);
	ktime_t duration;
	ktime_t now;
	int exist = 0;
#ifdef CONFIG_ENERGY_MONITOR
	ktime_t energy_mon_time;
#endif

	pr_debug("[SLWL] %s: name:%s, pid: %d, usid: %d, lock_flag: %u\n",
				__func__, name, pid, usid, lock_flag);

	/* Searching by name or pid */
	rcu_read_lock();
	if (usid >= 5000 && usid != special_usid) {
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (iter->usid == usid) {
				exist = 1;
				break;
			}
		}
	} else if (!strncmp(name, "no_task", strlen("no_task"))) {
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (iter->pid == pid) {
				exist = 1;
				break;
			}
		}
	} else {
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (!strncmp(iter->name, name, SLWL_NAME_LENGTH - 1)) {
				exist = 1;
				break;
			}
		}
	}
	rcu_read_unlock();

	if (!exist) {
		pr_err("[SLWL] %s: Can't find (%s:%u:%u)\n", __func__, name, pid, usid);
		return -ENODATA;
	}

	/* Update flag and time */
	rcu_read_lock();
	iter->lock_flag &= (~lock_flag);
	if(iter->lock_flag == NONLOCK &&
	   ktime_to_ms(iter->lock_time) != ktime_to_ms(ktime_zero)) {
	   	now = ktime_get();
		duration = ktime_sub(now, iter->lock_time);
		if (ktime_to_ns(duration) > ktime_to_ns(iter->max_time)) {
			iter->max_time = duration;
			iter->max_time_stamp = now;
		}
		iter->last_time = now;
		iter->last_prev_time = duration;
		iter->total_prev_time = ktime_add(iter->total_prev_time, iter->last_prev_time);
#ifdef CONFIG_SLEEP_MONITOR
		iter->slp_mon_time = ktime_add(iter->slp_mon_time, iter->last_prev_time);
#endif
#ifdef CONFIG_ENERGY_MONITOR
		energy_mon_time = ktime_set(0, 0);

		if (ktime_to_ms(iter->lock_time) > ktime_to_ms(iter->energy_mon_check_time))
			energy_mon_time = ktime_sub(now, iter->lock_time);
		else
			energy_mon_time = ktime_sub(now, iter->energy_mon_check_time);

		iter->energy_mon_time = ktime_add(iter->energy_mon_time, energy_mon_time);
		iter->energy_mon_dump_time = iter->energy_mon_time;
#endif
		iter->lock_time = ktime_zero;
	}
	rcu_read_unlock();

	return 0;
}

/* Add slave wakelock to list */
int add_slave_wakelock(
	const char *name, unsigned int pid, u32 usid, u8 lock_flag)
{
	struct slave_wakelock *slwl, *iter;
	ktime_t ktime_zero = ktime_set(0, 0);

	pr_debug("[SLWL] %s: name:%s, pid: %d, usid: %d, lock_flag: %u\n",
				__func__, name, pid, usid, lock_flag);

	/* Already added lock */
	if (usid >= 5000 && usid != special_usid) {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (iter->usid == usid) {
				iter->pid = pid;
				iter->lock_flag |= lock_flag;
				if (ktime_to_ms(iter->lock_time) == ktime_to_ms(ktime_zero)) {
					iter->active_count++;
					iter->last_time = ktime_get();
					iter->lock_time = iter->last_time;
				}
				rcu_read_unlock();
				return 0;
			}
		}
		rcu_read_unlock();
	} else {
		rcu_read_lock();
		list_for_each_entry_rcu(iter, &slwl_list, entry) {
			if (!strncmp(iter->name, name, SLWL_NAME_LENGTH - 1)) {
				iter->pid = pid;
				iter->lock_flag |= lock_flag;
				if (ktime_to_ms(iter->lock_time) == ktime_to_ms(ktime_zero)) {
					iter->active_count++;
					iter->last_time = ktime_get();
					iter->lock_time = iter->last_time;
				}
				rcu_read_unlock();
				return 0;
			}
		}
		rcu_read_unlock();
	}

	/* Add new instance */
	slwl = kmalloc(sizeof(struct slave_wakelock), GFP_KERNEL);
	if (!slwl)
		return -ENOMEM;

	init_slave_wakelock(slwl);

	slwl->pid = pid;
	slwl->usid = usid;
	strncpy(slwl->name, name, SLWL_NAME_LENGTH - 1);
	slwl->name[SLWL_NAME_LENGTH - 1] = 0;

	slwl->lock_flag |= lock_flag;
	slwl->active_count++;
	slwl->last_time = ktime_get();
	slwl->lock_time = slwl->last_time;

	rcu_read_lock();
	list_add_tail(&slwl->entry, &slwl_list);
	rcu_read_unlock();

	return 0;
}

static int print_slave_wakelock(struct seq_file *m,
		struct slave_wakelock *slwl, unsigned int idx)
{
	unsigned long flags;
	unsigned long active_count;
	int ret = 1;
	ktime_t now;
	ktime_t max_time, max_time_stamp;
	ktime_t active_since = ktime_set(0, 0);
	ktime_t total_time = ktime_set(0, 0);
	ktime_t ktime_zero = ktime_set(0, 0);
	char *ctx = NULL;
	unsigned n;
	int err;

	now = ktime_get();
	if (ktime_to_ms(slwl->lock_time) != ktime_to_ms(ktime_zero))
		active_since = ktime_sub(now, slwl->lock_time);
	total_time = ktime_add(slwl->total_prev_time, active_since);
	active_count = slwl->active_count;

	max_time = slwl->max_time;
	max_time_stamp = slwl->max_time_stamp;
	if (ktime_compare(active_since, max_time) > 0) {
		max_time = active_since;
		max_time_stamp = now;
	}

	if (slwl->usid >= 5000 && slwl->usid != special_usid) {
		err = security_secid_to_secctx(slwl->usid - 5001, &ctx, &n);
		if (!err) {
			spin_lock_irqsave(&slwl->lock, flags);
			seq_printf(m, "%-6d%-64s%-8u  %-7u"
						"%-12lu  %-12lld  0x%-9x  "
						"%-12lld    %-12lld  "
						"%-12lld  %-12lld    "
						"%-12lld  ",
						idx, ctx, slwl->pid, slwl->usid,
						active_count, ktime_to_ms(active_since), slwl->lock_flag,
						ktime_to_ms(slwl->last_prev_time), ktime_to_ms(total_time),
						ktime_to_ms(max_time), ktime_to_ms(max_time_stamp),
						ktime_to_ms(slwl->last_time));
#ifdef CONFIG_SLEEP_MONITOR
			seq_printf(m, "%12lld	 ",	ktime_to_ms(slwl->slp_mon_time));
#endif
#ifdef CONFIG_ENERGY_MONITOR
			seq_printf(m, "%-15lld %-21lld",
							ktime_to_ms(slwl->energy_mon_time),
							ktime_to_ms(slwl->energy_mon_check_time));
#endif
			seq_printf(m, "\n");
			spin_unlock_irqrestore(&slwl->lock, flags);
			security_release_secctx(ctx, n);

			return ret;
		}
	}

	spin_lock_irqsave(&slwl->lock, flags);
	seq_printf(m, "%-6d%-64s%-8u  %-7u"
				"%-12lu  %-12lld  0x%-9x  "
				"%-12lld    %-12lld  "
				"%-12lld  %-12lld    "
				"%-12lld  ",
				idx, slwl->name, slwl->pid, slwl->usid,
				active_count, ktime_to_ms(active_since), slwl->lock_flag,
				ktime_to_ms(slwl->last_prev_time), ktime_to_ms(total_time),
				ktime_to_ms(max_time), ktime_to_ms(max_time_stamp),
				ktime_to_ms(slwl->last_time));
#ifdef CONFIG_SLEEP_MONITOR
	seq_printf(m, "%12lld    ", ktime_to_ms(slwl->slp_mon_time));
#endif
#ifdef CONFIG_ENERGY_MONITOR
	seq_printf(m, "%-15lld %-21lld",
					ktime_to_ms(slwl->energy_mon_time),
					ktime_to_ms(slwl->energy_mon_check_time));
#endif
	seq_printf(m, "\n");
	spin_unlock_irqrestore(&slwl->lock, flags);

	return ret;
}

static int slave_wakelock_show(struct seq_file *m, void *unused)
{
	struct slave_wakelock *iter;
	unsigned int idx = 1;

	seq_puts(m, "idx   name                                                            "
				"pid       usid   "
				"active_count  active_since  active_type  "
				"last_prev_time  total_time    "
				"max_time      max_time_stamp  "
				"last_change   ");
#ifdef CONFIG_SLEEP_MONITOR
	seq_puts(m, "slp_mon_time	 ");
#endif
#ifdef CONFIG_ENERGY_MONITOR
	seq_puts(m, "energy_mon_time energy_mon_check_time");
#endif
	seq_puts(m, "\n");

	rcu_read_lock();
	list_for_each_entry_rcu(iter, &slwl_list, entry)
		print_slave_wakelock(m, iter, idx++);
	rcu_read_unlock();

	return 0;
}

static int slave_wakelock_open(struct inode *inode, struct file *file)
{
	return single_open(file, slave_wakelock_show, NULL);
}

static const struct  file_operations slave_wakelock_fops = {
	.owner = THIS_MODULE,
	.open = slave_wakelock_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int slave_wake_lock(const char *buf)
{
	struct task_struct *task;
	const char *str = buf;
	size_t len;
	int pid = 0;
	uid_t uid;
	u32 usid = 0, sid = 0;
	char name[SLWL_NAME_LENGTH] = {0,};
	u8 lock_flag = 0;
	int ret = 0;

	pr_debug("SLWL: %s: buf: %s\n", __func__, buf);

	while (*str && !isspace(*str))
		str++;

	/* If first character is '0' or space, return -EINVAL */
	len = str - buf;
	if (!len) {
		pr_err("[SLWL] %s: invalid input %s\n", __func__, buf);
		return -EINVAL;
	}

	/* Get lock type */
	if (!strncmp(buf, "displock", len))
		lock_flag |= DISPLOCK;
	else if(!strncmp(buf, "dimlock", len))
		lock_flag |= DIMLOCK;
	else if(!strncmp(buf, "mainlock", len))
		lock_flag |= MAINLOCK;
	else {
		pr_err("[SLWL] %s: invalid lock type %s\n", __func__, buf);
		return -EINVAL;
	}

	/* Get pid */
	if (*str && *str != '\n') {
		/* Find out if there's a pid string appended. */
		ret = kstrtos32(skip_spaces(str), 10, &pid);
		if (ret) {
			pr_err("[SLWL] %s: invalid pid %s\n", __func__, str);
			return -EINVAL;
		}
	}
	if (pid == 0) {
		/* If there's no pid, return -EINVAL */
		pr_err("[SLWL] %s: invalid pid %d\n", __func__, pid);
		return -EINVAL;
	}

	/* Get name */
	rcu_read_lock();
	task = find_task_by_vpid(pid);
	rcu_read_unlock();
	if (task) {
		uid = from_kuid_munged(current_user_ns(), task_uid(task));
		security_task_getsecid(task, &sid);
		usid = find_usid(uid, sid);
		strncpy(name, task->comm, SLWL_NAME_LENGTH - 1);
	} else if(pid > 100000)
		strncpy(name, internal_lock[pid - INTERNAL_LOCK_BASE], SLWL_NAME_LENGTH - 1);
	else
		strncpy(name, "no_task", strlen("no_task") + 1);

	/* Add slave wakelock to list */
	add_slave_wakelock(name, pid, usid, lock_flag);

	return 0;
}

int slave_wake_unlock(const char *buf)
{
	struct task_struct *task;
	const char *str = buf;
	size_t len;
	int pid = 0;
	uid_t uid;
	u32 usid = 0, sid = 0;
	char name[SLWL_NAME_LENGTH] = {0,};
	u8 lock_flag = 0;
	int ret = 0;

	pr_debug("SLWL: %s: buf: %s\n", __func__, buf);

	while (*str && !isspace(*str))
		str++;

	/* If first character is '0' or space, return -EINVAL */
	len = str - buf;
	if (!len) {
		pr_err("[SLWL] %s: invalid input %s\n", __func__, buf);
		return -EINVAL;
	}

	/* Get lock type */
	if (!strncmp(buf, "displock", len))
		lock_flag |= DISPLOCK;
	else if(!strncmp(buf, "dimlock", len))
		lock_flag |= DIMLOCK;
	else if(!strncmp(buf, "mainlock", len))
		lock_flag |= MAINLOCK;
	else {
		pr_err("[SLWL] %s: invalid lock type %s\n", __func__, buf);
		return -EINVAL;
	}

	/* Get pid */
	if (*str && *str != '\n') {
		/* Find out if there's a pid string appended. */
		ret = kstrtos32(skip_spaces(str), 10, &pid);
		if (ret) {
			pr_err("[SLWL] %s: invalid pid %s\n", __func__, str);
			return -EINVAL;
		}
	}
	if (pid == 0) {
		/* If there's no pid, return -EINVAL */
		pr_err("[SLWL] %s: invalid pid %d\n", __func__, pid);
		return -EINVAL;
	}

	/* Get name */
	rcu_read_lock();
	task = find_task_by_vpid(pid);
	rcu_read_unlock();
	if (task) {
		uid = from_kuid_munged(current_user_ns(), task_uid(task));
		security_task_getsecid(task, &sid);
		usid = find_usid(uid, sid);
		strncpy(name, task->comm, SLWL_NAME_LENGTH - 1);
	} else
		strncpy(name, "no_task", strlen("no_task") + 1);

	/* Update slave wakelock info */
	update_slave_wakelock(name, pid, usid, lock_flag);

	return 0;
}

static int slave_wakelock_init(void)
{
	pr_debug("%s\n", __func__);

	slave_wakelock_dentry = debugfs_create_file("slave_wakelocks",
		S_IRUGO, NULL, NULL, &slave_wakelock_fops);

	special_usid = -1;

#ifdef CONFIG_SLEEP_MONITOR
	register_sleep_monitor_cb();
#endif
#ifdef CONFIG_ENERGY_MONITOR
	booting_time = ktime_get();
#endif

	return 0;
}

static void slave_wakelock_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(slave_wakelock_init);
module_exit(slave_wakelock_exit);
