/*
 *  Copyright (C)  2011 Samsung Electronics co. ltd
 *    Yong-U Baek <yu.baek@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/pm_qos.h>
#include <linux/vmalloc.h>
#include <linux/cpuidle.h>
#include <linux/devfreq.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#define CONFIG_LOAD_ANALYZER_INTERNAL	1
#include <linux/load_analyzer.h>

#if defined(CONFIG_SLP_BUSY_LEVEL)
#include <linux/mman.h>
#endif

#if defined(CONFIG_SLP_CHECK_BUS_LOAD) && defined(CONFIG_CLK_MON)
#include <linux/clk_mon.h>
#endif

#include "load_analyzer_util.c"
#include "load_analyzer_cpu.c"
#include "load_analyzer_exynos9110.c"

#if defined (CONFIG_SLP_PERFORMANCE_MONITOR)
#include "load_analyzer_performance.c"
#endif

#if defined (CONFIG_CHECK_WORK_HISTORY)
#include "load_analyzer_workqueue.c"
#endif

#if defined (CONFIG_SLP_CHECK_PHY_ADDR)
#include "load_analyzer_addr.c"
#endif

#if defined (CONFIG_SLP_MINI_TRACER)
#include "load_analyzer_mini.c"
#endif

#if defined (CONFIG_SLP_CPU_TESTER)
#include "load_analyzer_cputester.c"
#endif

#if defined (CONFIG_SLP_INPUT_REC)
#include "load_analyzer_input.c"
#endif

#if defined(CONFIG_CHECK_TIME_IN_STATE)
#include "load_analyzer_time_state.c"
#endif

enum {
	NORMAL_MODE,
	RECOVERY_MODE,
};

static int before_cpu_task_history_num;
static int before_cpu_load_history_num;

/* PmQoS request for enhance wristup wake up performance */
int wristup_work_initialized = 0;
static struct pm_qos_request wristup_qos_request;
struct wristup_cpu_pmqos_work_s wristup_cpu_booster_work;

#if defined(CONFIG_SLP_INPUT_REC)
static int before_input_rec_history_num;
#endif

#if defined(CONFIG_SLP_BUSY_LEVEL)
extern int touch_event_register_notifier(struct notifier_block *nb, unsigned int list);
extern int touch_event_unregister_notifier(struct notifier_block *nb, unsigned int list);
int load_analyzer_init_done = 0;
#endif

/* for gem alloc info */
#if defined(CONFIG_GEM_ALLOC_DEBUGS)
#define MAX_GEM_TASK 128
#define GEM_ALLOC_THRESHOLD 150

static DEFINE_MUTEX(gem_mutex);
struct workqueue_struct *gem_alloc_wq;
gem_alloc_work_t *w_data = NULL;
static gem_alloc_t gem_alloc_arr[MAX_GEM_TASK];
static int gem_alloc_pos = -1;
char gem_alloc_buf[128];
#endif

static int cpu_load_analyzer_init(int mode)
{
#if defined(CONFIG_SLP_BUSY_LEVEL)
	high_cpu_load_boot_completed = jiffies + EXPECTED_CPU_BOOT_TIME;
#endif

	if (mode == RECOVERY_MODE) {
		cpu_task_history_num = before_cpu_task_history_num;
		cpu_load_history_num = before_cpu_load_history_num;
#if defined(CONFIG_SLP_INPUT_REC)
		input_rec_history_num = before_input_rec_history_num;
#endif
	}

	cpu_load_freq_history = vmalloc(CPU_LOAD_FREQ_HISTORY_SIZE);
	if (cpu_load_freq_history == NULL)
		return -ENOMEM;

	cpu_load_freq_history_view = vmalloc(CPU_LOAD_FREQ_HISTORY_SIZE);
	if (cpu_load_freq_history_view == NULL)
		return -ENOMEM;

	cpu_task_history = vmalloc(CPU_TASK_HISTORY_SIZE);
	if (cpu_task_history == NULL)
		return -ENOMEM;

	cpu_task_history_view = vmalloc(CPU_TASK_HISTORY_SIZE);
	if (cpu_task_history_view == NULL)
		return -ENOMEM;

#if defined (CONFIG_CHECK_WORK_HISTORY)
	cpu_work_history =	vmalloc(CPU_WORK_HISTORY_SIZE);
	if (cpu_work_history == NULL)
		return -ENOMEM;

	cpu_work_history_view = vmalloc(CPU_WORK_HISTORY_SIZE);
	if (cpu_work_history_view == NULL)
		return -ENOMEM;
#endif

#if defined (CONFIG_SLP_INPUT_REC)
	input_rec_history =	vmalloc(INPUT_REC_HISTORY_SIZE);
	if (input_rec_history == NULL)
		return -ENOMEM;

	input_rec_history_view = vmalloc(INPUT_REC_HISTORY_SIZE);
	if (input_rec_history_view == NULL)
		return -ENOMEM;
#endif

	if (store_killed_init() != 0)
		return -ENOMEM;

	memset(cpu_load_freq_history, 0, CPU_LOAD_FREQ_HISTORY_SIZE);
	memset(cpu_load_freq_history_view, 0, CPU_LOAD_FREQ_HISTORY_SIZE);
	memset(cpu_task_history, 0, CPU_TASK_HISTORY_SIZE);
	memset(cpu_task_history_view, 0, CPU_TASK_HISTORY_SIZE);

#if defined (CONFIG_CHECK_WORK_HISTORY)
	memset(cpu_work_history, 0, CPU_WORK_HISTORY_SIZE);
	memset(cpu_work_history_view, 0, CPU_WORK_HISTORY_SIZE);
#endif
	store_killed_memset();

#if 0
	pr_info("LA size %d %d %d sum %d", CPU_LOAD_FREQ_HISTORY_SIZE,
		CPU_TASK_HISTORY_SIZE, CPU_WORK_HISTORY_SIZE,
		(CPU_LOAD_FREQ_HISTORY_SIZE \
		+ CPU_TASK_HISTORY_SIZE + CPU_WORK_HISTORY_SIZE) * 2);
#endif

#if defined (CONFIG_CHECK_WORK_HISTORY)
	cpu_work_history_onoff = 1;
#endif

#if defined (CONFIG_SLP_INPUT_REC)
	input_rec_history_onoff = 1;
#endif
	cpu_task_history_onoff = 1;

	saved_task_info_onoff = 1;

	return 0;
}

static int cpu_load_analyzer_exit(void)
{
	before_cpu_task_history_num = cpu_task_history_num;
	before_cpu_load_history_num = cpu_load_history_num;

#if defined(CONFIG_SLP_INPUT_REC)
	before_input_rec_history_num = input_rec_history_num;
#endif

	cpu_task_history_onoff = 0;

#if defined (CONFIG_CHECK_WORK_HISTORY)
	cpu_work_history_onoff = 0;
#endif
#if defined (CONFIG_SLP_INPUT_REC)
	input_rec_history_onoff = 0;
#endif
	saved_task_info_onoff = 0;

	msleep(1); /* to prevent wrong access to released memory */

	if (cpu_load_freq_history != NULL)
		vfree(cpu_load_freq_history);

	if (cpu_load_freq_history_view != NULL)
		vfree(cpu_load_freq_history_view);

	if (cpu_task_history != NULL)
		vfree(cpu_task_history);
	if (cpu_task_history_view != NULL)
		vfree(cpu_task_history_view);

#if defined (CONFIG_CHECK_WORK_HISTORY)
	if (cpu_work_history != NULL)
		vfree(cpu_work_history);

	if (cpu_work_history_view != NULL)
		vfree(cpu_work_history_view);
#endif

#if defined (CONFIG_SLP_INPUT_REC)
	if (input_rec_history != NULL)
		vfree(input_rec_history);

	if (input_rec_history_view != NULL)
		vfree(input_rec_history_view);
#endif

	store_killed_exit();

	return 0;
}

enum {
	SCHED_HISTORY_NUM,
	INPUT_REC_NUM,
	GOVERNOR_HISTORY_NUM,
};

static int load_analyzer_reset(int mode, int value)
{
	int ret = 0;

	cpu_load_analyzer_exit();

	switch (mode) {
		case SCHED_HISTORY_NUM:
			cpu_task_history_num = value;
			break;
#if defined(CONFIG_SLP_INPUT_REC)
		case INPUT_REC_NUM:
			input_rec_history_num = value;
			break;
#endif
		case GOVERNOR_HISTORY_NUM:
			cpu_load_history_num = value;
			break;

		default:
			break;
	}

	ret = cpu_load_analyzer_init(NORMAL_MODE);
	if (ret < 0) {
		cpu_load_analyzer_init(RECOVERY_MODE);
		return -EINVAL;
	}

	return 0;
}

static int saved_load_analyzer_data_read_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret +=  snprintf(buf + ret, buf_size - ret, "%d\n", cpu_load_history_num);

	return ret;
}

static ssize_t saved_load_analyzer_data_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,
			saved_load_analyzer_data_read_sub);

	return size_for_copy;
}

#define CPU_LOAD_SCHED_NUM_MIN	1000
#define CPU_LOAD_SCHED_NUM_MAX	1000000
static ssize_t saved_sched_num_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int saved_sched_num = 0;
	char buf[32];
	size_t buf_size = min(count, (sizeof(buf)-1));

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	saved_sched_num = atoi(buf);
	if ((saved_sched_num < CPU_LOAD_SCHED_NUM_MIN)
		&& (saved_sched_num > CPU_LOAD_SCHED_NUM_MAX)) {

		pr_info("Wrong range value saved_sched_num = %d\n", saved_sched_num);

		return -EINVAL;
	}

	if (load_analyzer_reset(SCHED_HISTORY_NUM, saved_sched_num) < 0)
		return -EINVAL;

	return count;
}


static int saved_sched_num_read_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret +=  snprintf(buf + ret, buf_size - ret, "%d\n", cpu_task_history_num);

	return ret;
}

static ssize_t saved_sched_num_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos
						,saved_sched_num_read_sub);

	return size_for_copy;
}

#if defined(CONFIG_SLP_INPUT_REC)
#define INPUT_REC_NUM_MIN	1000
#define INPUT_REC_NUM_MAX	1000000
static ssize_t saved_input_rec_num_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	int saved_input_rec_num = 0;
	char buf[32];
	size_t buf_size = min(count, (sizeof(buf)-1));

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	saved_input_rec_num = atoi(buf);
	if ((saved_input_rec_num < INPUT_REC_NUM_MIN)
		&& (saved_input_rec_num > INPUT_REC_NUM_MAX)) {

		pr_info("Wrong range value saved_input_rec_num = %d\n", saved_input_rec_num);
		return -EINVAL;
	}

	if (load_analyzer_reset(INPUT_REC_NUM, saved_input_rec_num) < 0)
		return -EINVAL;

	return count;
}

static int saved_input_rec_num_read_sub(char *buf, int buf_size)
{
	int ret = 0;

	ret +=  snprintf(buf + ret, buf_size - ret, "%d\n", input_rec_history_num);

	return ret;
}

static ssize_t saved_input_rec_num_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos
						,saved_input_rec_num_read_sub);

	return size_for_copy;
}
#endif

#define CPU_HISTORY_NUM_MIN	1000
#define CPU_HISTORY_NUM_MAX	8000
static ssize_t saved_load_analyzer_data_write(struct file *file,
		const char __user *user_buf, size_t count,
		loff_t *ppos)
{
	int history_num;
	char buf[32];
	size_t buf_size = min(count, (sizeof(buf)-1));

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	history_num = atoi(buf);

	if ((history_num < CPU_HISTORY_NUM_MIN)
		&& (history_num > CPU_HISTORY_NUM_MAX)) {

		pr_info("Wrong range value cpu_load_history_num = %d\n", history_num);

		return -EINVAL;
	}

	if (load_analyzer_reset(GOVERNOR_HISTORY_NUM, history_num) < 0)
		return -EINVAL;

	return count;
}

static int active_app_pid_read_sub(char *buf, int buf_size)
{
	int ret = 0;
	char task_name[TASK_COMM_LEN+1] = {0,};

	get_name_from_pid(task_name, saved_load_factor.active_app_pid);

	ret +=  snprintf(buf + ret, buf_size - ret, "%s\n", task_name);

	return ret;
}

static ssize_t active_app_pid_read(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int size_for_copy;

	size_for_copy = wrapper_for_debug_fs(buffer, count, ppos,
			active_app_pid_read_sub);

	return size_for_copy;
}

static ssize_t active_app_pid_write(struct file *file,
		const char __user *user_buf, size_t count,
		loff_t *ppos)
{
	int active_app_pid;
	char buf[32];
	size_t buf_size = min(count, (sizeof(buf)-1));

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	active_app_pid = atoi(buf);

	store_external_load_factor(ACTIVE_APP_PID, active_app_pid);

	return count;
}

#define UN_KNOWN		0x00
#define TASK_HISTORY	0x01
#define WORK_HISTORY	0x02
#define INPUT_HISTORY	0x04

int data_part_str_to_num(char *str)
{
	int ret =0;

	if (strstr(str, "task") != NULL)
		ret += TASK_HISTORY;

	if (strstr(str, "work") != NULL)
		ret += WORK_HISTORY;

	if (strstr(str, "input") != NULL)
		ret += INPUT_HISTORY;

	return ret;
}

static ssize_t save_data_to_file_write(struct file *file,
		const char __user *user_buf, size_t count,
		loff_t *ppos)
{
	//int save_type;
	char file_path[100] = {0,};
	void *data = NULL;
	int size = 0, save_type;
	char buf[32];
	size_t buf_size = min(count, (sizeof(buf)-1));

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	save_type = data_part_str_to_num((char *)buf);

	if (save_type && TASK_HISTORY) {

	}

	if (save_type && WORK_HISTORY) {

	}

	if (save_type && INPUT_HISTORY) {
#if defined (CONFIG_SLP_INPUT_REC)
		input_rec_save_data();
#endif
	}

	if (save_type != UN_KNOWN) {
		/* default save data + */
		strcpy(file_path, "/opt/usr/media/cpu_load_freq_history.dat");
		data = cpu_load_freq_history_view;
		size = CPU_LOAD_FREQ_HISTORY_SIZE;
		save_data_to_file(file_path, data, size);

		strcpy(file_path, "/opt/usr/media/cpu_load_freq_history_view_cnt");
		data = &cpu_load_freq_history_view_cnt;
		size = sizeof(unsigned int);
		save_data_to_file(file_path, data, size);
		/* default save data - */
	}

	return count;
}

static ssize_t load_data_from_file_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	//int save_type;
	char file_path[100] = {0,};
	void *data = NULL;
	int size = 0, load_type;
	char buf[32];
	size_t buf_size = min(count, (sizeof(buf)-1));

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	load_type = data_part_str_to_num((char *)buf);

	if (load_type && TASK_HISTORY) {

	}

	if (load_type && WORK_HISTORY) {

	}

	if (load_type && INPUT_HISTORY) {
#if defined (CONFIG_SLP_INPUT_REC)
		input_rec_load_data();
#endif
	}

	if (load_type != UN_KNOWN) {
		/* default load data  + */
		strcpy(file_path, "/opt/usr/media/cpu_load_freq_history_view_cnt");
		data = &cpu_load_freq_history_view_cnt;
		size = sizeof(unsigned int);
		load_data_from_file(data, file_path, size);

		strcpy(file_path, "/opt/usr/media/cpu_load_freq_history.dat");
		data = cpu_load_freq_history_view;
		size = CPU_LOAD_FREQ_HISTORY_SIZE;
		load_data_from_file(data, file_path, size);
		/* default load data  - */

#if defined(CONFIG_SLP_INPUT_REC)
		b_input_load_data = 1;
#endif
	}

	return count;
}

#if defined(CONFIG_CHECK_NOT_CPUIDLE_CAUSE)
static const struct file_operations not_lpa_cause_check_fops = {
	.owner = THIS_MODULE,
	.read = not_lpa_cause_check,
};
#endif

static const struct file_operations debug_value_fops = {
	.owner = THIS_MODULE,
	.read = debug_value_read,
	.write =debug_value_write,
};

static const struct file_operations active_app_pid_fops = {
	.owner = THIS_MODULE,
	.read = active_app_pid_read,
	.write =active_app_pid_write,
};

static const struct file_operations saved_sched_num_fops = {
	.owner = THIS_MODULE,
	.read = saved_sched_num_read,
	.write =saved_sched_num_write,
};

#if defined(CONFIG_SLP_INPUT_REC)
static const struct file_operations saved_input_rec_num_fops = {
	.owner = THIS_MODULE,
	.read = saved_input_rec_num_read,
	.write = saved_input_rec_num_write,
};
#endif

static const struct file_operations saved_load_analyzer_data_fops = {
	.owner = THIS_MODULE,
	.read = saved_load_analyzer_data_read,
	.write =saved_load_analyzer_data_write,
};

static const struct file_operations save_data_to_file_fops = {
	.owner = THIS_MODULE,
	.write =save_data_to_file_write,
};

static const struct file_operations load_data_from_file_fops = {
	.owner = THIS_MODULE,
	.write =load_data_from_file_write,
};

static void wristup_cpu_pmqos_booster_work(struct work_struct *work)
{
	/* PmQoS request for enhance wristup wake up performance - timeout = 0.5 second */
	pm_qos_update_request_timeout(&wristup_qos_request,
		WRISTUP_CPU_MIN_FREQ, (WRISTUP_CPU_QOS_LOCK_TIME * USEC_PER_SEC) >> 1);
	store_wrist_up_state();

	pr_info("%s: wristup cpu booster with max freq\n", __func__);
}

#if defined(CONFIG_GEM_ALLOC_DEBUGS)
static void store_gem_alloc(struct work_struct *work)
{
	int i;
	bool check_exist = false;
	gem_alloc_work_t *g_data = (gem_alloc_work_t*) work;
	mutex_lock(&gem_mutex);

	if (gem_alloc_pos >= 0) {
		for (i = 0; i <= gem_alloc_pos; i++) {
			if (strcmp(gem_alloc_arr[i].task_name, g_data->data.task_name) == 0) {
				gem_alloc_arr[i].count ++;
				check_exist = true;
				break;
			}
		}
	}

	if (gem_alloc_pos > MAX_GEM_TASK - 2) {
		mutex_unlock(&gem_mutex);
		extract_gem_alloc_info(STACK_OVERFLOW);
		return;
	}

	if (!check_exist) {
		gem_alloc_pos ++;
		snprintf(gem_alloc_arr[gem_alloc_pos].task_name,
			GEM_TASK_NAME_LEN, g_data->data.task_name);
		gem_alloc_arr[gem_alloc_pos].count = 1;
	}

	mutex_unlock(&gem_mutex);
}

void extract_gem_alloc_info(gem_extract_type_t type) {
	int i, len = 0;
	// for store gem items with highest count are selected (maximum: 3)
	gem_alloc_t gem_sel[3] = { {"", 0, 0}, {"", 0, 0}, {"",0, 0},};
	mutex_lock(&gem_mutex);

	if (gem_alloc_pos + 1 <= 3) {
		for (i = 0; i <= gem_alloc_pos; i++) {
			len += snprintf(gem_alloc_buf + len, sizeof(gem_alloc_buf) - len, "%s(%d) ",
					gem_alloc_arr[i].task_name, gem_alloc_arr[i].count);
		}
		pr_info("GEM ALLOC: %d | %s", type, gem_alloc_buf);
		gem_alloc_pos = -1;
		mutex_unlock(&gem_mutex);
		return;
	}

	for (i = 0; i <= gem_alloc_pos; i++) {
		if (gem_alloc_arr[i].count > gem_sel[0].count) {
			gem_sel[2] = gem_sel[1];
			gem_sel[1] = gem_sel[0];
			gem_sel[0].count = gem_alloc_arr[i].count;
			gem_sel[0].pos = i;
		}
		else if (gem_alloc_arr[i].count > gem_sel[1].count) {
			gem_sel[2] = gem_sel[1];
			gem_sel[1].count = gem_alloc_arr[i].count;
			gem_sel[1].pos = i;
		}
		else if (gem_alloc_arr[i].count > gem_sel[2].count) {
			gem_sel[2].count = gem_alloc_arr[i].count;
			gem_sel[2].pos = i;
		}
	}

	switch (type)
	{
		case LCD_OFF:
			for (i = 0; i < 3; i++) {
				len += snprintf(gem_alloc_buf + len, sizeof(gem_alloc_buf) - len, "%s(%d) ",
						gem_alloc_arr[gem_sel[i].pos].task_name,
						gem_alloc_arr[gem_sel[i].pos].count);
			}

			gem_alloc_pos = -1;
			break;
		case STACK_OVERFLOW:
			for (i = 0; i < 3; i++) {
				snprintf(gem_sel[i].task_name, GEM_TASK_NAME_LEN, "%s", gem_alloc_arr[gem_sel[i].pos].task_name);
			}

			for (i = 0; i < 3; i++) {
				snprintf(gem_alloc_arr[i].task_name, GEM_TASK_NAME_LEN, "%s", gem_sel[i].task_name);
				gem_alloc_arr[i].count = gem_sel[i].count;
			}

			gem_alloc_pos = i;
			break;
		default:
			pr_info("GEM ALLOC: Unknown type");
			break;
	}

	pr_info("GEM ALLOC: %d | %s", type, gem_alloc_buf);
	mutex_unlock(&gem_mutex);
}

void reset_gem_alloc_buf(void)
{
	mutex_lock(&gem_mutex);
	memset(gem_alloc_buf, '\0', 1);
	mutex_unlock(&gem_mutex);
}

void add_gem_queue_work(char *task_name)
{
	snprintf(w_data->data.task_name, sizeof(w_data->data.task_name), "%s", task_name);
	queue_work(gem_alloc_wq, &w_data->work);
}
#endif

static int __init system_load_analyzer_init(void)
{
	int ret = 0;
	struct dentry *d;
	d = debugfs_create_dir("load_analyzer", NULL);
	if (d) {
		debugfs_cpu_bus(d);

#if defined (CONFIG_CHECK_WORK_HISTORY)
		debugfs_workqueue(d);
#endif

#if defined(CONFIG_SLP_INPUT_REC)
		debugfs_input_rec(d);
#endif

#if defined(CONFIG_CHECK_NOT_CPUIDLE_CAUSE)
		if (!debugfs_create_file("not_lpa_cause_check", 0600, d, NULL, &not_lpa_cause_check_fops))
			pr_err("%s : debugfs_create_file, error\n", "not_lpa_cause_check");
#endif

		if (!debugfs_create_file("debug_value", 0600, d, NULL, &debug_value_fops))
			pr_err("%s : debugfs_create_file, error\n", "debug_value");

#if defined(CONFIG_SLP_CHECK_PHY_ADDR)
		debugfs_addr(d);
#endif

#if defined(CONFIG_SLP_PERFORMANCE_MONITOR)
		debugfs_performance(d);
#endif

		if (!debugfs_create_file("active_app_pid", 0600, d, NULL, &active_app_pid_fops))
			pr_err("%s : debugfs_create_file, error\n", "active_app_pid");

#if defined(CONFIG_SLP_MINI_TRACER)
		debugfs_mini(d);
#endif

#if defined(CONFIG_CHECK_TIME_IN_STATE)
		debugfs_time_in_state(d);
#endif

#if defined (CONFIG_SLP_CPU_TESTER)
		debugfs_cpu_tester(d);
#endif

		if (!debugfs_create_file("saved_sched_num", 0600, d, NULL, &saved_sched_num_fops))
			pr_err("%s : debugfs_create_file, error\n", "saved_sched_num");

#if defined(CONFIG_SLP_INPUT_REC)
		if (!debugfs_create_file("saved_input_rec_num", 0600, d, NULL, &saved_input_rec_num_fops))
			pr_err("%s : debugfs_create_file, error\n", "saved_input_rec_num");
#endif

		if (!debugfs_create_file("saved_load_analyzer_data_num", 0600, d, NULL, &saved_load_analyzer_data_fops))
			pr_err("%s : debugfs_create_file, error\n", "saved_load_analyzer_data_num");

		if (!debugfs_create_file("save_data_to_file", 0600, d, NULL, &save_data_to_file_fops))
			pr_err("%s : debugfs_create_file, error\n", "save_data_to_file");

		if (!debugfs_create_file("load_data_from_file", 0600, d, NULL, &load_data_from_file_fops))
			pr_err("%s : debugfs_create_file, error\n", "load_data_from_file");
	}

#if defined(CONFIG_LOAD_ANALYZER_PMQOS)
	pm_qos_add_request(&pm_qos_min_cpu, PM_QOS_CLUSTER0_FREQ_MIN, 0);
	pm_qos_add_request(&pm_qos_max_cpu, PM_QOS_CLUSTER0_FREQ_MAX, INT_MAX);
	pm_qos_add_request(&pm_qos_min_cpu_num, PM_QOS_CPU_ONLINE_MIN, 0);
	pm_qos_add_request(&pm_qos_max_cpu_num, PM_QOS_CPU_ONLINE_MAX, CPU_NUM);
#endif

#if defined (CONFIG_SLP_CPU_TESTER)
	{
		int i;

		cpu_tester_wq = alloc_workqueue("cpu_tester_wq", WQ_HIGHPRI, 0);
		if (!cpu_tester_wq) {
			printk(KERN_ERR "Failed to create cpu_tester_wq workqueue\n");
			return -EFAULT;
		}

		for (i = 0; i < CPU_NUM; i++) {
			char str[64];
			sprintf(str, "cpu_tester%d_wq", i);
			cpu_load_wq[i] = alloc_workqueue(str, WQ_UNBOUND, 0);
			if (!cpu_load_wq[i]) {
				printk(KERN_ERR "Failed to create %s workqueue\n", str);
				return -EFAULT;
			} else{
				INIT_DELAYED_WORK(&(cpu_load_tester_work[i]), cpu_load_tester_work_start);
			}
		}

		INIT_DELAYED_WORK(&cpu_tester_work, cpu_tester_work_start);
		INIT_DELAYED_WORK(&cpu_freq_tester_work, cpu_freq_tester_work_start);
		INIT_DELAYED_WORK(&cpu_idle_tester_work, cpu_idle_tester_work_start);
	}
#endif
/*
#if defined(CONFIG_SLP_PERFORMANCE_MONITOR)
	load_analyzer_performance_init();
#endif
*/
	if (cpu_load_analyzer_init(NORMAL_MODE) != 0)
		pr_info("[%s] cpu_load_analyzer_init ERROR", __func__);

#if defined(CONFIG_SLP_MINI_TRACER)
	if (kernel_mini_tracer_init() != 0)
		pr_info("[%s] kernel_mini_tracer_init ERROR", __func__);
#endif

#if defined(CONFIG_CHECK_TIME_IN_STATE)
	if (la_time_in_state_monitor_init() != 0)
			pr_info("[%s] time_in_state_monitor ERROR", __func__);
#endif

#if defined(CONFIG_SLP_BUSY_LEVEL)
	touch_event_register_notifier(&touch_event_notifier, 0);
	load_analyzer_init_done = 1;
#endif

	/* PmQoS request for enhance wristup wake up performance */
	wristup_cpu_booster_work.wristup_cpu_pmqos_workqueue = create_workqueue("wristup_booster");

	if (!wristup_cpu_booster_work.wristup_cpu_pmqos_workqueue) {
		pr_err("cannot create wristup_booster workqueue\n");
		goto err_wristup_booster_wq;
	}

	INIT_WORK(&wristup_cpu_booster_work.wristup_cpu_pmqos_work, wristup_cpu_pmqos_booster_work);
	wristup_work_initialized = 1;
	pm_qos_add_request(&wristup_qos_request, PM_QOS_CLUSTER0_FREQ_MIN, 0);

	pr_info("%s: system load analyzer init done\n", __func__);

	work_detect_busy_level_queue = create_workqueue("wristup_booster");
	INIT_DELAYED_WORK(&work_detect_busy_level, work_detect_busy_level_function);

	/* for gem alloc workqueue */
#if defined(CONFIG_GEM_ALLOC_DEBUGS)
	gem_alloc_wq = create_workqueue("gem_alloc");

	if (!gem_alloc_wq) {
		pr_err("GEM ALLOC: Can't create gem alloc workqueue\n");
		destroy_workqueue(gem_alloc_wq);
	}

	w_data = kmalloc(sizeof(gem_alloc_work_t), GFP_KERNEL);
	INIT_WORK(&w_data->work, store_gem_alloc);
#endif
	return ret;

err_wristup_booster_wq:
	destroy_workqueue(wristup_cpu_booster_work.wristup_cpu_pmqos_workqueue);
	return ret;
}

static void __exit system_load_analyzer_exit(void)
{
	destroy_workqueue(wristup_cpu_booster_work.wristup_cpu_pmqos_workqueue);
	pm_qos_remove_request(&wristup_qos_request);
	cpu_load_analyzer_exit();
#if defined(CONFIG_SLP_MINI_TRACER)
	kernel_mini_tracer_exit();
#endif
#if defined(CONFIG_SLP_BUSY_LEVEL)
	load_analyzer_init_done = 0;
	touch_event_unregister_notifier(&touch_event_notifier, 0);
#endif

#if defined(CONFIG_GEM_ALLOC_DEBUGS)
	destroy_workqueue(gem_alloc_wq);
	kfree(w_data);
#endif
}

module_init(system_load_analyzer_init);
module_exit(system_load_analyzer_exit);

MODULE_AUTHOR("Yong-U, Baek <yu.baek@samsung.com>");
MODULE_DESCRIPTION("'SLP power debuger");
MODULE_LICENSE("GPL");
