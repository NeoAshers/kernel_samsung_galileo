/* drivers/misc/sensorhub_stat.c
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
 * 1.1   Hunsup Jung <hunsup.jung@samsung.com>       <2017.08.17>              *
 *                                                   Remove unnecessary code   *
 * ---- -------------------------------------------- ------------------------- *
 * 1.2   Junho Jang <vincent.jang@samsung.com>       <2018.03.20>              *
 *                                                   Minor refactoring   *
 * ---- -------------------------------------------- ------------------------- *
 * 1.3   Junho Jang <vincent.jang@samsung.com>       <2019.05.09>              *
 *                                                   Add a wrist up mode     *
 * ---- -------------------------------------------- ------------------------- *
 */

#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/suspend.h>

#include <linux/sensorhub_stat.h>
#ifdef CONFIG_ENERGY_MONITOR
#include <linux/power/energy_monitor.h>
#endif
#ifdef CONFIG_SLEEP_MONITOR
#include <linux/power/sleep_monitor.h>
#endif

#define SENSORHUB_STAT_PREFIX	"sensorhub_stat: "

#define LOCATION_RUNNING_TIME_SIZE     4

#define RESTING_HR_TIMESTAMP_SIZE     4
#define RESTING_HR_ACTIVITY_TYPE_SIZE 3
#define RESTING_HR_HEART_RATE_SIZE    2
#define RESTING_HR_RELIABILTY_SIZE    2
#define RESTING_HR_DURATION_SIZE    1

static int suspend_finished = 1;
static ktime_t resume_time;
static int last_wakeup_lib;

static DEFINE_SPINLOCK(sensorhub_lock);
static LIST_HEAD(sensorhub_list);

struct sensorhub_stat {
	struct list_head link;
	struct timespec init_time;
	int lib_number;
	unsigned long long last_gps_user;
	unsigned char last_gps_ext;
	atomic_t wakeup;
	atomic_t activity;
	atomic_t event;
	atomic_t success_cnt_post_suspend;
	atomic_t success_dur_post_suspend;
	atomic_t fail_count_post_suspend;
	atomic_t fail_dur_post_suspend;
	atomic_t success_cnt;
	atomic_t success_dur;
	atomic_t fail_cnt;
	atomic_t fail_dur;
	ktime_t ts;
	int suspend_count;

	bool active;
	ktime_t subtotal_time;
	ktime_t total_time;
	ktime_t ts_boot;

#ifdef CONFIG_ENERGY_MONITOR
	int emon_wakeup;
	int emon_activity;
	int emon_event;
#endif
};

static int sensorhub_stat_pm_suspend_prepare_cb(ktime_t ktime)
{
	struct sensorhub_stat *entry;
	unsigned long flags;
	u32 wakeup, activity, event;
	u32 cnt, succes_cnt, fail_cnt;
	u32 dur, success_dur, fail_dur;
	u32 entry_cnt = 0;

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (entry->ts.tv64 > ktime.tv64) {
			entry_cnt++;
			wakeup = (u32)(atomic_read(&entry->wakeup) + INT_MIN);
			activity = (u32)(atomic_read(&entry->activity) + INT_MIN);
			event = (u32)(atomic_read(&entry->event) + INT_MIN);
			succes_cnt = (u32)(atomic_read(&entry->success_cnt_post_suspend) + INT_MIN);
			success_dur = (u32)(atomic_read(&entry->success_dur_post_suspend) + INT_MIN);
			fail_cnt = (u32)(atomic_read(&entry->fail_count_post_suspend) + INT_MIN);
			fail_dur = (u32)(atomic_read(&entry->fail_dur_post_suspend) + INT_MIN);
			cnt = succes_cnt + fail_cnt;
			dur = success_dur + fail_dur;

			pr_info(SENSORHUB_STAT_PREFIX
					"%d: %u %u %u %u %u %u %u %u %u (%d %2d)\n",
					entry->lib_number, wakeup, activity, event,
					cnt, dur,
					succes_cnt, success_dur,
					fail_cnt, fail_dur, suspend_stats.success, entry_cnt);
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);
	suspend_finished = 0;

	return 0;
}

static int sensorhub_stat_pm_post_suspend_cb(ktime_t ktime)
{
	unsigned long flags;
	struct sensorhub_stat *entry;

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (entry->ts.tv64 > ktime.tv64) {
			/* xxx_post_suspend of resting hr used for energy monitor */
			if (entry->lib_number != SHUB_LIB_DATA_RESTING_HR) {
				/* Reset counter, so we can track sensor
				  * activity traffic during next post suspend.
				  */
				atomic_set(&entry->success_cnt_post_suspend, INT_MIN);
				atomic_set(&entry->success_dur_post_suspend, INT_MIN);
				atomic_set(&entry->fail_count_post_suspend, INT_MIN);
				atomic_set(&entry->fail_dur_post_suspend, INT_MIN);
			}
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);
	suspend_finished = 1;

	return 0;
}

static struct sensorhub_stat *find_sensorhub_stat(int lib_number)
{
	unsigned long flags;
	struct sensorhub_stat *entry;

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (lib_number == entry->lib_number) {
			spin_unlock_irqrestore(&sensorhub_lock, flags);
			return entry;
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	return NULL;
}

/* Create a new entry for tracking the specified sensorhub lib#. */
static struct sensorhub_stat *create_stat(int lib_number)
{
	unsigned long flags;
	struct sensorhub_stat *new_sensorhub_lib;

	/* Create the sensorhub stat struct and append it to the list. */
	new_sensorhub_lib = kmalloc(sizeof(struct sensorhub_stat), GFP_ATOMIC);
	if (!new_sensorhub_lib)
		return NULL;

	new_sensorhub_lib->lib_number = lib_number;

	/* Counters start at INT_MIN, so we can track 4GB of sensor activity. */
	atomic_set(&new_sensorhub_lib->wakeup, INT_MIN);
	atomic_set(&new_sensorhub_lib->activity, INT_MIN);
	atomic_set(&new_sensorhub_lib->event, INT_MIN);
	atomic_set(&new_sensorhub_lib->success_cnt_post_suspend, INT_MIN);
	atomic_set(&new_sensorhub_lib->success_dur_post_suspend, INT_MIN);
	atomic_set(&new_sensorhub_lib->fail_count_post_suspend, INT_MIN);
	atomic_set(&new_sensorhub_lib->fail_dur_post_suspend, INT_MIN);
	atomic_set(&new_sensorhub_lib->success_cnt, INT_MIN);
	atomic_set(&new_sensorhub_lib->success_dur, INT_MIN);
	atomic_set(&new_sensorhub_lib->fail_cnt, INT_MIN);
	atomic_set(&new_sensorhub_lib->fail_dur, INT_MIN);
	new_sensorhub_lib->suspend_count = 0;
#ifdef CONFIG_ENERGY_MONITOR
	new_sensorhub_lib->emon_wakeup = 0;
	new_sensorhub_lib->emon_activity = 0;
	new_sensorhub_lib->emon_event = 0;
#endif

	new_sensorhub_lib->active = false;
	new_sensorhub_lib->subtotal_time = ktime_set(0,0);
	new_sensorhub_lib->total_time = ktime_set(0,0);
	new_sensorhub_lib->ts_boot = ktime_set(0,0);

	/* Init gps info */
	new_sensorhub_lib->last_gps_user = 0;
	new_sensorhub_lib->last_gps_ext = 0;

	/* snapshot the current system time for calculating consumed enrergy */
	getnstimeofday(&new_sensorhub_lib->init_time);

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_add_tail(&new_sensorhub_lib->link, &sensorhub_list);
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	return new_sensorhub_lib;
}

// Big Endian
static u32 bytes_to_uint(const char* hub_data, int* cursor, int width)
{
	int i;
	u32 sum = 0;

	if (width > 4)
		pr_info("%s Invalid parameter\n", __func__);

	for (i = 0; i < width; i++) {
		sum = sum << 8;
		sum |= (hub_data[(*cursor)++] & 0xFF);
	}

	return sum;
}

// Little Endian
static u32 bytes_to_uint_little(const char* hub_data, int* cursor, int width)
{
	int i;
	u32 v, sum = 0;

	if (width > 4)
		pr_info("%s Invalid parameter\n", __func__);

	for ( i=0; i < width; i++) {
		v = hub_data[(*cursor)++] & 0xFF;
		sum |= (v << (8*i));
	}

	return sum;
}

static int sensorhub_stat_is_wakeup_blacklist(int lib_number)
{
	int ret = 0;

	if (lib_number == SHUB_LIB_DATA_WRIST_UP) {
		last_wakeup_lib = lib_number;
		ret = 1;
	} else if (lib_number == SHUB_LIB_DATA_PEDOMETER && last_wakeup_lib == -1)
		ret = 1;

	return ret;
}

static int sensorhub_stat_is_wakeup_whitelist(int lib_number)
{

	if (lib_number == SHUB_LIB_DATA_AUTO_BRIGHTNESS ||
		lib_number == SHUB_LIB_DATA_MCU_LOGGER)
		return 1;

	return 0;
}

static void sensorhub_stat_find_wakeup_suspect(
	struct sensorhub_stat *entry)
{
	struct sensorhub_stat *temp_entry;
	unsigned long flags;

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(temp_entry, &sensorhub_list, link) {
		if (entry->suspend_count == temp_entry->suspend_count &&
			entry != temp_entry) {
			atomic_dec(&entry->wakeup);
			break;
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);
}

static void sensorhub_stat_update_gps(
	struct sensorhub_stat *entry, const char *dataframe, int size)
{
	int i;
	int cursor = 0;
	const char *hub_data;
	u32 running_time, total_running_time;
	unsigned long long last_gps_user = 0;

	hub_data = &dataframe[3];

	/* Get gps running time */
	entry->ts = ktime_get();
	running_time = bytes_to_uint_little(hub_data,
					&cursor, LOCATION_RUNNING_TIME_SIZE);
	total_running_time = (u32)(atomic_read(&entry->success_dur) + INT_MIN);
	if (total_running_time > running_time) {
		pr_info(SENSORHUB_STAT_PREFIX
				"%s: running_time(%us) total_running_time(%us): clear emon time\n",
				__func__, running_time, total_running_time);
		atomic_set(&entry->fail_dur, INT_MIN);
		atomic_inc(&entry->fail_cnt);
	}
	atomic_set(&entry->success_dur, (int)running_time + INT_MIN);

	if (size > DATAFRAME_GPS_EXT) {
		/* Get last gps user */
		for (i = DATAFRAME_GPS_LAST_USER_END;
				i >= DATAFRAME_GPS_LAST_USER_START;
				i--) {
			last_gps_user = last_gps_user << 8;
			last_gps_user |= dataframe[i];
		}
		entry->last_gps_user = last_gps_user;

		/* Get last gps ext */
		entry->last_gps_ext = dataframe[DATAFRAME_GPS_EXT];
	}

	pr_info(SENSORHUB_STAT_PREFIX
			"%s: running_time(%us) dataframe_size(%d)\n",
			__func__, running_time, size);
}

static void sensorhub_stat_update_hr(
	struct sensorhub_stat *entry, const char *dataframe, int size)
{
	int i;
	int cursor = 0;
	int num_data;
	const char *hub_data;
	u32 fail_cnt = 0;
	u32 ts, activity_type, heart_rate, reliability, duration;

	num_data = dataframe[5];
	hub_data = &dataframe[6];

	for (i = 0; i < num_data; i++) {
		ts = bytes_to_uint(hub_data,
			&cursor, RESTING_HR_TIMESTAMP_SIZE);
		activity_type = bytes_to_uint(hub_data,
			&cursor, RESTING_HR_ACTIVITY_TYPE_SIZE);
		heart_rate = bytes_to_uint(hub_data, &cursor,
			RESTING_HR_HEART_RATE_SIZE);
		reliability = bytes_to_uint(hub_data,
			&cursor, RESTING_HR_RELIABILTY_SIZE);
		duration = bytes_to_uint(hub_data,
			&cursor, RESTING_HR_DURATION_SIZE);

		if (heart_rate >= 1000) {
			fail_cnt++;
			atomic_add(duration, &entry->fail_dur);
		} else
			atomic_add(duration, &entry->success_dur);

		pr_info(SENSORHUB_STAT_PREFIX
				"%s: %u %u %u %u %u\n",
				__func__, ts, activity_type, heart_rate, reliability, duration);
	}

	atomic_add(num_data - fail_cnt, &entry->success_cnt);
	atomic_add(fail_cnt, &entry->fail_cnt);
	entry->ts = ktime_get();
}

static void sensorhub_stat_update_wear(
	struct sensorhub_stat *entry, const char *dataframe, int size)
{
	int wear;
	ktime_t now, duration;

	if (!entry)
		return;

	wear = (int)dataframe[3];

	pr_info(SENSORHUB_STAT_PREFIX "%s: %d %d\n",
		__func__, wear, entry->active);

	now = ktime_get_boottime();
	if (wear == 1 && !entry->active) {
		entry->active = true; // TODO: need to change name, wear on
		duration = ktime_sub(now, entry->ts_boot);
		entry->subtotal_time =
			ktime_add(entry->subtotal_time, duration);
		entry->ts_boot = now;
	} else if (wear == 2 && entry->active) {
		entry->active = false;
		duration = ktime_sub(now, entry->ts_boot);
		entry->total_time =
			ktime_add(entry->total_time, duration);
		entry->ts_boot = now;
	}

	pr_info(SENSORHUB_STAT_PREFIX "%s: %d %d %lld %lld %lld\n",
		__func__, wear, entry->active, ktime_to_ms(duration),
		ktime_to_ms(entry->subtotal_time), ktime_to_ms(entry->total_time));
}

int sensorhub_stat_rcv(const char *dataframe, int size)
{
	int lib_number = (int)dataframe[2];
	struct sensorhub_stat *entry;
	unsigned int wakeup, activity, event;

	if ((entry = find_sensorhub_stat(lib_number)) == NULL &&
		((entry = create_stat(lib_number)) == NULL)) {
			return -1;
	}

	if (lib_number == SHUB_LIB_DATA_LOCATION && dataframe[1] == 0x05)
		sensorhub_stat_update_gps(entry, dataframe, size);

	if (lib_number == SHUB_LIB_DATA_RESTING_HR)
		sensorhub_stat_update_hr(entry, dataframe, size);

	if (lib_number == SHUB_LIB_DATA_WEARONOFF_MONITOR)
		sensorhub_stat_update_wear(entry, dataframe, size);

	atomic_inc(&entry->event);
	if (suspend_stats.success > entry->suspend_count)
		atomic_inc(&entry->activity);
	entry->suspend_count = suspend_stats.success;
	entry->ts = ktime_get();

	if (lib_number == SHUB_LIB_DATA_WRIST_UP) {
		struct sensorhub_stat *temp_entry;
		unsigned long flags;

		spin_lock_irqsave(&sensorhub_lock, flags);
		list_for_each_entry(temp_entry, &sensorhub_list, link) {
			if (temp_entry->active) {
  				atomic_inc(&temp_entry->event);
 			}
		}
		spin_unlock_irqrestore(&sensorhub_lock, flags);
	}

	if (suspend_finished)
		return 0;

	if (!sensorhub_stat_is_wakeup_whitelist(lib_number)) {
		atomic_inc(&entry->wakeup);
		atomic_inc(&entry->activity);
	}
	entry->suspend_count = suspend_stats.success + 1;

	if (!(sensorhub_stat_is_wakeup_whitelist(lib_number) ||
			sensorhub_stat_is_wakeup_blacklist(lib_number)))
		sensorhub_stat_find_wakeup_suspect(entry);

	wakeup = (unsigned int)(atomic_read(&entry->wakeup) + INT_MIN);
	activity = (unsigned int)(atomic_read(&entry->activity) + INT_MIN);
	event = (unsigned int)(atomic_read(&entry->event) + INT_MIN);
	pr_info(SENSORHUB_STAT_PREFIX
		"%s %d: %d %d %d (%d)\n",
		__func__, lib_number, wakeup, activity, event, entry->suspend_count);

	if (lib_number == SHUB_LIB_DATA_WRIST_UP) {
		struct sensorhub_stat *temp_entry;
		unsigned long flags;

		spin_lock_irqsave(&sensorhub_lock, flags);
		list_for_each_entry(temp_entry, &sensorhub_list, link) {
			if (temp_entry->active) {
				atomic_inc(&temp_entry->wakeup);
				atomic_inc(&temp_entry->activity);
  			}
		}
		spin_unlock_irqrestore(&sensorhub_lock, flags);
	}

	return 0;
}

int sensorhub_stat_rcv_instruction(unsigned char inst,
	unsigned char type, unsigned char *buf, size_t count)
{
	static unsigned char current_mode = 1;

	if (!buf)
		return -EINVAL;

	if (buf[0] == SHUB_LIB_DATA_WRIST_UP && buf[1] == 1) {
		struct sensorhub_stat *entry;
		unsigned char lib_number, mode, inst;
		ktime_t now, duration;

		lib_number = buf[0];
		mode = buf[2];
		inst = SHUB_INST_LIB_DATA_WRISTUP_MODE_0 + mode;

		pr_info(SENSORHUB_STAT_PREFIX"%s: %d %d %d\n",
			__func__, inst, current_mode, mode);

		if ((entry = find_sensorhub_stat(inst)) == NULL &&
			((entry = create_stat(inst)) == NULL)) {
				return -1;
		}

		atomic_inc(&entry->success_cnt);
		entry->suspend_count = suspend_stats.success;

		now = ktime_get_boottime();
		if (current_mode == mode) {
			duration = ktime_sub(now, entry->ts);
			entry->total_time =
				ktime_add(entry->total_time, duration);
		} else {
			struct sensorhub_stat *current_entry;
			unsigned char current_inst;

			current_inst = SHUB_INST_LIB_DATA_WRISTUP_MODE_0 + current_mode;

			if ((current_entry = find_sensorhub_stat(current_inst)) == NULL &&
				((current_entry = create_stat(current_inst)) == NULL)) {
					return -1;
			}

			current_entry->active = false;
			duration = ktime_sub(now, current_entry->ts);
			current_entry->total_time =
				ktime_add(current_entry->total_time, duration);
		}
		entry->active = true;
		entry->ts = now;
		current_mode = mode;
	}

	return 0;
}

#ifdef CONFIG_ENERGY_MONITOR
int sensorhub_stat_get_stat(int type,
	struct sensorhub_stat_info *sh_stat)
{
	unsigned long flags;
	int wakeup, activity, event;
	struct sensorhub_stat *entry;

	if (!sh_stat) {
		pr_err(SENSORHUB_STAT_PREFIX"%s: sh_stat is null\n", __func__);
		return -EINVAL;
	}

	memset(sh_stat, 0, sizeof(*sh_stat));

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		wakeup = (atomic_read(&entry->wakeup) + INT_MIN);
		activity = (atomic_read(&entry->activity) + INT_MIN);
		event = (atomic_read(&entry->event) + INT_MIN);
		if (entry->lib_number < SHUB_LIB_MAX) {
			sh_stat->wakeup[entry->lib_number] = wakeup - entry->emon_wakeup;
			sh_stat->activity[entry->lib_number] = activity - entry->emon_activity;
			sh_stat->event[entry->lib_number] = event - entry->emon_event;

			if (type != ENERGY_MON_TYPE_DUMP) {
				entry->emon_wakeup = wakeup;
				entry->emon_activity = activity;
				entry->emon_event = event;
			}

			pr_debug(SENSORHUB_STAT_PREFIX"%s: %d %d(%d) %d\n",
					__func__, entry->lib_number,
					sh_stat->wakeup[entry->lib_number],
					sh_stat->activity[entry->lib_number],
					sh_stat->event[entry->lib_number]);
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	return 0;
}

int sensorhub_stat_get_gps(
	struct sensorhub_gps_stat *sh_gps)
{
	struct sensorhub_stat *entry;
	unsigned long flags;
	int gps_time;

	if (!sh_gps) {
		pr_err(SENSORHUB_STAT_PREFIX"%s: sh_gps is null\n", __func__);
		return -EINVAL;
	}

	memset(sh_gps, 0, sizeof(*sh_gps));

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (entry->lib_number == SHUB_LIB_DATA_LOCATION) {
			gps_time = (atomic_read(&entry->success_dur) + INT_MIN);
			sh_gps->gps_time = gps_time;

			break;
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	pr_info(SENSORHUB_STAT_PREFIX"%s %d\n",
		__func__, sh_gps->gps_time);

	return 0;
}

int sensorhub_stat_get_gps_emon(int type,
	struct sensorhub_gps_stat *sh_gps)
{
	struct sensorhub_stat *entry;
	unsigned long flags;
	int gps_time, emon_gps_time;

	if (!sh_gps) {
		pr_err(SENSORHUB_STAT_PREFIX"%s: sh_gps is null\n", __func__);
		return -EINVAL;
	}

	memset(sh_gps, 0, sizeof(*sh_gps));

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (entry->lib_number == SHUB_LIB_DATA_LOCATION) {
			gps_time = (atomic_read(&entry->success_dur) + INT_MIN);
			emon_gps_time = (atomic_read(&entry->fail_dur) + INT_MIN);

			sh_gps->gps_time = gps_time - emon_gps_time;
			sh_gps->last_gps_user = entry->last_gps_user;
			sh_gps->last_gps_ext = entry->last_gps_ext;

			if (type != ENERGY_MON_TYPE_DUMP)
				atomic_set(&entry->fail_dur, (int)gps_time + INT_MIN);

			break;
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	pr_info(SENSORHUB_STAT_PREFIX"%s %d %llx %u\n",
		__func__, sh_gps->gps_time,
		sh_gps->last_gps_user, sh_gps->last_gps_ext);

	return 0;
}

int sensorhub_stat_get_hr_info(int type,
	struct sensorhub_hr_stat *sh_hr)
{
	struct sensorhub_stat *entry;
	unsigned long flags;
	int success_cnt, fail_cnt;
	int success_dur, fail_dur;
	int emon_success_cnt, emon_fail_cnt;
	int emon_success_dur, emon_fail_dur;

	if (!sh_hr) {
		pr_err(SENSORHUB_STAT_PREFIX"%s: sh_hr is null\n", __func__);
		return -EINVAL;
	}

	memset(sh_hr, 0, sizeof(*sh_hr));

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (entry->lib_number == SHUB_LIB_DATA_RESTING_HR) {
			success_cnt = atomic_read(&entry->success_cnt);
			success_dur = atomic_read(&entry->success_dur);
			fail_cnt = atomic_read(&entry->fail_cnt);
			fail_dur = atomic_read(&entry->fail_dur);
			emon_success_cnt = atomic_read(&entry->success_cnt_post_suspend);
			emon_success_dur = atomic_read(&entry->success_dur_post_suspend);
			emon_fail_cnt = atomic_read(&entry->fail_count_post_suspend);
			emon_fail_dur = atomic_read(&entry->fail_dur_post_suspend);

			sh_hr->success_cnt = success_cnt - emon_success_cnt;
			sh_hr->success_dur = success_dur - emon_success_dur;
			sh_hr->fail_cnt = fail_cnt - emon_fail_cnt;
			sh_hr->fail_dur = fail_dur - emon_fail_dur;

			if (type != ENERGY_MON_TYPE_DUMP) {
				atomic_set(&entry->success_cnt_post_suspend, success_cnt + INT_MIN);
				atomic_set(&entry->success_dur_post_suspend, success_dur + INT_MIN);
				atomic_set(&entry->fail_count_post_suspend, fail_cnt + INT_MIN);
				atomic_set(&entry->fail_dur_post_suspend, fail_dur + INT_MIN);
			}

			break;
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	pr_info(SENSORHUB_STAT_PREFIX"%s %d %d %d %d\n",
		__func__, sh_hr->success_cnt, sh_hr->success_dur,
		sh_hr->fail_cnt, sh_hr->fail_dur);

	return 0;
}

int sensorhub_stat_get_wu_info(
	struct sensorhub_wu_stat *sh_wu, size_t size)
{
	struct sensorhub_stat *entry;
	unsigned long flags;
	unsigned int wu_mode, wakeup;
	ktime_t subtotal_time, total_time;
	int i;

	if (!sh_wu || size < WU_MODE_MAX) {
		pr_err(SENSORHUB_STAT_PREFIX"%s: invalid arg\n", __func__);
		return -EINVAL;
	}

	memset(sh_wu, 0, sizeof(*sh_wu) * size);

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (entry->lib_number > SHUB_INST_LIB_DATA_WRISTUP_MODE_0 &&
				entry->lib_number < SHUB_INST_LIB_DATA_WRISTUP_MODE_MAX) {
			wu_mode = entry->lib_number - SHUB_INST_LIB_DATA_WRISTUP_MODE_0;
			wakeup = (unsigned int)(atomic_read(&entry->wakeup) + INT_MIN);

			subtotal_time = entry->subtotal_time;

			total_time = entry->total_time;
			if (entry->active) {
				ktime_t now, active_time;

				now = ktime_get_boottime();
				active_time = ktime_sub(now, entry->ts);
				total_time = ktime_add(total_time, active_time);
			}

			sh_wu[wu_mode].wu_mode = wu_mode;
			sh_wu[wu_mode].wakeup = wakeup;
			sh_wu[wu_mode].subtotal_time = subtotal_time;
			sh_wu[wu_mode].total_time = total_time;
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	for (i = 0; i < size; i++)
		pr_info(SENSORHUB_STAT_PREFIX"%s %d %u %u %llu %llu\n",
			__func__, i, sh_wu[i].wu_mode, sh_wu[i].wakeup,
			ktime_to_ms(sh_wu[i].subtotal_time), ktime_to_ms(sh_wu[i].total_time));

	return 0;
}

int sensorhub_stat_get_wo_info(
	struct sensorhub_wo_stat *sh_wo)
{
	struct sensorhub_stat *entry;
	unsigned long flags;
	ktime_t wear_on_time, wear_off_time;

	if (!sh_wo) {
		pr_err(SENSORHUB_STAT_PREFIX"%s: invalid arg\n", __func__);
		return -EINVAL;
	}

	wear_on_time = ktime_set(0,0);
	wear_off_time = ktime_set(0,0);

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		if (entry->lib_number == SHUB_LIB_DATA_WEARONOFF_MONITOR) {
			ktime_t now, active_time;

			wear_on_time = entry->total_time;
			wear_off_time = entry->subtotal_time;

			now = ktime_get_boottime();
			active_time = ktime_sub(now, entry->ts_boot);

			if (entry->active)
				wear_on_time = ktime_add(wear_on_time, active_time);	
			else
				wear_off_time = ktime_add(wear_off_time, active_time);
		}
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	sh_wo->wear_on_time = wear_on_time;
	sh_wo->wear_off_time = wear_off_time;

	pr_info(SENSORHUB_STAT_PREFIX"%s %llu %llu\n", __func__,
		ktime_to_ms(sh_wo->wear_off_time), ktime_to_ms(sh_wo->wear_on_time));

	return 0;
}
#else
inline int sensorhub_stat_get_stat(int type,
	struct sensorhub_stat_info *sh_stat) { return 0; }

inline int sensorhub_stat_get_gps(
	struct sensorhub_gps_stat *sh_gps) { return 0; }

inline int sensorhub_stat_get_gps_emon(int type,
	struct sensorhub_gps_stat *sh_gps) { return 0; }

inline int sensorhub_stat_get_hr_info(int type,
	struct sensorhub_hr_stat *sh_hr) { return 0; }

inline int sensorhub_stat_get_wu_info(
	struct sensorhub_wu_stat *sh_wu, size_t size) { return 0; }

inline int sensorhub_stat_get_wo_info(
	struct sensorhub_wo_stat *sh_wo) { return 0; }
#endif

int sensorhub_stat_is_wristup_wakeup(void)
{
	int ret = 0;

	if (last_wakeup_lib == SHUB_LIB_DATA_WRIST_UP) {
		last_wakeup_lib = -1;
		ret = 1;
	}
	pr_debug("%s: %d\n", __func__, ret);

	return ret;
}

static int sensorhub_stat_show(struct seq_file *m, void *v)
{
	unsigned long flags;
	unsigned int wakeup, activity, event;
	unsigned int succes_cnt, fail_cnt;
	unsigned int success_dur, fail_dur;
	unsigned int total_cnt, total_succes_cnt, total_fail_cnt;
	unsigned int total_dur, total_success_dur, total_fail_dur;
	ktime_t subtotal_time, total_time;
	struct sensorhub_stat *entry;

	seq_printf(m, "lib#    wakeup   activity        event  "
				"success_cnt  success_dur  fail_cnt  fail_dur  total_cnt  "
				"total_dur  total_success_cnt  total_success_dur  "
				"total_fail_cnt  total_fail_dur"
				"   subtotal_time      total_time"
				"  suspend_count   last_change\n");

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		wakeup = (unsigned int)(atomic_read(&entry->wakeup) + INT_MIN);
		activity = (unsigned int)(atomic_read(&entry->activity) + INT_MIN);
		event = (unsigned int)(atomic_read(&entry->event) + INT_MIN);
		succes_cnt = (unsigned int)(atomic_read(&entry->success_cnt_post_suspend) + INT_MIN);
		success_dur = (unsigned int)(atomic_read(&entry->success_dur_post_suspend) + INT_MIN);
		fail_cnt = (unsigned int)(atomic_read(&entry->fail_count_post_suspend) + INT_MIN);
		fail_dur = (unsigned int)(atomic_read(&entry->fail_dur_post_suspend) + INT_MIN);

		total_succes_cnt = (unsigned int)(atomic_read(&entry->success_cnt) + INT_MIN);
		total_success_dur = (unsigned int)(atomic_read(&entry->success_dur) + INT_MIN);
		total_fail_cnt = (unsigned int)(atomic_read(&entry->fail_cnt) + INT_MIN);
		total_fail_dur = (unsigned int)(atomic_read(&entry->fail_dur) + INT_MIN);
		total_cnt = total_succes_cnt + total_fail_cnt;
		total_dur = total_success_dur + total_fail_dur;

		subtotal_time = entry->subtotal_time;
		if (entry->lib_number == SHUB_LIB_DATA_WEARONOFF_MONITOR) {
			if (!entry->active) {
				ktime_t now, active_time;

				now = ktime_get_boottime();
				active_time = ktime_sub(now, entry->ts_boot);
				subtotal_time = ktime_add(subtotal_time, active_time);
			}
		}

		total_time = entry->total_time;
		if (entry->lib_number == SHUB_LIB_DATA_WEARONOFF_MONITOR) {
			if (entry->active) {
				ktime_t now, active_time;

				now = ktime_get_boottime();
				active_time = ktime_sub(now, entry->ts_boot);
				total_time = ktime_add(total_time, active_time);

				total_fail_dur = entry->active;
			}
		} else {
			if (entry->active) {
				ktime_t now, active_time;

				now = ktime_get_boottime();
				active_time = ktime_sub(now, entry->ts);
				total_time = ktime_add(total_time, active_time);
			}
		}

		seq_printf(m, "%4d %9u  %9u  %11u  %11u  %11u  %8u  %8u"
				"  %9u  %9u  %17u  %17u  %14u  %14u  "
				"%14llu  %14llu  "
				"%13d  %12llu\n",
				entry->lib_number, wakeup, activity, event,
				succes_cnt, success_dur, fail_cnt, fail_dur,
				total_cnt, total_dur, total_succes_cnt, total_success_dur,
				total_fail_cnt, total_fail_dur,
				ktime_to_ms(subtotal_time), ktime_to_ms(total_time),
				entry->suspend_count, ktime_to_ms(entry->ts));
	}
	spin_unlock_irqrestore(&sensorhub_lock, flags);

	return 0;
}

static int sensorhub_stat_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sensorhub_stat_show, NULL);
}

static const struct file_operations sensorhub_stat_fops = {
	.open       = sensorhub_stat_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int sensorhub_stat_pm_notifier(struct notifier_block *nb,
								unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		sensorhub_stat_pm_suspend_prepare_cb(resume_time);
		break;
	case PM_POST_SUSPEND:
		sensorhub_stat_pm_post_suspend_cb(resume_time);
		resume_time = ktime_get();
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block sensorhub_stat_notifier_block = {
	.notifier_call = sensorhub_stat_pm_notifier,
};

#ifdef CONFIG_SLEEP_MONITOR
static int continuous_hr_sleep_monitor_a_read32_cb(void *priv,
				unsigned int *raw_val, int check_level, int caller_type)
{
	unsigned long flags;
	u32 cnt = 0, success_cnt = 0, fail_cnt = 0;
	u32 dur = 0, success_dur = 0, fail_dur = 0;
	int mask = 0;
	struct sensorhub_stat *entry;

	if (caller_type != SLEEP_MONITOR_CALL_SUSPEND) {
		*raw_val = 0;
		return 0;
	}

	spin_lock_irqsave(&sensorhub_lock, flags);
	list_for_each_entry(entry, &sensorhub_list, link) {
		success_cnt = (u32)(atomic_read(&entry->success_cnt_post_suspend) + INT_MIN);
		success_dur = (u32)(atomic_read(&entry->success_dur_post_suspend) + INT_MIN);
		fail_cnt = (u32)(atomic_read(&entry->fail_count_post_suspend) + INT_MIN);
		fail_dur = (u32)(atomic_read(&entry->fail_dur_post_suspend) + INT_MIN);
	}
	cnt = success_cnt + fail_cnt;
	dur = success_dur + fail_dur;
	spin_unlock_irqrestore(&sensorhub_lock, flags);
	*raw_val = *raw_val | (success_cnt << 28);
	*raw_val = *raw_val | (fail_cnt << 24);
	mask = (2 << 24) - 1;
	*raw_val = *raw_val | ((dur > mask) ? mask : (dur & mask));

	return (cnt > 15) ? 15 : cnt;
}

static struct sleep_monitor_ops continuous_hr_sleep_monitor_a_ops = {
        .read_cb_func = continuous_hr_sleep_monitor_a_read32_cb,
};
#endif

static int __init sensorhub_stat_init(void)
{
	struct dentry *root;

	root = debugfs_create_dir("sensorhub_stat", NULL);
	if (!root) {
		pr_err(SENSORHUB_STAT_PREFIX
				"failed to create sap_pid_stat debugfs directory\n");
		return -ENOMEM;
	}

	if (!debugfs_create_file("stat", 0660, root, NULL, &sensorhub_stat_fops))
		goto error_debugfs;

	if (register_pm_notifier(&sensorhub_stat_notifier_block))
		goto error_debugfs;

#ifdef CONFIG_SLEEP_MONITOR
	sleep_monitor_register_ops(&resume_time,
			&continuous_hr_sleep_monitor_a_ops,
			SLEEP_MONITOR_CONHR);
#endif

	return 0;

error_debugfs:
	debugfs_remove_recursive(root);

	return -1;
}

late_initcall(sensorhub_stat_init);
