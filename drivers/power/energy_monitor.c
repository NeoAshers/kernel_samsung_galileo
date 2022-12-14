
/*
 * debugfs file to keep track of suspend
 *
 * Copyright (C) 2015 SAMSUNG, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the impliesd warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 *******************************************************************************
 *                                  HISTORY                                    *
 *******************************************************************************
 * ver   who                                         what                      *
 * ---- -------------------------------------------- ------------------------- *
 * 1.0   Sangin Lee <sangin78.lee@samsung.com>       Initial Release           *
 *       Sanghyeon Lee <sirano06.lee@samsung.com>                              *
 *       Junyoun Kim <junyouns.kim@samsung.com>                                *
 * ---- -------------------------------------------- ------------------------- *
 * 2.0   Hunsup Jung <hunsup.jung@samsung.com>       Remove unnecessary code   *
 *                                                   Add info - battery type   *
 *                                                            - battery temp   *
 *                                                   Remove info - status      *
 * ---- -------------------------------------------- ------------------------- *
 * 3.0   Junho Jang <vincent.jang@samsung.com>       Remove unnecessary code   *
 *                                                   Add info - sap activity, traffic   *
 *                                                   - wakeup source   *
 *                                                   - cpu time 	 *
 *                                                   - cpu frequency	 *
 *                                                   - cpu idle	 *
 *                                                   - gpu frequency	 *
 *                                                   - ff 	 *
 *                                                   - exynos sleep 	 *
 *                                                   - sensorhub event 	 *
 *                                                   - move model specific info. to DT	 *
 *                                                   - hw suspend energy estimator	 *
 *                                                   - alarm	 *
 *                                                   - netstat	 *
 * ---- -------------------------------------------- ------------------------- *
 * 3.1   Junho Jang <vincent.jang@samsung.com>        Add info - i/o counter   *
 *                                                   - input	 *
 * ---- -------------------------------------------- ------------------------- *
 * 3.2   Junho Jang <vincent.jang@samsung.com>        Add bafd   *
 * ---- -------------------------------------------- ------------------------- *
 * 3.3   Junho Jang <vincent.jang@samsung.com>        Add a net dev up time to bafd   *
 * ---- -------------------------------------------- ------------------------- *
 * 3.4   Junho Jang <vincent.jang@samsung.com>        Add wrist up wakeup count to batr   *
 *                                                   Add info - wakeup count of ws *
 * ---- -------------------------------------------- ------------------------- *
 * 3.5   Junho Jang <vincent.jang@samsung.com>        Add a wakeup index for GNSS   *
 *                                                     - wear off time *
 * ---- -------------------------------------------- ------------------------- *
 * 3.6   Junho Jang <vincent.jang@samsung.com>           *
 * ---- -------------------------------------------- ------------------------- *
 * 3.7   Junho Jang <vincent.jang@samsung.com>           *
 * ---- -------------------------------------------- ------------------------- *
 * 3.8   Junho Jang <vincent.jang@samsung.com>        Penalty score version up   *
 * ---- -------------------------------------------- ------------------------- *
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/security.h>
#include <linux/pm_wakeup.h>

#include <linux/power/energy_monitor.h>
#include <linux/battery/sec_battery.h>
#include <linux/time_history.h>
#include <linux/power/disp_stat.h>

#ifdef CONFIG_SENSORHUB_STAT
#include <linux/sensorhub_stat.h>
#endif
#ifdef CONFIG_ALARM_HISTORY
#include "../staging/android/alarm_history.h"
#endif
#ifdef CONFIG_SAP_PID_STAT
#include <linux/sap_pid_stat.h>
#endif
#ifdef CONFIG_USID_STAT
#include <linux/usid_stat.h>
#endif
#ifdef CONFIG_NET_STAT_TIZEN
#include <linux/net_stat_tizen.h>
#endif
#ifdef CONFIG_SLAVE_WAKELOCK
#include <linux/power/slave_wakelock.h>
#endif
#ifdef CONFIG_SID_SYS_STATS
#include <linux/sid_sys_stats.h>
#endif
#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
#include <linux/cpuidle_stat.h>
#endif
#ifdef CONFIG_CPU_FREQ_STAT_TIZEN
#include <linux/cpufreq_stats_tizen.h>
#endif
#ifdef CONFIG_GPUFREQ_STAT
#include <linux/gpufreq_stat.h>
#endif
#ifdef CONFIG_SENSORS_SEC_THERM_HISTORY
#include <linux/platform_data/sec_thermistor_history.h>
#endif
#ifdef CONFIG_FF_STAT_TIZEN
#include <linux/ff_stat_tizen.h>
#endif
#ifdef CONFIG_SLEEP_STAT
#include <linux/sleep_stat.h>
#endif
#ifdef CONFIG_LBS_STAT
#include <linux/lbs_stat.h>
#endif
#ifdef CONFIG_INPUT_STAT
#include <linux/power/input_stat.h>
#endif
#ifdef CONFIG_SEC_SYSFS
#include <linux/sec_sysfs.h>
static struct device *sec_energy_monitor;
#endif

#define CONFIG_ENERGY_MONITOR_WAKEUP_STAT
#define CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR

#define ENERGY_MON_VERSION "3.8"

#define ENERGY_MON_HISTORY_NUM 64
#define ENERGY_MON_MAX_MONITOR_INTERVAL 86400/* second */
#define ENERGY_MON_DEFAULT_MONITOR_INTERVAL 120/* second */
#define ENERGY_MON_DEFAULT_LOGGING_INTERVAL 3590/* second */

#define SEC_PER_HOUR 3600
#define MSEC_PER_HOUR 3600000UL
#define DEFAULT_BATTERY_CAPACITY 300 /* 300mAh */
#define DEFAULT_UNIT_BATTERY_mAs (DEFAULT_BATTERY_CAPACITY*SEC_PER_HOUR/100)

#ifdef CONFIG_ALARM_HISTORY
#define ALARM_STAT_ARRAY_SIZE 4
#endif
#ifdef CONFIG_SAP_PID_STAT
#define SAP_TRAFFIC_ARRAY_SIZE 4
#endif
#ifdef CONFIG_USID_STAT
#define TCP_TRAFFIC_ARRAY_SIZE 4
#endif
#ifdef CONFIG_NET_STAT_TIZEN
#define NET_STAT_ARRAY_SIZE 6
#endif
#ifdef CONFIG_PM_SLEEP
#define WS_ARRAY_SIZE 6
#endif
#ifdef CONFIG_SID_SYS_STATS
#define SID_CPUTIME_ARRAY_SIZE 6
#define SID_IO_ARRAY_SIZE 4
#endif
#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
#define BLOCKER_ARRAY_SIZE 4
#define BLOCKER_NAME_SIZE 9
#endif
#ifdef CONFIG_LBS_STAT
#define LBS_STAT_ARRAY_SIZE 4
#endif

/* max time supported by wakeup_stat */
#define ENERGY_MON_MAX_WAKEUP_STAT_TIME 16

/* sleep current estimator related constants */
#define ENERGY_MON_MAX_SLEEP_ESTIMATOR_CNT 20
#define MIN_SLEEP_TIME_S 500 /* second */

/* penalty score value */
#define ENERGY_MON_SCORE_VERSION "1.2"
#define ENERGY_MON_SCORE_FORCE_REBOOT	BIT(31)
#define ENERGY_MON_SCORE_SBM_FAST_DRAIN	BIT(30)
#define ENERGY_MON_SCORE_CP	BIT(20)
#define ENERGY_MON_SCORE_INPUT	BIT(19)
#define ENERGY_MON_SCORE_SH_WRISTUP	BIT(18)
#define ENERGY_MON_SCORE_ALARM_MGR	BIT(17)
#define ENERGY_MON_SCORE_ALARM	BIT(16)
#define ENERGY_MON_SCORE_CPU_TIME	BIT(15)
#define ENERGY_MON_SCORE_CPU_IDLE	BIT(14)
#define ENERGY_MON_SCORE_SOC_LPM	BIT(13)
#define ENERGY_MON_SCORE_CPU	BIT(12)
#define ENERGY_MON_SCORE_GPU	BIT(11)
#define ENERGY_MON_SCORE_TCP	BIT(10)
#define ENERGY_MON_SCORE_SAP	BIT(9)
#define ENERGY_MON_SCORE_SH	BIT(8)
#define ENERGY_MON_SCORE_DISP	BIT(7)
#define ENERGY_MON_SCORE_LBS	BIT(6)
#define ENERGY_MON_SCORE_GPS	BIT(5)
#define ENERGY_MON_SCORE_SLAVE_WAKELOCK	BIT(4)
#define ENERGY_MON_SCORE_WS	BIT(3)
#define ENERGY_MON_SCORE_SUSPEND	BIT(2)
#define ENERGY_MON_SCORE_SOC_SLEEP	BIT(1)
#define ENERGY_MON_SCORE_FAST_DRAIN	BIT(0)
#define ENERGY_MON_SCORE_MAX (ENERGY_MON_SCORE_FAST_DRAIN |\
	ENERGY_MON_SCORE_SOC_SLEEP |\
	ENERGY_MON_SCORE_SUSPEND |\
	ENERGY_MON_SCORE_WS |\
	ENERGY_MON_SCORE_SLAVE_WAKELOCK |\
	ENERGY_MON_SCORE_GPS |\
	ENERGY_MON_SCORE_LBS |\
	ENERGY_MON_SCORE_CPU_TIME |\
	ENERGY_MON_SCORE_CPU_IDLE |\
	ENERGY_MON_SCORE_SOC_LPM |\
	ENERGY_MON_SCORE_CPU |\
	ENERGY_MON_SCORE_GPU |\
	ENERGY_MON_SCORE_TCP |\
	ENERGY_MON_SCORE_SAP |\
	ENERGY_MON_SCORE_SH |\
	ENERGY_MON_SCORE_DISP |\
	ENERGY_MON_SCORE_ALARM |\
	ENERGY_MON_SCORE_SH_WRISTUP |\
	ENERGY_MON_SCORE_INPUT |\
	ENERGY_MON_SCORE_CP |\
	ENERGY_MON_SCORE_SBM_FAST_DRAIN)

#define FAIL_TO_SUSPEND (ENERGY_MON_SCORE_SUSPEND |\
	ENERGY_MON_SCORE_WS |\
	ENERGY_MON_SCORE_SLAVE_WAKELOCK)
#define FREQ_WAKEUP_FROM_SUSPEND (ENERGY_MON_SCORE_SH |\
	ENERGY_MON_SCORE_ALARM |\
	ENERGY_MON_SCORE_ALARM_MGR |\
	ENERGY_MON_SCORE_SH_WRISTUP |\
	ENERGY_MON_SCORE_INPUT)

#define CHECK_SCORE_BIT(SCORE, BIT) ((SCORE) & (BIT))

#define energy_mon_dbg(debug_level_mask, fmt, ...)\
do {\
	if (debug_level & debug_level_mask)\
		pr_info("energy_mon_%d: " fmt, debug_level_mask, ##__VA_ARGS__);\
} while (0)

enum {
	SKIP_PANALTY_CHECK_BOOT_TIME = BIT(0),
	SKIP_PANALTY_CHECK_KERNEL_TIME = BIT(1),
	SKIP_PANALTY_CHECK_THRESHOLD_CHARNGE = BIT(2),
	SKIP_PANALTY_CHECK_FILE_TRANSFER = BIT(3)
};

enum ENERGY_MON_DEBUG_LEVEL {
	ENERGY_MON_DEBUG_INFO = BIT(0),
	ENERGY_MON_DEBUG_ERR = BIT(1),
	ENERGY_MON_DEBUG_WARN = BIT(2),
	ENERGY_MON_DEBUG_DBG = BIT(3),
	ENERGY_MON_DEBUG_WAKEUP_STAT = BIT(4),
	ENERGY_MON_DEBUG_SLEEP_ESTIMATOR = BIT(5),
};

enum energy_mon_print_step {
	STEP_SUMMARY = 0,
	STEP_DISP_STAT,
#ifdef CONFIG_INPUT_STAT
	STEP_INPUT_STAT,
#endif
#ifdef CONFIG_SENSORHUB_STAT
	STEP_SENSORHUB_WAKEUP,
	STEP_SENSORHUB_ACTIVITY,
#endif
#ifdef CONFIG_ALARM_HISTORY
	STEP_ALARM_STAT,
#endif
#ifdef CONFIG_SAP_PID_STAT
	STEP_SAP_WAKEUP,
	STEP_SAP_ACTIVITY,
	STEP_SAP_TRAFFIC,
#endif
#ifdef CONFIG_USID_STAT
	STEP_USID_STAT,
#endif
#ifdef CONFIG_NET_STAT_TIZEN
	STEP_NET_STAT1,
	STEP_NET_STAT2,
	STEP_NET_STAT3,
#endif
#ifdef CONFIG_SLAVE_WAKELOCK
	STEP_SLAVE_WAKELOCK,
#endif
#ifdef CONFIG_PM_SLEEP
	STEP_WAKEUP_SOURCE,
	STEP_WAKEUP_COUNT,
#endif
#ifdef CONFIG_SID_SYS_STATS
	STEP_SID_CPUTIME_STATS1,
#ifdef CONFIG_PID_SYS_STATS
	STEP_SID_CPUTIME_STATS2,
#endif
	STEP_SID_IO_STATS,
#endif
#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
	STEP_CPUIDLE_STAT,
	STEP_CPUIDLE_LPM_BLOCKER,
#endif
#ifdef CONFIG_CPU_FREQ_STAT_TIZEN
	STEP_CPUFREQ_STATS,
#endif
#ifdef CONFIG_GPUFREQ_STAT
	STEP_GPUFREQ_STAT,
#endif
#ifdef CONFIG_SENSORS_SEC_THERM_HISTORY
	STEP_SEC_THERM_HISTORY,
#endif
#ifdef CONFIG_FF_STAT_TIZEN
	STEP_FF_STAT,
#endif
#ifdef CONFIG_SLEEP_STAT
	STEP_SLEEP_STAT,
#endif
#ifdef CONFIG_SENSORHUB_STAT
	STEP_GPS_STAT,
#endif
#ifdef CONFIG_LBS_STAT
	STEP_LBS_STAT,
#endif
	STEP_CE,
	STEP_MAX
};

enum energy_mon_print_type {
	ENERGY_MON_PRINT_TITLE = 0,
	ENERGY_MON_PRINT_MAIN,
	ENERGY_MON_PRINT_TAIL,
};

struct consumed_energy {
	unsigned int sleep;
	unsigned int disp_aod;
	unsigned int disp_on;
	unsigned int cpu_idle;
	unsigned int cpu_active;
	unsigned int gpu;
	unsigned int ff;
	unsigned int gps;
	unsigned int lbs;
	unsigned int hr;
	unsigned int bt;
	unsigned int wifi;
	unsigned int cp;
};

struct energy_mon_data {
	int type;
	int log_count;
	int suspend_count;
	int bat_status;
	int bat_capacity;
	int cable_type;
	int bat_temp;
#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
	int current_suspend; /* mAh */
#endif
	struct timespec ts_real;
	struct timespec ts_boot;
	struct timespec ts_kern;
	struct timespec ts_disp;
	struct timespec ts_aod;
	int wakeup_cause[ENERGY_MON_WAKEUP_MAX];
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	ktime_t wakeup_time[ENERGY_MON_WAKEUP_MAX];
#endif
	struct disp_stat_emon disp_stat;
#ifdef CONFIG_INPUT_STAT
	struct input_stat_emon input_stat;
#endif
#ifdef CONFIG_SENSORHUB_STAT
	struct sensorhub_stat_info sh_stat;
	struct sensorhub_wo_stat sh_wo;
#endif
#ifdef CONFIG_ALARM_HISTORY
	struct alarm_expire_stat alarm_stat[ALARM_STAT_ARRAY_SIZE];
#endif
#ifdef CONFIG_SAP_PID_STAT
	struct sap_pid_wakeup_stat sap_wakeup;
	struct sap_stat_traffic sap_traffic[SAP_TRAFFIC_ARRAY_SIZE];
#endif
#ifdef CONFIG_USID_STAT
	struct usid_stat_traffic tcp_traffic[TCP_TRAFFIC_ARRAY_SIZE];
#endif
#ifdef CONFIG_NET_STAT_TIZEN
	struct net_stat_tizen_emon net_stat[NET_STAT_ARRAY_SIZE];
#endif
#ifdef CONFIG_SLAVE_WAKELOCK
	struct emon_slave_wakelock slwl[SLWL_ARRAY_SIZE];
#endif
#ifdef CONFIG_PM_SLEEP
	struct emon_wakeup_source ws[WS_ARRAY_SIZE];
	struct emon_wakeup_source ws_wu[WS_ARRAY_SIZE];
#endif
#ifdef CONFIG_SID_SYS_STATS
	struct sid_sys_stats sid_cpu[SID_CPUTIME_ARRAY_SIZE];
#ifdef CONFIG_PID_SYS_STATS
	struct sid_sys_stats pid_cpu[SID_CPUTIME_ARRAY_SIZE];
#endif
	struct sid_io_stats sid_io[SID_IO_ARRAY_SIZE];
#endif
#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
	struct emon_cpuidle_stat cpuidle_stat;
	struct emon_lpm_blocker lpm_blocker[BLOCKER_ARRAY_SIZE];
#endif
#ifdef CONFIG_CPU_FREQ_STAT_TIZEN
	struct cpufreq_stats_tizen cpufreq_stats;
#endif
#ifdef CONFIG_GPUFREQ_STAT
	struct gpufreq_stat gpufreq_stat;
#endif
#ifdef CONFIG_SENSORS_SEC_THERM_HISTORY
	struct sec_therm_history_info therm_history[MAX_SEC_THERM_DEVICE_NUM];
#endif
#ifdef CONFIG_FF_STAT_TIZEN
	struct ff_stat_emon ff_stat;
#endif
#ifdef CONFIG_SLEEP_STAT
	struct sleep_stat sleep_stat;
#endif
#ifdef CONFIG_SENSORHUB_STAT
	struct sensorhub_gps_stat sh_gps;
#endif
#ifdef CONFIG_LBS_STAT
	struct lbs_stat_emon lbs_stat[LBS_STAT_ARRAY_SIZE];
#endif

	struct consumed_energy ce;
	unsigned int penalty_score;
};

struct energy_mon_penalty {
	unsigned int last_score;
	unsigned int last_score_index;
	unsigned int notify_mask;
	int threshold_time;
	int threshold_short_cpu_usage;
	int threshold_batt;
	int threshold_sbm_batt;
	int threshold_disp;
	int threshold_input;
	int threshold_alarm_mgr;
	int threshold_alarm;
	int threshold_sh;
	int threshold_sh_wristup;
	int threshold_gps;
	int threshold_lbs;
	int threshold_cpuidle;
	int threshold_cpu;
	int threshold_gpu;
	int threshold_ws;
	int threshold_ws_wu;
	int threshold_slwl;
	int threshold_suspend_fail;
	int threshold_sid;
	int threshold_pid;
	int threshold_booting_offset;
	int threshold_log_offset;
	int threshold_fr_wakeup[ENERGY_MON_WAKEUP_MAX];
	int threshold_fr_suspend_fail;
	int threshold_fr_suspend_failed_freeze;
	int threshold_fr_suspend_failed_prepare;
	int threshold_fr_suspend_failed_suspend;
	int threshold_fr_suspend_failed_suspend_late;
	int threshold_fr_suspend_failed_suspend_noirq;
	int threshold_fr_suspend_success;

	ktime_t last_threshold_change;
};

struct energy_mon_irq_map_table {
	const char *irq_name;
	int wakeup_idx;
};

struct fast_batt_drain_cause {
	unsigned int cpu_sid;
	unsigned int lbs_sid;
	char slwl_name[SLWL_NAME_LENGTH];
	char ws_name[WS_NAME_SIZE];
	unsigned int soc_sleep;
};

struct disp_power_value {
	unsigned int brightness;
	unsigned int value;
};

/* TODO: Currently we only support 2 cpus */
#define MAX_SUPPORT_CORE 2
struct active_power_value {
	unsigned int speed;
	unsigned int value[MAX_SUPPORT_CORE];
};

struct cp_power_value {
	unsigned int signal;
	unsigned int value;
};

struct energy_mon_power_value {
	unsigned int disp_aod;
	unsigned int disp_on_table_size;
	struct disp_power_value *disp_on;
	unsigned int cpu_sleep;
	unsigned int cpu_lpm;
	unsigned int cpu_idle;
	unsigned int cpu_active_table_size;
	struct active_power_value *cpu_active;
	unsigned int cpu_whitelist_table_size;
	unsigned int *cpu_whitelist;
	unsigned int gpu_active_table_size;
	struct active_power_value *gpu_active;
	unsigned int bt_active;
	unsigned int bt_tx;
	unsigned int bt_rx;
	unsigned int wifi_scan;
	unsigned int ff;
	unsigned int gps;
	unsigned int hr;
	unsigned int cp_active_table_size;
	struct cp_power_value *cp_active;
};

struct energy_monitor_dqa {
	int net_stat_cp1_idx;
	int net_stat_cp2_idx;
	int net_stat_wifi_idx;
	int net_stat_bt_idx;
};

struct energy_monitor_cb {
	int running;
	int data_index;
	int read_index;

	int charging_count;
	int discharging_count;
	int not_charging_count;
	int full_count;

	int wakeup_cause[ENERGY_MON_WAKEUP_MAX];
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	int last_wakeup_idx;
	ktime_t last_wakeup_time;
	ktime_t wakeup_time[ENERGY_MON_WAKEUP_MAX];
	ktime_t prev_wakeup_time[ENERGY_MON_WAKEUP_MAX]; /* should be same size as wakeup_time */
	unsigned int wakeup_time_stats[ENERGY_MON_WAKEUP_MAX][ENERGY_MON_MAX_WAKEUP_STAT_TIME + 1];
#endif
#ifdef CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR
	int estimator_index;
	long long estimator_average[ENERGY_MON_MAX_SLEEP_ESTIMATOR_CNT];
#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
	int current_suspend_cnt;
	int current_suspend_sum;
#endif
#endif
	int disp_state;
	struct timespec disp_total;
	struct timespec disp_last_on;

	struct energy_mon_data boot;
	struct energy_mon_data data[ENERGY_MON_HISTORY_NUM];
	struct energy_mon_data dump;

	struct energy_mon_data charging;
	struct energy_mon_data discharging;
	struct energy_mon_data not_charging;
	struct energy_mon_data full;

	struct energy_mon_data charging_dump;
	struct energy_mon_data discharging_dump;
	struct energy_mon_data not_charging_dump;
	struct energy_mon_data full_dump;

	struct fast_batt_drain_cause fbdc;
	struct energy_monitor_dqa dqa;

	struct wakeup_source *emon_ws;

	/* get the following info from device tree */
	unsigned int bat_size;
	unsigned int capacity_per_soc; /* uAh */
	long long unit_bat_capacity;
	int use_raw_soc;
	const char *ps_raw_soc;
	const char *ps_hw_suspend_energy;
	struct energy_mon_penalty penalty;
	int irq_map_table_size;
	struct energy_mon_irq_map_table *irq_map_table;
	struct energy_mon_power_value power_value;
};

/* global variable */
static struct energy_monitor_cb energy_mon;

static int energy_monitor_enable = 1;
static unsigned int monitor_interval; 	/* 0: Disable, 1~86400: Enable */
static unsigned int logging_interval;	/* 0: Disable, 1~86400: Enable */
static struct delayed_work monitor_work;

static int debug_level = ENERGY_MON_DEBUG_INFO |
					ENERGY_MON_DEBUG_ERR |
					ENERGY_MON_DEBUG_WARN |
					ENERGY_MON_DEBUG_SLEEP_ESTIMATOR |
					ENERGY_MON_DEBUG_WAKEUP_STAT;

static void energy_mon_get_time_info(struct energy_mon_data *p_curr)
{
	if (!p_curr) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: pointer is NULL\n", __func__);
		return;
	}

	p_curr->ts_real = current_kernel_time();
	get_monotonic_boottime(&p_curr->ts_boot);
	p_curr->ts_kern = ktime_to_timespec(ktime_get());
	p_curr->ts_disp = disp_stat_get_fb_total_time();
	p_curr->ts_aod = disp_stat_get_aod_total_time();
	p_curr->suspend_count = suspend_stats.success;

	/* Debug logs */
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s\n", __func__);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "ts_real: %15lu.%09lu\n", p_curr->ts_real.tv_sec,
																   p_curr->ts_real.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "ts_boot: %15lu.%09lu\n", p_curr->ts_boot.tv_sec,
																   p_curr->ts_boot.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "ts_kern: %15lu.%09lu\n", p_curr->ts_kern.tv_sec,
																   p_curr->ts_kern.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "ts_disp: %15lu.%09lu\n", p_curr->ts_disp.tv_sec,
																   p_curr->ts_disp.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "ts_aod: %15lu.%09lu\n", p_curr->ts_aod.tv_sec,
																  p_curr->ts_aod.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "suspend_count=%04d\n", p_curr->suspend_count);
}

static void energy_mon_get_battery_info(struct energy_mon_data *p_curr)
{
	struct power_supply *psy = NULL;
	union power_supply_propval value;
	enum power_supply_property props[] = {
		POWER_SUPPLY_PROP_STATUS,
		POWER_SUPPLY_PROP_ONLINE,
		POWER_SUPPLY_PROP_TEMP,
		POWER_SUPPLY_PROP_CAPACITY
	};
	int i, err;

	if (!p_curr) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: pointer is NULL\n", __func__);
		return;
	}

	psy = power_supply_get_by_name("battery");
	if (!psy) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: cannot find battery power supply\n", __func__);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		if (power_supply_get_property(psy, props[i], &value))
			continue;

		switch (props[i]) {
		case POWER_SUPPLY_PROP_STATUS:
			p_curr->bat_status = value.intval;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			p_curr->cable_type = value.intval;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			p_curr->bat_temp = value.intval;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			p_curr->bat_capacity = value.intval * 100;
			break;
		default:
			break;
		}
	}
	power_supply_put(psy);

	if (energy_mon.use_raw_soc) {
		psy = power_supply_get_by_name(energy_mon.ps_raw_soc);
		if (psy) {
			value.intval = SEC_FUELGAUGE_CAPACITY_TYPE_RAW;
			err = power_supply_get_property(psy,
					POWER_SUPPLY_PROP_CAPACITY, &value);
			p_curr->bat_capacity = value.intval;
			power_supply_put(psy);
		} else
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s: cannot find fg power supply\n", __func__);
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "bat_stat=%d, bat_capa=%03d\n",
		p_curr->bat_status,
		p_curr->bat_capacity);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "cable_type=%d, bat_temp=%03d\n",
		p_curr->cable_type,
		p_curr->bat_temp);

#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
		if (energy_mon.current_suspend_cnt > 1)
			p_curr->current_suspend =
				energy_mon.current_suspend_sum / energy_mon.current_suspend_cnt;
		energy_mon.current_suspend_cnt = 0;
		energy_mon.current_suspend_sum = 0;
#endif
}

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
static void energy_mon_get_wakeup_stat(
	struct energy_mon_data *p_curr, int backup_stat)
{
	int i;

	if (!p_curr) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: p_curr is NULL\n", __func__);
		return;
	}

	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++) {
		p_curr->wakeup_time[i] =
			ktime_sub(energy_mon.wakeup_time[i],
						energy_mon.prev_wakeup_time[i]);
	}

	if (backup_stat)
		memcpy(energy_mon.prev_wakeup_time,
				energy_mon.wakeup_time,
				sizeof(energy_mon.prev_wakeup_time));
}
#endif

static int calculate_permil(s64 residency, s64 profile_time)
{
	if (!residency)
		return 0;

	residency *= 1000;
	do_div(residency, profile_time);

	return (int)residency;
}

static int energy_monitor_get_log_offset(struct energy_mon_data *p_curr)
{
	int i;
	int offset = 0;

	for (i = 0; i < SAP_TRAFFIC_ARRAY_SIZE; i++) {
		/* hard code consumer name for now, need to get from device tree */
		if (!strncmp(p_curr->sap_traffic[i].aspid.id, "[FT]hostmanager", 15))
			continue;

		offset = p_curr->sap_traffic[i].snd / 1000000 * energy_mon.penalty.threshold_log_offset;
		return offset;
	}

	return offset;
}

static unsigned int energy_monitor_calc_penalty_battery(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	int diff_soc;
	long diff_boot = 0;
	long long average;
	int aod_offset = 0;
	int booting_offset = 0;
	int log_offset = 0;

	if (skip_penalty_check &
			(SKIP_PANALTY_CHECK_BOOT_TIME |
			SKIP_PANALTY_CHECK_THRESHOLD_CHARNGE |
			SKIP_PANALTY_CHECK_FILE_TRANSFER))
		return score;

	diff_soc = p_curr->bat_capacity - p_prev->bat_capacity;
	diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;

	average = abs(energy_mon.unit_bat_capacity *
		(p_curr->bat_capacity - p_prev->bat_capacity));
	do_div(average, diff_boot);

	/*
	 * penalty_score scheme
	 *	 - notify fast battery drain when average current above
	 *        threshold_batt + aod_offset + booting_offset + log_offset
	 */
	if (diff_soc < 0) {
		if (p_curr->ce.disp_aod > 0)
			aod_offset = p_curr->ce.disp_aod;
		if (p_curr->log_count == 0)
			booting_offset = energy_mon.penalty.threshold_booting_offset;
		log_offset = energy_monitor_get_log_offset(p_curr);

		if (average * 10 >= (long long)energy_mon.penalty.threshold_batt +
			aod_offset + booting_offset + log_offset)
			score = ENERGY_MON_SCORE_FAST_DRAIN;

		if (average * 10 >= (long long)energy_mon.penalty.threshold_sbm_batt +
			aod_offset + booting_offset)
			score |= ENERGY_MON_SCORE_SBM_FAST_DRAIN;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %lld %d %d %d %d %d\n", __func__,
		p_curr->log_count , score, average,
		energy_mon.penalty.threshold_batt,
		energy_mon.penalty.threshold_sbm_batt,
		aod_offset, booting_offset, log_offset);

	return score;
}

#ifdef CONFIG_DISP_STAT
static unsigned int energy_monitor_calc_penalty_disp(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time;
	unsigned int uAh = 0, aod_uAh;
	int i;

	if (energy_mon.power_value.disp_on_table_size > 0) {
		for (i = 0; i < energy_mon.power_value.disp_on_table_size; i++) {
			power_value = energy_mon.power_value.disp_on[i].value;
			residence_time = ktime_to_ms(p_curr->disp_stat.fb_time[i]);

			uAh += power_value * residence_time / MSEC_PER_HOUR;
		}

		p_curr->ce.disp_on = uAh;

		if (uAh > energy_mon.penalty.threshold_disp)
			score = ENERGY_MON_SCORE_DISP;
	}

	power_value = energy_mon.power_value.disp_aod;
	residence_time = ktime_to_ms(p_curr->disp_stat.aod_time);

	aod_uAh = power_value * residence_time / SEC_PER_HOUR;

	p_curr->ce.disp_aod = aod_uAh;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %u %u\n", __func__,
		p_curr->log_count, score, uAh, aod_uAh);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_disp(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_INPUT_STAT
static unsigned int energy_monitor_calc_penalty_input(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	int input_wakeup;

	input_wakeup = p_curr->wakeup_cause[ENERGY_MON_WAKEUP_INPUT] -
					p_prev->wakeup_cause[ENERGY_MON_WAKEUP_INPUT];

	if (input_wakeup > 0 && input_wakeup > energy_mon.penalty.threshold_input)
		score = ENERGY_MON_SCORE_INPUT;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x %d %d\n", __func__,
		p_curr->log_count, score,
		input_wakeup, energy_mon.penalty.threshold_input);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_input(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_SENSORHUB_STAT
static unsigned int energy_monitor_calc_penalty_sh(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	int sh_wakeup, sh_wakeup_wristup;

	sh_wakeup = p_curr->wakeup_cause[ENERGY_MON_WAKEUP_SSP] -
					p_prev->wakeup_cause[ENERGY_MON_WAKEUP_SSP];
	sh_wakeup_wristup = p_curr->wakeup_cause[ENERGY_MON_WAKEUP_WU] -
					p_prev->wakeup_cause[ENERGY_MON_WAKEUP_WU];

	if (sh_wakeup > 0 && sh_wakeup > energy_mon.penalty.threshold_sh)
		score |= ENERGY_MON_SCORE_SH;

	if (sh_wakeup_wristup > 0 && sh_wakeup_wristup > energy_mon.penalty.threshold_sh_wristup)
		score |= ENERGY_MON_SCORE_SH_WRISTUP;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x %d %d %d %d\n", __func__,
		p_curr->log_count, score,
		sh_wakeup, energy_mon.penalty.threshold_sh,
		sh_wakeup_wristup, energy_mon.penalty.threshold_sh_wristup);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_sh(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_ALARM_HISTORY
static unsigned int energy_monitor_is_alarm_whitelist(char *comm)
{
	/* hard code comm for now, need to get from device tree */
	if (strncmp(comm, "starter", sizeof("starter")) == 0)
		return 1;

	return 0;
}

static unsigned int energy_monitor_is_alarm_watchface(char *comm)
{
	/* hard code comm for now, need to get from device tree */
	if (strncmp(comm, "watchface-load", sizeof("watchface-load")) == 0 ||
			strncmp(comm, "com.watchface.", sizeof("com.watchface.")) == 0 ||
			strncmp(comm, "w-clock-viewer", sizeof("w-clock-viewer")) == 0)
		return 1;

	return 0;
}

static unsigned int energy_monitor_calc_penalty_alarm(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	int i;
	unsigned int score = 0;
	unsigned int residence_time;
	unsigned int aod_offset;
	int alarm_wakeup, alarm_mgr_wakeup;
	int badness;

	/*if (skip_penalty_check)
		return score;*/

	residence_time = ktime_to_ms(p_curr->disp_stat.aod_time);
	aod_offset = residence_time / 60000;
	alarm_wakeup = p_curr->wakeup_cause[ENERGY_MON_WAKEUP_RTC] -
					p_prev->wakeup_cause[ENERGY_MON_WAKEUP_RTC];
	alarm_mgr_wakeup = p_curr->alarm_stat[0].total_wakeup_count;

	badness = alarm_wakeup - alarm_mgr_wakeup;
	if (badness > 0 && badness > energy_mon.penalty.threshold_alarm)
		score = ENERGY_MON_SCORE_ALARM;

	for (i = 0; i < ALARM_STAT_ARRAY_SIZE; i++) {
		if (energy_monitor_is_alarm_whitelist(p_curr->alarm_stat[i].comm))
			continue;

		if (energy_monitor_is_alarm_watchface(p_curr->alarm_stat[i].comm)) {
			if (p_curr->alarm_stat[i].wakeup_count >
					energy_mon.penalty.threshold_alarm_mgr + aod_offset)
				score |= ENERGY_MON_SCORE_ALARM_MGR;
		} else {
			if (i == 0) {
				if (p_curr->alarm_stat[i].wakeup_count >
						energy_mon.penalty.threshold_alarm_mgr + aod_offset)
					score |= ENERGY_MON_SCORE_ALARM_MGR;
			} else {
				if (p_curr->alarm_stat[i].wakeup_count >
						energy_mon.penalty.threshold_alarm_mgr)
					score |= ENERGY_MON_SCORE_ALARM_MGR;
			}
		}
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x %d %d %d %d %d %d\n", __func__,
		p_curr->log_count, score,
		badness, alarm_wakeup, alarm_mgr_wakeup,
		energy_mon.penalty.threshold_alarm,
		energy_mon.penalty.threshold_alarm_mgr, aod_offset);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_alarm(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_SAP_PID_STAT
static unsigned int energy_monitor_calc_penalty_sap(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;

	if (skip_penalty_check)
		return score;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x\n", __func__, p_curr->log_count, score);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_sap(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_USID_STAT
static unsigned int energy_monitor_calc_penalty_tcp(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;

	if (skip_penalty_check)
		return score;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x\n", __func__, p_curr->log_count, score);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_tcp(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_SLAVE_WAKELOCK)
static unsigned int energy_monitor_is_slwl_whitelist(char *comm)
{
	/* hard code comm for now, need to get from device tree */
	if (strncmp(comm, "gpsd", sizeof("gpsd")) == 0)
		return 1;

	return 0;
}

static unsigned int energy_monitor_calc_penalty_ws_slwl(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	long diff_boot;
	long ws_time;
	long slwl_time;
	long long permil;

	if (skip_penalty_check & SKIP_PANALTY_CHECK_BOOT_TIME)
		return score;

	/* add a temporary fix for false detection when ws is used for gpsd */
	if (energy_mon.penalty.threshold_batt >= 30)
		return score;

	diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;
	ws_time = ktime_to_timeval(p_curr->ws[0].emon_total_time).tv_sec;

	permil = ws_time * 1000;
	do_div(permil, diff_boot);

	if (permil > energy_mon.penalty.threshold_ws) {
		score = ENERGY_MON_SCORE_WS;
		memset(energy_mon.fbdc.ws_name, 0, WS_NAME_SIZE);
		memcpy(energy_mon.fbdc.ws_name,
				p_curr->ws[0].name, WS_NAME_SIZE - 1);
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %lld %d\n", __func__,
		p_curr->log_count, score, permil, energy_mon.penalty.threshold_ws);

	slwl_time = ktime_to_timeval(p_curr->slwl[0].prevent_time).tv_sec;

	permil = slwl_time * 1000;
	do_div(permil, diff_boot);

	if (permil > energy_mon.penalty.threshold_slwl) {
		if (energy_monitor_is_slwl_whitelist(p_curr->slwl[0].slwl_name))
			score = 0;
		else {
			score |= ENERGY_MON_SCORE_SLAVE_WAKELOCK;
			memset(energy_mon.fbdc.slwl_name, 0, SLWL_NAME_LENGTH);
			memcpy(energy_mon.fbdc.slwl_name,
					p_curr->slwl[0].slwl_name, SLWL_NAME_LENGTH - 1);
		}
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %lld %d\n", __func__,
		p_curr->log_count, score, permil, energy_mon.penalty.threshold_slwl);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_ws_slwl(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
static unsigned int energy_monitor_calc_penalty_exynos_cpuidle(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	int permil;
	ktime_t active;
	ktime_t idle;
	ktime_t lpm;
	ktime_t zero = ktime_set(0,0);

	if (skip_penalty_check &
			(SKIP_PANALTY_CHECK_BOOT_TIME |
			SKIP_PANALTY_CHECK_KERNEL_TIME))
		return score;

	active = p_curr->cpuidle_stat.total_stat_time;
	idle = p_curr->cpuidle_stat.cpuidle[0].total_idle_time;
	permil = calculate_permil(ktime_to_ms(idle), ktime_to_ms(active));
	if (permil <= energy_mon.penalty.threshold_cpuidle)
		score = ENERGY_MON_SCORE_CPU_IDLE;

	lpm = p_curr->cpuidle_stat.lpm.usage.time;
	if (ktime_compare(lpm, zero) == 0)
		score |= ENERGY_MON_SCORE_SOC_LPM;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %d %d\n", __func__,
		p_curr->log_count, score, permil, energy_mon.penalty.threshold_cpuidle);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_exynos_cpuidle(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_CPU_FREQ_STAT_TIZEN
static unsigned int energy_monitor_get_cpu_active_power_value(
	unsigned int clk, unsigned int core)
{
	int i;

	/* TODO: Currently we only support 2 cpus */
	if (core >= MAX_SUPPORT_CORE) {
		return 0;
	}

 	if (energy_mon.power_value.cpu_active_table_size > 0) {
		for (i = 0; i < energy_mon.power_value.cpu_active_table_size; i++) {
			if (energy_mon.power_value.cpu_active[i].speed == clk)
				return energy_mon.power_value.cpu_active[i].value[core];
		}
	}

	return 0;
}

static unsigned int energy_monitor_calc_penalty_cpu(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time;
	int i, j;
	long uAh_sleep, diff_boot, diff_kernel, sleep_sec;
	s64 uAh_lpm, lpm_ms;
	s64 uAh_idle, idle_ms;
	s64 uAh_active[MAX_SUPPORT_CORE] = {0,};
	s64 real_active_ms[MAX_SUPPORT_CORE];
	s64 active_ms;
	ktime_t active;

	power_value = energy_mon.power_value.cpu_sleep;
	diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;
	diff_kernel = p_curr->ts_kern.tv_sec - p_prev->ts_kern.tv_sec;
	sleep_sec = diff_boot - diff_kernel;
	uAh_sleep = power_value * sleep_sec / SEC_PER_HOUR;
	p_curr->ce.sleep = uAh_sleep;

	power_value = energy_mon.power_value.cpu_lpm;
	lpm_ms = ktime_to_ms(p_curr->cpuidle_stat.lpm.usage.time);
	uAh_lpm = power_value * lpm_ms / MSEC_PER_HOUR;

	power_value = energy_mon.power_value.cpu_idle;
	idle_ms = ktime_to_ms(p_curr->cpuidle_stat.cpuidle[0].total_idle_time);
	uAh_idle = power_value * idle_ms / MSEC_PER_HOUR;
	p_curr->ce.cpu_idle = uAh_lpm + uAh_idle;

	active = p_curr->cpuidle_stat.cpuidle[0].hp.online_time;
	active_ms = ktime_to_ms(active);
	p_curr->ce.cpu_active = 0;
	for (i = 0; i < MAX_SUPPORT_CORE /*p_curr->cpuidle_stat.cpu_count*/; i++) {
		real_active_ms[i] = ktime_ms_delta(
			p_curr->cpuidle_stat.cpuidle[i].hp.online_time,
			p_curr->cpuidle_stat.cpuidle[i].total_idle_time);
		for (j = 0; j < p_curr->cpufreq_stats.state_num; j++) {
			power_value = energy_monitor_get_cpu_active_power_value(
							p_curr->cpufreq_stats.freq_table[j], i);
			residence_time = jiffies_to_msecs(
								p_curr->cpufreq_stats.time_in_state[j]);
			uAh_active[i] +=
				power_value * residence_time / SEC_PER_HOUR *
				real_active_ms[i] / active_ms;
		}
		p_curr->ce.cpu_active += uAh_active[i];
	}

	if (p_curr->ce.cpu_idle + p_curr->ce.cpu_active >
			energy_mon.penalty.threshold_cpu)
		score = ENERGY_MON_SCORE_CPU;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %llu %lld %lld %lld %lld %lld %lld %lld\n",
		__func__, p_curr->log_count, score,
		uAh_lpm, uAh_idle, uAh_active[0], uAh_active[1],
		lpm_ms, idle_ms, real_active_ms[0], real_active_ms[1]);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_cpu(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_GPUFREQ_STAT
static unsigned int energy_monitor_get_gpu_active_power_value(
	unsigned int clk, unsigned int core)
{
	int i;

	/* TODO: Currently we only support 2 cpus */
	if (core >= MAX_SUPPORT_CORE) {
		return 0;
	}

 	if (energy_mon.power_value.gpu_active_table_size > 0) {
		for (i = 0; i < energy_mon.power_value.gpu_active_table_size; i++) {
			if (energy_mon.power_value.gpu_active[i].speed == clk)
				return energy_mon.power_value.gpu_active[i].value[core];
		}
	}

	return 0;
}

static unsigned int energy_monitor_calc_penalty_gpu(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time;
	int residence_time_gpu = 0;
	struct timespec residence_time_disp;
	unsigned int residence_time_aod;
	unsigned int aod_offset;
	unsigned int uAh = 0;
	int permil = 0;
	int i;

	if (skip_penalty_check & SKIP_PANALTY_CHECK_KERNEL_TIME)
		return score;

	for (i = 0; i < p_curr->gpufreq_stat.table_size; i++) {
		power_value = energy_monitor_get_gpu_active_power_value(
						p_curr->gpufreq_stat.table[i].clock, 0);
		residence_time = jiffies_to_msecs(p_curr->gpufreq_stat.table[i].time);
		residence_time_gpu += residence_time;

		uAh += power_value * residence_time / SEC_PER_HOUR;
	}
	p_curr->ce.gpu = uAh;

	residence_time_aod = ktime_to_ms(p_curr->disp_stat.aod_time);
	aod_offset = (residence_time_aod / 60000) * 40;
	residence_time_gpu -= aod_offset;

	if (residence_time_gpu > 0) {
		residence_time_gpu /= 1000;
		residence_time_disp = timespec_sub(p_curr->ts_disp, p_prev->ts_disp);
		permil = calculate_permil((s64)residence_time_gpu, residence_time_disp.tv_sec);
	}

	if (permil > energy_mon.penalty.threshold_gpu)
		score = ENERGY_MON_SCORE_GPU;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %u %d %d %d\n", __func__,
		p_curr->log_count, score, uAh, permil,
		energy_mon.penalty.threshold_gpu, aod_offset);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_gpu(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_SLEEP_STAT
static unsigned int energy_monitor_calc_penalty_sleep(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	int i;

	if (p_curr->sleep_stat.fail >= energy_mon.penalty.threshold_suspend_fail)
		score |= ENERGY_MON_SCORE_SUSPEND;

	if (skip_penalty_check & SKIP_PANALTY_CHECK_BOOT_TIME)
		goto skip_check;

	if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS) {
		if ((skip_penalty_check & SKIP_PANALTY_CHECK_KERNEL_TIME) == 0) {
			if (p_curr->sleep_stat.soc.exynos.acpm_sleep_mif_down == 0 ||
				p_curr->sleep_stat.soc.exynos.acpm_sleep_soc_down == 0 ||
				p_curr->sleep_stat.soc.exynos.acpm_sicd_mif_down == 0 ||
				p_curr->sleep_stat.soc.exynos.acpm_sicd_soc_down == 0)
				score = ENERGY_MON_SCORE_SOC_SLEEP;
		}

		for (i = 0; i < (EXYNOS_MIF_MASTER_MAX - 2); i++) {
			if (i == 2) {
				if (p_curr->sleep_stat.soc.exynos.apm[i].total_time > 360 * 1000 &&
					p_curr->sleep_stat.soc.exynos.apm[i].total_time < 3700 * 1000)
					score = ENERGY_MON_SCORE_SOC_SLEEP;
			} else {
				if (p_curr->sleep_stat.soc.exynos.apm[i].total_time > 1800 * 1000 &&
					p_curr->sleep_stat.soc.exynos.apm[i].total_time < 3700 * 1000)
					score = ENERGY_MON_SCORE_SOC_SLEEP;
			}
		}
	}

skip_check:
	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x %d %d\n", __func__, p_curr->log_count, score,
		p_curr->sleep_stat.fail, energy_mon.penalty.threshold_suspend_fail);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_sleep(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_FF_STAT_TIZEN
static unsigned int energy_monitor_calc_penalty_ff(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time;
	unsigned int uAh = 0;

	power_value = energy_mon.power_value.ff;
	residence_time = ktime_to_ms(p_curr->ff_stat.total_time);
	uAh = power_value * residence_time / SEC_PER_HOUR;
	p_curr->ce.ff = uAh;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %u\n", __func__, p_curr->log_count, score, uAh);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_ff(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_SENSORHUB_STAT
static unsigned int energy_monitor_calc_penalty_gps(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time = 0;
	unsigned int uAh = 0;
	long diff_boot;
	long gps_time;
	int permil;

	power_value = energy_mon.power_value.gps;

	residence_time = p_curr->sh_gps.gps_time;
	if (residence_time > SEC_PER_HOUR)
		residence_time = 3600;
	uAh = power_value * residence_time * 1000 / SEC_PER_HOUR;
	p_curr->ce.gps = uAh;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %u\n", __func__, uAh);

	if (skip_penalty_check & SKIP_PANALTY_CHECK_BOOT_TIME)
		return score;

	diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;
	gps_time = p_curr->sh_gps.gps_time;

	permil = calculate_permil(gps_time, diff_boot);
	if (permil > energy_mon.penalty.threshold_gps)
		score = ENERGY_MON_SCORE_GPS;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %d %d\n", __func__,
		p_curr->log_count, score, permil, energy_mon.penalty.threshold_gps);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_gps(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}

static inline unsigned int energy_monitor_calc_penalty_hr(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_LBS_STAT
static unsigned int energy_monitor_calc_penalty_lbs(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int uAh = 0;
	ktime_t gps_time;
	s64 total_gps_time_ms = 0;
	int i;

	power_value = energy_mon.power_value.gps;

	for (i = 0; i < LBS_STAT_ARRAY_SIZE; i++) {
		gps_time = ktime_add(p_curr->lbs_stat[i].usage[LBS_METHOD_GPS].total_time,
			p_curr->lbs_stat[i].usage[LBS_METHOD_BATCH_GPS].total_time);
		total_gps_time_ms += ktime_to_ms(gps_time);
		if (total_gps_time_ms > 3600000)
			total_gps_time_ms = 3600000;
	}
	uAh = power_value * (unsigned int)total_gps_time_ms / SEC_PER_HOUR;
	p_curr->ce.lbs = uAh;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %u\n", __func__, p_curr->log_count, score, uAh);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_lbs(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_NET_STAT_TIZEN
static unsigned int energy_monitor_calc_penalty_wifi(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time = 0;
	unsigned int uAh = 0;
	int i;

	power_value = energy_mon.power_value.wifi_scan;

	/* currently we only support consumped energy for wifi scan */
	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
		if (p_curr->net_stat[i].scan_req)
			residence_time = ktime_to_ms(p_curr->net_stat[i].scan_time);
	uAh = power_value * residence_time / SEC_PER_HOUR;
	p_curr->ce.wifi = uAh;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %u\n", __func__, p_curr->log_count, score, uAh);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_wifi(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_NET_STAT_TIZEN
static unsigned int energy_monitor_calc_penalty_bt(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time = 0;
	unsigned int uAh = 0;
	int i;
	ktime_t zero = ktime_set(0,0);

	power_value = energy_mon.power_value.bt_active;

	/* currently we only support consumped energy for bt active */
	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
		if (ktime_compare(p_curr->net_stat[i].bt_active_time, zero) > 0)
			residence_time = ktime_to_ms(p_curr->net_stat[i].bt_active_time);

	uAh = power_value * residence_time / SEC_PER_HOUR;
	p_curr->ce.bt = uAh;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %u: %u\n", __func__,
		p_curr->log_count, score, uAh, residence_time);

	return score;
}
#else
static inline unsigned int energy_monitor_calc_penalty_bt(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_SLEEP_STAT
static unsigned int energy_monitor_get_cp_active_power_value(
	unsigned int signal)
{
	int i;

 	if (energy_mon.power_value.cp_active_table_size > 0) {
		for (i = 0; i < energy_mon.power_value.cp_active_table_size; i++) {
			if (energy_mon.power_value.cp_active[i].signal == signal)
				return energy_mon.power_value.cp_active[i].value;
		}
	}

	return 0;
}

static unsigned int energy_monitor_calc_penalty_cp(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int residence_time = 0;
	unsigned int uAh = 0;
	unsigned int threshold;

	if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS)
		residence_time = p_curr->sleep_stat.soc.exynos.apm[0].total_time;

	/* TODO: ce per signal strenth will be implemented in the future */
	power_value = energy_monitor_get_cp_active_power_value(0);
	uAh = power_value * residence_time / SEC_PER_HOUR;
	p_curr->ce.cp = uAh;

	threshold =
		(p_curr->ce.sleep + p_curr->ce.cpu_idle + p_curr->ce.cpu_active) * 3;
	if (uAh > threshold)
		score = ENERGY_MON_SCORE_CP;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x: %u %u %u %u\n", __func__,
		p_curr->log_count, score,
		p_curr->ce.cp, uAh, residence_time, threshold);

	return score;
}
#else
static inline unsigned int energy_monitor_get_cp_active_power_value(
	unsigned int signal)
{ return 0;}

static inline unsigned int energy_monitor_calc_penalty_cp(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

#ifdef CONFIG_SID_SYS_STATS
static unsigned int energy_monitor_is_sid_whitelist(u32 usid)
{
	//int i;
	//unsigned int whitelist_table_size;

	//whitelist_table_size = energy_mon.power_value.cpu_whitelist_table_size;

	//for (i = 0; i < whitelist_table_size; i++)
	if (usid < 5000) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: usid(%u) is in whitelist\n", __func__, usid);
		return 1;
	}

	return 0;
}

static unsigned int energy_monitor_is_sid_blacklist(u32 usid)
{
	/* hard code comm for now, need to get from device tree */
	if (usid == 651) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: usid(%u) is in blacklist\n", __func__, usid);
		return 1;
	}

	return 0;
}

static unsigned int energy_monitor_calc_penalty_sid(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int power_value;
	unsigned int permil;
	unsigned int uAh = 0;
	int i;

	energy_mon.fbdc.cpu_sid = 0;

	power_value = p_curr->ce.cpu_active;
	for (i = 0; i < SID_CPUTIME_ARRAY_SIZE; i++) {
		if(energy_monitor_is_sid_whitelist(p_curr->sid_cpu[i].usid))
			continue;

		permil = p_curr->sid_cpu[i].permil;
		if (permil >= 1000)
			continue;
		uAh = power_value * permil / 1000;
		if (uAh > energy_mon.penalty.threshold_sid) {
			energy_mon.fbdc.cpu_sid = p_curr->sid_cpu[i].sid;
			score = ENERGY_MON_SCORE_CPU_TIME;

			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: %d: score=0x%x: usid=%u: "
				"power_value=%u: permi=%u: uAh=%u\n",
				__func__, p_curr->log_count, score, p_curr->sid_cpu[i].usid,
				power_value, permil, uAh);

			return score;
		}
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x %u %u\n", __func__,
		p_curr->log_count, score, uAh, energy_mon.fbdc.cpu_sid);

	return score;
}

static unsigned int energy_monitor_calc_penalty_pid(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	unsigned int permil;
	int i;

	if (skip_penalty_check &
			(SKIP_PANALTY_CHECK_BOOT_TIME |
			SKIP_PANALTY_CHECK_KERNEL_TIME |
			SKIP_PANALTY_CHECK_FILE_TRANSFER))
		return score;

	for (i = 0; i < SID_CPUTIME_ARRAY_SIZE; i++) {
		if(energy_monitor_is_sid_blacklist(p_curr->sid_cpu[i].usid) == 0)
			continue;

		permil = p_curr->sid_cpu[i].permil;

		if (permil > energy_mon.penalty.threshold_pid) {
			energy_mon.fbdc.cpu_sid = p_curr->sid_cpu[i].sid;
			score = ENERGY_MON_SCORE_CPU_TIME;

			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: %d: score=0x%x: usid=%u: permi=%u: th=%d\n",
				__func__, p_curr->log_count, score, p_curr->sid_cpu[i].usid,
				permil, energy_mon.penalty.threshold_pid);

			return score;
		}
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d: score=0x%x\n", __func__,
		p_curr->log_count, score);

	return score;
}

#else
static inline unsigned int energy_monitor_calc_penalty_sid(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}

static inline unsigned int energy_monitor_calc_penalty_pid(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{ return 0;}
#endif

static unsigned int energy_monitor_is_fr_whitelist(char *comm)
{
	/* hard code comm for now, need to get from device tree */
	if (strncmp(comm, "mainlock", sizeof("mainlock")) == 0)
		return 1;

	return 0;
}

static unsigned int energy_monitor_calc_penalty_fr(
	struct energy_mon_data *p_curr,
	struct energy_mon_data *p_prev,
	int skip_penalty_check)
{
	unsigned int score = 0;
	long diff_boot;
	long ws_time;
	long long permil;
	int suspend_success;
	int i;
	int diff_wakeup[ENERGY_MON_WAKEUP_MAX];

	if (energy_mon.penalty.threshold_fr_suspend_success < 0)
		return score;

	if (skip_penalty_check & SKIP_PANALTY_CHECK_BOOT_TIME)
		return score;

	suspend_success = p_curr->sleep_stat.suspend_success -
						p_prev->sleep_stat.suspend_success;

	if (suspend_success > energy_mon.penalty.threshold_fr_suspend_success) {
		for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
			diff_wakeup[i] = p_curr->wakeup_cause[i] - p_prev->wakeup_cause[i];

		if (diff_wakeup[ENERGY_MON_WAKEUP_INPUT] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_INPUT])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_SSP] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_SSP])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_RTC] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_RTC])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_BT] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_BT])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_WIFI] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_WIFI])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_CP] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_CP])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_GNSS] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_GNSS])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_NFC] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_NFC])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;
		else if (diff_wakeup[ENERGY_MON_WAKEUP_ETC] >
				energy_mon.penalty.threshold_fr_wakeup[ENERGY_MON_WAKEUP_ETC])
			score = ENERGY_MON_SCORE_FORCE_REBOOT;

		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: %d: score=0x%x %d %d %d %d %d %d %d %d %d\n", __func__,
			p_curr->log_count, score,
			diff_wakeup[0], diff_wakeup[1], diff_wakeup[2],
			diff_wakeup[3], diff_wakeup[4],	diff_wakeup[5],
			diff_wakeup[6],	diff_wakeup[7], diff_wakeup[8]);

	} else if (suspend_success <= 0) {
		if (p_curr->sleep_stat.failed_freeze >
				energy_mon.penalty.threshold_fr_suspend_failed_freeze ||
			p_curr->sleep_stat.fail >
				energy_mon.penalty.threshold_fr_suspend_fail)
			score = ENERGY_MON_SCORE_FORCE_REBOOT;

		if (energy_monitor_is_fr_whitelist(p_curr->ws[0].name)) {
			diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;
			ws_time = ktime_to_timeval(p_curr->ws[0].emon_total_time).tv_sec;

			permil = ws_time * 1000;
			do_div(permil, diff_boot);

			if (permil > energy_mon.penalty.threshold_ws)
				score = 0;
		}

		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: %d: score=0x%x %d %d %d %d %d %d\n", __func__,
			p_curr->log_count, score,
			p_curr->sleep_stat.fail,
			p_curr->sleep_stat.failed_freeze,
			p_curr->sleep_stat.failed_prepare,
			p_curr->sleep_stat.failed_suspend,
			p_curr->sleep_stat.failed_suspend_late,
			p_curr->sleep_stat.failed_suspend_noirq);
	}

	return score;
}

static unsigned int energy_monitor_calc_penalty_score(
	struct energy_mon_data *p_curr, struct energy_mon_data *p_prev)
{
	struct timespec ts_last_th_change;
	long diff_boot = 0, diff_kernel = 0, diff_last_th_change;
	unsigned int score = 0;
	int skip_penalty_check = 0;

	if (!p_prev || !p_curr) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: pointer is NULL\n", __func__);
		return 0;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: prev[%d]=%d: curr[%d]=%d\n", __func__,
		p_prev->log_count, p_prev->bat_status,
		p_prev->log_count, p_curr->bat_status);

	if (p_prev->bat_status == POWER_SUPPLY_STATUS_CHARGING ||
		p_prev->bat_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
		p_prev->bat_status == POWER_SUPPLY_STATUS_FULL) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: if charging status, just skip\n", __func__);
		return 0;
	}

	diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;
	if (diff_boot <= energy_mon.penalty.threshold_time) {
		skip_penalty_check |= SKIP_PANALTY_CHECK_BOOT_TIME;
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: elapsed time smaller than threshold, skip_penalty_check=%d\n",
			__func__, skip_penalty_check);
	}

	diff_kernel = p_curr->ts_kern.tv_sec - p_prev->ts_kern.tv_sec;
	if (diff_kernel <= energy_mon.penalty.threshold_short_cpu_usage) {
		skip_penalty_check |= SKIP_PANALTY_CHECK_KERNEL_TIME;
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: elapsed time smaller than short_cpu_usage, "
			"skip_penalty_check=%d\n",
			__func__, skip_penalty_check);
	}

	ts_last_th_change = ktime_to_timespec(energy_mon.penalty.last_threshold_change);
	diff_last_th_change = p_curr->ts_boot.tv_sec - ts_last_th_change.tv_sec;

	if (diff_last_th_change <= energy_mon.penalty.threshold_time) {
		skip_penalty_check |= SKIP_PANALTY_CHECK_THRESHOLD_CHARNGE;
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: elapsed time from last threshold change smaller than "
			"threshold, skip_penalty_check=%d\n",
			__func__, skip_penalty_check);
	}

	// TODO: need to refactoring
	if (energy_monitor_get_log_offset(p_curr) > energy_mon.penalty.threshold_log_offset) {
		skip_penalty_check |= SKIP_PANALTY_CHECK_FILE_TRANSFER;
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: hostmanager use sap for file transfer, skip_penalty_check=%d\n",
			__func__, skip_penalty_check);
	}

	score |= energy_monitor_calc_penalty_disp(p_curr, p_prev, skip_penalty_check);
	/* energy_monitor_calc_penalty_battery must be behind energy_monitor_calc_penalty_disp */
	score |= energy_monitor_calc_penalty_battery(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_input(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_sh(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_alarm(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_sap(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_tcp(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_ws_slwl(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_exynos_cpuidle(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_cpu(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_gpu(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_sleep(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_ff(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_gps(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_lbs(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_wifi(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_bt(p_curr, p_prev, skip_penalty_check);
	/* energy_monitor_calc_penalty_cp must be behind energy_monitor_calc_penalty_cpu */
	score |= energy_monitor_calc_penalty_cp(p_curr, p_prev, skip_penalty_check);
	/* calculate_penalty_sid must be last in energy_monitor_calculate_penalty_xx functions */
	score |= energy_monitor_calc_penalty_sid(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_pid(p_curr, p_prev, skip_penalty_check);
	score |= energy_monitor_calc_penalty_fr(p_curr, p_prev, skip_penalty_check);

	score &= energy_mon.penalty.notify_mask;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %d: score=0x%x mask=0x%x\n",
		__func__, p_curr->log_count,score,energy_mon.penalty.notify_mask);

	return score;
}

static void energy_mon_get_additional_info(int type,
		struct energy_mon_data *p_curr,	struct energy_mon_data *p_prev)
{
	if (!p_curr) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,"%s: pointer is NULL\n", __func__);
		return;
	}

	disp_stat_get_stat(type, &p_curr->disp_stat);
#ifdef CONFIG_INPUT_STAT
	input_stat_get_stat_emon(type, &p_curr->input_stat);
#endif
#ifdef CONFIG_SENSORHUB_STAT
	sensorhub_stat_get_stat(type, &p_curr->sh_stat);
	sensorhub_stat_get_wo_info(&p_curr->sh_wo);
#endif
#ifdef CONFIG_ALARM_HISTORY
	alarm_history_get_stat(type, p_curr->alarm_stat, ALARM_STAT_ARRAY_SIZE);
#endif
#ifdef CONFIG_SAP_PID_STAT
	sap_stat_get_wakeup(&p_curr->sap_wakeup);
	sap_stat_get_traffic_emon(type, p_curr->sap_traffic, SAP_TRAFFIC_ARRAY_SIZE);
#endif
#ifdef CONFIG_USID_STAT
	usid_stat_get_traffic_emon(type, p_curr->tcp_traffic, TCP_TRAFFIC_ARRAY_SIZE);
#endif
#ifdef CONFIG_NET_STAT_TIZEN
	net_stat_tizen_get_stat_emon(type, p_curr->net_stat, NET_STAT_ARRAY_SIZE);
#endif
#ifdef CONFIG_SLAVE_WAKELOCK
	slave_wakelock_get_large_prevent_time(type, p_curr->slwl, SLWL_ARRAY_SIZE);
#endif
#ifdef CONFIG_PM_SLEEP
	pm_get_large_wakeup_sources(type, p_curr->ws, WS_ARRAY_SIZE);
	pm_get_large_wakeup_count(type, p_curr->ws_wu, WS_ARRAY_SIZE);
#endif
#ifdef CONFIG_SID_SYS_STATS
#ifdef CONFIG_PID_SYS_STATS
	get_sid_cputime_emon(type, p_curr->sid_cpu, p_curr->pid_cpu, SID_CPUTIME_ARRAY_SIZE);
#else
	get_sid_cputime_emon(type, p_curr->sid_cpu, SID_CPUTIME_ARRAY_SIZE);
#endif
	get_sid_io(type, p_curr->sid_io, SID_IO_ARRAY_SIZE);
#endif
#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
	cpuidle_stats_get_stats_emon(type, &p_curr->cpuidle_stat);
	cpuidle_stats_get_blocker(type, p_curr->lpm_blocker, BLOCKER_ARRAY_SIZE);
#endif
#ifdef CONFIG_CPU_FREQ_STAT_TIZEN
	cpufreq_stats_tizen_get_stats(type, &p_curr->cpufreq_stats);
#endif
#ifdef CONFIG_GPUFREQ_STAT
	gpufreq_stat_get_stat(type, &p_curr->gpufreq_stat);
#endif
#ifdef CONFIG_SENSORS_SEC_THERM_HISTORY
	if (type != ENERGY_MON_TYPE_DUMP) {
		int i;

		get_sec_therm_history_energy_mon(type, p_curr->therm_history);
		for (i = 0; i < MAX_SEC_THERM_DEVICE_NUM; i++)
			energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "thm: %d: %d~%d, %d[%d/%d] %d\n",
					i, p_curr->therm_history[i].min, p_curr->therm_history[i].max,
					p_curr->therm_history[i].sum / p_curr->therm_history[i].cnt,
					p_curr->therm_history[i].sum, p_curr->therm_history[i].cnt,
					p_curr->therm_history[i].reset);
	}
#endif
#ifdef CONFIG_FF_STAT_TIZEN
	ff_stat_tizen_get_stat_emon(type, &p_curr->ff_stat);
#endif
#ifdef CONFIG_SLEEP_STAT
	sleep_stat_get_stat_emon(type, &p_curr->sleep_stat);
#endif
#ifdef CONFIG_SENSORHUB_STAT
	sensorhub_stat_get_gps_emon(type, &p_curr->sh_gps);
#endif
#ifdef CONFIG_LBS_STAT
	lbs_stat_get_stat_emon(type, p_curr->lbs_stat, LBS_STAT_ARRAY_SIZE);
#endif
}

static int energy_monintor_inject_additional_info(
		int type, struct energy_mon_data *p_accu,
		struct energy_mon_data *p_curr,	struct energy_mon_data *p_prev)
{
	int i, j;
	ktime_t diff_wear_off;

	/* input stat */
	p_accu->input_stat.key_press += p_curr->input_stat.key_press;
	p_accu->input_stat.key_release += p_curr->input_stat.key_release;
	p_accu->input_stat.touch_press += p_curr->input_stat.touch_press;
	p_accu->input_stat.touch_release += p_curr->input_stat.touch_release;
	for (i = 0; i < Direction_MAX; i++)
		p_accu->input_stat.direction[i] += p_curr->input_stat.direction[i];
	for (i = 0; i < EV_WU_MAX; i++)
		p_accu->input_stat.ev_wakeup[i] += p_curr->input_stat.ev_wakeup[i];

	/* accumulate wear off time */
	diff_wear_off = ktime_sub(
		p_curr->sh_wo.wear_off_time, p_prev->sh_wo.wear_off_time);
	p_accu->sh_wo.wear_off_time = ktime_add(
		p_accu->sh_wo.wear_off_time, diff_wear_off);

	/* net stat */
	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++) {
		p_accu->net_stat[i].up_time =
			ktime_add(p_accu->net_stat[i].up_time, p_curr->net_stat[i].up_time);

		p_accu->net_stat[i].tx_bytes += p_curr->net_stat[i].tx_bytes;
		p_accu->net_stat[i].tx_packets += p_curr->net_stat[i].tx_packets;
		p_accu->net_stat[i].rx_bytes += p_curr->net_stat[i].rx_bytes;
		p_accu->net_stat[i].rx_packets += p_curr->net_stat[i].rx_packets;

		p_accu->net_stat[i].bt_active_time = ktime_add(
			p_accu->net_stat[i].bt_active_time,
			p_curr->net_stat[i].bt_active_time);
		p_accu->net_stat[i].bt_tx_time = ktime_add(
			p_accu->net_stat[i].bt_tx_time, p_curr->net_stat[i].bt_tx_time);
		p_accu->net_stat[i].bt_rx_time = ktime_add(
			p_accu->net_stat[i].bt_rx_time, p_curr->net_stat[i].bt_rx_time);

		if (p_curr->net_stat[i].scan_req) {
			p_accu->net_stat[i].scan_req += p_curr->net_stat[i].scan_req;
			p_accu->net_stat[i].scan_time = ktime_add(
				p_accu->net_stat[i].scan_time , p_curr->net_stat[i].scan_time);
		}
	}

	/* cpu idle */
	p_accu->cpuidle_stat.cpu_count = p_curr->cpuidle_stat.cpu_count;
	p_accu->cpuidle_stat.state_count = p_curr->cpuidle_stat.state_count;
	p_accu->cpuidle_stat.total_stat_time = ktime_add(
		p_accu->cpuidle_stat.total_stat_time,
		p_curr->cpuidle_stat.total_stat_time);
	for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++) {
		p_accu->cpuidle_stat.cpuidle[i].hp.online_time = ktime_add(
			p_accu->cpuidle_stat.cpuidle[i].hp.online_time,
			p_curr->cpuidle_stat.cpuidle[i].hp.online_time);
		p_accu->cpuidle_stat.cpuidle[i].total_idle_time = ktime_add(
			p_accu->cpuidle_stat.cpuidle[i].total_idle_time,
			p_curr->cpuidle_stat.cpuidle[i].total_idle_time);
	}
	for (j = 0; j < p_curr->cpuidle_stat.state_count; j++)
		for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++)
			p_accu->cpuidle_stat.cpuidle[i].usage[j].time = ktime_add(
				p_accu->cpuidle_stat.cpuidle[i].usage[j].time,
				p_curr->cpuidle_stat.cpuidle[i].usage[j].time);
	p_accu->cpuidle_stat.lpm.usage.time = ktime_add(
		p_accu->cpuidle_stat.lpm.usage.time,
		p_curr->cpuidle_stat.lpm.usage.time);

	/* cpu freq */
	p_accu->cpufreq_stats.state_num = p_curr->cpufreq_stats.state_num;
	for (i = 0; i < p_curr->cpufreq_stats.state_num; i++)
		p_accu->cpufreq_stats.time_in_state[i] +=
			p_curr->cpufreq_stats.time_in_state[i];

	/* gpu freq */
	p_accu->gpufreq_stat.table_size = p_curr->gpufreq_stat.table_size;
	for (i = 0; i < p_curr->gpufreq_stat.table_size; i++)
		p_accu->gpufreq_stat.table[i].time += p_curr->gpufreq_stat.table[i].time;

	/* ff stat */
	p_accu->ff_stat.total_time = ktime_add(
		p_accu->ff_stat.total_time, p_curr->ff_stat.total_time);
	p_accu->ff_stat.play_count += p_curr->ff_stat.play_count;

	/* sleep stat */
	p_accu->sleep_stat.soc_type = p_curr->sleep_stat.soc_type;
	p_accu->sleep_stat.fail += p_curr->sleep_stat.fail;
	p_accu->sleep_stat.failed_freeze += p_curr->sleep_stat.failed_freeze;
	p_accu->sleep_stat.failed_prepare += p_curr->sleep_stat.failed_prepare;
	p_accu->sleep_stat.failed_suspend += p_curr->sleep_stat.failed_suspend;
	p_accu->sleep_stat.failed_suspend_late +=
		p_curr->sleep_stat.failed_suspend_late;
	p_accu->sleep_stat.failed_suspend_noirq +=
		p_curr->sleep_stat.failed_suspend_noirq;
	p_accu->sleep_stat.suspend_success += p_curr->sleep_stat.suspend_success;

	if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS) {
		p_accu->sleep_stat.suspend_success = p_curr->sleep_stat.suspend_success;
		p_accu->sleep_stat.soc.exynos.acpm_sleep_early_wakeup +=
			p_curr->sleep_stat.soc.exynos.acpm_sleep_early_wakeup;
		p_accu->sleep_stat.soc.exynos.acpm_sleep_soc_down +=
			p_curr->sleep_stat.soc.exynos.acpm_sleep_soc_down;
		p_accu->sleep_stat.soc.exynos.acpm_sleep_mif_down +=
			p_curr->sleep_stat.soc.exynos.acpm_sleep_mif_down;
		p_accu->sleep_stat.soc.exynos.acpm_sicd_early_wakeup +=
			p_curr->sleep_stat.soc.exynos.acpm_sicd_early_wakeup;
		p_accu->sleep_stat.soc.exynos.acpm_sicd_soc_down +=
			p_curr->sleep_stat.soc.exynos.acpm_sicd_soc_down;
		p_accu->sleep_stat.soc.exynos.acpm_sicd_mif_down +=
			p_curr->sleep_stat.soc.exynos.acpm_sicd_mif_down;

		for (i = 0; i < EXYNOS_MIF_MASTER_MAX - 2; i++)
			p_accu->sleep_stat.soc.exynos.apm_wakeup[i].tcxo_req_count +=
				p_curr->sleep_stat.soc.exynos.apm_wakeup[i].tcxo_req_count;

		for (i = 0; i < EXYNOS_MIF_MASTER_MAX; i++)
			p_accu->sleep_stat.soc.exynos.apm[i].total_time +=
				p_curr->sleep_stat.soc.exynos.apm[i].total_time;
	}

	/* gps */
	p_accu->sh_gps.gps_time += p_curr->sh_gps.gps_time;

	/* ce */
	p_accu->ce.sleep += p_curr->ce.sleep;
	p_accu->ce.disp_aod += p_curr->ce.disp_aod;
	p_accu->ce.disp_on += p_curr->ce.disp_on;
	p_accu->ce.cpu_idle += p_curr->ce.cpu_idle;
	p_accu->ce.cpu_active += p_curr->ce.cpu_active;
	p_accu->ce.gpu += p_curr->ce.gpu;
	p_accu->ce.ff += p_curr->ce.ff;
	p_accu->ce.gps += p_curr->ce.gps;
	p_accu->ce.lbs += p_curr->ce.lbs;
	p_accu->ce.hr += p_curr->ce.hr;
	p_accu->ce.bt += p_curr->ce.bt;
	p_accu->ce.wifi += p_curr->ce.wifi;
	p_accu->ce.cp += p_curr->ce.cp;

	return 0;
}

static int energy_monitor_inject_data(int type,
	struct energy_mon_data *p_curr, struct energy_mon_data *p_prev)
{
	struct energy_mon_data *p_buff;
	struct timespec time_diff;
	int charging_discharging;
	int i;

	/*
	 * If type is battery and monitor, inject data to charging buffer and
	 * if type is dump, inject data to dump buffer and charging_dump buffer
	 */
	if (type == ENERGY_MON_TYPE_BATTERY || type == ENERGY_MON_TYPE_MONITOR) {
		if (!p_prev || !p_curr) {
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s: pointer is NULL\n", __func__);
			return -EINVAL;
		}

		if (p_prev->bat_status == POWER_SUPPLY_STATUS_CHARGING) {
			p_buff = &energy_mon.charging;
			energy_mon.charging_count++;
			charging_discharging = ENERGY_MON_STATE_CHARGING;
		} else if (p_prev->bat_status == POWER_SUPPLY_STATUS_DISCHARGING) {
			p_buff = &energy_mon.discharging;
			energy_mon.discharging_count++;
			charging_discharging = ENERGY_MON_STATE_DISCHARGING;
		} else if (p_prev->bat_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			p_buff = &energy_mon.not_charging;
			energy_mon.not_charging_count++;
			charging_discharging = ENERGY_MON_STATE_NOT_CHARGING;
		} else if (p_prev->bat_status == POWER_SUPPLY_STATUS_FULL) {
			p_buff = &energy_mon.full;
			energy_mon.full_count++;
			charging_discharging = ENERGY_MON_STATE_FULL;
		} else {
			/* If not logging case - invalid battery status, just return 0 */
			energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
					"%s: not logging case, type=%d, prev bat_status=%d\n",
					__func__, type, p_prev->bat_status);
			return 0;
		}
	} else if (type == ENERGY_MON_TYPE_DUMP) {
		if (energy_mon.dump.bat_status == POWER_SUPPLY_STATUS_CHARGING) {
			p_buff = &energy_mon.charging_dump;
			charging_discharging = ENERGY_MON_STATE_CHARGING;
		} else if (energy_mon.dump.bat_status == POWER_SUPPLY_STATUS_DISCHARGING) {
			p_buff = &energy_mon.discharging_dump;
			charging_discharging = ENERGY_MON_STATE_DISCHARGING;
		} else if (energy_mon.dump.bat_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			p_buff = &energy_mon.not_charging_dump;
			charging_discharging = ENERGY_MON_STATE_NOT_CHARGING;
		} else if (energy_mon.dump.bat_status == POWER_SUPPLY_STATUS_FULL) {
			p_buff = &energy_mon.full_dump;
			charging_discharging = ENERGY_MON_STATE_FULL;
		} else {
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s: unknown case\n", __func__);
			return -EPERM;
		}

		memcpy(&energy_mon.discharging_dump, &energy_mon.discharging,
				sizeof(struct energy_mon_data));
		memcpy(&energy_mon.charging_dump, &energy_mon.charging,
				sizeof(struct energy_mon_data));
		memcpy(&energy_mon.not_charging_dump, &energy_mon.not_charging,
				sizeof(struct energy_mon_data));
		memcpy(&energy_mon.full_dump, &energy_mon.full,
				sizeof(struct energy_mon_data));

		if (energy_mon.data_index == 0)
			p_prev = &energy_mon.boot;
		else {
			int prev_idx = (energy_mon.data_index + ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM;

			p_prev = &energy_mon.data[prev_idx];
		}
		p_curr = &energy_mon.dump;
	} else {
		/* If not logging case, just return 0 */
		energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
				"%s: not logging case, type=%d", __func__, type);
		return 0;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: use %d buffer\n", __func__, charging_discharging);

	/* Data checking routine */
	switch (charging_discharging) {
	case ENERGY_MON_STATE_DISCHARGING:
		if (p_curr->bat_capacity > p_prev->bat_capacity)
			energy_mon_dbg(ENERGY_MON_DEBUG_WARN,
				"%s: capacity is changed from %d to %d even discharged.\n",
				__func__, p_prev->bat_capacity, p_curr->bat_capacity);
		break;
	case ENERGY_MON_STATE_CHARGING:
		if (p_curr->bat_capacity < p_prev->bat_capacity)
			energy_mon_dbg(ENERGY_MON_DEBUG_WARN,
				"%s: capacity is changed from %d to %d even charged.\n",
				__func__, p_prev->bat_capacity, p_curr->bat_capacity);
		break;
	case ENERGY_MON_STATE_NOT_CHARGING:
		if (p_curr->bat_capacity < p_prev->bat_capacity)
			energy_mon_dbg(ENERGY_MON_DEBUG_WARN,
				"%s: capacity is changed from %d to %d in a state of not charging.\n",
				__func__, p_prev->bat_capacity, p_curr->bat_capacity);
		break;
	case ENERGY_MON_STATE_FULL:
		if (p_curr->bat_capacity < p_prev->bat_capacity)
			energy_mon_dbg(ENERGY_MON_DEBUG_WARN,
				"%s: capacity is changed from %d to %d in a state of full.\n",
				__func__, p_prev->bat_capacity, p_curr->bat_capacity);
		break;
	default:
		break;
	}

	if (p_curr->bat_capacity > p_prev->bat_capacity)
		energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
			"%s: capacity is changed from %d to %d\n", __func__,
			p_prev->bat_capacity, p_curr->bat_capacity);

	if (type == ENERGY_MON_TYPE_BATTERY ||
		type == ENERGY_MON_TYPE_MONITOR ||
		type == ENERGY_MON_TYPE_DUMP) {
		p_buff->log_count++;
		p_buff->bat_capacity += p_curr->bat_capacity - p_prev->bat_capacity;
		p_buff->suspend_count += p_curr->suspend_count - p_prev->suspend_count;
	}

	/* set the change time of the battery state*/
	if (type == ENERGY_MON_TYPE_BATTERY || type == ENERGY_MON_TYPE_DUMP)
		p_buff->ts_real = p_prev->ts_real;

	/* If diff_time is negative, change to zero */
	time_diff = timespec_sub(p_curr->ts_kern, p_prev->ts_kern);
	if (time_diff.tv_sec < 0) {
		time_diff.tv_sec = 0;
		time_diff.tv_nsec = 0;
	}
	p_buff->ts_kern = timespec_add(p_buff->ts_kern, time_diff);

	time_diff = timespec_sub(p_curr->ts_boot, p_prev->ts_boot);
	if (time_diff.tv_sec < 0) {
		time_diff.tv_sec = 0;
		time_diff.tv_nsec = 0;
	}
	p_buff->ts_boot = timespec_add(p_buff->ts_boot, time_diff);
#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
	p_buff->current_suspend += p_curr->current_suspend * (int)time_diff.tv_sec;
#endif

	time_diff = timespec_sub(p_curr->ts_disp, p_prev->ts_disp);
	if (time_diff.tv_sec < 0) {
		time_diff.tv_sec = 0;
		time_diff.tv_nsec = 0;
	}
	p_buff->ts_disp = timespec_add(p_buff->ts_disp, time_diff);

	time_diff = timespec_sub(p_curr->ts_aod, p_prev->ts_aod);
	if (time_diff.tv_sec < 0) {
		time_diff.tv_sec = 0;
		time_diff.tv_nsec = 0;
	}
	p_buff->ts_aod = timespec_add(p_buff->ts_aod, time_diff);

	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++) {
		p_buff->wakeup_cause[i] +=
			p_curr->wakeup_cause[i] - p_prev->wakeup_cause[i];
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
		p_buff->wakeup_time[i] = ktime_add(
			p_buff->wakeup_time[i], p_curr->wakeup_time[i]);
#endif
	}

	/* Inject addtional information */
	energy_monintor_inject_additional_info(type, p_buff, p_curr, p_prev);

	/* Debug logs */
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s\n", __func__);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"ts_boot: %15lu.%09lu\n",
		p_buff->ts_boot.tv_sec,
		p_buff->ts_boot.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"ts_kern: %15lu.%09lu\n",
		p_buff->ts_kern.tv_sec,
		p_buff->ts_kern.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"ts_disp: %15lu.%09lu\n",
		p_buff->ts_disp.tv_sec,
		p_buff->ts_disp.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"ts_aod: %15lu.%09lu\n",
		p_buff->ts_aod.tv_sec,
		p_buff->ts_aod.tv_nsec);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"bat_stat=%d, bat_capa=%03d\n",
		p_buff->bat_status,
		p_buff->bat_capacity);
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"suspend_count=%04d\n",
		p_buff->suspend_count);

	return 0;
}

int energy_monitor_marker(int type)
{
	/* Do common works */
	struct energy_mon_data *p_curr = NULL;
	struct energy_mon_data *p_prev = NULL;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: type=%d\n", __func__, type);

	if (type == ENERGY_MON_TYPE_BOOTING) {
		/* Call LCD on command at boot time */
		//disp_on_logging();
		energy_mon.running = 1;
	} else if (!energy_mon.running) {
		/* If marker is called before running(e.g. booting call), just ignore it */
		energy_mon_dbg(ENERGY_MON_DEBUG_WARN,
			"%s: called before running\n", __func__);
		return 0;
	}

	/* Assign proper buffer to save */
	if (type == ENERGY_MON_TYPE_BOOTING)
		p_curr = &energy_mon.boot;
	else if (type == ENERGY_MON_TYPE_DUMP)
		p_curr = &energy_mon.dump;
	else {
		p_curr = &energy_mon.data[energy_mon.data_index % ENERGY_MON_HISTORY_NUM];

		if (energy_mon.data_index == 0) {
			/* If it is 1st marker, use boot data as previous one */
			p_prev = &energy_mon.boot;
			energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
				"%s: use boot buffer as previous one\n", __func__);
		} else {
			p_prev = &energy_mon.data[(energy_mon.data_index - 1) % ENERGY_MON_HISTORY_NUM];
			energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
				"%s: use %d buffer as previous one\n",
				__func__, energy_mon.data_index);
		}
		p_curr->log_count = energy_mon.data_index;
	}

	/* Get time informations */
	energy_mon_get_time_info(p_curr);

	/* Get battery informations */
	energy_mon_get_battery_info(p_curr);

	/* Get wakeup reason informations */
	memcpy(p_curr->wakeup_cause,
			energy_mon.wakeup_cause, sizeof(energy_mon.wakeup_cause));

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	/* Get wakeup stat informations */
	if (type == ENERGY_MON_TYPE_BATTERY || type == ENERGY_MON_TYPE_MONITOR)
		energy_mon_get_wakeup_stat(p_curr, 1);
	else
		energy_mon_get_wakeup_stat(p_curr, 0);
#endif

	/* Get additional informations */
	if (type == ENERGY_MON_TYPE_BATTERY ||
		type == ENERGY_MON_TYPE_MONITOR ||
		type == ENERGY_MON_TYPE_DUMP)
		energy_mon_get_additional_info(type, p_curr, p_prev);

	/* Inject to charging/discharging buffer */
	if (type == ENERGY_MON_TYPE_BATTERY ||
		type == ENERGY_MON_TYPE_MONITOR ||
		type == ENERGY_MON_TYPE_DUMP)
		energy_monitor_inject_data(type, p_curr, p_prev);
	else if (type == ENERGY_MON_TYPE_BOOTING) {
		/* Compensate disp time at boot. Add ts_kern to ts_disp */
		energy_mon.boot.ts_disp = p_curr->ts_kern;
	}

	/* Add data_index after fill all datas if data ring buffer is used */
	if (type >= ENERGY_MON_TYPE_BATTERY && type < ENERGY_MON_TYPE_DUMP)
		energy_mon.data_index++;

	/* Calculate penalty score and notify
	 *  energy_monitor_penalty_score_notify must be behind energy_mon.data_index++
	 */
	if (type != ENERGY_MON_TYPE_DUMP) {
		p_curr->penalty_score =
			energy_monitor_calc_penalty_score(p_curr, p_prev);

		if (p_curr->penalty_score > 0) {
			energy_mon.penalty.last_score = p_curr->penalty_score;
			energy_mon.penalty.last_score_index = p_curr->log_count;
			energy_monitor_penalty_score_notify();

			energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: index=%d score=0x%x\n",
				__func__, energy_mon.penalty.last_score_index, energy_mon.penalty.last_score);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(energy_monitor_marker);

static int energy_monitor_background_marker(void)
{
	struct timespec ts_curr;
	struct timespec ts_elapsed;
	struct energy_mon_data *p_prev;

	get_monotonic_boottime(&ts_curr);

	if (energy_mon.data_index == 0)
		p_prev = &energy_mon.boot;
	else
		p_prev = &energy_mon.data[(energy_mon.data_index + ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	ts_elapsed = timespec_sub(ts_curr, p_prev->ts_boot);


	if (ts_elapsed.tv_sec >= logging_interval)
		energy_monitor_marker(ENERGY_MON_TYPE_MONITOR);

	return 0;
}

static int energy_monitor_find_wakeup_index(const char *irq_name)
{
	int i;

 	if (energy_mon.irq_map_table_size > 0) {
		for (i = 0; i < energy_mon.irq_map_table_size; i++) {
			if (!strcmp(irq_name, energy_mon.irq_map_table[i].irq_name))
				return energy_mon.irq_map_table[i].wakeup_idx;
		}
	}
	energy_mon_dbg(ENERGY_MON_DEBUG_WARN,
		"%s: %s: can not find index\n", __func__, irq_name);

	return ENERGY_MON_WAKEUP_ETC;
}

int energy_monitor_record_wakeup_reason(int irq, char *irq_name)
{
	struct irq_desc *desc = NULL;
	int wakeup_idx = -1;

	if (irq_name) {
		energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
			"%s: irq=N/A(%s)\n", __func__, irq_name);
		wakeup_idx = energy_monitor_find_wakeup_index(irq_name);
	} else if (irq > 0) {
		desc = irq_to_desc(irq);
		if (desc && desc->action && desc->action->name) {
			energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
				"%s: irq=%d(%s)\n", __func__, irq, desc->action->name);
			wakeup_idx = energy_monitor_find_wakeup_index(desc->action->name);
		}
	} else
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: irq=%d\n", __func__, irq);

	if (wakeup_idx >= 0) {
		energy_mon.wakeup_cause[wakeup_idx]++;
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
		energy_mon.last_wakeup_idx = wakeup_idx;
#endif
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: %02d/%02d/%02d/%02d/%02d/%02d/%02d/%02d/%02d\n",
		__func__,
		energy_mon.wakeup_cause[0],
		energy_mon.wakeup_cause[1],
		energy_mon.wakeup_cause[2],
		energy_mon.wakeup_cause[3],
		energy_mon.wakeup_cause[4],
		energy_mon.wakeup_cause[5],
		energy_mon.wakeup_cause[6],
		energy_mon.wakeup_cause[7],
		energy_mon.wakeup_cause[8]);

	return 0;
}
EXPORT_SYMBOL_GPL(energy_monitor_record_wakeup_reason);


/*
 * Functions for printing out
 */

static bool is_last_read_index(void)
{
	if (energy_mon.read_index - energy_mon.data_index >= ENERGY_MON_HISTORY_NUM)
		return 1;
	else
		return 0;
}

static ssize_t energy_monitor_print_time_logs(char *buf, int buf_size, int type)
{
	ssize_t ret = 0, temp_ret = 0;
	struct energy_mon_data *p_curr = NULL;
	struct energy_mon_data *p_prev = NULL;
	long long average;
	int need_to_show_average = 0;
	int diff_soc;
	__kernel_time_t diff_boot = 0, diff_kern, diff_disp, diff_aod, diff_cp = 0;
	s64 diff_wear_off = 0;
	int diff_wakeup[ENERGY_MON_WAKEUP_MAX];
	long kern_percent = 0, disp_percent = 0, aod_percent = 0, cp_percent = 0;
	struct rtc_time tm_real;
	char temp_buf[256];
	char score_print[10] = "N/A";
	char print_type;
	char bat_status;
	char average_c;
	int i;
#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
	int	current_suspend;
#endif
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	s64 total_time_ms;
	s64 wakeup_time_ms[ENERGY_MON_WAKEUP_MAX];
	ktime_t total_time = ktime_set(0,0);
#endif

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, type=%d\n", __func__, buf_size, type);

	/* Assign proper buffer to use */
	if (type == ENERGY_MON_TYPE_BOOTING) {
		print_type = 'b';
		p_curr = &energy_mon.boot;
		p_prev = NULL;
	} else if (type == ENERGY_MON_TYPE_DUMP) {
		print_type = 'd';
		need_to_show_average = 1;
		p_curr = &energy_mon.dump;
		if (energy_mon.data_index == 0)
			p_prev = &energy_mon.boot;
		else
			p_prev = &energy_mon.data[(energy_mon.data_index + ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	} else if (type == ENERGY_MON_TYPE_BATTERY || type == ENERGY_MON_TYPE_MONITOR) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index % ENERGY_MON_HISTORY_NUM];

		snprintf(score_print, 10, "%X", p_curr->penalty_score);

		/* If nothing is marked */
		if (energy_mon.data_index <= ENERGY_MON_HISTORY_NUM && energy_mon.read_index % ENERGY_MON_HISTORY_NUM == 0) {
			p_prev = &energy_mon.boot;
			need_to_show_average = 1;
		} else if ((energy_mon.read_index - energy_mon.data_index) % ENERGY_MON_HISTORY_NUM == 0)
			p_prev = NULL;
		else {
			p_prev = &energy_mon.data[(energy_mon.read_index + ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
			need_to_show_average = 1;
		}
	} else if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		need_to_show_average = 1;
		p_curr = &energy_mon.charging_dump;
		p_prev = NULL;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING) {
		print_type = 'D';
		need_to_show_average = 1;
		p_curr = &energy_mon.discharging_dump;
		p_prev = NULL;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING) {
		print_type = 'N';
		need_to_show_average = 1;
		p_curr = &energy_mon.not_charging_dump;
		p_prev = NULL;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		need_to_show_average = 1;
		p_curr = &energy_mon.full_dump;
		p_prev = NULL;
	} else {
		// TODO: Need to check return value
		// TODO: What shall I do if there is no valid case
		return 0;
	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p, p_prev=%p\n", __func__, p_curr, p_prev);

	if (p_curr->bat_status == POWER_SUPPLY_STATUS_CHARGING)
		bat_status = 'C';
	else if (p_curr->bat_status == POWER_SUPPLY_STATUS_DISCHARGING)
		bat_status = 'D';
	else if (p_curr->bat_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
		bat_status = 'N';
	else if (p_curr->bat_status == POWER_SUPPLY_STATUS_FULL)
		bat_status = 'F';
	else
		bat_status = 'U';

	if (need_to_show_average) {
		if (type == ENERGY_MON_TYPE_CHARGING ||
				type == ENERGY_MON_TYPE_DISCHARGING ||
				type == ENERGY_MON_TYPE_NOT_CHARGING ||
				type == ENERGY_MON_TYPE_FULL) {
			diff_soc = p_curr->bat_capacity;
			diff_boot = p_curr->ts_boot.tv_sec;
			diff_kern = p_curr->ts_kern.tv_sec;
			diff_disp = p_curr->ts_disp.tv_sec;
			diff_aod = p_curr->ts_aod.tv_sec;
#ifdef CONFIG_SLEEP_STAT
			if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS)
				diff_cp = p_curr->sleep_stat.soc.exynos.apm[0].total_time;
#endif
			diff_wear_off = ktime_to_ms(p_curr->sh_wo.wear_off_time)/1000;
			average = abs(energy_mon.unit_bat_capacity * p_curr->bat_capacity);
#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
			current_suspend = p_curr->current_suspend / diff_boot;
#endif
			memcpy(diff_wakeup, p_curr->wakeup_cause, sizeof(p_curr->wakeup_cause));
		} else {
			diff_soc = p_curr->bat_capacity - p_prev->bat_capacity;
			diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;
			diff_kern = p_curr->ts_kern.tv_sec - p_prev->ts_kern.tv_sec;
			diff_disp = p_curr->ts_disp.tv_sec - p_prev->ts_disp.tv_sec;
			diff_aod = p_curr->ts_aod.tv_sec - p_prev->ts_aod.tv_sec;
#ifdef CONFIG_SLEEP_STAT
			if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS)
				diff_cp = p_curr->sleep_stat.soc.exynos.apm[0].total_time;
#endif
			diff_wear_off = ktime_ms_delta(p_curr->sh_wo.wear_off_time,
								p_prev->sh_wo.wear_off_time)/1000;

			/* If diff_time is negative, change to zero */
			if (diff_boot < 0)
				diff_boot = 0;
			if (diff_kern < 0)
				diff_kern = 0;
			else if (diff_kern > diff_boot)
				diff_kern = diff_boot;
			if (diff_disp < 0)
				diff_disp = 0;
			else if (diff_disp > diff_boot)
				diff_disp = diff_boot;
			if (diff_aod < 0)
				diff_aod = 0;
			else if (diff_aod > diff_boot)
				diff_aod = diff_boot;

			average = abs(energy_mon.unit_bat_capacity *
						(p_curr->bat_capacity - p_prev->bat_capacity));

#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
			current_suspend = p_curr->current_suspend;
#endif

			for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
				diff_wakeup[i] = p_curr->wakeup_cause[i] - p_prev->wakeup_cause[i];
		}

		if (diff_soc > 0)
			average_c = '+';
		else
			average_c = ' ';

		/* To prevent Device by Zero */
		if (diff_boot) {
			kern_percent = diff_kern*10000/diff_boot;
			disp_percent = diff_disp*10000/diff_boot;
			aod_percent = diff_aod*10000/diff_boot;
			cp_percent = diff_cp*10/diff_boot;
			do_div(average, diff_boot);
		}

		temp_ret += snprintf(temp_buf + temp_ret, sizeof(temp_buf) - temp_ret,
				"/%6d/%5lu/%5lu/%5lu/%5lu/%5llu/%c%3ld.%02ldmA",
				diff_soc, diff_boot, diff_kern, diff_disp, diff_aod,
				diff_wear_off,
				average_c, (long)average/100, (long)average%100);
#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
		temp_ret += snprintf(temp_buf + temp_ret, sizeof(temp_buf) - temp_ret,
				"%4d.%01d", current_suspend, 0);
#endif
		temp_ret += snprintf(temp_buf + temp_ret, sizeof(temp_buf) - temp_ret,
				"/%3ld.%02ld/%3ld.%02ld/%3ld.%02ld/%3ld.%02ld",
				kern_percent/100, kern_percent%100,
				disp_percent/100, disp_percent%100,
				aod_percent/100, aod_percent%100,
				cp_percent/100, cp_percent%100);
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
		temp_ret += snprintf(temp_buf + temp_ret, sizeof(temp_buf) - temp_ret,
				"/%3d/%3d/%3d/%3d/%3d/%3d/%3d/%3d/%3d",
				diff_wakeup[0],
				diff_wakeup[1],
				diff_wakeup[2],
				diff_wakeup[3],
				diff_wakeup[4],
				diff_wakeup[5],
				diff_wakeup[6],
				diff_wakeup[7],
				diff_wakeup[8]);

		for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++) {
			total_time = ktime_add(total_time, p_curr->wakeup_time[i]);
			wakeup_time_ms[i] = ktime_to_ms(p_curr->wakeup_time[i]);
		}
		total_time_ms = ktime_to_ms(total_time);

		temp_ret += snprintf(temp_buf + temp_ret, sizeof(temp_buf) - temp_ret,
				"/%3lld/%3lld/%3lld/%3lld/%3lld/%3lld/%3lld/%3lld/%3lld/%3lld/%3lld\n",
				wakeup_time_ms[0] * 100 / total_time_ms,
				wakeup_time_ms[1] * 100 / total_time_ms,
				wakeup_time_ms[2] * 100 / total_time_ms,
				wakeup_time_ms[3] * 100 / total_time_ms,
				wakeup_time_ms[4] * 100 / total_time_ms,
				wakeup_time_ms[5] * 100 / total_time_ms,
				wakeup_time_ms[6] * 100 / total_time_ms,
				wakeup_time_ms[7] * 100 / total_time_ms,
				wakeup_time_ms[8] * 100 / total_time_ms,
				wakeup_time_ms[9] * 100 / total_time_ms,
				wakeup_time_ms[10] * 100 / total_time_ms);
#else
		temp_ret += snprintf(temp_buf + temp_ret, sizeof(temp_buf) - temp_ret,
				"/%3d/%3d/%3d/%3d/%3d/%3d/%3d\n",
				diff_wakeup[0],
				diff_wakeup[1],
				diff_wakeup[2],
				diff_wakeup[3],
				diff_wakeup[4],
				diff_wakeup[5],
				diff_wakeup[6],
				diff_wakeup[7],
				diff_wakeup[8]);
#endif
	} else {
		snprintf(temp_buf, sizeof(temp_buf), "\n");
	}
	rtc_time_to_tm(p_curr->ts_real.tv_sec + alarm_get_tz(), &tm_real);

	ret += snprintf(buf + ret, buf_size - ret,
		"%c/%03d/%c/%6d/%04d/%10lu/%04d-%02d-%02d %02d:%02d:%02d/%5s%s"
		, print_type, p_curr->log_count
		, bat_status
		, p_curr->bat_capacity, p_curr->suspend_count
		, p_curr->ts_real.tv_sec
		, tm_real.tm_year + 1900, tm_real.tm_mon + 1
		, tm_real.tm_mday, tm_real.tm_hour
		, tm_real.tm_min, tm_real.tm_sec
		, score_print
		, temp_buf);

	return ret;
}

static ssize_t energy_mon_summary_print(char *buf, int buf_size,
		ssize_t ret, enum energy_mon_print_type p_type)
{
	struct timespec dump_time;
	struct rtc_time tm_real;

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		/* Print out Energy monitor version */
		ret += snprintf(buf + ret, buf_size - ret,
				"energy_mon_status_raw/ver%s", ENERGY_MON_VERSION);

		/* Print out UTC and Local time */
		dump_time = current_kernel_time();
		rtc_time_to_tm(dump_time.tv_sec, &tm_real);
		ret += snprintf(buf + ret, buf_size - ret,
				"/%04d-%02d-%02d %02d:%02d:%02d(UTC)"
				, tm_real.tm_year + 1900, tm_real.tm_mon + 1
				, tm_real.tm_mday, tm_real.tm_hour
				, tm_real.tm_min, tm_real.tm_sec);
		rtc_time_to_tm(dump_time.tv_sec + alarm_get_tz(), &tm_real);
		ret += snprintf(buf + ret, buf_size - ret,
				"/%04d-%02d-%02d %02d:%02d:%02d(LOCAL)\n\n"
				, tm_real.tm_year + 1900, tm_real.tm_mon + 1
				, tm_real.tm_mday, tm_real.tm_hour
				, tm_real.tm_min, tm_real.tm_sec);

		ret += snprintf(buf + ret, buf_size - ret, "[summary]\n");
		ret += snprintf(buf + ret, buf_size - ret,
				"T/CNT/B/CAPA__/SUSP/UTC_TIME__/REAL_TIME_RTC_LOCAL/"
				"Score/"
				"dSOC__/dBOOT/dKERN/dDISP/_dAOD/dWOFF/"
				"_CUR_AVER/"
#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
				"HAVSC/"
#endif
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
				"KERN_%%/DISP_%%/AOD__%%/CP___%%"
				"/INP/SSP/RTC/BT_/WIF/CP_/GNS/NFC/ETC"
				"/INP/SSP/RTC/BT_/WIF/CP_/GNS/NFC/ETC/WU_/NT_\n");
#else
				"KERN_%%/DISP_%%/AOD__%%//CP___%%"
				"INP/SSP/RTC/BT_/WIF/CP_/GNS/NFC/ETC\n");
#endif
		ret += energy_monitor_print_time_logs(buf + ret, buf_size, ENERGY_MON_TYPE_BOOTING);
	} else if (p_type == ENERGY_MON_PRINT_MAIN)
		ret += energy_monitor_print_time_logs(buf + ret, buf_size, ENERGY_MON_TYPE_BATTERY);
	else {
		ret += energy_monitor_print_time_logs(buf + ret, buf_size, ENERGY_MON_TYPE_DUMP);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
		ret += energy_monitor_print_time_logs(buf + ret, buf_size, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_time_logs(buf + ret, buf_size, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_time_logs(buf + ret, buf_size, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_time_logs(buf + ret, buf_size, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	return ret;
}

static ssize_t energy_monitor_print_disp_stat(char *buf,
				int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < NUM_DIST_STAT_BR_LEVELS; i++)
			ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf),
						"/%07d", NUM_DIST_STAT_BR_LEVELS - i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[ds]\n%c/idx%s\n",
				print_type, temp_buf);
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < NUM_DIST_STAT_BR_LEVELS; i++)
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
				"/%07lld",
				ktime_to_ms(p_curr->disp_stat.fb_time[i]));
	ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
			"/%07lld",
			ktime_to_ms(p_curr->disp_stat.aod_time));

	ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
			"/%07u",
			p_curr->ce.disp_on);

	ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
			"/%07u",
			p_curr->ce.disp_aod);

	ret += snprintf(buf + ret, buf_size - ret,
			"%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

#ifdef CONFIG_INPUT_STAT
static ssize_t energy_monitor_print_input_stat_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	ret_temp = 0;
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.key_press);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.key_release);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.touch_press);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.touch_release);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[CounterClockwise]);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[Detent_Return]);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[Detent_Leave]);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[Clockwise]);
	for (i = 0; i < EV_WU_MAX; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%04u",
					p_curr->input_stat.ev_wakeup[i]);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	return ret;
}

static ssize_t energy_monitor_print_input_stat(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use*/
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
					"/k_p/k_r/t_p/t_r/wcc/w_r/w_l/w_c/"
					"ewtw/ew00/ew10/ew20/ew30/ew40/ew4x");

		ret += snprintf(buf + ret, buf_size - ret,
				"[is]\n%c/idx%s\n",
				print_type, temp_buf);
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret_temp = 0;
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.key_press);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.key_release);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.touch_press);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.touch_release);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[CounterClockwise]);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[Detent_Return]);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[Detent_Leave]);
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%03lu",
				p_curr->input_stat.direction[Clockwise]);
	for (i = 0; i < EV_WU_MAX; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%04u",
					p_curr->input_stat.ev_wakeup[i]);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_input_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_input_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_input_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_input_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}
#endif

#ifdef CONFIG_SENSORHUB_STAT
static ssize_t energy_monitor_print_sh_wakeup(char *buf,
				int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SHUB_LIB_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%02d", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[sh_wakeup]\n%c/idx%s\n",
				print_type, temp_buf);
	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < SHUB_LIB_MAX; i++)
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
				"/%02d",
				p_curr->sh_stat.wakeup[i]);

	ret += snprintf(buf + ret, buf_size - ret,
			"%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

static ssize_t energy_monitor_print_sh_event(char *buf,
				int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SHUB_LIB_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%02d", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[sh_event]\n%c/idx%s\n",
				print_type, temp_buf);
	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < SHUB_LIB_MAX; i++)
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
				"/%02d",
				p_curr->sh_stat.event[i]);

	ret += snprintf(buf + ret, buf_size - ret,
			"%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

#endif

#ifdef CONFIG_ALARM_HISTORY
static ssize_t energy_monitor_print_alarm_stat(char *buf, int buf_size,
		enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < ALARM_STAT_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@_sec", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[alarm]\n%c/idx%s\n",
				print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < ALARM_STAT_ARRAY_SIZE; i++) {
		ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%15s@%02lu%02lu",
				p_curr->alarm_stat[i].comm,
				p_curr->alarm_stat[i].wakeup_count,
				p_curr->alarm_stat[i].expire_count);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}
#endif

#ifdef CONFIG_SAP_PID_STAT
static ssize_t energy_monitor_print_sap_wakeup(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	struct energy_mon_data *p_prev = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Assign proper buffer to use */
	if (p_type == ENERGY_MON_PRINT_TITLE) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];

		if (energy_mon.data_index <= ENERGY_MON_HISTORY_NUM)
			p_prev = &energy_mon.boot;
		else
			p_prev = &energy_mon.data[(energy_mon.read_index +
				ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
		if (energy_mon.read_index == 0)
			p_prev = &energy_mon.boot;
		else
			p_prev = &energy_mon.data[(energy_mon.data_index +
				ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	} else if (p_type == ENERGY_MON_PRINT_MAIN) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
		p_prev = &energy_mon.data[(energy_mon.read_index +
			ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	} else {
		// TODO: Need to check return value
		// TODO: What shall I do if there is no valid case
		return 0;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SAP_STAT_SAPID_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp, "/%03d", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[sap_wakeup]\n%c/idx%s\n",print_type, temp_buf);
	}

	ret_temp = 0;

	if (p_prev) {
		for (i = 0; i < SAP_STAT_SAPID_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%03d",
						p_curr->sap_wakeup.wakeup_cnt[i] -
						p_prev->sap_wakeup.wakeup_cnt[i]);
	} else {
		for (i = 0; i < SAP_STAT_SAPID_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%03d",
						p_curr->sap_wakeup.wakeup_cnt[i]);
		}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p, p_prev=%p\n",
		__func__, p_curr, p_prev);
	return ret;
}

static ssize_t energy_monitor_print_sap_activity(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	struct energy_mon_data *p_prev = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Assign proper buffer to use */
	if (p_type == ENERGY_MON_PRINT_TITLE) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];

		if (energy_mon.data_index <= ENERGY_MON_HISTORY_NUM)
			p_prev = &energy_mon.boot;
		else
			p_prev = &energy_mon.data[(energy_mon.read_index +
				ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
		if (energy_mon.read_index == 0)
			p_prev = &energy_mon.boot;
		else
			p_prev = &energy_mon.data[(energy_mon.data_index +
				ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	} else if (p_type == ENERGY_MON_PRINT_MAIN) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
		p_prev = &energy_mon.data[(energy_mon.read_index +
			ENERGY_MON_HISTORY_NUM - 1) % ENERGY_MON_HISTORY_NUM];
	} else {
		// TODO: Need to check return value
		// TODO: What shall I do if there is no valid case
		return 0;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SAP_STAT_SAPID_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp, "/%03d", i);
		ret += snprintf(buf + ret, buf_size - ret, "[sap_activity]\n%c/idx%s\n"
			, print_type, temp_buf);
	}

	ret_temp = 0;

	if (p_prev) {
		for (i = 0; i < SAP_STAT_SAPID_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%03d",
						p_curr->sap_wakeup.activity_cnt[i] -
						p_prev->sap_wakeup.activity_cnt[i]);
	} else {
		for (i = 0; i < SAP_STAT_SAPID_MAX; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%03d",
						p_curr->sap_wakeup.activity_cnt[i]);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p, p_prev=%p\n",
		__func__, p_curr, p_prev);
	return ret;
}

static ssize_t energy_monitor_print_sap_traffic(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	struct energy_mon_data *p_prev = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	/* Assign proper buffer to use */
	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SAP_TRAFFIC_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@sndbytes(count)rcvbytes(count)", i);
		ret += snprintf(buf + ret, buf_size - ret, "[sap_traffic]\n%c/idx%s\n"
				, print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p, p_prev=%p\n", __func__, p_curr, p_prev);

	ret_temp = 0;
	for (i = 0; i < SAP_TRAFFIC_ARRAY_SIZE; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%15s@%8d(%5d)%8d(%5d)",
					p_curr->sap_traffic[i].aspid.id,
					p_curr->sap_traffic[i].snd,
					p_curr->sap_traffic[i].snd_count,
					p_curr->sap_traffic[i].rcv,
					p_curr->sap_traffic[i].rcv_count);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

#endif

#ifdef CONFIG_USID_STAT
static ssize_t energy_monitor_print_usid_stat_logs(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	/* Assign proper buffer to use */
	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < TCP_TRAFFIC_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@sndbytes(count)rcvbytes(count)", i);
		ret += snprintf(buf + ret, buf_size - ret, "[pid_stat]\n%c/idx%s\n"
				, print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n", __func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < TCP_TRAFFIC_ARRAY_SIZE; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%15s@%8d(%5d)%8d(%5d)",
					p_curr->tcp_traffic[i].comm,
					p_curr->tcp_traffic[i].snd,
					p_curr->tcp_traffic[i].snd_count,
					p_curr->tcp_traffic[i].rcv,
					p_curr->tcp_traffic[i].rcv_count);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}
#endif

#ifdef CONFIG_NET_STAT_TIZEN
static ssize_t energy_monitor_print_net_stat1_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%8lld(%5lld)%8lld(%5lld)",
					p_curr->net_stat[i].tx_bytes,
					p_curr->net_stat[i].tx_packets,
					p_curr->net_stat[i].rx_bytes,
					p_curr->net_stat[i].rx_packets);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	return ret;
}

static ssize_t energy_monitor_print_net_stat1(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use */
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/sndbytes(count)rcvbytes(count)");
		ret += snprintf(buf + ret, buf_size - ret, "[nstat1]\n%c/idx%s\n"
				, print_type, temp_buf);
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret_temp = 0;
	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%8lld(%5lld)%8lld(%5lld)",
					p_curr->net_stat[i].tx_bytes,
					p_curr->net_stat[i].tx_packets,
					p_curr->net_stat[i].rx_bytes,
					p_curr->net_stat[i].rx_packets);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_net_stat1_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_net_stat1_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_net_stat1_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_net_stat1_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}

static ssize_t energy_monitor_print_net_stat2_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++) {
		struct timespec up_time;

		up_time = ktime_to_timespec(p_curr->net_stat[i].up_time);
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%3d%3d%3d%3d%3d%3d%3d%5lu",
					p_curr->net_stat[i].state,
					p_curr->net_stat[i].up,
					p_curr->net_stat[i].down,
					p_curr->net_stat[i].wifi_state,
					p_curr->net_stat[i].connection,
					p_curr->net_stat[i].disconnection,
					p_curr->net_stat[i].connection_fail,
					up_time.tv_sec);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	return ret;
}

static ssize_t energy_monitor_print_net_stat2(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use */
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/  s  u  d  s  c  d  f   ut");
		ret += snprintf(buf + ret, buf_size - ret, "[nstat2]\n%c/idx%s\n"
				, print_type, temp_buf);

	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret_temp = 0;
	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++) {
		struct timespec up_time;

		up_time = ktime_to_timespec(p_curr->net_stat[i].up_time);
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%3d%3d%3d%3d%3d%3d%3d%5lu",
					p_curr->net_stat[i].state,
					p_curr->net_stat[i].up,
					p_curr->net_stat[i].down,
					p_curr->net_stat[i].wifi_state,
					p_curr->net_stat[i].connection,
					p_curr->net_stat[i].disconnection,
					p_curr->net_stat[i].connection_fail,
					up_time.tv_sec);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_net_stat2_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_net_stat2_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_net_stat2_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_net_stat2_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}

static ssize_t energy_monitor_print_net_stat3_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	memset(temp_buf, 0, sizeof(temp_buf));
	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
		if (p_curr->net_stat[i].scan_req)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%4d%4d %-8lld",
						p_curr->net_stat[i].scan_req,
						p_curr->net_stat[i].roaming_done,
						ktime_to_ms(p_curr->net_stat[i].scan_time));

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	return ret;
}

static ssize_t energy_monitor_print_net_stat3(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use */
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
					"/ req rom time");
		ret += snprintf(buf + ret, buf_size - ret, "[nstat3]\n%c/idx%s\n"
				, print_type, temp_buf);
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret_temp = 0;
	memset(temp_buf, 0, sizeof(temp_buf));
	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++)
		if (p_curr->net_stat[i].scan_req)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%4d%4d %-8lld",
						p_curr->net_stat[i].scan_req,
						p_curr->net_stat[i].roaming_done,
						ktime_to_ms(p_curr->net_stat[i].scan_time));

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_net_stat3_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_net_stat3_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_net_stat3_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_net_stat3_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}
#endif

#ifdef CONFIG_SLAVE_WAKELOCK
static ssize_t energy_monitor_print_slave_wakelock(char *buf,
		int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index % ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SLWL_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@_sec", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[slave_wakelock]\n%c/idx%s\n", print_type, temp_buf);
	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n", __func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < SLWL_ARRAY_SIZE; i++)
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
					"/%15s@%04ld",
					p_curr->slwl[i].slwl_name,
					ktime_to_timeval(p_curr->slwl[i].prevent_time).tv_sec);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n"
		, print_type, p_curr->log_count
		, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}
#endif

#ifdef CONFIG_PM_SLEEP
static ssize_t energy_monitor_print_wakeup_source(char *buf, int buf_size,
		enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < WS_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@_sec", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[ws]\n%c/idx%s\n",
				print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < WS_ARRAY_SIZE; i++) {
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%15s@%04ld",
					p_curr->ws[i].name,
					ktime_to_timeval(p_curr->ws[i].emon_total_time).tv_sec);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

static ssize_t energy_monitor_print_wakeup_count(char *buf, int buf_size,
		enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < WS_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@_cnt", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[wc]\n%c/idx%s\n",
				print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < WS_ARRAY_SIZE; i++) {
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%15s@%04ld",
						p_curr->ws_wu[i].name,
						p_curr->ws_wu[i].emon_wakeup_count);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

#endif

#ifdef CONFIG_SID_SYS_STATS
static ssize_t energy_monitor_print_sid_cputime_stats1(char *buf,
		int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SID_CPUTIME_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@_sec", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[sid]\n%c/idx%s\n",
				print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < SID_CPUTIME_ARRAY_SIZE; i++) {
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%7d@%08lld@%3u",
					p_curr->sid_cpu[i].usid,
					(unsigned long long)jiffies_to_msecs(
					cputime_to_jiffies(p_curr->sid_cpu[i].ttime)),
					p_curr->sid_cpu[i].permil);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

#ifdef CONFIG_PID_SYS_STATS
static ssize_t energy_monitor_print_sid_cputime_stats2(char *buf,
		int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SID_CPUTIME_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________________top%d@_sec", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[sid2]\n%c/idx%s\n",
				print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < SID_CPUTIME_ARRAY_SIZE; i++) {
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%15s@%08lld@%3u",
					p_curr->pid_cpu[i].comm,
					(unsigned long long)jiffies_to_msecs(
					cputime_to_jiffies(p_curr->pid_cpu[i].ttime)),
					p_curr->pid_cpu[i].permil);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}
#endif

static ssize_t energy_monitor_print_sid_io_stats(char *buf, int buf_size,
		enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n" , __func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	/* Assign proper buffer to use */
	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < SID_IO_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/_top%d@______rc(______rb)______wc(______wb)", i);
		ret += snprintf(buf + ret, buf_size - ret, "[io]\n%c/idx%s\n"
				, print_type, temp_buf);

	}

	ret_temp = 0;
	for (i = 0; i < SID_IO_ARRAY_SIZE; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%5d@%8llu(%8llu)%8llu(%8llu)",
					p_curr->sid_io[i].usid,
					p_curr->sid_io[i].rchar / 1024,
					p_curr->sid_io[i].read_bytes / 1024,
					p_curr->sid_io[i].wchar / 1024,
					p_curr->sid_io[i].write_bytes / 1024);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}
#endif

#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
static ssize_t energy_monitor_print_cpuidle_stat_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i, j;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	memset(temp_buf, 0, sizeof(temp_buf));
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/ %13llu",
				ktime_to_ms(p_curr->cpuidle_stat.total_stat_time));
	for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/ %13lld",
					ktime_to_ms(p_curr->cpuidle_stat.cpuidle[i].hp.online_time));
	for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/ %13llu",
					ktime_to_ms(p_curr->cpuidle_stat.cpuidle[i].total_idle_time));
	for (j = 0; j < p_curr->cpuidle_stat.state_count; j++)
		for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/ %13llu",
						ktime_to_ms(p_curr->cpuidle_stat.cpuidle[i].usage[j].time));
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/ %13llu",
				ktime_to_ms(p_curr->cpuidle_stat.lpm.usage.time));

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	return ret;
}

static ssize_t energy_monitor_print_cpuidle_stat(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i, j;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use */
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
					"/        active/     c0_active"
					"/     c1_active/ c0_total_idle"
					"/ c1_total_idle/    c0_cstate1"
					"/    c1_cstate1/    c0_cstate2"
					"/    c1_cstate2/           lpm");
		ret += snprintf(buf + ret, buf_size - ret, "[cpuidle]\n%c/idx%s\n"
				, print_type, temp_buf);
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret_temp = 0;
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/ %13llu",
				ktime_to_ms(p_curr->cpuidle_stat.total_stat_time));
	for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/ %13lld",
					ktime_to_ms(p_curr->cpuidle_stat.cpuidle[i].hp.online_time));
	for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/ %13llu",
					ktime_to_ms(p_curr->cpuidle_stat.cpuidle[i].total_idle_time));
	for (j = 0; j < p_curr->cpuidle_stat.state_count; j++)
		for (i = 0; i < p_curr->cpuidle_stat.cpu_count; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/ %13llu",
						ktime_to_ms(p_curr->cpuidle_stat.cpuidle[i].usage[j].time));
	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/ %13llu",
				ktime_to_ms(p_curr->cpuidle_stat.lpm.usage.time));

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_cpuidle_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_cpuidle_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_cpuidle_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_cpuidle_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}

static ssize_t energy_monitor_print_lpm_blocker(char *buf, int buf_size,
		enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char name[BLOCKER_NAME_SIZE] = {0,};
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < BLOCKER_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/___________top%d@_count", i);
		ret += snprintf(buf + ret, buf_size - ret,
				"[lpm_blocker]\n%c/idx%s\n",
				print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n",
		__func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < BLOCKER_ARRAY_SIZE; i++) {
		if (p_curr->lpm_blocker[i].name)
			memcpy(name, p_curr->lpm_blocker[i].name, BLOCKER_NAME_SIZE-1);
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/ %8s@%12lld",
					name,
					p_curr->lpm_blocker[i].count);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}
#endif

#ifdef CONFIG_CPU_FREQ_STAT_TIZEN
static ssize_t energy_monitor_print_cpufreq_stat_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	memset(temp_buf, 0, sizeof(temp_buf));
	for (i = 0; i < p_curr->cpufreq_stats.state_num; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%10llu",
					(unsigned long long)
					jiffies_to_msecs(p_curr->cpufreq_stats.time_in_state[i]));

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s/\n",
			print_type, p_curr->log_count,
			temp_buf);

	return ret;
}

static ssize_t energy_monitor_print_cpufreq_stats(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use */
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < p_curr->cpufreq_stats.state_num; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%10u", p_curr->cpufreq_stats.freq_table[i]);
		ret += snprintf(buf + ret, buf_size - ret, "[cpu_freq]\n%c/idx%s\n"
				, print_type, temp_buf);
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret_temp = 0;
	for (i = 0; i < p_curr->cpufreq_stats.state_num; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%10llu",
					(unsigned long long)
					jiffies_to_msecs(p_curr->cpufreq_stats.time_in_state[i]));

	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%10u",
				p_curr->ce.cpu_idle + p_curr->ce.cpu_active);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_cpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_cpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_cpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_cpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}
#endif

#ifdef CONFIG_GPUFREQ_STAT
static ssize_t energy_monitor_print_gpufreq_stat_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	memset(temp_buf, 0, sizeof(temp_buf));
	for (i = 0; i < p_curr->gpufreq_stat.table_size; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%10llu",
					(unsigned long long)
					jiffies_to_msecs(p_curr->gpufreq_stat.table[i].time));

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s/\n",
			print_type, p_curr->log_count,
			temp_buf);

	return ret;
}

static ssize_t energy_monitor_print_gpufreq_stat(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use*/
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < p_curr->gpufreq_stat.table_size; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"/%10u",
						p_curr->gpufreq_stat.table[i].clock);
		ret += snprintf(buf + ret, buf_size - ret, "[gpu_freq]\n%c/idx%s\n"
				, print_type, temp_buf);
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret_temp = 0;
	for (i = 0; i < p_curr->gpufreq_stat.table_size; i++)
		ret_temp += snprintf(temp_buf + ret_temp,
					sizeof(temp_buf) - ret_temp,
					"/%10llu",
					(unsigned long long)
					jiffies_to_msecs(p_curr->gpufreq_stat.table[i].time));

	ret_temp += snprintf(temp_buf + ret_temp,
				sizeof(temp_buf) - ret_temp,
				"/%10u",
				p_curr->ce.gpu);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count,
			temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_gpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_gpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_gpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_gpufreq_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}
#endif

#ifdef CONFIG_SENSORS_SEC_THERM_HISTORY
static ssize_t energy_monitor_print_sec_therm_history(char *buf,
	int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += snprintf(buf + ret, buf_size - ret, "\n");
		return ret;
	}

	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index % ENERGY_MON_HISTORY_NUM];

	/* Assign proper buffer to use */
	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < MAX_SEC_THERM_DEVICE_NUM; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
							sizeof(temp_buf) - ret_temp,
							"//___%12s ___",
							sec_therm_dev_name[i]); /* len : 20 */
		ret_temp += snprintf(temp_buf + ret_temp,
						sizeof(temp_buf) - ret_temp,
						"\n%c    ", print_type);
		for (i = 0; i < MAX_SEC_THERM_DEVICE_NUM; i++)
			ret_temp += snprintf(temp_buf + ret_temp,
							sizeof(temp_buf) - ret_temp,
							"//RND/MIN/MAX/AVG/CNT");
		ret += snprintf(buf + ret, buf_size - ret,
					"[therm_hist]\n%c/idx%s\n", print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n", __func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < MAX_SEC_THERM_DEVICE_NUM; i++)
		if (p_curr->therm_history[i].reset) {
			ret_temp += snprintf(temp_buf + ret_temp,
							sizeof(temp_buf) - ret_temp,
							"//  -/  -/  -/  -/  -");
		} else {
			ret_temp += snprintf(temp_buf + ret_temp,
							sizeof(temp_buf) - ret_temp,
							"//%3d/%3d/%3d/%3d/%3d",
							p_curr->therm_history[i].round,
							p_curr->therm_history[i].min,
							p_curr->therm_history[i].max,
							p_curr->therm_history[i].sum / p_curr->therm_history[i].cnt,
							p_curr->therm_history[i].cnt);
		}
	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n",
			print_type, p_curr->log_count, temp_buf);

	return ret;
}
#endif

#ifdef CONFIG_FF_STAT_TIZEN
static ssize_t energy_monitor_print_ff_stat_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0;
	char print_type;
	struct energy_mon_data *p_curr = NULL;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/",
			print_type, p_curr->log_count);
	ret += snprintf(buf + ret, buf_size - ret, "%12lld/",
			ktime_to_ms(p_curr->ff_stat.total_time));
	ret += snprintf(buf + ret, buf_size - ret, "%12lu/\n",
			p_curr->ff_stat.play_count);

	return ret;
}

static ssize_t energy_monitor_print_ff_stat(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0;
	struct energy_mon_data *p_curr = NULL;
	char print_type;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use*/
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret += snprintf(buf + ret, buf_size - ret, "[ff]\n");
		ret += snprintf(buf + ret, buf_size - ret,
				"%c/idx/________time/_______count/\n", print_type);
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/",
			print_type, p_curr->log_count);
	ret += snprintf(buf + ret, buf_size - ret, "%12lld/",
			ktime_to_ms(p_curr->ff_stat.total_time));
	ret += snprintf(buf + ret, buf_size - ret, "%12lu/",
			p_curr->ff_stat.play_count);
	ret += snprintf(buf + ret, buf_size - ret, "%12u/\n",
			p_curr->ce.ff);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_ff_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_ff_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_ff_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_ff_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);
	return ret;
}
#endif

#ifdef CONFIG_SLEEP_STAT
static ssize_t energy_monitor_print_sleep_stat_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0;
	char print_type;
	struct energy_mon_data *p_curr = NULL;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/",
			print_type, p_curr->log_count);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.fail);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_freeze);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_prepare);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_suspend);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_suspend_late);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_suspend_noirq);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.suspend_success);
	if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS) {
		int i;

		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sleep_early_wakeup);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sleep_soc_down);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sleep_mif_down);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sicd_early_wakeup);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sicd_soc_down);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sicd_mif_down);

		for (i = 0; i < EXYNOS_MIF_MASTER_MAX - 2; i++) {
			ret += snprintf(buf + ret, buf_size - ret, "%5d/",
					p_curr->sleep_stat.soc.exynos.apm_wakeup[i].tcxo_req_count);
		}

		for (i = 0; i < EXYNOS_MIF_MASTER_MAX; i++) {
			ret += snprintf(buf + ret, buf_size - ret, "%8lld/",
					p_curr->sleep_stat.soc.exynos.apm[i].total_time);
		}
	}
	ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}

static ssize_t energy_monitor_print_sleep_stat(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0;
	struct energy_mon_data *p_curr = NULL;
	char print_type;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use*/
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret += snprintf(buf + ret, buf_size - ret, "[sleep_stat]\n");
		ret += snprintf(buf + ret, buf_size - ret, "%c/idx", print_type);
		ret += snprintf(buf + ret, buf_size - ret,
				"/ fail/ ffrz/ fprp/ fsup/ fsul/ fsun/ sups");
		if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS) {
			ret += snprintf(buf + ret, buf_size - ret,
					"/ slew/ slsd/ slmd/ siew/ sisd/ simd");
			ret += snprintf(buf + ret, buf_size - ret,
					"/ xocp/xoaud/xogns/xovts/xowlb/xohub");
			ret += snprintf(buf + ret, buf_size - ret,
					"/  mif_cp/ mif_aud/mif_gnss/ mif_vts/mif_wlbt/mif_chub/  mif_ap/apm_chub");
		}
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/",
			print_type, p_curr->log_count);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.fail);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_freeze);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_prepare);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_suspend);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_suspend_late);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.failed_suspend_noirq);
	ret += snprintf(buf + ret, buf_size - ret, "%5d/",
			p_curr->sleep_stat.suspend_success);
	if (p_curr->sleep_stat.soc_type == SLEEP_STAT_SOC_EXYNOS) {
		int i;

		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sleep_early_wakeup);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sleep_soc_down);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sleep_mif_down);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sicd_early_wakeup);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sicd_soc_down);
		ret += snprintf(buf + ret, buf_size - ret, "%5d/",
				p_curr->sleep_stat.soc.exynos.acpm_sicd_mif_down);

		for (i = 0; i < EXYNOS_MIF_MASTER_MAX - 2; i++) {
			ret += snprintf(buf + ret, buf_size - ret, "%5d/",
					p_curr->sleep_stat.soc.exynos.apm_wakeup[i].tcxo_req_count);
		}

		for (i = 0; i < EXYNOS_MIF_MASTER_MAX; i++) {
			ret += snprintf(buf + ret, buf_size - ret, "%8lld/",
					p_curr->sleep_stat.soc.exynos.apm[i].total_time);
		}
	}
	ret += snprintf(buf + ret, buf_size - ret, "\n");

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_sleep_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_sleep_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_sleep_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_sleep_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);
	return ret;
}
#endif

#ifdef CONFIG_SENSORHUB_STAT // GPS, HR
static ssize_t energy_monitor_print_gps_stat_dump(
	char *buf, int buf_size, enum energy_mon_type type)
{
	ssize_t ret = 0;
	char print_type;
	struct energy_mon_data *p_curr = NULL;

	if (!buf || buf_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: invalid arguments\n", __func__);
		return ret;
	}

	if (type == ENERGY_MON_TYPE_CHARGING) {
		print_type = 'C';
		p_curr = &energy_mon.charging_dump;
	} else if (type == ENERGY_MON_TYPE_DISCHARGING){
		print_type = 'D';
		p_curr = &energy_mon.discharging_dump;
	} else if (type == ENERGY_MON_TYPE_NOT_CHARGING){
		print_type = 'N';
		p_curr = &energy_mon.not_charging_dump;
	} else if (type == ENERGY_MON_TYPE_FULL) {
		print_type = 'F';
		p_curr = &energy_mon.full_dump;
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: %d does not support\n", __func__, type);
		return ret;
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/%04d/\n",
			print_type, p_curr->log_count, p_curr->sh_gps.gps_time);

	return ret;
}

static ssize_t energy_monitor_print_gps_stat(
	char *buf, int buf_size, enum energy_mon_print_type p_type)
{
	ssize_t ret = 0;
	struct energy_mon_data *p_curr = NULL;
	char print_type;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n", __func__, buf_size, p_type);

	/* Set print type & assign proper buffer to use*/
	print_type = '*';
	p_curr = &energy_mon.data[energy_mon.read_index %
		ENERGY_MON_HISTORY_NUM];

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret += snprintf(buf + ret, buf_size - ret, "[gps]\n");
		ret += snprintf(buf + ret, buf_size - ret, "%c/idx/time/", print_type);
		ret += snprintf(buf + ret, buf_size - ret, "___last_gps_user/ext/\n");
	} else if (p_type == ENERGY_MON_PRINT_TAIL) {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/%04d/",
			print_type, p_curr->log_count, p_curr->sh_gps.gps_time);
	ret += snprintf(buf + ret, buf_size - ret, "%016llx/",
			p_curr->sh_gps.last_gps_user);
	ret += snprintf(buf + ret, buf_size - ret, " %02u/\n",
			p_curr->sh_gps.last_gps_ext);

	if (p_type == ENERGY_MON_PRINT_TAIL) {
		ret += energy_monitor_print_gps_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_CHARGING);
		ret += energy_monitor_print_gps_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_DISCHARGING);
		ret += energy_monitor_print_gps_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_NOT_CHARGING);
		ret += energy_monitor_print_gps_stat_dump(buf + ret,
			buf_size - ret, ENERGY_MON_TYPE_FULL);
		ret += snprintf(buf + ret, buf_size - ret, "\n");
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}

#if 0
static ssize_t energy_monitor_print_hr_stat(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0;
	struct energy_mon_data *p_curr = NULL;
	char print_type;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret += snprintf(buf + ret, buf_size - ret, "[hr]\n");
		ret += snprintf(buf + ret, buf_size - ret, "%c/idx/succ/cn/fail/cn/\n",
				print_type);
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/%04d/%02d/%04d/%02d/\n",
			print_type, p_curr->log_count,
			p_curr->sh_hr.success_dur, p_curr->sh_hr.success_cnt,
			p_curr->sh_hr.fail_dur, p_curr->sh_hr.fail_cnt);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n", __func__, p_curr);

	return ret;
}
#endif
#endif

#ifdef CONFIG_LBS_STAT
static ssize_t energy_monitor_print_lbs_stat(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0, ret_temp = 0;
	struct energy_mon_data *p_curr = NULL;
	char temp_buf[300];
	char print_type;
	int i;
	ktime_t gps_time;
	s64 gps_time_ms;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n",
		__func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	/* Assign proper buffer to use */
	if (p_type == ENERGY_MON_PRINT_TITLE) {
		for (i = 0; i < LBS_STAT_ARRAY_SIZE; i++)
			ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf)-ret_temp,
						"/_top%d@____________msec", i);
		ret += snprintf(buf + ret, buf_size - ret, "[lbs_stat]\n%c/idx%s\n"
				, print_type, temp_buf);

	}
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n", __func__, p_curr);

	ret_temp = 0;
	for (i = 0; i < LBS_STAT_ARRAY_SIZE; i++) {
		gps_time = ktime_add(p_curr->lbs_stat[i].usage[LBS_METHOD_GPS].total_time,
			p_curr->lbs_stat[i].usage[LBS_METHOD_BATCH_GPS].total_time);
		gps_time_ms = ktime_to_ms(gps_time);
		if (gps_time_ms > 3600000)
			gps_time_ms = 3600000;
		ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
					"/%5d@%7lld(%7lld)",
					p_curr->lbs_stat[i].usid,
					ktime_to_ms(p_curr->lbs_stat[i].total_time),
					gps_time_ms);
	}
	ret_temp += snprintf(temp_buf + ret_temp, sizeof(temp_buf) - ret_temp,
				"/%5d",
				p_curr->ce.lbs);

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d%s\n"
			, print_type, p_curr->log_count
			, temp_buf);

	if (p_type == ENERGY_MON_PRINT_TAIL)
		ret += snprintf(buf + ret, buf_size - ret, "\n");

	return ret;
}
#endif

static ssize_t energy_monitor_print_ce(char *buf, int buf_size,
	enum energy_mon_print_type p_type)
{
	ssize_t ret = 0;
	unsigned int sum;
	struct energy_mon_data *p_curr = NULL;
	char print_type;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: buf_size=%d, p_type=%d\n"
		, __func__, buf_size, p_type);

	/* Set print type */
	if (p_type != ENERGY_MON_PRINT_TAIL) {
		print_type = '*';
		p_curr = &energy_mon.data[energy_mon.read_index %
			ENERGY_MON_HISTORY_NUM];
	} else {
		print_type = 'd';
		p_curr = &energy_mon.dump;
	}

	if (p_type == ENERGY_MON_PRINT_TITLE) {
		ret += snprintf(buf + ret, buf_size - ret, "[ce]\n");
		ret += snprintf(buf + ret, buf_size - ret, "%c/idx", print_type);
		ret += snprintf(buf + ret, buf_size - ret,
				"/    da/    do/    sl/    ci/    ca/    ga/    ff/"
				"    gp/    lb/    hr/    bt/    wf/    cp/   sum\n");
	}

	ret += snprintf(buf + ret, buf_size - ret, "%c/%03d/",
			print_type, p_curr->log_count);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.disp_aod);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.disp_on);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.sleep);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.cpu_idle);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.cpu_active);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.gpu);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.ff);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.gps);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.lbs);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.hr);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.bt);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.wifi);
	ret += snprintf(buf + ret, buf_size - ret, "%6d/",
			p_curr->ce.cp);
	sum = p_curr->ce.disp_aod + p_curr->ce.disp_on +
		p_curr->ce.sleep + p_curr->ce.cpu_idle + p_curr->ce.cpu_active +
		p_curr->ce.gpu + p_curr->ce.ff +
		p_curr->ce.gps + p_curr->ce.lbs + p_curr->ce.hr + p_curr->ce.bt +
		p_curr->ce.wifi + p_curr->ce.cp;
	ret += snprintf(buf + ret, buf_size - ret, "%6d/ %4d\n",
			sum, sum / energy_mon.capacity_per_soc);

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: p_curr=%p\n", __func__, p_curr);
	return ret;
}

static ssize_t energy_mon_print(char *buf, int buf_size, ssize_t ret,
	enum energy_mon_print_step print_step, enum energy_mon_print_type p_type)
{
	switch (print_step) {
	case STEP_SUMMARY:
		ret += energy_mon_summary_print(buf, buf_size, ret, p_type);
		break;
	case STEP_DISP_STAT:
		ret += energy_monitor_print_disp_stat(buf + ret, buf_size, p_type);
		break;
#ifdef CONFIG_INPUT_STAT
	case STEP_INPUT_STAT:
		ret += energy_monitor_print_input_stat(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_SENSORHUB_STAT
	case STEP_SENSORHUB_WAKEUP:
		ret += energy_monitor_print_sh_wakeup(buf + ret, buf_size, p_type);
		break;
	case STEP_SENSORHUB_ACTIVITY:
		ret += energy_monitor_print_sh_event(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_ALARM_HISTORY
	case STEP_ALARM_STAT:
		ret += energy_monitor_print_alarm_stat(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_SAP_PID_STAT
	case STEP_SAP_WAKEUP:
		ret += energy_monitor_print_sap_wakeup(buf + ret, buf_size, p_type);
		break;
	case STEP_SAP_ACTIVITY:
		ret += energy_monitor_print_sap_activity(buf + ret, buf_size, p_type);
		break;
	case STEP_SAP_TRAFFIC:
		ret += energy_monitor_print_sap_traffic(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_USID_STAT
	case STEP_USID_STAT:
		ret += energy_monitor_print_usid_stat_logs(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_NET_STAT_TIZEN
	case STEP_NET_STAT1:
		ret += energy_monitor_print_net_stat1(buf + ret, buf_size, p_type);
		break;
	case STEP_NET_STAT2:
		ret += energy_monitor_print_net_stat2(buf + ret, buf_size, p_type);
		break;
	case STEP_NET_STAT3:
		ret += energy_monitor_print_net_stat3(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_SLAVE_WAKELOCK
	case STEP_SLAVE_WAKELOCK:
		ret += energy_monitor_print_slave_wakelock(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_PM_SLEEP
	case STEP_WAKEUP_SOURCE:
		ret += energy_monitor_print_wakeup_source(buf + ret, buf_size, p_type);
		break;
	case STEP_WAKEUP_COUNT:
		ret += energy_monitor_print_wakeup_count(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_SID_SYS_STATS
	case STEP_SID_CPUTIME_STATS1:
		ret += energy_monitor_print_sid_cputime_stats1(buf + ret, buf_size, p_type);
		break;
#ifdef CONFIG_PID_SYS_STATS
	case STEP_SID_CPUTIME_STATS2:
		ret += energy_monitor_print_sid_cputime_stats2(buf + ret, buf_size, p_type);
		break;
#endif
	case STEP_SID_IO_STATS:
		ret += energy_monitor_print_sid_io_stats(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_ARM64_EXYNOS_CPUIDLE_STAT
	case STEP_CPUIDLE_STAT:
		ret += energy_monitor_print_cpuidle_stat(buf + ret, buf_size, p_type);
		break;
	case STEP_CPUIDLE_LPM_BLOCKER:
		ret += energy_monitor_print_lpm_blocker(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_CPU_FREQ_STAT_TIZEN
	case STEP_CPUFREQ_STATS:
		ret += energy_monitor_print_cpufreq_stats(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_GPUFREQ_STAT
	case STEP_GPUFREQ_STAT:
		ret += energy_monitor_print_gpufreq_stat(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_SENSORS_SEC_THERM_HISTORY
	case STEP_SEC_THERM_HISTORY:
		ret += energy_monitor_print_sec_therm_history(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_FF_STAT_TIZEN
	case STEP_FF_STAT:
		ret += energy_monitor_print_ff_stat(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_SLEEP_STAT
	case STEP_SLEEP_STAT:
		ret += energy_monitor_print_sleep_stat(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_SENSORHUB_STAT
	case STEP_GPS_STAT:
		ret += energy_monitor_print_gps_stat(buf + ret, buf_size, p_type);
		break;
#endif
#ifdef CONFIG_LBS_STAT
	case STEP_LBS_STAT:
		ret += energy_monitor_print_lbs_stat(buf + ret, buf_size, p_type);
		break;
#endif
	case STEP_CE:
		ret += energy_monitor_print_ce(buf + ret, buf_size, p_type);
		break;
	default:
		break;
	}

	return ret;
}

static ssize_t read_status_raw(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	static char buf[PAGE_SIZE];
	static enum energy_mon_print_step print_step;
	static int need_to_print_title;
	ssize_t ret = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s:read_index=%d, write_index=%d\n",
		__func__, energy_mon.read_index, energy_mon.data_index);

	if (*ppos == 0) {
		energy_monitor_marker(ENERGY_MON_TYPE_DUMP);
		energy_monitor_inject_data(ENERGY_MON_TYPE_DUMP, NULL, NULL);

		/* Start print step */
		print_step = STEP_SUMMARY;
		/* Initialize read index */
		energy_mon.read_index = energy_mon.data_index;
		ret += energy_mon_print(buf, sizeof(buf), ret, print_step, ENERGY_MON_PRINT_TITLE);
		need_to_print_title = 0;
	} else if (print_step != STEP_MAX) {
		 if (energy_mon.data_index == 0 || is_last_read_index()) {
			ret += energy_mon_print(buf, sizeof(buf), ret, print_step, ENERGY_MON_PRINT_TAIL);

			/* Go to next print_step */
			print_step++;
			/* Initialize read index */
			energy_mon.read_index = energy_mon.data_index;
			need_to_print_title = 1;
		} else {
			/* Skip buffer when it is not used yet */
			while (energy_mon.data[energy_mon.read_index % ENERGY_MON_HISTORY_NUM].log_count == -1)
				energy_mon.read_index++;

			if (need_to_print_title == 1) {
				ret += energy_mon_print(buf, sizeof(buf), ret, print_step, ENERGY_MON_PRINT_TITLE);
				need_to_print_title = 0;
			} else
				ret += energy_mon_print(buf, sizeof(buf), ret, print_step, ENERGY_MON_PRINT_MAIN);

			energy_mon.read_index++;
		}
	}

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret))
			return -EFAULT;
		*ppos += ret;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: ret = %d\n", __func__, (int)ret);

	return ret;
}

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
static char *energy_mononitor_get_ws_name_string(int wakeup_idx)
{
	/* Need to make sync with enum energy_mon_wakeup_source in the energy_monitor.h */
	static char *wakeup_text[] = {
		"INPUT", "SSP", "RTC", "BT", "WIFI", "CP", "GNSS", "NFC", "ETC", "WU", "NT"
	};

	if (wakeup_idx >= ENERGY_MON_WAKEUP_MAX)
		return NULL;

	return wakeup_text[wakeup_idx];
}

static ssize_t read_status_wakeup_time(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char buf[1536];
	int i, j;
	int wakeup_count;
	s64 wakeup_time_ms, average_time_ms, total_time_ms;
	ktime_t total_time = ktime_set(0,0);

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		ret += snprintf(buf + ret, sizeof(buf) - ret,
				"WAKEUP/CNT/__TIME_TOTAL/TIME_AVERA/PCT\n");
		for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
			total_time = ktime_add(total_time, energy_mon.wakeup_time[i]);
		total_time_ms = ktime_to_ms(total_time);

		for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++) {
			if (i == energy_mon.last_wakeup_idx)
				wakeup_count = energy_mon.wakeup_cause[i] - 1;
			else
				wakeup_count = energy_mon.wakeup_cause[i];

			wakeup_time_ms = ktime_to_ms(energy_mon.wakeup_time[i]);
			if (wakeup_count)
				average_time_ms = wakeup_time_ms / wakeup_count;
			else
				average_time_ms = 0;

			ret += snprintf(buf + ret, sizeof(buf) - ret,
				"%6s/%3d/%12lld/%10lld/%3lld\n",
				energy_mononitor_get_ws_name_string(i), wakeup_count,
				wakeup_time_ms, average_time_ms,
				wakeup_time_ms * 100 / total_time_ms);
		}

		for (i = 0; i < ENERGY_MON_MAX_WAKEUP_STAT_TIME + 1; i++) {
			ret += snprintf(buf + ret, sizeof(buf) - ret,
					"%2d ~ %2ds : ", i, i + 1);
			for (j = 0; j < ENERGY_MON_WAKEUP_MAX; j++) {
				ret += snprintf(buf + ret, sizeof(buf) - ret,
						"%3d ", energy_mon.wakeup_time_stats[j][i]);
			}
			ret += snprintf(buf + ret, sizeof(buf) - ret, "\n");
		}
	}

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret))
			return -EFAULT;
		*ppos += ret;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: ret = %d\n", __func__, (int)ret);

	if (ret > sizeof(buf))
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: buffer overflow!!! ret = %d\n", __func__, (int)ret);

	return ret;
}
#endif

#ifdef CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR
static ssize_t read_sleep_current(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	static char buf[800];
	int i;
	long long average_sum = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		ret += snprintf(buf + ret, sizeof(buf) - ret, "CNT/AVERAGE_CUR\n");
		for (i = 0; i < energy_mon.estimator_index; i++) {
			ret += snprintf(buf + ret, sizeof(buf) - ret,
					"%3d/%3ld.%02ldmA\n",
					i,
					(long)energy_mon.estimator_average[i]/100,
					(long)energy_mon.estimator_average[i]%100);
			average_sum += energy_mon.estimator_average[i];
		}
		if (energy_mon.estimator_index != 0) {
			do_div(average_sum, energy_mon.estimator_index);
			ret += snprintf(buf + ret, sizeof(buf) - ret,
					"avg/%3ld.%02ldmA\n",
					(long)average_sum / 100,
					(long)average_sum % 100);
		}
	}

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret))
			return -EFAULT;
		*ppos += ret;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: ret = %d\n", __func__, (int)ret);

	if (ret > sizeof(buf))
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: buffer overflow!!! ret = %d\n", __func__, (int)ret);

	return ret;
}
#endif /* CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR */

static ssize_t read_monitor_interval(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char buf[10];

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0)
		ret += snprintf(buf, sizeof(buf), "%u\n", monitor_interval);

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret))
			return -EFAULT;
		*ppos += ret;
	}

	return ret;
}

static ssize_t write_monitor_interval(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{
	unsigned int new_interval = 0;
	char buf[16];
	ssize_t len;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	if (!kstrtouint(buf, 0, &new_interval)) {
		/* Maximum interval is 1 day */
		if (new_interval > ENERGY_MON_MAX_MONITOR_INTERVAL) {
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s: max interval is 1 day\n", __func__);
			return -EINVAL;
		}
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: kstrtouint is failed\n", __func__);
		return -EINVAL;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"monitor interval: old=%d new=%d\n", monitor_interval, new_interval);
	monitor_interval = new_interval;

	if (monitor_interval > 0) {
		schedule_delayed_work(&monitor_work, monitor_interval * HZ);
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: monitor thread is started\n", __func__);
	} else if (monitor_interval == 0) {
		cancel_delayed_work(&monitor_work);
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: monior thread is canceled\n", __func__);
	}

	return count;
}

static ssize_t read_logging_interval(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char buf[16];

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0)
		ret += snprintf(buf, sizeof(buf), "%u\n", logging_interval);

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret))
			return -EFAULT;
		*ppos += ret;
	}

	return ret;
}

static ssize_t write_logging_interval(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	unsigned int new_interval = 0;
	char buf[16];
	ssize_t len;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	if (!kstrtouint(buf, 0, &new_interval)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"logging interval: old=%d new=%d\n", logging_interval, new_interval);
		logging_interval = new_interval;
	} else
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: kstrtouint is failed\n", __func__);

	return len;
}

static const struct file_operations status_raw_fops = {
	.read = read_status_raw,
};

static const struct file_operations status_raw2_fops = {
	.read = read_status_raw,
};

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
static const struct file_operations status_wakeup_time_fops = {
	.read = read_status_wakeup_time,
};
#endif /* CONFIG_ENERGY_MONITOR_WAKEUP_STAT */

#ifdef CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR
static const struct file_operations sleep_current_fops = {
	.read = read_sleep_current,
};
#endif /* CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR */

static const struct file_operations monitor_interval_fops = {
	.write		= write_monitor_interval,
	.read		= read_monitor_interval,
};

static const struct file_operations logging_interval_fops = {
	.write		= write_logging_interval,
	.read		= read_logging_interval,
};

#ifdef CONFIG_SEC_SYSFS
static ssize_t energy_monitor_get_last_discharging_info(
	struct device *dev, struct device_attribute *attr, char *buf) {
	static struct energy_mon_data last_discharging_info;
	static bool is_first = true;
	int diff_soc;
	__kernel_time_t diff_boot = 0;

	energy_monitor_marker(ENERGY_MON_TYPE_DUMP);
	energy_monitor_inject_data(ENERGY_MON_TYPE_DUMP, NULL, NULL);

	if (is_first) {
		diff_soc = energy_mon.discharging_dump.bat_capacity * -1;
		diff_boot = energy_mon.discharging_dump.ts_boot.tv_sec;
		is_first = false;
	} else {
		diff_soc = (energy_mon.discharging_dump.bat_capacity -
						last_discharging_info.bat_capacity) * -1;
		diff_boot = energy_mon.discharging_dump.ts_boot.tv_sec -
						last_discharging_info.ts_boot.tv_sec;
	}

	if (diff_soc < 0)
		diff_soc = 0;
	if (diff_boot < 0)
		diff_boot = 0;

	memcpy(&last_discharging_info, &energy_mon.discharging_dump, sizeof(struct energy_mon_data));

	return scnprintf(buf, PAGE_SIZE, "dSOC:%d dBOOT:%lu\n", diff_soc, diff_boot);
}
static DEVICE_ATTR(get_last_discharging_info, 0440,
	energy_monitor_get_last_discharging_info,
	NULL);

static ssize_t energy_monitor_show_penalty_notify_mask(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret += scnprintf(buf, PAGE_SIZE, "0x%x\n", energy_mon.penalty.notify_mask);

	return ret;
}

static ssize_t energy_monitor_store_penalty_notify_mask(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int err;
	unsigned int request_val;
	unsigned int notify_mask;
	unsigned int old_mask;
	unsigned int new_mask;
	char mask_str[18] = {0,};
	char *sptr, *token;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %s\n", __func__, buf);

	if (count > 17)
		return -EINVAL;

	scnprintf(mask_str, sizeof(mask_str)-1, "%s", buf);
	sptr = mask_str;

	/* request penalty */
	token = strsep(&sptr, ":");
	if (token == NULL)
		return -EINVAL;
	err = kstrtouint(token, 16, &request_val);
	if (err < 0)
		return err;

	/* penalty mask */
	token = strsep(&sptr, ":");
	if (token == NULL)
		return -EINVAL;
	err = kstrtouint(token, 16, &notify_mask);
	if (err < 0)
		return err;

	old_mask = energy_mon.penalty.notify_mask;
	new_mask = (old_mask & ~notify_mask) | request_val;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: 0x%x 0x%x 0x%x 0x%x\n",
		__func__, request_val, notify_mask, old_mask, new_mask);

	energy_mon.penalty.notify_mask = new_mask;

	return count;
}

static DEVICE_ATTR(penalty_notify_mask, 0600,
	energy_monitor_show_penalty_notify_mask,
	energy_monitor_store_penalty_notify_mask);

static ssize_t energy_monitor_show_penalty_score(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret += scnprintf(buf, PAGE_SIZE, "%d\n", energy_mon.penalty.last_score);

	return ret;
}

static ssize_t energy_monitor_store_penalty_score(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned int val;
	int err;

	err = kstrtouint(buf, 10, &val);
	if (err)
		return err;

	if (val > ENERGY_MON_SCORE_MAX)
		return -EINVAL;

	energy_mon.penalty.last_score = val;

	return count;
}

static DEVICE_ATTR(penalty_score, 0660,
	energy_monitor_show_penalty_score,
	energy_monitor_store_penalty_score);

static ssize_t energy_monitor_store_penalty_threshold(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	const char *str;
	size_t len;
	char threshold_type[10] = {0,};
	int threshold = 0;
	int err;

	str = buf;

	while (*str && !isspace(*str))
		str++;

	len = str - buf;
	if (!len || len >= sizeof(threshold_type))
		return -EINVAL;

	if (*str && *str != '\n') {
		/* Find out if there's a threshold string appended. */
		err = kstrtos32(skip_spaces(str), 10, &threshold);
		if (err)
			return -EINVAL;
	}

	memcpy(threshold_type, buf, len);

	if (!strcmp(threshold_type, "time"))
		energy_mon.penalty.threshold_time = threshold;

	if (!strcmp(threshold_type, "short_cpu_usage"))
		energy_mon.penalty.threshold_short_cpu_usage = threshold;

	if (!strcmp(threshold_type, "batt"))
		energy_mon.penalty.threshold_batt = threshold * 1000; /* uA */

	if (!strcmp(threshold_type, "sbm_batt"))
		energy_mon.penalty.threshold_sbm_batt = threshold * 1000; /* uA */

	if (!strcmp(threshold_type, "disp"))
		energy_mon.penalty.threshold_disp = threshold;

	if (!strcmp(threshold_type, "input"))
		energy_mon.penalty.threshold_input = threshold;

	if (!strcmp(threshold_type, "alarm_mgr"))
		energy_mon.penalty.threshold_alarm_mgr = threshold;

	if (!strcmp(threshold_type, "alarm"))
		energy_mon.penalty.threshold_alarm = threshold;

	if (!strcmp(threshold_type, "sh"))
		energy_mon.penalty.threshold_sh = threshold;

	if (!strcmp(threshold_type, "sh_wristup"))
		energy_mon.penalty.threshold_sh_wristup = threshold;

	if (!strcmp(threshold_type, "gps"))
		energy_mon.penalty.threshold_gps = threshold;

	if (!strcmp(threshold_type, "lbs"))
		energy_mon.penalty.threshold_lbs = threshold;

	if (!strcmp(threshold_type, "cpuidle"))
		energy_mon.penalty.threshold_cpuidle= threshold;

	if (!strcmp(threshold_type, "ws"))
		energy_mon.penalty.threshold_ws = threshold;

	if (!strcmp(threshold_type, "slwl"))
		energy_mon.penalty.threshold_slwl = threshold;

	if (!strcmp(threshold_type, "suspend_fail"))
		energy_mon.penalty.threshold_suspend_fail = threshold;

	if (!strcmp(threshold_type, "sid"))
		energy_mon.penalty.threshold_sid = threshold;

	if (!strcmp(threshold_type, "pid"))
		energy_mon.penalty.threshold_pid = threshold;

	if (!strcmp(threshold_type, "booting_offset"))
		energy_mon.penalty.threshold_booting_offset = threshold;

	if (!strcmp(threshold_type, "log_offset"))
		energy_mon.penalty.threshold_log_offset = threshold;

	energy_mon.penalty.last_threshold_change = ktime_get_boottime();

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %s: %d\n",
			__func__, threshold_type, threshold);

	return count;
}

static DEVICE_ATTR(penalty_threshold, 0200,
	NULL,
	energy_monitor_store_penalty_threshold);

static ssize_t energy_monitor_show_penalty_cause(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret += scnprintf(buf, PAGE_SIZE, "%s\n", ENERGY_MON_SCORE_VERSION);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 0 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 1 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 2 */

	/* Bit 3 */
	if (energy_mon.penalty.last_score & ~ENERGY_MON_SCORE_WS) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%s\n",
				energy_mon.fbdc.ws_name);
	}

	/* Bit 4 */
	if (energy_mon.penalty.last_score & ~ENERGY_MON_SCORE_SLAVE_WAKELOCK) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%s\n",
				energy_mon.fbdc.slwl_name);
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 5 */

	/* Bit 6 */
	if (energy_mon.penalty.last_score & ~ENERGY_MON_SCORE_LBS) {
		char *ctx = NULL;
		unsigned n;

		if (security_secid_to_secctx(energy_mon.fbdc.lbs_sid, &ctx, &n) == 0) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%s \n", ctx);
			security_release_secctx(ctx, n);
		}
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 7 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 8 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 9 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 10 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 11 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 12 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 13 */
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n"); /* Bit 14 */

	/* Bit 15 */
	if (energy_mon.penalty.last_score & ~ENERGY_MON_SCORE_CPU_TIME) {
		if (energy_mon.fbdc.cpu_sid >= 5000) {
			char *ctx = NULL;
			unsigned n;

			if (security_secid_to_secctx(energy_mon.fbdc.cpu_sid, &ctx, &n) == 0) {
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%s \n", ctx);
				security_release_secctx(ctx, n);
			}
		}
	}

	return ret;
}

static DEVICE_ATTR(penalty_cause, 0400,
	energy_monitor_show_penalty_cause,
	NULL);

static ssize_t energy_monitor_show_penalty_reason(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i = 0;
	int p = 0;
	char suspect[256];
	unsigned int penalty_score;
	struct energy_mon_data *p_penalty;
	struct rtc_time tm_local;
	static char *mif_requester_text[EXYNOS_MIF_MASTER_MAX - 1] = {
		"CP", "AUD", "GNSS", "VTS", "WLBT", "CHUB", "AP"
	};

	p_penalty = &energy_mon.data[energy_mon.penalty.last_score_index % ENERGY_MON_HISTORY_NUM];
	penalty_score = p_penalty->penalty_score;
	rtc_time_to_tm(p_penalty->ts_real.tv_sec + alarm_get_tz(), &tm_local);

	memset(suspect, 0, sizeof(suspect));
	if (CHECK_SCORE_BIT(penalty_score, FAIL_TO_SUSPEND)) {
		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_SUSPEND))
			p += scnprintf(suspect + p, sizeof(suspect) - p, "%s/", p_penalty->ws_wu[0].name);

		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_SLAVE_WAKELOCK)) {
			if (p_penalty->slwl[0].usid >= 5000 &&
				p_penalty->slwl[0].usid != p_penalty->slwl[0].special_usid) {
				char *ctx = NULL;
				u32 n;
				int err;

				err = security_secid_to_secctx(p_penalty->slwl[0].usid - 5001, &ctx, &n);
				if (!err) {
					p += scnprintf(suspect + p, sizeof(suspect) - p, "%s/", ctx);
					security_release_secctx(ctx, n);
				}
			} else
				p += scnprintf(suspect + p, sizeof(suspect) - p, "%s/", p_penalty->slwl[0].slwl_name);
		}

		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_WS))
			p += scnprintf(suspect + p, sizeof(suspect) - p, "%s", p_penalty->ws[0].name);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%04d-%02d-%02d %02d:%02d:%02d: system fails to suspend due to (%s)\n",
				tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
				tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec,
				suspect);
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_SOC_LPM)) {
		if (p_penalty->lpm_blocker[0].name != NULL) {
			p += scnprintf(suspect + p, sizeof(suspect) - p, "%s/",
					p_penalty->lpm_blocker[0].name);

			if (p_penalty->lpm_blocker[1].name != NULL)
				p += scnprintf(suspect + p, sizeof(suspect) - p, "%s",
						p_penalty->lpm_blocker[1].name);

			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"%04d-%02d-%02d %02d:%02d:%02d: system fails to lpm due to (%s)\n",
					tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
					tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec,
					suspect);
		} else
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"%04d-%02d-%02d %02d:%02d:%02d: system fails to lpm\n",
					tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
					tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_SOC_SLEEP)) {
		if (p_penalty->sleep_stat.soc.exynos.acpm_sleep_mif_down == 0 ||
				p_penalty->sleep_stat.soc.exynos.acpm_sleep_soc_down == 0) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"%04d-%02d-%02d %02d:%02d:%02d: system fails to sleep with mif off\n",
					tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
					tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
		} else if (p_penalty->sleep_stat.soc.exynos.acpm_sicd_mif_down == 0 ||
					p_penalty->sleep_stat.soc.exynos.acpm_sicd_soc_down == 0) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"%04d-%02d-%02d %02d:%02d:%02d: system fails to sicd with mif off\n",
					tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
					tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
		} else {
			for (i = 0; i < (EXYNOS_MIF_MASTER_MAX - 2); i++) {
				if (i == 2) {
					if (p_penalty->sleep_stat.soc.exynos.apm[i].total_time > 360 * 1000 &&
						p_penalty->sleep_stat.soc.exynos.apm[i].total_time < 3700 * 1000)
						break;
				} else {
					if (p_penalty->sleep_stat.soc.exynos.apm[i].total_time > 1800 * 1000 &&
						p_penalty->sleep_stat.soc.exynos.apm[i].total_time < 3700 * 1000)
						break;
				}
			}
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"%04d-%02d-%02d %02d:%02d:%02d: battery drain happend while sleep due to mif on by (%s)\n",
					tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
					tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec,
					mif_requester_text[i]);
		}
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_GPS)) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%04d-%02d-%02d %02d:%02d:%02d: battery drain happend due to high gps usage\n",
				tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
				tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_LBS)) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%04d-%02d-%02d %02d:%02d:%02d: battery drain happend due to high gps usage by (%s)\n",
				tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
				tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec,
				suspect);
	} else if (CHECK_SCORE_BIT(penalty_score, FREQ_WAKEUP_FROM_SUSPEND)) {
		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_SH))
			p += scnprintf(suspect + p, sizeof(suspect) - p, "sensor hub/");

		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_SH_WRISTUP))
			p += scnprintf(suspect + p, sizeof(suspect) - p, "wrist up/");

		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_ALARM))
			p += scnprintf(suspect + p, sizeof(suspect) - p, "kernel/");

		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_ALARM_MGR)) {
			for (i = 0; i < ALARM_STAT_ARRAY_SIZE; i++) {
				if (!energy_monitor_is_alarm_whitelist(p_penalty->alarm_stat[i].comm) &&
						p_penalty->alarm_stat[i].wakeup_count > energy_mon.penalty.threshold_alarm_mgr)
					p += scnprintf(suspect + p, sizeof(suspect) - p, "%s/", p_penalty->alarm_stat[i].comm);
			}
		}

		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_INPUT))
			p += scnprintf(suspect + p , sizeof(suspect) - p, "input devices");

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%04d-%02d-%02d %02d:%02d:%02d: system is frequently awakened from sleep state by (%s)\n",
				tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
				tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec, suspect);
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_CPU_TIME)) {
		if (energy_monitor_is_sid_blacklist(p_penalty->sid_cpu[0].usid)) {
			p += scnprintf(suspect + p, sizeof(suspect) - p, "%d/%s/",
					p_penalty->sid_cpu[0].usid, p_penalty->pid_cpu[0].comm);
		} else {
			if (p_penalty->sid_cpu[0].usid >= 5000) {
				char *ctx = NULL;
				unsigned n;

				if (security_secid_to_secctx(p_penalty->sid_cpu[0].sid, &ctx, &n) == 0) {
					p += scnprintf(suspect + p, sizeof(suspect) - p, "%s\n", ctx);
					security_release_secctx(ctx, n);
				}
			}
		}
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%04d-%02d-%02d %02d:%02d:%02d: battery drain happend due to high cpu usage by (%s)\n",
				tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
				tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec, suspect);
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_CPU)) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%04d-%02d-%02d %02d:%02d:%02d: battery drain happend due to high cpu usage\n",
				tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
				tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_GPU)) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%04d-%02d-%02d %02d:%02d:%02d: battery drain happend due to gpu usage\n",
				tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
				tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
	} else if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_FAST_DRAIN)) {
		if (CHECK_SCORE_BIT(penalty_score, ENERGY_MON_SCORE_CP)) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"%04d-%02d-%02d %02d:%02d:%02d: battery drain happend due to high cp usage\n",
					tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
					tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
		} else {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"%04d-%02d-%02d %02d:%02d:%02d: battery drains over threshold\n",
					tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
					tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec);
		}
	}

	return ret;
}

static DEVICE_ATTR(penalty_reason, 0400,
	energy_monitor_show_penalty_reason,
	NULL);

static ssize_t energy_monitor_show_penalty_current(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct energy_mon_data *p_prev;
	struct energy_mon_data *p_penalty;
	__kernel_time_t diff_boot = 0;
	long long avg_curr;

	if (energy_mon.penalty.last_score_index == 0)
		p_prev = &energy_mon.boot;
	else
		p_prev = &energy_mon.data[(energy_mon.penalty.last_score_index - 1) % ENERGY_MON_HISTORY_NUM];
	p_penalty = &energy_mon.data[energy_mon.penalty.last_score_index % ENERGY_MON_HISTORY_NUM];

	if (p_penalty->penalty_score) {
		diff_boot = p_penalty->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;

		/* Return error if diff_time is negative or zero */
		if (diff_boot <= 0)
			return -ERANGE;

		avg_curr = abs(energy_mon.unit_bat_capacity *
					(p_penalty->bat_capacity - p_prev->bat_capacity));
			do_div(avg_curr, diff_boot);

		ret = scnprintf(buf, PAGE_SIZE, "%lld\n", avg_curr);
	}

	return ret;
}

static DEVICE_ATTR(penalty_current, 0400,
	energy_monitor_show_penalty_current,
	NULL);

static ssize_t energy_monitor_show_batr(
	struct device *dev, struct device_attribute *attr,
	char *buf)
{
	ssize_t ret = 0;
	int i;
	struct energy_mon_data *p_curr;
	struct energy_mon_data *p_chg ;
	struct energy_mon_data *p_dischg;
	struct energy_mon_data *p_notchg;
	struct energy_mon_data *p_full;
	unsigned int scan_req = 0;
	struct timespec scan_time = {0,};
	struct sensorhub_wo_stat sh_wo;
	int diff_wakeup[ENERGY_MON_WAKEUP_MAX];
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	s64 total_time_ms;
	s64 wakeup_time_ms[ENERGY_MON_WAKEUP_MAX];
	ktime_t total_time = ktime_set(0,0);
#endif
	s64 wakeup_time_pct[ENERGY_MON_WAKEUP_MAX] = {0,};
	struct input_stat_emon *input_stat;
	struct sap_stat_traffic sap_traffic[SAP_TRAFFIC_ARRAY_SIZE];
#ifdef CONFIG_USID_STAT
	struct usid_stat_traffic tcp_traffic[TCP_TRAFFIC_ARRAY_SIZE];
#endif
	struct net_stat_tizen_emon net_stat[NET_STAT_ARRAY_SIZE];
	int mo_tx_byte = 0, mo_tx_pkt = 0, mo_rx_byte = 0, mo_rx_pkt = 0;
	int wf_tx_byte = 0, wf_tx_pkt = 0, wf_rx_byte = 0, wf_rx_pkt = 0;
	int moup = 0, modn = 0, motm = 0;
	int	wfup = 0, wfdn = 0, wftm = 0;
	int btup = 0, btdn = 0, bttm = 0;
	struct timespec up_time;
	struct emon_wakeup_source ws[WS_ARRAY_SIZE];
	static char usid_buf[3][256];
	struct sid_sys_stats sid_cpu[SID_CPUTIME_ARRAY_SIZE];
	struct emon_cpuidle_stat *cpuidle_stat;
	struct ff_stat_emon ff_stat;
	struct sleep_stat *sleep_stat;
	unsigned int mif_total_time[EXYNOS_MIF_MASTER_MAX] = {0,};
	struct sensorhub_gps_stat *sh_gps;
	struct lbs_stat_emon *lbs_stat;

	input_stat = kzalloc(sizeof(*input_stat), GFP_KERNEL);
	if (!input_stat)
		return -ENOMEM;

	cpuidle_stat = kzalloc(sizeof(*cpuidle_stat), GFP_KERNEL);
	if (!cpuidle_stat) {
		kfree(input_stat);
		return -ENOMEM;
	}

	sleep_stat = kzalloc(sizeof(*sleep_stat), GFP_KERNEL);
	if (!sleep_stat) {
		kfree(input_stat);
		kfree(cpuidle_stat);
		return -ENOMEM;
	}

	sh_gps = kzalloc(sizeof(*sh_gps), GFP_KERNEL);
	if (!sh_gps) {
		kfree(input_stat);
		kfree(cpuidle_stat);
		kfree(sleep_stat);
		return -ENOMEM;
	}

	lbs_stat = kzalloc(sizeof(*lbs_stat), GFP_KERNEL);
	if (!lbs_stat) {
		kfree(input_stat);
		kfree(cpuidle_stat);
		kfree(sleep_stat);
		kfree(sh_gps);
		return -ENOMEM;
	}

	energy_monitor_marker(ENERGY_MON_TYPE_DUMP);
	energy_monitor_inject_data(ENERGY_MON_TYPE_DUMP, NULL, NULL);

	p_curr = &energy_mon.dump;
	p_chg = &energy_mon.charging_dump;
	p_dischg = &energy_mon.discharging_dump;
	p_notchg = &energy_mon.not_charging_dump;
	p_full = &energy_mon.full_dump;

	for (i = 0; i < NET_STAT_ARRAY_SIZE; i++) {
		if (energy_mon.discharging_dump.net_stat[i].scan_req) {
			scan_req = energy_mon.discharging_dump.net_stat[i].scan_req;
			scan_time = ktime_to_timespec(
				energy_mon.discharging_dump.net_stat[i].scan_time);
		}
	}

	sensorhub_stat_get_wo_info(&sh_wo);

	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
		diff_wakeup[i] = p_dischg->wakeup_cause[i];

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++) {
		total_time = ktime_add(total_time, p_dischg->wakeup_time[i]);
		wakeup_time_ms[i] = ktime_to_ms(p_dischg->wakeup_time[i]);
	}
	total_time_ms = ktime_to_ms(total_time);

	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
		wakeup_time_pct[i] = wakeup_time_ms[i] * 100 / total_time_ms;
#endif

	input_stat_get_stat(input_stat);

	sap_stat_get_traffic_batr(&sap_traffic[0], SAP_TRAFFIC_ARRAY_SIZE);
#ifdef CONFIG_USID_STAT
	usid_stat_get_traffic_batr(&tcp_traffic[0], TCP_TRAFFIC_ARRAY_SIZE);
#endif
	net_stat_tizen_get_stat(&net_stat[0], NET_STAT_ARRAY_SIZE);

	if (energy_mon.dqa.net_stat_cp1_idx >= 0) {
		mo_tx_byte += net_stat[energy_mon.dqa.net_stat_cp1_idx].tx_bytes;
		mo_tx_pkt += net_stat[energy_mon.dqa.net_stat_cp1_idx].tx_packets;
		mo_rx_byte += net_stat[energy_mon.dqa.net_stat_cp1_idx].rx_bytes;
		mo_rx_pkt += net_stat[energy_mon.dqa.net_stat_cp1_idx].rx_packets;

		moup = net_stat[energy_mon.dqa.net_stat_cp1_idx].up;
		modn = net_stat[energy_mon.dqa.net_stat_cp1_idx].down;
		up_time = ktime_to_timespec(net_stat[energy_mon.dqa.net_stat_cp1_idx].up_time);
		motm = (int)up_time.tv_sec;
	}

	if (energy_mon.dqa.net_stat_cp2_idx >= 0) {
		mo_tx_byte += net_stat[energy_mon.dqa.net_stat_cp2_idx].tx_bytes;
		mo_tx_pkt += net_stat[energy_mon.dqa.net_stat_cp2_idx].tx_packets;
		mo_rx_byte += net_stat[energy_mon.dqa.net_stat_cp2_idx].rx_bytes;
		mo_rx_pkt += net_stat[energy_mon.dqa.net_stat_cp2_idx].rx_packets;

		up_time = ktime_to_timespec(net_stat[energy_mon.dqa.net_stat_cp2_idx].up_time);
		if (up_time.tv_sec > motm) {
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: motm: %d %lu\n", __func__, motm, up_time.tv_sec);
			motm = (int)up_time.tv_sec;
		}
	}

	if (energy_mon.dqa.net_stat_wifi_idx >= 0) {
		mo_tx_byte += net_stat[energy_mon.dqa.net_stat_wifi_idx].tx_bytes;
		mo_tx_pkt += net_stat[energy_mon.dqa.net_stat_wifi_idx].tx_packets;
		mo_rx_byte += net_stat[energy_mon.dqa.net_stat_wifi_idx].rx_bytes;
		mo_rx_pkt += net_stat[energy_mon.dqa.net_stat_wifi_idx].rx_packets;

		wfup = net_stat[energy_mon.dqa.net_stat_wifi_idx].up;
		wfdn = net_stat[energy_mon.dqa.net_stat_wifi_idx].down;
		up_time = ktime_to_timespec(net_stat[energy_mon.dqa.net_stat_wifi_idx].up_time);
		wftm = (int)up_time.tv_sec;
	}

	if (energy_mon.dqa.net_stat_bt_idx >= 0) {
		btup = net_stat[energy_mon.dqa.net_stat_bt_idx].up;
		btdn = net_stat[energy_mon.dqa.net_stat_bt_idx].down;
		up_time = ktime_to_timespec(net_stat[energy_mon.dqa.net_stat_bt_idx].up_time);
		bttm = (int)up_time.tv_sec;
	}

	pm_get_large_wakeup_sources_batr(&ws[0], WS_ARRAY_SIZE);

	get_sid_cputime(&sid_cpu[0], SID_CPUTIME_ARRAY_SIZE);
	memset(usid_buf, 0, sizeof(usid_buf));
	for (i = 0; i < 3; i++) {
		if (sid_cpu[i].usid < 5000) {
			snprintf(&usid_buf[i][0], 256, "%d", sid_cpu[i].usid);
		} else {
			char *ctx = NULL;
			unsigned int n;
			int err;

			err = security_secid_to_secctx(sid_cpu[i].sid, &ctx, &n);
			if (err)
				snprintf(&usid_buf[i][0], 256, "%d", sid_cpu[i].usid);
			else {
				snprintf(&usid_buf[i][0], 256, "%s", ctx);
				security_release_secctx(ctx, n);
			}
		}
	}

	cpuidle_stats_get_stats(cpuidle_stat);

	ff_stat_tizen_get_stat(&ff_stat);

	sleep_stat_get_stat(sleep_stat);
	for (i = 0; i < EXYNOS_MIF_MASTER_MAX; i++) {
		mif_total_time[i] = sleep_stat->soc.exynos.apm[i].total_time / 1000U;
	}

	sensorhub_stat_get_gps(sh_gps);
	lbs_stat_get_stat(lbs_stat);

	ret += scnprintf(buf, PAGE_SIZE,
			"{\"TIME\":%lu,"
			"\"BOOT_TM\":%lu,"
			"\"RUN_TM\":%lu,\"SCRON_TM\":%lu,"
			"\"SCROFF_TM\":%lu,\"BATDISC\":%d,"
			"\"SCROFF_UPTM\":%lu,\"AOD_TM\":%lu,"
			"\"CHG_TM\":%lu,\"BATCHG\":%d,"
			"\"NOTCHG_TM\":%lu,\"BATNOTCHG\":%d,"
			"\"FULL_TM\":%lu,\"BATFULL\":%d,"
			"\"WFSCN_TM\":%lu,\"WFSCN_CNT\":%u,"
			"\"WO_TM\":%llu,"
			"\"CUR_SLP\":%d,"
			"\"INP_WU\":%d,\"SSP_WU\":%d,\"RTC_WU\":%d,"
			"\"BT_WU\":%d,\"WIF_WU\":%d,\"CP_WU\":%d,"
			"\"GNS_WU\":%d,\"NFC_WU\":%d,\"ETC_WU\":%d,"
			"\"INP_PCT\":%d,\"SSP_PCT\":%d,\"RTC_PCT\":%d,"
			"\"BT_PCT\":%d,\"WIF_PCT\":%d,\"CP_PCT\":%d,"
			"\"GNS_PCT\":%d,\"NFC_PCT\":%d,\"ETC_PCT\":%d,"
			"\"WU_PCT\":%d,\"NT_PCT\":%d,"
			"\"INPEV_WU00\":%d,\"INPEV_WU10\":%d,"
			"\"INPEV_WU20\":%d,\"INPEV_WU30\":%d,"
			"\"INPEV_WU40\":%d,\"INPEV_WU4X\":%d,"
			"\"SAP1_NAME\":\"%s\",\"SAP1_BYTE\":%d,\"SAP1_PKT\":%d,"
			"\"SAP2_NAME\":\"%s\",\"SAP2_BYTE\":%d,\"SAP2_PKT\":%d,"
			"\"SAP3_NAME\":\"%s\",\"SAP3_BYTE\":%d,\"SAP3_PKT\":%d,"
#ifdef CONFIG_USID_STAT
			"\"TCP1_NAME\":\"%s\",\"TCP1_BYTE\":%d,\"TCP1_PKT\":%d,"
			"\"TCP2_NAME\":\"%s\",\"TCP2_BYTE\":%d,\"TCP2_PKT\":%d,"
			"\"TCP3_NAME\":\"%s\",\"TCP3_BYTE\":%d,\"TCP3_PKT\":%d,"
#endif
			"\"MO_TX_BYTE\":%d,\"MO_TX_PKT\":%d,"
			"\"MO_RX_BYTE\":%d,\"MO_RX_PKT\":%d,"
			"\"WF_TX_BYTE\":%d,\"WF_TX_PKT\":%d,"
			"\"WF_RX_BYTE\":%d,\"WF_RX_PKT\":%d,"
			"\"MOUP\":%d,\"MODN\":%d,\"MOTM\":%d,"
			"\"WFUP\":%d,\"WFDN\":%d,\"WFTM\":%d,"
			"\"BTUP\":%d,\"BTDN\":%d,\"BTTM\":%d,"
			"\"WS1_NAME\":\"%s\",\"WS1_TM\":%d,"
			"\"WS2_NAME\":\"%s\",\"WS2_TM\":%d,"
			"\"WS3_NAME\":\"%s\",\"WS3_TM\":%d,"
			"\"CPU1_NAME\":\"%s\",\"CPU1_TM\":%d,\"CPU1_PM\":%d,"
			"\"CPU2_NAME\":\"%s\",\"CPU2_TM\":%d,\"CPU2_PM\":%d,"
			"\"CPU3_NAME\":\"%s\",\"CPU3_TM\":%d,\"CPU3_PM\":%d,"
			"\"CPU_ACT\":%d,"
			"\"CPU_IDL\":%d,"
			"\"CPU_LPM\":%d,"
			"\"FF_TM\":%d,\"FF_CNT\":%d,"
			"\"SUP_FAIL\":%d,"
			"\"SUP_FFRZ\":%d,"
			"\"SUP_FPRP\":%d,"
			"\"SUP_FSUP\":%d,"
			"\"SUP_FSUL\":%d,"
			"\"SUP_FSUN\":%d,"
			"\"SUP_SUCC\":%d,"
			"\"MIF_CP\":%d,\"MIF_AUD\":%d,\"MIF_GNSS\":%d,\"MIF_VTS\":%d,\"MIF_WLBT\":%d,\"MIF_CHUB\":%d,\"MIF_AP\":%d,"
			"\"GPS1_RUN\":%d,\"GPS2_RUN\":%d"
			"}\n",
			p_curr->ts_real.tv_sec + alarm_get_tz(),
			p_chg->ts_boot.tv_sec + p_dischg->ts_boot.tv_sec + p_notchg->ts_boot.tv_sec + p_full->ts_boot.tv_sec,
			p_dischg->ts_boot.tv_sec, p_dischg->ts_disp.tv_sec,
			p_dischg->ts_boot.tv_sec - p_dischg->ts_disp.tv_sec, p_dischg->bat_capacity,
			p_dischg->ts_kern.tv_sec - p_dischg->ts_disp.tv_sec, p_dischg->ts_aod.tv_sec,
			p_chg->ts_boot.tv_sec, p_chg->bat_capacity,
			p_notchg->ts_boot.tv_sec, p_notchg->bat_capacity,
			p_full->ts_boot.tv_sec, p_full->bat_capacity,
			scan_time.tv_sec, scan_req,
			ktime_to_ms(sh_wo.wear_on_time),
			p_dischg->current_suspend,
			diff_wakeup[0], diff_wakeup[1], diff_wakeup[2],
			diff_wakeup[3], diff_wakeup[4], diff_wakeup[5],
			diff_wakeup[6], diff_wakeup[7], diff_wakeup[8],
			(int)wakeup_time_pct[0], (int)wakeup_time_pct[1], (int)wakeup_time_pct[2],
			(int)wakeup_time_pct[3], (int)wakeup_time_pct[4], (int)wakeup_time_pct[5],
			(int)wakeup_time_pct[6], (int)wakeup_time_pct[7], (int)wakeup_time_pct[8],
			(int)wakeup_time_pct[9], (int)wakeup_time_pct[10],
			input_stat->ev_wakeup[EV_WU_00], input_stat->ev_wakeup[EV_WU_10],
			input_stat->ev_wakeup[EV_WU_20], input_stat->ev_wakeup[EV_WU_30],
			input_stat->ev_wakeup[EV_WU_40], input_stat->ev_wakeup[EV_WU_GT_40],
			sap_traffic[0].aspid.id,
			sap_traffic[0].snd + sap_traffic[0].rcv,
			sap_traffic[0].snd_count + sap_traffic[0].rcv_count,
			sap_traffic[1].aspid.id,
			sap_traffic[1].snd + sap_traffic[1].rcv,
			sap_traffic[1].snd_count + sap_traffic[1].rcv_count,
			sap_traffic[2].aspid.id,
			sap_traffic[2].snd + sap_traffic[2].rcv,
			sap_traffic[2].snd_count + sap_traffic[2].rcv_count,
#ifdef CONFIG_USID_STAT
			tcp_traffic[0].comm,
			tcp_traffic[0].snd + tcp_traffic[0].rcv,
			tcp_traffic[0].snd_count + tcp_traffic[0].rcv_count,
			tcp_traffic[1].comm,
			tcp_traffic[1].snd + tcp_traffic[1].rcv,
			tcp_traffic[1].snd_count + tcp_traffic[1].rcv_count,
			tcp_traffic[2].comm,
			tcp_traffic[2].snd + tcp_traffic[2].rcv,
			tcp_traffic[2].snd_count + tcp_traffic[2].rcv_count,
#endif
			mo_tx_byte, mo_tx_pkt,
			mo_rx_byte, mo_rx_pkt,
			wf_tx_byte,	wf_tx_pkt,
			wf_rx_byte,	wf_rx_pkt,
			moup, modn, motm,
			wfup, wfdn, wftm,
			btup, btdn, bttm,
			ws[0].name, (int)ktime_to_timeval(ws[0].emon_total_time).tv_sec,
			ws[1].name, (int)ktime_to_timeval(ws[1].emon_total_time).tv_sec,
			ws[2].name, (int)ktime_to_timeval(ws[2].emon_total_time).tv_sec,
			&usid_buf[0][0],
			(int)jiffies_to_msecs(cputime_to_jiffies(sid_cpu[0].ttime)),
			sid_cpu[0].permil,
			&usid_buf[1][0],
			(int)jiffies_to_msecs(cputime_to_jiffies(sid_cpu[1].ttime)),
			sid_cpu[1].permil,
			&usid_buf[2][0],
			(int)jiffies_to_msecs(cputime_to_jiffies(sid_cpu[2].ttime)),
			sid_cpu[2].permil,
			(int)ktime_to_ms(cpuidle_stat->total_stat_time),
			(int)ktime_to_ms(cpuidle_stat->cpuidle[0].total_idle_time),
			(int)ktime_to_ms(cpuidle_stat->lpm.usage.time),
			(int)ktime_to_ms(ff_stat.total_time), (int)ff_stat.play_count,
			sleep_stat->fail,
			sleep_stat->failed_freeze,
			sleep_stat->failed_prepare,
			sleep_stat->failed_suspend,
			sleep_stat->failed_suspend_late,
			sleep_stat->failed_suspend_noirq,
			sleep_stat->suspend_success,
			mif_total_time[0], mif_total_time[1], mif_total_time[2], mif_total_time[3], mif_total_time[4], mif_total_time[5], mif_total_time[6],
			sh_gps->gps_time, (int)ktime_to_ms(lbs_stat->total_gps_time)
			);

	kfree(input_stat);
	kfree(cpuidle_stat);
	kfree(sleep_stat);
	kfree(sh_gps);
	kfree(lbs_stat);

	return ret;
}

static DEVICE_ATTR(batr, 0400,
	energy_monitor_show_batr,
	NULL);

static ssize_t energy_monitor_show_bafd(
	struct device *dev, struct device_attribute *attr,
	char *buf)
{
	ssize_t ret = 0;
	struct energy_mon_data *p_curr = NULL;
	struct energy_mon_data *p_prev = NULL;
	long long avg_curr;
	int i;
	int prev_data_index, curr_data_index;
	int diff_soc;
	__kernel_time_t diff_boot = 0, diff_kern, diff_disp, diff_aod;
	s64 diff_wear_off = 0;
	int diff_wakeup[ENERGY_MON_WAKEUP_MAX];
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	s64 total_time_ms;
	s64 wakeup_time_ms[ENERGY_MON_WAKEUP_MAX];
	ktime_t total_time = ktime_set(0,0);
#endif
	s64 wakeup_time_pct[ENERGY_MON_WAKEUP_MAX] = {0,};
	int mo_tx_byte = 0, mo_tx_pkt = 0, mo_rx_byte = 0, mo_rx_pkt = 0;
	int wf_tx_byte = 0, wf_tx_pkt = 0, wf_rx_byte = 0, wf_rx_pkt = 0;
	int moup = 0, modn = 0, motm = 0;
	int	wfup = 0, wfdn = 0, wftm = 0;
	int btup = 0, btdn = 0, bttm = 0;
	struct timespec up_time;
	char usid_buf[3][256];
	int gpu_freq[7] = {0,};
	unsigned int mif_total_time[EXYNOS_MIF_MASTER_MAX] = {0,};
	ktime_t gps_time;
	s64 gps_time_ms;

	if (energy_mon.data_index < 2)
		return ret;

	prev_data_index = energy_mon.data_index - 2;
	curr_data_index = energy_mon.data_index - 1;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %d %d\n", __func__,
		prev_data_index, curr_data_index);

	p_prev = &energy_mon.data[prev_data_index % ENERGY_MON_HISTORY_NUM];
	p_curr = &energy_mon.data[curr_data_index % ENERGY_MON_HISTORY_NUM];

	diff_soc = p_curr->bat_capacity - p_prev->bat_capacity;
	diff_boot = p_curr->ts_boot.tv_sec - p_prev->ts_boot.tv_sec;
	diff_kern = p_curr->ts_kern.tv_sec - p_prev->ts_kern.tv_sec;
	diff_disp = p_curr->ts_disp.tv_sec - p_prev->ts_disp.tv_sec;
	diff_aod = p_curr->ts_aod.tv_sec - p_prev->ts_aod.tv_sec;

	/* If diff_time is negative, change to zero */
	if (diff_boot < 0)
		diff_boot = 0;
	if (diff_kern < 0)
		diff_kern = 0;
	else if (diff_kern > diff_boot)
		diff_kern = diff_boot;
	if (diff_disp < 0)
		diff_disp = 0;
	else if (diff_disp > diff_boot)
		diff_disp = diff_boot;
	if (diff_aod < 0)
		diff_aod = 0;
	else if (diff_aod > diff_boot)
		diff_aod = diff_boot;
	diff_wear_off = ktime_ms_delta(p_curr->sh_wo.wear_on_time, p_prev->sh_wo.wear_on_time)/1000;

	avg_curr = abs(energy_mon.unit_bat_capacity *
				(p_curr->bat_capacity - p_prev->bat_capacity));
	/* To prevent Device by Zero */
	if (diff_boot)
		do_div(avg_curr, diff_boot);

	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
		diff_wakeup[i] = p_curr->wakeup_cause[i] - p_prev->wakeup_cause[i];

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++) {
		total_time = ktime_add(total_time, p_curr->wakeup_time[i]);
		wakeup_time_ms[i] = ktime_to_ms(p_curr->wakeup_time[i]);
	}
	total_time_ms = ktime_to_ms(total_time);

	for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
		wakeup_time_pct[i] = wakeup_time_ms[i] * 100 / total_time_ms;
#endif

	if (energy_mon.dqa.net_stat_cp1_idx >= 0) {
		mo_tx_byte += p_curr->net_stat[energy_mon.dqa.net_stat_cp1_idx].tx_bytes;
		mo_tx_pkt += p_curr->net_stat[energy_mon.dqa.net_stat_cp1_idx].tx_packets;
		mo_rx_byte += p_curr->net_stat[energy_mon.dqa.net_stat_cp1_idx].rx_bytes;
		mo_rx_pkt += p_curr->net_stat[energy_mon.dqa.net_stat_cp1_idx].rx_packets;

		moup = p_curr->net_stat[energy_mon.dqa.net_stat_cp1_idx].up;
		modn = p_curr->net_stat[energy_mon.dqa.net_stat_cp1_idx].down;
		up_time = ktime_to_timespec(
			p_curr->net_stat[energy_mon.dqa.net_stat_cp1_idx].up_time);
		motm = (int)up_time.tv_sec;
	}

	if (energy_mon.dqa.net_stat_cp2_idx >= 0) {
		mo_tx_byte += p_curr->net_stat[energy_mon.dqa.net_stat_cp2_idx].tx_bytes;
		mo_tx_pkt += p_curr->net_stat[energy_mon.dqa.net_stat_cp2_idx].tx_packets;
		mo_rx_byte += p_curr->net_stat[energy_mon.dqa.net_stat_cp2_idx].rx_bytes;
		mo_rx_pkt += p_curr->net_stat[energy_mon.dqa.net_stat_cp2_idx].rx_packets;

		up_time = ktime_to_timespec(
			p_curr->net_stat[energy_mon.dqa.net_stat_cp2_idx].up_time);
		if (up_time.tv_sec > motm) {
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: motm: %d %lu\n", __func__, motm, up_time.tv_sec);
			motm = (int)up_time.tv_sec;
		}
	}

	if (energy_mon.dqa.net_stat_wifi_idx >= 0) {
		mo_tx_byte += p_curr->net_stat[energy_mon.dqa.net_stat_wifi_idx].tx_bytes;
		mo_tx_pkt += p_curr->net_stat[energy_mon.dqa.net_stat_wifi_idx].tx_packets;
		mo_rx_byte += p_curr->net_stat[energy_mon.dqa.net_stat_wifi_idx].rx_bytes;
		mo_rx_pkt += p_curr->net_stat[energy_mon.dqa.net_stat_wifi_idx].rx_packets;

		wfup = p_curr->net_stat[energy_mon.dqa.net_stat_wifi_idx].up;
		wfdn = p_curr->net_stat[energy_mon.dqa.net_stat_wifi_idx].down;
		up_time = ktime_to_timespec(
			p_curr->net_stat[energy_mon.dqa.net_stat_wifi_idx].up_time);
		wftm = (int)up_time.tv_sec;
	}

	if (energy_mon.dqa.net_stat_bt_idx >= 0) {
		btup = p_curr->net_stat[energy_mon.dqa.net_stat_bt_idx].up;
		btdn = p_curr->net_stat[energy_mon.dqa.net_stat_bt_idx].down;
		up_time = ktime_to_timespec(
			p_curr->net_stat[energy_mon.dqa.net_stat_bt_idx].up_time);
		bttm = (int)up_time.tv_sec;
	}

	memset(usid_buf, 0, sizeof(usid_buf));
	for (i = 0; i < 3; i++) {
		if (p_curr->sid_cpu[i].usid < 5000) {
			snprintf(&usid_buf[i][0], 256, "%d", p_curr->sid_cpu[i].usid);
		} else {
			char *ctx = NULL;
			unsigned int n;
			int err;

			err = security_secid_to_secctx( p_curr->sid_cpu[i].sid, &ctx, &n);
			if (err)
				snprintf(&usid_buf[i][0], 256, "%d", p_curr->sid_cpu[i].usid);
			else {
				snprintf(&usid_buf[i][0], 256, "%s", ctx);
				security_release_secctx(ctx, n);
			}
		}
	}

	for (i = 0; i < 7 ; i++) {
		if (i >= p_curr->gpufreq_stat.table_size)
			break;

		gpu_freq[i] = (int)jiffies_to_msecs(p_curr->gpufreq_stat.table[i].time);
	}

	for (i = 0; i < EXYNOS_MIF_MASTER_MAX; i++) {
		mif_total_time[i] = p_curr->sleep_stat.soc.exynos.apm[i].total_time / 1000U;
	}

	for (i = 0; i < LBS_STAT_ARRAY_SIZE; i++) {
		gps_time = ktime_add(p_curr->lbs_stat[i].usage[LBS_METHOD_GPS].total_time,
			p_curr->lbs_stat[i].usage[LBS_METHOD_BATCH_GPS].total_time);
		gps_time_ms = ktime_to_ms(gps_time);
		if (gps_time_ms > 3600000)
			gps_time_ms = 3600000;
	}

	ret += scnprintf(buf, PAGE_SIZE,
			"{\"TIME\":%lu,"
			"\"BOOT_TM\":%lu,"
			"\"RUN_TM\":%lu,"
			"\"UP_TM\":%lu,"
			"\"SCRON_TM\":%lu,"
			"\"AOD_TM\":%lu,"
			"\"WO_TM\":%llu,"
			"\"TYPE\":%d,"
			"\"CUR_AVG\":%lld,"
			"\"CUR_SLP\":%d,"
			"\"INP_WU\":%d,\"SSP_WU\":%d,\"RTC_WU\":%d,"
			"\"BT_WU\":%d,\"WIF_WU\":%d,\"CP_WU\":%d,"
			"\"GNS_WU\":%d,\"NFC_WU\":%d,\"ETC_WU\":%d,"
			"\"INP_PCT\":%d,\"SSP_PCT\":%d,\"RTC_PCT\":%d,"
			"\"BT_PCT\":%d,\"WIF_PCT\":%d,\"CP_PCT\":%d,"
			"\"GNS_PCT\":%d,\"NFC_PCT\":%d,\"ETC_PCT\":%d,"
			"\"WU_PCT\":%d,\"NT_PCT\":%d,"
			"\"INPEV_WU00\":%d,\"INPEV_WU10\":%d,"
			"\"INPEV_WU20\":%d,\"INPEV_WU30\":%d,"
			"\"INPEV_WU40\":%d,\"INPEV_WU4X\":%d,"
			"\"SAP1_NAME\":\"%s\",\"SAP1_BYTE\":%d,\"SAP1_PKT\":%d,"
			"\"SAP2_NAME\":\"%s\",\"SAP2_BYTE\":%d,\"SAP2_PKT\":%d,"
			"\"SAP3_NAME\":\"%s\",\"SAP3_BYTE\":%d,\"SAP3_PKT\":%d,"
#ifdef CONFIG_USID_STAT
			"\"TCP1_NAME\":\"%s\",\"TCP1_BYTE\":%d,\"TCP1_PKT\":%d,"
			"\"TCP2_NAME\":\"%s\",\"TCP2_BYTE\":%d,\"TCP2_PKT\":%d,"
			"\"TCP3_NAME\":\"%s\",\"TCP3_BYTE\":%d,\"TCP3_PKT\":%d,"
#endif
			"\"MO_TX_BYTE\":%d,\"MO_TX_PKT\":%d,"
			"\"MO_RX_BYTE\":%d,\"MO_RX_PKT\":%d,"
			"\"WF_TX_BYTE\":%d,\"WF_TX_PKT\":%d,"
			"\"WF_RX_BYTE\":%d,\"WF_RX_PKT\":%d,"
			"\"MOUP\":%d,\"MODN\":%d,\"MOTM\":%d,"
			"\"WFUP\":%d,\"WFDN\":%d,\"WFTM\":%d,"
			"\"BTUP\":%d,\"BTDN\":%d,\"BTTM\":%d,"
			"\"PM1_NAME\":\"%s\",\"PM1_TM\":%d,"
			"\"PM2_NAME\":\"%s\",\"PM2_TM\":%d,"
			"\"PM3_NAME\":\"%s\",\"PM3_TM\":%d,"
			"\"WS1_NAME\":\"%s\",\"WS1_TM\":%d,"
			"\"WS2_NAME\":\"%s\",\"WS2_TM\":%d,"
			"\"WS3_NAME\":\"%s\",\"WS3_TM\":%d,"
			"\"WS1_WU_NAME\":\"%s\",\"WS1_WU_CNT\":%ld,"
			"\"WS2_WU_NAME\":\"%s\",\"WS2_WU_CNT\":%ld,"
			"\"WS3_WU_NAME\":\"%s\",\"WS3_WU_CNT\":%ld,"
			"\"CPU1_NAME\":\"%s\",\"CPU1_TM\":%d,\"CPU1_PM\":%d,"
			"\"CPU2_NAME\":\"%s\",\"CPU2_TM\":%d,\"CPU2_PM\":%d,"
			"\"CPU3_NAME\":\"%s\",\"CPU3_TM\":%d,\"CPU3_PM\":%d,"
			"\"CPU_ACT\":%d,"
			"\"CPU_IDL\":%d,"
			"\"CPU_LPM\":%d,"
			"\"GPU_FQ0\":%d,"
			"\"GPU_FQ1\":%d,"
			"\"GPU_FQ2\":%d,"
			"\"GPU_FQ3\":%d,"
			"\"GPU_FQ4\":%d,"
			"\"GPU_FQ5\":%d,"
			"\"GPU_FQ6\":%d,"
			"\"FF_TM\":%d,\"FF_CNT\":%d,"
			"\"SUP_FAIL\":%d,"
			"\"SUP_FFRZ\":%d,"
			"\"SUP_FPRP\":%d,"
			"\"SUP_FSUP\":%d,"
			"\"SUP_FSUL\":%d,"
			"\"SUP_FSUN\":%d,"
			"\"SUP_SUCC\":%d,"
			"\"MIF_CP\":%d,\"MIF_AUD\":%d,\"MIF_GNSS\":%d,\"MIF_VTS\":%d,\"MIF_WLBT\":%d,\"MIF_CHUB\":%d,\"MIF_AP\":%d,"
			"\"GPS1_RUN\":%d,\"GPS2_RUN\":%d"
			"}\n",
			p_curr->ts_real.tv_sec + alarm_get_tz(),
			p_curr->ts_boot.tv_sec,
			diff_boot, diff_kern, diff_disp, diff_aod, diff_wear_off,
			p_curr->penalty_score,
			avg_curr,
			p_curr->current_suspend,
			diff_wakeup[0], diff_wakeup[1], diff_wakeup[2],
			diff_wakeup[3], diff_wakeup[4], diff_wakeup[5],
			diff_wakeup[6], diff_wakeup[7], diff_wakeup[8],
			(int)wakeup_time_pct[0], (int)wakeup_time_pct[1], (int)wakeup_time_pct[2],
			(int)wakeup_time_pct[3], (int)wakeup_time_pct[4], (int)wakeup_time_pct[5],
			(int)wakeup_time_pct[6], (int)wakeup_time_pct[7], (int)wakeup_time_pct[8],
			(int)wakeup_time_pct[9], (int)wakeup_time_pct[10],
			p_curr->input_stat.ev_wakeup[EV_WU_00], p_curr->input_stat.ev_wakeup[EV_WU_10],
			p_curr->input_stat.ev_wakeup[EV_WU_20],	p_curr->input_stat.ev_wakeup[EV_WU_30],
			p_curr->input_stat.ev_wakeup[EV_WU_40], p_curr->input_stat.ev_wakeup[EV_WU_GT_40],
			p_curr->sap_traffic[0].aspid.id,
			p_curr->sap_traffic[0].snd + p_curr->sap_traffic[0].rcv,
			p_curr->sap_traffic[0].snd_count + p_curr->sap_traffic[0].rcv_count,
			p_curr->sap_traffic[1].aspid.id,
			p_curr->sap_traffic[1].snd + p_curr->sap_traffic[1].rcv,
			p_curr->sap_traffic[1].snd_count + p_curr->sap_traffic[1].rcv_count,
			p_curr->sap_traffic[2].aspid.id,
			p_curr->sap_traffic[2].snd + p_curr->sap_traffic[2].rcv,
			p_curr->sap_traffic[2].snd_count + p_curr->sap_traffic[2].rcv_count,
#ifdef CONFIG_USID_STAT
			p_curr->tcp_traffic[0].comm,
			p_curr->tcp_traffic[0].snd + p_curr->tcp_traffic[0].rcv,
			p_curr->tcp_traffic[0].snd_count + p_curr->tcp_traffic[0].rcv_count,
			p_curr->tcp_traffic[1].comm,
			p_curr->tcp_traffic[1].snd + p_curr->tcp_traffic[1].rcv,
			p_curr->tcp_traffic[1].snd_count + p_curr->tcp_traffic[1].rcv_count,
			p_curr->tcp_traffic[2].comm,
			p_curr->tcp_traffic[2].snd + p_curr->tcp_traffic[2].rcv,
			p_curr->tcp_traffic[2].snd_count + p_curr->tcp_traffic[2].rcv_count,
#endif
			mo_tx_byte, mo_tx_pkt,
			mo_rx_byte, mo_rx_pkt,
			wf_tx_byte,	wf_tx_pkt,
			wf_rx_byte,	wf_rx_pkt,
			moup, modn, motm,
			wfup, wfdn, wftm,
			btup, btdn, bttm,
			p_curr->slwl[0].slwl_name, (int)ktime_to_timeval(p_curr->slwl[0].prevent_time).tv_sec,
			p_curr->slwl[1].slwl_name, (int)ktime_to_timeval(p_curr->slwl[1].prevent_time).tv_sec,
			p_curr->slwl[2].slwl_name, (int)ktime_to_timeval(p_curr->slwl[2].prevent_time).tv_sec,
			p_curr->ws[0].name, (int)ktime_to_timeval(p_curr->ws[0].emon_total_time).tv_sec,
			p_curr->ws[1].name,	(int)ktime_to_timeval(p_curr->ws[1].emon_total_time).tv_sec,
			p_curr->ws[2].name,	(int)ktime_to_timeval(p_curr->ws[2].emon_total_time).tv_sec,
			p_curr->ws_wu[0].name, p_curr->ws_wu[0].emon_wakeup_count,
			p_curr->ws_wu[1].name, p_curr->ws_wu[1].emon_wakeup_count,
			p_curr->ws_wu[2].name, p_curr->ws_wu[2].emon_wakeup_count,
			&usid_buf[0][0],
			(int)jiffies_to_msecs(cputime_to_jiffies(p_curr->sid_cpu[0].ttime)),
			p_curr->sid_cpu[0].permil,
			&usid_buf[1][0],
			(int)jiffies_to_msecs(cputime_to_jiffies(p_curr->sid_cpu[1].ttime)),
			p_curr->sid_cpu[1].permil,
			&usid_buf[2][0],
			(int)jiffies_to_msecs(cputime_to_jiffies(p_curr->sid_cpu[2].ttime)),
			p_curr->sid_cpu[2].permil,
			(int)ktime_to_ms(p_curr->cpuidle_stat.total_stat_time),
			(int)ktime_to_ms(p_curr->cpuidle_stat.cpuidle[0].total_idle_time),
			(int)ktime_to_ms(p_curr->cpuidle_stat.lpm.usage.time),
			gpu_freq[0],
			gpu_freq[1],
			gpu_freq[2],
			gpu_freq[3],
			gpu_freq[4],
			gpu_freq[5],
			gpu_freq[6],
			(int)ktime_to_ms(p_curr->ff_stat.total_time), (int)p_curr->ff_stat.play_count,
			p_curr->sleep_stat.fail,
			p_curr->sleep_stat.failed_freeze,
			p_curr->sleep_stat.failed_prepare,
			p_curr->sleep_stat.failed_suspend,
			p_curr->sleep_stat.failed_suspend_late,
			p_curr->sleep_stat.failed_suspend_noirq,
			p_curr->sleep_stat.suspend_success,
			mif_total_time[0], mif_total_time[1], mif_total_time[2], mif_total_time[3], mif_total_time[4], mif_total_time[5], mif_total_time[6],
			p_curr->sh_gps.gps_time, (int)gps_time_ms
			);

	return ret;
}

static DEVICE_ATTR(bafd, 0400,
	energy_monitor_show_bafd,
	NULL);

static struct attribute *sec_energy_monitor_attrs[] = {
	&dev_attr_get_last_discharging_info.attr,
	&dev_attr_penalty_notify_mask.attr,
	&dev_attr_penalty_score.attr,
	&dev_attr_penalty_threshold.attr,
	&dev_attr_penalty_cause.attr,
	&dev_attr_penalty_reason.attr,
	&dev_attr_penalty_current.attr,
	&dev_attr_batr.attr,
	&dev_attr_bafd.attr,
	NULL
};

static const struct attribute_group sec_energy_monitor_attr_group = {
	.attrs = sec_energy_monitor_attrs,
};

void energy_monitor_penalty_score_notify(void)
{
	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s\n", __func__);
	__pm_wakeup_event(energy_mon.emon_ws, 200); /* 200 ms */
	sysfs_notify(power_kobj, NULL, "emon_penalty_score");
}
#endif

static int energy_monitor_debug_init(void)
{
	struct dentry *d;
#ifdef CONFIG_SEC_SYSFS
	int err;
#endif

	d = debugfs_create_dir("energy_monitor", NULL);
	if (d) {
		if (!debugfs_create_file("status_raw",
				0600, d, NULL, &status_raw_fops))
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s : debugfs_create_file, error\n", "status_raw");
		if (!debugfs_create_file("status_raw2",
				0600, d, NULL, &status_raw2_fops))
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s : debugfs_create_file, error\n", "status_raw2");
#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
		if (!debugfs_create_file("status_wakeup_time",
				0600, d, NULL, &status_wakeup_time_fops))
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s : debugfs_create_file, error\n", "status_wakeup_time");
#endif
#ifdef CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR
		if (!debugfs_create_file("sleep_current",
				0600, d, NULL, &sleep_current_fops))
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s : debugfs_create_file, error\n", "sleep_current");
#endif
		if (!debugfs_create_file("monitor_interval",
				0600, d, NULL, &monitor_interval_fops))
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s : debugfs_create_file, error\n", "monitor_interval");
		if (!debugfs_create_file("logging_interval",
				0600, d, NULL, &logging_interval_fops))
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s : debugfs_create_file, error\n", "monitor_interval");

		debugfs_create_u32("debug_level", 0600, d, &debug_level);
		debugfs_create_u32("enable", 0600, d, &energy_monitor_enable);
	}

#ifdef CONFIG_SEC_SYSFS
	sec_energy_monitor = sec_device_create(NULL, "energy_monitor");
	if (IS_ERR(sec_energy_monitor)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"sec_energy_monitor create fail\n");
		return -ENODEV;
	}

	err = sysfs_create_group(&sec_energy_monitor->kobj,
			&sec_energy_monitor_attr_group);
	if (err < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"sec_energy_monitor_attr create fail\n");
		return err;
	}
#endif

	return 0;
}

#if defined(CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR)
int energy_monitor_sleep_current_estimator(unsigned long event)
{
	struct power_supply *psy = NULL;
	union power_supply_propval value;
	int err = -1;

	static struct timespec ts_susp;
	static int raw_soc_susp;
	static int valid_suspend;

	struct timespec ts_resume;
	struct timespec ts_elapsed;
	static int valid_count;
	long long average;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		valid_suspend = 0;
		psy = power_supply_get_by_name("battery");
		if (!psy) {
			energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
				"%s: cannot find battery power supply\n", __func__);
			break;
		}

		err = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_STATUS, &value);
		if (err) {
			power_supply_put(psy);
			break;
		}
		power_supply_put(psy);

		/* Check only in discharging */
		if (value.intval == POWER_SUPPLY_STATUS_DISCHARGING) {
			if (energy_mon.use_raw_soc) {
				psy = power_supply_get_by_name(energy_mon.ps_raw_soc);
				if (psy) {
					value.intval = SEC_FUELGAUGE_CAPACITY_TYPE_RAW;
					err = power_supply_get_property(psy,
							POWER_SUPPLY_PROP_CAPACITY, &value);
					if (err < 0)
						value.intval = -1;
					raw_soc_susp = value.intval;

					power_supply_put(psy);
				} else {
					energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
						"%s: cannot find fg power supply\n", __func__);
					raw_soc_susp = -1;
				}
			} else {
				psy = power_supply_get_by_name("battery");
				if (psy) {
					err = power_supply_get_property(psy,
							POWER_SUPPLY_PROP_CAPACITY, &value);
					if (err < 0)
						value.intval = -1;
					raw_soc_susp = value.intval * 100;

					power_supply_put(psy);
				}
			}
			ts_susp = current_kernel_time();
			valid_suspend = 1;
		}
		break;
	case PM_POST_SUSPEND:
		if (energy_mon.use_raw_soc) {
			psy = power_supply_get_by_name(energy_mon.ps_raw_soc);
			if (psy) {
				value.intval = SEC_FUELGAUGE_CAPACITY_TYPE_RAW;
				err = power_supply_get_property(psy,
						POWER_SUPPLY_PROP_CAPACITY, &value);
				if (err < 0)
					value.intval = -1;

				power_supply_put(psy);
			}
		} else {
			psy = power_supply_get_by_name("battery");
			if (psy) {
				err = power_supply_get_property(psy,
					POWER_SUPPLY_PROP_CAPACITY, &value);
				if (err < 0)
					value.intval = -1;

				power_supply_put(psy);
			}
		}
		ts_resume = current_kernel_time();

		/* Calculate elapsed time and power consumption */
		ts_elapsed = timespec_sub(ts_resume, ts_susp);

		if (valid_suspend &&
			ts_elapsed.tv_sec > MIN_SLEEP_TIME_S &&
			raw_soc_susp - value.intval > 0) {
			average = (raw_soc_susp - value.intval) *
				energy_mon.unit_bat_capacity;

			valid_count++;
			do_div(average, ts_elapsed.tv_sec);

			/* Save only first 20 results */
			if (energy_mon.estimator_index < ENERGY_MON_MAX_SLEEP_ESTIMATOR_CNT)
				energy_mon.estimator_average[energy_mon.estimator_index++] = average;

			energy_mon_dbg(ENERGY_MON_DEBUG_SLEEP_ESTIMATOR,
				"%s: %3ld.%02ldmA from %u to %u for %lus\n",
				__func__, (long)average/100,
				(long)average%100, raw_soc_susp,
				value.intval, ts_elapsed.tv_sec);
		}

#ifdef CONFIG_HW_SUSPEND_ENERGY_ESTIMATOR
		if (ts_elapsed.tv_sec > 8) {
			psy = power_supply_get_by_name(energy_mon.ps_hw_suspend_energy);
			if (psy) {
				if (valid_suspend) {
					value.intval = POWER_SUPPLY_CURRENT_SUSPEND_DISCHARGING;
					err = power_supply_get_property(psy,
							POWER_SUPPLY_PROP_CURRENT_SUSPEND, &value);
				} else {
					value.intval = POWER_SUPPLY_CURRENT_SUSPEND_CHARGING;
					err = power_supply_get_property(psy,
							POWER_SUPPLY_PROP_CURRENT_SUSPEND, &value);
				}
				power_supply_put(psy);

				if (!err) {
					energy_mon.current_suspend_cnt++;
					energy_mon.current_suspend_sum += value.intval;

					energy_mon_dbg(ENERGY_MON_DEBUG_SLEEP_ESTIMATOR,
						"%s: hw: %d %d %d: %3dmA for %lus\n", __func__,
						energy_mon.current_suspend_sum,
						energy_mon.current_suspend_cnt,
						energy_mon.current_suspend_sum/energy_mon.current_suspend_cnt,
						value.intval, ts_elapsed.tv_sec);
				} else
					energy_mon_dbg(ENERGY_MON_DEBUG_SLEEP_ESTIMATOR,
						"%s: suspend current read fail\n", __func__);
			} else
				energy_mon_dbg(ENERGY_MON_DEBUG_SLEEP_ESTIMATOR,
					"%s: no power supply for current suspend\n", __func__);
		}else {
			energy_mon_dbg(ENERGY_MON_DEBUG_SLEEP_ESTIMATOR,
				"%s: elapse time too short for current suspend\n", __func__);
		}
#endif
		break;
	}
	return 0;
}
#endif /* CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR */

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
static unsigned int energy_monitor_find_wakeup_stat_wakeup_index(
						int last_wakeup_idx)
{
	unsigned int index;

	if (last_wakeup_idx == ENERGY_MON_WAKEUP_SSP &&
			sensorhub_stat_is_wristup_wakeup())
		index = ENERGY_MON_WAKEUP_WU;
	else if (last_wakeup_idx == ENERGY_MON_WAKEUP_BT &&
			sap_stat_is_noti_wakeup())
		index = ENERGY_MON_WAKEUP_NOTI;
	else
		index = last_wakeup_idx;

	return index;
}

static unsigned int energy_monitor_find_wakeup_stat_time_index(
						ktime_t time)
{
	unsigned int index;
	s64 time_sec;

	time_sec = ktime_to_ms(time) / MSEC_PER_SEC;
	if (time_sec > ENERGY_MON_MAX_WAKEUP_STAT_TIME)
		index = ENERGY_MON_MAX_WAKEUP_STAT_TIME;
	else
		index = (unsigned int)time_sec;

	return index;
}

static int energy_monitor_update_wakeup_stat(unsigned long event)
{
	ktime_t now, delta;
	unsigned int wakeup_idx, time_idx;
	int last_wakeup_idx = energy_mon.last_wakeup_idx;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		if (energy_mon.last_wakeup_idx < ENERGY_MON_WAKEUP_MAX &&
				energy_mon.last_wakeup_idx >= 0) {
			now = ktime_get();
			delta = ktime_sub(now, energy_mon.last_wakeup_time);

			wakeup_idx =
				energy_monitor_find_wakeup_stat_wakeup_index(last_wakeup_idx);

			energy_mon.wakeup_time[wakeup_idx] =
				ktime_add(energy_mon.wakeup_time[wakeup_idx], delta);

			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s for %lld (%d)\n",
				energy_mononitor_get_ws_name_string(wakeup_idx),
				ktime_to_ms(delta),
				suspend_stats.success);

			if (delta.tv64 >= 0) {
				time_idx =
					energy_monitor_find_wakeup_stat_time_index(delta);

				energy_mon.wakeup_time_stats[wakeup_idx][time_idx]++;

				energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
					"ms=%lld: wakeup_idx=%d %d: time_idx=%d\n",
					ktime_to_ms(delta),
					last_wakeup_idx, wakeup_idx,
					time_idx);
			}

			if (wakeup_idx > ENERGY_MON_WAKEUP_ETC)
				energy_mon.wakeup_cause[wakeup_idx]++;
		}
		energy_mon.last_wakeup_idx = -1;
		break;

	case PM_POST_SUSPEND:
		energy_mon.last_wakeup_time = ktime_get();
		break;
	}

	return 0;
}
#endif

static int energy_monitor_pm_notifier(struct notifier_block *nb,
				unsigned long event, void *unused)
{

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG, "%s: event=%lu\n", __func__, event);

	/* If monitor interval is 0, do nothing */
	if (monitor_interval) {
		switch (event) {
		case PM_SUSPEND_PREPARE:
			energy_monitor_background_marker();
			cancel_delayed_work(&monitor_work);
			break;

		case PM_POST_SUSPEND:
			schedule_delayed_work(&monitor_work, monitor_interval * HZ);
			break;
		}
	}

#if defined(CONFIG_ENERGY_MONITOR_SLEEP_CURRENT_ESTIMATOR)
	energy_monitor_sleep_current_estimator(event);
#endif

#ifdef CONFIG_ENERGY_MONITOR_WAKEUP_STAT
	energy_monitor_update_wakeup_stat(event);
#endif

	return NOTIFY_DONE;
}

static struct notifier_block energy_monitor_notifier_block = {
	.notifier_call = energy_monitor_pm_notifier,
	.priority = 0,
};

static void energy_monitor_work(struct work_struct *work)
{
	static int thread_cnt;

	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: (count=%d)\n", __func__, thread_cnt++);

	/* Call background marker when logging interval is setted */
	if (logging_interval)
		energy_monitor_background_marker();

	if (monitor_interval)
		schedule_delayed_work(&monitor_work, monitor_interval * HZ);
	else
		cancel_delayed_work(&monitor_work);
}

static int __init energy_monitor_dt_get_dqa_value(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/tizen_energy_monitor/dqa");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value in DT\n",
			__func__);
		goto fail;
	}

	/* get net_stat cp1 index*/
	if (of_property_read_s32(root, "net_stat_cp1_idx",
			&energy_mon.dqa.net_stat_cp1_idx)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching dqa,net_stat_cp1_idx in DT\n");
		energy_mon.dqa.net_stat_cp1_idx = -1;
	}

	/* get net_stat cp2 index*/
	if (of_property_read_s32(root, "net_stat_cp2_idx",
			&energy_mon.dqa.net_stat_cp2_idx)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching dqa,net_stat_cp2_idx in DT\n");
		energy_mon.dqa.net_stat_cp2_idx = -1;
	}

	/* get net_stat wifi index*/
	if (of_property_read_s32(root, "net_stat_wifi_idx",
			&energy_mon.dqa.net_stat_wifi_idx)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching dqa,net_stat_wifi_idx in DT\n");
		energy_mon.dqa.net_stat_wifi_idx = -1;
	}

	/* get net_stat bt index*/
	if (of_property_read_s32(root, "net_stat_bt_idx",
			&energy_mon.dqa.net_stat_bt_idx)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching dqa,net_stat_bt_idx in DT\n");
		energy_mon.dqa.net_stat_bt_idx = -1;
	}

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %d %d %d %d\n", __func__,
		energy_mon.dqa.net_stat_cp1_idx, energy_mon.dqa.net_stat_cp2_idx,
		energy_mon.dqa.net_stat_wifi_idx, energy_mon.dqa.net_stat_bt_idx);

	return 0;
fail:
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_etc_power_value(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/tizen_energy_monitor/power_value");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value in DT\n",
			__func__);
		goto fail;
	}

	/* get power value of ff */
	if (of_property_read_u32(root, "ff",
			&energy_mon.power_value.ff))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,ff in DT\n");

	/* get power value of gps */
	if (of_property_read_u32(root, "gps",
			&energy_mon.power_value.gps))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,gps in DT\n");

	/* get power value of hr */
	if (of_property_read_u32(root, "hr",
			&energy_mon.power_value.hr))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,hr in DT\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %u %u %u\n",
		__func__, energy_mon.power_value.ff,
		energy_mon.power_value.gps, energy_mon.power_value.hr);

	return 0;
fail:
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_cp_power_value(void)
{
	struct device_node *root;
	int i;
	int size, num_signal;
	unsigned int *temp_array = NULL;
	unsigned int alloc_size;

	/* alloc memory for power_value.cp_active */
	root = of_find_node_by_path("/tizen_energy_monitor/power_value/cp");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value,cp in DT\n",
			__func__);
		goto alloc_fail;
	}

	size = of_property_count_u32_elems(root, "active");
	if (size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,cp,active in DT\n",
			__func__);
		goto alloc_fail;
	}
	energy_mon.power_value.cp_active_table_size = size;

	alloc_size = size * sizeof(struct cp_power_value);
	energy_mon.power_value.cp_active = kzalloc(alloc_size, GFP_KERNEL);
	if (!energy_mon.power_value.cp_active)
		goto alloc_fail;

	/* alloc temp array for cp active */
	alloc_size = size * sizeof(unsigned int);
	temp_array = kzalloc(alloc_size, GFP_KERNEL);
	if (!temp_array)
		goto fail;

	/* get power value for cp active */
	num_signal = of_property_count_u32_elems(root, "signal");
	if (num_signal < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,cp,signal in DT\n",
			__func__);
		goto fail;
	}
	if (energy_mon.power_value.cp_active_table_size != num_signal) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: cp,signal count does not match cp,active count\n",
			__func__);
		goto fail;
	}

	if (!of_property_read_u32_array(root, "active", temp_array, size)) {
		for (i = 0; i < size; i++) {
			energy_mon.power_value.cp_active[i].value = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.cp_active[%d].value=%d\n",
				__func__, i, energy_mon.power_value.cp_active[i].value);
		}
	} else
		goto fail;

	if (!of_property_read_u32_array(root, "signal", temp_array, num_signal)) {
		for (i = 0; i < num_signal; i++) {
			energy_mon.power_value.cp_active[i].signal = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.cp_active[%d].signal=%d\n",
				__func__, i, energy_mon.power_value.cp_active[i].signal);
		}
	} else
		goto fail;

	kfree(temp_array);

	return 0;
fail:
	kfree(energy_mon.power_value.cp_active);
	kfree(temp_array);
alloc_fail:
	energy_mon.power_value.cp_active_table_size = 0;
	energy_mon.power_value.cp_active = NULL;
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_wifi_power_value(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/tizen_energy_monitor/power_value/wifi");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value,wifi in DT\n",
			__func__);
		goto fail;
	}

	/* get power value of wifi scan */
	if (of_property_read_u32(root, "scan",
			&energy_mon.power_value.wifi_scan))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,wifi,scan in DT\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %u\n",
		__func__, energy_mon.power_value.wifi_scan);

	return 0;
fail:
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_bt_power_value(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/tizen_energy_monitor/power_value/bt");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value,bt in DT\n",
			__func__);
		goto fail;
	}

	/* get power value of bt */
	if (of_property_read_u32(root, "active",
			&energy_mon.power_value.bt_active))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,bt,active in DT\n");

	if (of_property_read_u32(root, "tx",
			&energy_mon.power_value.bt_tx))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,bt,tx in DT\n");

	if (of_property_read_u32(root, "rx",
			&energy_mon.power_value.bt_rx))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,bt,rx in DT\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %u %u %d\n",
		__func__,
		energy_mon.power_value.bt_active,
		energy_mon.power_value.bt_tx,
		energy_mon.power_value.bt_rx);

	return 0;
fail:
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_gpu_power_value(void)
{
	struct device_node *root;
	int i;
	int size, num_speed;
	unsigned int *temp_array = NULL;
	unsigned int alloc_size;

	/* alloc memory for power_value.gpu_active */
	root = of_find_node_by_path("/tizen_energy_monitor/power_value/gpu");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value,gpu in DT\n",
			__func__);
		goto alloc_fail;
	}

	size = of_property_count_u32_elems(root, "active");
	if (size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,gpu,active in DT\n",
			__func__);
		goto alloc_fail;
	}
	energy_mon.power_value.gpu_active_table_size = size;

	alloc_size = size * sizeof(struct active_power_value);
	energy_mon.power_value.gpu_active = kzalloc(alloc_size, GFP_KERNEL);
	if (!energy_mon.power_value.gpu_active)
		goto alloc_fail;

	/* alloc temp array for gpu active */
	alloc_size = size * sizeof(unsigned int);
	temp_array = kzalloc(alloc_size, GFP_KERNEL);
	if (!temp_array)
		goto fail;

	/* get power value for gpu active */
	num_speed = of_property_count_u32_elems(root, "speed");
	if (num_speed < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,gpu,speed in DT\n",
			__func__);
		goto fail;
	}
	if (energy_mon.power_value.gpu_active_table_size != num_speed) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: gpu,speed count does not match gpu,active count\n",
			__func__);
		goto fail;
	}

	if (!of_property_read_u32_array(root, "active", temp_array, size)) {
		for (i = 0; i < size; i++) {
			energy_mon.power_value.gpu_active[i].value[0] = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.gpu_active[%d].value=%d\n",
				__func__, i, energy_mon.power_value.gpu_active[i].value[0]);
		}
	} else
		goto fail;

	if (!of_property_read_u32_array(root, "speed", temp_array, num_speed)) {
		for (i = 0; i < num_speed; i++) {
			energy_mon.power_value.gpu_active[i].speed = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.gpu_active[%d].speed=%d\n",
				__func__, i, energy_mon.power_value.gpu_active[i].speed);
		}
	} else
		goto fail;

	kfree(temp_array);

	return 0;
fail:
	kfree(energy_mon.power_value.gpu_active);
	kfree(temp_array);
alloc_fail:
	energy_mon.power_value.gpu_active_table_size = 0;
	energy_mon.power_value.gpu_active = NULL;
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_cpu_power_value(void)
{
	struct device_node *root;
	int i;
	int active_size, whitelist_size, num_speed;
	unsigned int *temp_array = NULL;
	unsigned int alloc_size;

	/* alloc memory for power_value.cpu_active */
	root = of_find_node_by_path("/tizen_energy_monitor/power_value/cpu");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value,cpu in DT\n",
			__func__);
		goto active_alloc_fail;
	}

	active_size = of_property_count_u32_elems(root, "active");
	if (active_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,cpu,active in DT\n",
			__func__);
		goto active_alloc_fail;
	}
	energy_mon.power_value.cpu_active_table_size = active_size;

	alloc_size = active_size * sizeof(struct active_power_value);
	energy_mon.power_value.cpu_active = kzalloc(alloc_size, GFP_KERNEL);
	if (!energy_mon.power_value.cpu_active)
		goto active_alloc_fail;

	/* alloc array for cpu white list */
	whitelist_size = of_property_count_u32_elems(root, "whitelist");
	if (whitelist_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,cpu,whitelist in DT\n",
			__func__);
		goto whitelist_alloc_fail;
	}
	energy_mon.power_value.cpu_whitelist_table_size = whitelist_size;

	alloc_size = whitelist_size * sizeof(unsigned int);
	energy_mon.power_value.cpu_whitelist = kzalloc(alloc_size, GFP_KERNEL);
	if (!energy_mon.power_value.cpu_whitelist)
		goto whitelist_alloc_fail;

	alloc_size = whitelist_size * sizeof(unsigned int);
	temp_array = kzalloc(alloc_size, GFP_KERNEL);
	if (!temp_array)
		goto fail;

	if (!of_property_read_u32_array(root, "whitelist",
					temp_array, whitelist_size)) {
		for (i = 0; i < whitelist_size; i++) {
			energy_mon.power_value.cpu_whitelist[i] = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.cpu,whitelist[%d]=%d\n",
				__func__, i, energy_mon.power_value.cpu_whitelist[i]);
		}
	}
	else
		goto fail;
	kfree(temp_array);


	/* get power value of cpu idle */
	if (of_property_read_u32(root, "sleep",
			&energy_mon.power_value.cpu_sleep))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,cpu,idle in DT\n");

	if (of_property_read_u32(root, "lpm",
			&energy_mon.power_value.cpu_lpm))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,cpu,lpm in DT\n");

	if (of_property_read_u32(root, "idle",
			&energy_mon.power_value.cpu_idle))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matchingpower_value,cpu,idle in DT\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %u %u %u\n",
		__func__, energy_mon.power_value.cpu_sleep,
		energy_mon.power_value.cpu_lpm, energy_mon.power_value.cpu_idle);

	/* alloc temp array for cpu active */
	alloc_size = active_size * sizeof(unsigned int);
	temp_array = kzalloc(alloc_size, GFP_KERNEL);
	if (!temp_array)
		goto fail;

	/* get power value for cpu active */
	num_speed = of_property_count_u32_elems(root, "speed");
	if (num_speed < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,cpu,speed in DT\n",
			__func__);
		goto fail;
	}
	if (energy_mon.power_value.cpu_active_table_size != num_speed) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: cpu_speed count does not match cpu_active count\n",
			__func__);
		goto fail;
	}

	if (!of_property_read_u32_array(root, "active",
					temp_array, active_size)) {
		for (i = 0; i < active_size; i++) {
			energy_mon.power_value.cpu_active[i].value[0] = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.cpu_active[%d].value=%d\n",
				__func__, i, energy_mon.power_value.cpu_active[i].value[0]);
		}
	} else
		goto fail;

	if (!of_property_read_u32_array(root, "cpu1_offset",
					temp_array, active_size)) {
		for (i = 0; i < active_size; i++) {
			energy_mon.power_value.cpu_active[i].value[1] = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.cpu_active[%d].core1_offset=%d\n",
				__func__, i, energy_mon.power_value.cpu_active[i].value[1]);
		}
	} else
		goto fail;

	if (!of_property_read_u32_array(root, "speed",
					temp_array, num_speed)) {
		for (i = 0; i < num_speed; i++) {
			energy_mon.power_value.cpu_active[i].speed = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.cpu_active[%d].speed=%d\n",
				__func__, i, energy_mon.power_value.cpu_active[i].speed);
		}
	} else
		goto fail;

	kfree(temp_array);

	return 0;
fail:
	kfree(energy_mon.power_value.cpu_whitelist);
whitelist_alloc_fail:
	kfree(energy_mon.power_value.cpu_active);
active_alloc_fail:
	kfree(temp_array);
	energy_mon.power_value.cpu_active_table_size = 0;
	energy_mon.power_value.cpu_active = NULL;
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_disp_power_value(void)
{
	struct device_node *root;
	int i;
	int size, num_br;
	unsigned int *temp_array = NULL;
	unsigned int alloc_size;

	/* alloc memory for power_value.disp.on */
	root = of_find_node_by_path("/tizen_energy_monitor/power_value/disp");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing power_value,disp in DT\n",
			__func__);
		goto alloc_fail;
	}

	size = of_property_count_u32_elems(root, "on");
	if (size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,disp,on in DT\n",
			__func__);
		goto alloc_fail;
	}
	energy_mon.power_value.disp_on_table_size = size;

	alloc_size = size * sizeof(struct disp_power_value);
	energy_mon.power_value.disp_on = kzalloc(alloc_size, GFP_KERNEL);
	if (!energy_mon.power_value.disp_on)
		goto alloc_fail;

	/* alloc temp array for disp on */
	alloc_size = size * sizeof(unsigned int);
	temp_array = kzalloc(alloc_size, GFP_KERNEL);
	if (!temp_array)
		goto fail;

	/* get power value for disp on */
	num_br = of_property_count_u32_elems(root, "brightness");
	if (num_br < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing power_value,disp,brightness in DT\n",
			__func__);
		goto fail;
	}
	if (energy_mon.power_value.disp_on_table_size != num_br) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: disp,brightness count does not match disp,on count\n",
			__func__);
		goto fail;
	}

	if (!of_property_read_u32_array(root, "on", temp_array, size)) {
		for (i = 0; i < size; i++) {
			energy_mon.power_value.disp_on[i].value = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.disp_on[%d].value=%d\n",
				__func__, i, energy_mon.power_value.disp_on[i].value);
		}
	} else
		goto fail;

	if (!of_property_read_u32_array(root, "brightness", temp_array, num_br)) {
		for (i = 0; i < num_br; i++) {
			energy_mon.power_value.disp_on[i].brightness = temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: power_value.disp_on[%d].brightness=%d\n",
				__func__, i, energy_mon.power_value.disp_on[i].brightness);
		}
	} else
		goto fail;

	kfree(temp_array);

	/* get power value of aod */
	if (of_property_read_u32(root, "aod",
			&energy_mon.power_value.disp_aod))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching power_value,disp,aod in DT\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s: %u\n",
		__func__, energy_mon.power_value.disp_aod);

	return 0;
fail:
	kfree(energy_mon.power_value.disp_on);
	kfree(temp_array);
alloc_fail:
	energy_mon.power_value.disp_on_table_size = 0;
	energy_mon.power_value.disp_on = NULL;
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

#define MAX_IRQ_TABLE 32
static int __init energy_monitor_dt_get_irq_map_table(
	struct device_node *np)
{
	struct device_node *root;
	int i;
	int size, idx_size;
	const char *temp_irq_name[MAX_IRQ_TABLE];
	int temp_wakeup_idx[MAX_IRQ_TABLE];
	unsigned int alloc_size;

	/* alloc memory for irq_map_table */
	root = of_find_node_by_name(np, "irq_map_table");
	if (!root) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s:missing irq_map_table in DT\n",
			__func__);
		goto alloc_fail;
	}

	size = of_property_count_strings(root, "irq-name");
	if (size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing irq_map_table,irq-name in DT\n",
			__func__);
		goto alloc_fail;
	}
	if (size > MAX_IRQ_TABLE) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: irq_map_table count(%d) reach the max count which allowd\n",
			__func__, size);
		goto alloc_fail;
	}
	energy_mon.irq_map_table_size = size;

	alloc_size = size * sizeof(struct energy_mon_irq_map_table);
	energy_mon.irq_map_table = kzalloc(alloc_size, GFP_KERNEL);
	if (!energy_mon.irq_map_table)
		goto alloc_fail;

	/* get irq name */
	of_property_read_string_array(root, "irq-name", temp_irq_name, size);
	for (i = 0; i < size; i++) {
		energy_mon.irq_map_table[i].irq_name = temp_irq_name[i];
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"%s: irq_name[%d]=%s\n", __func__,
			i, energy_mon.irq_map_table[i].irq_name);
	}

	/* get wakeup index */
	idx_size = of_property_count_u32_elems(root, "wakeup-idx");
	if (idx_size < 0) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: missing irq_map_table,wakeup-idx in DT\n",
			__func__);
		goto fail;
	}
	if (size != idx_size) {
		energy_mon_dbg(ENERGY_MON_DEBUG_ERR,
			"%s: irq-name count does not match wakeup-idx count\n",
			__func__);
		goto fail;
	}

	if (!of_property_read_u32_array(root, "wakeup-idx",
					temp_wakeup_idx, idx_size)) {
		for (i = 0; i < idx_size; i++) {
			energy_mon.irq_map_table[i].wakeup_idx = temp_wakeup_idx[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: wakeup-idx[%d]=%d\n",
				__func__, i, energy_mon.irq_map_table[i].wakeup_idx);
		}
	} else
		goto fail;

	return 0;
fail:
	kfree(energy_mon.irq_map_table);
alloc_fail:
	energy_mon.irq_map_table_size = 0;
	energy_mon.irq_map_table = NULL;
	energy_mon_dbg(ENERGY_MON_DEBUG_ERR, "%s: parsing fail\n", __func__);

	return -1;
}

static int __init energy_monitor_dt_get_fr_threshold_value(
	struct device_node *np)
{
	int i;
	u32 temp_array[ENERGY_MON_WAKEUP_MAX];

	if (!of_property_read_u32_array(np,
			"threshold_fr_wakeup", temp_array, ENERGY_MON_WAKEUP_MAX)) {
		for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++) {
			energy_mon.penalty.threshold_fr_wakeup[i] = (int)temp_array[i];
			energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
				"%s: penalty.threshold_fr_wakeup[%d]=%d\n",
				__func__, i, energy_mon.penalty.threshold_fr_wakeup[i]);
		}
	} else {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_wakeup\n");

		for (i = 0; i < ENERGY_MON_WAKEUP_MAX; i++)
			energy_mon.penalty.threshold_fr_wakeup[i] = -1;
	}

	if(of_property_read_s32(np, "threshold_fr_suspend_fail",
				&energy_mon.penalty.threshold_fr_suspend_fail)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_suspend_fail\n");
		energy_mon.penalty.threshold_fr_suspend_fail = -1;
	}
	if(of_property_read_s32(np, "threshold_fr_suspend_failed_freeze",
				&energy_mon.penalty.threshold_fr_suspend_failed_freeze)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_suspend_failed_freeze\n");
		energy_mon.penalty.threshold_fr_suspend_failed_freeze = -1;
	}
	if(of_property_read_s32(np, "threshold_fr_suspend_failed_prepare",
				&energy_mon.penalty.threshold_fr_suspend_failed_prepare)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_suspend_failed_prepare\n");
		energy_mon.penalty.threshold_fr_suspend_failed_prepare = -1;
	}
	if(of_property_read_s32(np, "threshold_fr_suspend_failed_suspend",
				&energy_mon.penalty.threshold_fr_suspend_failed_suspend)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_suspend_failed_suspend\n");
		energy_mon.penalty.threshold_fr_suspend_failed_suspend = -1;
	}
	if(of_property_read_s32(np, "threshold_fr_suspend_failed_suspend_late",
				&energy_mon.penalty.threshold_fr_suspend_failed_suspend_late)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_suspend_failed_suspend_late\n");
		energy_mon.penalty.threshold_fr_suspend_failed_suspend_late = -1;
	}
	if(of_property_read_s32(np, "threshold_fr_suspend_failed_suspend_noirq",
				&energy_mon.penalty.threshold_fr_suspend_failed_suspend_noirq)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_suspend_failed_suspend_noirq\n");
		energy_mon.penalty.threshold_fr_suspend_failed_suspend_noirq = -1;
	}
	if(of_property_read_s32(np, "threshold_fr_suspend_success",
				&energy_mon.penalty.threshold_fr_suspend_success)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_fr_suspend_success\n");
		energy_mon.penalty.threshold_fr_suspend_success = -1;
	}

	return 0;
}

static int __init energy_monitor_dt_init(void)
{
	struct device_node *np = of_find_node_by_name(NULL, "tizen_energy_monitor");
	unsigned int bat_size;

	if (of_property_read_u32(np, "bat_size",
			&bat_size)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"bat_size is mandatory for energy monitor\n");
		energy_mon.bat_size = DEFAULT_BATTERY_CAPACITY;
	} else
		energy_mon.bat_size = bat_size;

	energy_mon.capacity_per_soc =
		energy_mon.bat_size * 1000 / 100 / 100; /* uAh per 0.01soc */
	energy_mon.unit_bat_capacity = energy_mon.bat_size * 3600 / 100;

	if (of_property_read_s32(np, "monitor_interval",
			&monitor_interval)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: monitor_interval: set default value\n");
		monitor_interval = ENERGY_MON_DEFAULT_MONITOR_INTERVAL;
	}

	if (of_property_read_s32(np, "logging_interval",
			&logging_interval)) {
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: logging_interval: set default value\n");
		logging_interval = ENERGY_MON_DEFAULT_LOGGING_INTERVAL;
	}

	if (of_property_read_s32(np, "use_raw_soc",
			&energy_mon.use_raw_soc))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: use_raw_soc\n");

	if (of_property_read_string(np, "ps_raw_soc",
			(char const **)&energy_mon.ps_raw_soc))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: ps_raw_soc\n");
		energy_mon.ps_raw_soc = "sec-fuelgauge";

	if (of_property_read_string(np, "ps_hw_suspend_energy",
			(char const **)&energy_mon.ps_hw_suspend_energy))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: ps_hw_suspend_energy\n");
		energy_mon.ps_hw_suspend_energy = "sec-fuelgauge";

	if (of_property_read_u32(np, "notify_mask",
			&energy_mon.penalty.notify_mask))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: notify_mask\n");

	if (of_property_read_s32(np, "threshold_time",
			&energy_mon.penalty.threshold_time))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_time\n");

	if (of_property_read_s32(np, "threshold_short_cpu_usage",
			&energy_mon.penalty.threshold_short_cpu_usage))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_short_cpu_usage\n");

	if (of_property_read_s32(np, "threshold_batt",
			&energy_mon.penalty.threshold_batt))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_batt\n");

	if (of_property_read_s32(np, "threshold_sbm_batt",
			&energy_mon.penalty.threshold_sbm_batt))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_sbm_batt\n");

	if (of_property_read_s32(np, "threshold_disp",
			&energy_mon.penalty.threshold_disp))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_disp\n");

	if (of_property_read_s32(np, "threshold_input",
			&energy_mon.penalty.threshold_input))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_input\n");

	if (of_property_read_s32(np, "threshold_alarm_mgr",
			&energy_mon.penalty.threshold_alarm_mgr))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_alarm_mgr\n");

	if (of_property_read_s32(np, "threshold_alarm",
			&energy_mon.penalty.threshold_alarm))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_alarm\n");

	if (of_property_read_s32(np, "threshold_sh",
			&energy_mon.penalty.threshold_sh))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_sh\n");

	if (of_property_read_s32(np, "threshold_sh_wristup",
			&energy_mon.penalty.threshold_sh_wristup))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_sh_wristup\n");

	if (of_property_read_s32(np, "threshold_gps",
			&energy_mon.penalty.threshold_gps))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_gps\n");

	if (of_property_read_s32(np, "threshold_lbs",
			&energy_mon.penalty.threshold_lbs))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_lbs\n");

	if (of_property_read_s32(np, "threshold_cpuidle",
			&energy_mon.penalty.threshold_cpuidle))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_cpuidle\n");

	if (of_property_read_s32(np, "threshold_cpu",
			&energy_mon.penalty.threshold_cpu))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_cpu\n");

	if (of_property_read_s32(np, "threshold_gpu",
			&energy_mon.penalty.threshold_gpu))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_gpu\n");

	if (of_property_read_s32(np, "threshold_ws",
			&energy_mon.penalty.threshold_ws))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_ws\n");

	if (of_property_read_s32(np, "threshold_slwl",
			&energy_mon.penalty.threshold_slwl))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_slwl\n");

	if (of_property_read_s32(np, "threshold_suspend_fail",
			&energy_mon.penalty.threshold_suspend_fail))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_suspend_fail\n");

	if (of_property_read_s32(np, "threshold_sid",
			&energy_mon.penalty.threshold_sid))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_sid\n");

	if (of_property_read_s32(np, "threshold_pid",
			&energy_mon.penalty.threshold_pid))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_pid\n");

	if (of_property_read_s32(np, "threshold_booting_offset",
			&energy_mon.penalty.threshold_booting_offset))
	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_booting_offset\n");

	if (of_property_read_s32(np, "threshold_log_offset",
			&energy_mon.penalty.threshold_log_offset))
		energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
			"No matching property: threshold_log_offset\n");

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO,
		"%s: %d %d %s %s %x %d %d %d %d %d %d %d %d"
		"%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
		__func__,
		energy_mon.bat_size,
		energy_mon.use_raw_soc,
		energy_mon.ps_raw_soc,
		energy_mon.ps_hw_suspend_energy,
		energy_mon.penalty.notify_mask,
		energy_mon.penalty.threshold_time,
		energy_mon.penalty.threshold_short_cpu_usage,
		energy_mon.penalty.threshold_batt,
		energy_mon.penalty.threshold_sbm_batt,
		energy_mon.penalty.threshold_disp,
		energy_mon.penalty.threshold_input,
		energy_mon.penalty.threshold_alarm_mgr,
		energy_mon.penalty.threshold_alarm,
		energy_mon.penalty.threshold_sh,
		energy_mon.penalty.threshold_sh_wristup,
		energy_mon.penalty.threshold_gps,
		energy_mon.penalty.threshold_lbs,
		energy_mon.penalty.threshold_cpuidle,
		energy_mon.penalty.threshold_cpu,
		energy_mon.penalty.threshold_gpu,
		energy_mon.penalty.threshold_ws,
		energy_mon.penalty.threshold_slwl,
		energy_mon.penalty.threshold_suspend_fail,
		energy_mon.penalty.threshold_sid,
		energy_mon.penalty.threshold_pid,
		energy_mon.penalty.threshold_booting_offset,
		energy_mon.penalty.threshold_log_offset);

	energy_monitor_dt_get_fr_threshold_value(np);
	energy_monitor_dt_get_irq_map_table(np);
	energy_monitor_dt_get_disp_power_value();
	energy_monitor_dt_get_cpu_power_value();
	energy_monitor_dt_get_gpu_power_value();
	energy_monitor_dt_get_wifi_power_value();
	energy_monitor_dt_get_bt_power_value();
	energy_monitor_dt_get_cp_power_value();
	energy_monitor_dt_get_etc_power_value();
	energy_monitor_dt_get_dqa_value();

	return 0;
}

static int __init energy_monitor_init(void)
{
	int ret;
	int i;

	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s\n", __func__);

	ret = energy_monitor_dt_init();
	if (ret)
		return ret;

	energy_mon.emon_ws = wakeup_source_register("emon");
	if (!energy_mon.emon_ws)
		return -ENOMEM;

	/* Initialize datas */
	for (i = 0; i < ENERGY_MON_HISTORY_NUM ; i++)
		energy_mon.data[i].log_count = -1;

	energy_mon.last_wakeup_idx = -1;

	/* Check size of control block for information */
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: sizeof(struct energy_mon_data) is %lu\n",
		__func__, sizeof(struct energy_mon_data));
	energy_mon_dbg(ENERGY_MON_DEBUG_DBG,
		"%s: sizeof(energy_mon) is %lu(ENERGY_MON_HISTORY_NUM=%d)\n",
		__func__, sizeof(energy_mon), ENERGY_MON_HISTORY_NUM);

	energy_monitor_debug_init();

	register_pm_notifier(&energy_monitor_notifier_block);

	INIT_DELAYED_WORK(&monitor_work, energy_monitor_work);
	if (monitor_interval)
		schedule_delayed_work(&monitor_work, monitor_interval * HZ);

	return 0;
}

static void energy_monitor_exit(void)
{
	energy_mon_dbg(ENERGY_MON_DEBUG_INFO, "%s\n", __func__);
	wakeup_source_unregister(energy_mon.emon_ws);
}

late_initcall(energy_monitor_init);
module_exit(energy_monitor_exit);
