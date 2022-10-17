/*
 *  include/linux/sleep_stat.h
 *
 *  Copyright (C) 2018 Junho Jang <vincent.jang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SLEEP_STAT_H
#define __SLEEP_STAT_H

/* EXYNOS_MIF_MASTER_MAX should be same as MIF_MASTER_MAX */
#define EXYNOS_MIF_MASTER_MAX		8

struct apm_req_info {
	unsigned int total_cnt;
	unsigned long long total_time;
};

struct mif_wakeup_source {
	ktime_t last_time;
	unsigned int wakeup_count;
};

struct apm_wakeup_source {
	ktime_t last_time;
	unsigned int apm_req_count;
	unsigned int tcxo_req_count;
	unsigned int pwr_req_count;
};

struct sleep_stat_exynos {
	int acpm_sleep_early_wakeup;
	int acpm_sleep_soc_down;
	int acpm_sleep_mif_down;
	int acpm_sicd_early_wakeup;
	int acpm_sicd_soc_down;
	int acpm_sicd_mif_down;
	struct apm_req_info apm[EXYNOS_MIF_MASTER_MAX];
	struct mif_wakeup_source mif_wakeup[EXYNOS_MIF_MASTER_MAX];
	struct apm_wakeup_source apm_wakeup[EXYNOS_MIF_MASTER_MAX];
};

#define SLEEP_STAT_SOC_EXYNOS	0x10
#define SLEEP_STAT_SOC_QC		0x20

struct sleep_stat {
 	int fail;
	int failed_freeze;
	int failed_prepare;
	int failed_suspend;
	int failed_suspend_late;
	int failed_suspend_noirq;
	int suspend_success;

	int soc_type;
	union {
		struct sleep_stat_exynos exynos;
	} soc;
};

#ifdef CONFIG_SLEEP_STAT
extern int sleep_stat_get_stat(
					struct sleep_stat *sleep_stat);
extern int sleep_stat_get_stat_emon(int type,
 					struct sleep_stat *sleep_stat);
extern int sleep_stat_register_soc(int soc_type, void *soc_read);
#else
static inline int sleep_stat_get_stat(
 					struct sleep_stat *sleep_stat) {	return 0;}
static inline int sleep_stat_get_stat_emon(int type,
 					struct sleep_stat *sleep_stat) {	return 0;}
static inline int sleep_stat_register_soc(int soc_type, void *soc_read)
	{	return 0;}
#endif

#endif /* __SLEEP_STAT_H */

