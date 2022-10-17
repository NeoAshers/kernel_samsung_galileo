/*
 *  include/linux/ff_stat_tizen.h
 *
 *  Copyright (C) 2018 Junho Jang <vincent.jang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __FF_STAT_TIZEN_H
#define __FF_STAT_TIZEN_H

#include <linux/input.h>

 struct ff_stat_emon {
	 ktime_t total_time;
	 unsigned long play_count;
 };

#ifdef CONFIG_FF_STAT_TIZEN
extern int ff_stat_tizen_get_stat(struct ff_stat_emon *emon_stat);
extern int ff_stat_tizen_get_stat_emon(int type, struct ff_stat_emon *emon_stat);
#else
static inline int ff_stat_tizen_get_stat(struct ff_stat_emon *emon_stat) {	return 0;}
static inline int ff_stat_tizen_get_stat_emon(int type, struct ff_stat_emon *emon_stat) {return 0;}
#endif
#endif /* __FF_STAT_TIZEN_H */

