/* common.h
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PANEL_COMMON_H__
#define __PANEL_COMMON_H__

#define PANEL_30HZ_1FRAME_MSEC  34 /* 34 ms */
#define PANEL_60HZ_1FRAME_MSEC  17 /* 17 ms */

int panel_vsync_init(void);
int panel_vsync_cleanup(void);
void panel_vsync_wakeup_interrupt(ktime_t tstamp);
int panel_vsync_wait(int one_frame_delay);

#endif  /* __PANEL_COMMON_H__ */
