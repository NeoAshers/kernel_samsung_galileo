/* common.c
 *
 * Samsung SoC LCD driver.
 *
 * Copyright (c) 2019 Samsung Electronics
 *
 * Sangmin, Lee <lsmin.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include "common.h"

struct panel_vsync {
	wait_queue_head_t wait;
	ktime_t timestamp;
	bool inited;
};

static struct panel_vsync sync_instance;
int panel_vsync_init(void)
{
	if (sync_instance.inited) {
		pr_info("panel_vsync is already inited\n");
		return -1;
	}
	init_waitqueue_head(&sync_instance.wait);
	sync_instance.inited = true;

	pr_info("%s done\n", __func__);

	return 0;
}

int panel_vsync_cleanup(void)
{
	if (!sync_instance.inited)
		return -1;

	wake_up_interruptible_all(&sync_instance.wait);
	sync_instance.inited = false;

	pr_info("%s done\n", __func__);

	return 0;
}

void panel_vsync_wakeup_interrupt(ktime_t tstamp)
{
	sync_instance.timestamp = tstamp;
	wake_up_interruptible_all(&sync_instance.wait);
}

int panel_vsync_wait(int one_frame_delay)
{
	int ret;
	ktime_t timestamp;

	if (!sync_instance.inited)
		return -1;

	timestamp = sync_instance.timestamp;
	ret = wait_event_interruptible_timeout(sync_instance.wait,
		!ktime_equal(timestamp, sync_instance.timestamp),
		msecs_to_jiffies(one_frame_delay));

	if (!ret)
		pr_err("%s:wait for vsync timeout\n", __func__);

	return ret;
}

static int panel_id;
static int __init panel_id_cmdline(char *mode)
{
	char *pt;

	panel_id = 0;

	if (mode == NULL)
		return 1;

	for (pt = mode; *pt != 0; pt++) {
		panel_id <<= 4;
		switch (*pt) {
		case '0' ... '9':
			panel_id += *pt - '0';
		break;
		case 'a' ... 'f':
			panel_id += 10 + *pt - 'a';
		break;
		case 'A' ... 'F':
			panel_id += 10 + *pt - 'A';
		break;
		}
	}

	pr_info("%s:panel_id[0x%06x]", __func__, panel_id);

	return 0;
}
__setup("lcdtype=", panel_id_cmdline);

int get_panel_id(void)
{
	return panel_id;
}
EXPORT_SYMBOL(get_panel_id);

#ifdef CONFIG_EXYNOS_DSIM_MULTI_PANEL_SUPPORT
#ifdef CONFIG_NOBLESSE
#define 	SDC_ID1	0x40
int get_panel_vendor(void)
{
	pr_info("%s[0x%02x]\n", __func__, (panel_id >> 16));

	return ((panel_id >> 16) == SDC_ID1) ? 0 : 1;
}
#else
int get_panel_vendor(void)
{
	return 0;
}
#endif
#endif
EXPORT_SYMBOL(get_panel_vendor);
