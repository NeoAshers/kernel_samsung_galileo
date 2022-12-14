/* rm69091_mipi_lcd.c
 *
 * Samsung SoC MIPI LCD driver.
 *
 * Copyright (c) 2020 Samsung Electronics
 *
 * SeungBeom, Park <sb1.parki@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __RM69091_MIPI_LCD_H__
#define __RM69091_MIPI_LCD_H__

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/lcd.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/trm.h>
#include "../dsim.h"

#include <video/mipi_display.h>
#include <linux/platform_device.h>

enum temp_range_t {
	TEMP_RANGE_0 = 0,	/* 0 < temperature */
	TEMP_RANGE_1,		/* -10 < temperature <= 0 */
	TEMP_RANGE_2,		/* temperature <= -10 */
	TEMP_RANGE_MAX,
};

enum temp_ref_t {
	TEMP_REF_1 = -10,
	TEMP_REF_0 = 0,
	TEMP_REF_MAX,
};

enum lpm_level_t {
	LPM_BR_LOW,
	LPM_BR_CHARGING,
	LPM_BR_MID,
	LPM_BR_HIGH,
	LPM_BR_MAX,
};

struct dqa_data_t {
	ktime_t			disp_stime;
	ktime_t			hbm_stime;
	ktime_t		 	aod_stime;
	unsigned int	disp_time;
	unsigned int	hbm_time;
	unsigned int 	aod_high_time;
	unsigned int 	aod_low_time;
	unsigned int 	disp_cnt;
};

struct rm69091 {
	struct device		*dev;
	struct device		*esd_dev;
	struct class		*esd_class;
	struct dsim_device	*dsim;
	struct lcd_device	*ld;
	struct backlight_device	*bd;
	struct regulator	*vdd3;
	struct regulator	*vci;
	struct work_struct	det_work;
	struct mutex		mipi_lock;
	struct delayed_work	debug_dwork;
	struct delayed_work	esd_dwork;
	struct dqa_data_t	dqa_data;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*gpio_aod;
	struct pinctrl_state	*gpio_off;
	enum temp_range_t	temp_stage;
	enum lpm_level_t	min_lpm_lv;
	int			temperature;
	const unsigned char	*br_map;
	unsigned char		br_level;
	unsigned char		hbm_level;
	unsigned int		esd_cnt;
	unsigned int		reset_gpio;
	unsigned int		te_gpio;
	unsigned int		det_gpio;
	unsigned int		err_gpio;
	unsigned int		esd_irq;
	unsigned int		err_irq;
	unsigned int		power;
	bool			lpm_on;
	bool			hbm_on;
	bool			mcd_on;
	bool			ub_con_connected;
	bool			enable_curve;
	bool			enable_crack_check;
	bool			enable_fd;
	int			lpm_br_max;
	int			lpm_br_normal;
	int			lpm_br_min;
	int			lpm_br_charging;
	enum aod_panel_refresh_rate aod_refresh;
};

#if defined(ENABLE_PRINT_DISPLAY_DQA)
void rm69091_print_dqa(struct rm69091 *lcd, int display_on, int aod);
#endif
#endif
