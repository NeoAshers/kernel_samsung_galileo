/*s6e36w4x01_mipi_lcd.h
 *
 * Samsung SoC MIPI LCD driver.
 *
 * Copyright (c) 2019 Samsung Electronics
 *
 * SeungBeom, Park <sb1.parki@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __S6E36W4X01_MIPI_LCD_H__
#define __S6E36W4X01_MIPI_LCD_H__

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/lcd.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/trm.h>

#include <video/mipi_display.h>
#include <linux/platform_device.h>
#include "s6e36w4x01_sc_ctrl.h"
#include "mdnie_lite.h"
#include "../dsim.h"

#define MAX_BR_INFO		74
#define DEFAULT_BUFF_SIZE	32

enum {
	AO_NODE_OFF = 0,
	AO_NODE_ALPM = 1,
	AO_NODE_SELF = 2,
};

enum {
	HLPM_NIT_OFF,
	HLPM_NIT_LOW,
	HLPM_NIT_HIGH,
};

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

struct s6e36w4x01 {
	struct device		*dev;
	struct device		*esd_dev;
	struct class		*esd_class;
	struct dsim_device	*dsim;
	struct lcd_device	*ld;
	struct backlight_device	*bd;
	struct mdnie_lite_device	*mdnie;
	struct smart_dimming	*dimming;
	struct regulator	*vdd3;
	struct regulator	*vci;
	struct delayed_work	esd_dwork;
	struct delayed_work	debug_dwork;
	struct delayed_work	afpc_dwork;
	struct mutex		afpc_lock;
	struct mutex		mipi_lock;
	struct sc_time_st	sc_time;
	bool			panel_sc_state;
	bool			panel_icon_state;
	unsigned char		*br_map;
	unsigned int		reset_gpio;
	unsigned int		te_gpio;
	unsigned int		det_gpio;
	unsigned int		err_gpio;
	unsigned int		esd_irq;
	unsigned int		esd_cnt;
	unsigned int		err_irq;
	unsigned int		power;
	unsigned int		acl;
	unsigned int		refresh;
	unsigned char		default_hbm;
	unsigned char		hbm_gamma[GAMMA_CMD_CNT];
	unsigned char		hbm_elvss[HBM_ELVSS_CMD_CNT];
	unsigned char		*gamma_tbl[MAX_GAMMA_CNT];
	unsigned int		dbg_cnt;
	unsigned int		ao_mode;
	enum temp_range_t	temp_stage;
	enum lpm_level_t	min_lpm_lv;
	int			temperature;
#ifdef CONFIG_COPR_SUPPORT
	unsigned int		copr;
#endif
	bool			hbm_on;
	bool			alpm_on;
	bool			hlpm_on;
	bool			lp_mode;
	bool			boot_power_on;
	bool			br_ctl;
	bool			scm_on;
	enum aod_mode		aod_mode;
	int			hlpm_nit;
	bool			mcd_on;
	bool			irq_on;
	char			*icon_buf;
	char			*afpc_buf;
	char			*sc_buf;
	char			*sc_dft_buf;
	unsigned int		sc_rate;
	ktime_t			hbm_stime;
	unsigned int		hbm_total_time;
	ktime_t			disp_stime;
	unsigned int		disp_total_time;
	unsigned int		disp_cnt;
	ktime_t		 	aod_stime;
	unsigned int		aod_total_time;
	unsigned int		aod_high_time;
	unsigned int		aod_low_time;
	int			prev_hlpm_nit;
	char			resolution;
	bool			enable_ddi_debug;
	bool			enable_hop;
	bool			enable_fd;
	bool			enable_afpc;
	bool			enable_circle_mask;
	bool			ub_con_connected;
	int			last_brightness;
	unsigned char		br_level;
#ifdef CONFIG_SLEEP_MONITOR
	unsigned int		act_cnt;
#endif
	int			lpm_br_max;
	int			lpm_br_normal;
	int			lpm_br_min;
        int			lpm_br_charging;
	enum aod_panel_refresh_rate aod_refresh_status;
	enum aod_panel_refresh_rate aod_refresh;
};
#if defined(ENABLE_PRINT_DISPLAY_DQA)
void s6e36w4x01_print_dqa(struct s6e36w4x01 *lcd, int display_on, int aod);
#endif
#endif
