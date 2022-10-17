/* s6e36w4x01_mipi_lcd.c
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>
#include <linux/platform_device.h>
#include <linux/lcd.h>

#include "../dsim.h"
#include "s6e36w4x01_dimming.h"
#include "s6e36w4x01_param.h"
#include "s6e36w4x01_mipi_lcd.h"
#include "s6e36w4x01_lcd_ctrl.h"
#include "s6e36w4x01_sc_ctrl.h"
#include "decon_lcd.h"

#ifdef CONFIG_SLEEP_MONITOR
#include <linux/power/sleep_monitor.h>
#endif

#define DISPLAY_DQA 1
#define BACKLIGHT_DEV_NAME	"s6e36w4x01-bl"
#define LCD_DEV_NAME		"s6e36w4x01"

static const int hlpm_brightness[HLPM_NIT_HIGH+1] = {0, 5, 40};
const struct s6e36w4x01 *g_s6e36w4x01;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend    s6e36w4x01_early_suspend;
#endif

extern int get_panel_id(void);

#define	DEBUG_READ_DELAY	100 /* 100 ms */
#define	AFPC_WRITE_DELAY	20 /* 20 ms */

extern int s6e36w4x01_print_debug_reg(int id);
extern void s6e36w4x01_testkey_enable(u32 id);
extern void s6e36w4x01_testkey_disable(u32 id);

#if defined(ENABLE_PRINT_DISPLAY_DQA)
void s6e36w4x01_print_dqa(struct s6e36w4x01 *lcd, int display_on, int aod)
{
	if (aod)
		dev_info(lcd->dev, "[DQA] AOD %s -> display[%d:%d] AOD[%d:%d]\n",
			display_on ? "on" : "off", lcd->disp_cnt, lcd->disp_total_time,
			lcd->aod_high_time, lcd->aod_total_time);
	else
		dev_info(lcd->dev, "[DQA] %s -> display[%d:%d] AOD[%d:%d]\n",
			display_on ? "on" : "off", lcd->disp_cnt, lcd->disp_total_time,
			lcd->aod_high_time, lcd->aod_total_time);
}
#endif

static const char* get_power_state_str(int cmd)
{
	const char* power_state[FB_BLANK_POWERDOWN+1] = {
		"UNBLANK", "NORMAL", "V_SUSPEND", "H_SUSPEND", "POWERDOWN",
	};

	if (cmd > FB_BLANK_POWERDOWN)
		return "";

	return &power_state[cmd][0];
}

static void s6e36w4x01_debug_dwork(struct work_struct *work)
{
	struct s6e36w4x01 *lcd = container_of(work,
				struct s6e36w4x01, debug_dwork.work);
	struct dsim_device *dsim = lcd->dsim;
	int ret;

	cancel_delayed_work(&lcd->debug_dwork);

	if (POWER_IS_OFF(lcd->power)) {
		dev_err(lcd->dev, "%s:panel off\n", __func__);
		return;
	}

	s6e36w4x01_testkey_enable(dsim->id);

	ret = s6e36w4x01_print_debug_reg(dsim->id);
	if (ret) {
		dev_err(lcd->dev, "%s:failed print_debug_reg\n", __func__);
	}

	s6e36w4x01_testkey_disable(dsim->id);
}

static void s6e36w4x01_send_esd_event(struct s6e36w4x01 *lcd)
{
	char *event_string = "LCD_ESD=ON";
	char *envp[] = {event_string, NULL};

	if (lcd->esd_dev != NULL)
		kobject_uevent_env(&lcd->esd_dev->kobj, KOBJ_CHANGE, envp);
	else
		pr_err("%s: esd_dev is NULL\n", __func__);
}

#define ESD_RETRY_TIMEOUT		(10*1000) /* 10 seconds */
static void s6e36w4x01_esd_dwork(struct work_struct *work)
{
	struct s6e36w4x01 *lcd = container_of(work,
				struct s6e36w4x01, esd_dwork.work);

	if (POWER_IS_OFF(lcd->power)) {
		dev_err(lcd->dev, "%s:panel off.\n", __func__);
		return;
	}

	cancel_delayed_work(&lcd->esd_dwork);
	schedule_delayed_work(&lcd->esd_dwork,
			msecs_to_jiffies(ESD_RETRY_TIMEOUT));

	s6e36w4x01_send_esd_event(lcd);

	lcd->esd_cnt++;
	dev_info(lcd->dev, "%s:Send uevent. ESD DETECTED[%d]\n",
					__func__, lcd->esd_cnt);
}

extern unsigned int system_rev;
static void s6e36w4x01_show_state(struct dsim_device *dsim)
{
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);
	struct mdnie_lite_device *mdnie = lcd->mdnie;
	struct backlight_device	*bd = lcd->bd;

	pr_info("%s:system_rev[0x%02x] panel_id[0x%06x]\n",
				__func__, system_rev, get_panel_id());
	pr_info("%s:power[%s] aod[%d]\n",
				__func__, get_power_state_str(lcd->power), lcd->aod_mode);
	pr_info("%s:mdnie[0x%02x] brightness[%d] hbm[%s]\n",
				__func__, mdnie->scenario, bd->props.brightness,
				lcd->hbm_on ? "on":"off");
	pr_info("%s:apfc[%s] hop[%s] esd_cnt[%d]\n",
				__func__, lcd->enable_afpc ? "on":"off",
				lcd->enable_hop ? "on":"off", lcd->esd_cnt);

	return;
}

int s6e36w4x01_ub_con_det(struct dsim_device *dsim, bool status)
{
	struct lcd_device *panel = dsim->ld;
	struct dsim_resources *res = &dsim->res;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);

	if (panel == NULL) {
		pr_err("%s:panel is NULL\n", __func__);
		return -1;
	}

	lcd->ub_con_connected = status;

	if (!lcd->ub_con_connected) {
		pr_info("%s[disconnected]\n", __func__);
		if (res->err_fg > 0) {
			disable_irq(lcd->esd_irq);
			cancel_delayed_work_sync(&lcd->esd_dwork);
		}
	}

	return 0;
}

extern unsigned int system_rev;
#ifdef CONFIG_NOBLESSE
static int s6e36w4x01_set_enable_fd(struct s6e36w4x01 *lcd, bool enable)
{
#ifndef CONFIG_TIZEN_SEC_KERNEL_ENG
	dev_err(lcd->dev, "%s:only support in eng binary\n", __func__);
	return -EPERM;
#endif

	lcd->enable_fd = enable;
	dev_err(lcd->dev, "%s:enable_fd[%d]\n",	__func__, lcd->enable_fd);

	return 0;
}
#else
static int s6e36w4x01_set_enable_fd(struct s6e36w4x01 *lcd, bool enable)
{
#ifndef CONFIG_RENAISSANCE
	dev_err(lcd->dev, "%s:only support in renaissance project\n", __func__);
	return -EPERM;
#endif

#ifndef CONFIG_TIZEN_SEC_KERNEL_ENG
	dev_err(lcd->dev, "%s:only support in eng binary\n", __func__);
	return -EPERM;
#endif

#ifdef CONFIG_RENAISSANCE_LARGE
	if (system_rev <= 2) {
		dev_err(lcd->dev, "%s:not support board revision[%d]\n", __func__, system_rev);
		return -EPERM;
	}
#endif

#ifdef CONFIG_RENAISSANCE_SMALL
	if (system_rev <= 3) {
		dev_err(lcd->dev, "%s:not support board revision[%d]\n", __func__, system_rev);
		return -EPERM;
	}
#endif

	lcd->enable_fd = enable;
	dev_err(lcd->dev, "%s:enable_fd[%d]\n",	__func__, lcd->enable_fd);

	return 0;
}
#endif
static ssize_t s6e36w4x01_enable_fd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->enable_fd);
}

extern void s6e36w4x01_enable_fd(struct s6e36w4x01 *lcd);
extern void s6e36w4x01_disable_fd(struct s6e36w4x01 *lcd);
static ssize_t s6e36w4x01_enable_fd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;
	bool value;

	ret = strtobool(buf, &value);
	if (ret < 0) {
		dev_err(lcd->dev,
			"%s: failed to read parameter value\n",	__func__);
		return -EIO;
	}

	ret = s6e36w4x01_set_enable_fd(lcd, value);
	if (ret < 0) {
		dev_err(lcd->dev,
			"%s: failed to enable fd[%d]\n", __func__, ret);
		return -EIO;
	}

	pr_info("%s:enable_fd[%d]\n", __func__, lcd->enable_fd);

	if (lcd->power > FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		dev_err(lcd->dev, "%s:aod enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->enable_fd)
		s6e36w4x01_enable_fd(lcd);
	else
		s6e36w4x01_disable_fd(lcd);

	return size;
}

extern int s6e36w4x01_set_value_vbias(struct s6e36w4x01 *lcd, unsigned char value);
extern unsigned char s6e36w4x01_get_value_vbias(void);
static ssize_t s6e36w4x01_vbias_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	pr_info("%s:[0x%02x]", __func__, s6e36w4x01_get_value_vbias());

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", s6e36w4x01_get_value_vbias());
}

static ssize_t s6e36w4x01_vbias_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	unsigned char value;
	int ret;

	ret = kstrtou8(buf, 0, &value);
	if (ret < 0) {
		dev_err(lcd->dev,
			"%s: failed to read parameter value\n",	__func__);
		return -EIO;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		dev_err(lcd->dev, "%s:aod enabled\n", __func__);
		return -EPERM;
	}

	ret = s6e36w4x01_set_value_vbias(lcd, value);
	if (ret < 0) {
		dev_err(lcd->dev,
			"%s: failed to set_vbias[%d]\n", __func__, ret);
		return -EIO;
	}

	pr_info("%s:set_vbias[0x%02x]\n", __func__, value);

	return size;
}

static ssize_t s6e36w4x01_mcd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->mcd_on ? "on" : "off");
}

extern int s6e36w4x01_hbm_on(struct s6e36w4x01 *lcd);
extern int s6e36w4x01_hbm_off(struct s6e36w4x01 *lcd);
extern void s6e36w4x01_mcd_test_on(struct s6e36w4x01 *lcd);
extern void s6e36w4x01_mcd_test_off(struct s6e36w4x01 *lcd);
static ssize_t s6e36w4x01_mcd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int old_hbm;

	if (lcd->alpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->hbm_on) {
		old_hbm = lcd->hbm_on;
		lcd->hbm_on = 0;
		s6e36w4x01_hbm_off(lcd);
		lcd->hbm_on = old_hbm;
	}

	if (!strncmp(buf, "on", 2)) {
		lcd->mcd_on = true;
		s6e36w4x01_mcd_test_on(lcd);
	} else if (!strncmp(buf, "off", 3)) {
		lcd->mcd_on = false;
		s6e36w4x01_mcd_test_off(lcd);
	} else
		dev_err(dev, "%s:invalid command.\n", __func__);

	pr_info("%s:[%s]\n", __func__, lcd->mcd_on ? "on":"off");

	return size;
}

static ssize_t s6e36w4x01_circle_mask_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->enable_circle_mask ? "on" : "off");
}

extern void s6e36w4x01_circle_mask_on(struct s6e36w4x01 *lcd);
extern void s6e36w4x01_circle_mask_off(struct s6e36w4x01 *lcd);
static ssize_t s6e36w4x01_circle_mask_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int value;

	if (!strncmp(buf, "on", 2))
		value = 1;
	else if (!strncmp(buf, "off", 3))
		value = 0;
	else {
		dev_warn(dev, "invalid comman (use on or off)\n");
		return size;
	}

	dev_info(lcd->dev, "%s:cur[%d] new[%d]\n",
		__func__, lcd->enable_circle_mask, value);

	lcd->enable_circle_mask = value;

	if (lcd->alpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->enable_circle_mask)
		s6e36w4x01_circle_mask_on(lcd);
	else
		s6e36w4x01_circle_mask_off(lcd);

	return size;
}


static ssize_t s6e36w4x01_hlpm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	pr_debug("%s[%d]\n", __func__, hlpm_brightness[lcd->hlpm_nit]);

	return scnprintf(buf, PAGE_SIZE, "%d\n", hlpm_brightness[lcd->hlpm_nit]);
}

extern void s6e36w4x01_hlpm_on_10hz(struct s6e36w4x01 *lcd);
extern void s6e36w4x01_hlpm_off(struct s6e36w4x01 *lcd);
static ssize_t s6e36w4x01_hlpm_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	static int last_value = -1;
	unsigned char value;
	int ret;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	ret = kstrtou8(buf, 0, &value);
	if (ret < 0) {
		dev_err(lcd->dev,
			"%s: failed to read parameter value\n",	__func__);
		return -EIO;
	}

	if (value > AOD_SCLK_DIGITAL)
		value = AOD_SCLK_DIGITAL;

	if (value == 0) {
		if (last_value != -1)
			s6e36w4x01_aod_exit(lcd->dsim, last_value);
		goto out;
	}

	ret = s6e36w4x01_aod_enter(lcd->dsim, value);
	if (ret) {
		dev_err(lcd->dev,
			"%s: failed s6e36w4x01_aod_enter[%d]\n", __func__, ret);
		return -EIO;
	}
	last_value = value;

out:
	pr_info("%s: aod mode[%d]\n", __func__, value);

	return size;
}

static ssize_t s6e36w4x01_hop_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	pr_info("%s:val[%d]\n", __func__, lcd->enable_hop);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->enable_hop);
}

extern void s6e36w4x01_disable_hop(struct s6e36w4x01 *lcd);
extern void s6e36w4x01_enable_hop(struct s6e36w4x01 *lcd);
static ssize_t s6e36w4x01_hop_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;
	bool value;

	ret = strtobool(buf, &value);
	if (ret < 0) {
		dev_err(lcd->dev,
			"%s: failed to read parameter value\n",	__func__);
		return -EIO;
	}

	lcd->enable_hop = value;
	pr_info("%s:val[%d]\n", __func__, lcd->enable_hop);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->enable_hop)
		s6e36w4x01_enable_hop(lcd);
	else
		s6e36w4x01_disable_hop(lcd);

	return size;
}

static ssize_t s6e36w4x01_alpm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int len = 0;

	pr_info("%s:val[%d]\n", __func__, lcd->ao_mode);

	switch (lcd->ao_mode) {
	case AO_NODE_OFF:
		len = scnprintf(buf, PAGE_SIZE, "off\n");
		break;
	case AO_NODE_ALPM:
		len = scnprintf(buf, PAGE_SIZE, "on\n");
		break;
	case AO_NODE_SELF:
		len = scnprintf(buf, PAGE_SIZE, "self\n");
		break;
	default:
		dev_warn(dev, "invalid status.\n");
		break;
	}

	return len;
}

static ssize_t s6e36w4x01_alpm_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	if (!strncmp(buf, "on", 2))
		lcd->ao_mode = AO_NODE_ALPM;
	else if (!strncmp(buf, "off", 3))
		lcd->ao_mode = AO_NODE_OFF;
	else if (!strncmp(buf, "self", 4))
		lcd->ao_mode = AO_NODE_SELF;
	else
		dev_warn(dev, "invalid command.\n");

	pr_info("%s:val[%d]\n", __func__, lcd->ao_mode);

	return size;
}

void s6e36w4x01_set_resolution(int id, int resolution);
static ssize_t s6e36w4x01_resolution_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	unsigned long resolution;
	int ret;

	ret = kstrtoul(buf, 0, &resolution);
	if (ret) {
		dsim_err("%s:failed buffer read[%d]\n", __func__, ret);
		return ret;
	}

	if (resolution < 0)
		resolution = 0;
	if (resolution > 2)
		resolution = 2;

	s6e36w4x01_set_resolution(dsim->id, (int)resolution);
	lcd->resolution = (char)resolution;

	pr_info("%s:val[%d]\n", __func__, lcd->resolution);

	return size;
}

static ssize_t s6e36w4x01_resolution_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n,", lcd->resolution);
}

static ssize_t s6e36w4x01_esd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->esd_cnt);
}

static ssize_t s6e36w4x01_esd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	cancel_delayed_work(&lcd->esd_dwork);
	schedule_delayed_work(&lcd->esd_dwork, 0);

	return size;
}

#define	DDI_DEBUG_REG	0x9c
#define	DDI_DEBUG_REG_SIZE	251
int s6e36w4x01_read_mtp_reg(int id, u32 addr, char* buffer, u32 size);
static ssize_t s6e36w4x01_ddi_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	char read_buf[DDI_DEBUG_REG_SIZE] = {0, };
	char show_buf[DEFAULT_BUFF_SIZE] = {0, };
	int ret = 0;
	int i;

	if (!lcd->enable_ddi_debug) {
		pr_info("%s: enable_ddi_debug disabled\n", __func__);
		return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->enable_ddi_debug);
	}

	ret = s6e36w4x01_read_mtp_reg(dsim->id,
		DDI_DEBUG_REG, &read_buf[0], DDI_DEBUG_REG_SIZE);
	if (ret) {
		pr_err("%s: read failed 0x9c register[%d]\n", __func__, ret);
		return -EIO;
	}

	for (i = 0; i < DDI_DEBUG_REG_SIZE; i++) {
		if (i == 0) {
			ret += scnprintf(show_buf, DEFAULT_BUFF_SIZE, "[%03d]", i);
			strncat(buf, show_buf, DEFAULT_BUFF_SIZE);
		} else if (!(i %8)) {
			ret += scnprintf(show_buf, DEFAULT_BUFF_SIZE, "\n[%03d]", i);
			strncat(buf, show_buf, DEFAULT_BUFF_SIZE);
		}
		ret += scnprintf(show_buf, DEFAULT_BUFF_SIZE, " 0x%02x,", read_buf[i]);
		strncat(buf, show_buf, DEFAULT_BUFF_SIZE);
	}

	strncat(buf, "\n", strlen("\n"));
	ret += strlen("\n");

	return ret;
}

void s6e36w4x01_write_ddi_debug(struct s6e36w4x01 *lcd);
static ssize_t s6e36w4x01_ddi_debug_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device	*dsim = lcd->dsim;
	int ret;
	bool enable;

	ret = strtobool(buf, &enable);
	if (ret) {
		dsim_err("%s:failed buffer read[%d]\n", __func__, ret);
		return ret;
	}

	lcd->enable_ddi_debug = enable;

	pr_info("%s:[%d]\n", __func__, lcd->enable_ddi_debug);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->enable_ddi_debug) {
		s6e36w4x01_testkey_enable(dsim->id);
		s6e36w4x01_write_ddi_debug(lcd);
		s6e36w4x01_testkey_disable(dsim->id);
	}

	return size;
}

#define 	MAX_MTP_READ_SIZE	0xff
static unsigned char mtp_read_data[MAX_MTP_READ_SIZE] = {0, };
static int read_data_length = 0;
static int read_data_addr;
static int read_data_offset;
extern int s6e36w4x01_read_mtp_reg(int id, u32 addr, char* buffer, u32 size);
static ssize_t s6e36w4x01_read_mtp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	char buffer[LDI_MTP4_LEN*8] = {0, };
	int i, ret = 0;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	ret = s6e36w4x01_read_mtp_reg(dsim->id,
		read_data_addr+read_data_offset, &mtp_read_data[0], read_data_length);
	if (ret) {
		pr_err("%s: read failed[%d]\n", __func__, ret);
		return -EIO;
	}

	for ( i = 0; i < read_data_length; i++) {
		if ((i !=0) && !(i%8)) {
			strcat(buf, "\n");
			ret += strlen("\n");
		}

		ret += scnprintf(buffer, LDI_MTP4_LEN, "0x%02x", mtp_read_data[i]);
		strcat(buf, buffer);

		if ( i < (read_data_length-1)) {
			strcat(buf, ", ");
			ret += strlen(", ");
		}
	}

	strcat(buf, "\n");
	ret += strlen("\n");

	pr_info("%s: length=%d\n", __func__, read_data_length);
	pr_info("%s\n", buf);

	return ret;
}

static ssize_t s6e36w4x01_read_mtp_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	u32 buff[3] = {0, }; //addr, offset, size
	int i, ret;

	ret = sscanf(buf, "%x %d %d", &buff[0], &buff[1], &buff[2]);
	if (ret != 3) {
		pr_err("%s: failed read params[%d]\n", __func__, ret);
		return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		if (buff[i] > MAX_MTP_READ_SIZE) {
			pr_err("%s:EINVAL %x %d %d\n", __func__, buff[0], buff[1], buff[2]);
			return -EINVAL;
		}
	}

	read_data_addr = buff[0];
	read_data_offset = buff[1];
	read_data_length = buff[2];

	pr_info("%s:addr[0x%x] offset[%d] length[%d]\n", __func__, buff[0], buff[1], buff[2]);

	return size;
}

extern void s6e36w4x01_write_mtp_reg(int id, char* buffer, u32 size);
static ssize_t s6e36w4x01_write_mtp_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	char *p, *arg = (char *)buf;
	u8 *tx_buf = NULL;
	int ret, val, len, i = 0;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	p = strsep(&arg, " ");
	if (sscanf(p, "%d", &len) != 1) {
		pr_err("%s:No size for mtp store\n", __func__);
		return -EINVAL;
	}

	if (len <= 0) {
		pr_err("%s:size is wrong[%d]\n", __func__, len);
		return -EINVAL;
	}

	pr_info("%s:size[%d]\n", __func__, len);

	tx_buf = kzalloc(len, GFP_KERNEL);
	if (!tx_buf) {
		pr_err("%s:Fail to kmalloc for tx_buf\n", __func__);
		goto out;
	}

	while ((p = strsep(&arg, " ")) != NULL && i < len) {
		ret = sscanf(p, "%02x", &val);
		if (ret != 1)
			pr_err("%s:fail to sscanf\n", __func__);
		tx_buf[i++] = val;
	}

	s6e36w4x01_write_mtp_reg(dsim->id, tx_buf, len);

	kfree(tx_buf);
	tx_buf = NULL;

out:

	return size;
}

static ssize_t s6e36w4x01_mtp_table_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	struct dim_data *dim_data = panel->dim_data;
	char buffer[DEFAULT_BUFF_SIZE];
	int i, ret = 0;

	if (dim_data == NULL) {
		dev_err(lcd->dev, "%s: dim_data is NULL\n", __func__);
		return -EIO;
	}

	ret += scnprintf(buffer, DEFAULT_BUFF_SIZE, "mtp[%4d %4d %4d]\n",
		  dim_data->vt_mtp[CI_RED], dim_data->vt_mtp[CI_GREEN], dim_data->vt_mtp[CI_BLUE]);
	strncat(buf, buffer, DEFAULT_BUFF_SIZE);

	ret += scnprintf(buffer, DEFAULT_BUFF_SIZE, "mtp[%4d %4d %4d]\n",
		  dim_data->v0_mtp[CI_RED], dim_data->v0_mtp[CI_GREEN], dim_data->v0_mtp[CI_BLUE]);
	strncat(buf, buffer, DEFAULT_BUFF_SIZE);

	for (i = 0; i < NUM_VREF; i++) {
		ret += scnprintf(buffer, DEFAULT_BUFF_SIZE, "mtp[%4d %4d %4d]\n",
			  dim_data->mtp[i][CI_RED], dim_data->mtp[i][CI_GREEN], dim_data->mtp[i][CI_BLUE]);
		strncat(buf, buffer, DEFAULT_BUFF_SIZE);
	}

	return ret;
}

static ssize_t s6e36w4x01_cr_map_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	char buffer[DEFAULT_BUFF_SIZE];
	int ret = 0;
	int i;

	for (i = 0; i <= MAX_BRIGHTNESS; i++) {
		ret += scnprintf(buffer, DEFAULT_BUFF_SIZE, " %3d,", candela_tbl[lcd->br_map[i]]);
		strncat(buf, buffer, DEFAULT_BUFF_SIZE);
		if ((i == 0) || !(i %10)) {
			strncat(buf, "\n", strlen("\n"));
			ret += strlen("\n");
		}
	}

	strncat(buf, "\n", strlen("\n"));
	ret += strlen("\n");
	pr_info("%s:%s\n", __func__, buf);

	return ret;
}

static ssize_t s6e36w4x01_br_map_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	char buffer[DEFAULT_BUFF_SIZE];
	int ret = 0;
	int i;

	for (i = 0; i <= MAX_BRIGHTNESS; i++) {
		ret += scnprintf(buffer, DEFAULT_BUFF_SIZE, " %3d,", lcd->br_map[i]);
		strncat(buf, buffer, DEFAULT_BUFF_SIZE);
		if ((i == 0) || !(i %10)) {
			strncat(buf, "\n", strlen("\n"));
			ret += strlen("\n");
		}
	}

	strncat(buf, "\n", strlen("\n"));
	ret += strlen("\n");
	pr_info("%s:%s\n", __func__, buf);

	return ret;
}


static ssize_t s6e36w4x01_cell_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	char temp[LDI_MTP4_MAX_PARA];
	char result_buff[CELL_ID_LEN+1];
	int ret;

	if (lcd->alpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	memset(&temp[0], 0x00, LDI_MTP4_MAX_PARA);
	ret = s6e36w4x01_read_mtp_reg(dsim->id, LDI_MTP4, &temp[0], LDI_MTP4_MAX_PARA);
	if (ret) {
		pr_err("%s:failed read LDI_MTP4[%d]\n", __func__, ret);
		return -EIO;
	}

	scnprintf(result_buff, CELL_ID_LEN, "%02x%02x%02x%02x%02x%02x%02x",
				temp[4], temp[5], temp[6], temp[7],
				temp[8], temp[9], temp[10]);
	strncat(buf, result_buff, CELL_ID_LEN);

	memset(&temp[0], 0x00, WHITE_COLOR_LEN);
	ret = s6e36w4x01_read_mtp_reg(dsim->id, LDI_WHITE_COLOR, &temp[0], WHITE_COLOR_LEN);
	if (ret) {
		pr_err("%s:failed read LDI_WHITE_COLOR[%d]\n", __func__, ret);
		return -EIO;
	}

	scnprintf(result_buff, CELL_ID_LEN, "%02x%02x%02x%02x\n",
				temp[0], temp[1], temp[2], temp[3]);
	strncat(buf, result_buff, CELL_ID_LEN);

	pr_info("%s:[%s]", __func__, buf);

	return strlen(buf);
}

static ssize_t s6e36w4x01_svc_chipid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	char buff[SVC_CHIP_ID_LEN];
	int ret;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	ret = s6e36w4x01_read_mtp_reg(dsim->id, SVC_CHIP_ID, &buff[0], SVC_CHIP_ID_LEN);
	if (ret) {
		pr_err("%s:failed read SVC_CHIP_ID[%d]\n", __func__, ret);
		return -EIO;
	}

	pr_info("%s:%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
			__func__, buff[5], buff[6], buff[7], buff[8], buff[9], buff[10],
			buff[11], buff[12], buff[13], buff[14], buff[15],
			buff[16], buff[17], buff[18], buff[19], buff[20]);

	if (buff[5] == 0) {
		ret = scnprintf(buf, PAGE_SIZE, "\n");
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
				buff[5], buff[6], buff[7], buff[8], buff[9], buff[10], buff[11],
				buff[12], buff[13], buff[14], buff[15], buff[16], buff[17],
				buff[18], buff[19], buff[20]);
		pr_info("%s:%s", __func__, buf);
	}

	return ret;
}

static ssize_t s6e36w4x01_hbm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->hbm_on ? "on" : "off");
}

static ssize_t s6e36w4x01_hbm_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;

	if (!strncmp(buf, "on", 2))
		lcd->hbm_on = 1;
	else if (!strncmp(buf, "off", 3))
		lcd->hbm_on = 0;
	else {
		dev_warn(dev, "invalid comman (use on or off)d.\n");
		goto out;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d][%s]\n", __func__,
				lcd->power, lcd->hbm_on ? "ON" : "OFF");
		goto out;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled[%s]\n", __func__, lcd->hbm_on ? "ON" : "OFF");
		goto out;
	}

	if (lcd->mcd_on) {
		pr_info("%s:mcd enabled[%s]\n", __func__, lcd->hbm_on ? "ON" : "OFF");
		goto out;
	}

	if (lcd->hbm_on) {
		ret = s6e36w4x01_hbm_on(lcd);
		if (ret) {
			pr_err("%s:failed HBM ON[%d]\n", __func__, ret);
			return -EIO;
		}
	} else {
		ret = s6e36w4x01_hbm_off(lcd);
		if (ret) {
			pr_err("%s:failed HBM OFF[%d]\n", __func__, ret);
			return -EIO;
		}
	}

	dev_info(lcd->dev, "HBM %s.\n", lcd->hbm_on ? "ON" : "OFF");

out:
	return size;
}

#define 	ELVSS_MIN_TEMP	-127
#define 	ELVSS_MAX_TEMP	127
extern unsigned char s6e36w4x01_get_offset_comp(void);
extern enum temp_range_t s6e36w4x01_get_temp_stage(struct s6e36w4x01 *lcd, int temp);
extern void s6e36w4x01_set_offset_comp(struct s6e36w4x01 *lcd, int temperature);
extern int s6e36w4x01_write_elvss_temp(int id);
static ssize_t s6e36w4x01_temperature_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "stage[%d] temp[%d] reg[0x%02x]\n",
		lcd->temp_stage, lcd->temperature, s6e36w4x01_get_offset_comp());
}

static ssize_t s6e36w4x01_temperature_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	long value;
	int ret;

	ret = kstrtol(buf, 0, &value);
	if (ret) {
		dsim_err("%s:failed buffer read[%d]\n", __func__, ret);
		return ret;
	}

	if (value > ELVSS_MAX_TEMP)
		value = ELVSS_MAX_TEMP;
	else if (value < ELVSS_MIN_TEMP)
		value = ELVSS_MIN_TEMP;

	lcd->temperature = (int)value;
	lcd->temp_stage = s6e36w4x01_get_temp_stage(lcd, lcd->temperature);

	pr_info("%s:temp[%lu][%d]\n", __func__, value, lcd->temp_stage);

	s6e36w4x01_set_offset_comp(lcd, (int)value);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_err("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	s6e36w4x01_write_elvss_temp(dsim->id);

	return size;
}

static ssize_t s6e36w4x01_octa_chip_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	int ret;
	char temp[DEFAULT_BUFF_SIZE];

	if (lcd->alpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	memset(&panel->chip[0], 0x00, LDI_CHIP_LEN);

	ret = s6e36w4x01_read_mtp_reg(dsim->id, LDI_CHIP_ID, &panel->chip[0], LDI_CHIP_LEN);
	if (ret) {
		pr_err("%s:failed read LDI_CHIP_ID[%d]\n", __func__, ret);
		return -EIO;
	}

	scnprintf(temp, DEFAULT_BUFF_SIZE, "%02x%02x%02x%02x%02x\n",
				panel->chip[0], panel->chip[1], panel->chip[2],
				panel->chip[3], panel->chip[4]);
	strncat(buf, temp, DEFAULT_BUFF_SIZE);

	pr_info("%s: %s\n", __func__, temp);

	return strlen(buf);
}

static ssize_t s6e36w4x01_lcd_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	char temp[LDI_ID_LEN];
	int ret;

	if (lcd->alpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	memset(&temp[0], 0, LDI_ID_LEN);
	ret = s6e36w4x01_read_mtp_reg(dsim->id, LDI_ID_REG, &temp[0], LDI_ID_LEN);
	if (ret) {
		pr_err("%s:failed read LDI_ID_REG[%d]\n", __func__, ret);
		return -EIO;
	}

	scnprintf(buf, PAGE_SIZE, "SDC_%02x%02x%02x\n", temp[0], temp[1], temp[2]);
	pr_info("%s:[%s]", __func__, buf);

	return strlen(buf);
}

static ssize_t s6e36w4x01_lcd_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;

	s6e36w4x01_show_state(dsim);

	return 0;
}

static ssize_t s6e36w4x01_br_level_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->br_level);
}

extern int s6e36w4x01_gamma_ctrl(struct s6e36w4x01 *lcd, u32 backlightlevel);
static ssize_t s6e36w4x01_br_level_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	unsigned char level;
	int ret;

	if (lcd->power > FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "%s:control before lcd enable\n", __func__);
		return -EPERM;
	}

	ret = kstrtou8(buf, 0, &level);
	if (ret) {
		pr_err("%s:Failed to get br level\n", __func__);
		return -EINVAL;
	}

	if (level >= MAX_BR_INFO)
		level = (MAX_BR_INFO -1);

	lcd->br_level = level;

	ret = s6e36w4x01_gamma_ctrl(lcd, lcd->br_level);
	if (ret) {
		pr_err("%s:Failed s6e36w4x01_gamma_ctrl\n", __func__);
		return -EIO;
	}

	pr_info("%s:level[%d]\n", __func__, lcd->br_level);

	return size;
}

#ifdef DISPLAY_DQA
static ssize_t s6e36w4x01_display_model_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct decon_lcd *lcd_info = &dsim->lcd_info;

	return scnprintf(buf, PAGE_SIZE, "SDC_%s\n", lcd_info->model_name);;
}

static ssize_t s6e36w4x01_display_chipid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	char temp[DEFAULT_BUFF_SIZE];

	scnprintf(temp, DEFAULT_BUFF_SIZE, "%02x%02x%02x%02x%02x\n",
		panel->chip[0], panel->chip[1], panel->chip[2],
		panel->chip[3], panel->chip[4]);
	strncat(buf, temp, DEFAULT_BUFF_SIZE);

	pr_info("%s: %s\n", __func__, temp);

	return strlen(buf);
}

static ssize_t s6e36w4x01_product_date_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	int year, month, day, hour, minute, second, msecond;

	year = (panel->date[0] >> 4) + 1;
	month = panel->date[0] & 0x0f;
	day = panel->date[1] & 0x0f;
	hour = panel->date[2] & 0x0f;
	minute = panel->date[3];
	second = panel->date[4];
	msecond = ((panel->date[5] & 0x0f) << 8) + panel->date[6];

	return scnprintf(buf, PAGE_SIZE, "%d-%02d-%02d %02d:%02d:%02d:%04d\n", (2010+year), month, day, hour, minute, second, msecond);
}

static ssize_t s6e36w4x01_white_x_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;

	return scnprintf(buf, PAGE_SIZE, "%d\n", panel->coordinate[0]);
}

static ssize_t s6e36w4x01_white_y_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;

	return scnprintf(buf, PAGE_SIZE, "%d\n", panel->coordinate[1]);
}

static ssize_t s6e36w4x01_lcdm_id1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;

	return scnprintf(buf, PAGE_SIZE, "%d\n", panel->id[0]);
}

static ssize_t s6e36w4x01_lcdm_id2_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;

	return scnprintf(buf, PAGE_SIZE, "%d\n", panel->id[1]);
}

static ssize_t s6e36w4x01_lcdm_id3_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;

	return scnprintf(buf, PAGE_SIZE, "%d\n", panel->id[2]);
}

static ssize_t s6e36w4x01_pndsie_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dsim->comm_err_cnt);
	dsim->comm_err_cnt = 0;

	return ret;
}

static ssize_t s6e36w4x01_qct_no_te_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dsim->decon_timeout_cnt);
	dsim->decon_timeout_cnt = 0;
	dsim_store_timeout_count(dsim);

	return ret;
}

static ssize_t s6e36w4x01_lbhd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->hbm_total_time);
	lcd->hbm_total_time = 0;

	return ret;
}

static ssize_t s6e36w4x01_lod_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->disp_total_time);
	lcd->disp_total_time = 0;

	return ret;
}

static ssize_t s6e36w4x01_daod_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->aod_total_time);
	lcd->aod_total_time = 0;

	return ret;
}

static ssize_t s6e36w4x01_dahl_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->aod_high_time);
	lcd->aod_high_time = 0;

	return ret;
}

static ssize_t s6e36w4x01_dall_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->aod_low_time);
	lcd->aod_low_time = 0;

	return ret;
}

static ssize_t s6e36w4x01_locnt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->disp_cnt);
	lcd->disp_cnt = 0;

	return ret;
}
#endif

static ssize_t s6e36w4x01_lpm_br_max_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	pr_info("%s: %d\n", __func__, lcd->lpm_br_max);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_max);
}

static ssize_t s6e36w4x01_lpm_br_normal_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	pr_info("%s: %d\n", __func__, lcd->lpm_br_normal);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_normal);
}

static ssize_t s6e36w4x01_lpm_br_min_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	pr_info("%s: %d\n", __func__, lcd->lpm_br_min);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_min);
}

static ssize_t s6e36w4x01_lpm_br_charging_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

        pr_info("%s: %d\n", __func__, lcd->lpm_br_charging);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_charging);
}
#ifdef CONFIG_TIZEN_SEC_KERNEL_ENG
static ssize_t s6e36w4x01_min_lpm_level_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);

	pr_info("%s:%d/%d\n", __func__, lcd->min_lpm_lv, LPM_BR_MAX-1);

	return scnprintf(buf, PAGE_SIZE, "%d/%d\n", lcd->min_lpm_lv, LPM_BR_MAX-1);
}

static ssize_t s6e36w4x01_min_lpm_level_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	unsigned char level;
	int ret;

	ret = kstrtou8(buf, 0, &level);
	if (ret) {
		pr_err("%s:Failed to get lpm level\n", __func__);
		return -EINVAL;
	}

	if (level >= LPM_BR_MAX)
		level = LPM_BR_HIGH;

	lcd->min_lpm_lv = level;

	pr_info("%s:[%d]\n", __func__, lcd->min_lpm_lv);

	return size;
}
#endif
extern ssize_t s6e36w4x01_dimming_table_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size);
extern ssize_t s6e36w4x01_dimming_table_show(struct device *dev,
				struct device_attribute *attr, char *buf);
extern ssize_t s6e36w4x01_sc_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size);
extern ssize_t s6e36w4x01_sc_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static struct device_attribute s6e36w4x01_dev_attrs[] = {
	__ATTR(resolution, S_IRUGO | S_IWUSR, s6e36w4x01_resolution_show,
					s6e36w4x01_resolution_store),
	__ATTR(esd, S_IRUGO | S_IWUSR, s6e36w4x01_esd_show,
					s6e36w4x01_esd_store),
	__ATTR(sc, S_IRUGO | S_IWUSR, s6e36w4x01_sc_show, s6e36w4x01_sc_store),
	__ATTR(ddi_debug, S_IRUGO | S_IWUSR, s6e36w4x01_ddi_debug_show,
					s6e36w4x01_ddi_debug_store),
	__ATTR(hop, S_IRUGO | S_IWUSR, s6e36w4x01_hop_show,
					s6e36w4x01_hop_store),
	__ATTR(alpm, S_IRUGO | S_IWUSR, s6e36w4x01_alpm_show,
					s6e36w4x01_alpm_store),
	__ATTR(hlpm, S_IRUGO | S_IWUSR, s6e36w4x01_hlpm_show,
					s6e36w4x01_hlpm_store),
	__ATTR(cell_id, S_IRUGO, s6e36w4x01_cell_id_show, NULL),
	__ATTR(SVC_OCTA_CHIPID, S_IRUGO, s6e36w4x01_svc_chipid_show, NULL),
	__ATTR(hbm, S_IRUGO | S_IWUSR, s6e36w4x01_hbm_show,
					s6e36w4x01_hbm_store),
	__ATTR(temperature, S_IRUGO | S_IWUSR, s6e36w4x01_temperature_show,
					s6e36w4x01_temperature_store),
	__ATTR(chip_id, S_IRUGO, s6e36w4x01_octa_chip_id_show, NULL),
	__ATTR(cr_map, S_IRUGO, s6e36w4x01_cr_map_show, NULL),
	__ATTR(br_map, S_IRUGO, s6e36w4x01_br_map_show, NULL),
	__ATTR(dim_mtp, S_IRUGO, s6e36w4x01_mtp_table_show, NULL),
	__ATTR(dim_table, S_IRUGO | S_IWUSR, s6e36w4x01_dimming_table_show,
					s6e36w4x01_dimming_table_store),
	__ATTR(read_mtp, S_IRUGO | S_IWUSR, s6e36w4x01_read_mtp_show,
					s6e36w4x01_read_mtp_store),
	__ATTR(write_mtp, S_IRUGO | S_IWUSR, s6e36w4x01_read_mtp_show,
					s6e36w4x01_write_mtp_store),
	__ATTR(lcd_type, S_IRUGO , s6e36w4x01_lcd_type_show, NULL),
	__ATTR(mcd_test, S_IRUGO | S_IWUSR, s6e36w4x01_mcd_show,
					s6e36w4x01_mcd_store),
	__ATTR(circle_mask, S_IRUGO | S_IWUSR, s6e36w4x01_circle_mask_show,
					s6e36w4x01_circle_mask_store),
	__ATTR(enable_fd, S_IRUGO | S_IWUSR, s6e36w4x01_enable_fd_show,
					s6e36w4x01_enable_fd_store),
	__ATTR(state, S_IRUGO, s6e36w4x01_lcd_state_show, NULL),
	__ATTR(vbias, S_IRUGO | S_IWUSR, s6e36w4x01_vbias_show,
					s6e36w4x01_vbias_store),
	__ATTR(br_level, S_IRUGO | S_IWUSR, s6e36w4x01_br_level_show,
					s6e36w4x01_br_level_store),
#ifdef DISPLAY_DQA
	__ATTR(disp_model, S_IRUGO , s6e36w4x01_display_model_show, NULL),
	__ATTR(disp_chipid, S_IRUGO, s6e36w4x01_display_chipid_show, NULL),
	__ATTR(prod_date, S_IRUGO , s6e36w4x01_product_date_show, NULL),
	__ATTR(white_x, S_IRUGO , s6e36w4x01_white_x_show, NULL),
	__ATTR(white_y, S_IRUGO , s6e36w4x01_white_y_show, NULL),
	__ATTR(lcdm_id1, S_IRUGO , s6e36w4x01_lcdm_id1_show, NULL),
	__ATTR(lcdm_id2, S_IRUGO , s6e36w4x01_lcdm_id2_show, NULL),
	__ATTR(lcdm_id3, S_IRUGO , s6e36w4x01_lcdm_id3_show, NULL),
	__ATTR(pndsie, S_IRUGO , s6e36w4x01_pndsie_show, NULL),
	__ATTR(qct_no_te, S_IRUGO, s6e36w4x01_qct_no_te_show, NULL),
	__ATTR(daod, S_IRUGO, s6e36w4x01_daod_show, NULL),
	__ATTR(dahl, S_IRUGO, s6e36w4x01_dahl_show, NULL),
	__ATTR(dall, S_IRUGO, s6e36w4x01_dall_show, NULL),
	__ATTR(lbhd, S_IRUGO , s6e36w4x01_lbhd_show, NULL),
	__ATTR(lod, S_IRUGO , s6e36w4x01_lod_show, NULL),
	__ATTR(locnt, S_IRUGO , s6e36w4x01_locnt_show, NULL),
#endif
	__ATTR(lpm_br_max, S_IRUGO , s6e36w4x01_lpm_br_max_show, NULL),
	__ATTR(lpm_br_normal, S_IRUGO , s6e36w4x01_lpm_br_normal_show, NULL),
	__ATTR(lpm_br_min, S_IRUGO , s6e36w4x01_lpm_br_min_show, NULL),
	__ATTR(lpm_br_charging, S_IRUGO, s6e36w4x01_lpm_br_charging_show, NULL),
#ifdef CONFIG_TIZEN_SEC_KERNEL_ENG
	__ATTR(min_lpm_lv, S_IRUGO | S_IWUSR, s6e36w4x01_min_lpm_level_show,
							s6e36w4x01_min_lpm_level_store),
#endif
};

extern void s6e36w4x01_mdnie_set(u32 id, enum mdnie_scenario scenario);
extern void s6e36w4x01_mdnie_outdoor_set(u32 id, enum mdnie_outdoor on);
static ssize_t s6e36w4x01_scenario_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct mdnie_lite_device *mdnie = lcd->mdnie;

	return scnprintf(buf, PAGE_SIZE, "%d\n", mdnie->scenario);
}

static ssize_t s6e36w4x01_scenario_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct mdnie_lite_device *mdnie = lcd->mdnie;
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 0, &value);
	if (ret) {
		dsim_err("%s:failed buffer read[%d]\n", __func__, ret);
		return ret;
	}

	pr_info("%s:cur[%d] new[%lu]\n", __func__, mdnie->scenario, value);

	if (mdnie->scenario == value)
		return size;

	mdnie->scenario = value;

	if (lcd->alpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return size;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return size;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return size;
	}

	s6e36w4x01_mdnie_set(dsim->id, mdnie->scenario);

	schedule_delayed_work(&lcd->debug_dwork,
			msecs_to_jiffies(DEBUG_READ_DELAY));

	return size;
}

static ssize_t s6e36w4x01_outdoor_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct mdnie_lite_device *mdnie = lcd->mdnie;

	return scnprintf(buf, PAGE_SIZE, "%d\n", mdnie->outdoor);
}

static ssize_t s6e36w4x01_outdoor_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct mdnie_lite_device *mdnie = lcd->mdnie;
	bool value;
	int ret;

	ret = strtobool(buf, &value);
	if (ret) {
		dsim_err("%s:failed buffer read[%d]\n", __func__, ret);
		return ret;
	}

	mdnie->outdoor = value;

	if (lcd->alpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return size;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return size;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_info("%s:aod enabled\n", __func__);
		return size;
	}

	s6e36w4x01_mdnie_outdoor_set(dsim->id, mdnie->outdoor);

	return size;
}

static struct device_attribute mdnie_attrs[] = {
	__ATTR(scenario, 0664, s6e36w4x01_scenario_show, s6e36w4x01_scenario_store),
	__ATTR(outdoor, 0664, s6e36w4x01_outdoor_show, s6e36w4x01_outdoor_store),
};

void s6e36w4x01_mdnie_lite_init(struct s6e36w4x01 *lcd)
{
	static struct class *mdnie_class;
	struct mdnie_lite_device *mdnie;
	int i;

	mdnie = kzalloc(sizeof(struct mdnie_lite_device), GFP_KERNEL);
	if (!mdnie) {
		pr_err("failed to allocate mdnie object.\n");
		return;
	}

	mdnie_class = class_create(THIS_MODULE, "extension");
	if (IS_ERR(mdnie_class)) {
		pr_err("Failed to create class(mdnie)!\n");
		goto err_free_mdnie;
	}

	mdnie->dev = device_create(mdnie_class, NULL, 0, NULL, "mdnie");
	if (IS_ERR(&mdnie->dev)) {
		pr_err("Failed to create device(mdnie)!\n");
		goto err_free_mdnie;
	}

	for (i = 0; i < ARRAY_SIZE(mdnie_attrs); i++) {
		if (device_create_file(mdnie->dev, &mdnie_attrs[i]) < 0)
			pr_err("Failed to create device file(%s)!\n",
				mdnie_attrs[i].attr.name);
	}

	mdnie->scenario = SCENARIO_UI;
	lcd->mdnie = mdnie;

	dev_set_drvdata(mdnie->dev, lcd);

	return;

err_free_mdnie:
	kfree(mdnie);
}

static int s6e36w4x01_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

void s6e36w4x01_mdnie_restore(struct s6e36w4x01 *lcd, bool aod_state)
{
	struct mdnie_lite_device *mdnie = lcd->mdnie;
	struct dsim_device *dsim = lcd->dsim;

	if ((mdnie->scenario == SCENARIO_UI) ||
		(mdnie->scenario == SCENARIO_GRAY))
		return;

	if (aod_state) {
		switch (mdnie->scenario) {
			case SCENARIO_GRAY:
			case SCENARIO_GRAY_NEGATIVE:
				s6e36w4x01_mdnie_set(dsim->id, SCENARIO_GRAY);
				break;
			case SCENARIO_UI:
			case SCENARIO_GALLERY:
			case SCENARIO_VIDEO:
			case SCENARIO_VTCALL:
			case SCENARIO_CAMERA:
			case SCENARIO_BROWSER:
			case SCENARIO_NEGATIVE:
			case SCENARIO_EMAIL:
			case SCENARIO_EBOOK:
			case SCENARIO_CURTAIN:
			default:
				s6e36w4x01_mdnie_set(dsim->id, SCENARIO_UI);
				break;
		}
		panel_vsync_wait(PANEL_30HZ_1FRAME_MSEC);
	} else {
		panel_vsync_wait(PANEL_30HZ_1FRAME_MSEC);
		s6e36w4x01_mdnie_set(dsim->id, mdnie->scenario);
	}
}

static int s6e36w4x01_update_brightness(struct s6e36w4x01 *lcd)
{
	struct backlight_device *bd = lcd->bd;
	int brightness = bd->props.brightness, ret = 0;

	pr_info("%s[%d]lv[%d]pwr[%d]aod[%d]hbm[%d]\n",
		__func__, brightness, lcd->br_map[brightness],
		lcd->power, lcd->aod_mode, lcd->hbm_on);

	if (lcd->hbm_on)
		ret = s6e36w4x01_hbm_on(lcd);
	else {
		lcd->br_level = lcd->br_map[brightness];
		ret = s6e36w4x01_gamma_ctrl(lcd, lcd->br_level);
	}

	return ret;
}

static int s6e36w4x01_set_brightness(struct backlight_device *bd)
{
	struct dsim_device *dsim;
	struct s6e36w4x01 *lcd = bl_get_data(bd);
	int brightness = bd->props.brightness, ret = 0;

	if (lcd == NULL) {
		pr_err("%s: LCD is NULL\n", __func__);
		return -ENODEV;
	}

	dsim = lcd->dsim;
	if (dsim == NULL) {
		pr_err("%s: dsim is NULL\n", __func__);
		return -ENODEV;
	}

	if (brightness < MIN_BRIGHTNESS || brightness > MAX_BRIGHTNESS) {
		pr_err("%s:Brightness should be in the range of %d ~ %d\n",
			__func__, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
		return -EINVAL;
	}

	if (brightness == lcd->last_brightness) {
		pr_debug("%s:Brightness is same with last_brightness[%d]\n",
				__func__, brightness);
		return 0;
	}

	if (lcd->br_map[brightness] == lcd->br_map[lcd->last_brightness]) {
		pr_debug("%s:br_map is same with last br_map[%d]\n",
				__func__, lcd->br_map[brightness]);
		lcd->last_brightness = brightness;
		return 0;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_debug("%s:invalid power[%d]\n", __func__, lcd->power);
		return 0;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_err("%s:aod enabled\n", __func__);
		return 0;
	}

	if (lcd->ub_con_connected == false) {
		pr_info("%s:ub_con disconnected\n", __func__);
		return 0;
	}

	ret = s6e36w4x01_update_brightness(lcd);
	if (ret) {
		pr_err("%s:failed change_brightness\n", __func__);
		goto out;
	}

	lcd->last_brightness = brightness;

	panel_vsync_wait(PANEL_60HZ_1FRAME_MSEC);

out:
	return ret;
}

static int s6e36w4x01_get_power(struct lcd_device *ld)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(&ld->dev);
	struct dsim_device *dsim = lcd->dsim;
	unsigned int power = lcd->power;

#ifdef CONFIG_SEC_UB_DISCONNECT
	if ((power == FB_BLANK_POWERDOWN) &&
		(!dsim_ub_get_conn_state(lcd->dsim))) {
			power = (FB_BLANK_POWERDOWN + 1);
	}
#endif

	if ((power == FB_BLANK_POWERDOWN) &&
		(dsim->decon_recov_working == true)) {
		pr_info("%s:decon_recov_working[%d] ret[%d]\n",
			__func__, power, FB_BLANK_UNBLANK);
		return FB_BLANK_UNBLANK;
	}

	if (power == (FB_BLANK_POWERDOWN + 1))
		pr_info("%s[%d]\n", __func__, power);
	else
		pr_debug("%s[%d]\n", __func__, power);

	return power;
}

static int s6e36w4x01_set_power(struct lcd_device *ld, int power)
{
	struct s6e36w4x01 *lcd = dev_get_drvdata(&ld->dev);

	if (power > FB_BLANK_POWERDOWN) {
		pr_err("%s: invalid power state.[%d]\n", __func__, power);
		return -EINVAL;
	}

	lcd->power = power;
	wristup_booster_set_lcd_power(power);

	pr_info("%s[%s]\n", __func__, get_power_state_str(lcd->power));

	return 0;
}

static irqreturn_t s6e36w4x01_esd_interrupt(int irq, void *dev_id)
{
	struct s6e36w4x01 *lcd = dev_id;
	struct dsim_device *dsim = lcd->dsim;
	struct dsim_resources *res = &dsim->res;
	int ret;

	if (lcd->power > FB_BLANK_NORMAL) {
		dev_dbg(lcd->dev, "%s:invalid power[%d]\n",
			__func__, lcd->power);
		return IRQ_HANDLED;
	}

	ret = gpio_get_value(res->err_fg);
	if (!ret) {
		dev_dbg(lcd->dev, "%s: Invalid interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	cancel_delayed_work(&lcd->esd_dwork);
	schedule_delayed_work(&lcd->esd_dwork, 0);

	return IRQ_HANDLED;
}

static struct lcd_ops s6e36w4x01_lcd_ops = {
	.get_power = s6e36w4x01_get_power,
	.set_power = s6e36w4x01_set_power,
};


static const struct backlight_ops s6e36w4x01_backlight_ops = {
	.get_brightness = s6e36w4x01_get_brightness,
	.update_status = s6e36w4x01_set_brightness,
};

#ifdef CONFIG_SLEEP_MONITOR
int s6e36w4x01_sleep_monitor_cb(void *priv, unsigned int *raw_val,
		int check_level, int caller_type)
{
	struct s6e36w4x01 *lcd = priv;
	int state = DEVICE_UNKNOWN;
	int mask = (1 << SLEEP_MONITOR_DEVICE_BIT_WIDTH) - 1;
	unsigned int cnt = 0;
	int ret;

	switch (lcd->power) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
		state = DEVICE_ON_ACTIVE1;
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		state = DEVICE_ON_LOW_POWER;
		break;
	case FB_BLANK_POWERDOWN:
		state = DEVICE_POWER_OFF;
		break;
	}

	switch (caller_type) {
	case SLEEP_MONITOR_CALL_SUSPEND:
	case SLEEP_MONITOR_CALL_RESUME:
		cnt = lcd->act_cnt;
		if (state == DEVICE_ON_LOW_POWER)
			cnt++;
		break;
	case SLEEP_MONITOR_CALL_ETC:
		break;
	default:
		break;
	}

	*raw_val = cnt | state << 24;

	/* panel on count 1~15*/
	if (cnt > mask)
		ret = mask;
	else
		ret = cnt;

	pr_info("%s: caller[%d], dpms[%d] panel on[%d], raw[0x%x]\n",
			__func__, caller_type, lcd->power, ret, *raw_val);

	lcd->act_cnt = 0;

	return ret;
}

static struct sleep_monitor_ops s6e36w4x01_sleep_monitor_ops = {
	.read_cb_func = s6e36w4x01_sleep_monitor_cb,
};
#endif

int s6e36w4x01_afpc_get_panel(struct dsim_device *dsim, struct afpc_panel_v2 *afpc_panel)
{
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);
	unsigned char	chip[LDI_CHIP_LEN];
	int ret = 0;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		ret = -EPERM;
	}

	memset(&chip, 0x00, LDI_CHIP_LEN);
	ret = s6e36w4x01_read_mtp_reg(dsim->id, LDI_CHIP_ID, &chip[0], LDI_CHIP_LEN);
	if (ret) {
		pr_err("%s:failed read LDI_CHIP_ID[%d]\n", __func__, ret);
		return -EIO;
	}

	scnprintf(afpc_panel->id, PAGE_SIZE, "%02x%02x%02x%02x%02x\n",
					chip[0], chip[1], chip[2], chip[3], chip[4]);

	pr_info("%s: %s\n", __func__, afpc_panel->id);

	return ret;
}

extern void s6e36w4x01_sc_afpc_dwork(struct work_struct *work);
extern int s6e36w4x01_sc_alloc(struct s6e36w4x01 *lcd);
extern int s6e36w4x01_init_dimming(struct dsim_device *dsim);
static int s6e36w4x01_probe(struct dsim_device *dsim)
{
	struct dsim_resources *res = &dsim->res;
	struct s6e36w4x01 *lcd;
	int ret, i;

	pr_info("%s\n", __func__);

	if (get_panel_id() == -1) {
		pr_err("%s: No lcd attached!\n", __func__);
		return -ENODEV;
	}

	lcd = devm_kzalloc(dsim->dev,
			sizeof(struct s6e36w4x01), GFP_KERNEL);
	if (lcd == NULL) {
		pr_err("%s: failed to allocate s6e36w4x01 structure.\n", __func__);
		return -ENOMEM;
	}

	lcd->dev = dsim->dev;
	lcd->dsim = dsim;
	g_s6e36w4x01 = lcd;

	mutex_init(&lcd->mipi_lock);
	mutex_init(&lcd->afpc_lock);

	lcd->bd = backlight_device_register(BACKLIGHT_DEV_NAME,
		lcd->dev, lcd, &s6e36w4x01_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		pr_err("%s: failed to register backlight device[%d]\n",
			__func__, (int)PTR_ERR(lcd->bd));
		ret = PTR_ERR(lcd->bd);
		goto err_bd;
	}
	lcd->bd->props.max_brightness = MAX_BRIGHTNESS;
	lcd->bd->props.brightness = DEFAULT_BRIGHTNESS;
	lcd->last_brightness = -1;
	lcd->enable_hop = true;
	lcd->ub_con_connected = true;
	lcd->enable_circle_mask = true;
	lcd->min_lpm_lv = LPM_BR_LOW;
	lcd->temp_stage = TEMP_RANGE_MAX;
	lcd->aod_refresh = AOD_PANEL_REFRESH_RATE_LOW;
	lcd->aod_refresh_status = AOD_PANEL_REFRESH_RATE_HIGH;
#ifdef CONFIG_SEC_FACTORY
	lcd->enable_fd = true;
	lcd->enable_circle_mask = false;
#endif

	dsim->ld = lcd_device_register(LCD_DEV_NAME,
			lcd->dev, lcd, &s6e36w4x01_lcd_ops);
	if (IS_ERR(dsim->ld)) {
		pr_err("%s: failed to register lcd ops[%d]\n",
			__func__, (int)PTR_ERR(dsim->ld));
		ret = PTR_ERR(lcd->bd);
		goto err_ld;
	}
	lcd->ld = dsim->ld;

	ret = s6e36w4x01_init_dimming(dsim);
	if (ret) {
		pr_err("%s : failed to init_dimming[%d]\n", __func__, ret);
		goto err_create_file;
	}

	lcd->esd_class = class_create(THIS_MODULE, "lcd_event");
	if (IS_ERR(lcd->esd_class)) {
		dev_err(lcd->dev, "%s: Failed to create esd_class[%d]\n",
			__func__, (int)PTR_ERR(lcd->esd_class));
		ret = PTR_ERR(lcd->esd_class);
		goto err_esd_class;
	}

	lcd->esd_dev = device_create(lcd->esd_class, NULL, 0, NULL, "esd");
	if (IS_ERR(lcd->esd_dev)) {
		dev_err(lcd->dev, "Failed to create esd_dev\n");
		goto err_esd_dev;
	}

	if (res->err_fg > 0) {
		ret = gpio_request_one(res->err_fg, GPIOF_IN, "err_fg");
		if (ret < 0) {
			dev_err(lcd->dev, "failed to get err_fg GPIO\n");
			goto err_esd_dev;
		}
		INIT_DELAYED_WORK(&lcd->esd_dwork, s6e36w4x01_esd_dwork);
		lcd->esd_irq = gpio_to_irq(res->err_fg);
		dev_dbg(lcd->dev, "esd_irq_num [%d]\n", lcd->esd_irq);

		ret = request_threaded_irq(lcd->esd_irq, NULL, s6e36w4x01_esd_interrupt,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT , "err_fg", lcd);
		if (ret < 0) {
			dev_err(lcd->dev, "failed to request det irq.\n");
			goto err_err_fg;
		}
	}

	for (i = 0; i < ARRAY_SIZE(s6e36w4x01_dev_attrs); i++) {
		ret = device_create_file(&lcd->ld->dev, &s6e36w4x01_dev_attrs[i]);
		if (ret < 0) {
			dev_err(&lcd->ld->dev, "failed to add ld dev sysfs entries\n");
			for (i--; i >= 0; i--)
				device_remove_file(&lcd->ld->dev, &s6e36w4x01_dev_attrs[i]);
			goto err_create_file;
		}
	}

	ret = sysfs_create_link(&dsim->sec_dev->kobj, &lcd->ld->dev.kobj, "panel");
	if (ret < 0) {
		dev_err(lcd->dev, "%s: Failed to create panel symbolic link %d\n", __func__, ret);
		goto err_create_file;
	}
	ret = sysfs_create_link(&dsim->sec_dev->kobj, &lcd->bd->dev.kobj, "backlight");
	if (ret < 0) {
		dev_err(lcd->dev, "%s: Failed to create backlight symbolic link %d\n", __func__, ret);
		goto err_create_file;
	}

	s6e36w4x01_mdnie_lite_init(lcd);

	ret = s6e36w4x01_sc_alloc(lcd);
	if (ret) {
		pr_err("%s : failed to sc alloc[%d]\n", __func__, ret);
		goto err_create_file;
	}

#ifdef CONFIG_SLEEP_MONITOR
	sleep_monitor_register_ops(lcd, &s6e36w4x01_sleep_monitor_ops,
		SLEEP_MONITOR_LCD);
	lcd->act_cnt = 1;
#endif
	INIT_DELAYED_WORK(&lcd->debug_dwork, s6e36w4x01_debug_dwork);
	INIT_DELAYED_WORK(&lcd->afpc_dwork, s6e36w4x01_sc_afpc_dwork);
	schedule_delayed_work(&lcd->debug_dwork,
			msecs_to_jiffies(DEBUG_READ_DELAY));

#ifdef CONFIG_SEC_FACTORY
	if (lcd->enable_fd)
		s6e36w4x01_enable_fd(lcd);
#endif

	pr_info("%s done\n", __func__);

	return 0;

err_create_file:
err_err_fg:
	device_destroy(lcd->esd_class, lcd->esd_dev->devt);
err_esd_dev:
	class_destroy(lcd->esd_class);
err_esd_class:
	lcd_device_unregister(lcd->ld);
err_ld:
	backlight_device_unregister(lcd->bd);
err_bd:
	devm_kfree(dsim->dev, lcd->br_map);
	mutex_destroy(&lcd->afpc_lock);
	mutex_destroy(&lcd->mipi_lock);
	devm_kfree(dsim->dev, lcd);
	return ret;
}

extern void s6e36w4x01_init_ctrl(int id, struct s6e36w4x01 *lcd);
static int s6e36w4x01_init(struct dsim_device *dsim)
{
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);
	struct mdnie_lite_device *mdnie = lcd->mdnie;

	s6e36w4x01_init_ctrl(dsim->id, lcd);

	if (mdnie->outdoor == OUTDOOR_ON)
		s6e36w4x01_mdnie_outdoor_set(dsim->id, mdnie->outdoor);
	else
		s6e36w4x01_mdnie_set(dsim->id, mdnie->scenario);

	pr_info("%s\n", __func__);

	return 0;
}

extern void s6e36w4x01_enable(int id);
static int s6e36w4x01_displayon(struct dsim_device *dsim)
{
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);
	struct dsim_resources *res = &dsim->res;

	/* restore brightness level */
	s6e36w4x01_update_brightness(lcd);

	s6e36w4x01_enable(dsim->id);

	lcd->power = FB_BLANK_UNBLANK;

	if (res->err_fg > 0)
		enable_irq(lcd->esd_irq);

	lcd->panel_sc_state = false;
	lcd->panel_icon_state = false;

	if (lcd->enable_afpc) {
		schedule_delayed_work(&lcd->afpc_dwork,
			msecs_to_jiffies(AFPC_WRITE_DELAY));
	}

	schedule_delayed_work(&lcd->debug_dwork,
			msecs_to_jiffies(DEBUG_READ_DELAY));

#ifdef CONFIG_SLEEP_MONITOR
		lcd->act_cnt++;
#endif
	lcd->disp_stime = ktime_get();
	lcd->disp_cnt++;
#if defined(ENABLE_PRINT_DISPLAY_DQA)
	s6e36w4x01_print_dqa(lcd, 1, 0);
#endif
	pr_info("%s[%s]\n", __func__, get_power_state_str(lcd->power));

	return 0;
}

extern void s6e36w4x01_disable(int id);
static int s6e36w4x01_suspend(struct dsim_device *dsim)
{
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);
	struct dsim_resources *res = &dsim->res;
	ktime_t disp_time;

	cancel_delayed_work_sync(&lcd->afpc_dwork);
	cancel_delayed_work_sync(&lcd->debug_dwork);

	mutex_lock(&lcd->afpc_lock);
	lcd->power = FB_BLANK_POWERDOWN;

	if (lcd->ub_con_connected) {
		if (res->err_fg > 0) {
			disable_irq(lcd->esd_irq);
			cancel_delayed_work_sync(&lcd->esd_dwork);
		}
		s6e36w4x01_disable(dsim->id);
	} else
		pr_err("%s:ub_con disconnected\n", __func__);

	s6e36w4x01_disable(dsim->id);
	mutex_unlock(&lcd->afpc_lock);

	disp_time = ktime_get();
	lcd->disp_total_time += ((unsigned int)ktime_ms_delta(disp_time, lcd->disp_stime) / 1000);
#if defined(ENABLE_PRINT_DISPLAY_DQA)
	s6e36w4x01_print_dqa(lcd, 0, 1);
#endif
	pr_info("%s[%s]\n", __func__, get_power_state_str(lcd->power));

	return 0;
}

static int s6e36w4x01_resume(struct dsim_device *dsim)
{
	return 0;
}

static int s6e36w4x01_dump(struct dsim_device *dsim)
{
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);
	int ret;

	pr_info("%s:power=[%s]\n", __func__, get_power_state_str(lcd->power));

	ret = s6e36w4x01_print_debug_reg(dsim->id);
	if (ret) {
		dev_err(lcd->dev, "%s:failed print_debug_reg\n", __func__);
	}

	return 0;
}

static void s6e36w4x01_set_temperature(struct dsim_device *dsim, int temp)
{
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);
	enum temp_range_t temp_stage;
	int ret;

	lcd->temperature = temp;
	temp_stage = s6e36w4x01_get_temp_stage(lcd, lcd->temperature);
	if (temp_stage != lcd->temp_stage) {
		pr_info("%s:stage[%d] temp[%d]\n", __func__, temp_stage, temp);
	} else {
		pr_debug("%s:same stage[%d] temp[%d]\n", __func__, temp_stage, temp);
		return;
	}

	lcd->temp_stage = temp_stage;
	s6e36w4x01_set_offset_comp(lcd, lcd->temperature);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return;
	}

	if (lcd->aod_mode != AOD_DISABLE) {
		pr_err("%s:aod enabled\n", __func__);
		return;
	}

	ret = s6e36w4x01_write_elvss_temp(dsim->id);;
	if (ret) {
		dev_err(lcd->dev, "%s:failed update_temp_stage\n", __func__);
		return;
	}

	return;
}

struct dsim_lcd_driver s6e36w4x01_mipi_lcd_driver = {
	.probe		= s6e36w4x01_probe,
	.init		= s6e36w4x01_init,
	.displayon	= s6e36w4x01_displayon,
	.suspend		= s6e36w4x01_suspend,
	.resume		= s6e36w4x01_resume,
	.dump		= s6e36w4x01_dump,
	.aod_time_set = s6e36w4x01_set_aod_time,
	.aod_analog_set = s6e36w4x01_set_aod_analog,
	.aod_digital_set = s6e36w4x01_set_aod_digital,
	.aod_icon_set = s6e36w4x01_set_aod_icon,
	.aod_move_set = s6e36w4x01_set_aod_move,
	.aod_mode_enter = s6e36w4x01_aod_enter,
	.aod_mode_exit = s6e36w4x01_aod_exit,
#ifdef CONFIG_EXYNOS_DECON_LCD_S6E36W4X01_AFPC
	.afpc_compensation_set = s6e36w4x01_afpc_set_compensation,
	.afpc_panel_get = s6e36w4x01_afpc_get_panel,
#endif
	.ub_con_det = s6e36w4x01_ub_con_det,
	.show_state = s6e36w4x01_show_state,
	.aod_ctrl	= s6e36w4x01_aod_ctrl,
	.temperature_set = s6e36w4x01_set_temperature,
	.metadata_set	= s6e36w4x01_metadata_set,
};
