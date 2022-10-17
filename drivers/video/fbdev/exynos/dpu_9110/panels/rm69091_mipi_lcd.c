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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>
#include <linux/platform_device.h>
#include <linux/lcd.h>

#include "../dsim.h"
#include "rm69091_param.h"
#include "rm69091_mipi_lcd.h"
#include "rm69091_lcd_ctrl.h"
#include "decon_lcd.h"

#define DISPLAY_DQA
#define BACKLIGHT_DEV_NAME	"rm69091-bl"
#define LCD_DEV_NAME		"rm69091"
#define	DEBUG_READ_DELAY	100 /* 100 ms */

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend    rm69091_early_suspend;
#endif

const char* power_state_str[FB_BLANK_POWERDOWN+1] = {
	"UNBLANK", "NORMAL", "V_SUSPEND", "H_SUSPEND", "POWERDOWN",
};

extern int get_panel_id(void);

#if defined(ENABLE_PRINT_DISPLAY_DQA)
void rm69091_print_dqa(struct rm69091 *lcd, int display_on, int aod)
{
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	if (aod)
		dev_info(lcd->dev, "[DQA] AOD %s -> display[%d:%d] AOD[%d:%d]\n",
			display_on ? "on" : "off", dqa_data->disp_cnt, dqa_data->disp_time,
			dqa_data->aod_high_time, dqa_data->aod_low_time);
	else
		dev_info(lcd->dev, "[DQA] %s -> display[%d:%d] AOD[%d:%d]\n",
			display_on ? "on" : "off", dqa_data->disp_cnt, dqa_data->disp_time,
			dqa_data->aod_high_time, dqa_data->aod_low_time);
}
#endif

static void rm69091_debug_dwork(struct work_struct *work)
{
	struct rm69091 *lcd = container_of(work,
				struct rm69091, debug_dwork.work);
	int ret;

	cancel_delayed_work(&lcd->debug_dwork);

	if (POWER_IS_OFF(lcd->power)) {
		dev_err(lcd->dev, "%s:panel off.\n", __func__);
		return;
	}

	ret = rm69091_print_debug_reg(lcd);
	if (ret)
		pr_info("%s:failed rm69091_print_debug_reg[%d]\n", __func__, ret);

	return;
}

static void rm69091_send_esd_event(struct rm69091 *lcd)
{
	struct device *esd_dev = lcd->esd_dev;
	char *event_str = "LCD_ESD=ON";
	char *envp[] = {event_str, NULL};
	int ret;

	if (esd_dev == NULL) {
		pr_err("%s: esd_dev is NULL\n", __func__);
		return;
	}

	ret = kobject_uevent_env(&esd_dev->kobj, KOBJ_CHANGE, envp);
	if (ret)
		dev_err(lcd->dev, "%s:kobject_uevent_env fail\n", __func__);
	else
		dev_info(lcd->dev, "%s:event:[%s]\n", __func__, event_str);

	return;
}

#define ESD_RETRY_TIMEOUT		(10*1000) /* 10 seconds */
static void rm69091_esd_dwork(struct work_struct *work)
{
	struct rm69091 *lcd = container_of(work,
				struct rm69091, esd_dwork.work);

	cancel_delayed_work(&lcd->esd_dwork);

	if (POWER_IS_OFF(lcd->power)) {
		dev_err(lcd->dev, "%s:panel off.\n", __func__);
		return;
	}

	/* this dwork should be canceled by panel suspend.*/
	schedule_delayed_work(&lcd->esd_dwork,
			msecs_to_jiffies(ESD_RETRY_TIMEOUT));

	rm69091_send_esd_event(lcd);
	lcd->esd_cnt++;
}

static ssize_t rm69091_br_map_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	char buffer[32];
	int ret = 0;
	int i;

	for (i = 0; i <= MAX_BRIGHTNESS; i++) {
		ret += scnprintf(buffer, 32, " %3d,", lcd->br_map[i]);
		strncat(buf, buffer, 5);
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

#ifdef CONFIG_TIZEN_SEC_KERNEL_ENG
static ssize_t rm69091_min_lpm_level_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	pr_info("%s:%d/%d\n", __func__, lcd->min_lpm_lv, LPM_BR_MAX-1);

	return scnprintf(buf, PAGE_SIZE, "%d/%d\n", lcd->min_lpm_lv, LPM_BR_MAX-1);
}

static ssize_t rm69091_min_lpm_level_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
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

static ssize_t rm69091_hbm_level_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	pr_info("%s:[%d]\n", __func__, lcd->hbm_level);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->hbm_level);
}

static ssize_t rm69091_hbm_level_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	unsigned char level;
	int ret;

	if (lcd->power > FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "%s:control before lcd enable\n", __func__);
		return -EPERM;
	}

	ret = kstrtou8(buf, 0, &level);
	if (ret) {
		pr_err("%s:Failed to get hbm level\n", __func__);
		return -EINVAL;
	}

	lcd->hbm_level = level;
	pr_info("%s:HBM level[%d]\n", __func__, level);

	ret = rm69091_hbm_on(lcd);
	if (ret) {
		pr_err("%s:failed HBM ON[%d]\n", __func__, ret);
		return -EIO;
	}

	return size;
}

extern void rm69091_crack_check_on(struct rm69091 *lcd);
extern void rm69091_crack_check_off(struct rm69091 *lcd);
static ssize_t rm69091_crack_check_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->enable_crack_check ? "on" : "off");
}

static ssize_t rm69091_crack_check_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	bool value;

	if (!strncmp(buf, "on", 2)) {
		value = true;
	} else if (!strncmp(buf, "off", 3)) {
		value = false;
	} else {
		pr_err("invalid comman (use on or off)\n");
		return size;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->lpm_on) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	lcd->enable_crack_check = value;
	dev_info(lcd->dev, "%s[%d]\n", __func__, lcd->enable_crack_check);

	if (lcd->enable_crack_check)
		rm69091_crack_check_on(lcd);
	else
		rm69091_crack_check_off(lcd);

	return size;
}

static ssize_t rm69091_fd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	dev_info(lcd->dev, "%s:%d\n", __func__, lcd->enable_fd);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->enable_fd ? "on" : "off");
}

extern void rm69091_enable_fd(struct rm69091 *lcd);
static ssize_t rm69091_fd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int value;

	if (!strncmp(buf, "on", 2))
		value = 1;
	else if (!strncmp(buf, "off", 3))
		value = 0;
	else {
		dev_warn(dev, "invalid comman (use on or off)\n");
		return size;
	}

	dev_info(lcd->dev, "%s:cur[%d] new[%d]\n", __func__, lcd->enable_fd, value);

	lcd->enable_fd = value;

	if (lcd->enable_fd) {
		if (lcd->power > FB_BLANK_NORMAL) {
			pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
			return -EPERM;
		}

		if (lcd->lpm_on) {
			pr_info("%s:aod enabled\n", __func__);
			return -EPERM;
		}

		rm69091_enable_fd(lcd);
	}

	return size;
}

static ssize_t rm69091_curve_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->enable_curve ? "on" : "off");
}

extern void rm69091_curve_on(struct rm69091 *lcd);
extern void rm69091_curve_off(struct rm69091 *lcd);
static ssize_t rm69091_curve_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
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
		__func__, lcd->enable_curve, value);

	lcd->enable_curve = value;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->lpm_on) {
		pr_info("%s:aod enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->enable_curve)
		rm69091_curve_on(lcd);
	else
		rm69091_curve_off(lcd);

	return size;
}

static ssize_t rm69091_br_level_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	pr_info("%s:[%d]\n", __func__, lcd->br_level);

	return scnprintf(buf, PAGE_SIZE, "%d\n", lcd->br_level);
}

static ssize_t rm69091_br_level_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
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

	ret = rm69091_gamma_ctrl(dsim->id, level);
	if (ret) {
		pr_err("%s:Failed rm69091_gamma_ctrl\n", __func__);
		return -EIO;
	}

	pr_info("%s:level[%d]\n", __func__, level);

	return size;
}

static ssize_t rm69091_normal_aod_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	pr_info("%s:[%d]\n", __func__, lcd->lpm_on);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->lpm_on ? "on" : "off");
}

static ssize_t rm69091_normal_aod_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int ret;
	bool value;

	if (lcd->power > FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "%s:control before lcd enable\n", __func__);
		return -EPERM;
	}

	if (!strncmp(buf, "on", 2)) {
		value = true;
	} else if (!strncmp(buf, "off", 3)) {
		value = false;
	} else {
		dev_err(dev, "%s:invalid command(use on or off)\n", __func__);
		return -EINVAL;
	}

	if (value) {
		ret = rm69091_normal_aod_on(lcd);
		if (ret) {
			pr_err("%s:failed LPM ON[%d]\n", __func__, ret);
			return -EIO;
		}
	} else {
		ret = rm69091_normal_aod_off(lcd);
		if (ret) {
			pr_err("%s:failed LPM OFF[%d]\n", __func__, ret);
			return -EIO;
		}
	}

	lcd->lpm_on = value;
	pr_info("%s:val[%d]\n", __func__, lcd->lpm_on);

	return size;
}

static ssize_t rm69091_mcd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->mcd_on ? "on" : "off");
}

static ssize_t rm69091_mcd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int old_hbm;

	if (lcd->lpm_on) {
		pr_info("%s:alpm is enabled\n", __func__);
		return -EPERM;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->hbm_on) {
		old_hbm = lcd->hbm_on;
		lcd->hbm_on = 0;
		rm69091_hbm_off(lcd);
		lcd->hbm_on = old_hbm;
	}

	if (!strncmp(buf, "on", 2)) {
		lcd->mcd_on = true;
		rm69091_pcd_test_on(lcd);
	} else if (!strncmp(buf, "off", 3)) {
		lcd->mcd_on = false;
		rm69091_pcd_test_off(lcd);
	} else
		dev_err(dev, "%s:invalid command.\n", __func__);

	pr_info("%s:[%s]\n", __func__, lcd->mcd_on ? "on":"off");

	return size;
}

static ssize_t rm69091_hbm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	pr_debug("%s:[%d]\n", __func__, lcd->hbm_on);

	return scnprintf(buf, PAGE_SIZE, "%s\n", lcd->hbm_on ? "on" : "off");
}

static ssize_t rm69091_hbm_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int ret;

	if (!strncmp(buf, "on", 2))
		lcd->hbm_on = true;
	else if (!strncmp(buf, "off", 3))
		lcd->hbm_on = false;
	else {
		pr_err("%s:invalid command(use on or off)\n", __func__);
		return -EINVAL;
	}

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:hbm control before lcd enable\n", __func__);
		goto out;
	}

	if (lcd->lpm_on) {
		pr_info("%s:aod enabled\n", __func__);
		goto out;
	}

	if (lcd->hbm_on) {
		ret = rm69091_hbm_on(lcd);
		if (ret) {
			pr_err("%s:failed HBM ON[%d]\n", __func__, ret);
			return -EIO;
		}
	} else {
		ret = rm69091_hbm_off(lcd);
		if (ret) {
			pr_err("%s:failed HBM OFF[%d]\n", __func__, ret);
			return -EIO;
		}
	}

	pr_info("%s:HBM[%s]\n", __func__, lcd->hbm_on ? "ON" : "OFF");

out:
	return size;
}

static ssize_t rm69091_lcd_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	pr_info("%s:BOE_%06x\n", __func__, get_panel_id());
	return scnprintf(buf, PAGE_SIZE, "BOE_%06x\n", get_panel_id());
}

#define 	MAX_MTP_READ_SIZE	0xff
static unsigned char mtp_read_data[MAX_MTP_READ_SIZE] = {0, };
static int read_data_length = 0;
static int read_data_addr;
static int read_data_offset;
extern int rm69091_read_mtp_reg(int id, u32 addr, char* buffer, u32 size);
static ssize_t rm69091_read_mtp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	char buffer[LDI_MTP4_LEN*8] = {0, };
	int i, ret = 0;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	ret = rm69091_read_mtp_reg(dsim->id,
		read_data_addr+read_data_offset, &mtp_read_data[0], read_data_length);
	if (ret) {
		pr_err("%s: read failed[%d]\n", __func__, ret);
		return -EIO;
	}

	for ( i = 0; i < read_data_length; i++) {
		if ((i !=0) && !(i%8)) {
			strncat(buf, "\n", strlen("\n"));
			ret += strlen("\n");
		}

		ret += scnprintf(buffer, LDI_MTP4_LEN, "0x%02x", mtp_read_data[i]);
		strncat(buf, buffer, 4);

		if ( i < (read_data_length-1)) {
			strncat(buf, ", ", strlen(", "));
			ret += strlen(", ");
		}
	}

	strncat(buf, "\n", strlen("\n"));
	ret += strlen("\n");

	pr_info("%s: length=%d\n", __func__, read_data_length);
	pr_info("%s\n", buf);

	return ret;
}

static ssize_t rm69091_read_mtp_store(struct device *dev,
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

extern void rm69091_write_mtp_reg(int id, char* buffer, u32 size);
static ssize_t rm69091_write_mtp_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
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

	rm69091_write_mtp_reg(dsim->id, tx_buf, len);

	kfree(tx_buf);
	tx_buf = NULL;

out:

	return size;
}


static ssize_t rm69091_esd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	unsigned int esd_cnt = lcd->esd_cnt;

	lcd->esd_cnt = 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", esd_cnt);
}

static ssize_t rm69091_esd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	cancel_delayed_work(&lcd->esd_dwork);
	schedule_delayed_work(&lcd->esd_dwork, 0);

	return size;
}

extern int rm69091_read_cell_id(struct rm69091 *lcd, char* buffer);
static ssize_t rm69091_cell_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	char read_buf[PANEL_ID_LEN] = {0, };
	int ret;

	ret = rm69091_read_cell_id(lcd, &read_buf[0]);
	if (ret) {
		pr_err("%s:failed read panel_id[%d]\n", __func__, ret);
		return -EIO;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%02d%02d%02d%02d%02d%02d%02d%02x%02x%02x%02x\n",
				read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4],
				read_buf[5], read_buf[6], read_buf[7], read_buf[8], read_buf[9],
				read_buf[10]);

	pr_info("%s:%s\n", __func__, buf);

	return ret;
}

static ssize_t rm69091_svc_chipid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int ret;

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_info("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	/* dummy interface is showing 0, because boe panel is not support svc_chipid */
	ret = scnprintf(buf, PAGE_SIZE, "0\n");

	return ret;
}

extern unsigned int system_rev;
static void rm69091_show_state(struct dsim_device *dsim)
{
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);
	struct backlight_device	*bd = lcd->bd;

	pr_info("%s:system_rev[0x%02x] panel_id[0x%06x]\n",
				__func__, system_rev, get_panel_id());
	pr_info("%s:power[%d] aod[%s]\n",
				__func__, lcd->power, lcd->lpm_on ? "on":"off");
	pr_info("%s:brightness[%d] hbm[%s]\n",
				__func__, bd->props.brightness,
				lcd->hbm_on ? "on":"off");
	pr_info("%s:esd_cnt[%d]\n",	__func__,  lcd->esd_cnt);

	return;
}

static ssize_t rm69091_lcd_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;

	rm69091_show_state(dsim);

	return 0;
}

static ssize_t rm69091_lpm_br_max_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_max);

	pr_info("%s: %d\n", __func__, lcd->lpm_br_max);

	return ret;
}

static ssize_t rm69091_lpm_br_normal_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_normal);

	pr_info("%s: %d\n", __func__, lcd->lpm_br_normal);

	return ret;
}

static ssize_t rm69091_lpm_br_min_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_min);

	pr_info("%s: %d\n", __func__, lcd->lpm_br_min);

	return ret;
}

static ssize_t rm69091_lpm_br_charging_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd->lpm_br_charging);

	pr_info("%s: %d\n", __func__, lcd->lpm_br_charging);

	return ret;
}

extern unsigned char rm69091_get_offset_comp(void);
extern enum temp_range_t rm69091_get_temp_stage(int temp);
extern void rm69091_set_offset_comp(enum temp_range_t range);
extern void rm69091_write_elvss_temp(struct rm69091 *lcd);
static ssize_t rm69091_temperature_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "stage[%d] temp[%d] reg[0x%02x]\n",
		lcd->temp_stage, lcd->temperature, rm69091_get_offset_comp());
}

static ssize_t rm69091_temperature_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	long value;
	int ret;

	ret = kstrtol(buf, 0, &value);
	if (ret) {
		dsim_err("%s:failed buffer read[%d]\n", __func__, ret);
		return ret;
	}

	lcd->temperature = (int)value;
	lcd->temp_stage = rm69091_get_temp_stage(lcd->temperature);

	pr_info("%s:temp[%d][%d]\n", __func__, (int)value, lcd->temp_stage);

	rm69091_set_offset_comp(lcd->temp_stage);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return -EPERM;
	}

	if (lcd->lpm_on) {
		pr_err("%s:lpm enabled\n", __func__);
		return -EPERM;
	}

	rm69091_write_elvss_temp(lcd);

	return size;
}

static struct device_attribute rm69091_dev_attrs[] = {
	__ATTR(state, S_IRUGO, rm69091_lcd_state_show, NULL),
	__ATTR(br_map, S_IRUGO, rm69091_br_map_show, NULL),
	__ATTR(br_level, S_IRUGO | S_IWUSR, rm69091_br_level_show, rm69091_br_level_store),
	__ATTR(hbm_level, S_IRUGO | S_IWUSR, rm69091_hbm_level_show, rm69091_hbm_level_store),
	__ATTR(curve, S_IRUGO | S_IWUSR, rm69091_curve_show, rm69091_curve_store),
	__ATTR(enable_fd, S_IRUGO | S_IWUSR, rm69091_fd_show, rm69091_fd_store),
	__ATTR(crack_check, S_IRUGO | S_IWUSR, rm69091_crack_check_show, rm69091_crack_check_store),
	__ATTR(esd, S_IRUGO | S_IWUSR, rm69091_esd_show, rm69091_esd_store),
	__ATTR(lpm, S_IRUGO | S_IWUSR, rm69091_normal_aod_show, rm69091_normal_aod_store),
	__ATTR(hbm, S_IRUGO | S_IWUSR, rm69091_hbm_show, rm69091_hbm_store),
	__ATTR(mcd_test, S_IRUGO | S_IWUSR, rm69091_mcd_show, rm69091_mcd_store),
	__ATTR(lcd_type, S_IRUGO , rm69091_lcd_type_show, NULL),
	__ATTR(read_mtp, S_IRUGO | S_IWUSR, rm69091_read_mtp_show,
					rm69091_read_mtp_store),
	__ATTR(write_mtp, S_IRUGO | S_IWUSR, rm69091_read_mtp_show,
					rm69091_write_mtp_store),
	__ATTR(lpm_br_max, S_IRUGO , rm69091_lpm_br_max_show, NULL),
	__ATTR(lpm_br_normal, S_IRUGO , rm69091_lpm_br_normal_show, NULL),
	__ATTR(lpm_br_min, S_IRUGO , rm69091_lpm_br_min_show, NULL),
	__ATTR(lpm_br_charging, S_IRUGO, rm69091_lpm_br_charging_show, NULL),
	__ATTR(temperature, S_IRUGO | S_IWUSR, rm69091_temperature_show,
							rm69091_temperature_store),
	__ATTR(cell_id, S_IRUGO , rm69091_cell_id_show, NULL),
	__ATTR(SVC_OCTA_CHIPID, S_IRUGO, rm69091_svc_chipid_show, NULL),
#ifdef CONFIG_TIZEN_SEC_KERNEL_ENG
	__ATTR(min_lpm_lv, S_IRUGO | S_IWUSR, rm69091_min_lpm_level_show,
							rm69091_min_lpm_level_store),
#endif
};

#ifdef DISPLAY_DQA
static ssize_t rm69091_display_model_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct decon_lcd *lcd_info = &dsim->lcd_info;
	int size;

	pr_info("%s:%s\n", __func__, lcd_info->model_name);

	size = scnprintf(buf, PAGE_SIZE, "BOE_%s\n", lcd_info->model_name);

	return size;
}

static ssize_t rm69091_lcdm_id1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	int ret;

	pr_info("%s:%d\n", __func__, panel->id[0]);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", panel->id[0]);

	return ret;
}

static ssize_t rm69091_lcdm_id2_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	int ret;

	pr_info("%s:%d\n", __func__, panel->id[1]);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", panel->id[1]);

	return ret;
}

static ssize_t rm69091_lcdm_id3_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	int ret;

	pr_info("%s:%d\n", __func__, panel->id[2]);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", panel->id[2]);

	return ret;
}

static ssize_t rm69091_pndsie_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	int ret;

	pr_info("%s:%d\n", __func__, dsim->comm_err_cnt);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dsim->comm_err_cnt);

	dsim->comm_err_cnt = 0;

	return ret;
}

static ssize_t rm69091_qct_no_te_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dsim_device *dsim = lcd->dsim;
	int ret;

	pr_info("%s:%d\n", __func__, dsim->decon_timeout_cnt);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dsim->decon_timeout_cnt);
	dsim->decon_timeout_cnt = 0;
	dsim_store_timeout_count(dsim);

	return ret;
}

static ssize_t rm69091_lbhd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	int ret;

	pr_info("%s:%d\n", __func__, dqa_data->hbm_time);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dqa_data->hbm_time);
	dqa_data->hbm_time = 0;

	return ret;
}

static ssize_t rm69091_lod_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	int ret;

	pr_info("%s:%d\n", __func__, dqa_data->disp_time);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dqa_data->disp_time);
	dqa_data->disp_time = 0;

	return ret;
}

static ssize_t rm69091_daod_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	int ret;

	pr_info("%s:%d\n", __func__,
			(dqa_data->aod_high_time + dqa_data->aod_low_time));

	ret = scnprintf(buf, PAGE_SIZE, "%d\n",
			(dqa_data->aod_high_time + dqa_data->aod_low_time));

	return ret;
}

static ssize_t rm69091_dahl_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	int ret;

	pr_info("%s:%d\n", __func__, dqa_data->aod_high_time);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dqa_data->aod_high_time);
	dqa_data->aod_high_time = 0;

	return ret;
}

static ssize_t rm69091_dall_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	int ret;

	pr_info("%s:%d\n", __func__, dqa_data->aod_low_time);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dqa_data->aod_low_time);
	dqa_data->aod_low_time = 0;

	return ret;
}

static ssize_t rm69091_locnt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rm69091 *lcd = dev_get_drvdata(dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	int ret;

	pr_info("%s:%d\n", __func__, dqa_data->disp_cnt);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", dqa_data->disp_cnt);
	dqa_data->disp_cnt = 0;

	return ret;
}

static struct device_attribute rm69091_dqa_attrs[] = {
	__ATTR(disp_model, S_IRUGO , rm69091_display_model_show, NULL),
	__ATTR(lcdm_id1, S_IRUGO , rm69091_lcdm_id1_show, NULL),
	__ATTR(lcdm_id2, S_IRUGO , rm69091_lcdm_id2_show, NULL),
	__ATTR(lcdm_id3, S_IRUGO , rm69091_lcdm_id3_show, NULL),
	__ATTR(pndsie, S_IRUGO , rm69091_pndsie_show, NULL),
	__ATTR(qct_no_te, S_IRUGO, rm69091_qct_no_te_show, NULL),
	__ATTR(daod, S_IRUGO, rm69091_daod_show, NULL),
	__ATTR(dahl, S_IRUGO, rm69091_dahl_show, NULL),
	__ATTR(dall, S_IRUGO, rm69091_dall_show, NULL),
	__ATTR(lbhd, S_IRUGO , rm69091_lbhd_show, NULL),
	__ATTR(lod, S_IRUGO , rm69091_lod_show, NULL),
	__ATTR(locnt, S_IRUGO , rm69091_locnt_show, NULL),
};
#endif

static int rm69091_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int rm69091_update_brightness(struct rm69091 *lcd)
{
	struct backlight_device *bd = lcd->bd;
	struct dsim_device *dsim = lcd->dsim;
	int brightness = bd->props.brightness, ret = 0;

	if (lcd->hbm_on) {
		ret = rm69091_hbm_on(lcd);
		if (ret) {
			pr_err("%s:failed change_brightness\n", __func__);
			goto out;
		}
	} else {
		ret = rm69091_gamma_ctrl(dsim->id, lcd->br_map[brightness]);
		if (ret) {
			pr_err("%s:failed change_brightness\n", __func__);
			goto out;
		}
	}

	pr_debug("%s:br[%d] lv[%d] hbm[%d]\n", __func__,
				brightness, lcd->br_map[brightness], lcd->hbm_on);
out:
	return ret;
}

static int rm69091_set_brightness(struct backlight_device *bd)
{
	struct rm69091 *lcd = bl_get_data(bd);
	int brightness = bd->props.brightness, ret = 0;

	if (lcd == NULL) {
		pr_err("%s:LCD is NULL\n", __func__);
		return -ENODEV;
	}

	if (brightness < MIN_BRIGHTNESS || brightness > MAX_BRIGHTNESS) {
		pr_err("Brightness should be in the range of 0 ~ 255\n");
		return -EINVAL;
	}

	pr_info("%s:br[%d] lv[%d] hbm[%d] pwr[%s]\n", __func__,
				brightness, lcd->br_map[brightness],
				lcd->hbm_on, power_state_str[lcd->power]);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_debug("%s:invalid power[%d]\n", __func__, lcd->power);
		return 0;
	}

	if (lcd->lpm_on) {
		pr_err("%s:LPM enabled\n", __func__);
		return 0;
	}

	if (lcd->ub_con_connected == false) {
		pr_info("%s:ub_con disconnected\n", __func__);
		return 0;
	}

	ret = rm69091_update_brightness(lcd);
	if (ret)
		pr_err("%s:failed to update brightness\n", __func__);

	panel_vsync_wait(PANEL_60HZ_1FRAME_MSEC);

	return ret;
}

int rm69091_metadata_set(struct dsim_device *dsim, struct decon_metadata *metadata)
{
	struct aod_config *aod_cfg = &metadata->aod_cfg;
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);

	pr_info("%s:ops[%d] req[%d] mode[%d] refresh_rate[%d]\n",
		__func__, metadata->ops, metadata->aod_cfg.req, metadata->aod_cfg.mode,
		metadata->aod_cfg.panel_refresh_rate);

	if (metadata->ops != METADATA_OP_AOD_SET_INFO) {
		pr_err("%s:invalid ops[%d]\n", __func__, metadata->ops);
		return -EINVAL;
	}

	if ((aod_cfg->req != AOD_SET_PANEL_REFRESH_RATE) &&
		(aod_cfg->req != AOD_SET_CONFIG)) {
		pr_err("%s:invalid request type[%d]\n", __func__, aod_cfg->req);
		return -EINVAL;
	}

	if (aod_cfg->mode < AOD_NORMAL) {
		pr_err("%s:invalid aod mode[%d]\n", __func__, aod_cfg->mode);
		return -EINVAL;
	}

	if (aod_cfg->panel_refresh_rate > AOD_PANEL_REFRESH_RATE_HIGH) {
		pr_err("%s:invalid refresh rate[%d]\n", __func__, aod_cfg->panel_refresh_rate);
		return -EINVAL;
	}

	lcd->aod_refresh = aod_cfg->panel_refresh_rate;

	return 0;
}

static int rm69091_aod_ctrl(struct dsim_device *dsim, enum aod_state state, enum aod_mode aod_type)
{
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);
	int ret = 0;

	pr_debug("%s:state[%d] type[%d]\n", __func__, state, aod_type);

	if (aod_type < AOD_NORMAL) {
		pr_err("%s:Invalid aod_type[%d]\n", __func__, aod_type);
		return -EINVAL;
	}

	lcd->lpm_on = (state == AOD_EXIT ? false : true);

	if (state == AOD_ENTER) {
		if (lcd->hbm_on) {
			ret = rm69091_hbm_off(lcd);
			if (ret) {
				pr_err("%s:failed HBM OFF[%d]\n", __func__, ret);
			}
		}
		ret = rm69091_normal_aod_on(lcd);
	} else if (state == AOD_EXIT) {
		ret = rm69091_normal_aod_off(lcd);
	} else if (state == AOD_CHANGE) {
		pr_info("%s:skip AOD_CHANGE\n", __func__);
	} else {
	   pr_err("%s:Invalid aod state[%d]\n", __func__, state);
	}

	return ret;
}

static int rm69091_pinctrl_configure(struct rm69091 *lcd)
{
	int retval = 0;

	lcd->pinctrl = devm_pinctrl_get(lcd->dev);
	if (IS_ERR(lcd->pinctrl)) {
		if (PTR_ERR(lcd->pinctrl) == -EPROBE_DEFER) {
			pr_err("%s:failed to get pinctrl\n", __func__);
			retval = -ENODEV;
		}
		pr_info("%s:Target does not use pinctrl\n", __func__);
		lcd->pinctrl = NULL;
		goto out;
	}

	if (lcd->pinctrl) {
		lcd->gpio_aod = pinctrl_lookup_state(lcd->pinctrl, "aod_on");
		if (IS_ERR(lcd->gpio_aod)) {
			pr_info("%s:Target does not use gpio_aod pinctrl\n", __func__);
			lcd->gpio_aod = NULL;
		}

		lcd->gpio_off = pinctrl_lookup_state(lcd->pinctrl, "aod_off");
		if (IS_ERR(lcd->gpio_off)) {
			pr_info("%s:Target does not use gpio_off pinctrl\n", __func__);
			lcd->gpio_off = NULL;
		}
	}

	if (lcd->pinctrl && lcd->gpio_off) {
		if (pinctrl_select_state(lcd->pinctrl, lcd->gpio_off))
			pr_err("%s:failed to turn on gpio_off\n", __func__);
	}

out:
	return retval;
}

static int rm69091_get_power(struct lcd_device *ld)
{
	struct rm69091 *lcd = dev_get_drvdata(&ld->dev);
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

static int rm69091_set_power(struct lcd_device *ld, int power)
{
	struct rm69091 *lcd = dev_get_drvdata(&ld->dev);

	if (power > FB_BLANK_POWERDOWN) {
		pr_err("%s:invalid power state.[%d]\n", __func__, power);
		return -EINVAL;
	}

	lcd->power = power;
	wristup_booster_set_lcd_power(power);

	pr_info("%s[%s]\n", __func__, power_state_str[lcd->power]);

	return 0;
}

static struct lcd_ops rm69091_lcd_ops = {
	.get_power = rm69091_get_power,
	.set_power = rm69091_set_power,
};

static const struct backlight_ops rm69091_backlight_ops = {
	.get_brightness = rm69091_get_brightness,
	.update_status = rm69091_set_brightness,
};

static int rm69091_probe(struct dsim_device *dsim)
{
	struct panel_private *panel = &dsim->priv;
	struct rm69091 *lcd;
	int panel_id;
	int ret = 0, i;

	pr_info("%s\n", __func__);

	if (get_panel_id() == -1) {
		pr_err("%s:No lcd attached!\n", __func__);
		return -ENODEV;
	}

	lcd = devm_kzalloc(dsim->dev,
			sizeof(struct rm69091), GFP_KERNEL);
	if (!lcd) {
		pr_err("%s:failed to allocate rm69091 structure\n", __func__);
		return -ENOMEM;
	}

	lcd->dev = dsim->dev;
	lcd->dsim = dsim;
	lcd->hbm_level = HBM_DEFAULT_LEVEL;
	lcd->min_lpm_lv = LPM_BR_LOW;
	lcd->br_map = &rm69091_br_map[0];
	panel_id = get_panel_id();

	panel->id[0] = ((panel_id >> 16) & 0xff);
	panel->id[1] = ((panel_id >> 8) & 0xff);
	panel->id[2] = (panel_id & 0xff);

	mutex_init(&lcd->mipi_lock);

	lcd->bd = backlight_device_register(BACKLIGHT_DEV_NAME,
		lcd->dev, lcd, &rm69091_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		pr_err("%s:failed to register backlight device[%d]\n",
			__func__, (int)PTR_ERR(lcd->bd));
		ret = PTR_ERR(lcd->bd);
		goto err_bd;
	}
	lcd->bd->props.max_brightness = MAX_BRIGHTNESS;
	lcd->bd->props.brightness = DEFAULT_BRIGHTNESS;
	lcd->ub_con_connected = true;
	lcd->temp_stage = TEMP_RANGE_MAX;
	lcd->aod_refresh = AOD_PANEL_REFRESH_RATE_LOW;

	lcd->lpm_br_max = LPM_BR_MAX_INDEX;
	lcd->lpm_br_normal = LPM_BR_NORMAL_INDEX;
	lcd->lpm_br_min = LPM_BR_MIN_INDEX;
	lcd->lpm_br_charging = LPM_BR_CHARGING_INDEX;

#ifdef CONFIG_SEC_FACTORY
	lcd->enable_curve = false;
	lcd->enable_fd = true;
#else
	lcd->enable_curve = true;
#endif

	dsim->ld = lcd_device_register(LCD_DEV_NAME,
			lcd->dev, lcd, &rm69091_lcd_ops);
	if (IS_ERR(dsim->ld)) {
		pr_err("%s:failed to register lcd ops[%d]\n",
			__func__, (int)PTR_ERR(dsim->ld));
		ret = PTR_ERR(lcd->bd);
		goto err_ld;
	}
	lcd->ld = dsim->ld;

	lcd->esd_class = class_create(THIS_MODULE, "lcd_event");
	if (IS_ERR(lcd->esd_class)) {
		dev_err(lcd->dev, "%s:Failed to create esd_class[%d]\n",
			__func__, (int)PTR_ERR(lcd->esd_class));
		ret = PTR_ERR(lcd->esd_class);
		goto err_esd_class;
	}

	lcd->esd_dev = device_create(lcd->esd_class, lcd->dev, 0, lcd, "esd");
	if (IS_ERR(lcd->esd_dev)) {
		dev_err(lcd->dev, "%s:Failed to create esd_dev\n", __func__);
		goto err_esd_dev;
	}
	INIT_DELAYED_WORK(&lcd->esd_dwork, rm69091_esd_dwork);

	for (i = 0; i < ARRAY_SIZE(rm69091_dev_attrs); i++) {
		ret = device_create_file(&lcd->ld->dev, &rm69091_dev_attrs[i]);
		if (ret < 0) {
			dev_err(&lcd->ld->dev,
				"%s:failed to add rm69091_dev_attrs\n", __func__);
			for (i--; i >= 0; i--)
				device_remove_file(&lcd->ld->dev, &rm69091_dev_attrs[i]);
			goto err_create_dev_file;
		}
	}

#ifdef DISPLAY_DQA
	for (i = 0; i < ARRAY_SIZE(rm69091_dqa_attrs); i++) {
		ret = device_create_file(&lcd->ld->dev, &rm69091_dqa_attrs[i]);
		if (ret < 0) {
			dev_err(&lcd->ld->dev,
				"%s:failed to add rm69091_dqa_attrs\n", __func__);
			for (i--; i >= 0; i--)
				device_remove_file(&lcd->ld->dev, &rm69091_dqa_attrs[i]);
			goto err_create_dqa_file;
		}
	}
#endif

	ret = sysfs_create_link(&dsim->sec_dev->kobj, &lcd->ld->dev.kobj, "panel");
	if (ret < 0) {
		dev_err(lcd->dev, "%s: Failed to create panel symbolic link %d\n", __func__, ret);
		goto err_create_dev_file;
	}
	ret = sysfs_create_link(&dsim->sec_dev->kobj, &lcd->bd->dev.kobj, "backlight");
	if (ret < 0) {
		dev_err(lcd->dev, "%s: Failed to create backlight symbolic link %d\n", __func__, ret);
		goto err_create_dev_file;
	}

	ret = rm69091_pinctrl_configure(lcd);
	if (ret)
		dev_err(&lcd->ld->dev,
				"%s:failed rm69091_pinctrl_configure\n", __func__);

#ifdef CONFIG_SEC_FACTORY
	if (lcd->enable_fd)
		rm69091_enable_fd(lcd);
#endif

	INIT_DELAYED_WORK(&lcd->debug_dwork, rm69091_debug_dwork);
	schedule_delayed_work(&lcd->debug_dwork,
			msecs_to_jiffies(DEBUG_READ_DELAY));

	pr_info("%s done\n", __func__);

	return 0;

#ifdef DISPLAY_DQA
err_create_dqa_file:
	for (i--; i >= 0; i--)
		device_remove_file(&lcd->ld->dev, &rm69091_dev_attrs[i]);
#endif
err_create_dev_file:
	device_destroy(lcd->esd_class, lcd->esd_dev->devt);
err_esd_dev:
	class_destroy(lcd->esd_class);
err_esd_class:
	lcd_device_unregister(lcd->ld);
err_ld:
	backlight_device_unregister(lcd->bd);
err_bd:
	mutex_destroy(&lcd->mipi_lock);
	devm_kfree(dsim->dev, lcd);
	return ret;
}

static int rm69091_pre_reset(struct dsim_device *dsim)
{
	struct dsim_resources *res = &dsim->res;
	int ret;

	pr_info("%s\n", __func__);

	msleep(5);

	ret = gpio_request_one(res->lcd_reset, GPIOF_OUT_INIT_HIGH, "lcd_reset");
	if (ret < 0) {
		dsim_err("failed to get LCD reset GPIO\n");
		return -EINVAL;
	}

	msleep(1);
	gpio_set_value(res->lcd_reset, 0);
	msleep(1);
	gpio_set_value(res->lcd_reset, 1);
	msleep(10);
	gpio_free(res->lcd_reset);

	return 0;
}

int rm69091_ub_con_det(struct dsim_device *dsim, bool status)
{
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);

	lcd->ub_con_connected = status;

	if (!lcd->ub_con_connected)
		pr_info("%s[disconnected]\n", __func__);

	return 0;
}

static int rm69091_displayon(struct dsim_device *dsim)
{
	struct lcd_device *lcd_dev = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&lcd_dev->dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;

	lcd->power = FB_BLANK_UNBLANK;

	rm69091_update_brightness(lcd);
	rm69091_enable(dsim->id);

	schedule_delayed_work(&lcd->debug_dwork,
			msecs_to_jiffies(DEBUG_READ_DELAY));

	dqa_data->disp_stime = ktime_get();
	dqa_data->disp_cnt++;

#if defined(ENABLE_PRINT_DISPLAY_DQA)
	rm69091_print_dqa(lcd, 1, 0);
#endif
	pr_info("%s\n", __func__);
	return 0;
}

static int rm69091_init(struct dsim_device *dsim)
{
	usleep_range(1000, 1100);

	rm69091_init_ctrl(dsim->id, &dsim->lcd_info);

	pr_info("%s[0x%06x]\n", __func__, get_panel_id());
	return 0;
}

static int rm69091_suspend(struct dsim_device *dsim)
{
	struct lcd_device *lcd_dev = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&lcd_dev->dev);
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	ktime_t now_time;

	cancel_delayed_work_sync(&lcd->debug_dwork);
	cancel_delayed_work_sync(&lcd->esd_dwork);


	if (lcd->ub_con_connected)
		rm69091_disable(dsim->id);
	else
		pr_err("%s:ub_con disconnected\n", __func__);

	lcd->power = FB_BLANK_POWERDOWN;

	now_time = ktime_get();
	dqa_data->disp_time += ((unsigned int)ktime_ms_delta(now_time,
										dqa_data->disp_stime) / 1000);

	if (lcd->pinctrl && lcd->gpio_off) {
		if (pinctrl_select_state(lcd->pinctrl, lcd->gpio_off))
			pr_err("%s:failed to turn on gpio_off\n", __func__);
	}

#if defined(ENABLE_PRINT_DISPLAY_DQA)
	rm69091_print_dqa(lcd, 0, 1);
#endif
	pr_info("%s\n", __func__);

	return 0;
}

static int rm69091_resume(struct dsim_device *dsim)
{
	return 0;
}

static int rm69091_dump(struct dsim_device *dsim)
{
	struct lcd_device *lcd_dev = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&lcd_dev->dev);
	int ret;

	ret = rm69091_print_debug_reg(lcd);
	if (ret)
		pr_info("%s:failed rm69091_print_debug_reg[%d]\n", __func__, ret);

	return 0;
}

static void rm69091_set_temperature(struct dsim_device *dsim, int temp)
{
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);
	enum temp_range_t temp_stage;

	lcd->temperature = temp;
	temp_stage = rm69091_get_temp_stage(lcd->temperature);
	if (temp_stage != lcd->temp_stage) {
		pr_info("%s:stage[%d] temp[%d]\n", __func__, temp_stage, temp);
	} else {
		pr_debug("%s:same stage[%d] temp[%d]\n", __func__, temp_stage, temp);
		return;
	}

	lcd->temp_stage = temp_stage;
	rm69091_set_offset_comp(lcd->temp_stage);

	if (lcd->power > FB_BLANK_NORMAL) {
		pr_err("%s:invalid power[%d]\n", __func__, lcd->power);
		return;
	}

	if (lcd->lpm_on) {
		pr_err("%s:lpm enabled\n", __func__);
		return;
	}

	rm69091_write_elvss_temp(lcd);

	return;
}


struct dsim_lcd_driver rm69091_mipi_lcd_driver = {
	.probe		= rm69091_probe,
	.init		= rm69091_init,
	.displayon	= rm69091_displayon,
	.pre_reset	= rm69091_pre_reset,
 	.suspend	= rm69091_suspend,
	.resume		= rm69091_resume,
	.dump		= rm69091_dump,
	.aod_ctrl	= rm69091_aod_ctrl,
	.show_state = rm69091_show_state,
	.ub_con_det = rm69091_ub_con_det,
	.temperature_set = rm69091_set_temperature,
};
