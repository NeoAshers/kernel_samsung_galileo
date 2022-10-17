/* s6e36w4x01_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2020 Samsung Electronics
 *
 * SeungBeom, Park <sb1.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "../dsim.h"
#include <video/mipi_display.h>

#include "s6e36w4x01_dimming.h"
#include "s6e36w4x01_param.h"
#include "s6e36w4x01_lcd_ctrl.h"
#include "s6e36w4x01_mipi_lcd.h"

static unsigned char GAMMA_SET[GAMMA_CMD_CNT] = {
	0xCA,
	0x07, 0x00, 0x00, 0x00,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x80, 0x80, 0x80,
	0x00, 0x00, 0x00,
};

extern unsigned char s6e36w4x01_gamma_set[MAX_BR_INFO][GAMMA_CMD_CNT];
extern const char s6e36w4x01_vbias[MAX_BR_INFO];
extern const char s6e36w4x01_vbias_30hz_latest[MAX_BR_INFO];
extern const char s6e36w4x01_vbias_10hz_latest[MAX_BR_INFO];
extern const char s6e36w4x01_vbias_30hz_0x83[MAX_BR_INFO];
extern const char s6e36w4x01_vbias_10hz_0x83[MAX_BR_INFO];
extern const unsigned char s6e36w4x01_mps_con[MAX_BR_INFO];
extern const unsigned char s6e36w4x01_elvss[MAX_BR_INFO];
extern const unsigned char s6e36w4x01_aid_latest[MAX_BR_INFO*2];
extern const unsigned char s6e36w4x01_aid_0x83[MAX_BR_INFO*2];
extern const unsigned char s6e36w4x01_aid[MAX_BR_INFO*2];
extern const unsigned char s6e36w4x01_vint_latest[MAX_BR_INFO];
extern const unsigned char s6e36w4x01_vint_0x83[MAX_BR_INFO];

static unsigned char VBIAS_SETTING[] = {
	0xBA,
	0x21, 0x02, 0x32, 0x50,
	0xCC, 0x1E, 0x1E, 0x00
};

static unsigned char ELVSS_NORMAL[] = {
	0xB5,
	0x19, 0xDC, 0x07
};

static unsigned char ELVSS_HBM[] = {
	0xB5,
	0x19, 0xDC, 0x07
};

static unsigned char AID_SETTING[3] = {
	0xB1,
	0x00, 0x30,
};

static unsigned char VINT_SETTING[3] = {
	0xF4,
	0x00, 0x0
};

static const unsigned char HLPM_ON_ETC[] = {
	0xFD,
	0x88, 0x05, 0x3E, 0x80,
	0x0A, 0x00, 0xEC, 0x25,
	0x00, 0x00, 0x0A, 0xAA,
	0x44, 0xD0
};

static const char *mdnie_sc_str[] = {
	"SCENARIO_UI",
	"SCENARIO_GALLERY",
	"SCENARIO_VIDEO",
	"SCENARIO_VTCALL",
	"SCENARIO_CAMERA",
	"SCENARIO_BROWSER",
	"SCENARIO_NEGATIVE",
	"SCENARIO_EMAIL",
	"SCENARIO_EBOOK",
	"SCENARIO_GRAY",
	"SCENARIO_CURTAIN",
	"SCENARIO_GRAY_NEGATIVE",
	"SCENARIO_MAX",
};

static const char* get_mdnie_sc_str(enum mdnie_scenario scenario)
{
	if (scenario > SCENARIO_MAX)
		return "";

	return &mdnie_sc_str[scenario][0];
}

void s6e36w4x01_testkey_enable(u32 id)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) TEST_KEY_ON_0,
		ARRAY_SIZE(TEST_KEY_ON_0)) < 0)
		dsim_err("failed to send TEST_KEY_ON_0.\n");
}

void s6e36w4x01_testkey_disable(u32 id)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct lcd_device *panel = dsim->ld;
	struct s6e36w4x01 *lcd = dev_get_drvdata(&panel->dev);

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) TEST_KEY_OFF_0,
		ARRAY_SIZE(TEST_KEY_OFF_0)) < 0)
		dsim_err("%s: failed to send TEST_KEY_OFF_0.\n", __func__);

	mutex_unlock(&lcd->mipi_lock);
}

void s6e36w4x01_testkey_enable_lv3(u32 id)
{
	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) TEST_KEY_ON_FC,
		ARRAY_SIZE(TEST_KEY_ON_FC)) < 0)
		dsim_err("failed to send TEST_KEY_ON_FC.\n");
}

void s6e36w4x01_testkey_disable_lv3(u32 id)
{
	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) TEST_KEY_OFF_FC,
		ARRAY_SIZE(TEST_KEY_OFF_FC)) < 0)
		dsim_err("%s: failed to send TEST_KEY_OFF_FC.\n", __func__);
}

void s6e36w4x01_mdnie_outdoor_set(int id, enum mdnie_outdoor on)
{
	s6e36w4x01_testkey_enable(id);

	if (on) {
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) OUTDOOR_TUNE,
			ARRAY_SIZE(OUTDOOR_TUNE)) < 0)
			dsim_err("failed to send OUTDOOR_TUNE.\n");
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MDNIE_OUTD_ON,
			ARRAY_SIZE(MDNIE_OUTD_ON)) < 0)
			dsim_err("failed to send MDNIE_OUTD_ON.\n");
	} else
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MDNIE_CTL_OFF,
			ARRAY_SIZE(MDNIE_CTL_OFF)) < 0)
			dsim_err("failed to send MDNIE_CTL_OFF.\n");

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) PANEL_UPDATE,
		ARRAY_SIZE(PANEL_UPDATE)) < 0)
		dsim_err("failed to send PANEL_UPDATE.\n");

	s6e36w4x01_testkey_disable(id);

	return;
}

void s6e36w4x01_mdnie_set(u32 id, enum mdnie_scenario scenario)
{
	if (scenario >= SCENARIO_MAX) {
		dsim_err("%s: Invalid scenario (%d)\n", __func__, scenario);
		return;
	}

	s6e36w4x01_testkey_enable(id);

	switch (scenario) {
	case SCENARIO_GRAY:
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) GRAY_TUNE,
			ARRAY_SIZE(GRAY_TUNE)) < 0)
			dsim_err("failed to send GRAY_TUNE.\n");
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MDNIE_GRAY_ON,
			ARRAY_SIZE(MDNIE_GRAY_ON)) < 0)
			dsim_err("failed to send MDNIE_GRAY_ON.\n");
		break;
	case SCENARIO_NEGATIVE:
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) NEGATIVE_TUNE,
			ARRAY_SIZE(NEGATIVE_TUNE)) < 0)
			dsim_err("failed to send NEGATIVE_TUNE.\n");
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MDNIE_NEGATIVE_ON,
			ARRAY_SIZE(MDNIE_NEGATIVE_ON)) < 0)
			dsim_err("failed to send MDNIE_NEGATIVE_ON.\n");
		break;
	case SCENARIO_GRAY_NEGATIVE:
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) GRAY_NEGATIVE_TUNE,
			ARRAY_SIZE(GRAY_NEGATIVE_TUNE)) < 0)
			dsim_err("failed to send GRAY_NEGATIVE_TUNE.\n");
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MDNIE_GRAY_NEGATIVE_ON,
			ARRAY_SIZE(MDNIE_GRAY_NEGATIVE_ON)) < 0)
			dsim_err("failed to send MDNIE_GRAY_NEGATIVE_ON.\n");
		break;
	default:
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MDNIE_CTL_OFF,
			ARRAY_SIZE(MDNIE_CTL_OFF)) < 0)
			dsim_err("failed to send MDNIE_CTL_OFF.\n");
		break;
	}

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) PANEL_UPDATE,
		ARRAY_SIZE(PANEL_UPDATE)) < 0)
		dsim_err("failed to send PANEL_UPDATE.\n");

	s6e36w4x01_testkey_disable(id);

	pr_info("%s[%s]\n", __func__, get_mdnie_sc_str(scenario));

	return;
}


int s6e36w4x01_read_mtp_reg(int id, u32 addr, char* buffer, u32 size)
{
	int ret = 0;

	s6e36w4x01_testkey_enable(id);

	if(dsim_rd_data(id, MIPI_DSI_DCS_READ,
		addr, size, buffer) < 0) {
		dsim_err("%s: failed to read 0x%x reg\n", __func__, addr);
		ret = -EIO;
	}

	s6e36w4x01_testkey_disable(id);

	return ret;
}

static int s6e36w4x01_print_reg(char *reg, int size)
{
	int i, j, offset = 0;

	pr_info("%s:cmd[0x%02x], size[%d]\n", __func__, reg[0], size);

	size -= 1;

	for (i = 0; offset < size; i++) {
		pr_info("%s:[%2d]", __func__, i);
		for (j = 0; ((j < 10) && (offset < size)); j++) {
			offset = (i * 10) + j;
			pr_cont(" 0x%02x", reg[offset]);
		}
		pr_cont("\n");
	}
	return 0;
}

void s6e36w4x01_write_mtp_reg(int id, char* buffer, u32 size)
{
	s6e36w4x01_testkey_enable(id);

	if (size > 2) {
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) buffer,	size) < 0)
			dsim_err("failed to send long cmd.\n");
	} else {
		if(dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			buffer[0], buffer[1]) < 0)
			dsim_err("failed to send short cmd.\n");
	}

	s6e36w4x01_testkey_disable(id);

	s6e36w4x01_print_reg(buffer, size);
}

int s6e36w4x01_set_value_vbias(struct s6e36w4x01 *lcd, unsigned char value)
{
	struct dsim_device	*dsim = lcd->dsim;

	VBIAS_SETTING[3] = value;

	s6e36w4x01_write_mtp_reg(dsim->id, &VBIAS_SETTING[0], ARRAY_SIZE(VBIAS_SETTING));

	return 0;
}

unsigned char s6e36w4x01_get_value_vbias(void)
{
	return VBIAS_SETTING[3];
}

static const char enable_fd_first_sq[4][2] = {
	{0xb0, 0x03}, {0xb5, 0xaa},
	{0xb0, 0x24}, {0xb5, 0xb3}
};

static const char enable_fd_second_sq[4][2] = {
	{0xb0, 0x24}, {0xb5, 0x07},
	{0xb0, 0x03}, {0xb5, 0x2a}
};

void s6e36w4x01_enable_fd(struct s6e36w4x01 *lcd)
{
	struct dsim_device	*dsim = lcd->dsim;
	int i;

	/* Test Key Enable */
	s6e36w4x01_testkey_enable(dsim->id);

	msleep(60);

	for (i = 0; i < 4; i++) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			enable_fd_first_sq[i][0], enable_fd_first_sq[i][1]) < 0)
			dsim_err("failed to send enable_fd_first_sq[%d].\n", i);
	}

	/* 20ms delay */
	msleep(20);

	for (i = 0; i < 4; i++) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			enable_fd_second_sq[i][0], enable_fd_second_sq[i][1]) < 0)
			dsim_err("failed to send enable_fd_second_sq[%d].\n", i);
	}

	/* 120ms delay */
	msleep(120);

	/* Test key disable */
	s6e36w4x01_testkey_disable(dsim->id);

	pr_info("%s[%d]\n", __func__, lcd->enable_fd);
}

void s6e36w4x01_disable_fd(struct s6e36w4x01 *lcd)
{
	struct dsim_device	*dsim = lcd->dsim;
	int i;
	const char disable_fd_first_sq[4][2] = {
		{0xb0, 0x03}, {0xb5, 0xaa},
		{0xb0, 0x24}, {0xb5, 0xb2}
	};
	const char disable_fd_second_sq[4][2] = {
		{0xb0, 0x24}, {0xb5, 0x07},
		{0xb0, 0x03}, {0xb5, 0x2a}
	};

	/* Test Key Enable */
	s6e36w4x01_testkey_enable(dsim->id);

	for (i = 0; i < 4; i++) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			disable_fd_first_sq[i][0], disable_fd_first_sq[i][1]) < 0)
			dsim_err("failed to send disable_fd_first_sq[%d].\n", i);
	}

	/* 20ms delay */
	msleep(20);

	for (i = 0; i < 4; i++) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			disable_fd_second_sq[i][0], disable_fd_second_sq[i][1]) < 0)
			dsim_err("failed to send disable_fd_second_sq[%d].\n", i);
	}

	/* 120ms delay */
	msleep(120);

	/* Test key disable */
	s6e36w4x01_testkey_disable(dsim->id);

	pr_info("%s[%d]\n", __func__, lcd->enable_fd);
}

void s6e36w4x01_write_ddi_debug(struct s6e36w4x01 *lcd);
void s6e36w4x01_init_ctrl(int id, struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	int i;

	s6e36w4x01_testkey_enable(id);

	if(dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE,
		SLPOUT[0], 0) < 0)
		dsim_err("failed to send SLEEP_OUT.\n");

	msleep(20);

	if (lcd->enable_ddi_debug) {
		dsim_info("%s:enable ddi debug\n", __func__);
		s6e36w4x01_write_ddi_debug(lcd);
	}

	if (lcd->enable_fd) {
		msleep(100);
		dsim_info("%s:enable_fd\n", __func__);
		for (i = 0; i < 4; i++) {
			if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				enable_fd_first_sq[i][0], enable_fd_first_sq[i][1]) < 0)
				dsim_err("failed to send enable_fd_first_sq[%d].\n", i);
		}
		msleep(20);
		for (i = 0; i < 4; i++) {
			if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
				enable_fd_second_sq[i][0], enable_fd_second_sq[i][1]) < 0)
				dsim_err("failed to send enable_fd_second_sq[%d].\n", i);
		}
	}

	/* Common Setting */
	/* TE(Vsync) ON/OFF */
	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long)TEON,
		ARRAY_SIZE(TEON)) < 0)
		dsim_err("failed to send TE_ON.\n");

	/* Brightness Setting */
	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) GAMMA_SET,
		ARRAY_SIZE(GAMMA_SET)) < 0)
		dsim_err("failed to send GAMMA_SET.\n");

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) AID_SETTING,
		ARRAY_SIZE(AID_SETTING)) < 0)
		dsim_err("failed to send AID_SETTING.\n");

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) ELVSS_NORMAL,
		ARRAY_SIZE(ELVSS_NORMAL)) < 0)
		dsim_err("failed to send ELVSS_SET.\n");

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) F4_GPARA,
		ARRAY_SIZE(F4_GPARA)) < 0)
		dsim_err("failed to send F4_GPARA.\n");

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			NORMAL_VINT[0], NORMAL_VINT[1]) < 0)
			dsim_err("failed to send NORMAL_VINT\n");
	} else {
		if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			NORMAL_VINT_OLD[0], NORMAL_VINT_OLD[1]) < 0)
			dsim_err("failed to send NORMAL_VINT_OLD\n");
	}

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) VBIAS_SETTING,
		ARRAY_SIZE(VBIAS_SETTING)) < 0)
		dsim_err("failed to send VBIAS_SETTING.\n");

	if(dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		GAMMA_UPDATE[0], GAMMA_UPDATE[1]) < 0)
		dsim_err("failed to send GAMMA_UPDATE.\n");

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) FFC_ON,
		ARRAY_SIZE(FFC_ON)) < 0)
		dsim_err("failed to send FFC_ON.\n");

	if (lcd->enable_circle_mask) {
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) CIRCLE_MASK_ON,
			ARRAY_SIZE(CIRCLE_MASK_ON)) < 0)
			dsim_err("failed to send CIRCLE_MASK_ON.\n");
	} else {
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) CIRCLE_MASK_OFF,
			ARRAY_SIZE(CIRCLE_MASK_OFF)) < 0)
			dsim_err("failed to send CIRCLE_MASK_OFF.\n");
	}

	/* Test key disable */
	s6e36w4x01_testkey_disable(id);
}

void s6e36w4x01_enable(int id)
{
	/* Test Key Enable */
	s6e36w4x01_testkey_enable(id);

	/* display on */
	if(dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE,
		DISPLAY_ON[0],	0) < 0)
		dsim_err("failed to send DISPLAY_ON.\n");

	/* Test key disable */
	s6e36w4x01_testkey_disable(id);
}

/* follow Panel Power off sequence */
void s6e36w4x01_disable(int id)
{
	/* Test Key Enable */
	s6e36w4x01_testkey_enable(id);

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE,
		DISPLAY_OFF[0],0) < 0)
		dsim_err("fail to write DISPLAY_OFF .\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE,
		SLEEP_IN[0], 0) < 0)
		dsim_err("fail to write SLEEP_IN .\n");

	/* 120ms delay */
	msleep(120);

	/* Test key disable */
	s6e36w4x01_testkey_disable(id);
}

int s6e36w4x01_brightness_control(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	int ret;

	s6e36w4x01_testkey_enable(dsim->id);

	ret = dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) GAMMA_SET, ARRAY_SIZE(GAMMA_SET));
	if (ret < 0) {
		dsim_err("failed to send GAMMA_SET[%d]\n", ret);
		goto out;
	}

	ret = dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) AID_SETTING, ARRAY_SIZE(AID_SETTING));
	if (ret < 0) {
		dsim_err("failed to send AID_SETTING[%d]\n", ret);
		goto out;
	}

	ret = dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) ELVSS_NORMAL, ARRAY_SIZE(ELVSS_NORMAL));
	if (ret < 0) {
		dsim_err("failed to send ELVSS_NORMAL[%d]\n", ret);
		goto out;
	}

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_83) {
		ret = dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) F4_GPARA, ARRAY_SIZE(F4_GPARA));
		if (ret < 0) {
			dsim_err("failed to send F4_GPARA[%d]\n", ret);
			goto out;
		}

		ret = dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			VINT_SETTING[0], VINT_SETTING[1]);
		if (ret < 0) {
			dsim_err("failed to send VINT_SETTING[%d]\n", ret);
			goto out;
		}
	}

	ret = dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) VBIAS_SETTING, ARRAY_SIZE(VBIAS_SETTING));
	if (ret < 0) {
		dsim_err("failed to send VBIAS_SETTING[%d]\n", ret);
		goto out;
	}

	ret = dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		PANEL_UPDATE[0], PANEL_UPDATE[1]);
	if (ret < 0) {
		dsim_err("failed to send PANEL_UPDATE[%d]\n", ret);
		goto out;
	}

out:
	s6e36w4x01_testkey_disable(dsim->id);

	return ret;
}


#define 	VBIAS_60HZ	0x20
#define 	VBIAS_30HZ	0x21

#define 	VBIAS_NORMAL	0x32
#define 	VBIAS_AOD	0x36
int s6e36w4x01_gamma_ctrl(struct s6e36w4x01 *lcd, u32 level)
{
	struct dsim_device *dsim = lcd->dsim;
	int i, ret;

	if (lcd == (void *)NULL) {
		pr_err("%s: LCD is NULL\n", __func__);
		return 0;
	}

	for (i=0; i < GAMMA_CMD_CNT; i++) {
		GAMMA_SET[i] = s6e36w4x01_gamma_set[level][i];
	}

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		AID_SETTING[1] = s6e36w4x01_aid_latest[(level*2)];
		AID_SETTING[2] = s6e36w4x01_aid_latest[(level*2)+1];
	} else 	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_83) {
		AID_SETTING[1] = s6e36w4x01_aid_0x83[(level*2)];
		AID_SETTING[2] = s6e36w4x01_aid_0x83[(level*2)+1];
	} else {
		AID_SETTING[1] = s6e36w4x01_aid[(level*2)];
		AID_SETTING[2] = s6e36w4x01_aid[(level*2)+1];
	}

	ELVSS_NORMAL[2] = s6e36w4x01_mps_con[level];
	ELVSS_NORMAL[3] = s6e36w4x01_elvss[level];

	if (lcd->enable_hop)
		VBIAS_SETTING[1] = VBIAS_30HZ;
	else
		VBIAS_SETTING[1] = VBIAS_60HZ;

	VBIAS_SETTING[3] = VBIAS_NORMAL;

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias_30hz_latest[level]);
		VINT_SETTING[1] = s6e36w4x01_vint_latest[level];
	} else if (dsim->priv.panel_rev >= S6E36W4X01_ID3_83) {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias_30hz_0x83[level]);
		VINT_SETTING[1] = s6e36w4x01_vint_0x83[level];
	} else {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias[level]);
	}

	ret = s6e36w4x01_brightness_control(lcd);
	if (ret < 0) {
		pr_err("%s:failed brightness_control\n", __func__);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_NOBLESSE_SMALL
static unsigned char SMALL_HLPM_10HZ_84[] = {
	0xBA,
	0x21, 0x00, 0x32, 0xA0, 0xCC, 0x1E, 0x1E, 0x00
};

static unsigned char SMALL_HLPM_30HZ_84[] = {
	0xBA,
	0x20, 0x00, 0x32, 0xA0, 0xCC, 0x1E, 0x1E, 0x00
};
#define SMALL_HLPM_VBIAS_OFFSET	0x12
#endif

void s6e36w4x01_hlpm_on_30hz(struct s6e36w4x01 *lcd, bool enable)
{
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	struct backlight_device *bd = lcd->bd;
	int brightness = bd->props.brightness;

	if (lcd->enable_fd)
		s6e36w4x01_disable_fd(lcd);

	s6e36w4x01_testkey_enable(dsim->id);
	s6e36w4x01_testkey_enable_lv3(dsim->id);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_OPT_1[0], HLPM_OPT_1[1]) < 0)
		dsim_err("failed to send HLPM_OPT_1.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) HLPM_OPT_2,
		ARRAY_SIZE(HLPM_OPT_2)) < 0)
		dsim_err("failed to send HLPM_OPT_2.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_OPT_3[0], HLPM_OPT_3[1]) < 0)
		dsim_err("failed to send HLPM_OPT_3.\n");


	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_MODE_SET[0], HLPM_MODE_SET[1]) < 0)
		dsim_err("failed to send HLPM_MODE_SET.\n");

	if (brightness < BRIGHTNESS_LEVEL_40NIT) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			HLPM_5NIT_ON[0], HLPM_5NIT_ON[1]) < 0)
			dsim_err("failed to send HLPM_5NIT_ON.\n");
		lcd->hlpm_nit = HLPM_NIT_LOW;
	} else {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			HLPM_40NIT_ON[0], HLPM_40NIT_ON[1]) < 0)
			dsim_err("failed to send HLPM_40NIT_ON.\n");
		lcd->hlpm_nit = HLPM_NIT_HIGH;
	}

#ifdef CONFIG_NOBLESSE_SMALL
	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) SMALL_HLPM_30HZ_84, ARRAY_SIZE(SMALL_HLPM_30HZ_84)) < 0)
		dsim_err("failed to send SMALL_HLPM_30HZ_84.\n");
#else /* Large panel */
	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) LARGE_HLPM_30HZ_84, ARRAY_SIZE(LARGE_HLPM_30HZ_84)) < 0)
		dsim_err("failed to send LARGE_HLPM_30HZ_84.\n");
#endif

	s6e36w4x01_testkey_disable_lv3(dsim->id);
	s6e36w4x01_testkey_disable(dsim->id);

	pr_info("%s:hnit[%d]br[%d]id[%d]\n", __func__,
			lcd->hlpm_nit, brightness, panel->panel_rev);

	return;
}

void s6e36w4x01_hlpm_on_10hz(struct s6e36w4x01 *lcd, bool enable)
{
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *panel = &dsim->priv;
	struct backlight_device *bd = lcd->bd;
	int brightness = bd->props.brightness;

	if (lcd->enable_fd)
		s6e36w4x01_disable_fd(lcd);

#ifdef CONFIG_NOBLESSE_SMALL
	SMALL_HLPM_10HZ_84[8] = dsim->priv.vbias + SMALL_HLPM_VBIAS_OFFSET;
#endif

	s6e36w4x01_testkey_enable(dsim->id);
	s6e36w4x01_testkey_enable_lv3(dsim->id);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_OPT_1[0], HLPM_OPT_1[1]) < 0)
		dsim_err("failed to send HLPM_OPT_1.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) HLPM_OPT_2,
		ARRAY_SIZE(HLPM_OPT_2)) < 0)
		dsim_err("failed to send HLPM_OPT_2.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_OPT_3[0], HLPM_OPT_3[1]) < 0)
		dsim_err("failed to send HLPM_OPT_3.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_MODE_SET[0], HLPM_MODE_SET[1]) < 0)
		dsim_err("failed to send HLPM_MODE_SET.\n");

	if (brightness < BRIGHTNESS_LEVEL_40NIT) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			HLPM_5NIT_ON[0], HLPM_5NIT_ON[1]) < 0)
			dsim_err("failed to send HLPM_5NIT_ON.\n");
		lcd->hlpm_nit = HLPM_NIT_LOW;
	} else {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			HLPM_40NIT_ON[0], HLPM_40NIT_ON[1]) < 0)
			dsim_err("failed to send HLPM_40NIT_ON.\n");
		lcd->hlpm_nit = HLPM_NIT_HIGH;
	}

#ifdef CONFIG_NOBLESSE_SMALL
	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) SMALL_HLPM_10HZ_84, ARRAY_SIZE(SMALL_HLPM_10HZ_84)) < 0)
		dsim_err("failed to send SMALL_HLPM_10HZ_84.\n");
#else /* Large panel */
	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) LARGE_HLPM_10HZ_84, ARRAY_SIZE(LARGE_HLPM_10HZ_84)) < 0)
		dsim_err("failed to send LARGE_HLPM_10HZ_84.\n");
#endif

	s6e36w4x01_testkey_disable_lv3(dsim->id);
	s6e36w4x01_testkey_disable(dsim->id);

	pr_info("%s:hnit[%d]br[%d]id[%d]\n", __func__,
			lcd->hlpm_nit, brightness, panel->panel_rev);

	return;
}

void s6e36w4x01_hlpm_off(struct s6e36w4x01 *lcd, bool enable)
{
	struct dsim_device *dsim = lcd->dsim;
	struct backlight_device *bd = lcd->bd;
	int brightness = bd->props.brightness;

	s6e36w4x01_testkey_enable(dsim->id);
	s6e36w4x01_testkey_enable_lv3(dsim->id);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_OPT_1[0], HLPM_OPT_1[1]) < 0)
		dsim_err("failed to send HLPM_OPT_1.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) HLPM_OPT_2,
		ARRAY_SIZE(HLPM_OPT_2)) < 0)
		dsim_err("failed to send HLPM_OPT_2.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HLPM_OPT_3[0], HLPM_OPT_3[1]) < 0)
		dsim_err("failed to send HLPM_OPT_3.\n");

	if (lcd->hlpm_nit == HLPM_NIT_LOW) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			NORMAL_5nit_ON[0], NORMAL_5nit_ON[1]) < 0)
			dsim_err("failed to send NORMAL_5nit_ON.\n");
	} else {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			NORMAL_40nit_ON[0], NORMAL_40nit_ON[1]) < 0)
			dsim_err("failed to send NORMAL_40nit_ON.\n");
	}

	lcd->hlpm_nit = HLPM_NIT_OFF;
	s6e36w4x01_testkey_disable_lv3(dsim->id);
	s6e36w4x01_testkey_disable(dsim->id);

	if (lcd->enable_fd)
		s6e36w4x01_enable_fd(lcd);

	s6e36w4x01_gamma_ctrl(lcd, lcd->br_map[brightness]);

	pr_info("%s:br[%d]\n", __func__, brightness);

	return;
}

unsigned char s6e36w4x01_get_normal_aod_level(struct s6e36w4x01 *lcd)
{
	unsigned char aod_br_tbl[LPM_BR_MAX+1];
	enum lpm_level_t lpm_lv;

	aod_br_tbl[LPM_BR_LOW] = lcd->br_map[lcd->lpm_br_min];
	aod_br_tbl[LPM_BR_CHARGING] = lcd->br_map[lcd->lpm_br_charging];
	aod_br_tbl[LPM_BR_MID] = lcd->br_map[lcd->lpm_br_normal];
	aod_br_tbl[LPM_BR_HIGH] = lcd->br_map[lcd->lpm_br_max];

	if (lcd->br_level >= lcd->br_map[lcd->lpm_br_max])
		lpm_lv = LPM_BR_HIGH;
	else if (lcd->br_level >= lcd->br_map[lcd->lpm_br_normal])
		lpm_lv = LPM_BR_MID;
	else if (lcd->br_level >= lcd->br_map[lcd->lpm_br_charging])
		lpm_lv = LPM_BR_CHARGING;
	else
		lpm_lv = LPM_BR_LOW;

	if (lpm_lv < lcd->min_lpm_lv)
		lpm_lv = lcd->min_lpm_lv;

	return aod_br_tbl[lpm_lv];
}

void s6e36w4x01_normal_aod_on(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	unsigned char aod_level;
	int i, ret;

	if (lcd == NULL) {
		pr_err("%s: LCD is NULL\n", __func__);
		return;
	}

	aod_level = s6e36w4x01_get_normal_aod_level(lcd);

	for (i=0; i < GAMMA_CMD_CNT; i++) {
		GAMMA_SET[i] = s6e36w4x01_gamma_set[aod_level][i];
	}

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		AID_SETTING[1] = s6e36w4x01_aid_latest[(aod_level*2)];
		AID_SETTING[2] = s6e36w4x01_aid_latest[(aod_level*2)+1];
	} else 	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_83) {
		AID_SETTING[1] = s6e36w4x01_aid_0x83[(aod_level*2)];
		AID_SETTING[2] = s6e36w4x01_aid_0x83[(aod_level*2)+1];
	} else {
		AID_SETTING[1] = s6e36w4x01_aid[(aod_level*2)];
		AID_SETTING[2] = s6e36w4x01_aid[(aod_level*2)+1];
	}

	ELVSS_NORMAL[2] = s6e36w4x01_mps_con[aod_level];
	ELVSS_NORMAL[3] = s6e36w4x01_elvss[aod_level];

	VBIAS_SETTING[1] = VBIAS_30HZ;
	VBIAS_SETTING[3] = VBIAS_AOD;

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias_10hz_latest[aod_level]);
	} else if (dsim->priv.panel_rev >= S6E36W4X01_ID3_83) {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias_10hz_0x83[aod_level]);
	} else {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias[aod_level]);
	}

	ret = s6e36w4x01_brightness_control(lcd);
	if (ret < 0)
		pr_err("%s:failed brightness_control\n", __func__);

	pr_info("%s:br_lv[%d] aod_lv[%d]\n", __func__, lcd->br_level, aod_level);

	return;
}

void s6e36w4x01_normal_aod_off(struct s6e36w4x01 *lcd)
{
	struct backlight_device *bd = lcd->bd;
	int brightness = bd->props.brightness;
	int ret;

	ret = s6e36w4x01_gamma_ctrl(lcd, lcd->br_map[brightness]);
	if (ret) {
		pr_err("%s:failed s6e36w4x01_gamma_ctrl[%d]\n", __func__, ret);
		return;
	}

	pr_info("%s\n", __func__);

	return;
}

int s6e36w4x01_hbm_on(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	if (lcd == (void *)NULL) {
		pr_info("%s:LCD is NULL\n", __func__);
		return 0;
	}

	s6e36w4x01_testkey_enable(dsim->id);

	VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias;

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) ELVSS_HBM,
		ARRAY_SIZE(ELVSS_HBM)) < 0)
		dsim_err("failed to send ELVSS_HBM.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) F4_GPARA,
		ARRAY_SIZE(F4_GPARA)) < 0)
		dsim_err("failed to send F4_GPARA\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HBM_VINT[0], HBM_VINT[1]) < 0)
		dsim_err("failed to send HBM_VINT.\n");

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) VBIAS_SETTING,
		ARRAY_SIZE(VBIAS_SETTING)) < 0)
		dsim_err("failed to send VBIAS_SETTING.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HBM_ACL_ON[0], HBM_ACL_ON[1]) < 0)
		dsim_err("failed to send HBM_ACL_ON.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		ACL_ON[0], ACL_ON[1]) < 0)
		dsim_err("failed to send ACL_ON.\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HBM_ON[0], HBM_ON[1]) < 0)
		dsim_err("failed to send HBM_ON.\n");

	s6e36w4x01_testkey_disable(dsim->id);

	lcd->hbm_stime = ktime_get_boottime();

	pr_info("%s\n", __func__);

	return 0;
}

int s6e36w4x01_hbm_off(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	struct backlight_device *bd = lcd->bd;
	int brightness = bd->props.brightness;
	ktime_t hbm_time;

	if (lcd == (void *)NULL) {
		pr_info("%s:LCD is NULL\n", __func__);
		return 0;
	}

	s6e36w4x01_testkey_enable(dsim->id);

	VBIAS_SETTING[3] = VBIAS_NORMAL;

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias_30hz_latest[lcd->br_map[brightness]]);
	} else if (dsim->priv.panel_rev >= S6E36W4X01_ID3_83) {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias_30hz_0x83[lcd->br_map[brightness]]);
	} else {
		VBIAS_SETTING[S6E36W4_VBIAS_SIZE] = dsim->priv.vbias + (s6e36w4x01_vbias[lcd->br_map[brightness]]);
	}
	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) ELVSS_NORMAL,
		ARRAY_SIZE(ELVSS_NORMAL)) < 0)
		dsim_err("failed to send ELVSS_NORMAL\n");

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) VBIAS_SETTING,
		ARRAY_SIZE(VBIAS_SETTING)) < 0)
		dsim_err("failed to send VBIAS_SETTING\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HBM_ACL_OFF[0], HBM_ACL_OFF[1]) < 0)
		dsim_err("failed to send HBM_ACL_OFF\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		ACL_OFF[0], ACL_OFF[1]) < 0)
		dsim_err("failed to send ACL_OFF\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		HBM_OFF[0], HBM_OFF[1]) < 0)
		dsim_err("failed to send HBM_OFF\n");

	s6e36w4x01_testkey_disable(dsim->id);

	hbm_time = ktime_get_boottime();
	lcd->hbm_total_time += ((unsigned int)ktime_ms_delta(hbm_time, lcd->hbm_stime) / 1000);

	pr_info("%s\n", __func__);

	return 0;
}

void s6e36w4x01_set_offset_comp(struct s6e36w4x01 *lcd, int temperature)
{
	if (temperature < 0) {
		temperature *= (-1);
		ELVSS_NORMAL[1] = (0x80 | (temperature & 0xff));
	} else
		ELVSS_NORMAL[1] = (temperature & 0xff);

	dsim_info("%s:elvss[1]=[0x%02x]\n",	__func__, ELVSS_NORMAL[1]);
}

unsigned char s6e36w4x01_get_offset_comp(void)
{
	return ELVSS_NORMAL[1];
}

int s6e36w4x01_write_elvss_temp(int id)
{
	s6e36w4x01_testkey_enable(id);

	if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) ELVSS_NORMAL,
		ARRAY_SIZE(ELVSS_NORMAL)) < 0)
		dsim_err("failed to send ELVSS_NORMAL.\n");

	s6e36w4x01_testkey_disable(id);

	return 0;
}

enum temp_range_t s6e36w4x01_get_temp_stage(struct s6e36w4x01 *lcd, int temp)
{
	if (temp <= TEMP_REF_1)
		return TEMP_RANGE_2;
	else if (temp <= TEMP_REF_0)
		return TEMP_RANGE_1;
	else
		return TEMP_RANGE_0;
}

void s6e36w4x01_circle_mask_on(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	/* Test Key Enable */
	s6e36w4x01_testkey_enable(dsim->id);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) CIRCLE_MASK_ON,
		ARRAY_SIZE(CIRCLE_MASK_ON)) < 0)
		dsim_err("failed to send CIRCLE_MASK_ON.\n");

	/* Test key disable */
	s6e36w4x01_testkey_disable(dsim->id);
}

void s6e36w4x01_circle_mask_off(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	/* Test Key Enable */
	s6e36w4x01_testkey_enable(dsim->id);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) CIRCLE_MASK_OFF,
		ARRAY_SIZE(CIRCLE_MASK_OFF)) < 0)
		dsim_err("failed to send CIRCLE_MASK_OFF.\n");

	/* Test key disable */
	s6e36w4x01_testkey_disable(dsim->id);
}

void s6e36w4x01_mcd_test_on(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	if (lcd == (void *)NULL) {
		pr_info("%s: LCD is NULL\n", __func__);
		return;
	}

	s6e36w4x01_testkey_enable(dsim->id);

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MCD_TEST_ON_NOBLESSE,
			ARRAY_SIZE(MCD_TEST_ON_NOBLESSE)) < 0)
			dsim_err("failed to send MCD_TEST_ON_NOBLESSE\n");
	} else {
		if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MCD_TEST_ON_NOBLESSE_OLD,
			ARRAY_SIZE(MCD_TEST_ON_NOBLESSE_OLD)) < 0)
			dsim_err("failed to send MCD_TEST_ON_NOBLESSE_OLD\n");
	}

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		MCD_TIMMING_UPDATE[0], MCD_TIMMING_UPDATE[1]) < 0)
		dsim_err("failed to send MCD_TIMMING_UPDATE\n");

	s6e36w4x01_testkey_disable(dsim->id);

	msleep(100);
}

void s6e36w4x01_mcd_test_off(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	if (lcd == (void *)NULL) {
		pr_info("%s: LCD is NULL\n", __func__);
		return;
	}

	s6e36w4x01_testkey_enable(dsim->id);

	if (dsim->priv.panel_rev >= S6E36W4X01_ID3_84) {
		if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MCD_TEST_OFF_NOBLESSE,
			ARRAY_SIZE(MCD_TEST_OFF_NOBLESSE)) < 0)
			dsim_err("failed to send MCD_TEST_OFF_NOBLESSE\n");
	} else {
		if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) MCD_TEST_OFF_NOBLESSE_OLD,
			ARRAY_SIZE(MCD_TEST_OFF_NOBLESSE_OLD)) < 0)
			dsim_err("failed to send MCD_TEST_OFF_NOBLESSE_OLD\n");
	}


	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		MCD_TIMMING_UPDATE[0], MCD_TIMMING_UPDATE[1]) < 0)
		dsim_err("failed to send MCD_TIMMING_UPDATE\n");

	s6e36w4x01_testkey_disable(dsim->id);

	msleep(100);
}

int s6e36w4x01_print_debug_reg(int id)
{
	const char te_state_reg = 0x0a;
	const char panel_state_reg = 0x0e;
	char reg_1, reg_2;
	int ret;

	ret = dsim_rd_data(id, MIPI_DSI_DCS_READ, te_state_reg, 1, &reg_1);
	if (ret) {
		dsim_err("%s:failed to read te_state_reg\n", __func__);
		goto out;
	}

	ret = dsim_rd_data(id, MIPI_DSI_DCS_READ, panel_state_reg, 1, &reg_2);
	if (ret < 0) {
		dsim_err("%s:failed to read panel_state_reg\n", __func__);
		goto out;
	}

	pr_info("%s:0x%02x[0x%02x] 0x%02x[0x%02x]\n",
			__func__, te_state_reg, reg_1, panel_state_reg, reg_2);

out:
	return ret;
}

void s6e36w4x01_set_resolution(int id, int resolution)
{
	/* Test Key Enable */
	s6e36w4x01_testkey_enable(id);

	switch(resolution) {
	case 0:
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) CASET_360,
			ARRAY_SIZE(CASET_360)) < 0)
			dsim_err("failed to send CASET_360.\n");
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) PASET_360,
			ARRAY_SIZE(PASET_360)) < 0)
			dsim_err("failed to send PASET_360.\n");
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) OMOK_OFF,
			ARRAY_SIZE(OMOK_OFF)) < 0)
			dsim_err("failed to send OMOK_OFF.\n");
		break;
	case 1:
			if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) CASET_240,
			ARRAY_SIZE(CASET_240)) < 0)
			dsim_err("failed to send CASET_240.\n");
			if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long) PASET_240,
				ARRAY_SIZE(PASET_240)) < 0)
				dsim_err("failed to send PASET_240.\n");
			if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long) OMOK_240,
				ARRAY_SIZE(OMOK_240)) < 0)
				dsim_err("failed to send OMOK_240.\n");
		break;
	case 2:
			if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) CASET_180,
			ARRAY_SIZE(CASET_180)) < 0)
			dsim_err("failed to send CASET_180.\n");
			if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long) PASET_180,
				ARRAY_SIZE(PASET_180)) < 0)
				dsim_err("failed to send PASET_180.\n");
			if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long) OMOK_180,
				ARRAY_SIZE(OMOK_180)) < 0)
				dsim_err("failed to send OMOK_180.\n");
		break;

	default:
		pr_err("%s: unknown resolution\n", __func__);
	};
	/* Test key disable */
	s6e36w4x01_testkey_disable(id);

	pr_info("%s:[%d]\n", __func__, resolution);
}

void s6e36w4x01_write_ddi_debug(struct s6e36w4x01 *lcd)
{

	struct dsim_device *dsim = lcd->dsim;
	const char b0_first_cmd[] = {0xb0, 0x0a, 0xf2};
	const char enable_cmd[] = {0xf2, 0xc0};

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) b0_first_cmd,
		ARRAY_SIZE(b0_first_cmd)) < 0)
		dsim_err("failed to send b0_first_cmd.\n");

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			enable_cmd[0], enable_cmd[1]) < 0)
			dsim_err("failed to send enable_cmd.\n");
}

void s6e36w4x01_enable_hop(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	s6e36w4x01_testkey_enable(dsim->id);
	s6e36w4x01_testkey_enable_lv3(dsim->id);

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		NORMAL_30HZ[0], NORMAL_30HZ[1]) < 0)
		dsim_err("failed to send NORMAL_30HZ.\n");

	s6e36w4x01_testkey_disable_lv3(dsim->id);
	s6e36w4x01_testkey_disable(dsim->id);
}

void s6e36w4x01_disable_hop(struct s6e36w4x01 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	s6e36w4x01_testkey_enable(dsim->id);
	s6e36w4x01_testkey_enable_lv3(dsim->id);

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		NORMAL_60HZ[0], NORMAL_60HZ[1]) < 0)
		dsim_err("failed to send NORMAL_60HZ.\n");

	s6e36w4x01_testkey_disable_lv3(dsim->id);
	s6e36w4x01_testkey_disable(dsim->id);
}

int s6e36w4x01_read_init_info(struct dsim_device *dsim, unsigned char *mtp, unsigned char *hbm)
{
	struct panel_private *panel = &dsim->priv;
	unsigned char buf[S6E36W4_MTP_DATE_SIZE+1] = { 0, };
	unsigned char bufForCoordi[ S6E36W4_COORDINATE_LEN] = { 0, };
	unsigned char bufForProductDate[S6E36W4_PRODUCT_DATE_LEN] = {0, };
	int i = 0;
	int ret = 0;

	s6e36w4x01_testkey_enable(dsim->id);

	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,  S6E36W4_ID_REG,  S6E36W4_ID_LEN, panel->id);
	if (ret < 0) {
		dsim_err("%s : can't find connected panel. check panel connection\n", __func__);
		goto read_fail;
	}

	dsim_info("%s: READ ID:[0x%02x%02x%02x]\n",
		__func__, panel->id[0], panel->id[1], panel->id[2]);

	dsim->priv.current_model = (dsim->priv.id[1] >> 4) & 0x0F;
	dsim->priv.panel_rev = dsim->priv.id[2] & 0x0F;
	dsim->priv.panel_line = (dsim->priv.id[0] >> 6) & 0x03;
	dsim->priv.panel_material = dsim->priv.id[1] & 0x01;
	dsim_info("%s model is %d, panel rev:[%d], panel line:[%d], panel material:[%d]\n",
		__func__, dsim->priv.current_model, dsim->priv.panel_rev,
		dsim->priv.panel_line, dsim->priv.panel_material);

	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,  S6E36W4_VBIAS_ADDR,  S6E36W4_VBIAS_SIZE, buf);
	if (ret < 0) {
		dsim_err("failed to read vbias, check panel connection\n");
		goto read_fail;
	}
	dsim->priv.vbias = buf[S6E36W4_VBIAS_SIZE-1]; /* BAh 8th Para */
	dsim_info("%s:VBIAS:[0x%02x]\n", __func__, dsim->priv.vbias);

	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,  S6E36W4_MTP_ADDR,  S6E36W4_MTP_DATE_SIZE, buf);
	if (ret < 0) {
		dsim_err("failed to read mtp, check panel connection\n");
		goto read_fail;
	}
	memcpy(mtp, buf,  S6E36W4_MTP_SIZE);
	memcpy(panel->date, &buf[40], ARRAY_SIZE(panel->date));
	dsim_info("%s:READ MTP SIZE : %d\n", __func__,  S6E36W4_MTP_SIZE);
	dsim_info("%s:=========== MTP INFO =========== \n", __func__);
	for (i = 0; i <  S6E36W4_MTP_SIZE; i++)
		dsim_info("%s:MTP[%2d] : %02d : %02x\n", __func__, i, mtp[i], mtp[i]);

	// coordinate
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,  S6E36W4_COORDINATE_REG,  S6E36W4_COORDINATE_LEN, bufForCoordi);
	if (ret < 0) {
		dsim_err("fail to read coordinate on command.\n");
		goto read_fail;
	}
	dsim->priv.coordinate[0] = bufForCoordi[0] << 8 | bufForCoordi[1];      /* X */
	dsim->priv.coordinate[1] = bufForCoordi[2] << 8 | bufForCoordi[3];      /* Y */
	dsim_info("%s:READ coordi:[0x%x 0x%x]\n", __func__, panel->coordinate[0], panel->coordinate[1]);

	// product date
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ, S6E36W4_PRODUCT_DATE_REG, S6E36W4_PRODUCT_DATE_LEN, bufForProductDate);
	if (ret < 0) {
		dsim_err("fail to read product date on command.\n");
		goto read_fail;
	}
	memcpy(&dsim->priv.date, &bufForProductDate[S6E36W4_COORDINATE_LEN], S6E36W4_PRODUCT_DATE_LEN - S6E36W4_COORDINATE_LEN);
	dsim_info("%s:READ product date:[%d %d %d %d %d %d %d]\n", __func__,
		panel->date[0], panel->date[1], panel->date[2], panel->date[3], panel->date[4], panel->date[5], panel->date[6]);

	// code
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,  S6E36W4_CODE_REG,  S6E36W4_CODE_LEN, panel->code);
	if (ret < 0) {
		dsim_err("fail to read code on command.\n");
		goto read_fail;
	}
	dsim_info("%s:READ code:[%02x, %02x, %02x, %02x, %02x]\n", __func__,
			dsim->priv.code[0], dsim->priv.code[1], dsim->priv.code[2],
			dsim->priv.code[3], dsim->priv.code[4]);

	// tset
	panel->tset_set[0] =  S6E36W4_TSET_REG;
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,  S6E36W4_TSET_REG,  S6E36W4_TSET_LEN, &(panel->tset_set[1]));
	if (ret < 0) {
		dsim_err("fail to read code on command.\n");
		goto read_fail;
	}
	dsim_info("%s:READ tset:[%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x]\n", __func__,
			panel->tset_set[0], panel->tset_set[1], panel->tset_set[2], panel->tset_set[3],
			panel->tset_set[4], panel->tset_set[5], panel->tset_set[6], panel->tset_set[7]);

	 // elvss
	panel->elvss_set[0] = S6E36W4_ELVSS_REG;
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,  S6E36W4_ELVSS_REG,  S6E36W4_ELVSS_LEN, &(panel->elvss_set[1]));
	if (ret < 0) {
		dsim_err("fail to read elvss on command.\n");
		goto read_fail;
	}
	dsim_info("%s:READ elvss[%02x, %02x]\n", __func__, panel->elvss_set[0], panel->elvss_set[1]);

	// chip id
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ, LDI_CHIP_ID, LDI_CHIP_LEN, &(panel->chip[0]));
	if (ret < 0) {
		dsim_err("fail to read chip id on command.\n");
		goto read_fail;
	}
	dsim_info("%s:READ chip_id[%02x, %02x, %02x, %02x, %02x]\n", __func__,
			panel->chip[0], panel->chip[1], panel->chip[2],
			panel->chip[3], panel->chip[4]);

	ret = 0;
read_fail:
	s6e36w4x01_testkey_disable(dsim->id);

	return ret;
}
