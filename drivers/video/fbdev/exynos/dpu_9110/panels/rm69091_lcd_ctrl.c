/* rm69091_lcd_ctrl.c
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

#include "rm69091_param.h"
#include "rm69091_lcd_ctrl.h"
#include "rm69091_mipi_lcd.h"

#include "../dsim.h"
#include <video/mipi_display.h>

#define RM69091_ID3_81	0x81

#define ELVSS_TEMP_INDEX		1
#define ELVSS_RANGE_0_VALUE		0x1c
#define ELVSS_RANGE_1_VALUE		0x16
#define ELVSS_RANGE_2_VALUE		0x07

#define VINT_TEMP_INDEX		1
#define VINT_RANGE_0_VALUE		0x0C
#define VINT_RANGE_1_VALUE		0x17
#define VINT_RANGE_2_VALUE		0x21

static unsigned char RM69091_ELVSS[] = {
	0x6a,
	ELVSS_RANGE_0_VALUE, 0x00,
};

static unsigned char RM69091_VINT[] = {
	0x63,
	VINT_RANGE_0_VALUE, 0x00,
};

void rm69091_write_elvss_temp(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_01[0], RM69091_MANUFACTURE_01[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_01\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_ELVSS[0], RM69091_ELVSS[1]) < 0)
		pr_err("failed to send RM69091_ELVSS\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_VINT[0], RM69091_VINT[1]) < 0)
		pr_err("failed to send RM69091_VINT\n");

	mutex_unlock(&lcd->mipi_lock);
}

enum temp_range_t rm69091_get_temp_stage(int temp)
{
	if (temp > TEMP_REF_0)
		return TEMP_RANGE_0;
	else if (temp > TEMP_REF_1)
		return TEMP_RANGE_1;
	else
		return TEMP_RANGE_2;
}

void rm69091_set_offset_comp(enum temp_range_t range)
{
	if (range == TEMP_RANGE_0) {
		RM69091_ELVSS[ELVSS_TEMP_INDEX] = ELVSS_RANGE_0_VALUE;
		RM69091_VINT[VINT_TEMP_INDEX] = VINT_RANGE_0_VALUE;
	} else if (range == TEMP_RANGE_1) {
		RM69091_ELVSS[ELVSS_TEMP_INDEX] = ELVSS_RANGE_1_VALUE;
		RM69091_VINT[VINT_TEMP_INDEX] = VINT_RANGE_1_VALUE;
	} else if (range == TEMP_RANGE_2) {
		RM69091_ELVSS[ELVSS_TEMP_INDEX] = ELVSS_RANGE_2_VALUE;
		RM69091_VINT[VINT_TEMP_INDEX] = VINT_RANGE_2_VALUE;
	} else {
		RM69091_ELVSS[ELVSS_TEMP_INDEX] = ELVSS_RANGE_0_VALUE;
		RM69091_VINT[VINT_TEMP_INDEX] = VINT_RANGE_0_VALUE;
	}

	pr_info("%s:range[%d] elvss[%d]=[0x%02x] vint[%d]=[0x%02x]\n",
			__func__, range,
			ELVSS_TEMP_INDEX, RM69091_ELVSS[ELVSS_TEMP_INDEX],
			VINT_TEMP_INDEX, RM69091_VINT[VINT_TEMP_INDEX]);
}

unsigned char rm69091_get_offset_comp(void)
{
	return RM69091_ELVSS[ELVSS_TEMP_INDEX];
}

int rm69091_read_mtp_reg(int id, u32 addr, char* buffer, u32 size)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);
	int ret = 0;

	if (buffer == NULL) {
		pr_err("%s:buffer is NULL\n", __func__);
		return -ENXIO;
	}

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
		pr_err("failed to send RM69091_USER_CMD\n");

	if(dsim_rd_data(id, MIPI_DSI_DCS_READ,
		addr, size, buffer) < 0) {
		pr_err("%s: failed to read 0x%x reg\n", __func__, addr);
		ret = -EIO;
	}

	mutex_unlock(&lcd->mipi_lock);

	return ret;
}

int rm69091_read_cell_id(struct rm69091 *lcd, char* buffer)
{
	struct dsim_device *dsim = lcd->dsim;
	int i, ret = 0;

	if (buffer == NULL) {
		pr_err("%s:buffer is NULL\n", __func__);
		return -ENXIO;
	}

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_0B[0], RM69091_MANUFACTURE_0B[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_0B\n");

	for (i = 0; i < PANEL_ID_LEN; i++) {
		if(dsim_rd_data(dsim->id, MIPI_DSI_DCS_READ,
			PANEL_ID_ADDR+i, 1, &buffer[0+i]) < 0) {
			pr_err("%s: failed to read 0x%x reg\n", __func__, (PANEL_ID_ADDR+i));
			ret = -EIO;
			break;
		}
	}

	mutex_unlock(&lcd->mipi_lock);

	return ret;
}


static int rm69091_print_reg(char *reg, int size)
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

void rm69091_write_mtp_reg(int id, char* buffer, u32 size)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);

	mutex_lock(&lcd->mipi_lock);

	if (size > 2) {
		if(dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) buffer,	size) < 0)
			pr_err("failed to send long cmd.\n");
	} else {
		if(dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			buffer[0], buffer[1]) < 0)
			pr_err("failed to send short cmd.\n");
	}

	mutex_unlock(&lcd->mipi_lock);

	rm69091_print_reg(buffer, size);
}


int rm69091_print_debug_reg(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	int ret;

	const char panel_state_reg = 0x0a;
	char read_buf[1] = {0, };

	ret = rm69091_read_mtp_reg(dsim->id, panel_state_reg, &read_buf[0], ARRAY_SIZE(read_buf));
	if (ret) {
		pr_err("%s:failed to read 0x%02x reg[%d]\n", __func__, panel_state_reg, ret);
		return ret;
	}

	pr_info("%s:0x%02x[0x%02x]\n", __func__, panel_state_reg, read_buf[0]);

	return 0;
}

void rm69091_enable_fd(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	pr_info("%s\n", __func__);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_01[0], RM69091_MANUFACTURE_01[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_01\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_FD_SQC_01[0], RM69091_FD_SQC_01[1]) < 0)
		pr_err("failed to send RM69091_FD_SQC_01\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_FD_SQC_02[0], RM69091_FD_SQC_02[1]) < 0)
		pr_err("failed to send RM69091_FD_SQC_02\n");
}

void rm69091_init_ctrl(int id, struct decon_lcd * lcd)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct panel_private *private = &dsim->priv;
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *ddata = dev_get_drvdata(&panel->dev);

	mutex_lock(&ddata->mipi_lock);

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_USER_CMD,
		ARRAY_SIZE(RM69091_USER_CMD)) < 0)
		pr_err("failed to send RM69091_USER_CMD\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_TE_ON,
		ARRAY_SIZE(RM69091_TE_ON)) < 0)
		pr_err("failed to send RM69091_TE_ON\n");

	if (private->id[2] >= RM69091_ID3_81) {
		if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) RM69091_MANUFACTURE_01,
			ARRAY_SIZE(RM69091_MANUFACTURE_01)) < 0)
			pr_err("failed to send RM69091_MANUFACTURE_01\n");

		if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) RM69091_IDLE_EXT_POWER,
			ARRAY_SIZE(RM69091_IDLE_EXT_POWER)) < 0)
			pr_err("failed to send RM69091_IDLE_EXT_POWER\n");
	}

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_USER_CMD,
		ARRAY_SIZE(RM69091_USER_CMD)) < 0)
		pr_err("failed to send RM69091_USER_CMD\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_DISPLAY_CTRL,
		ARRAY_SIZE(RM69091_DISPLAY_CTRL)) < 0)
		pr_err("failed to send RM69091_DISPLAY_CTRL\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_COLUMN_SET,
		ARRAY_SIZE(RM69091_COLUMN_SET)) < 0)
		pr_err("failed to send RM69091_COLUMN_SET\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_ROW_SET,
		ARRAY_SIZE(RM69091_ROW_SET)) < 0)
		pr_err("failed to send RM69091_ROW_SET\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_DISP_BRIGHTNESS,
		ARRAY_SIZE(RM69091_DISP_BRIGHTNESS)) < 0)
		pr_err("failed to send RM69091_DISP_BRIGHTNESS\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_SLEEP_OUT,
		ARRAY_SIZE(RM69091_SLEEP_OUT)) < 0)
		pr_err("failed to send RM69091_SLEEP_OUT\n");

	if (!ddata->enable_curve) {
		if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) RM69091_MANUFACTURE_0C,
			ARRAY_SIZE(RM69091_MANUFACTURE_0C)) < 0)
			pr_err("failed to send RM69091_MANUFACTURE_0C\n");

		if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
			(unsigned long) RM69091_DISABLE_CURVE,
			ARRAY_SIZE(RM69091_DISABLE_CURVE)) < 0)
			pr_err("failed to send RM69091_DISABLE_CURVE\n");
	}

	msleep(50);

	mutex_unlock(&ddata->mipi_lock);
}

void rm69091_enable(int id)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *ddata = dev_get_drvdata(&panel->dev);

	mutex_lock(&ddata->mipi_lock);

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_DISP_ON,
		ARRAY_SIZE(RM69091_DISP_ON)) < 0)
		pr_err("failed to send RM69091_DISP_ON\n");

	if (ddata->enable_fd)
		rm69091_enable_fd(ddata);

	mutex_unlock(&ddata->mipi_lock);


	return;
}

void rm69091_disable(int id)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);

	mutex_lock(&lcd->mipi_lock);

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_DISP_OFF,
		ARRAY_SIZE(RM69091_DISP_OFF)) < 0)
		pr_err("failed to send RM69091_DISP_OFF\n");

	if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_SLEEP_IN,
		ARRAY_SIZE(RM69091_SLEEP_IN)) < 0)
		pr_err("failed to send RM69091_SLEEP_IN\n");

	msleep(83); /* Wait 5 Frames(83ms @60Hz) */

	mutex_unlock(&lcd->mipi_lock);

	return;
}

int rm69091_gamma_ctrl(int id, u8 level)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	struct lcd_device *panel = dsim->ld;
	struct rm69091 *lcd = dev_get_drvdata(&panel->dev);
	unsigned char gamma_cmd[3] = {0x51, 0x00, 0x00};

	mutex_lock(&lcd->mipi_lock);

	gamma_cmd[1] = level;
	lcd->br_level = gamma_cmd[1];

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_USER_CMD,
		ARRAY_SIZE(RM69091_USER_CMD)) < 0)
		pr_err("failed to send RM69091_USER_CMD\n");

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) gamma_cmd,
		ARRAY_SIZE(gamma_cmd)) < 0)
		pr_err("failed to send gamma_cmd\n");

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_MANUFACTURE_01,
		ARRAY_SIZE(RM69091_MANUFACTURE_01)) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_01\n");

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_ELVSS,
		ARRAY_SIZE(RM69091_ELVSS)) < 0)
		pr_err("failed to send RM69091_ELVSS\n");

	if (dsim_wr_data(dsim->id, MIPI_DSI_DCS_LONG_WRITE,
		(unsigned long) RM69091_VINT,
		ARRAY_SIZE(RM69091_VINT)) < 0)
		pr_err("failed to send RM69091_VINT\n");

	mutex_unlock(&lcd->mipi_lock);

	return 0;
}

int rm69091_hbm_on(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	struct dqa_data_t *dqa_data = &lcd->dqa_data;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
		pr_err("failed to send RM69091_USER_CMD\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_HBM_ON[0], RM69091_HBM_ON[1]) < 0)
		pr_err("failed to send RM69091_HBM_ON\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_01[0], RM69091_MANUFACTURE_01[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_01\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_HBM_DIMM[0], RM69091_HBM_DIMM[1]) < 0)
		pr_err("failed to send RM69091_HBM_DIMM\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
		pr_err("failed to send RM69091_USER_CMD\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_HBM_BRT[0], RM69091_HBM_BRT[1]) < 0)
		pr_err("failed to send RM69091_HBM_BRT\n");

	mutex_unlock(&lcd->mipi_lock);

	dqa_data->hbm_stime = ktime_get_boottime();

	pr_info("%s\n", __func__);

	return 0;
}

int rm69091_hbm_off(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	ktime_t now_time;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
		pr_err("failed to send RM69091_USER_CMD\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_HBM_OFF[0], RM69091_HBM_OFF[1]) < 0)
		pr_err("failed to send RM69091_HBM_OFF\n");

	mutex_unlock(&lcd->mipi_lock);

	now_time = ktime_get_boottime();
	dqa_data->hbm_time += ((unsigned int)ktime_ms_delta(now_time,
								dqa_data->hbm_stime) / 1000);

	pr_info("%s\n", __func__);

	return 0;
}

int rm69091_pcd_test_on(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_04[0], RM69091_MANUFACTURE_04[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_04\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_PCD_ON[0], RM69091_PCD_ON[1]) < 0)
		pr_err("failed to send RM69091_PCD_ON\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_0C[0], RM69091_MANUFACTURE_0C[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_0C\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_HIGH_07[0], RM69091_MUX_HIGH_07[1]) < 0)
		pr_err("failed to send RM69091_MUX_HIGH_07\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_HIGH_08[0], RM69091_MUX_HIGH_08[1]) < 0)
		pr_err("failed to send RM69091_MUX_HIGH_08\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_HIGH_09[0], RM69091_MUX_HIGH_09[1]) < 0)
		pr_err("failed to send RM69091_MUX_HIGH_09\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_HIGH_0A[0], RM69091_MUX_HIGH_0A[1]) < 0)
		pr_err("failed to send RM69091_MUX_HIGH_0A\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_HIGH_0B[0], RM69091_MUX_HIGH_0B[1]) < 0)
		pr_err("failed to send RM69091_MUX_HIGH_0B\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_HIGH_0C[0], RM69091_MUX_HIGH_0C[1]) < 0)
		pr_err("failed to send RM69091_MUX_HIGH_0C\n");

	mutex_unlock(&lcd->mipi_lock);

	pr_info("%s\n", __func__);

	return 0;
}

int rm69091_pcd_test_off(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_04[0], RM69091_MANUFACTURE_04[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_04\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_PCD_OFF[0], RM69091_PCD_OFF[1]) < 0)
		pr_err("failed to send RM69091_PCD_OFF\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_0C[0], RM69091_MANUFACTURE_0C[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_0C\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_NORMAL_07[0], RM69091_MUX_NORMAL_07[1]) < 0)
		pr_err("failed to send RM69091_MUX_NORMAL_07\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_NORMAL_08[0], RM69091_MUX_NORMAL_08[1]) < 0)
		pr_err("failed to send RM69091_MUX_NORMAL_08\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_NORMAL_09[0], RM69091_MUX_NORMAL_09[1]) < 0)
		pr_err("failed to send RM69091_MUX_NORMAL_09\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_NORMAL_0A[0], RM69091_MUX_NORMAL_0A[1]) < 0)
		pr_err("failed to send RM69091_MUX_NORMAL_0A\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_NORMAL_0B[0], RM69091_MUX_NORMAL_0B[1]) < 0)
		pr_err("failed to send RM69091_MUX_NORMAL_0B\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MUX_NORMAL_0C[0], RM69091_MUX_NORMAL_0C[1]) < 0)
		pr_err("failed to send RM69091_MUX_NORMAL_0C\n");

	mutex_unlock(&lcd->mipi_lock);

	pr_info("%s\n", __func__);

	return 0;
}

unsigned char rm69091_get_normal_aod_level(struct rm69091 *lcd)
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

int rm69091_normal_aod_on(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *private = &dsim->priv;
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	unsigned char aod_br_cmd[2] = {0x51, 0x52};
	ktime_t disp_time;

	aod_br_cmd[1] = rm69091_get_normal_aod_level(lcd);

	mutex_lock(&lcd->mipi_lock);

	if (private->id[2] >= RM69091_ID3_81) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
			pr_err("failed to send RM69091_USER_CMD\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			aod_br_cmd[0], aod_br_cmd[1]) < 0)
			pr_err("failed to send aod_br_cmd\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_LPM_ON[0], RM69091_LPM_ON[1]) < 0)
			pr_err("failed to send RM69091_LPM_ON\n");
	} else {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_MANUFACTURE_01[0], RM69091_MANUFACTURE_01[1]) < 0)
			pr_err("failed to send RM69091_MANUFACTURE_01\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_FRAME_CHANGE_30[0], RM69091_FRAME_CHANGE_30[1]) < 0)
			pr_err("failed to send RM69091_FRAME_CHANGE_30\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
			pr_err("failed to send RM69091_USER_CMD\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			aod_br_cmd[0], aod_br_cmd[1]) < 0)
			pr_err("failed to send aod_br_cmd\n");
	}

	mutex_unlock(&lcd->mipi_lock);

	pr_info("%s:br_lv[%d] aod_lv[%d]\n", __func__, lcd->br_level, aod_br_cmd[1]);

	dqa_data->aod_stime = ktime_get_boottime();
	disp_time = ktime_get();
	dqa_data->disp_time += ((unsigned int)ktime_ms_delta(disp_time, dqa_data->disp_stime) / 1000);

#if defined(ENABLE_PRINT_DISPLAY_DQA)
	rm69091_print_dqa(lcd, 1, 1);
#endif

	return 0;
}

int rm69091_normal_aod_off(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;
	struct panel_private *private = &dsim->priv;
	struct dqa_data_t *dqa_data = &lcd->dqa_data;
	struct backlight_device *bd = lcd->bd;
	unsigned char gamma_cmd[2] = {0x51, 0x00};
	int brightness = bd->props.brightness;
	ktime_t now_time;

	mutex_lock(&lcd->mipi_lock);

	if (private->id[2] >= RM69091_ID3_81) {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
			pr_err("failed to send RM69091_USER_CMD\n");

		gamma_cmd[1] = lcd->br_map[brightness];
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			gamma_cmd[0], gamma_cmd[1]) < 0)
			pr_err("failed to send gamma_cmd\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_LPM_OFF[0], RM69091_LPM_OFF[1]) < 0)
			pr_err("failed to send RM69091_LPM_OFF\n");
	} else {
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_USER_CMD[0], RM69091_USER_CMD[1]) < 0)
			pr_err("failed to send RM69091_USER_CMD\n");

		gamma_cmd[1] = lcd->br_map[brightness];
		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			gamma_cmd[0], gamma_cmd[1]) < 0)
			pr_err("failed to send gamma_cmd\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_MANUFACTURE_01[0], RM69091_MANUFACTURE_01[1]) < 0)
			pr_err("failed to send RM69091_MANUFACTURE_01\n");

		if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
			RM69091_FRAME_CHANGE_60[0], RM69091_FRAME_CHANGE_60[1]) < 0)
			pr_err("failed to send RM69091_FRAME_CHANGE_60\n");
	}

	mutex_unlock(&lcd->mipi_lock);

	rm69091_write_elvss_temp(lcd);

	now_time = ktime_get_boottime();
	dqa_data->aod_high_time += ((unsigned int)ktime_ms_delta(now_time,
				dqa_data->aod_stime) / 1000);

	dqa_data->disp_stime = ktime_get();
	dqa_data->disp_cnt++;
#if defined(ENABLE_PRINT_DISPLAY_DQA)
	rm69091_print_dqa(lcd, 0, 1);
#endif

	pr_info("%s:br_lv[%d] aod_lv[%d] temp_stage[%d]\n", __func__,
		lcd->br_level, gamma_cmd[1], lcd->temp_stage);

	return 0;
}

void rm69091_curve_on(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_0C[0], RM69091_MANUFACTURE_0C[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_0C\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_ACTIVE_180X[0], RM69091_ACTIVE_180X[1]) < 0)
		pr_err("failed to send RM69091_ACTIVE_180X\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_ACTIVE_180Y[0], RM69091_ACTIVE_180Y[1]) < 0)
		pr_err("failed to send RM69091_ACTIVE_180Y\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_ENABLE_CURVE[0], RM69091_ENABLE_CURVE[1]) < 0)
		pr_err("failed to send RM69091_ENABLE_CURVE\n");

	mutex_unlock(&lcd->mipi_lock);

}

void rm69091_curve_off(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_0C[0], RM69091_MANUFACTURE_0C[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_0C\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_DISABLE_CURVE[0], RM69091_DISABLE_CURVE[1]) < 0)
		pr_err("failed to send RM69091_DISABLE_CURVE\n");

	mutex_unlock(&lcd->mipi_lock);

}

void rm69091_crack_check_on(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_01[0], RM69091_MANUFACTURE_01[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_01\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_ENABLE_CRACK_CHECK[0], RM69091_ENABLE_CRACK_CHECK[1]) < 0)
		pr_err("failed to send RM69091_ENABLE_CRACK_CHECK\n");

	mutex_unlock(&lcd->mipi_lock);
}

void rm69091_crack_check_off(struct rm69091 *lcd)
{
	struct dsim_device *dsim = lcd->dsim;

	mutex_lock(&lcd->mipi_lock);

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_MANUFACTURE_0C[0], RM69091_MANUFACTURE_0C[1]) < 0)
		pr_err("failed to send RM69091_MANUFACTURE_0C\n");

	if(dsim_wr_data(dsim->id, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		RM69091_DISABLE_CRACK_CHECK[0], RM69091_DISABLE_CRACK_CHECK[1]) < 0)
		pr_err("failed to send RM69091_DISABLE_CRACK_CHECK\n");

	mutex_unlock(&lcd->mipi_lock);
}
