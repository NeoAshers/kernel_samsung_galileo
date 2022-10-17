/* rm69091_param.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *
 * SeungBeom, Park <sb1.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __RM69091_PARAM_H__
#define __RM69091_PARAM_H__

#define MIN_BRIGHTNESS			0
#define MAX_BRIGHTNESS			100
#define DEFAULT_BRIGHTNESS		MAX_BRIGHTNESS
#define HBM_BRIHGTNESS			120
#define HBM_DEFAULT_LEVEL		251
#define LPM_MAX_LEVEL			2

#define POWER_IS_ON(pwr)	((pwr) == FB_BLANK_UNBLANK)
#define POWER_IS_OFF(pwr)	((pwr) == FB_BLANK_POWERDOWN)
#define POWER_IS_NRM(pwr)	((pwr) == FB_BLANK_NORMAL)

#define LDI_CASET		0x2A
#define LDI_PASET		0x2B
#define LDI_MTP1_LEN		6
#define LDI_MTP2_LEN		12
#define LDI_MTP3_LEN		4
#define LDI_MTP4_LEN		32

#define PANEL_ID_ADDR		0x00
#define PANEL_ID_LEN		0x0b

#define LPM_BR_MAX_INDEX		44
#define LPM_BR_NORMAL_INDEX		40
#define LPM_BR_MIN_INDEX		1
#define LPM_BR_CHARGING_INDEX		20

static const unsigned char rm69091_br_map[MAX_BRIGHTNESS+1] = {
	40, 40, 41, 42, 42, 43, 44, 45, 46, 46,
	47, 48, 49, 49, 50, 51, 52, 53, 53, 54,
	55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
	66, 67, 68, 69, 70, 71, 72, 73, 74, 75,
	76, 77, 79, 80, 82, 83, 84, 86, 87, 89,
	90, 92, 93, 95, 96, 98, 100, 101, 103, 104,
	106, 108, 110, 112, 114, 116, 118, 120, 122, 124,
	126, 129, 132, 134, 137, 140, 143, 146, 148, 151,
	154, 157, 160, 162, 165, 168, 171, 174, 176, 179,
	182, 189, 196, 202, 209, 216, 224, 232, 239, 247,
	255,
};

static const unsigned char RM69091_USER_CMD[] = {
	0xfe,
	0x00, 0x00,
};

static const unsigned char RM69091_MANUFACTURE_01[] = {
	0xfe,
	0x01, 0x00,
};

static const unsigned char RM69091_MANUFACTURE_04[] = {
	0xfe,
	0x04, 0x00,
};

static const unsigned char RM69091_MANUFACTURE_0B[] = {
	0xfe,
	0x0b, 0x00,
};

static const unsigned char RM69091_MANUFACTURE_0C[] = {
	0xfe,
	0x0c, 0x00,
};

static const unsigned char RM69091_COLUMN_SET[] = {
	0x2a,
	0x00, 0x00, 0x01, 0x67,
};

static const unsigned char RM69091_ROW_SET[] = {
	0x2b,
	0x00, 0x00, 0x01, 0x67,
};

static const unsigned char RM69091_DISP_BRIGHTNESS[] = {
	0x51,
	0x00, 0x00,
};

static const unsigned char RM69091_ACTIVE_180X[] = {
	0x34,
	0xb4, 0x00,
};

static const unsigned char RM69091_ACTIVE_180Y[] = {
	0x35,
	0xb4, 0x00,
};

static const unsigned char RM69091_ENABLE_CURVE[] = {
	0x36,
	0x80, 0x00,
};

static const unsigned char RM69091_DISABLE_CURVE[] = {
	0x36,
	0x00, 0x00,
};

static const unsigned char RM69091_ENABLE_CRACK_CHECK[] = {
	0x3F,
	0x2E,
};

static const unsigned char RM69091_DISABLE_CRACK_CHECK[] = {
	0x3F,
	0x50,
};

static const unsigned char RM69091_50NIT_BRT[] = {
	0x51,
	0x52,
};

static const unsigned char RM69091_SLEEP_OUT[] = {
	0x11,
	0x00, 0x00,
};

static const unsigned char RM69091_SLEEP_IN[] = {
	0x10,
	0x00, 0x00,
};

static const unsigned char RM69091_TE_ON[] = {
	0x35,
	0x02, 0x00,
};

static const unsigned char RM69091_IDLE_EXT_POWER[] = {
	0x66,
	0x40, 0x00,
};

static const unsigned char RM69091_MANUFACTURE_CMD[] = {
	0xfe,
	0x07, 0x00,
};

static const unsigned char RM69091_DISPLAY_CTRL[] = {
	0x53,
	0x20, 0x00,
};

static const unsigned char RM69091_DISP_ON[] = {
	0x29,
	0x00, 0x00,
};

static const unsigned char RM69091_DISP_OFF[] = {
	0x28,
	0x00, 0x00,
};

static const unsigned char RM69091_LPM_ON[] = {
	0x39,
	0x00, 0x00,
};

static const unsigned char RM69091_LPM_OFF[] = {
	0x38,
	0x00, 0x00,
};

static const unsigned char RM69091_HBM_BRT[] = {
	0x63,
	0xff, 0x00,
};

static const unsigned char RM69091_HBM_DIMM[] = {
	0xc8,
	0x80, 0x00,
};

static const unsigned char RM69091_HBM_ON[] = {
	0x66,
	0x02, 0x00,
};

static const unsigned char RM69091_HBM_OFF[] = {
	0x66,
	0x00, 0x00,
};

static const unsigned char RM69091_GAMMA_SEL_98[] = {
	0xc5,
	0x98, 0x00,
};

static const unsigned char RM69091_GAMMA_SEL_18[] = {
	0xc5,
	0x18, 0x00,
};

static const unsigned char RM69091_FRAME_CHANGE_60[] = {
	0x29,
	0x01, 0x00,
};

static const unsigned char RM69091_FRAME_CHANGE_30[] = {
	0x29,
	0x41, 0x00,
};

static const unsigned char RM69091_PCD_ON[] = {
	0x78,
	0xfa, 0x00,
};

static const unsigned char RM69091_PCD_OFF[] = {
	0x78,
	0xff, 0x00,
};

static const unsigned char RM69091_MUX_HIGH_07[] = {
	0x07,
	0xff, 0x00,
};

static const unsigned char RM69091_MUX_HIGH_08[] = {
	0x08,
	0xff, 0x00,
};

static const unsigned char RM69091_MUX_HIGH_09[] = {
	0x09,
	0xff, 0x00,
};

static const unsigned char RM69091_MUX_HIGH_0A[] = {
	0x0a,
	0xff, 0x00,
};

static const unsigned char RM69091_MUX_HIGH_0B[] = {
	0x0b,
	0xff, 0x00,
};

static const unsigned char RM69091_MUX_HIGH_0C[] = {
	0x0c,
	0xff, 0x00,
};

static const unsigned char RM69091_MUX_NORMAL_07[] = {
	0x07,
	0xf1, 0x00,
};

static const unsigned char RM69091_MUX_NORMAL_08[] = {
	0x08,
	0xf2, 0x00,
};

static const unsigned char RM69091_MUX_NORMAL_09[] = {
	0x09,
	0xf3, 0x00,
};

static const unsigned char RM69091_MUX_NORMAL_0A[] = {
	0x0a,
	0xf4, 0x00,
};

static const unsigned char RM69091_MUX_NORMAL_0B[] = {
	0x0b,
	0xf5, 0x00,
};

static const unsigned char RM69091_MUX_NORMAL_0C[] = {
	0x0c,
	0xf6, 0x00,
};

static const unsigned char RM69091_FD_SQC_01[] = {
	0x69,
	0x12, 0x00,
};

static const unsigned char RM69091_FD_SQC_02[] = {
	0x6b,
	0x33, 0x00,
};
#endif
