/* drivers/video/fbdev/exynos/dpu_9110/panels/rm69091_lcd_ctrl.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __RM69091_LCD_CTRL_H__
#define __RM69091_LCD_CTRL_H__

#include "decon_lcd.h"
#include "rm69091_mipi_lcd.h"

int rm69091_read_mtp_reg(int id, u32 addr, char* buffer, u32 size);
int rm69091_print_debug_reg(struct rm69091 *lcd);
void rm69091_init_ctrl(int id, struct decon_lcd *lcd);
void rm69091_enable(int id);
void rm69091_disable(int id);
int rm69091_gamma_ctrl(int id, u8 level);
int rm69091_hbm_on(struct rm69091 *lcd);
int rm69091_hbm_off(struct rm69091 *lcd);
int rm69091_pcd_test_on(struct rm69091 *lcd);
int rm69091_pcd_test_off(struct rm69091 *lcd);
int rm69091_normal_aod_on(struct rm69091 *lcd);
int rm69091_normal_aod_off(struct rm69091 *lcd);
#endif /*__rm69091_LCD_CTRL_H__*/
