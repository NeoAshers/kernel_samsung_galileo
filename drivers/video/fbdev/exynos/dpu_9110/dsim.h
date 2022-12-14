/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for Samsung EXYNOS SoC MIPI-DSI Master driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __SAMSUNG_DSIM_H__
#define __SAMSUNG_DSIM_H__

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <media/v4l2-subdev.h>
#include <soc/samsung/exynos-powermode.h>

#include "./panels/decon_lcd.h"
#include "./panels/common.h"

#if defined(CONFIG_SOC_EXYNOS9110)
#include "regs-dsim.h"
#else
#include "regs-dsim_8895.h"
#endif

#if defined(CONFIG_EXYNOS_DECON_LCD_S6E3HA2K)
#include "./panels/s6e3ha2k_param.h"
#elif defined(CONFIG_EXYNOS_DECON_LCD_S6E3HF4)
#include "./panels/s6e3hf4_param.h"
#elif defined(CONFIG_EXYNOS_DECON_LCD_EMUL_DISP)
#include "./panels/emul_disp_param.h"
#elif defined(CONFIG_EXYNOS_DECON_LCD_S6E3HA6)
#include "./panels/s6e3ha6_param.h"
#elif defined(CONFIG_EXYNOS_DECON_LCD_S6E3AA2)
#include "./panels/s6e3aa2_param.h"
#endif

#if 0
#if defined(CONFIG_EXYNOS_DECON_LCD_S6E36W2X01)
#include "./panels/s6e36w2x01_param.h"
#endif

#if defined(CONFIG_EXYNOS_DECON_LCD_S6E36W3X01)
#include "./panels/s6e36w3x01_param.h"
#endif
#endif

extern int dsim_log_level;

#define DSIM_MODULE_NAME		"exynos-dsim"
#define DSIM_DDI_ID_LEN			3

#if defined(CONFIG_SOC_EXYNOS9110)
#define DSIM_PIXEL_FORMAT_RGB24		0x3E
#define DSIM_PIXEL_FORMAT_RGB18_PACKED	0x1E
#define DSIM_PIXEL_FORMAT_RGB18		0x2E
#define DSIM_PIXEL_FORMAT_RGB30_PACKED		0x0D
#else
#define DSIM_PIXEL_FORMAT_RGB24		0x0
#define DSIM_PIXEL_FORMAT_RGB18_PACKED	0x1
#define DSIM_PIXEL_FORMAT_RGB18		0x2
#endif
#define DSIM_RX_FIFO_MAX_DEPTH		64
#define MAX_DSIM_CNT			1
#define MAX_DSIM_DATALANE_CNT		4

#define MIPI_WR_TIMEOUT			msecs_to_jiffies(50)
#define MIPI_RD_TIMEOUT			msecs_to_jiffies(100)

#define dsim_err(fmt, ...)							\
	do {									\
		if (dsim_log_level >= 3) {					\
			pr_err(pr_fmt(fmt), ##__VA_ARGS__);			\
		}								\
	} while (0)

#define dsim_warn(fmt, ...)							\
	do {									\
		if (dsim_log_level >= 4) {					\
			pr_warn(pr_fmt(fmt), ##__VA_ARGS__);			\
		}								\
	} while (0)

#define dsim_info(fmt, ...)							\
	do {									\
		if (dsim_log_level >= 6)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)

#define dsim_dbg(fmt, ...)							\
	do {									\
		if (dsim_log_level >= 7)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)

#define call_panel_ops(q, op, args...)				\
	(((q)->panel_ops->op) ? ((q)->panel_ops->op(args)) : 0)

extern struct dsim_device *dsim_drvdata[MAX_DSIM_CNT];
extern struct dsim_lcd_driver s6e3ha2k_mipi_lcd_driver;
extern struct dsim_lcd_driver emul_disp_mipi_lcd_driver;
extern struct dsim_lcd_driver s6e3hf4_mipi_lcd_driver;
extern struct dsim_lcd_driver s6e3ha6_mipi_lcd_driver;
extern struct dsim_lcd_driver s6e3ha8_mipi_lcd_driver;
extern struct dsim_lcd_driver s6e3aa2_mipi_lcd_driver;
extern struct dsim_lcd_driver s6e36w2x01_mipi_lcd_driver;
extern struct dsim_lcd_driver s6e36w3x01_mipi_lcd_driver;
extern struct dsim_lcd_driver s6e36w4x01_mipi_lcd_driver;
extern struct dsim_lcd_driver rm6d010_mipi_lcd_driver;
extern struct dsim_lcd_driver rm69330_mipi_lcd_driver;
extern struct dsim_lcd_driver rm69091_mipi_lcd_driver;

/* define video timer interrupt */
enum {
	DSIM_VBP = 0,
	DSIM_VSYNC,
	DSIM_V_ACTIVE,
	DSIM_VFP,
};

/* define dsi bist pattern */
enum {
	DSIM_COLOR_BAR = 0,
	DSIM_GRAY_GRADATION,
	DSIM_USER_DEFINED,
	DSIM_PRB7_RANDOM,
};

/* define DSI lane types. */
enum {
	DSIM_LANE_CLOCK	= (1 << 0),
	DSIM_LANE_DATA0	= (1 << 1),
	DSIM_LANE_DATA1	= (1 << 2),
	DSIM_LANE_DATA2	= (1 << 3),
	DSIM_LANE_DATA3	= (1 << 4),
};

/* DSI Error report bit definitions */
enum {
	MIPI_DSI_ERR_SOT			= (1 << 0),
	MIPI_DSI_ERR_SOT_SYNC			= (1 << 1),
	MIPI_DSI_ERR_EOT_SYNC			= (1 << 2),
	MIPI_DSI_ERR_ESCAPE_MODE_ENTRY_CMD	= (1 << 3),
	MIPI_DSI_ERR_LOW_POWER_TRANSMIT_SYNC	= (1 << 4),
	MIPI_DSI_ERR_HS_RECEIVE_TIMEOUT		= (1 << 5),
	MIPI_DSI_ERR_FALSE_CONTROL		= (1 << 6),
	/* Bit 7 is reserved */
	MIPI_DSI_ERR_ECC_SINGLE_BIT		= (1 << 8),
	MIPI_DSI_ERR_ECC_MULTI_BIT		= (1 << 9),
	MIPI_DSI_ERR_CHECKSUM			= (1 << 10),
	MIPI_DSI_ERR_DATA_TYPE_NOT_RECOGNIZED	= (1 << 11),
	MIPI_DSI_ERR_VCHANNEL_ID_INVALID	= (1 << 12),
	MIPI_DSI_ERR_INVALID_TRANSMIT_LENGTH	= (1 << 13),
	/* Bit 14 is reserved */
	MIPI_DSI_ERR_PROTOCAL_VIOLATION		= (1 << 15),
	/* DSI_PROTOCAL_VIOLATION[15] is for protocol violation that is caused EoTp
	 * missing So this bit is egnored because of not supportung @S.LSI AP */
	/* FALSE_ERROR_CONTROL[6] is for detect invalid escape or turnaround sequence.
	 * This bit is not supporting @S.LSI AP because of non standard
	 * ULPS enter/exit sequence during power-gating */
	/* Bit [14],[7] is reserved */
	MIPI_DSI_ERR_BIT_MASK			= (0x3f3f), /* Error_Range[13:0] */
};

/* operation state of dsim driver */
enum dsim_state {
	DSIM_STATE_INIT,
	DSIM_STATE_ON,		/* HS clock was enabled. */
	DSIM_STATE_ULPS,	/* DSIM was entered ULPS state */
	DSIM_STATE_OFF		/* DSIM is suspend state */
};

enum dphy_charic_value {
	M_PLL_CTRL1,
	M_PLL_CTRL2,
	B_DPHY_CTRL2,
	B_DPHY_CTRL3,
	B_DPHY_CTRL4,
	M_DPHY_CTRL1,
	M_DPHY_CTRL2,
	M_DPHY_CTRL3,
	M_DPHY_CTRL4
};

struct dsim_pll_param {
	u32 p;
	u32 m;
	u32 s;
#if defined(CONFIG_SOC_EXYNOS9110)
	u32 k;
#endif
	u32 pll_freq; /* in/out parameter: Mhz */
};

struct dsim_clks {
	u32 hs_clk;
	u32 esc_clk;
	u32 byte_clk;
	u32 word_clk;
};

struct dphy_timing_value {
	u32 bps;
	u32 clk_prepare;
	u32 clk_zero;
	u32 clk_post;
	u32 clk_trail;
	u32 hs_prepare;
	u32 hs_zero;
	u32 hs_trail;
	u32 lpx;
	u32 hs_exit;
	u32 b_dphyctl;
};

struct dsim_resources {
	struct clk *pclk;
	struct clk *dphy_esc;
	struct clk *dphy_byte;
	struct clk *rgb_vclk0;
	struct clk *pclk_disp;
	struct clk *aclk;
	int lcd_power[2];
	int lcd_reset;
	int err_fg;
	int ub_det;
	int irq;
	void __iomem *regs;
	void __iomem *ss_regs;
#if defined(CONFIG_SOC_EXYNOS9110)
	void __iomem *phy_regs;
#endif
	struct regulator *regulator_1p8v;
	struct regulator *regulator_3p3v;
};


struct panel_private {
	struct backlight_device *bd;
	unsigned char id[3];
	unsigned char code[5];

	unsigned char elvss_set[30];
	unsigned char elvss_len;
	unsigned char elvss_start_offset;
	unsigned char elvss_temperature_offset;
	unsigned char elvss_tset_offset;

	unsigned char tset_set[9];
	unsigned char tset_len;

	unsigned char aid_set[11];
	unsigned char aid_len;
	unsigned char aid_reg_offset;

	unsigned char vint_set[3];
	unsigned char vint_len;
	unsigned char vint_table[20];
	unsigned int vint_dim_table[20];
	unsigned char vint_table_len;

	unsigned char panel_rev;
	unsigned char panel_line;
	unsigned char panel_material;
	unsigned char current_model;

	unsigned int coordinate[2];
	unsigned char date[7];
	unsigned char		chip[5];
	unsigned int power;
	unsigned int lcdConnected;

	unsigned char vbias;

	void *dim_data;
	void *dim_info;
};

struct dsim_device {
	int id;
	enum dsim_state state;
	struct device *dev;
#ifdef CONFIG_SEC_SYSFS
	struct device *sec_dev;
#endif
	struct dsim_resources res;

	unsigned int data_lane;
	u32 data_lane_cnt;
	struct phy *phy;
	spinlock_t slock;

	struct dsim_lcd_driver *panel_ops;
	struct decon_lcd lcd_info;
	struct lcd_device	*ld;

	struct v4l2_subdev sd;
	struct dsim_clks clks;
	struct timer_list cmd_timer;

	struct mutex cmd_lock;

	struct completion ph_wr_comp;
	struct completion ph_wr_pl_comp;
	struct completion rd_comp;

	int total_underrun_cnt;
	int comm_err_cnt;
	int decon_timeout_cnt;
	bool decon_recov_working;

#ifdef CONFIG_DPHY_APB_CONTROL
	struct platform_device *dphy_apb_pdev;
#endif
	struct panel_private priv;

	int idle_ip_index;
#ifdef CONFIG_SEC_SYSFS
	struct delayed_work	ub_dwork;
	int ub_con_irq;
	int ub_con_intr_cnt;
	bool ub_con_cnt_en;
	bool ub_con_connected;
#endif
	/* true  - fb reserved     */
	/* false - fb not reserved */
	bool fb_reservation;
	phys_addr_t phys_addr;
	u32 init_step;
};

struct dsim_lcd_driver {
	int (*probe)(struct dsim_device *dsim);
	int (*suspend)(struct dsim_device *dsim);
	int (*displayon)(struct dsim_device *dsim);
	int (*resume)(struct dsim_device *dsim);
	int (*dump)(struct dsim_device *dsim);
	int (*aod_ctrl)(struct dsim_device *dsim, enum aod_state state, enum aod_mode aod_type);
	int (*metadata_set)(struct dsim_device *dsim, struct decon_metadata *metadata);
	int (*aod_time_set)(struct dsim_device *dsim, struct sclk_time_cfg_v2 *time_cfg);
	int (*aod_analog_set)(struct dsim_device *dsim, struct sclk_analog_cfg_v2 *analog_cfg);
	int (*aod_digital_set)(struct dsim_device *dsim, struct sclk_digital_cfg_v2 *digital_cfg);
	int (*aod_icon_set)(struct dsim_device *dsim, struct sclk_icon_cfg_v2 *icon_cfg);
	int (*aod_move_set)(struct dsim_device *dsim, struct sclk_move_cfg_v2 *move_cfg);
	int (*aod_mode_enter)(struct dsim_device *dsim, enum aod_mode watch_type);
	int (*aod_mode_exit)(struct dsim_device *dsim, enum aod_mode watch_type);
	int (*afpc_compensation_set)(struct dsim_device *dsim, struct afpc_compensation_v2 *comp);
	int (*afpc_panel_get)(struct dsim_device *dsim, struct afpc_panel_v2 *afpc_panel);
	int (*init)(struct dsim_device *dsim);
	int (*preinit)(struct dsim_device *dsim);
	int (*pre_reset)(struct dsim_device *dsim);
	int (*ub_con_det)(struct dsim_device *dsim, bool status);
	void (*show_state)(struct dsim_device *dsim);
	void (*temperature_set)(struct dsim_device *dsim, int temp);
};

int dsim_write_data(struct dsim_device *dsim, u32 id, unsigned long d0, u32 d1);
int dsim_read_data(struct dsim_device *dsim, u32 id, u32 addr, u32 cnt, u8 *buf);
int dsim_wait_for_cmd_done(struct dsim_device *dsim);
int dsim_store_timeout_count(struct dsim_device *dsim);

static inline struct dsim_device *get_dsim_drvdata(u32 id)
{
	return dsim_drvdata[id];
}

static inline int dsim_rd_data(u32 id, u32 cmd_id, u32 addr, u32 size, u8 *buf)
{
	int ret;
	struct dsim_device *dsim = get_dsim_drvdata(id);

	ret = dsim_read_data(dsim, cmd_id, addr, size, buf);
	if (ret)
		return ret;

	return 0;
}

static inline int dsim_wr_data(u32 id, u32 cmd_id, unsigned long d0, u32 d1)
{
	int ret;
	struct dsim_device *dsim = get_dsim_drvdata(id);

	ret = dsim_write_data(dsim, cmd_id, d0, d1);
	if (ret)
		return ret;

	return 0;
}

static inline int dsim_wait_for_cmd_completion(u32 id)
{
	int ret;
	struct dsim_device *dsim = get_dsim_drvdata(id);

	ret = dsim_wait_for_cmd_done(dsim);

	return ret;
}

/* register access subroutines */
static inline u32 dsim_read(u32 id, u32 reg_id)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	return readl(dsim->res.regs + reg_id);
}

static inline u32 dsim_read_mask(u32 id, u32 reg_id, u32 mask)
{
	u32 val = dsim_read(id, reg_id);
	val &= (mask);
	return val;
}

static inline void dsim_write(u32 id, u32 reg_id, u32 val)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	writel(val, dsim->res.regs + reg_id);
}

static inline void dsim_write_mask(u32 id, u32 reg_id, u32 val, u32 mask)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	u32 old = dsim_read(id, reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, dsim->res.regs + reg_id);
}

#if defined(CONFIG_SOC_EXYNOS9110)
/* DPHY register access subroutines */
static inline u32 dsim_phy_read(u32 id, u32 reg_id)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);

	return readl(dsim->res.phy_regs + reg_id);
}

static inline u32 dsim_phy_read_mask(u32 id, u32 reg_id, u32 mask)
{
	u32 val = dsim_phy_read(id, reg_id);

	val &= (mask);
	return val;
}
static inline void dsim_phy_write(u32 id, u32 reg_id, u32 val)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);

	writel(val, dsim->res.phy_regs + reg_id);
}

static inline void dsim_phy_write_mask(u32 id, u32 reg_id, u32 val, u32 mask)
{
	struct dsim_device *dsim = get_dsim_drvdata(id);
	u32 old = dsim_phy_read(id, reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, dsim->res.phy_regs + reg_id);
	/* printk("offset : 0x%8x, value : 0x%x\n", reg_id, val); */
}
#endif

/* CAL APIs list */
int dsim_reg_check_already_init(u32 id);
int dsim_reg_init(u32 id, struct decon_lcd *lcd_info,
			u32 data_lane_cnt, struct dsim_clks *clks);
int dsim_reg_set_clocks(u32 id, struct dsim_clks *clks,
			struct stdphy_pms *dphy_pms, u32 en);
int dsim_reg_set_lanes(u32 id, u32 lanes, u32 en);
void dsim_reg_set_int(u32 id, u32 en);
u32 dsim_reg_rx_fifo_is_empty(u32 id);
u32 dsim_reg_get_rx_fifo(u32 id);
int dsim_reg_rx_err_handler(u32 id, u32 rx_fifo);
void dsim_reg_sw_reset(u32 id);
void dsim_reg_dphy_reset(u32 id);
void dsim_reg_dphy_resetn(u32 id, u32 en);
int dsim_reg_exit_ulps_and_start(u32 id, u32 ddi_type, u32 lanes);
int dsim_reg_stop_and_enter_ulps(u32 id, u32 ddi_type, u32 lanes);
void dsim_reg_start(u32 id);
void dsim_reg_stop(u32 id, u32 lanes);
void dsim_reg_wr_tx_payload(u32 id, u32 payload);
u32 dsim_reg_header_fifo_is_empty(u32 id);
u32 dsim_reg_payload_fifo_is_empty(u32 id);
void dsim_reg_clear_int(u32 id, u32 int_src);
void dsim_reg_set_fifo_ctrl(u32 id, u32 cfg);
void dsim_reg_enable_shadow_read(u32 id, u32 en);
u32 dsim_reg_is_writable_fifo_state(u32 id);
void dsim_reg_wr_tx_header(u32 id, u32 data_id, unsigned long data0, u32 data1, u32 bta_type);
void dsim_set_bist(u32 id, u32 en);
void dsim_reg_set_partial_update(u32 id, struct decon_lcd *lcd_info);
void dsim_reg_set_bta_type(u32 id, u32 bta_type);
void dsim_reg_set_num_of_transfer(u32 id, u32 num_of_transfer);
void dsim_reg_set_cmd_transfer_mode(u32 id, u32 lp);

void dsim_reg_function_reset(u32 id);
void dsim_reg_set_esc_clk_on_lane(u32 id, u32 en, u32 lane);
void dsim_reg_enable_word_clock(u32 id, u32 en);
void dsim_reg_set_esc_clk_prescaler(u32 id, u32 en, u32 p);
u32 dsim_reg_is_pll_stable(u32 id);

#if defined(CONFIG_SOC_EXYNOS9110)
void dsim_reg_set_link_clock(u32 id, u32 en);
void dsim_reg_set_video_mode(u32 id, u32 mode);
void dsim_reg_enable_shadow(u32 id, u32 en);
#endif

#ifdef CONFIG_SEC_SYSFS
bool dsim_ub_get_conn_state(struct dsim_device *dsim);
#endif

#define DSIM_IOC_ENTER_ULPS		_IOW('D', 0, u32)
#define DSIM_IOC_GET_LCD_INFO		_IOW('D', 5, struct decon_lcd *)
#define DSIM_IOC_DUMP			_IOW('D', 8, u32)
#define DSIM_IOC_GET_WCLK		_IOW('D', 9, u32)

#if defined(CONFIG_SOC_EXYNOS9110)
#define DSIM_IOC_SET_CONFIG		_IOW('D', 10, u32)
#endif
#define DSIM_IOC_FREE_FB_RES	_IOW('D', 11, u32)
#define DSIM_IOC_SET_CMD_LPMODE _IOW('D', 50, u32)

#endif /* __SAMSUNG_DSIM_H__ */
