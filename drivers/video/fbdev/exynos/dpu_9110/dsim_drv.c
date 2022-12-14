/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SoC MIPI-DSIM driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/exynos-ss.h>
#include <video/mipi_display.h>
#ifdef CONFIG_EXYNOS9110_BTS
#include <soc/samsung/cal-if.h>
#endif
#include <dt-bindings/clock/exynos9110.h>
#if defined(CONFIG_ION_EXYNOS)
#include <linux/exynos_iovmm.h>
#endif

#include <linux/of.h>
#include <linux/of_platform.h>

#include <linux/of_reserved_mem.h>
#include "../../../../../mm/internal.h"

#include <linux/sec_sysfs.h>
#include "decon.h"
#include "dsim.h"

#ifdef PHY_FRAMEWORK_DIS
void __iomem *dphy_isolation;
#endif

int dsim_log_level = 6;

struct dsim_device *dsim_drvdata[MAX_DSIM_CNT];
EXPORT_SYMBOL(dsim_drvdata);

static int dsim_runtime_suspend(struct device *dev);
static int dsim_runtime_resume(struct device *dev);

#ifdef CONFIG_SEC_SYSFS
static ssize_t dsim_store_ub_con_det(struct device *dev, struct device_attribute *devattr, const char *buf, size_t size);
static ssize_t dsim_show_ub_con_intr_cnt(struct device *dev, struct device_attribute *devattr, char *buf);
static ssize_t dsim_show_ub_gpio(struct device *dev, struct device_attribute *devattr, char *buf);
static irqreturn_t dsim_ub_con_det_irq_handler(int irq, void *dev_id);
static int dsim_ub_con_init(struct dsim_device *dsim);
#endif

extern int get_panel_id(void);

#ifdef CONFIG_DPHY_APB_CONTROL
static int dsim_dphy_apb_enable(struct dsim_device *dsim, u32 en)
{
/* cam power control
 * dphy apb is located in IS BLK
 * so IS BLK power should be turned on in order to access dphy apb SFR
 */
#ifdef CONFIG_EXYNOS_PD
	if (en) {
		pm_runtime_get_sync(&dsim->dphy_apb_pdev->dev);
		dsim_dbg("dphy apb enable\n");
	} else {
		pm_runtime_put_sync(&dsim->dphy_apb_pdev->dev);
		dsim_dbg("dphy apb disable\n");
	}
#endif

	return 0;
}
#endif

static void __dsim_dump(struct dsim_device *dsim)
{
	/* change to updated register read mode (meaning: SHADOW in DECON) */
	dsim_info("=== DSIM %d LINK SFR DUMP ===\n", dsim->id);
	dsim_reg_enable_shadow_read(dsim->id, 0);
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
			dsim->res.regs, 0xFC, false);

#if defined(CONFIG_SOC_EXYNOS9110)
#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif
	dsim_info("=== DSIM %d DPHY SFR DUMP ===\n", dsim->id);
	/* DPHY dump */
	/* PMSK */
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
		dsim->res.phy_regs + 0x0c00, 0x40, false);
	/* CLOCK lane */
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
		dsim->res.phy_regs + 0x1080, 0x40, false);

	/* Data lane : D0 */
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
		dsim->res.phy_regs + 0x1480, 0x30, false);

	/* Data lane : D1 */
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
		dsim->res.phy_regs + 0x1880, 0x30, false);

	/* Data lane : D2 */
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
		dsim->res.phy_regs + 0x1C80, 0x30, false);

	/* Data lane : D3 */
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
		dsim->res.phy_regs + 0x2080, 0x30, false);
#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif
#endif

	/* restore to avoid size mismatch (possible config error at DECON) */
	dsim_reg_enable_shadow_read(dsim->id, 1);
}

static void dsim_dump(struct dsim_device *dsim)
{
	dsim_info("=== DSIM SFR DUMP ===\n");
	__dsim_dump(dsim);

	/* Show panel status */
	call_panel_ops(dsim, dump, dsim);
}

static void dsim_long_data_wr(struct dsim_device *dsim, unsigned long d0, u32 d1)
{
	unsigned int data_cnt = 0, payload = 0;

	/* in case that data count is more then 4 */
	for (data_cnt = 0; data_cnt < d1; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((d1 - data_cnt) < 4) {
			if ((d1 - data_cnt) == 3) {
				payload = *(u8 *)(d0 + data_cnt) |
				    (*(u8 *)(d0 + (data_cnt + 1))) << 8 |
					(*(u8 *)(d0 + (data_cnt + 2))) << 16;
			dsim_dbg("count = 3 payload = %x, %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)));
			} else if ((d1 - data_cnt) == 2) {
				payload = *(u8 *)(d0 + data_cnt) |
					(*(u8 *)(d0 + (data_cnt + 1))) << 8;
			dsim_dbg("count = 2 payload = %x, %x %x\n", payload,
				*(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)));
			} else if ((d1 - data_cnt) == 1) {
				payload = *(u8 *)(d0 + data_cnt);
			}

			dsim_reg_wr_tx_payload(dsim->id, payload);
		/* send 4bytes per one time. */
		} else {
			payload = *(u8 *)(d0 + data_cnt) |
				(*(u8 *)(d0 + (data_cnt + 1))) << 8 |
				(*(u8 *)(d0 + (data_cnt + 2))) << 16 |
				(*(u8 *)(d0 + (data_cnt + 3))) << 24;

			dsim_dbg("count = 4 payload = %x, %x %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)),
				*(u8 *)(d0 + (data_cnt + 3)));

			dsim_reg_wr_tx_payload(dsim->id, payload);
		}
	}
}

static int dsim_wait_for_cmd_fifo_empty(struct dsim_device *dsim, bool must_wait)
{
	int ret = 0;

	if (!must_wait) {
		/* timer is running, but already command is transferred */
		if (dsim_reg_header_fifo_is_empty(dsim->id))
			del_timer(&dsim->cmd_timer);

		dsim_dbg("%s Doesn't need to wait fifo_completion\n", __func__);
		return ret;
	} else {
		del_timer(&dsim->cmd_timer);
		dsim_dbg("%s Waiting for fifo_completion...\n", __func__);
	}

	if (!wait_for_completion_timeout(&dsim->ph_wr_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_header_fifo_is_empty(dsim->id)) {
			reinit_completion(&dsim->ph_wr_comp);
			dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
			return 0;
		}
		ret = -ETIMEDOUT;
	}

	if ((dsim->state == DSIM_STATE_ON) && (ret == -ETIMEDOUT)) {
		dsim_err("%s have timed out\n", __func__);
		__dsim_dump(dsim);
	}
	return ret;
}

static int dsim_wait_for_cmd_pl_fifo_empty(struct dsim_device *dsim)
{
	int ret = 0;

	if (!wait_for_completion_timeout(&dsim->ph_wr_pl_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_payload_fifo_is_empty(dsim->id)) {
			reinit_completion(&dsim->ph_wr_pl_comp);
			dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PL_FIFO_EMPTY);
			return 0;
		}
		ret = -ETIMEDOUT;
	}

	if ((dsim->state == DSIM_STATE_ON) && (ret == -ETIMEDOUT)) {
		dsim_err("%s have timed out\n", __func__);
		__dsim_dump(dsim);
	}
	return ret;
}

/* wait for until SFR fifo is empty */
int dsim_wait_for_cmd_done(struct dsim_device *dsim)
{
	int ret = 0;
	/* FIXME: hiber only support for DECON0 */
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	ret = dsim_wait_for_cmd_fifo_empty(dsim, true);
	mutex_unlock(&dsim->cmd_lock);

	decon_hiber_unblock(decon);

	return ret;
}

static bool dsim_fifo_empty_needed(struct dsim_device *dsim, unsigned int data_id,
	unsigned long data0)
{
	/* read case or partial update command */
	if (data_id == MIPI_DSI_DCS_READ
			|| data0 == MIPI_DCS_SET_COLUMN_ADDRESS
			|| data0 == MIPI_DCS_SET_PAGE_ADDRESS) {
		dsim_dbg("%s: id:%d, data=%ld\n", __func__, data_id, data0);
		return true;
	}

	/* Check a FIFO level whether writable or not */
	if (!dsim_reg_is_writable_fifo_state(dsim->id))
		return true;

	return false;
}

int dsim_write_data(struct dsim_device *dsim, u32 id, unsigned long d0, u32 d1)
{
	int ret = 0;
	bool must_wait = true;
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	if (dsim->state != DSIM_STATE_ON) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		ret = -EINVAL;
		goto err_exit;
	}
	DPU_EVENT_LOG_CMD(&dsim->sd, id, d0);

	reinit_completion(&dsim->ph_wr_comp);
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);

	/* Run write-fail dectector */
	mod_timer(&dsim->cmd_timer, jiffies + MIPI_WR_TIMEOUT);

	switch (id) {
	/* short packet types of packet types for command. */
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
	case MIPI_DSI_DSC_PRA:
	case MIPI_DSI_COLOR_MODE_OFF:
	case MIPI_DSI_COLOR_MODE_ON:
	case MIPI_DSI_SHUTDOWN_PERIPHERAL:
	case MIPI_DSI_TURN_ON_PERIPHERAL:
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, false);
		must_wait = dsim_fifo_empty_needed(dsim, id, d0);
		break;

	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
	case MIPI_DSI_DCS_READ:
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, true);
		must_wait = dsim_fifo_empty_needed(dsim, id, d0);
		break;

	/* long packet types of packet types for command. */
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_DSC_PPS:
		reinit_completion(&dsim->ph_wr_pl_comp);
		dsim_long_data_wr(dsim, d0, d1);
		dsim_reg_wr_tx_header(dsim->id, id, d1 & 0xff,
				(d1 & 0xff00) >> 8, false);
		ret = dsim_wait_for_cmd_pl_fifo_empty(dsim);
		if (ret < 0)
			dsim_err("ID(%d): DSIM cmd wr pl timeout 0x%lx\n", id, d0);
		else
			must_wait = false;
		/* Empty state of PL fifo assure that PH fifo state is empty */
		/* so Checking PH fifo is not needed behind PL fifo empty */
		break;

	default:
		dsim_info("data id %x is not supported.\n", id);
		ret = -EINVAL;
	}

	ret = dsim_wait_for_cmd_fifo_empty(dsim, must_wait);
	if (ret < 0) {
		dsim->comm_err_cnt++;
		dsim_err("ID(%d): DSIM cmd wr timeout 0x%lx\n", id, d0);
	}

err_exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

int dsim_read_data(struct dsim_device *dsim, u32 id, u32 addr, u32 cnt, u8 *buf)
{
	u32 rx_fifo, rx_size = 0;
	int i, j, ret = 0;
	u32 rx_fifo_depth = DSIM_RX_FIFO_MAX_DEPTH;
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block_exit(decon);

	if (dsim->state != DSIM_STATE_ON) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		decon_hiber_unblock(decon);
		return -EINVAL;
	}

	reinit_completion(&dsim->rd_comp);

	/* Init RX FIFO before read and clear DSIM_INTSRC */
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_RX_DATA_DONE);

	/* Set the maximum packet size returned */
	dsim_write_data(dsim,
		MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, cnt, 0);

	/* Read request */
	dsim_write_data(dsim, id, addr, 0);
	if (!wait_for_completion_timeout(&dsim->rd_comp, MIPI_RD_TIMEOUT)) {
		dsim->comm_err_cnt++;
		dsim_err("MIPI DSIM read Timeout!\n");
		return -ETIMEDOUT;
	}

	mutex_lock(&dsim->cmd_lock);
	DPU_EVENT_LOG_CMD(&dsim->sd, id, (char)addr);

	do {
		rx_fifo = dsim_reg_get_rx_fifo(dsim->id);

		/* Parse the RX packet data types */
		switch (rx_fifo & 0xff) {
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			ret = dsim_reg_rx_err_handler(dsim->id, rx_fifo);
			if (ret < 0) {
				__dsim_dump(dsim);
				goto exit;
			}
			break;
		case MIPI_DSI_RX_END_OF_TRANSMISSION:
			dsim_dbg("EoTp was received from LCD module.\n");
			break;
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			dsim_dbg("Short Packet was received from LCD module.\n");
			for (i = 0; i <= cnt; i++)
				buf[i] = (rx_fifo >> (8 + i * 8)) & 0xff;
			break;
		case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
			dsim_dbg("Long Packet was received from LCD module.\n");
			rx_size = (rx_fifo & 0x00ffff00) >> 8;
			dsim_dbg("rx fifo : %8x, response : %x, rx_size : %d\n",
					rx_fifo, rx_fifo & 0xff, rx_size);
			/* Read data from RX packet payload */
			for (i = 0; i < rx_size >> 2; i++) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < 4; j++)
					buf[(i*4)+j] = (u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			if (rx_size % 4) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < rx_size % 4; j++)
					buf[4 * i + j] =
						(u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			break;
		default:
			dsim_err("Packet format is invaild.\n");
			__dsim_dump(dsim);
			ret = -EBUSY;
			goto exit;
		}
	} while (!dsim_reg_rx_fifo_is_empty(dsim->id) && --rx_fifo_depth);

	ret = rx_size;
	if (!rx_fifo_depth) {
		dsim_err("Check DPHY values about HS clk.\n");
		__dsim_dump(dsim);
		ret = -EBUSY;
	}
exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

int dsim_store_timeout_count(struct dsim_device *dsim)
{
	int ret;

	ret = decon_write_timeout_count(dsim->decon_timeout_cnt);

	return ret;
}

static void dsim_cmd_fail_detector(unsigned long arg)
{
	struct dsim_device *dsim = (struct dsim_device *)arg;
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block(decon);

	dsim_dbg("%s +\n", __func__);
	if (dsim->state != DSIM_STATE_ON) {
		dsim_err("%s: DSIM is not ready. state(%d)\n", __func__,
				dsim->state);
		goto exit;
	}

	/* If already FIFO empty even though the timer is no pending */
	if (!timer_pending(&dsim->cmd_timer)
			&& dsim_reg_header_fifo_is_empty(dsim->id)) {
		reinit_completion(&dsim->ph_wr_comp);
		dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
		goto exit;
	}

	__dsim_dump(dsim);

exit:
	decon_hiber_unblock(decon);
	dsim_dbg("%s -\n", __func__);
	return;
}

#ifdef CONFIG_EXYNOS9110_BTS
static void dsim_bts_print_info(struct bts_decon_info *info)
{
	int i;

	for (i = 0; i < BTS_DPP_MAX; ++i) {
		if (!info->dpp[i].used)
			continue;

		dsim_info("\t\tDPP[%d] b(%d) s(%d %d) d(%d %d %d %d) r(%d)\n",
				i, info->dpp[i].bpp,
				info->dpp[i].src_w, info->dpp[i].src_h,
				info->dpp[i].dst.x1, info->dpp[i].dst.x2,
				info->dpp[i].dst.y1, info->dpp[i].dst.y2,
				info->dpp[i].rotation);
	}
}
#endif

static void dsim_underrun_info(struct dsim_device *dsim)
{
#if defined(CONFIG_EXYNOS9110_BTS)
	struct decon_device *decon;
	int i;

	dsim_info("\tMIF(%lu), INT(%lu), DISP(%lu)\n",
			cal_dfs_get_rate(ACPM_DVFS_MIF),
			cal_dfs_get_rate(ACPM_DVFS_INT),
			cal_dfs_get_rate(ACPM_DVFS_DISP));

	for (i = 0; i < MAX_DECON_CNT; ++i) {
		decon = get_decon_drvdata(i);

		if (decon) {
			dsim_info("\tDECON%d: bw(%u %u), disp(%u %u), p(%u)\n",
					decon->id,
					decon->bts.prev_total_bw,
					decon->bts.total_bw,
					decon->bts.prev_max_disp_freq,
					decon->bts.max_disp_freq,
					decon->bts.peak);
			dsim_bts_print_info(&decon->bts.bts_info);
		}
	}
#endif
}

static irqreturn_t dsim_irq_handler(int irq, void *dev_id)
{
	unsigned int int_src;
	struct dsim_device *dsim = dev_id;
	struct decon_device *decon = get_decon_drvdata(0);
#ifdef CONFIG_EXYNOS_PD
	int active;
#endif

	spin_lock(&dsim->slock);

#ifdef CONFIG_EXYNOS_PD
	active = pm_runtime_active(dsim->dev);
	if (!active) {
		dsim_info("dsim power(%d), state(%d)\n", active, dsim->state);
		spin_unlock(&dsim->slock);
		return IRQ_HANDLED;
	}
#endif

	int_src = readl(dsim->res.regs + DSIM_INTSRC);

	if (int_src & DSIM_INTSRC_SFR_PH_FIFO_EMPTY) {
		del_timer(&dsim->cmd_timer);
		complete(&dsim->ph_wr_comp);
		dsim_dbg("dsim%d PH_FIFO_EMPTY irq occurs\n", dsim->id);
	}
	if (int_src & DSIM_INTSRC_SFR_PL_FIFO_EMPTY)
		complete(&dsim->ph_wr_pl_comp);
	if (int_src & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsim->rd_comp);
	if (int_src & DSIM_INTSRC_FRAME_DONE)
		dsim_dbg("dsim%d framedone irq occurs\n", dsim->id);
	if (int_src & DSIM_INTSRC_ERR_RX_ECC)
		dsim_err("RX ECC Multibit error was detected!\n");

	if (int_src & DSIM_INTSRC_UNDER_RUN) {
		dsim->total_underrun_cnt++;
		dsim_info("dsim%d underrun irq occurs(%d)\n", dsim->id,
				dsim->total_underrun_cnt);
		dsim_underrun_info(dsim);
	}
	if (int_src & DSIM_INTSRC_VT_STATUS) {
		dsim_dbg("dsim%d vt_status(vsync) irq occurs\n", dsim->id);
		if (decon) {
			decon->vsync.timestamp = ktime_get();
			wake_up_interruptible_all(&decon->vsync.wait);
			panel_vsync_wakeup_interrupt(decon->vsync.timestamp);
		}
	}

	dsim_reg_clear_int(dsim->id, int_src);

	spin_unlock(&dsim->slock);

	return IRQ_HANDLED;
}

/* DPHY RESET is controlled by IP */
void dpu_sysreg_set_dphy(struct dsim_device *dsim, void __iomem *sysreg)
{
	u32 val;

	val = SEL_RESET_DPHY_MASK(dsim->id);
	writel(val, sysreg + DISP_DPU_MIPI_PHY_CON);
}

void dpu_sysreg_dphy_resetn(struct dsim_device *dsim, void __iomem *sysreg, int en)
{
	u32 val;

	val = readl(sysreg + DISP_DPU_MIPI_PHY_CON);
	val = ( val & ~(1 << 0) ) | (en << 0);
	writel(val, sysreg + DISP_DPU_MIPI_PHY_CON);
}

static void dsim_clocks_info(struct dsim_device *dsim)
{
}

static int dsim_get_clocks(struct dsim_device *dsim)
{
	dsim->res.aclk = devm_clk_get(dsim->dev, "aclk");
	if (IS_ERR_OR_NULL(dsim->res.aclk)) {
		dsim_err("failed to get aclk\n");
		return PTR_ERR(dsim->res.aclk);
	}

	return 0;
}

static int dsim_get_gpios(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct dsim_resources *res = &dsim->res;

	dsim_dbg("%s +\n", __func__);

	if (of_get_property(dev->of_node, "gpios", NULL) != NULL)  {
		/* panel reset */
		res->lcd_reset = of_get_gpio(dev->of_node, 0);
		if (res->lcd_reset < 0) {
			dsim_err("failed to get lcd reset GPIO");
			return -ENODEV;
		}
		res->ub_det= of_get_gpio(dev->of_node, 1);
		if (res->ub_det < 0) {
			res->ub_det = -1;
			dsim_err("failed to get ub_det GPIO");
		}
		res->err_fg= of_get_gpio(dev->of_node, 2);
		if (res->err_fg < 0) {
			res->err_fg = -1;
			dsim_err("failed to get err_fg GPIO");
		}
		res->lcd_power[0] = of_get_gpio(dev->of_node, 3);
		if (res->lcd_power[0] < 0) {
			res->lcd_power[0] = -1;
		}
		res->lcd_power[1] = of_get_gpio(dev->of_node, 4);
		if (res->lcd_power[1] < 0) {
			res->lcd_power[1] = -1;
		}
	}

	dsim_dbg("%s -\n", __func__);
	return 0;
}

static int dsim_reset_panel(struct dsim_device *dsim)
{
	struct dsim_resources *res = &dsim->res;
	int ret;

	dsim_dbg("%s +\n", __func__);

	ret = gpio_request_one(res->lcd_reset, GPIOF_OUT_INIT_HIGH, "lcd_reset");
	if (ret < 0) {
		dsim_err("failed to get LCD reset GPIO\n");
		return -EINVAL;
	}
	usleep_range(5000, 6000);
	gpio_set_value(res->lcd_reset, 0);
	usleep_range(5000, 6000);
	gpio_set_value(res->lcd_reset, 1);
	usleep_range(10000, 11000);
	gpio_free(res->lcd_reset);

	dsim_dbg("%s -\n", __func__);
	return 0;
}

static int dsim_get_regulator(struct dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct dsim_resources *res = &dsim->res;

	char *str_regulator_1p8v = NULL;
	char *str_regulator_3p3v = NULL;

	res->regulator_1p8v = NULL;
	res->regulator_3p3v = NULL;

	if(!of_property_read_string(dev->of_node, "regulator_1p8v", (const char **)&str_regulator_1p8v)) {
		res->regulator_1p8v = regulator_get(dev, str_regulator_1p8v);
		if (IS_ERR(res->regulator_1p8v)) {
			dsim_err("%s : dsim regulator 1.8V get failed\n", __func__);
			regulator_put(res->regulator_1p8v);
			res->regulator_1p8v = NULL;
		}
	}

	if(!of_property_read_string(dev->of_node, "regulator_3p3v", (const char **)&str_regulator_3p3v)) {
		res->regulator_3p3v = regulator_get(dev, str_regulator_3p3v);
		if (IS_ERR(res->regulator_3p3v)) {
			dsim_err("%s : dsim regulator 3.3V get failed\n", __func__);
			regulator_put(res->regulator_3p3v);
			res->regulator_3p3v = NULL;
		}
	}

	return 0;
}

static int dsim_set_panel_power(struct dsim_device *dsim, bool on)
{
	struct dsim_resources *res = &dsim->res;
	int ret;

	dsim_dbg("%s(%d) +\n", __func__, on);

	if (on) {
		if (res->regulator_1p8v > 0) {
			ret = regulator_enable(res->regulator_1p8v);
			if (ret) {
				dsim_err("%s : dsim regulator 1.8V enable failed\n", __func__);
				return ret;
			}
			usleep_range(1000, 1100);
		}

		if (res->regulator_3p3v > 0) {
			ret = regulator_enable(res->regulator_3p3v);
			if (ret) {
				dsim_err("%s : dsim regulator 3.3V enable failed\n", __func__);
				return ret;
			}
			usleep_range(10000, 11000);
		}

		if (res->lcd_power[0] > 0) {
			ret = gpio_request_one(res->lcd_power[0],
					GPIOF_OUT_INIT_HIGH, "lcd_power0");
			if (ret < 0) {
				dsim_err("failed LCD power on\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[0]);
			usleep_range(10000, 11000);
		}

		if (res->lcd_power[1] > 0) {
			ret = gpio_request_one(res->lcd_power[1],
					GPIOF_OUT_INIT_HIGH, "lcd_power1");
			if (ret < 0) {
				dsim_err("failed 2nd LCD power on\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[1]);
			usleep_range(10000, 11000);
		}
	} else {
		ret = gpio_request_one(res->lcd_reset, GPIOF_OUT_INIT_LOW,
				"lcd_reset");
		if (ret < 0) {
			dsim_err("failed LCD reset off\n");
			return -EINVAL;
		}
		gpio_free(res->lcd_reset);

		if (res->lcd_power[0] > 0) {
			ret = gpio_request_one(res->lcd_power[0],
					GPIOF_OUT_INIT_LOW, "lcd_power0");
			if (ret < 0) {
				dsim_err("failed LCD power off\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[0]);
			usleep_range(11000, 12000);
		}

		if (res->lcd_power[1] > 0) {
			ret = gpio_request_one(res->lcd_power[1],
					GPIOF_OUT_INIT_LOW, "lcd_power1");
			if (ret < 0) {
				dsim_err("failed 2nd LCD power off\n");
				return -EINVAL;
			}
			gpio_free(res->lcd_power[1]);
			usleep_range(5000, 6000);
		}

		if (res->regulator_1p8v > 0) {
			ret = regulator_disable(res->regulator_1p8v);
			if (ret) {
				dsim_err("%s : dsim regulator 1.8V disable failed\n", __func__);
				return ret;
			} else {
				if (regulator_is_enabled(res->regulator_1p8v))
					dsim_err("%s:dsim regulator 1.8V is ON\n", __func__);
			}
		}

		if (res->regulator_3p3v > 0) {
			ret = regulator_disable(res->regulator_3p3v);
			if (ret) {
				dsim_err("%s : dsim regulator 3.3V disable failed\n", __func__);
				return ret;
			} else {
				if (regulator_is_enabled(res->regulator_3p3v))
					dsim_err("%s:dsim regulator 3.3V is ON\n", __func__);
			}
		}
	}

	dsim_dbg("%s(%d) -\n", __func__, on);

	return 0;
}

static int dsim_enable(struct dsim_device *dsim)
{
	int ret = 0;
	enum dsim_state state = dsim->state;

	if (dsim->state == DSIM_STATE_ON)
		return 0;

#if defined(CONFIG_EXYNOS_PD)
	pm_runtime_get_sync(dsim->dev);
#else
	dsim_runtime_resume(dsim->dev);
#endif

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

	/* DPHY power on : iso release */
#ifdef PHY_FRAMEWORK_DIS
	writel(0x1, dphy_isolation);
#else
	phy_power_on(dsim->phy);
#endif

	/* Panel power on */
	dsim_set_panel_power(dsim, 1);

	/* check whether the bootloader init has been done */
	if (dsim->state == DSIM_STATE_INIT) {
		if (dsim_reg_is_pll_stable(dsim->id)) {
			dsim_info("dsim%d PLL is stabled in bootloader, so skip DSIM link/DPHY init.\n", dsim->id);
			goto init_end;
		}
	}

	if (!dsim_reg_check_already_init(dsim->id)) {
		/* Panel pre reset should be set before LP-11 */
		call_panel_ops(dsim, pre_reset, dsim);
	}

#if defined(CONFIG_SOC_EXYNOS9110)
	/* choose OSC_CLK */
	dsim_reg_set_link_clock(dsim->id, 0);
#endif

	/* Enable DPHY reset : DPHY reset start */
#ifdef SEL_RESET_FROM_SYSREG
	dpu_sysreg_dphy_resetn(dsim, dsim->res.ss_regs, 0);
#else
	dpu_sysreg_set_dphy(dsim, dsim->res.ss_regs);
	dsim_reg_dphy_resetn(dsim->id, 0);
#endif

	dsim_reg_sw_reset(dsim->id);

	dsim_reg_set_clocks(dsim->id, &dsim->clks, &dsim->lcd_info.dphy_pms, 1);

	dsim_reg_set_lanes(dsim->id, dsim->data_lane, 1);

#ifdef SEL_RESET_FROM_SYSREG
	dpu_sysreg_dphy_resetn(dsim, dsim->res.ss_regs, 1);
#else
	/* Release DPHY reset */
	dsim_reg_dphy_resetn(dsim->id, 1);
#endif

#if defined(CONFIG_SOC_EXYNOS9110)
	/* Selection to word clock */
	dsim_reg_set_link_clock(dsim->id, 1);
#endif
	dsim_reg_set_esc_clk_on_lane(dsim->id, 1, dsim->data_lane);
	dsim_reg_enable_word_clock(dsim->id, 1);

	if (dsim_reg_init(dsim->id, &dsim->lcd_info, dsim->data_lane_cnt,
				&dsim->clks) < 0 ) {
		dsim_info("dsim_%d already enabled", dsim->id);
		ret = -EBUSY;
	} else {
		dsim_info("dsim_%d enabled", dsim->id);
		/* Panel reset should be set after LP-11 */
		if (!dsim->panel_ops->pre_reset)
			dsim_reset_panel(dsim);
	}

#ifdef CONFIG_SEC_SYSFS
#ifdef CONFIG_SEC_UB_DISCONNECT
	dsim->ub_con_connected = dsim_ub_get_conn_state(dsim);
	call_panel_ops(dsim, ub_con_det, dsim, dsim->ub_con_connected);
#else
	dsim->ub_con_connected = true;
#endif
	enable_irq(dsim->ub_con_irq);
#endif

init_end:
	dsim_reg_start(dsim->id);
	dsim->state = DSIM_STATE_ON;
	enable_irq(dsim->res.irq);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif

	if (state != DSIM_STATE_INIT) {
#if defined(CONFIG_EXYNOS_DECON_LCD_S6E3AA2)
		/* Panel lane # is changed to  2 by using LP mode*/
		dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif

		call_panel_ops(dsim, preinit, dsim);

#if defined(CONFIG_EXYNOS_DECON_LCD_S6E3AA2)
		/* MIPI mode is changed to HS mode*/
		dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif

		call_panel_ops(dsim, init, dsim);
	}

	return ret;
}

static int dsim_disable(struct dsim_device *dsim)
{
	if (dsim->state == DSIM_STATE_OFF)
		return 0;

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

#ifdef CONFIG_SEC_SYSFS
#ifdef CONFIG_SEC_UB_DISCONNECT
	cancel_delayed_work_sync(&dsim->ub_dwork);
#endif
	disable_irq(dsim->ub_con_irq);
#endif

	call_panel_ops(dsim, suspend, dsim);

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	del_timer(&dsim->cmd_timer);
	dsim->state = DSIM_STATE_OFF;
	mutex_unlock(&dsim->cmd_lock);

	dsim_reg_stop(dsim->id, dsim->data_lane);
	disable_irq(dsim->res.irq);

	/* HACK */
#ifdef PHY_FRAMEWORK_DIS
	writel(0x0, dphy_isolation);
#else
	phy_power_off(dsim->phy);
#endif
	dsim_set_panel_power(dsim, 0);

#if defined(CONFIG_EXYNOS_PD)
	pm_runtime_put_sync(dsim->dev);
#else
	dsim_runtime_suspend(dsim->dev);
#endif

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif

	return 0;
}

static int dsim_enter_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:+\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	if (dsim->state != DSIM_STATE_ON) {
		ret = -EBUSY;
		goto err;
	}

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	dsim->state = DSIM_STATE_ULPS;
	mutex_unlock(&dsim->cmd_lock);

	/* disable interrupts */
	dsim_reg_set_int(dsim->id, 0);

	disable_irq(dsim->res.irq);
	ret = dsim_reg_stop_and_enter_ulps(dsim->id, dsim->lcd_info.ddi_type,
			dsim->data_lane);
	if (ret < 0)
		dsim_dump(dsim);
#ifdef PHY_FRAMEWORK_DIS
	writel(0x0, dphy_isolation);
#else
	phy_power_off(dsim->phy);
#endif

#if defined(CONFIG_EXYNOS_PD)
	pm_runtime_put_sync(dsim->dev);
#else
	dsim_runtime_suspend(dsim->dev);
#endif

	DPU_EVENT_LOG(DPU_EVT_ENTER_ULPS, &dsim->sd, start);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif
err:
	dsim_dbg("%s -\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:-\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	return ret;
}

static int dsim_exit_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:+\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	if (dsim->state != DSIM_STATE_ULPS) {
		ret = -EBUSY;
		goto err;
	}

#if defined(CONFIG_EXYNOS_PD)
	pm_runtime_get_sync(dsim->dev);
#else
	dsim_runtime_resume(dsim->dev);
#endif

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

	/* DPHY power on : iso release */
#ifdef PHY_FRAMEWORK_DIS
	writel(0x1, dphy_isolation);
#else
	phy_power_on(dsim->phy);
#endif

	enable_irq(dsim->res.irq);

#if defined(CONFIG_SOC_EXYNOS9110)
	/* choose OSC_CLK */
	dsim_reg_set_link_clock(dsim->id, 0);
#endif

	/* Enable DPHY reset : DPHY reset start */
#ifdef SEL_RESET_FROM_SYSREG
	dpu_sysreg_dphy_resetn(dsim, dsim->res.ss_regs, 0);
#else
	dpu_sysreg_set_dphy(dsim, dsim->res.ss_regs);
	dsim_reg_dphy_resetn(dsim->id, 0);
#endif

	/* DSIM Link SW reset */
	dsim_reg_sw_reset(dsim->id);

	dsim_reg_set_clocks(dsim->id, &dsim->clks,
			&dsim->lcd_info.dphy_pms, 1);

	dsim_reg_set_lanes(dsim->id, dsim->data_lane, 1);

#ifdef SEL_RESET_FROM_SYSREG
	dpu_sysreg_dphy_resetn(dsim, dsim->res.ss_regs, 1);
#else
	/* Release DPHY reset */
	dsim_reg_dphy_resetn(dsim->id, 1);
#endif

#if defined(CONFIG_SOC_EXYNOS9110)
	dsim_reg_set_link_clock(dsim->id, 1);	/* Selection to word clock */
#endif

	dsim_reg_set_esc_clk_on_lane(dsim->id, 1, dsim->data_lane);
	dsim_reg_enable_word_clock(dsim->id, 1);

	if (dsim_reg_init(dsim->id, &dsim->lcd_info, dsim->data_lane_cnt,
				&dsim->clks) < 0 ) {
		dsim_info("dsim_%d already enabled", dsim->id);
		return -EBUSY;
	}
	ret = dsim_reg_exit_ulps_and_start(dsim->id, dsim->lcd_info.ddi_type,
			dsim->data_lane);
	if (ret < 0)
		dsim_dump(dsim);

	dsim->state = DSIM_STATE_ON;

	DPU_EVENT_LOG(DPU_EVT_EXIT_ULPS, &dsim->sd, start);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif
err:
	dsim_dbg("%s -\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:-\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	return 0;
}

static int dsim_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);

	if (enable)
		return dsim_enable(dsim);
	else
		return dsim_disable(dsim);
}

static int dsim_free_fb_resource(struct dsim_device *dsim)
{
	/* unmap */
	iovmm_unmap_oto(dsim->dev, dsim->phys_addr);

	/* unreserve memory */
	of_reserved_mem_device_release(dsim->dev);

	/* update state */
	dsim->fb_reservation = false;
	dsim->phys_addr = 0xdead;

	return 0;
}

static int dsim_acquire_fb_resource(struct dsim_device *dsim)
{
	int ret = 0;

	ret = of_reserved_mem_device_init_by_idx(dsim->dev, dsim->dev->of_node, 0);
	if (ret)
		dsim_err("failed reserved mem device init: %d\n", ret);
	else
		dsim->fb_reservation = true;

	ret = iovmm_map_oto(dsim->dev, dsim->phys_addr, dsim->lcd_info.xres * dsim->lcd_info.yres * 4);
	if (ret) {
		dsim_err("failed one to one mapping: %d\n", ret);
		BUG();
	}

	return ret;
}

static long dsim_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);
	int ret = 0;

	switch (cmd) {
	case DSIM_IOC_GET_LCD_INFO:
		v4l2_set_subdev_hostdata(sd, &dsim->lcd_info);
		break;

	case DSIM_IOC_ENTER_ULPS:
		if ((unsigned long)arg)
			ret = dsim_enter_ulps(dsim);
		else
			ret = dsim_exit_ulps(dsim);
		break;

	case DSIM_IOC_DUMP:
		dsim_dump(dsim);
		break;

	case DSIM_IOC_GET_WCLK:
		v4l2_set_subdev_hostdata(sd, &dsim->clks.word_clk);
		break;

	case EXYNOS_DPU_GET_ACLK:
		return clk_get_rate(dsim->res.aclk);

	case DSIM_IOC_SET_CMD_LPMODE:
		if ((unsigned long)arg) {
			/* Set Command LP mode */
			dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
		} else {
			/* Set Command HS mode */
			dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
		}
		break;

	case DSIM_IOC_FREE_FB_RES:
		ret = dsim_free_fb_resource(dsim);
		break;

	default:
		dsim_err("unsupported ioctl");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_core_ops dsim_sd_core_ops = {
	.ioctl = dsim_ioctl,
};

static const struct v4l2_subdev_video_ops dsim_sd_video_ops = {
	.s_stream = dsim_s_stream,
};

static const struct v4l2_subdev_ops dsim_subdev_ops = {
	.core = &dsim_sd_core_ops,
	.video = &dsim_sd_video_ops,
};

static void dsim_init_subdev(struct dsim_device *dsim)
{
	struct v4l2_subdev *sd = &dsim->sd;

	v4l2_subdev_init(sd, &dsim_subdev_ops);
	sd->owner = THIS_MODULE;
	sd->grp_id = dsim->id;
	snprintf(sd->name, sizeof(sd->name), "%s.%d", "dsim-sd", dsim->id);
	v4l2_set_subdevdata(sd, dsim);
}

static int dsim_cmd_sysfs_write(struct dsim_device *dsim, bool on)
{
	int ret = 0;

	if (on)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_ON, 0);
	else
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_OFF, 0);
	if (ret < 0)
		dsim_err("Failed to write test data!\n");
	else
		dsim_dbg("Succeeded to write test data!\n");

	return ret;
}

static int dsim_cmd_sysfs_read(struct dsim_device *dsim)
{
	int ret = 0;
	unsigned int id;
	u8 buf[4];

	/* dsim sends the request for the lcd id and gets it buffer */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
		MIPI_DCS_GET_DISPLAY_ID, DSIM_DDI_ID_LEN, buf);
	id = *(unsigned int *)buf;
	if (ret < 0)
		dsim_err("Failed to read panel id!\n");
	else
		dsim_info("Suceeded to read panel id : 0x%08x\n", id);

	return ret;
}

static ssize_t dsim_cmd_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

#ifdef CONFIG_SEC_SYSFS
#ifdef CONFIG_SEC_UB_DISCONNECT
#define	UB_DWORK_DELAY		3000 /* 3000 ms */

static void  dsim_ub_send_uevent(struct dsim_device *dsim)
{
	struct device *sec_dev = dsim->sec_dev;
	char *envp[] = {"PANEL_CONN=DISCONNECTED", NULL};

	kobject_uevent_env(&sec_dev->kobj, KOBJ_CHANGE, envp);
	schedule_delayed_work(&dsim->ub_dwork, msecs_to_jiffies(UB_DWORK_DELAY));

	dsim_info("%s[%s]\n", __func__, envp[0]);
}

static void dsim_ub_dwork(struct work_struct *work)
{
	struct dsim_device *dsim = container_of(work,
				struct dsim_device, ub_dwork.work);

	cancel_delayed_work(&dsim->ub_dwork);

	if (dsim->state == DSIM_STATE_OFF) {
		pr_err("%s:power off[%d]\n", __func__, dsim->state);
		return;
	}

	dsim_ub_send_uevent(dsim);
}
#endif

bool dsim_ub_get_conn_state(struct dsim_device *dsim)
{
	struct dsim_resources *res = &dsim->res;

	return !gpio_get_value(res->ub_det);
}

static irqreturn_t dsim_ub_con_det_irq_handler(int irq, void *dev_id)
{
	struct dsim_device *dsim = dev_id;
#ifdef CONFIG_SEC_UB_DISCONNECT
	int	ret;
#endif

	dsim->ub_con_connected = dsim_ub_get_conn_state(dsim);
	if (dsim->ub_con_connected) {
		dsim_dbg("%s:already connected[%d]\n", __func__,	dsim->ub_con_connected);
		goto out;
	}

#ifdef CONFIG_SEC_UB_DISCONNECT
	ret = call_panel_ops(dsim, ub_con_det, dsim, dsim->ub_con_connected);
	if(!ret)
		dsim_ub_send_uevent(dsim);
#endif

	if (dsim->ub_con_cnt_en)
		dsim->ub_con_intr_cnt++;

	dsim_info("%s:intr_cnt[%d] connected[%d]\n", __func__,
		dsim->ub_con_intr_cnt, dsim->ub_con_connected);

out:
	return IRQ_HANDLED;
}

#ifdef CONFIG_SEC_UB_DISCONNECT
static ssize_t dsim_store_ub_discon_test(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int cmd;

	sscanf(buf, "%d", &cmd);

	dsim_info("%s:[%d]\n", __func__, cmd);

	dsim_ub_send_uevent(dsim);

	return size;
};
#endif

static ssize_t dsim_store_ub_con_det(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	unsigned long cmd;
	int ret;

	ret = kstrtoul(buf, 0, &cmd);
	if (ret) {
		dsim_err("%s:failed buffer read[%d]\n", __func__, ret);
		return ret;
	}

	switch(cmd) {
	case 0:
		dsim->ub_con_cnt_en = 0;
		dsim_info("%s:disable ub_con_det test\n", __func__);
		break;
	case 1:
		dsim->ub_con_cnt_en = 1;
		dsim_info("%s:enable ub_con_det test\n", __func__);
		break;
	case 2:
		dsim->ub_con_intr_cnt = 0;
		dsim_info("%s:clear ub_con_det count\n", __func__);
		break;
	default:
		dsim_err("%s:unkown cmd[%lu]\n", __func__, cmd);
			break;
	}

	return size;
};

static ssize_t dsim_show_ub_con_intr_cnt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	dsim_info("show ub_con_det count[%d]\n", dsim->ub_con_intr_cnt);

	return snprintf(buf, PAGE_SIZE, "%d\n", dsim->ub_con_intr_cnt);
}

static ssize_t dsim_show_ub_gpio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	struct dsim_resources *res = &dsim->res;

	return snprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(res->ub_det));
}

#ifdef CONFIG_SEC_UB_DISCONNECT
static DEVICE_ATTR(ub_discon_test, S_IWUSR | S_IWGRP, NULL, dsim_store_ub_discon_test);
#endif
static DEVICE_ATTR(ub_con_det, S_IWUSR | S_IWGRP, NULL, dsim_store_ub_con_det);
static DEVICE_ATTR(ub_con_intr_cnt, S_IRUGO, dsim_show_ub_con_intr_cnt, NULL);
static DEVICE_ATTR(ub_gpio, S_IRUGO, dsim_show_ub_gpio, NULL);

static struct attribute *ub_con_attributes[] = {
#ifdef CONFIG_SEC_UB_DISCONNECT
	&dev_attr_ub_discon_test.attr,
#endif
	&dev_attr_ub_con_det.attr,
	&dev_attr_ub_con_intr_cnt.attr,
	&dev_attr_ub_gpio.attr,
	NULL,
};

static struct attribute_group ub_con_attr_group = {
	.attrs = ub_con_attributes,
};

static int dsim_ub_con_init(struct dsim_device *dsim)
{
	struct dsim_resources *res = &dsim->res;
	int ret = 0;

	if (dsim->sec_dev == NULL) {
		dsim_err("%s:sec_dev is NULL\n", __func__);
		return -ENODEV;
	}

	dsim->ub_con_irq = gpio_to_irq(res->ub_det);
	if (dsim->ub_con_irq < 0) {
		dsim_err("%s:Failed to get ub_det irq\n", __func__);
		return -1;
	}

#ifdef CONFIG_SEC_UB_DISCONNECT
	INIT_DELAYED_WORK(&dsim->ub_dwork, dsim_ub_dwork);
#endif
	ret = request_threaded_irq(dsim->ub_con_irq, NULL, dsim_ub_con_det_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ub_det", dsim);
	if (ret < 0) {
		dsim_err("%s:Failed to request det irq\n", __func__);
		return -1;
	}

	ret = sysfs_create_group(&dsim->sec_dev->kobj, &ub_con_attr_group);
	if (unlikely(ret)) {
		dsim_err("%s:Failed to create ub_con sysfs group\n", __func__);
		return -1;
	}

	return ret;
}

static int dsim_create_sec_class(struct dsim_device *dsim)
{
	if (dsim == NULL) {
		dsim_err("%s: dsim is NULL\n", __func__);
		return -ENODEV;
	}

	dsim->sec_dev = sec_device_create(dsim, "lcd");
	if (unlikely(!dsim->sec_dev)) {
		dsim_err("%s: failed to create sec_dev\n", __func__);
		return -ENODEV;
	}
	dev_set_drvdata(dsim->sec_dev, dsim);

	return 0;
}
#endif

static ssize_t dsim_cmd_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long cmd;
	struct dsim_device *dsim = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &cmd);
	if (ret)
		return ret;

	switch (cmd) {
	case 1:
		ret = dsim_cmd_sysfs_read(dsim);
		call_panel_ops(dsim, dump, dsim);
		if (ret)
			return ret;
		break;
	case 2:
		ret = dsim_cmd_sysfs_write(dsim, true);
		dsim_info("Dsim write command, display on!!\n");
		if (ret)
			return ret;
		break;
	case 3:
		ret = dsim_cmd_sysfs_write(dsim, false);
		dsim_info("Dsim write command, display off!!\n");
		if (ret)
			return ret;
		break;
	default :
		dsim_info("unsupportable command\n");
		break;
	}

	return count;
}
static DEVICE_ATTR(cmd_rw, 0644, dsim_cmd_sysfs_show, dsim_cmd_sysfs_store);

int dsim_create_cmd_rw_sysfs(struct dsim_device *dsim)
{
	int ret = 0;

	ret = device_create_file(dsim->dev, &dev_attr_cmd_rw);
	if (ret)
		dsim_err("failed to create command read & write sysfs\n");

	return ret;
}

#ifdef CONFIG_EXYNOS_DSIM_MULTI_PANEL_SUPPORT
int get_panel_vendor(void);
#endif
static void dsim_parse_lcd_info(struct dsim_device *dsim)
{
	u32 res[14];
	struct device_node *node;
	unsigned int mres_num = 1;
	u32 mres_w[3] = {0, };
	u32 mres_h[3] = {0, };
	u32 mres_dsc_w[3] = {0, };
	u32 mres_dsc_h[3] = {0, };
	u32 mres_dsc_en[3] = {0, };
	u32 hdr_num = 0;
	u32 hdr_type[HDR_CAPA_NUM] = {0, };
	u32 hdr_mxl = 0;
	u32 hdr_mal = 0;
	u32 hdr_mnl = 0;
	int k;

#ifdef CONFIG_EXYNOS_DSIM_MULTI_PANEL_SUPPORT
	int rc;
	const char *out_prop;
	u32 panel_info_num;
	int panel_info_boot = get_panel_vendor();

	rc = of_property_read_u32(dsim->dev->of_node, "panel_info_num",\
			(u32 *)&panel_info_num);
	if (rc) {
		dsim_err("%s: failed to get panel_info_num\n", __func__);
		return;
	}

	if ((panel_info_num == 0) || ((panel_info_num-1) < panel_info_boot)) {
		dsim_err("%s:invalidd panel info num[%d][%d]\n",
				__func__, panel_info_boot, panel_info_num);
		return;
	}

	if (panel_info_num > 1) {
		dsim->lcd_info.support_multi_panel = 1;
		dsim->lcd_info.panel_vendor = panel_info_boot;
	}

	rc = of_property_read_string_index(dsim->dev->of_node,
				"panel_info_name", panel_info_boot, &out_prop);
	if (rc < 0) {
		dsim_err("%s:failed to get %d map_id string[%s]\n", __func__, panel_info_boot, out_prop);
		return;
	}
	node = of_parse_phandle(dsim->dev->of_node, out_prop, 0);
	of_property_read_string(node, "model_name", &dsim->lcd_info.model_name);
#else
	of_property_read_string(dsim->dev->of_node, "model_name", &dsim->lcd_info.model_name);
	node = of_parse_phandle(dsim->dev->of_node, "lcd_info", 0);
#endif

#ifdef CONFIG_TIZEN
	dsim_info("panel model name [%s]\n",dsim->lcd_info.model_name);
	of_property_read_string(node, "panel_name", &dsim->lcd_info.name);
#endif
	of_property_read_u32(node, "mode", &dsim->lcd_info.mode);
	dsim_info("%s mode\n", dsim->lcd_info.mode ? "command" : "video");

	of_property_read_u32_array(node, "resolution", res, 2);
	dsim->lcd_info.xres = res[0];
	dsim->lcd_info.yres = res[1];
	dsim_info("LCD(%s) resolution: xres(%d), yres(%d)\n",
			of_node_full_name(node), res[0], res[1]);

	of_property_read_u32_array(node, "size", res, 2);
	dsim->lcd_info.width = res[0];
	dsim->lcd_info.height = res[1];
	dsim_dbg("LCD size: width(%d), height(%d)\n", res[0], res[1]);

	of_property_read_u32(node, "timing,refresh", &dsim->lcd_info.fps);
	dsim_dbg("LCD refresh rate(%d)\n", dsim->lcd_info.fps);

	of_property_read_u32_array(node, "timing,h-porch", res, 3);
	dsim->lcd_info.hbp = res[0];
	dsim->lcd_info.hfp = res[1];
	dsim->lcd_info.hsa = res[2];
	dsim_dbg("hbp(%d), hfp(%d), hsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32_array(node, "timing,v-porch", res, 3);
	dsim->lcd_info.vbp = res[0];
	dsim->lcd_info.vfp = res[1];
	dsim->lcd_info.vsa = res[2];
	dsim_dbg("vbp(%d), vfp(%d), vsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32(node, "timing,dsi-hs-clk", &dsim->lcd_info.hs_clk);
	dsim->clks.hs_clk = dsim->lcd_info.hs_clk;
	dsim_dbg("requested hs clock(%d)\n", dsim->lcd_info.hs_clk);

#if defined(CONFIG_EXYNOS_DSIM_DITHER)
	of_property_read_u32_array(node, "timing,pmsk", res, 14);
#else
	of_property_read_u32_array(node, "timing,pmsk", res, 4);
#endif
	dsim->lcd_info.dphy_pms.p = res[0];
	dsim->lcd_info.dphy_pms.m = res[1];
	dsim->lcd_info.dphy_pms.s = res[2];
	dsim->lcd_info.dphy_pms.k = res[3];
	dsim_dbg("p(%d), m(%d), s(%d), k(%d)\n", res[0], res[1], res[2], res[3]);
#if defined(CONFIG_EXYNOS_DSIM_DITHER)
	dsim->lcd_info.dphy_pms.mfr = res[4];
	dsim->lcd_info.dphy_pms.mrr = res[5];
	dsim->lcd_info.dphy_pms.sel_pf = res[6];
	dsim->lcd_info.dphy_pms.icp = res[7];
	dsim->lcd_info.dphy_pms.afc_enb = res[8];
	dsim->lcd_info.dphy_pms.extafc = res[9];
	dsim->lcd_info.dphy_pms.feed_en = res[10];
	dsim->lcd_info.dphy_pms.fsel = res[11];
	dsim->lcd_info.dphy_pms.fout_mask = res[12];
	dsim->lcd_info.dphy_pms.rsel = res[13];
	dsim_dbg(" mfr(%d), mrr(0x%x), sel_pf(%d), icp(%d)\n",
				res[4], res[5], res[6], res[7]);
	dsim_dbg(" afc_enb(%d), extafc(%d), feed_en(%d), fsel(%d)\n",
				res[8], res[9], res[10], res[11]);
	dsim_dbg(" fout_mask(%d), rsel(%d)\n", res[12], res[13]);
#endif

	of_property_read_u32(node, "timing,dsi-escape-clk",
			&dsim->lcd_info.esc_clk);
	dsim->clks.esc_clk = dsim->lcd_info.esc_clk;
	dsim_dbg("requested escape clock(%d)\n", dsim->lcd_info.esc_clk);

	of_property_read_u32(node, "mic_en", &dsim->lcd_info.mic_enabled);
	dsim_info("mic enabled (%d)\n", dsim->lcd_info.mic_enabled);

	of_property_read_u32(node, "type_of_ddi", &dsim->lcd_info.ddi_type);
	dsim_dbg("ddi type(%d)\n", dsim->lcd_info.ddi_type);

	of_property_read_u32(node, "dsc_en", &dsim->lcd_info.dsc_enabled);
	dsim_info("dsc is %s\n", dsim->lcd_info.dsc_enabled ? "enabled" : "disabled");

	if (dsim->lcd_info.dsc_enabled) {
		of_property_read_u32(node, "dsc_cnt", &dsim->lcd_info.dsc_cnt);
		dsim_info("dsc count(%d)\n", dsim->lcd_info.dsc_cnt);
		of_property_read_u32(node, "dsc_slice_num",
				&dsim->lcd_info.dsc_slice_num);
		dsim_info("dsc slice count(%d)\n", dsim->lcd_info.dsc_slice_num);
		of_property_read_u32(node, "dsc_slice_h",
				&dsim->lcd_info.dsc_slice_h);
		dsim_info("dsc slice height(%d)\n", dsim->lcd_info.dsc_slice_h);
	}

	of_property_read_u32(node, "data_lane", &dsim->data_lane_cnt);
	dsim_info("using data lane count(%d)\n", dsim->data_lane_cnt);

	dsim->lcd_info.data_lane = dsim->data_lane_cnt;

	of_property_read_u32(node, "mres_en", &dsim->lcd_info.dt_lcd_mres.mres_en);
	dsim_info("mres_en(%d)\n", dsim->lcd_info.dt_lcd_mres.mres_en);
	dsim->lcd_info.mres_mode = 1; /* 1=WQHD, 2=FHD, 3=HD */
	dsim->lcd_info.dt_lcd_mres.mres_number = mres_num; /* default = 1 */

	if (dsim->lcd_info.dt_lcd_mres.mres_en) {
		of_property_read_u32(node, "mres_number", &mres_num);
		dsim->lcd_info.dt_lcd_mres.mres_number = mres_num;
		dsim_info("mres_number(%d)\n", mres_num);

		of_property_read_u32_array(node, "mres_width", mres_w, mres_num);
		of_property_read_u32_array(node, "mres_height", mres_h, mres_num);
		of_property_read_u32_array(node, "mres_dsc_width", mres_dsc_w, mres_num);
		of_property_read_u32_array(node, "mres_dsc_height", mres_dsc_h, mres_num);
		of_property_read_u32_array(node, "mres_dsc_en", mres_dsc_en, mres_num);

		switch (mres_num) {
		case 3:
			dsim->lcd_info.dt_lcd_mres.res_info[2].width = mres_w[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].height = mres_h[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].dsc_en = mres_dsc_en[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].dsc_width = mres_dsc_w[2];
			dsim->lcd_info.dt_lcd_mres.res_info[2].dsc_height = mres_dsc_h[2];
		case 2:
			dsim->lcd_info.dt_lcd_mres.res_info[1].width = mres_w[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].height = mres_h[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].dsc_en = mres_dsc_en[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].dsc_width = mres_dsc_w[1];
			dsim->lcd_info.dt_lcd_mres.res_info[1].dsc_height = mres_dsc_h[1];
		case 1:
			dsim->lcd_info.dt_lcd_mres.res_info[0].width = mres_w[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].height = mres_h[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].dsc_en = mres_dsc_en[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].dsc_width = mres_dsc_w[0];
			dsim->lcd_info.dt_lcd_mres.res_info[0].dsc_height = mres_dsc_h[0];
			break;
		default:
			dsim->lcd_info.dt_lcd_mres.res_info[0].width = dsim->lcd_info.width;
			dsim->lcd_info.dt_lcd_mres.res_info[0].height = dsim->lcd_info.height;
			dsim_warn("check multi-resolution configurations at DT\n");
			break;
		}
		dsim_info("[LCD multi(%d)-resolution info] 1st(%dx%d), 2nd(%dx%d), 3rd(%dx%d)\n",
				mres_num, mres_w[0], mres_h[0],
				mres_w[1], mres_h[1], mres_w[2], mres_h[2]);
	} else {
		dsim->lcd_info.dt_lcd_mres.res_info[0].width = dsim->lcd_info.width;
		dsim->lcd_info.dt_lcd_mres.res_info[0].height = dsim->lcd_info.height;
	}

	if (dsim->lcd_info.mode == DECON_MIPI_COMMAND_MODE) {
		of_property_read_u32_array(node, "cmd_underrun_lp_ref",
				dsim->lcd_info.cmd_underrun_lp_ref,
				dsim->lcd_info.dt_lcd_mres.mres_number);
		for (k = 0; k < dsim->lcd_info.dt_lcd_mres.mres_number; k++)
			dsim_info("mres[%d] cmd_underrun_lp_ref(%d)\n", k,
					dsim->lcd_info.cmd_underrun_lp_ref[k]);
	} else {
		of_property_read_u32(node, "vt_compensation",
				&dsim->lcd_info.vt_compensation);
		dsim_info("vt_compensation(%d)\n", dsim->lcd_info.vt_compensation);
	}

	/* HDR info */
	of_property_read_u32(node, "hdr_num", &hdr_num);
	dsim->lcd_info.dt_lcd_hdr.hdr_num = hdr_num;
	dsim_info("hdr_num(%d)\n", hdr_num);

	if (hdr_num != 0) {
		of_property_read_u32_array(node, "hdr_type", hdr_type, hdr_num);
		for (k = 0; k < hdr_num; k++) {
			dsim->lcd_info.dt_lcd_hdr.hdr_type[k] = hdr_type[k];
			dsim_info("hdr_type[%d] = %d\n", k, hdr_type[k]);
		}

		of_property_read_u32(node, "hdr_max_luma", &hdr_mxl);
		of_property_read_u32(node, "hdr_max_avg_luma", &hdr_mal);
		of_property_read_u32(node, "hdr_min_luma", &hdr_mnl);
		dsim->lcd_info.dt_lcd_hdr.hdr_max_luma = hdr_mxl;
		dsim->lcd_info.dt_lcd_hdr.hdr_max_avg_luma = hdr_mal;
		dsim->lcd_info.dt_lcd_hdr.hdr_min_luma = hdr_mnl;
		dsim_info("hdr_max_luma(%d), hdr_max_avg_luma(%d), hdr_min_luma(%d)\n",
				hdr_mxl, hdr_mal, hdr_mnl);
	}
}

static int dsim_parse_dt(struct dsim_device *dsim, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dsim_err("no device tree information\n");
		return -EINVAL;
	}

	dsim->id = of_alias_get_id(dev->of_node, "dsim");
	dsim_info("dsim(%d) probe start..\n", dsim->id);

#ifndef PHY_FRAMEWORK_DIS
	dsim->phy = devm_phy_get(dev, "dsim_dphy");
	if (IS_ERR_OR_NULL(dsim->phy)) {
		dsim_err("failed to get phy\n");
		return PTR_ERR(dsim->phy);
	}
#endif

	dsim->dev = dev;
	dsim_get_gpios(dsim);
	dsim_get_regulator(dsim);
	dsim_parse_lcd_info(dsim);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim->dphy_apb_pdev = of_find_device_by_node(of_find_node_by_name(NULL, "dsim_dphy_apb"));
#endif

	return 0;
}

extern unsigned int system_rev;
static void dsim_register_panel(struct dsim_device *dsim)
{
#ifdef CONFIG_EXYNOS_DSIM_MULTI_PANEL_SUPPORT
	if (get_panel_vendor()) {
#if IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_RM69091)
		dsim->panel_ops = &rm69091_mipi_lcd_driver;
#endif
	} else {
#if IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W4X01_L)
		dsim->panel_ops = &s6e36w4x01_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W4X01_S)
		dsim->panel_ops = &s6e36w4x01_mipi_lcd_driver;
#endif
	}
#else
#if IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_RM6D010)
	dsim->panel_ops = &rm6d010_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W4X01_S) &&\
		IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_RM69330)
	if (system_rev >= 2)
		dsim->panel_ops = &s6e36w4x01_mipi_lcd_driver;
	else
		dsim->panel_ops = &rm69330_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W4X01_L) &&\
			IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W3X01_S)
		if (system_rev >= 2)
			dsim->panel_ops = &s6e36w4x01_mipi_lcd_driver;
		else
			dsim->panel_ops = &s6e36w3x01_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_RM69330)
	dsim->panel_ops = &rm69330_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W3X01_L)
	dsim->panel_ops = &s6e36w3x01_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W3X01_S)
	dsim->panel_ops = &s6e36w3x01_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W4X01_L)
	dsim->panel_ops = &s6e36w4x01_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E36W4X01_S)
	dsim->panel_ops = &s6e36w4x01_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3HA2K)
	dsim->panel_ops = &s6e3ha2k_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3HF4)
	dsim->panel_ops = &s6e3hf4_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3HA6)
	dsim->panel_ops = &s6e3ha6_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3HA8)
	dsim->panel_ops = &s6e3ha8_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3AA2)
	dsim->panel_ops = &s6e3aa2_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_EMUL_DISP)
	dsim->panel_ops = &emul_disp_mipi_lcd_driver;
#else
	dsim->panel_ops = &s6e3ha2k_mipi_lcd_driver;
#endif
#endif
}

static int dsim_get_data_lanes(struct dsim_device *dsim)
{
	int i;

	if (dsim->data_lane_cnt > MAX_DSIM_DATALANE_CNT) {
		dsim_err("%d data lane couldn't be supported\n",
				dsim->data_lane_cnt);
		return -EINVAL;
	}

	dsim->data_lane = DSIM_LANE_CLOCK;
	for (i = 1; i < dsim->data_lane_cnt + 1; ++i)
		dsim->data_lane |= 1 << i;

	dsim_info("%s: lanes(0x%x)\n", __func__, dsim->data_lane);

	return 0;
}

static int dsim_init_resources(struct dsim_device *dsim, struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dsim_err("failed to get mem resource\n");
		return -ENOENT;
	}
	dsim_info("res: start(0x%x), end(0x%x)\n", (u32)res->start, (u32)res->end);

	dsim->res.regs = devm_ioremap_resource(dsim->dev, res);
	if (!dsim->res.regs) {
		dsim_err("failed to remap DSIM SFR region\n");
		return -EINVAL;
	}

#if defined(CONFIG_SOC_EXYNOS9110)
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dsim_err("failed to get mem resource\n");
		return -ENOENT;
	}
	dsim_info("dphy res: start(0x%x), end(0x%x)\n", (u32)res->start, (u32)res->end);

	dsim->res.phy_regs = devm_ioremap_resource(dsim->dev, res);
	if (!dsim->res.phy_regs) {
		dsim_err("failed to remap DSIM DPHY SFR region\n");
		return -EINVAL;
	}
#endif

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dsim_err("failed to get irq resource\n");
		return -ENOENT;
	}

	dsim->res.irq = res->start;
	ret = devm_request_irq(dsim->dev, res->start,
			dsim_irq_handler, 0, pdev->name, dsim);
	if (ret) {
		dsim_err("failed to install DSIM irq\n");
		return -EINVAL;
	}
	disable_irq(dsim->res.irq);

	dsim->res.ss_regs = dpu_get_sysreg_addr();
	if (IS_ERR_OR_NULL(dsim->res.ss_regs)) {
		dsim_err("failed to get sysreg addr\n");
		return -EINVAL;
	}

	return 0;
}

static struct dsim_device *p_dsim;
static int dsim_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct dsim_device *dsim = NULL;
	p_dsim = NULL;

	if (get_panel_id() == -1)
	{
		dsim_err("No lcd attached!\n");
		return 0;
	}

	dsim = devm_kzalloc(dev, sizeof(struct dsim_device), GFP_KERNEL);
	if (!dsim) {
		dsim_err("failed to allocate dsim device.\n");
		ret = -ENOMEM;
		goto err;
	}
	p_dsim = dsim;
	dsim->init_step = 1;

	ret = dsim_parse_dt(dsim, dev);
	if (ret) {
		dsim->init_step = 2;
		goto err_dt;
	}

	dsim_drvdata[dsim->id] = dsim;
	ret = dsim_get_clocks(dsim);
	if (ret) {
		dsim->init_step = 3;
		goto err_dt;
	}

	spin_lock_init(&dsim->slock);
	mutex_init(&dsim->cmd_lock);
	init_completion(&dsim->ph_wr_comp);
	init_completion(&dsim->ph_wr_pl_comp);
	init_completion(&dsim->rd_comp);

	ret = dsim_init_resources(dsim, pdev);
	if (ret) {
		dsim->init_step = 4;
		goto err_dt;
	}

	dsim_init_subdev(dsim);
	platform_set_drvdata(pdev, dsim);
	dsim_register_panel(dsim);
	setup_timer(&dsim->cmd_timer, dsim_cmd_fail_detector,
			(unsigned long)dsim);

	pm_runtime_enable(dev);

	dsim->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	if (dsim->idle_ip_index < 0)
		dsim_warn("idle ip index is not provided for DPU.\n");

#ifdef CONFIG_DISPLAY_EARLY_DPMS
	device_set_early_complete(dev, EARLY_COMP_MASTER);
#endif

	if (dsim->lcd_info.mode == DECON_VIDEO_MODE)
		dsim_acquire_fb_resource(dsim);

#if defined(CONFIG_ION_EXYNOS)
	ret = iovmm_activate(dev);
	if (ret) {
		dsim_err("failed to activate iovmm\n");
		dsim->init_step = 5;
		goto err_dt;
	}
	iovmm_set_fault_handler(dev, dpu_sysmmu_fault_handler, NULL);
#endif

	ret = dsim_get_data_lanes(dsim);
	if (ret) {
		dsim->init_step = 6;
		goto err_dt;
	}

	/* HACK */
#ifdef PHY_FRAMEWORK_DIS
	dphy_isolation = ioremap(0x11860730, 0x4);
#else
	phy_init(dsim->phy);
#endif
	panel_vsync_init();
	dsim->state = DSIM_STATE_INIT;
	dsim_enable(dsim);

	/* TODO: If you want to enable DSIM BIST mode. you must turn on LCD here */

#ifdef CONFIG_SEC_SYSFS
	ret = dsim_create_sec_class(dsim);
	if (ret)
		dsim_err("failed create_sec_class\n");
#endif

#if !defined(BRINGUP_DSIM_BIST)
	call_panel_ops(dsim, probe, dsim);
#else
	/* TODO: This is for dsim BIST mode in zebu emulator. only for test*/
#if defined(CONFIG_EXYNOS_DECON_LCD_S6E3AA2)
	/* Panel lane # is changed to  2 by using LP mode*/
	dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif

	call_panel_ops(dsim, resume, dsim);
	call_panel_ops(dsim, displayon, dsim);
#if defined(CONFIG_EXYNOS_DECON_LCD_S6E3AA2)
	/* MIPI mode is changed to HS mode*/
	dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif

	dsim_set_bist(dsim->id, true);
#endif

	/* for debug */
	/* dsim_dump(dsim); */

	dsim_clocks_info(dsim);
	dsim_create_cmd_rw_sysfs(dsim);

#ifdef CONFIG_SEC_SYSFS
	ret = dsim_ub_con_init(dsim);
	if (ret)
		dsim_err("failed ub_con_init\n");
#endif

	dsim_info("dsim%d driver(%s mode) has been probed.\n", dsim->id,
		dsim->lcd_info.mode == DECON_MIPI_COMMAND_MODE ? "cmd" : "video");
	dsim->init_step = 7;

	return 0;

err_dt:
	kfree(dsim);
err:
	return ret;
}

static int dsim_remove(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);
	if (!dsim) {
		dsim_warn("dsim is not initialized\n");
		return 0;
	}

	pm_runtime_disable(&pdev->dev);
	mutex_destroy(&dsim->cmd_lock);
	dsim_info("dsim%d driver removed\n", dsim->id);

	return 0;
}

static void dsim_shutdown(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);
	if (!dsim) {
		dsim_warn("dsim is not initialized\n");
		return;
	}

	DPU_EVENT_LOG(DPU_EVT_DSIM_SHUTDOWN, &dsim->sd, ktime_set(0, 0));
	dsim_info("%s + state:%d\n", __func__, dsim->state);

	dsim_disable(dsim);
	panel_vsync_cleanup();

	dsim_info("%s -\n", __func__);
}

static int dsim_runtime_suspend(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_SUSPEND, &dsim->sd, ktime_set(0, 0));
	dsim_dbg("%s +\n", __func__);

	clk_disable_unprepare(dsim->res.aclk);
	/* enable idle status for display */
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);

	dsim_dbg("%s -\n", __func__);

	return 0;
}

static int dsim_runtime_resume(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_RESUME, &dsim->sd, ktime_set(0, 0));
	dsim_dbg("%s: +\n", __func__);
	/* This code can make all DPU clock gating is enabled
	 * without dependency with Qchannel setting
	 */
	//void __iomem *addr0 = ioremap(0x14980800, 0x4);
	//writel(0x0, addr0);

	clk_prepare_enable(dsim->res.aclk);

	/* disable idle status for display */
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);

	dsim_dbg("%s -\n", __func__);

	return 0;
}

static const struct of_device_id dsim_of_match[] = {
#if defined(CONFIG_SOC_EXYNOS9110)
	{ .compatible = "samsung,exynos9-dsim" },
#else
	{ .compatible = "samsung,exynos8-dsim" },
#endif
	{},
};
MODULE_DEVICE_TABLE(of, dsim_of_match);

static const struct dev_pm_ops dsim_pm_ops = {
	.runtime_suspend	= dsim_runtime_suspend,
	.runtime_resume		= dsim_runtime_resume,
};

static struct platform_driver dsim_driver __refdata = {
	.probe			= dsim_probe,
	.remove			= dsim_remove,
	.shutdown		= dsim_shutdown,
	.driver = {
		.name		= DSIM_MODULE_NAME,
		.owner		= THIS_MODULE,
		.pm		= &dsim_pm_ops,
		.of_match_table	= of_match_ptr(dsim_of_match),
		.suppress_bind_attrs = true,
	}
};

static int __init dsim_init(void)
{
	int ret = platform_driver_register(&dsim_driver);
	if (ret)
		pr_err("dsim driver register failed\n");

	return ret;
}
late_initcall(dsim_init);

static void __exit dsim_exit(void)
{
	platform_driver_unregister(&dsim_driver);
}
module_exit(dsim_exit);

static int rmem_device_init(struct reserved_mem *rmem, struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	dsim->phys_addr = rmem->base;

	return 0;
}

/* of_reserved_mem_device_release(dev) when reserved memory is no logner required */
static void rmem_device_release(struct reserved_mem *rmem, struct device *dev)
{
	struct page *first = phys_to_page(PAGE_ALIGN(rmem->base));
	struct page *last = phys_to_page((rmem->base + rmem->size) & PAGE_MASK);
	struct page *page;

	pr_info("%s: base=%pa, size=%pa, first=%pa, last=%pa\n",
			__func__, &rmem->base, &rmem->size, first, last);

	for (page = first; page != last; page++) {
		__ClearPageReserved(page);
		set_page_count(page, 1);
		__free_pages(page, 0);
		adjust_managed_page_count(page, 1);
	}
}

static const struct reserved_mem_ops rmem_ops = {
	.device_init	= rmem_device_init,
	.device_release = rmem_device_release,
};

static int __init fb_handover_setup(struct reserved_mem *rmem)
{
	pr_info("%s: base=%pa, size=%pa\n", __func__, &rmem->base, &rmem->size);

	rmem->ops = &rmem_ops;
	return 0;
}
RESERVEDMEM_OF_DECLARE(fb_handover, "exynos,fb_handover", fb_handover_setup);

MODULE_DESCRIPTION("Samusung EXYNOS DSIM driver");
MODULE_LICENSE("GPL");
