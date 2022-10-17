/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *
 * Boojin Kim <boojin.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/iio/iio.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/random.h>
#include <linux/rtc.h>
#include <linux/clk.h>
#include <linux/timekeeping.h>

#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>


#ifdef CONFIG_EXYNOS_ITMON
#include <soc/samsung/exynos-itmon.h>
#endif

#include "chub.h"
#include "chub_ipc.h"
#include "chub_dbg.h"
#include "../../soc/samsung/cal-if/pmucal_shub.h"
#ifdef CONFIG_NANOHUB
#include "main.h"
#endif
#ifndef CONFIG_CHRE_SENSORHUB_HAL
#include "bl_image.h"
#include "os_image.h"
#endif
#if defined(CONFIG_SOC_EXYNOS9110)
#include <linux/smc.h>
#define EXYNOS_CHUB (2)
#define EXYNOS_SET_CONN_TZPC (0)
extern int exynos_smc(unsigned long cmd, unsigned long arg1, unsigned long arg2,
						unsigned long arg3);
#endif

#ifdef CONFIG_SENSORS_SSP
#include "../../sensorhub/slsi_noblesse/ssp_platform.h"
#endif

#define WAIT_TRY_CNT (3)
#define WAIT_TIMEOUT_MS (1000)
enum { CHUB_ON, CHUB_OFF };
enum { C2A_ON, C2A_OFF };

#ifdef CONFIG_SENSORS_SSP
static char ipc_rx_buf[PACKET_SIZE_MAX] ={0,};
#endif

static int contexthub_ipc_reset(struct contexthub_ipc_info *ipc, enum mailbox_event event);
static DEFINE_MUTEX(reset_mutex);

static int contexthub_get_token(struct contexthub_ipc_info *ipc)
{
	if (atomic_read(&ipc->in_reset))
		return -EINVAL;

	atomic_inc(&ipc->in_use_ipc);
	return 0;
}

static void contexthub_put_token(struct contexthub_ipc_info *ipc)
{
	atomic_dec(&ipc->in_use_ipc);
}

/* host interface functions */
int contexthub_is_run(struct contexthub_ipc_info *ipc)
{
	if (!ipc->powermode)
		return 1;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	return nanohub_irq1_fired(ipc->data);
#else
	return 1;
#endif
}

/* request contexthub to host driver */
int contexthub_request(struct contexthub_ipc_info *ipc)
{
	if (!ipc->powermode)
		return 0;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	return request_wakeup_timeout(ipc->data, WAIT_TIMEOUT_MS);
#else
	return 0;
#endif
}

/* rlease contexthub to host driver */
void contexthub_release(struct contexthub_ipc_info *ipc)
{
	if (!ipc->powermode)
		return;

#ifdef CONFIG_CHRE_SENSORHUB_HAL
	release_wakeup(ipc->data);
#endif
}

static inline void contexthub_notify_host(struct contexthub_ipc_info *ipc)
{
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	nanohub_handle_irq1(ipc->data);
#else
	/* TODO */
#endif
}

#ifdef CONFIG_NANOHUB
/* by nanohub kernel RxBufStruct. packet header is 10 + 2 bytes to align */
struct rxbuf {
	u8 pad;
	u8 pre_preamble;
	u8 buf[PACKET_SIZE_MAX];
	u8 post_preamble;
};

static int nanohub_mailbox_open(void *data)
{
	return 0;
}

static void nanohub_mailbox_close(void *data)
{
	(void)data;
}

static int nanohub_mailbox_write(void *data, uint8_t *tx, int length,
				 int timeout)
{
	struct nanohub_data *ipc = data;

	return contexthub_ipc_write(ipc->pdata->mailbox_client, tx, length, timeout);
}

static int nanohub_mailbox_read(void *data, uint8_t *rx, int max_length,
				int timeout)
{
	struct nanohub_data *ipc = data;

	return contexthub_ipc_read(ipc->pdata->mailbox_client, rx, max_length, timeout);
}

void nanohub_mailbox_comms_init(struct nanohub_comms *comms)
{
	comms->seq = 1;
	comms->timeout_write = 544;
	comms->timeout_ack = 272;
	comms->timeout_reply = 512;
	comms->open = nanohub_mailbox_open;
	comms->close = nanohub_mailbox_close;
	comms->write = nanohub_mailbox_write;
	comms->read = nanohub_mailbox_read;
}
#endif

static int contexthub_read_process(uint8_t *rx, u8 *raw_rx, u32 size)
{
#if defined(CONFIG_NANOHUB)
	struct rxbuf *rxstruct;
	struct nanohub_packet *packet;

	rxstruct = (struct rxbuf *)raw_rx;
	packet = (struct nanohub_packet *)&rxstruct->pre_preamble;
	memcpy_fromio(rx, (void *)packet, size);

	return NANOHUB_PACKET_SIZE(packet->len);
#else
	memcpy_fromio(rx, (void *)raw_rx, size);
	return size;
#endif
}

static int contexthub_ipc_drv_init(struct contexthub_ipc_info *chub)
{
	struct device *chub_dev = chub->dev;
	int ret = 0;

	chub->ipc_map = ipc_get_chub_map();
	if (!chub->ipc_map) {
		dev_info(chub_dev, "%s: fails to get ipc map\n", __func__);
		return -EINVAL;
	}

	/* init debug-log */
	chub->ipc_map->logbuf.eq = 0;
	chub->ipc_map->logbuf.dq = 0;
	chub->fw_log = log_register_buffer(chub_dev, 0,
					   (void *)&chub->ipc_map->logbuf.eq,
					   "fw", 1);
	if (!chub->fw_log) {
		dev_info(chub_dev, "%s: fails to init debug-log\n", __func__);
		return -EINVAL;
	}
#ifdef LOWLEVEL_DEBUG
	chub->dd_log_buffer = vmalloc(SZ_256K + sizeof(struct LOG_BUFFER *));
	chub->dd_log_buffer->index_reader = 0;
	chub->dd_log_buffer->index_writer = 0;
	chub->dd_log_buffer->size = SZ_256K;
	chub->dd_log =
	    log_register_buffer(chub_dev, 1, chub->dd_log_buffer, "dd", 0);
#endif

	dev_info(chub_dev,
		 "IPC map information\n\tinfo(base:%p size:%zu)\n\tipc(base:%p size:%zu)\n\tlogbuf(base:%p size:%d)\n",
		 chub, sizeof(struct contexthub_ipc_info),
		 ipc_get_base(IPC_REG_IPC), sizeof(struct ipc_map_area),
		 ipc_get_base(IPC_REG_LOG), chub->ipc_map->logbuf.size);

	ret = chub_dbg_init(chub);
	if (ret)
		dev_err(chub_dev, "%s: fails. ret:%d\n", __func__, ret);

	return ret;
}

#ifdef PACKET_LOW_DEBUG
static void debug_dumpbuf(unsigned char *buf, int len)
{
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET, 16, 1, buf, len,
		       false);
}
#endif

static inline bool read_is_locked(struct contexthub_ipc_info *ipc)
{
	return atomic_read(&ipc->read_lock.cnt) != 0;
}

static inline void read_get_locked(struct contexthub_ipc_info *ipc)
{
	atomic_inc(&ipc->read_lock.cnt);
}

static inline void read_put_unlocked(struct contexthub_ipc_info *ipc)
{
	atomic_dec(&ipc->read_lock.cnt);
}

//api for GIC disabled
static int contexthub_alive_noirq(struct contexthub_ipc_info *ipc)
{
    int cnt = 100;
    int start_index = ipc_hw_read_int_start_index(AP);
    unsigned int status;
		int irq_num = IRQ_EVT_CHUB_ALIVE + start_index;

    ipc_hw_write_shared_reg(AP, AP_WAKE, SR_3);
    ipc_hw_gen_interrupt(AP, IRQ_EVT_CHUB_ALIVE);

    ipc->chub_alive_lock.flag = 0;
    while(cnt--) {
        mdelay(1);
        status = ipc_hw_read_int_status_reg(AP);
        if (status & (1 << irq_num)) {
            ipc_hw_clear_int_pend_reg(AP, irq_num);
            ipc->chub_alive_lock.flag = 1;
            return 0;
        }
    }
    return -1;
}

/* simple alive check function : don't use ipc map */
static bool contexthub_lowlevel_alive(struct contexthub_ipc_info *ipc)
{
	int val;

	ipc->chub_alive_lock.flag = 0;
	ipc_hw_write_shared_reg(AP, AP_WAKE, SR_3);
	ipc_hw_gen_interrupt(AP, IRQ_EVT_CHUB_ALIVE);
	val = wait_event_timeout(ipc->chub_alive_lock.event,
				 ipc->chub_alive_lock.flag,
				 msecs_to_jiffies(WAIT_TIMEOUT_MS));

	return ipc->chub_alive_lock.flag;
}

/* Debug & reset fuctions */
#define CHUB_RESET_THOLD (5)
/* handle errors of chub driver and fw  */
#define CHUB_RESET_ENABLE
static void handle_debug_work(struct contexthub_ipc_info *ipc, enum chub_err_type err)
{
	int need_reset = 0;
	int alive = contexthub_lowlevel_alive(ipc);

	dev_info(ipc->dev, "%s: err:%d, alive:%d, status:%d, in-reset:%d\n",
		__func__, err, alive, __raw_readl(&ipc->chub_status),
		__raw_readl(&ipc->in_reset));
	if ((atomic_read(&ipc->chub_status) == CHUB_ST_ERR) || !alive)
		need_reset = 1;

	/* reset */
	if (need_reset) {
#if defined(CHUB_RESET_ENABLE)
		int ret;

		dev_info(ipc->dev, "%s: request silent reset. err:%d, alive:%d, status:%d, in-reset:%d\n",
			__func__, err, alive, __raw_readl(&ipc->chub_status),
			__raw_readl(&ipc->in_reset));
		ret = contexthub_reset(ipc, 1, err);
		if (ret)
			dev_warn(ipc->dev, "%s: fails to reset:%d. status:%d\n",
				__func__, ret, __raw_readl(&ipc->chub_status));
		else
			dev_info(ipc->dev, "%s: chub reset! should be recovery\n",
				__func__);
#else
		dev_info(ipc->dev, "%s: chub hang. wait for sensor driver reset, err:%d, alive:%d, status:%d, in-reset:%d\n",
			__func__, err, alive, __raw_readl(&ipc->chub_status),
			__raw_readl(&ipc->in_reset));

		atomic_set(&ipc->chub_status, CHUB_ST_HANG);
#endif
	}
}

static void contexthub_handle_debug(struct contexthub_ipc_info *ipc,
	enum chub_err_type err,	bool enable_wq)
{
	dev_info(ipc->dev, "%s: err:%d(cnt:%d), enable_wq:%d\n",
		__func__, err, ipc->err_cnt[err], enable_wq);

	if ((err == CHUB_ERR_ITMON) || (err == CHUB_ERR_FW_WDT) || (err == CHUB_ERR_FW_FAULT)) {
		atomic_set(&ipc->chub_status, CHUB_ST_ERR);
		goto out;
	}
	if (err < CHUB_ERR_NEED_RESET) {
		if (ipc->err_cnt[err] > CHUB_RESET_THOLD) {
			atomic_set(&ipc->chub_status, CHUB_ST_ERR);
			ipc->err_cnt[err] = 0;
			dev_info(ipc->dev, "%s: err:%d(cnt:%d), enter error status\n",
				__func__, err, ipc->err_cnt[err]);
		} else {
			ipc->err_cnt[err]++;
			return;
		}
	}

	/* get chub-fw err */
	if (err == CHUB_ERR_NANOHUB) {
		enum ipc_debug_event fw_evt;

		if (contexthub_get_token(ipc)) {
			dev_warn(ipc->dev, "%s: get token\n", __func__);
			return;
		}
		fw_evt = ipc_read_debug_event(AP);
		if (fw_evt == IPC_DEBUG_CHUB_FAULT)
			err = CHUB_ERR_FW_FAULT;
		else if ((fw_evt == IPC_DEBUG_CHUB_ASSERT) || (fw_evt == IPC_DEBUG_CHUB_ERROR))
			err = CHUB_ERR_FW_ERROR;
		else
			dev_warn(ipc->dev, "%s: unsupported fw_evt: %d\n", __func__, fw_evt);

		ipc_write_debug_event(AP, 0);
		contexthub_put_token(ipc);
	}

	/* set status in CHUB_ST_ERR */
out:
	/* handle err */
	if (enable_wq) {
		ipc->cur_err |= (1 << err);
		schedule_work(&ipc->debug_work);
	} else {
		handle_debug_work(ipc, err);
	}
}

static DEFINE_MUTEX(dbg_mutex);
static void handle_debug_work_func(struct work_struct *work)
{
	struct contexthub_ipc_info *ipc =
	    container_of(work, struct contexthub_ipc_info, debug_work);
	int i;

	dev_info(ipc->dev, "%s: cur_err:0x%x\n", __func__, ipc->cur_err);
	for (i = 0; i < CHUB_ERR_MAX; i++) {
		if (ipc->cur_err & (1 << i)) {
			dev_info(ipc->dev, "%s: loop: err:%d, cur_err:0x%x\n", __func__, i, ipc->cur_err);
			handle_debug_work(ipc, i);
			ipc->cur_err &= ~(1 << i);
		}
	}
}

static inline void clear_err_cnt(struct contexthub_ipc_info *ipc, enum chub_err_type err)
{
	if (ipc->err_cnt[err])
		ipc->err_cnt[err] = 0;
}

int contexthub_ipc_read(struct contexthub_ipc_info *ipc, uint8_t *rx,
				int max_length, int timeout)
{
	unsigned long flag;
	int size = 0;
	int ret = 0;
	void *rxbuf;

	if (!ipc->read_lock.flag) {
		spin_lock_irqsave(&ipc->read_lock.event.lock, flag);
		read_get_locked(ipc);
		ret =
			wait_event_interruptible_timeout_locked(ipc->read_lock.event,
								ipc->read_lock.flag,
								msecs_to_jiffies(timeout));
		read_put_unlocked(ipc);
		spin_unlock_irqrestore(&ipc->read_lock.event.lock, flag);
		if (ret < 0)
			dev_warn(ipc->dev,
				 "fails to get read ret:%d timeout:%d, flag:0x%x",
				 ret, timeout, ipc->read_lock.flag);

		if (!ipc->read_lock.flag)
			goto fail_get_channel;
	}

	ipc->read_lock.flag--;

	if (contexthub_get_token(ipc)) {
		dev_warn(ipc->dev, "no-active: read fails\n");
		return 0;
	}
	rxbuf = ipc_read_data(IPC_DATA_C2A, &size);

	if (size > 0) {
		clear_err_cnt(ipc, CHUB_ERR_READ_FAIL);
		ret = contexthub_read_process(rx, rxbuf, size);
	}
	contexthub_put_token(ipc);
	return ret;

fail_get_channel:
	contexthub_handle_debug(ipc, CHUB_ERR_READ_FAIL, 0);
	return -EINVAL;
}

int contexthub_ipc_write(struct contexthub_ipc_info *ipc,
				uint8_t *tx, int length, int timeout)
{
	int ret;

	if (contexthub_get_token(ipc)) {
		dev_warn(ipc->dev, "no-active: write fails\n");
		return 0;
	}

	ret = ipc_write_data(IPC_DATA_A2C, tx, (u16)length);
	contexthub_put_token(ipc);
	if (ret) {
		pr_err("%s: fails to write data: ret:%d, len:%d errcnt:%d\n",
			__func__, ret, length, ipc->err_cnt[CHUB_ERR_WRITE_FAIL]);
		contexthub_handle_debug(ipc, CHUB_ERR_WRITE_FAIL, 0);
		length = 0;
	} else {
		clear_err_cnt(ipc, CHUB_ERR_WRITE_FAIL);
	}
	return length;
}

static void check_rtc_time(void)
{
	struct rtc_device *chub_rtc = rtc_class_open(CONFIG_RTC_SYSTOHC_DEVICE);
	struct rtc_device *ap_rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	struct rtc_time chub_tm, ap_tm;
	time64_t chub_t, ap_t;

	rtc_read_time(ap_rtc, &chub_tm);
	rtc_read_time(chub_rtc, &ap_tm);

	chub_t = rtc_tm_sub(&chub_tm, &ap_tm);

	if (chub_t) {
		pr_info("nanohub %s: diff_time: %llu\n", __func__, chub_t);
		rtc_set_time(chub_rtc, &ap_tm);
	};

	chub_t = rtc_tm_to_time64(&chub_tm);
	ap_t = rtc_tm_to_time64(&ap_tm);
}

static void contexthub_config_init(struct contexthub_ipc_info *chub)
{
	/* BAAW-P-APM-CHUB for CHUB to access APM_CMGP */
	if (chub->chub_baaw) {
		/* baaw start */
		IPC_HW_WRITE_BAAW_CHUB0(chub->chub_baaw, chub->baaw_val[0]);
	        /* baaw end */
	        IPC_HW_WRITE_BAAW_CHUB1(chub->chub_baaw, chub->baaw_val[1]);
	        /* baaw remap */
	        IPC_HW_WRITE_BAAW_CHUB2(chub->chub_baaw, chub->baaw_val[2]);
	        /* baaw rw access enable */
	        IPC_HW_WRITE_BAAW_CHUB3(chub->chub_baaw, BAAW_RW_ACCESS_ENABLE);
	}
	/* enable mailbox ipc */
	ipc_set_base(chub->sram);
	ipc_set_owner(AP, chub->mailbox, IPC_SRC);
}

int contexthub_ipc_write_event(struct contexthub_ipc_info *ipc, enum mailbox_event event)
{
	u32 val;
	int ret = 0;
	int need_ipc = 0;

	switch (event) {
	case MAILBOX_EVT_INIT_IPC:
		ret = contexthub_ipc_drv_init(ipc);
		break;
	case MAILBOX_EVT_POWER_ON:
		ipc_set_chub_bootmode(BOOTMODE_COLD);
		ret = contexthub_ipc_reset(ipc, event);
		break;
	case MAILBOX_EVT_RESET:
		if (atomic_read(&ipc->chub_status) == CHUB_ST_SHUTDOWN) {
			ret = contexthub_ipc_reset(ipc, event);
			log_schedule_flush_all();
		} else {
			dev_err(ipc->dev,
				"contexthub status isn't shutdown. fails to reset\n");
			ret = -EINVAL;
		}
		break;
	case MAILBOX_EVT_SHUTDOWN:
		/* assert */
		ret = pmucal_shub_reset_assert();
		if (ret) {
			pr_err("%s: reset assert fail\n", __func__);
			return ret;
		}
		/* release_config */
		ret = pmucal_shub_reset_release_config();
		if (ret) {
			pr_err("%s: reset release_config fail\n", __func__);
			return ret;
		}
		/* TZPC setting */
		pr_info("%s: TZPC\n", __func__);
		ret = exynos_smc(SMC_CMD_CONN_IF, ((uint64_t) EXYNOS_CHUB << 32) | EXYNOS_SET_CONN_TZPC, 0, 0);
		if (ret) {
			pr_err("%s: TZPC setting fail\n", __func__);
			return -ret;
		}
		atomic_set(&ipc->chub_status, CHUB_ST_SHUTDOWN);
		contexthub_config_init(ipc);
		break;
	case MAILBOX_EVT_CHUB_ALIVE:
		val = contexthub_lowlevel_alive(ipc);
		if (val) {
			atomic_set(&ipc->chub_status, CHUB_ST_RUN);
			dev_info(ipc->dev, "chub is alive");
			clear_err_cnt(ipc, CHUB_ERR_CHUB_NO_RESPONSE);
		} else {
			dev_err(ipc->dev,
				"chub isn't alive, should be reset. status:%d\n",
				atomic_read(&ipc->chub_status));
			atomic_set(&ipc->chub_status, CHUB_ST_NO_RESPONSE);
			contexthub_handle_debug(ipc, CHUB_ERR_CHUB_NO_RESPONSE, 0);
			ret = -EINVAL;
		}
		break;
	default:
		need_ipc = 1;
		break;
	}

	if (need_ipc) {
		if (contexthub_get_token(ipc)) {
			dev_warn(ipc->dev, "%s event:%d/%d fails chub isn't active, status:%d, inreset:%d\n",
				__func__, event, MAILBOX_EVT_MAX, atomic_read(&ipc->chub_status), atomic_read(&ipc->in_reset));
			return -EINVAL;
		}
		switch(event) {
#ifdef CONFIG_NANOHUB_MAILBOX
		case MAILBOX_EVT_ERASE_SHARED:
			memset_io(ipc_get_base(IPC_REG_SHARED), 0,
				   ipc_get_offset(IPC_REG_SHARED));
			break;
		case MAILBOX_EVT_ENABLE_IRQ:
			/* if enable, mask from CHUB IRQ, else, unmask from CHUB IRQ */
			ipc_hw_unmask_irq(AP, IRQ_EVT_C2A_INT);
			ipc_hw_unmask_irq(AP, IRQ_EVT_C2A_INTCLR);
			break;
		case MAILBOX_EVT_DISABLE_IRQ:
			ipc_hw_mask_irq(AP, IRQ_EVT_C2A_INT);
			ipc_hw_mask_irq(AP, IRQ_EVT_C2A_INTCLR);
			break;
#endif
		default:
			if ((int)event < IPC_DEBUG_UTC_MAX) {
				ipc->utc_run = event;
				if ((int)event == IPC_DEBUG_UTC_TIME_SYNC) {
					check_rtc_time();
				}
				ipc_write_debug_event(AP, event);
				ipc_add_evt(IPC_EVT_A2C, IRQ_EVT_A2C_DEBUG);
			}
			break;
		}
		contexthub_put_token(ipc);
	}

	return ret;
}

#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
#define SENSORTASK_KICK_MS (5000)
#define SENSORTASK_NO_TS (3)
#endif
int contexthub_poweron(struct contexthub_ipc_info *data)
{
	int ret = 0;
	struct device *dev = data->dev;

	if (!atomic_read(&data->chub_status)) {
		ret = contexthub_download_image(data, IPC_REG_BL);
		if (ret) {
			dev_warn(dev, "fails to download bootloader\n");
			return ret;
		}

		ret = contexthub_ipc_write_event(data, MAILBOX_EVT_INIT_IPC);
		if (ret) {
			dev_warn(dev, "fails to init ipc\n");
			return ret;
		}

		ret = contexthub_download_image(data, IPC_REG_OS);
		if (ret) {
			dev_warn(dev, "fails to download kernel\n");
			return ret;
		}
		ret = contexthub_ipc_write_event(data, MAILBOX_EVT_POWER_ON);
		if (ret) {
			dev_warn(dev, "fails to poweron\n");
			return ret;
		}

		if (atomic_read(&data->chub_status) == CHUB_ST_RUN)
			dev_info(dev, "contexthub power-on");
		else
			dev_warn(dev, "contexthub fails to power-on");
	} else {
		ret = -EINVAL;
	}

	if (ret)
		dev_warn(dev, "fails to %s with %d. Status is %d\n",
			 __func__, ret, atomic_read(&data->chub_status));
#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
	dev_info(dev, "contexthub schedule sensor_alive_work\n");
	schedule_delayed_work(&data->sensor_alive_work, msecs_to_jiffies(SENSORTASK_KICK_MS * 4));
	data->sensor_alive_work_run = true;
#endif
	return ret;
}

int contexthub_reset(struct contexthub_ipc_info *data, bool force_load, int dump)
{
	int ret = 0;
	int trycnt = 0;
	int cpu_status = 0;

	dev_info(data->dev, "%s: force:%d, status:%d, in-reset:%d, dump:%d, user:%d\n",
		__func__, force_load, atomic_read(&data->chub_status), atomic_read(&data->in_reset), dump, atomic_read(&data->in_use_ipc));
	mutex_lock(&reset_mutex);
	if (!force_load && (atomic_read(&data->chub_status) == CHUB_ST_RUN)) {
		mutex_unlock(&reset_mutex);
		dev_info(data->dev, "%s: out status:%d\n", __func__, atomic_read(&data->chub_status));
		return 0;
	}
	atomic_inc(&data->in_reset);
	__pm_stay_awake(&data->ws_reset);

	/* debug dump */
	if (dump) {
		chub_dbg_printk_gpr(data);
		/* Enable kernel panic only for test binary */
		//panic("[SSP] Debug Kernel Panic: %s:err:%d\n", __func__, dump);
		pr_err("[SSP] %s: dump %d\n", __func__, dump);
		data->err_cnt[CHUB_ERR_NONE] = dump;
		chub_dbg_dump_hw(data, data->cur_err);
	}

	while (atomic_read(&data->in_use_ipc)) {
		msleep(WAIT_CHUB_MS);
		if (++trycnt > RESET_WAIT_TRY_CNT) {
			dev_info(data->dev, "%s: can't get lock. in_use_ipc: %d\n", __func__, atomic_read(&data->in_use_ipc));
			ret = -EINVAL;
			goto out;
		}
		dev_info(data->dev, "%s: wait for ipc user free: %d\n", __func__, atomic_read(&data->in_use_ipc));
	};

#if defined(CONFIG_SOC_EXYNOS9110)
	/* chub reseted, and then reset release */
	cpu_status = pmucal_shub_cpu_status();
	if (cpu_status == 0) {
		pr_info("%s: chub_cpu_status(%d) is reset, should be reset release\n",
			__func__, cpu_status);
		ret = pmucal_shub_reset_release();
		if (ret)
			pr_warn("%s: fails to reset_release. ret:%d, cpu_status:%d\n",
				__func__, ret, cpu_status);
		cpu_status = pmucal_shub_cpu_status();
		if (cpu_status == 0) {
			pr_warn("%s: fails to reset_release. cpu_status:%d\n",
				__func__, cpu_status);
		}
	}
#endif

	ret = contexthub_ipc_write_event(data, MAILBOX_EVT_SHUTDOWN);
	pr_info("%s: chub block reset %d, cpu_status: %d\n", __func__, force_load, cpu_status);
	if (ret) {
		pr_warn("%s: fails to shutdown. ret:%d\n", __func__, ret);
		goto out;
	}
	memset_io(ipc_get_base(IPC_REG_IPC_A2C), 0, ipc_get_offset(IPC_REG_IPC_A2C));
	memset_io(ipc_get_base(IPC_REG_IPC_C2A), 0, ipc_get_offset(IPC_REG_IPC_C2A));
	memset_io(ipc_get_base(IPC_REG_IPC_EVT_A2C), 0, ipc_get_offset(IPC_REG_IPC_EVT_A2C));
	memset_io(ipc_get_base(IPC_REG_IPC_EVT_C2A), 0, ipc_get_offset(IPC_REG_IPC_EVT_C2A));
	if (force_load) {
		ret = contexthub_download_image(data, IPC_REG_BL);
		if (!ret)
			ret = contexthub_download_image(data, IPC_REG_OS);

		if (ret) {
			pr_warn("%s: fails to download image. ret:%d\n", __func__, ret);
			goto out;
		}
	}

	ret = contexthub_ipc_write_event(data, MAILBOX_EVT_RESET);
	if (ret)
		pr_warn("%s: fails to reset. ret:%d\n", __func__, ret);
out:
	atomic_set(&data->in_use_ipc, 0);
	__pm_relax(&data->ws_reset);
	atomic_dec(&data->in_reset);
	mutex_unlock(&reset_mutex);

#ifdef CONFIG_SENSORS_SSP
			ssp_platform_start_refrsh_task(data->ssp_data);
#endif
	return ret;
}

int contexthub_download_image(struct contexthub_ipc_info *data, enum ipc_region reg)
{
	const struct firmware *entry;
	struct device *dev = data->dev;
	int ret = 0;
	int size = 0;

	if (reg == IPC_REG_BL) {
		ret = request_firmware(&entry, "bl.unchecked.bin", dev);
		if (ret) {
#if defined(CONFIG_CHRE_SENSORHUB_HAL)
			dev_err(dev, "%s, bl request_firmware failed\n",
				__func__);
			return ret;
#else
			memcpy_toio(ipc_get_base(IPC_REG_BL), bl_unchecked_bin, bl_unchecked_bin_len);
			return 0;
#endif
		}
		/* Pointer '&entry->size' is dereferenced at chub.c:760
		after the referenced memory was deallocated at firmware_class.c:1340
		by passing as 1st parameter to function 'release_firmware' */
		size = (int)entry->size;
		memcpy_toio(ipc_get_base(IPC_REG_BL), entry->data, entry->size);
		release_firmware(entry);

		dev_info(dev, "%s: bootloader(size:0x%x) on %lx\n",
			 __func__, size,
			 (unsigned long)ipc_get_base(IPC_REG_BL));
	} else {
		ret = request_firmware(&entry, CHUB_OS_NAME, dev);
		if (ret) {
#if defined(CONFIG_CHRE_SENSORHUB_HAL)
			dev_err(dev, "%s, %s request_firmware failed\n",
				__func__, CHUB_OS_NAME);
			return ret;
#else
			memcpy_toio(ipc_get_base(IPC_REG_OS), os_chub_bin, os_chub_bin_len);
			return 0;
#endif
		}
		/* Pointer '&entry->size' is dereferenced at chub.c:760
		after the referenced memory was deallocated at firmware_class.c:1340
		by passing as 1st parameter to function 'release_firmware' */
		size = (int)entry->size;
		memcpy_toio(ipc_get_base(IPC_REG_OS), entry->data, entry->size);
		release_firmware(entry);

		dev_info(dev, "%s: %s(size:0x%x) on %lx\n", __func__,
			 CHUB_OS_NAME, size,
			 (unsigned long)ipc_get_base(IPC_REG_OS));
	}

	return 0;
}

int contexthub_download_bl(struct contexthub_ipc_info *data)
{
	return contexthub_reset(data, 1, 0);
}

int contexthub_download_kernel(struct device *dev)
{
	const struct firmware *fw_entry;
	int ret;

	ret = request_firmware(&fw_entry, CHUB_OS_NAME, dev);
	if (ret) {
		dev_err(dev, "%s: err=%d\n", __func__, ret);
		return -EIO;
	}
	memcpy_toio(ipc_get_base(IPC_REG_OS), fw_entry->data, fw_entry->size);
	release_firmware(fw_entry);
	return 0;
}

static int contexthub_ipc_reset(struct contexthub_ipc_info *ipc,
				enum mailbox_event event)
{
	u32 val;
	int ret = 0;
	int trycnt = 0;

	/* clear ipc value */
	ipc_init();
	atomic_set(&ipc->wakeup_chub, 1);
	atomic_set(&ipc->irq1_apInt, 1);
	atomic_set(&ipc->read_lock.cnt, 0x0);
	ipc->read_lock.flag = 0;

	ipc_hw_write_shared_reg(AP, ipc->os_load, SR_BOOT_MODE);
	ipc_set_chub_clk((u32) ipc->clkrate);

	switch (event) {
	case MAILBOX_EVT_POWER_ON:
#ifdef NEED_TO_RTC_SYNC
		check_rtc_time();
#endif
		if (atomic_read(&ipc->chub_status) == CHUB_ST_NO_POWER) {
			atomic_set(&ipc->chub_status, CHUB_ST_POWER_ON);

			/* enable Dump GPR */
			IPC_HW_WRITE_DUMPGPR_CTRL(ipc->chub_dumpgpr, 0x1);

			/* pmu reset-release on CHUB */
			val =
			    __raw_readl(ipc->pmu_chub_reset +
					REG_CHUB_RESET_CHUB_OPTION);
			__raw_writel((val | CHUB_RESET_RELEASE_VALUE),
				     ipc->pmu_chub_reset +
				     REG_CHUB_RESET_CHUB_OPTION);

		} else {
			ret = -EINVAL;
			dev_warn(ipc->dev,
				 "fails to contexthub power on. Status is %d\n",
				 atomic_read(&ipc->chub_status));
		}
		break;
	case MAILBOX_EVT_RESET:
#if defined(CONFIG_SOC_EXYNOS9110)
		ret = pmucal_shub_reset_release();
#else
#error
#endif
		break;
	default:
		break;
	}

	if (ret)
		return ret;
	else {
		/* wait active */
		dev_info(ipc->dev, "%s: alive check\n", __func__);
		trycnt = 0;
		do {
			msleep(WAIT_CHUB_MS);
			contexthub_ipc_write_event(ipc, MAILBOX_EVT_CHUB_ALIVE);
			if (++trycnt > WAIT_TRY_CNT)
				break;
		} while ((atomic_read(&ipc->chub_status) != CHUB_ST_RUN));

		if (atomic_read(&ipc->chub_status) == CHUB_ST_RUN) {
			dev_info(ipc->dev, "%s done. contexthub status is %d\n",
				 __func__, atomic_read(&ipc->chub_status));
			return 0;
		} else {
			dev_warn(ipc->dev, "%s fails. contexthub status is %d\n",
				 __func__, atomic_read(&ipc->chub_status));
			return -ETIMEDOUT;
		}
	}
}

#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
static void sensor_alive_check_func(struct work_struct *work)
{
	struct contexthub_ipc_info *ipc =
			container_of(to_delayed_work(work), struct contexthub_ipc_info, sensor_alive_work);

	pr_info("%s: ipc:%p\n", __func__, ipc);
	if (ipc->sensor_cnt_last == ipc->ipc_map->sensor_cnt ||
		ipc->event_rtc_last == ipc->ipc_map->event_rtc_cnt) {
		ipc->sensor_cnt_no_update++;
	} else {
		ipc->sensor_cnt_last = ipc->ipc_map->sensor_cnt;
		ipc->event_flush_last = ipc->ipc_map->event_flush_cnt;
		ipc->event_rtc_last = ipc->ipc_map->event_rtc_cnt;
		ipc->event_hrm_last = ipc->ipc_map->event_hrm_cnt;
		ipc->rtc_expired_last = ipc->ipc_map->rtc_expired_cnt;
		ipc->sensor_cnt_no_update = 0;
	}

	pr_info("%s: sensor_cnt:%d/%d, evflush:%d/%d, evrtc:%d/%d, evhrm:%d/%d, rtcexp:%d/%d, no_cnt:%d\n",
		__func__, ipc->ipc_map->sensor_cnt, ipc->sensor_cnt_last,
			ipc->ipc_map->event_flush_cnt, ipc->event_flush_last,
			ipc->ipc_map->event_rtc_cnt, ipc->event_rtc_last,
			ipc->ipc_map->event_hrm_cnt, ipc->event_hrm_last,
			ipc->ipc_map->rtc_expired_cnt, ipc->rtc_expired_last,
			ipc->sensor_cnt_no_update);
	if (ipc->sensor_cnt_no_update > SENSORTASK_NO_TS) {
		contexthub_reset(ipc, true, 0xff);
		ipc->sensor_cnt_last = ipc->ipc_map->sensor_cnt = ipc->sensor_cnt_no_update = 0;
	}
	schedule_delayed_work(&ipc->sensor_alive_work, msecs_to_jiffies(SENSORTASK_KICK_MS));
}
#endif
static void handle_irq(struct contexthub_ipc_info *ipc, enum irq_evt_chub evt)
{
	enum chub_err_type err;

	switch (evt) {
	case IRQ_EVT_C2A_DEBUG:
		err = (ipc_read_debug_event(AP) == IPC_DEBUG_CHUB_FAULT) ? CHUB_ERR_FW_FAULT : CHUB_ERR_NANOHUB;
		pr_info("%s: c2a_debug: debug:%d, err:%d\n", __func__, ipc_read_debug_event(AP), err);
		contexthub_handle_debug(ipc, CHUB_ERR_NANOHUB, 1);
		break;
	case IRQ_EVT_C2A_INT:
		if (atomic_read(&ipc->irq1_apInt) == C2A_OFF) {
			atomic_set(&ipc->irq1_apInt, C2A_ON);
			contexthub_notify_host(ipc);
		}
		break;
	case IRQ_EVT_C2A_INTCLR:
		atomic_set(&ipc->irq1_apInt, C2A_OFF);
		break;
	default:
		if (evt < IRQ_EVT_CH_MAX) {
#ifdef CONFIG_SENSORS_SSP
			void *raw_rx_buf = 0;
			u32 size = 0;

			if (contexthub_get_token(ipc)) {
				dev_warn(ipc->dev, "%s: get token\n", __func__);
				return;
			}
			raw_rx_buf = ipc_read_data(IPC_DATA_C2A, &size);

			if (size > 0 && raw_rx_buf) {
				clear_err_cnt(ipc, CHUB_ERR_READ_FAIL);
				contexthub_read_process(ipc_rx_buf, (void *)raw_rx_buf, size);
				//ssp_sensorhub_log(__func__, ipc_rx_buf, size);
				ssp_handle_recv_packet(ipc->ssp_data, ipc_rx_buf,size);
			}
			contexthub_put_token(ipc);
#else // CONFIG_SENSORS_SSP
			int lock;

			ipc->read_lock.flag++;
			/* TODO: requered.. ? */
			spin_lock(&ipc->read_lock.event.lock);
			lock = read_is_locked(ipc);
			spin_unlock(&ipc->read_lock.event.lock);
			if (lock)
				wake_up_interruptible_sync(&ipc->read_lock.event);
#endif
		} else {
			dev_warn(ipc->dev, "%s: invalid %d event",
				 __func__, evt);
		}
		break;
	};
}

static irqreturn_t contexthub_irq_handler(int irq, void *data)
{
	struct contexthub_ipc_info *ipc = data;
	int start_index = ipc_hw_read_int_start_index(AP);
	unsigned int status = ipc_hw_read_int_status_reg(AP);
	struct ipc_evt_buf *cur_evt;
	enum chub_err_type err = 0;
	enum irq_chub evt = 0;
	int irq_num = IRQ_EVT_CHUB_ALIVE + start_index;

	//pr_info("[SSP]%s: irq_num(%d)\n", __func__, irq_num);

	/* chub alive interrupt handle */
	if (status & (1 << irq_num)) {
		status &= ~(1 << irq_num);
		ipc_hw_clear_int_pend_reg(AP, irq_num);
		/* set wakeup flag for chub_alive_lock */
		ipc->chub_alive_lock.flag = 1;
		wake_up(&ipc->chub_alive_lock.event);
	}

	/* chub ipc interrupt handle */
	while (status) {
		cur_evt = ipc_get_evt(IPC_EVT_C2A);

		if (cur_evt) {
			evt = cur_evt->evt;
			irq_num = cur_evt->irq + start_index;

			/* check match evtq and hw interrupt pending */
			if (!(status & (1 << irq_num))) {
				err = CHUB_ERR_EVTQ_NO_HW_TRIGGER;
				break;
			}
		} else {
			err = CHUB_ERR_EVTQ_EMTPY;
			break;
		}

		handle_irq(ipc, (u32)evt);
		ipc_hw_clear_int_pend_reg(AP, irq_num);
		status &= ~(1 << irq_num);
	}

	if (err) {
		pr_err("inval irq err(%d):start_irqnum:%d,evt(%p):%d,irq_hw:%d,status_reg:0x%x(0x%x,0x%x)\n",
		       err, start_index, cur_evt, evt, irq_num,
		       status, ipc_hw_read_int_status_reg(AP),
		       ipc_hw_read_int_gen_reg(AP));
		ipc_hw_clear_all_int_pend_reg(AP);
		contexthub_handle_debug(ipc, err, 1);
	} else {
		clear_err_cnt(ipc, CHUB_ERR_EVTQ_EMTPY);
		clear_err_cnt(ipc, CHUB_ERR_EVTQ_NO_HW_TRIGGER);
	}
	return IRQ_HANDLED;
}

static irqreturn_t contexthub_irq_wdt_handler(int irq, void *data)
{
	struct contexthub_ipc_info *ipc = data;

	dev_info(ipc->dev, "context generated WDT timeout.\n");
	disable_irq_nosync(ipc->irq_wdt);
	ipc->irq_wdt_disabled = 1;
	contexthub_handle_debug(ipc, CHUB_ERR_FW_WDT, 1);
	return IRQ_HANDLED;
}


static __init int contexthub_ipc_hw_init(struct platform_device *pdev,
					 struct contexthub_ipc_info *chub)
{
	int ret = 0;
	int irq = 0;
	int idx = 0;
	struct resource *res;
	const char *os;
	struct device *dev = &pdev->dev;
	struct device_node *node;
	//unsigned int baaw_val[BAAW_VAL_MAX];
	int trycnt = 0;
	u32 val;

	node = dev->of_node;
	if (!node) {
		dev_err(dev, "driver doesn't support non-dt\n");
		return -ENODEV;
	}

	/* get os type from dt */
	os = of_get_property(node, "os-type", NULL);
	if (!strcmp(os, "none")) {
		dev_err(dev, "no use contexthub\n");
		return -ENODEV;
	} else if (!strcmp(os, "pass")) {
		chub->os_load = 0;
	} else {
		chub->os_load = 1;

		/* Use of vulnerable function 'strcpy', This function is unsafe, use strncpy instead.*/
		strncpy(chub->os_name, os, MAX_FILE_LEN);
	}
	pr_info("%s: %s\n", __func__, os);

	/* get mailbox interrupt */
	irq = irq_of_parse_and_map(node, 0);
	if (irq < 0) {
		dev_err(dev, "failed to get irq:%d\n", irq);
		return -EINVAL;
	}

	/* request irq handler */
#ifdef CONFIG_SENSORS_SSP
	ret = devm_request_threaded_irq(dev, irq, NULL, contexthub_irq_handler,
							 IRQF_ONESHOT, dev_name(dev), chub);
#else
	ret = devm_request_irq(dev, irq, contexthub_irq_handler,
			       0, dev_name(dev), chub);
#endif
	if (ret) {
		dev_err(dev, "failed to request irq:%d, ret:%d\n", irq, ret);
		return ret;
	}

	/* get wdt interrupt optionally */
	irq = irq_of_parse_and_map(node, 1);
	if (irq > 0) {
		/* request irq handler */
		ret = devm_request_irq(dev, irq,
				       contexthub_irq_wdt_handler, 0,
				       dev_name(dev), chub);
		if (ret) {
			dev_err(dev, "failed to request wdt irq:%d, ret:%d\n",
				irq, ret);
			return ret;
		}
	} else {
		dev_info(dev, "don't use wdt irq:%d\n", irq);
	}

	/* get mailbox SFR */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->mailbox = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->mailbox)) {
		dev_err(dev, "fails to get mailbox sfr\n");
		return PTR_ERR(chub->mailbox);
	}

	/* get SRAM base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->sram)) {
		dev_err(dev, "fails to get sram\n");
		return PTR_ERR(chub->sram);
	}

	/* get chub gpr base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->chub_dumpgpr = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->chub_dumpgpr)) {
		dev_err(dev, "fails to get dumpgpr\n");
		return PTR_ERR(chub->chub_dumpgpr);
	}

	/* get pmu reset base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->pmu_chub_reset = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->pmu_chub_reset)) {
		dev_err(dev, "fails to get dumpgpr\n");
		return PTR_ERR(chub->pmu_chub_reset);
	}
#if defined(CONFIG_SOC_EXYNOS9810)
	/* get pmu reset enable base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->pmu_chub_cpu = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->pmu_chub_cpu)) {
		dev_err(dev, "fails to get pmu_chub_cpu\n");
		return PTR_ERR(chub->pmu_chub_cpu);
	}
#elif defined(CONFIG_SOC_EXYNOS9110)
	/* get pmu osc rco */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->pmu_osc_rco = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->pmu_osc_rco)) {
		dev_err(dev, "fails to get pmu_osc_rco\n");
		return PTR_ERR(chub->pmu_osc_rco);
	}

	/* get pmu rtc control */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->pmu_rtc_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->pmu_rtc_ctrl)) {
		dev_err(dev, "fails to get pmu_rtc_ctrl\n");
		return PTR_ERR(chub->pmu_rtc_ctrl);
	}

	/* get pmu chub control base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->pmu_chub_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->pmu_chub_ctrl)) {
		dev_err(dev, "fails to get pmu_chub_ctrl\n");
		return PTR_ERR(chub->pmu_chub_ctrl);
	}

	/* get pmu chub reset release status */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->pmu_chub_reset_stat = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->pmu_chub_reset_stat)) {
		dev_err(dev, "fails to get pmu_chub_reset_stat\n");
		return PTR_ERR(chub->pmu_chub_reset_stat);
	}
#endif

	/* get chub baaw base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx++);
	chub->chub_baaw = devm_ioremap_resource(dev, res);
	if (IS_ERR(chub->chub_baaw)) {
		pr_err("driver failed to get chub_baaw\n");
		chub->chub_baaw = 0;	/* it can be set on other-side (vts) */
	}

	/* pmu MUX Unset */
	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);
	__raw_writel((val & ~(0x1 << 4)),
		     chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);

	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);
	__raw_writel((val & ~(0x1 << 4)),
		     chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);

	/* CHUB Block Reset Release */
	val = __raw_readl(chub->pmu_chub_ctrl);
	__raw_writel((val | (0x1 << 9)), chub->pmu_chub_ctrl);

	/* Check Reset Sequence Status */
	do {
		msleep(WAIT_TIMEOUT_MS / 1000);
		val = __raw_readl(chub->pmu_chub_reset_stat);
		val = (val >> 12) & 0x7;
		if (++trycnt > WAIT_TRY_CNT)
			break;
	} while (val != 0x5);

	/* pmu MUX Set */
	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);
	__raw_writel((val | (0x1 << 4)),
		     chub->pmu_osc_rco + REG_CTRL_REFCLK_PMU);

	val = __raw_readl(chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);
	__raw_writel((val | (0x1 << 4)),
		     chub->pmu_osc_rco + REG_CTRL_REFCLK_CHUB_VTS);

	ret = exynos_smc(SMC_CMD_CONN_IF,
			 (uint64_t)EXYNOS_CHUB << 32 |
			 EXYNOS_SET_CONN_TZPC, 0, 0);
	if (ret) {
		dev_err(dev, "%s: exynos_smc failed\n", __func__);
		return ret;
	}

	if (chub->chub_baaw) {
		for (idx = 0; idx < BAAW_VAL_MAX; idx++) {
			ret =
			    of_property_read_u32_index(node,
						       "baaw,baaw-p-apm-chub",
						       idx, &chub->baaw_val[idx]);
			if (ret) {
				dev_err(dev,
					"fails to get baaw-p-apm-chub %d\n",
					idx);
				return -ENODEV;
			}
		}
	}

	/* BAAW-P-APM-CHUB for CHUB to access APM_CMGP */
	if (chub->chub_baaw) {
		/* baaw start */
		IPC_HW_WRITE_BAAW_CHUB0(chub->chub_baaw, chub->baaw_val[0]);
		/* baaw end */
		IPC_HW_WRITE_BAAW_CHUB1(chub->chub_baaw, chub->baaw_val[1]);
		/* baaw remap */
		IPC_HW_WRITE_BAAW_CHUB2(chub->chub_baaw, chub->baaw_val[2]);
		/* baaw rw access enable */
		IPC_HW_WRITE_BAAW_CHUB3(chub->chub_baaw, chub->baaw_val[3]);
	}

#if defined(CONFIG_SOC_EXYNOS9110)
	/* pmu rtc_control Set */
	val = __raw_readl(chub->pmu_rtc_ctrl);
	__raw_writel((val | (0x1 << 0)), chub->pmu_rtc_ctrl);

    /* Set CMU_CHUB CHUB_BUS as 49.152Mhz CLK_RCO_VTS in FW */
    chub->clkrate = 24576000 * 2;
	dev_info(dev, "%s clk selection of CMU_CHUB is %lu.\n", __func__, chub->clkrate);
#endif

	dev_info(dev, "%s with %lu clk is done.\n", __func__, chub->clkrate);
	return 0;
}

static ssize_t chub_download_bl(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	int ret;

	ret = contexthub_download_bl(ipc);
	return ret < 0 ? ret : count;
}

static ssize_t chub_download_kernel(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int ret = contexthub_download_kernel(dev);

	return ret < 0 ? ret : count;
}

static ssize_t chub_poweron(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	int ret = contexthub_poweron(ipc);

	return ret < 0 ? ret : count;
}

static ssize_t chub_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);
	int ret;

	ret = contexthub_reset(ipc, 1, 0);
	return ret < 0 ? ret : count;
}

static struct device_attribute attributes[] = {
	__ATTR(download_bl, 0220, NULL, chub_download_bl), /* donwload bl & os, and reset */
	__ATTR(download_kernel, 0220, NULL, chub_download_kernel),
	__ATTR(reset, 0220, NULL, chub_reset),
	__ATTR(poweron, 0220, NULL, chub_poweron),
};

#ifdef CONFIG_EXYNOS_ITMON
static int chub_itmon_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct contexthub_ipc_info *data = container_of(nb, struct contexthub_ipc_info, itmon_nb);
	struct itmon_notifier *itmon_data = nb_data;

	if (itmon_data && itmon_data->master &&
		(!strncmp("CM4_SHUB",  itmon_data->master, sizeof("CM4_SHUB") - 1) ||
		!strncmp("PDMA_SHUB", itmon_data->master, sizeof("PDMA_SHUB") - 1))) {
		dev_info(data->dev, "%s: chub(%s) itmon detected: action:%lu!!\n",
			__func__, itmon_data->master, action);
		contexthub_handle_debug(data, CHUB_ERR_ITMON, 1);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}
#endif

static int contexthub_ipc_probe(struct platform_device *pdev)
{
	struct contexthub_ipc_info *chub = NULL;
	int need_to_free = 0;
	int ret = 0;
	int i;
#ifdef CONFIG_NANOHUB
	struct iio_dev *iio_dev;
#endif

	chub = chub_dbg_get_memory(DBG_NANOHUB_DD_AREA);
	if (!chub) {
		chub =
		    devm_kzalloc(&pdev->dev, sizeof(struct contexthub_ipc_info),
				 GFP_KERNEL);
		need_to_free = 1;
	}
	if (IS_ERR(chub)) {
		dev_err(&pdev->dev, "%s failed to get ipc memory\n", __func__);
		return PTR_ERR(chub);
	}

	/* parse dt and hw init */
	ret = contexthub_ipc_hw_init(pdev, chub);
	if (ret) {
		dev_err(&pdev->dev, "%s failed to get init hw with ret %d\n",
			__func__, ret);
		goto err;
	}
#ifdef CONFIG_CHRE_SENSORHUB_HAL
	/* nanohub probe */
	iio_dev = nanohub_probe(&pdev->dev, NULL);
	if (IS_ERR(iio_dev)) {
		dev_err(&pdev->dev, "%s failed to init host driver\n",
			__func__);
		ret = PTR_ERR(iio_dev);
		goto err;
	}

	chub->data = iio_priv(iio_dev);
	nanohub_mailbox_comms_init(&chub->data->comms);

	/* set wakeup irq number on nanohub driver */
	chub->data->irq1 = IRQ_EVT_A2C_WAKEUP;
	chub->data->irq2 = 0;
	chub->data->pdata->mailbox_client = chub;
#endif

#ifdef CONFIG_SENSORS_SSP
	chub->ssp_data = ssp_device_probe(&pdev->dev);
	if(IS_ERR(chub->ssp_data)) {
		dev_err(chub->dev, "[Chub] ssp_probe failed \n");
		return PTR_ERR(chub->ssp_data);
	}
	ssp_platform_init(chub->ssp_data, chub);
#endif

	chub->dev = &pdev->dev;
	platform_set_drvdata(pdev, chub);
	chub->cur_err = 0;
	atomic_set(&chub->chub_status, CHUB_ST_NO_POWER);
	init_waitqueue_head(&chub->read_lock.event);
	init_waitqueue_head(&chub->chub_alive_lock.event);
	INIT_WORK(&chub->debug_work, handle_debug_work_func);
#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
	INIT_DELAYED_WORK(&chub->sensor_alive_work, sensor_alive_check_func);
#endif
	contexthub_config_init(chub);

	for (i = 0, ret = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(chub->dev, &attributes[i]);
		if (ret)
			dev_warn(chub->dev, "Failed to create file: %s\n",
				 attributes[i].attr.name);
	}


#ifdef CONFIG_EXYNOS_ITMON
	chub->itmon_nb.notifier_call = chub_itmon_notifier;
	itmon_notifier_chain_register(&chub->itmon_nb);
#endif

	wakeup_source_init(&chub->ws_reset, "chub_reboot");
	dev_info(&pdev->dev, "%s is done\n", __func__);
	return 0;
err:
	if (chub) {
		if (need_to_free)
			devm_kfree(&pdev->dev, chub);
	}
	dev_err(&pdev->dev, "%s is fail with ret %d\n", __func__, ret);
	return ret;
}

static int contexthub_ipc_remove(struct platform_device *pdev)
{
	struct contexthub_ipc_info *chub = platform_get_drvdata(pdev);

	wakeup_source_trash(&chub->ws_reset);
	return 0;
}

static int contexthub_suspend(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
		return 0;

#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
	cancel_delayed_work(&ipc->sensor_alive_work);
	ipc->sensor_alive_work_run = false;
#endif
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int contexthub_resume_noirq(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
		return 0;

	contexthub_alive_noirq(ipc);
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int contexthub_suspend_noirq(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
		return 0;

	dev_info(dev, "%s: send ap sleep\n", __func__);
	ipc_hw_write_shared_reg(AP, AP_SLEEP, SR_3);
	ipc_hw_gen_interrupt(AP, IRQ_EVT_CHUB_ALIVE);
	dev_info(dev, "%s: out\n", __func__);
	return 0;
}

static int contexthub_resume(struct device *dev)
{
	struct contexthub_ipc_info *ipc = dev_get_drvdata(dev);

	if (atomic_read(&ipc->chub_status) != CHUB_ST_RUN)
		return 0;
#ifdef CONFIG_CONTEXTHUB_SENSOR_DEBUG
	if (ipc->sensor_alive_work_run == false) {
		dev_info(dev, "%s: schedule sensor_alive_work\n", __func__);
		schedule_delayed_work(&ipc->sensor_alive_work, msecs_to_jiffies(1000));
	}
#endif

	dev_info(dev, "%s: send ap wakeup\n", __func__);

	ipc_hw_write_shared_reg(AP, AP_WAKE, SR_3);
	contexthub_lowlevel_alive(ipc);

	dev_info(dev, "%s: out\n", __func__);
	return 0;
}


static const struct dev_pm_ops contexthub_pm_ops = {
	.suspend = contexthub_suspend,
	.suspend_noirq = contexthub_suspend_noirq,
	.resume = contexthub_resume,
	.resume_noirq = contexthub_resume_noirq,
};

static const struct of_device_id contexthub_ipc_match[] = {
	{.compatible = "samsung,exynos-contexthub"},
	{},
};

static struct platform_driver samsung_contexthub_ipc_driver = {
	.probe = contexthub_ipc_probe,
	.remove = contexthub_ipc_remove,
	.driver = {
		   .name = "contexthub-ipc",
		   .owner = THIS_MODULE,
		   .of_match_table = contexthub_ipc_match,
			.pm = &contexthub_pm_ops,
		   },
};

int contexthub_mailbox_init(void)
{
	return platform_driver_register(&samsung_contexthub_ipc_driver);
}

void __exit contexthub_mailbox_cleanup(void)
{
	platform_driver_unregister(&samsung_contexthub_ipc_driver);
}

module_init(contexthub_mailbox_init);
module_exit(contexthub_mailbox_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Exynos contexthub mailbox Driver");
MODULE_AUTHOR("Boojin Kim <boojin.kim@samsung.com>");
