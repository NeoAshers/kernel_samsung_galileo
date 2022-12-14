/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/bitops.h>
#include "ssp.h"

bool ssp_dbg;
bool ssp_pkt_dbg;

#define dprint(fmt, args...) \
		if (unlikely(ssp_dbg))	 printk(KERN_INFO "[SSPBBD]:(%s:%d): " fmt, __func__,__LINE__, ##args)

#define DEBUG_SHOW_HEX_SEND(msg, len) \
		if (unlikely(ssp_pkt_dbg))	print_hex_dump(KERN_INFO, "SSP->MCU: ", DUMP_PREFIX_NONE, 16, 1, (msg), (len), true);
#define DEBUG_SHOW_HEX_RECV(msg, len) \
		if (unlikely(ssp_pkt_dbg))	print_hex_dump(KERN_INFO, "SSP<-MCU: ", DUMP_PREFIX_NONE, 16, 1, (msg), (len), true);

extern void clean_msg(struct ssp_msg *msg);

static unsigned char packet_alarm_cnt;
void ssp_dump_all_status(const char *buf, int st_alarm_cnt)
{
	unsigned long long irq_sec;
	unsigned long irq_nsec;
	int alarm_cnt_diff;
	int bbd_rbuf_cnt;
	int bcm_wbuf_cnt;
	const char *str;

	if (st_alarm_cnt < 0)
		alarm_cnt_diff = INT_MAX;
	else
		alarm_cnt_diff = (int)packet_alarm_cnt - st_alarm_cnt;

	get_bcm_irq_time_last(&irq_sec, &irq_nsec);
	bbd_rbuf_cnt = bbd_on_read_buf_cnt(1);
	bcm_wbuf_cnt = bcm_spi_rw_buf_cnt(1);

	if (buf)
		str = buf;
	else
		str = "none";

	pr_err("[SSPBBD]: %s(): RB_CNT:%d, WB_CNT:%d, Diff_ACNT:%d, L_irq_t:%llu.%lu\n",
		str, bbd_rbuf_cnt, bcm_wbuf_cnt,
			alarm_cnt_diff, irq_sec, irq_nsec);

	bcm477x_debug_info(str);
	print_ssp_pkt_dump("bbd_send_pkt");
	print_ssp_pkt_dump("bcm_spi_wpkt");
}

/**
 * transfer ssp data to mcu thru bbd driver.
 *
 * @data:	ssp data pointer
 * @msg:	ssp message
 * @done:	completion
 * @timeout:	wait response time (ms)
 *
 * @return:	 1 = success, -1 = failed to send data to bbd
 *		-2 = failed to get response from mcu
 */
int bbd_do_transfer(struct ssp_data *data, struct ssp_msg *msg,
		struct completion *done, int timeout) {
	int status = 0;
	bool msg_dead = false, ssp_down = false;
	bool use_no_irq = false;
	int start_pkt_acnt = INT_MIN;

	if(data == NULL || msg == NULL) {
		pr_err("%s():[SSPBBD] data or msg is NULL\n", __func__);
		return -1;
	}

	mutex_lock(&data->comm_mutex);

	if (timeout) {
		wake_lock(&data->ssp_comm_wake_lock);
	}

	ssp_down = data->bSspShutdown;

	if (ssp_down) {
		pr_err("[SSPBBD]: ssp_down == true. returning\n");
		clean_msg(msg);
		mdelay(5);
		if (timeout) {
			wake_unlock(&data->ssp_comm_wake_lock);
		}
		mutex_unlock(&data->comm_mutex);
		return -1;
	}

	msg->dead_hook = &msg_dead;
	msg->dead = false;
	msg->done = done;
	use_no_irq = (msg->length == 0);

	mutex_lock(&data->pending_mutex);

	if (!use_no_irq)
		start_pkt_acnt = (int)packet_alarm_cnt;

	if (bbd_send_packet((unsigned char *)msg, 9) > 0) {
		status = 1;
		DEBUG_SHOW_HEX_SEND(msg, 9)
	} else {
		pr_err("[SSP]: %s bbd_send_packet fail!!\n", __func__);
		data->uTimeOutCnt++;
		clean_msg(msg);
		mutex_unlock(&data->pending_mutex);
		if (timeout) {
			wake_unlock(&data->ssp_comm_wake_lock);
		}
		mutex_unlock(&data->comm_mutex);
		return -1;
	}

	if (!use_no_irq) {
//		mutex_lock(&data->pending_mutex);	//moved UP
		list_add_tail(&msg->list, &data->pending_list);
//		mutex_unlock(&data->pending_mutex);	//moved down
	}

	mutex_unlock(&data->pending_mutex);

	if (status == 1 && done != NULL) {
		dprint("waiting completion ...\n");
		if (wait_for_completion_timeout(done, msecs_to_jiffies(timeout)) == 0) {
			ssp_dump_all_status("completion tout", start_pkt_acnt);
			status = -2;

			if (!use_no_irq && !msg_dead) {
				mutex_lock(&data->pending_mutex);
				if (msg->list.next!=NULL && msg->list.next!=LIST_POISON1)
					list_del(&msg->list);
				mutex_unlock(&data->pending_mutex);
			}
		}else{
			dprint("completion is cleared !\n");
		}
	}

	mutex_lock(&data->pending_mutex);
	if (msg != NULL && !msg_dead) {
		msg->done = NULL;
		msg->dead_hook = NULL;

		if (status != 1)
			msg->dead = true;
		if (status == -2)
			data->uTimeOutCnt++;
	}
	mutex_unlock(&data->pending_mutex);

	if (use_no_irq)
		clean_msg(msg);

	if (timeout) {
		wake_unlock(&data->ssp_comm_wake_lock);
	}
	mutex_unlock(&data->comm_mutex);

	return status;
}

/****************************************************************
 *
 * Callback fucntios
 *
 ****************************************************************/

/**
 * callback function:
 *	called this function by bbdpl when packet comes from MCU
 *
 * @ssh_data:	ssh data pointer
 *
 * @return:	0 = success, -1 = failure
 *
 */
int callback_bbd_on_packet_alarm(void *ssh_data)
{
	struct ssp_data *data = (struct ssp_data *)ssh_data;

	if (ssh_data == NULL) {
		pr_warn("[SSPBBD] %s: There are no ssp_data!!\n",
			__func__);
		return -1;
	}
#if 0
	/* check if already processing work queue */
	if(mutex_is_locked(&data->pending_mutex)) {
		dprint("already processing work queue\n");
		return 0;
	}
#endif
	packet_alarm_cnt++;

	if(queue_work(data->bbd_on_packet_wq, &data->work_bbd_on_packet)){
		/* in case of adding the work to queue */
		dprint("queue_work ok!!\n");
		return 0;
	}

	return 0;
}

/**
 * callback function
 *	called this function by bbdpl whenever mcu is(not) ready
 *
 * @ssh_data: 	ssh data pointer
 * @ready:	true = ready, false = not ready
 *
 * @return:	0 = success, -1 = failure
 */
int callback_bbd_on_mcu_ready(void *ssh_data, bool ready)
{
	struct ssp_data *data = (struct ssp_data *)ssh_data;

	if (data == NULL)
		return -1;

	if (ready == true) {
		/* Start queue work for initializing MCU */
		wake_lock_timeout(&data->ssp_wake_lock, 2 * HZ);
		queue_work(data->bbd_mcu_ready_wq, &data->work_bbd_mcu_ready);
	} else {
		/* If we've got new reset after booting, report that to FW */
		if (data->iHubState == EVT2FW_HUB_STATE_NOT_READY) {
			pr_warn("[SSPBBD] forcely skip reset state reporting at booting\n");
    } else if (data->bWOMode) {
			data->iHubState = EVT2FW_HUB_STATE_WOM_RESET;
			set_bit(EVT2FW_HUB_STATE_WOM_RESET,
				&data->ulHubStateBit);
			ssp_sensorhub_report_notice(data,
				MSG2SSP_HUB_STATUS_NOTI);
		} else {
			data->iHubState = EVT2FW_HUB_STATE_IN_RESET;
			set_bit(EVT2FW_HUB_STATE_IN_RESET,
				&data->ulHubStateBit);
			ssp_sensorhub_report_notice(data,
				MSG2SSP_HUB_STATUS_NOTI);
		}

		/* Disable SSP */
		ssp_enable(data, false);	
		dprint("MCU is not ready and disabled SSP\n");
	}

	return 0;
}

/**
 * callback function
 *	called this function by bbdpl when received control command from LHD
 *
 * @ssh_data:	ssh data pointer
 * @str_ctrl:	string type control command
 *
 * @return:	0 = success, -1 = failure
 */
int callback_bbd_on_control(void *ssh_data, const char *str_ctrl)
{
	if(!ssh_data || !str_ctrl)
		return -1;

	dprint("Received string command from LHD(=%s)\n", str_ctrl);

	return 0;
}

/**
 * callback function
 *	called this function by bbdpl to check current wom state
 *
 * @ssh_data:	ssh data pointer
 * @value:		value
 *
 * @return:	0 = success, -1 = failure
 */
int callback_bbd_get_wom_state(void *ssh_data, bool *value)
{
	struct ssp_data *data = (struct ssp_data *)ssh_data;

	if (!data || !value)
		return -1;

	*value = data->bWOMode;

	return 0;
}

/****************************************************************
 *
 * Work queue fucntios
 *
 ****************************************************************/

/**
 * Work queue function for MCU ready 
 *	This is called by bbdpl when MCU is ready and then
 *	initialize MCU

 * @work:	work structure
 *
 * @return:	none
 */
void bbd_mcu_ready_work_func(struct work_struct *work)
{
	struct ssp_data *data = container_of(work, struct ssp_data, work_bbd_mcu_ready);
	int ret = 0;
	int retries = 0;

	msleep(1000);
//	dprint("MCU is ready.(work_queue)\n");
	pr_err("[SSPBBD] MCU is ready.(work_queue)\n");

	clean_pending_list(data);

	ssp_enable(data, true);
retries:
	ret = initialize_mcu(data);
	if (ret != SUCCESS) {
#ifndef SSP_BIG_DATA_RESET_CNT
		data->uResetCnt++;
#endif
		mdelay(100);
		if(++retries > 3) {
			pr_err("[SSPBBD] fail to initialize mcu(%d)\n", ret);
			ssp_enable(data, false);
			return;
		}
		goto retries;
	}
	pr_err("[SSPBBD] mcu is initiialized (retries=%d)\n", retries);

	if (!data->bWOMode) {
		/* recover previous state */
		sync_sensor_state(data);
		ssp_sensorhub_report_notice(data, MSG2SSP_AP_STATUS_RESET);

		if (data->uLastAPState != 0)
			ssp_send_cmd(data, data->uLastAPState, 0);
		if (data->uLastResumeState != 0)
			ssp_send_cmd(data, data->uLastResumeState, 0);

		data->bSspReady = true;

		/* Final step to report the MCU state */
		data->iHubState = EVT2FW_HUB_STATE_READY_DONE;
		set_bit(EVT2FW_HUB_STATE_READY_DONE, &data->ulHubStateBit);
		ssp_sensorhub_report_notice(data, MSG2SSP_HUB_STATUS_NOTI);
	} else {
		pr_info("[SSPBBD] WOM enabled, skip some initial setting!!\n");

		data->iHubState = EVT2FW_HUB_STATE_WOM_READY_DONE;
		set_bit(EVT2FW_HUB_STATE_WOM_READY_DONE, &data->ulHubStateBit);
		ssp_sensorhub_report_notice(data, MSG2SSP_HUB_STATUS_NOTI);
	}

}

/**
 * This work queue fucntion starts from alarm callback function
 *
 * @work:	work structure
 *
 * @return:	none
 */
#define BBD_PULL_TIMEOUT	1000	/* the timeout for getting data from bbdpl */

#define MAX_SSP_DATA_SIZE	(4*BBD_MAX_DATA_SIZE)
unsigned char rBuff[MAX_SSP_DATA_SIZE] = {-1};

void bbd_on_packet_work_func(struct work_struct *work)
{
	struct ssp_data *data = container_of(work, struct ssp_data, work_bbd_on_packet);
	unsigned short chLength = 0, msg_options = 0;
	unsigned char msg_type = 0;
	int iRet = 0;
	unsigned char *pData = NULL, *p, *q;
	int nDataLen = 0;

	u64 timestamp;
	struct timespec ts;

	iRet = bbd_pull_packet(rBuff, sizeof(rBuff), BBD_PULL_TIMEOUT);
	if (iRet <= 0) {
		// This is not error condition because previous run of this function may have read all packets from BBD
		// This happens if this work function is scheduled while it starts running.
		pr_warn("[SSP]: %s: bbd_pull_packet timeout(%d)\n",
			__func__, iRet);
		return;
	}

	p = rBuff;
	q = rBuff + iRet;	// q points end of currently received data bytes

process_one:
	DEBUG_SHOW_HEX_RECV(p, 4)

	memcpy(&msg_options, p, 2); p+=2;
	msg_type = msg_options & SSP_SPI_MASK;
	memcpy(&chLength, p, 2); p+=2;
	pData = p;	// pData points current frame's data
	if (msg_type == AP2HUB_READ || msg_type == HUB2AP_WRITE)
	p += chLength;	// now p points next frame

	nDataLen = q - pData;

	// wait until we receive full frame
	while (q<p && q < rBuff+sizeof(rBuff)) {
		iRet = bbd_pull_packet(q, rBuff+sizeof(rBuff)-q, BBD_PULL_TIMEOUT);
		if (iRet<=0) {
			pr_err("[SSP]: %s bbd_pull_packet fail 2!! (iRet=%d)\n", __func__, iRet);
			return;
		}
		q += iRet;
		nDataLen += iRet;
	}

	switch (msg_type) {
	case AP2HUB_READ:
	case AP2HUB_WRITE:
		mutex_lock(&data->pending_mutex);
		if (!list_empty(&data->pending_list)) {
			struct ssp_msg *msg, *n;
			bool found = false;

			list_for_each_entry_safe(msg, n, &data->pending_list, list)
			{
				if (msg->options == msg_options) {
					list_del(&msg->list);
					found = true;
					break;
				}
			}

			if (!found) {
				pr_err("[SSP]: %s %d - Not match error\n", __func__, msg_options);
				goto exit;
			}

			if (msg->dead && !msg->free_buffer) {
				msg->buffer = (char*) kzalloc(msg->length, GFP_KERNEL);
				msg->free_buffer = 1;
			} // For dead msg, make a temporary buffer to read.

			if(msg->buffer == NULL) {
				pr_err("[SSPBBD]: %s() : msg->buffer is NULL\n", __func__);
				goto exit;
			}
			if (msg_type == AP2HUB_READ) {
				if(nDataLen <= 0) {
					dprint("Waiting 2nd message...(msg=%p, length=%d)\n",
						msg, msg->length);
					iRet = bbd_pull_packet(msg->buffer, msg->length, 
								BBD_PULL_TIMEOUT);
					dprint("Received 2nd message. (iRet=%d)\n",  iRet);
				}else{
					memcpy(msg->buffer, pData, msg->length);
					nDataLen -= msg->length;
				}
				DEBUG_SHOW_HEX_RECV(msg->buffer, msg->length)
			}
			if (msg_type == AP2HUB_WRITE) {
				iRet = bbd_send_packet(msg->buffer, msg->length);
				if(iRet <= 0) {
					pr_err("[SSP]: %s bbd_send_packet fail!!(AP2HUB_WRITE)\n",
						 __func__);
					goto exit;
				}

				DEBUG_SHOW_HEX_SEND(msg->buffer, msg->length)

				if (msg_options & AP2HUB_RETURN) {
					msg->options = AP2HUB_READ | AP2HUB_RETURN;
					msg->length = 1;
					list_add_tail(&msg->list, &data->pending_list);
					goto exit;
				}
			}
			if (msg->done != NULL && !completion_done(msg->done)){
				dprint("complete(mg->done)\n");
				complete(msg->done);
			}
			if (msg->dead_hook != NULL)
				*(msg->dead_hook) = true;

			clean_msg(msg);
		} else
			pr_err("[SSP]List empty error(%d)\n", msg_type);
exit:
		mutex_unlock(&data->pending_mutex);
		break;
	case HUB2AP_WRITE:
		{
		char* buffer = (char*) kzalloc(chLength, GFP_KERNEL);
		if (buffer == NULL) {
			pr_err("[SSP] %s, failed to alloc memory for buffer\n", __func__);
			iRet = -ENOMEM;
			break;
		}
		if(nDataLen <= 0) {
			dprint("Waiting 2nd message...(chLength=%d)\n", chLength);
			iRet = bbd_pull_packet(buffer, chLength, BBD_PULL_TIMEOUT);
			dprint("Received 2nd message. (iRet=%d)\n",  iRet);
		}else{
			memcpy(buffer, pData, chLength);
			iRet = chLength;
		}
		DEBUG_SHOW_HEX_RECV(buffer, chLength)
		if (iRet < 0)
			pr_err("[SSP] %s bbd_pull_packet fail.(iRet=%d)\n", __func__,iRet);
		else {
			ts = ktime_to_timespec(ktime_get_boottime());
			timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;			
			data->timestamp = timestamp;
			
			parse_dataframe(data, buffer, chLength);
		}
		//iRet = spi_read(data->spi, buffer, chLength);
		kfree(buffer);
		break;
		}
	default:
		pr_err("[SSP]No type error(%d)\n", msg_type);
		break;
	}

	if (iRet < 0) {
		pr_err("[SSP]: %s - MSG2SSP_SSD error %d\n", __func__, iRet);
	}

	if (p<q)
		goto process_one;
}
