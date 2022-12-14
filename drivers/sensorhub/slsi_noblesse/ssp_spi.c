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
#include "ssp.h"
#ifdef CONFIG_SENSORHUB_STAT
#include <linux/sensorhub_stat.h>
#endif

#define LIMIT_DELAY_CNT		200
#define RECEIVEBUFFERSIZE	12
#define DEBUG_SHOW_DATA	0

int bbd_do_transfer(struct ssp_data *data, struct ssp_msg *msg,
		struct completion *done, int timeout);

void clean_msg(struct ssp_msg *msg) {
	if (msg->free_buffer)
		kfree(msg->buffer);
	kfree(msg);
}

static int do_transfer(struct ssp_data *data, struct ssp_msg *msg,
		struct completion *done, int timeout)
{
	return bbd_do_transfer(data, msg, done, timeout);
}

int ssp_spi_async(struct ssp_data *data, struct ssp_msg *msg)
{
	int status = 0;

	if (msg->length)
		return ssp_spi_sync(data, msg, 2000);

	status = do_transfer(data, msg, NULL, 0);

	return status;
}

int ssp_spi_sync(struct ssp_data *data, struct ssp_msg *msg, int timeout) {
	DECLARE_COMPLETION_ONSTACK(done);
	int status = 0;

	if (msg->length == 0) {
		pr_err("[SSP]: %s length must not be 0\n", __func__);
		clean_msg(msg);
		return status;
	}

	status = do_transfer(data, msg, &done, timeout);

	return status;
}

int select_irq_msg(struct ssp_data *data) {
	struct ssp_msg *msg, *n;
	bool found = false;
	u16 chLength = 0, msg_options = 0;
	u8 msg_type = 0;
	int iRet = 0;
	char* buffer;
	char chTempBuf[4] = { -1 };

	data->bHandlingIrq = true;
	iRet = spi_read(data->spi, chTempBuf, sizeof(chTempBuf));
	if (iRet < 0) {
		pr_err("[SSP] %s spi_read fail, ret = %d\n", __func__, iRet);
		data->bHandlingIrq = false;
		return iRet;
	}

	memcpy(&msg_options, &chTempBuf[0], 2);
	msg_type = msg_options & SSP_SPI_MASK;
	memcpy(&chLength, &chTempBuf[2], 2);

	switch (msg_type) {
	case AP2HUB_READ:
	case AP2HUB_WRITE:
		mutex_lock(&data->pending_mutex);
		if (!list_empty(&data->pending_list)) {
			list_for_each_entry_safe(msg, n, &data->pending_list, list)
			{
				if (msg->options == msg_options) {
					list_del(&msg->list);
					found = true;
					goto exit;
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

			if (msg_type == AP2HUB_READ)
				iRet = spi_read(data->spi, msg->buffer, msg->length);
			if (msg_type == AP2HUB_WRITE) {
				iRet = spi_write(data->spi, msg->buffer, msg->length);

				if (msg_options & AP2HUB_RETURN) {
					msg->options = AP2HUB_READ | AP2HUB_RETURN;
					msg->length = 1;
					list_add_tail(&msg->list, &data->pending_list);
					goto exit;
				}
			}

			if (msg->done != NULL && !completion_done(msg->done))
				complete(msg->done);
			if (msg->dead_hook != NULL)
				*(msg->dead_hook) = true;

			clean_msg(msg);
		} else
			pr_err("[SSP]List empty error(%d)\n", msg_type);
	exit:
		mutex_unlock(&data->pending_mutex);
		break;
	case HUB2AP_WRITE:
		buffer = (char*) kzalloc(chLength, GFP_KERNEL);
		if (buffer == NULL) {
			pr_err("[SSP] %s, failed to alloc memory for buffer\n", __func__);
			iRet = -ENOMEM;
			break;
		}
		iRet = spi_read(data->spi, buffer, chLength);
		if (iRet < 0)
			pr_err("[SSP] %s spi_read fail\n", __func__);
		else
			parse_dataframe(data, buffer, chLength);
		kfree(buffer);
		break;
	default:
		pr_err("[SSP]No type error(%d)\n", msg_type);
		break;
	}

	if (iRet < 0) {
		pr_err("[SSP]: %s - MSG2SSP_SSD error %d\n", __func__, iRet);
		data->bHandlingIrq = false;
		return ERROR;
	}

	data->bHandlingIrq = false;
	return SUCCESS;
}

void clean_pending_list(struct ssp_data *data) {
	struct ssp_msg *msg, *n;

	mutex_lock(&data->pending_mutex);

	list_for_each_entry_safe(msg, n, &data->pending_list, list)
	{
		list_del(&msg->list);
		if (msg->done != NULL && !completion_done(msg->done))
			complete(msg->done);
		if (msg->dead_hook != NULL)
			*(msg->dead_hook) = true;

		clean_msg(msg);
	}
	mutex_unlock(&data->pending_mutex);
}

int ssp_send_cmd(struct ssp_data *data, char command, int arg)
{
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = command;
	msg->length = 0;
	msg->options = AP2HUB_WRITE;
	msg->data = arg;
	msg->free_buffer = 0;

	iRet = ssp_spi_async(data, msg);
	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - command 0x%x failed %d\n",
				__func__, command, iRet);
		return ERROR;
	}

	ssp_dbg("[SSP]: %s - command 0x%x %d\n", __func__, command, arg);

	return SUCCESS;
}

int send_instruction(struct ssp_data *data, u8 uInst,
	u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command;
	int iRet = 0;
	struct ssp_msg *msg;

	u64 timestamp;
	struct timespec ts;

	if ((!(data->uSensorState & (1 << uSensorType)))
		&& (uInst <= CHANGE_DELAY)) {
		pr_err("[SSP]: %s - Bypass Inst Skip! - %u\n",
			__func__, uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_REMOVE;
		break;
	case ADD_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_ADD;
		ts = ktime_to_timespec(ktime_get_boottime());
		timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

		if (data->cameraGyroSyncMode && uSensorType == GYROSCOPE_SENSOR) {
			data->lastTimestamp[uSensorType] = 0ULL;
		} else {
			data->lastTimestamp[uSensorType] = timestamp + 5000000ULL;
		}
		break;
	case CHANGE_DELAY:
		command = MSG2SSP_INST_CHANGE_DELAY;
		break;
	case GO_SLEEP:
		command = MSG2SSP_AP_STATUS_SLEEP;
		data->uLastAPState = MSG2SSP_AP_STATUS_SLEEP;
		break;
	case REMOVE_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_REMOVE;
		break;
	case ADD_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_ADD;
		break;
	default:
		pr_debug("[SSP] %s, Just passthrough Inst = 0x%x\n",
			__func__, uInst);
		command = uInst;
		break;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		iRet = -ENOMEM;
		return iRet;
	}
	msg->cmd = command;
	msg->length = uLength + 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(uLength + 1, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = uSensorType;
	memcpy(&msg->buffer[1], uSendBuf, uLength);

#ifdef CONFIG_SENSORHUB_STAT
	sensorhub_stat_rcv_instruction(command, uSensorType, uSendBuf, uLength);
#endif

	ssp_dbg("[SSP]: %s - Inst = 0x%x, Sensor Type = 0x%x, data = %u\n",
			__func__, command, uSensorType, msg->buffer[1]);

	iRet = ssp_spi_async(data, msg);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Instruction CMD Fail %d\n", __func__, iRet);
		return ERROR;
	}

	return iRet;
}

int send_instruction_sync(struct ssp_data *data, u8 uInst,
	u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command;
	int iRet = 0, timeout = 1000;
	char buffer[10] = { 0, };
	struct ssp_msg *msg;

	u64 timestamp;
	struct timespec ts;

	if ((!(data->uSensorState & (1 << uSensorType)))
		&& (uInst <= CHANGE_DELAY)) {
		pr_err("[SSP]: %s - Bypass Inst Skip! - %u\n",
			__func__, uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_REMOVE;
		break;
	case ADD_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_ADD;
		ts = ktime_to_timespec(ktime_get_boottime());
		timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

		if (data->cameraGyroSyncMode && uSensorType == GYROSCOPE_SENSOR) {
			data->lastTimestamp[uSensorType] = 0ULL;
		} else {
			data->lastTimestamp[uSensorType] = timestamp + 5000000ULL;
		}
		break;
	case CHANGE_DELAY:
		command = MSG2SSP_INST_CHANGE_DELAY;
		break;
	case GO_SLEEP:
		command = MSG2SSP_AP_STATUS_SLEEP;
		data->uLastAPState = MSG2SSP_AP_STATUS_SLEEP;
		break;
	case REMOVE_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_REMOVE;
		break;
	case ADD_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_ADD;
		break;
  case EXT_CMD:
    if (uLength > 10) {
      pr_err("[SSP]: %s exceed size(%u) for cmd(0x%x)!!\n",
        __func__, uLength, uSensorType);
      return -EINVAL;
    }
    timeout = 3000;
    command = uSensorType;
    break;
	default:
		pr_info("[SSP] %s, Just passthrough Inst = 0x%x\n",
			__func__, uInst);
		command = uInst;
		break;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = command;

	msg->options = AP2HUB_WRITE | AP2HUB_RETURN;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	if (uInst == EXT_CMD) {
		msg->length = uLength;
		memcpy(&msg->buffer[0], uSendBuf, uLength);
	} else {
		msg->length = uLength + 1;
		msg->buffer[0] = uSensorType;
		memcpy(&msg->buffer[1], uSendBuf, uLength);
	};

	ssp_dbg("[SSP]: %s - Inst Sync = 0x%x, Sensor Type = %u, data = %u\n",
			__func__, command, uSensorType, msg->buffer[0]);

	iRet = ssp_spi_sync(data, msg, timeout);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Instruction CMD Fail %d\n", __func__, iRet);
		return ERROR;
	}

	return buffer[0];
}

int get_chipid(struct ssp_data *data)
{
	int iRet, iReties = 0;
	char buffer = 0;
	struct ssp_msg *msg;

retries:
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_WHOAMI;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = &buffer;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 1000);

	if (buffer != DEVICE_ID && iReties++ < 2) {
	 mdelay(5);
	 pr_err("[SSP] %s - get chip ID retry\n", __func__);
	 goto retries;
	}

	if (iRet == SUCCESS)
		return buffer;

	pr_err("[SSP] %s - get chip ID failed %d\n", __func__, iRet);
	return ERROR;
}

int set_sensor_position(struct ssp_data *data)
{
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_FORMATION;
	msg->length = 3;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(3, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = data->accel_position;
	msg->buffer[1] = data->accel_position;
	msg->buffer[2] = data->mag_position;

	iRet = ssp_spi_async(data, msg);

	pr_info("[SSP] Sensor Posision A : %u, G : %u, M: %u, P: %u\n",
			data->accel_position, data->accel_position, data->mag_position, 0);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s -fail to set_sensor_position %d\n", __func__, iRet);
		iRet = ERROR;
	}

	return iRet;
}

u64 get_sensor_scanning_info(struct ssp_data *data) {
	int iRet = 0, z = 0;
	u64 result = 0;
	char bin[SENSOR_MAX + 1];

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_SCANNING;
	msg->length = 8;
	msg->options = AP2HUB_READ;
	msg->buffer = (char*) &result;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 3000);

	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);

	bin[SENSOR_MAX] = '\0';
	for (z = 0; z < SENSOR_MAX; z++)
		bin[SENSOR_MAX - 1 - z] = (result & (1 << z)) ? '1' : '0';
	pr_err("[SSP]: state: %s\n", bin);

	return result;
}

unsigned int get_firmware_rev(struct ssp_data *data) {
	int iRet;
	u32 result = SSP_INVALID_REVISION;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_FIRMWARE_REV;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->buffer = (char*) &result;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 1000);
	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);

	return result;
}

int set_big_data_start(struct ssp_data *data, u8 type, u32 length) {
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_START_BIG_DATA;
	msg->length = 5;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(5, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = type;
	memcpy(&msg->buffer[1], &length, 4);

	iRet = ssp_spi_async(data, msg);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);
		iRet = ERROR;
	}

	return iRet;
}

int set_time(struct ssp_data *data) {
	int iRet;
	struct ssp_msg *msg;
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("[SSP]: %s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n", __func__,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
			tm.tm_sec, ts.tv_nsec);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_SET_TIME;
	msg->length = 12;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(12, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = tm.tm_hour;
	msg->buffer[1] = tm.tm_min;
	msg->buffer[2] = tm.tm_sec;
	msg->buffer[3] = tm.tm_hour > 11 ? 64 : 0;
	msg->buffer[4] = tm.tm_wday;
	msg->buffer[5] = tm.tm_mon + 1;
	msg->buffer[6] = tm.tm_mday;
	msg->buffer[7] = tm.tm_year % 100;
	memcpy(&msg->buffer[8], &ts.tv_nsec, 4);

	iRet = ssp_spi_async(data, msg);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);
		iRet = ERROR;
	}

	return iRet;
}

int get_time(struct ssp_data *data) {
	int iRet;
	char buffer[12] = { 0, };
	struct ssp_msg *msg;
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("[SSP]: %s ap %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n", __func__,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
			tm.tm_sec, ts.tv_nsec);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_GET_TIME;
	msg->length = 12;
	msg->options = AP2HUB_READ;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 1000);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - i2c failed %d\n", __func__, iRet);
		return 0;
	}

	tm.tm_hour = buffer[0];
	tm.tm_min = buffer[1];
	tm.tm_sec = buffer[2];
	tm.tm_mon = msg->buffer[5] - 1;
	tm.tm_mday = buffer[6];
	tm.tm_year = buffer[7] + 100;
	rtc_tm_to_time(&tm, &ts.tv_sec);
	memcpy(&ts.tv_nsec, &msg->buffer[8], 4);

	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("[SSP]: %s mcu %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n", __func__,
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
			tm.tm_sec, ts.tv_nsec);

	return iRet;
}

#ifdef CONFIG_SSP_RTC
unsigned int get_rtc_diff(struct ssp_data *data)
{
    int iRet;
    u64 result = 0;

    struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
    if (msg == NULL) {
        pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
                __func__);
        return -ENOMEM;
    }
    msg->cmd = MSG2SSP_AP_RTC_DIFF;
    msg->length = 4;
    msg->options = AP2HUB_READ;
    msg->buffer = (char *) &result;
    msg->free_buffer = 0;

    iRet = ssp_spi_sync(data, msg, 1000);
    if (iRet != SUCCESS)
        pr_err("[SSP] : %s - transfer fail %d\n", __func__, iRet);

    return result;
}
#endif

#ifdef CONFIG_INPUT_FF_MEMLESS_NOTIFY
int send_motor_state(struct ssp_data *data)
{
	int iRet = 0;
	struct ssp_msg *msg;

	pr_debug("[SSP] %s start\n", __func__);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_SET_MOTOR_STATUS;
	msg->length = 3;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(3, GFP_KERNEL);
	msg->free_buffer = 1;

	/*Set duration*/
	msg->buffer[0] = (char)(data->motor_duration & 0x00FF);
	msg->buffer[1] = (char)((data->motor_duration & 0xFF00)>>8);
	msg->buffer[2] = (unsigned char)(data->motor_flag);

	iRet = ssp_spi_async(data, msg);
	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - fail %d\n",
				__func__, iRet);
		return iRet;
	}
	pr_debug("[SSP] %s -> duration:%d flag:%d\n", __func__, data->motor_duration, data->motor_flag);
	return (data->motor_flag << 16 | data->motor_duration);
}
#endif

int sensorhub_dump_read(struct ssp_data *data, u8* buffer)
{
	int iRet = 0;
	struct ssp_msg *msg;

	pr_info("[SSP]: %s - sensorhub dump start\n", __func__);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_SENSOR_REG_DUMP;
	msg->length = SENSORHUB_DUMP_SIZE;
	msg->options = AP2HUB_READ;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 3000);

	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - spi fail %d\n", __func__, iRet);

	pr_info("[SSP]: %s - sensorhub dump end\n", __func__);

	return iRet;
}
