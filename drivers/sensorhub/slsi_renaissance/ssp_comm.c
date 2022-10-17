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

#define LIMIT_DELAY_CNT		200
#define RECEIVEBUFFERSIZE	12
#define DEBUG_SHOW_DATA	0

#define SSP_CMD_SIZE 256
#define SSP_MSG_HEADER_SIZE 9
#define SSP_MSG_CP2AP_HEADER_SIZE	4

#define MAKE_WORD(H,L) ((((u16)H) << 8 ) & 0xff00 ) | ((((u16)L)) & 0x00ff )

#if 0
static void ssp_sensorhub_log(const char *func_name,
				const char *data, int length)
{
	char buf[6];
	char *log_str;
	int log_size;
	int i;

	if (likely(length <= BIG_DATA_SIZE))
		log_size = length;
	else
		log_size = PRINT_TRUNCATE * 2 + 1;

	log_size = sizeof(buf) * log_size + 1;
	log_str = kzalloc(log_size, GFP_ATOMIC);
	if (unlikely(!log_str)) {
		sensorhub_err("allocate memory for data log err");
		return;
	}

	for (i = 0; i < length; i++) {
		if (length < BIG_DATA_SIZE ||
			i < PRINT_TRUNCATE || i >= length - PRINT_TRUNCATE) {
			snprintf(buf, sizeof(buf), "0x%x", (signed char)data[i]);
			strlcat(log_str, buf, log_size);

			if (i < length - 1)
				strlcat(log_str, ", ", log_size);
		}
		if (length > BIG_DATA_SIZE && i == PRINT_TRUNCATE)
			strlcat(log_str, "..., ", log_size);
	}

	pr_info("[SSP]: %s - %s (%d)\n", func_name, log_str, length);
	kfree(log_str);
}
#endif

void handle_packet(struct ssp_data *data, char *packet, int packet_size)
{
#if 0
	ssp_sensorhub_log(__func__,packet,packet_size);
#else
	u16 msg_length = 0, msg_options = 0;
	u8 msg_cmd = 0;
	char *buffer;
	//char read_buffer[SSP_CMD_SIZE] = {0,};
	u64 timestamp;
	struct timespec ts;

#if 0
	if(packet_size < SSP_MSG_HEADER_SIZE) {
		pr_info("[SSP] %s nanohub packet size is small/(%s)", __func__, packet);
		return;
	}
#endif

	msg_options = MAKE_WORD(packet[1], packet[0]);;
	msg_length = MAKE_WORD(packet[3], packet[2]);

	if (msg_length == 0) {
		pr_err("[SSP] %s lengh is zero %d %d %d", __func__, msg_cmd, msg_length, msg_options);
		return;
	}

	if ((msg_options == AP2HUB_READ) || (msg_options == AP2HUB_WRITE)
			|| (msg_options == AP2HUB_RETURN)) {
		bool found = false;
		struct ssp_msg *msg, *n;

		pr_info("[SSP][Debug] %s %d %d %d", __func__, packet[4], msg_length, msg_options);

		mutex_lock(&data->pending_mutex);
		//spin_lock(&data->pending_wait.lock);
		if (!list_empty(&data->pending_list)) {
			list_for_each_entry_safe(msg, n, &data->pending_list, list) {

				if(msg->options == 0x05){
					msg->length = 1;
					msg->options = 0x04;
				}
				if ((msg->length == msg_length) && (msg->options == msg_options)) {
					list_del(&msg->list);
					found = true;
					break;
				}
			}

			if (!found) {
				pr_err("[SSP] %s %d %d %d - Not match error", __func__, msg_cmd, msg_length, msg_options);
				goto exit;
			}

			if (msg_options == AP2HUB_READ || (msg_options == AP2HUB_RETURN) ) {

				msg->length = msg_length;
				if (msg->length != 0) {
					if (msg->buffer != NULL) {
						kfree(msg->buffer);
					}
					msg->buffer = kzalloc(msg->length, GFP_KERNEL);
					memcpy(msg->buffer, packet + SSP_MSG_CP2AP_HEADER_SIZE, msg->length);
					pr_info("[SSP][Debug1] %s %d %d", __func__, msg->buffer[0], msg->length);
				} else {
					pr_err("[SSP] %s %d %d %d - error length 0", __func__, msg_cmd, msg_length, msg_options);
				}
			}

			if (msg->done != NULL && !completion_done(msg->done)) {
				complete(msg->done);
			}
		} else {
			pr_err("[SSP] %s List empty error(%d %d %d)", __func__, msg_cmd, msg_length, msg_options);
		}

exit:

		mutex_unlock(&data->pending_mutex);
		//spin_unlock(&data->pending_wait.lock);

	} else if (msg_options == HUB2AP_WRITE) {
		buffer = kzalloc(msg_length, GFP_KERNEL);
		memcpy(buffer, &packet[SSP_MSG_CP2AP_HEADER_SIZE], msg_length);

		ts = ktime_to_timespec(ktime_get_boottime());
		timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
		data->timestamp = timestamp;
		parse_dataframe(data, buffer, msg_length);
		kfree(buffer);
	} else {
		pr_info("[SSP] %s msg_cmd does not define. cmd is %d", __func__, msg_cmd);
	}

	return;
#endif
}





void clean_msg(struct ssp_msg *msg) {
	if (msg->free_buffer)
		kfree(msg->buffer);
	kfree(msg);
}

static char ssp_cmd_data[SSP_CMD_SIZE];

static int do_transfer(struct ssp_data *data, struct ssp_msg *msg, int timeout)
{
#if 0
	return 0;
#else
	int status = 0;
	int ret = 0;
	bool is_ssp_shutdown;
	u16 ulength = 0;

	pr_info("[SSP][Debug] %s msg->option = (0x%x), msg->cmd = (0x%x), msg->length(%d)",
					__func__, msg->options, msg->cmd, msg->length);

	mutex_lock(&data->comm_mutex);
	//spin_lock(&data->comm_wait.lock);

	if (!is_sensorhub_working(data)) {
		pr_err("[SSP] %s ssp shutdown, do not parse",__func__);
		mutex_unlock(&data->comm_mutex);
		//spin_unlock(&data->comm_wait.lock);

		return -EIO;
	}

	memcpy(ssp_cmd_data, msg, SSP_MSG_HEADER_SIZE);

	if (msg->length > 0) {
		memcpy(&ssp_cmd_data[SSP_MSG_HEADER_SIZE], msg->buffer, msg->length);
	} else if (msg->length > (SSP_CMD_SIZE - SSP_MSG_HEADER_SIZE)) {
		pr_err("[SSP] %s command size over !",__func__);
		mutex_unlock(&data->comm_mutex);
		//spin_unlock(&data->comm_wait.lock);
		return -EINVAL;
	}

	ulength = SSP_MSG_HEADER_SIZE + msg->length;
	ret = sensorhub_comms_write(data, ssp_cmd_data, ulength, timeout);

	if (ret != ulength) {
		pr_err("[SSP] %s comm write fail!!",__func__);
		status = ERROR;
		goto exit;
	}

	status = SUCCESS;

	if (msg->done != NULL) {
		mutex_lock(&data->pending_mutex);
		//spin_lock(&data->pending_wait.lock);
		list_add_tail(&msg->list, &data->pending_list);
		mutex_unlock(&data->pending_mutex);
		//spin_unlock(&data->pending_wait.lock);
	}

	exit:

	mutex_unlock(&data->comm_mutex);
	//spin_unlock(&data->comm_wait.lock);

	if (status < 0) {
		is_ssp_shutdown = !is_sensorhub_working(data);
		data->cnt_com_fail += (is_ssp_shutdown)? 0 : 1;
		pr_err("[SSP] %s cnt_com_fail %d , ssp_down %d ",__func__, data->cnt_com_fail, is_ssp_shutdown);

		return status;
	}

	if ((status >= 0) && (msg->done != NULL) && (timeout > 0)) {
		ret = wait_for_completion_timeout(msg->done,
		msecs_to_jiffies(timeout));

		if (msg->clean_pending_list_flag) {
			msg->clean_pending_list_flag = 0;
			pr_err("[SSP] %s communication fail so recovery_mcu func call",__func__);

			return -EINVAL;
		}

		/* when timeout is happened */
		if (!ret) {
			msg->done = NULL;
			list_del(&msg->list);
			is_ssp_shutdown = !is_sensorhub_working(data);
			data->cnt_timeout += (is_ssp_shutdown)? 0 : 1;

			if (msg->done != NULL) {
				list_del(&msg->list);
			}
			pr_err("[SSP] %s cnt_timeout %d, ssp_down %d !!",
				__func__, data->cnt_timeout, is_ssp_shutdown);

			return -EINVAL;
		}

	}

	return status;
#endif
}

void clean_pending_list(struct ssp_data *data) {
	struct ssp_msg *msg, *n;

	mutex_lock(&data->pending_mutex);
	//spin_lock(&data->pending_wait.lock);

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
	//spin_unlock(&data->pending_wait.lock);
}

#if 1
int ssp_send_command(struct ssp_data *data, u8 cmd, u16 options, int timeout,
                       char *send_buf, int send_buf_len, char **receive_buf, int *receive_buf_len, u32 send_data)
{
	int status = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	DECLARE_COMPLETION_ONSTACK(done);

	msg->cmd = cmd;
	msg->options = options;
	msg->length = send_buf_len;
	msg->data = send_data;

	if (timeout > 0) {
		if (send_buf != NULL) {
			msg->buffer = kzalloc(send_buf_len, GFP_KERNEL);
			memcpy(msg->buffer, send_buf, send_buf_len);
		} else {
			msg->buffer = send_buf;
		}
		msg->done = &done;
	} else {
		msg->buffer = kzalloc(send_buf_len, GFP_KERNEL);
		memcpy(msg->buffer, send_buf, send_buf_len);
		msg->done = NULL;
	}

	//pr_info("%s cmd %d options %d send_buf_len %d timeout %d", __func__, cmd, options, send_buf_len, timeout);

	if (do_transfer(data, msg, timeout) < 0) {
		pr_err("%s do_transfer error\n", __func__);
		status = ERROR;
	}
	else
		status = SUCCESS;

	//pr_info("%s options %d msg->buf[0]= %d, send_buf_len %d ",__func__, options, msg->buffer[0], send_buf_len);
	//mutex_lock(&data->cmd_mutex);
	if (((msg->options == AP2HUB_READ) && (receive_buf != NULL) &&
	     ((receive_buf_len != NULL) && (msg->length != 0))) &&
	    (status != ERROR)) {
		if (timeout > 0) {
			*receive_buf = kzalloc(msg->length, GFP_KERNEL);
			*receive_buf_len = msg->length;
			memcpy(*receive_buf, msg->buffer, msg->length);
			//pr_info("%s options %d receive_buf[0]= %d, send_buf_len %d ",__func__, options, , send_buf_len);
		} else {
			pr_err("%s AP2HUB_READ zero timeout", __func__);
			//mutex_unlock(&data->cmd_mutex);
			return -EINVAL;
		}
	}

	clean_msg(msg);
	//mutex_unlock(&data->cmd_mutex);

	return status;
}

#else
static int ssp_send_command(struct ssp_data *data, u8 cmd, u16 options, int timeout,
					char *send_buf, u16 send_buf_len, char **receive_buf, u16 *receive_buf_len)
{
	int status = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	DECLARE_COMPLETION_ONSTACK(done);

	msg->cmd = cmd;
	msg->options = options;
	msg->length = send_buf_len;

	if (timeout > 0) {
		if (send_buf != NULL) {
			msg->buffer = kzalloc(send_buf_len, GFP_KERNEL);
			memcpy(msg->buffer, send_buf, send_buf_len);
		} else {
			msg->buffer = send_buf;
		}
		msg->done = &done;
	} else {
		msg->buffer = kzalloc(send_buf_len, GFP_KERNEL);
		memcpy(msg->buffer, send_buf, send_buf_len);
		msg->done = NULL;
	}

	pr_info("[SSP] %s cmd %d options %d send_buf_len %d timeout %d",
							__func__, cmd, options, send_buf_len, timeout);

	status = do_transfer(data, msg, timeout);
	if (status != SUCCESS) {
		pr_err("[SSP] %s do_transfer error", __func__);
		status = ERROR;
	}

	if (((msg->cmd == AP2HUB_READ) && (receive_buf != NULL) &&
	     ((receive_buf_len != NULL) && (msg->length != 0))) &&
	    (status != ERROR)) {
		if (timeout > 0) {
			*receive_buf = kzalloc(msg->length, GFP_KERNEL);
			*receive_buf_len = msg->length;
			memcpy(*receive_buf, msg->buffer, msg->length);

			pr_err("[SSP][Debug] %s status(%d), recive_buf(%d), recive_length(%d)",
		               __func__, status, msg->buffer[0], msg->length);
			ssp_sensorhub_log(__func__, msg->buffer, msg->length);
		} else {
			pr_err("[SSP]	%s AP2HUB_READ zero timeout", __func__);
			return -EINVAL;
		}
	}
	else
	{

	}
	pr_err("[SSP][Debug] %s status(%d), recive_buf(%d), recive_length(%d)",
		               __func__, status, msg->buffer[0], msg->length);

	clean_msg(msg);

	if(status < 0) {
		if(data->is_reset_started == false) {
			recovery_mcu(data);
		}
		pr_err("[SSP] %s status=%d, is_reset_started=%d", __func__, status, data->is_reset_started);
	}
	return status;
}

static int make_command(struct ssp_data *data, struct ssp_msg *msg)
{
	int ret = 0;
	char *buffer = NULL;

#if 0
	if (data->fw_dl_state == FW_DL_STATE_DOWNLOADING) {
		ssp_errf("Skip make command! DL state = %d", data->fw_dl_state);
		return SUCCESS;
	} else
#endif
  if (!(data->sensor_probe_state)) {
		pr_err("[SSP] %s skip! - %u", __func__, msg->cmd);
		return FAIL;
	}

	buffer = kzalloc(msg->length, GFP_KERNEL);
	memcpy(buffer, msg->buffer, msg->length);

	ret = ssp_send_command(data, msg->cmd, msg->options, 0,
												buffer, msg->length, NULL, NULL);

	if (ret != SUCCESS) {
		pr_err("[SSP] %s ssp_send_command(0x%x) Fail %d", __func__, msg->cmd, ret);
		goto exit;
	}

exit:
	if (buffer != NULL) {
		kfree(buffer);
	}
	return ret;
}
#endif

int ssp_ipc_async(struct ssp_data *data, struct ssp_msg *msg)
{
#if 1
	return 0;
#else
	int status = 0;
	char *buffer = NULL;

	if (msg->length){
		buffer = kzalloc(msg->length, GFP_KERNEL);
		memcpy(buffer, msg->buffer, msg->length);
		status = ssp_send_command(data, msg->cmd, msg->options, 2000,
													buffer, msg->length, &msg->buffer, &msg->length);

		if (status != SUCCESS) {
			pr_err("[SSP] %s ssp_send_command(0x%x) Fail %d", __func__, msg->cmd, status);
			goto exit;
		}
	}
	else
		status = make_command(data, msg);

exit:
	if (buffer != NULL) {
		kfree(buffer);
	}
	return status;
#endif
}

int ssp_ipc_sync(struct ssp_data *data, struct ssp_msg *msg, int timeout) {
#if 1
	return 0;
#else
	DECLARE_COMPLETION_ONSTACK(done);
	int status = 0;
	char *buffer = NULL;

	if (msg->length == 0) {
		pr_err("[SSP]: %s length must not be 0\n", __func__);
		clean_msg(msg);
		return status;
	}
	buffer = kzalloc(msg->length, GFP_KERNEL);
	memcpy(buffer, msg->buffer, msg->length);

	status = ssp_send_command(data, msg->cmd, msg->options, timeout,
													buffer, msg->length, &msg->buffer, &msg->length);
	if (status != SUCCESS) {
		pr_err("[SSP] %s ssp_send_command(0x%x) Fail %d", __func__, msg->cmd, status);
		goto exit;
	}

exit:
	if (buffer != NULL) {
		kfree(buffer);
	}
	return status;
#endif
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
	msg->length = 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(1, GFP_KERNEL);
	msg->data = arg;
	msg->free_buffer = 0;

#if 0
		iRet = ssp_ipc_async(data, msg);
#else
		iRet = ssp_send_command(data, msg->cmd, msg->options, 0,
													 msg->buffer, msg->length, NULL, NULL, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - command 0x%x failed %d\n",
				__func__, command, iRet);
		kfree(msg);
		return ERROR;
	}

	ssp_dbg("[SSP]: %s - command 0x%x %d\n", __func__, command, arg);
	kfree(msg);

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

	ssp_dbg("[SSP]: %s - Inst = 0x%x, Sensor Type = 0x%x, data = %u\n",
			__func__, command, uSensorType, msg->buffer[1]);

#if 0
		iRet = ssp_ipc_async(data, msg);
#else
		iRet = ssp_send_command(data, msg->cmd, msg->options, 0,
													 msg->buffer, msg->length, NULL, NULL, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Instruction CMD Fail %d\n", __func__, iRet);
		kfree(msg);
		return ERROR;
	}

	kfree(msg);
	return iRet;
}

int send_instruction_sync(struct ssp_data *data, u8 uInst,
	u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char *rbuf = NULL;
	int rbuf_length = 0;
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
#if 0
	iRet = ssp_ipc_sync(data, msg, timeout);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, timeout,
													msg->buffer, msg->length, &rbuf, &rbuf_length, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Instruction CMD Fail %d\n", __func__, iRet);
		kfree(msg);
		return ERROR;
	}
	memcpy(&buffer, rbuf, rbuf_length);

	kfree(msg);
	return buffer[0];
}

int get_chipid(struct ssp_data *data)
{
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	int ret;
	char *buffer = NULL;
	char send_buf[1] = {0,};
	int buffer_length = 0;

	msg->cmd = MSG2SSP_AP_WHOAMI;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = send_buf;
	msg->free_buffer = 0;

	ret = ssp_send_command(data, msg->cmd, msg->options, 2000,
	                       msg->buffer, msg->length, &buffer, &buffer_length, msg->data);

	if (ret != SUCCESS) {
		pr_err("[SSP][Debug] get_firmware_rev error %d", ret);
	} else if (buffer_length != msg->length) {
		pr_err("[SSP][Debug] get_firmware_rev VERSION_INFO length is wrong");
	} else {
		memcpy(&send_buf, buffer, buffer_length);
		pr_err("[SSP][Debug] get_firmware_rev %d", send_buf[0]);
	}

	kfree(msg);
	return send_buf[0];
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

#if 0
	iRet = ssp_ipc_sync(data, msg, 2000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 0,
	                       msg->buffer, msg->length, NULL, NULL, msg->data);
#endif

	pr_info("[SSP] Sensor Posision A : %u, G : %u, M: %u, P: %u\n",
			data->accel_position, data->accel_position, data->mag_position, 0);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s -fail to set_sensor_position %d\n", __func__, iRet);
		iRet = ERROR;
	}

	kfree(msg);
	return iRet;
}

u64 get_sensor_scanning_info(struct ssp_data *data) {
	int iRet = 0, z = 0;
	int length = 0;
	char *buffer = NULL;
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
	msg->buffer = (char*) kzalloc(8, GFP_KERNEL);
	msg->free_buffer = 0;

#if 0
	iRet = ssp_ipc_sync(data, msg, 3000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
	                       msg->buffer, msg->length, &buffer, &length, msg->data);
#endif
	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - ipc fail %d\n", __func__, iRet);

	memcpy(&result, buffer, length);

	data->sensor_probe_state = result;
	bin[SENSOR_MAX] = '\0';
	for (z = 0; z < SENSOR_MAX; z++)
		bin[SENSOR_MAX - 1 - z] = (result & (1 << z)) ? '1' : '0';
	pr_err("[SSP] %s state: %s\n",__func__, bin);

	kfree(msg);
	return result;
}

unsigned int get_firmware_rev(struct ssp_data *data) {
	char *rbuf = NULL;
	int rbuf_length = 0;
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
#if 0
	iRet = ssp_ipc_sync(data, msg, 1000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 1000,
													 msg->buffer, msg->length, &rbuf, &rbuf_length, msg->data);
#endif
	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);

	memcpy(&result, rbuf, rbuf_length);
	kfree(msg);
	return result;
}

unsigned int get_feature_list(struct ssp_data *data) {
	int iRet = 0;
	int length = 0;
	char *buffer = NULL;
	unsigned int result = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_FEATURE_LIST_INFO;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->buffer = (char*) kzalloc(4, GFP_KERNEL);
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
	                       msg->buffer, msg->length, &buffer, &length, msg->data);

	if (iRet != SUCCESS){
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);
	}
	else{
		memcpy(&result, buffer, length);
	}

	pr_err("[SSP]: %s - 0x%x (%d)\n", __func__, result, length);


	if(msg->buffer != NULL){
		kfree(msg->buffer);
	}

	kfree(msg);
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

#if 0
		iRet = ssp_ipc_async(data, msg);
#else
		iRet = ssp_send_command(data, msg->cmd, msg->options, 0,
													 msg->buffer, msg->length, NULL, NULL, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);
		iRet = ERROR;
	}

	kfree(msg);
	return iRet;
}

#ifdef CONFIG_SSP_RTC
unsigned int get_rtc_diff(struct ssp_data *data)
{
	char *buf = NULL;
	int buf_length = 0;
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
#if 0
	iRet = ssp_ipc_sync(data, msg, 1000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 1000,
	msg->buffer, msg->length, &buf, &buf_length, msg->data);
#endif

	if (iRet != SUCCESS)
		pr_err("[SSP] : %s - transfer fail %d\n", __func__, iRet);

	memcpy(&result, buf, buf_length);
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

#if 0
		iRet = ssp_ipc_async(data, msg);
#else
		iRet = ssp_send_command(data, msg->cmd, msg->options, 0,
													 msg->buffer, msg->length, NULL, NULL, msg->data);
#endif
	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - fail %d\n",
				__func__, iRet);
		kfree(msg);
		return iRet;
	}
	pr_debug("[SSP] %s -> duration:%d flag:%d\n", __func__, data->motor_duration, data->motor_flag);
	kfree(msg);
	return (data->motor_flag << 16 | data->motor_duration);
}
#endif

int sensorhub_dump_read(struct ssp_data *data, u8* buffer)
{
	char *buf = NULL;
	int buf_length = 0;
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
#if 0
	iRet = ssp_ipc_sync(data, msg, 3000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
													msg->buffer, msg->length, &buf, &buf_length, msg->data);
#endif

	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - spi fail %d\n", __func__, iRet);

	memcpy(&buffer, buf, buf_length);
	pr_info("[SSP]: %s - sensorhub dump end\n", __func__);

	kfree(msg);
	return iRet;
}
