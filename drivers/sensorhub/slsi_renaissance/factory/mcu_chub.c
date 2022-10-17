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
#include "../ssp.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define SMART_ALERT_MOTION  8
#define LPM_AUTO_ROTATION   7

ssize_t mcu_revision_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	unsigned int module_rev = 0;

	module_rev = get_module_rev(data);

	pr_info("[SSP]: %s - [%8u], [%8u] \n", __func__,
				data->uCurFirmRev, module_rev);

	return snprintf(buf, PAGE_SIZE, "SLSI%8u,SLSI%8u\n", data->uCurFirmRev,
			module_rev);
}

ssize_t mcu_model_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->model_name);
}

ssize_t mcu_update_kernel_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssp_dbg("[SSPBBD]: %s:%d: Ignored some code section.\n",
		__func__, __LINE__);
	return snprintf(buf, PAGE_SIZE, "NG\n");
}

ssize_t mcu_update_kernel_crashed_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssp_dbg("[SSPBBD]: %s:%d: Ignored some code section.\n",
		__func__, __LINE__);
	return snprintf(buf, PAGE_SIZE, "NG\n");
}

ssize_t mcu_update_ums_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssp_dbg("[SSPBBD]: %s:%d: Ignored some code section.\n",
		__func__, __LINE__);
	return snprintf(buf, PAGE_SIZE, "NG\n");
}

ssize_t mcu_sensor_state(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%llu\n", data->uSensorState);
}

ssize_t mcu_reset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	pr_info("[SSP]: %s, reset ssp mcu\n", __func__);
	reset_mcu(data);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}

ssize_t mcu_ready_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	if((is_sensorhub_working(data)) && (data->is_reset_started == false))
		data->bSspReady = true;
	else
		data->bSspReady = false;

	return snprintf(buf, PAGE_SIZE, "%d\n", data->bSspReady);
}

ssize_t mcu_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int status = 1, iDelaycnt = 0;

	data->bDumping = true;
	set_big_data_start(data, BIG_TYPE_DUMP, 0);
	msleep(300);
	while (data->bDumping) {
		mdelay(10);
		if (iDelaycnt++ > 1000) {
			status = 0;
			break;
		}
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", status ? "OK" : "NG");
}

static char buffer[FACTORY_DATA_MAX];

ssize_t mcu_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	struct ssp_msg *msg;
	int buffer_length = 0;
	char *temp_buf;

	if (sysfs_streq(buf, "1")) {
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		if (msg == NULL) {
			pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
				__func__);
			return -ENOMEM;
		}
		temp_buf = kzalloc(5, GFP_KERNEL);
		msg->cmd = MCU_FACTORY;
		msg->length = 5;
		msg->options = AP2HUB_READ;
		msg->buffer = temp_buf;
		msg->free_buffer = 0;

		memset(msg->buffer, 0, 5);
#if 0
		iRet = ssp_spi_async(data, msg);
#else
		iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
			msg->buffer, msg->length, &temp_buf, &buffer_length, msg->data);
#endif

		memcpy(buffer, temp_buf, 5);

		pr_info("%s : %d %d %d %d %d\n",  __func__, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
		kfree(msg);
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ssp_dbg("[SSP]: MCU Factory Test Start! - %d\n", iRet);

	return size;
}

ssize_t mcu_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bMcuTestSuccessed = false;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->bSspShutdown == true) {
		ssp_dbg("[SSP]: %s - MCU Bin is crashed\n", __func__);
		return snprintf(buf, PAGE_SIZE, "NG,NG,NG\n");
	}

	pr_info("[SSP] MCU Factory Test Data : %u, %u, %u, %u, %u\n", buffer[0],
			buffer[1], buffer[2], buffer[3], buffer[4]);

		/* system clock, RTC, I2C Master, I2C Slave, externel pin */
	if ((buffer[0] == SUCCESS)
			&& (buffer[1] == SUCCESS)
			&& (buffer[2] == SUCCESS)
			&& (buffer[3] == SUCCESS)
			&& (buffer[4] == SUCCESS))
		bMcuTestSuccessed = true;

	ssp_dbg("[SSP]: MCU Factory Test Result - %s, %s, %s\n",
		data->model_name, (bMcuTestSuccessed ? "OK" : "NG"), "OK");

	return snprintf(buf, PAGE_SIZE, "%s,%s,%s\n", data->model_name,
		(bMcuTestSuccessed ? "OK" : "NG"), "OK");
}

ssize_t mcu_sleep_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	struct ssp_msg *msg;
	int buffer_length = 0;
	char *temp_buf;

	if (sysfs_streq(buf, "1")) {
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		if (msg == NULL) {
			pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
				__func__);
			return -ENOMEM;
		}
		temp_buf = kzalloc(FACTORY_DATA_MAX, GFP_KERNEL);
		msg->cmd = MCU_SLEEP_FACTORY;
		msg->length = FACTORY_DATA_MAX;
		msg->options = AP2HUB_READ;
		msg->buffer = temp_buf;
		msg->free_buffer = 0;
#if 0
		// iRet = ssp_spi_async(data, msg);
		iRet = ssp_spi_sync(data, msg, 10000);
#else
		iRet = ssp_send_command(data, msg->cmd, msg->options, 10000,
			msg->buffer, msg->length, &temp_buf, &buffer_length, msg->data);

		memcpy(buffer, temp_buf, FACTORY_DATA_MAX);
		kfree(msg);
#endif
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ssp_dbg("[SSP]: MCU Sleep Factory Test Start! - %d\n", iRet);

	return size;
}

ssize_t mcu_sleep_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int iDataIdx, iSensorData = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct sensor_value *fsb;
	u16 chLength = 0;
	int count = 0;

	fsb = kzalloc(sizeof(struct sensor_value)*SENSOR_MAX, GFP_KERNEL);

	memcpy(&chLength, buffer, 2);
	memset(fsb, 0, sizeof(struct sensor_value) * SENSOR_MAX);

	for (iDataIdx = 2; iDataIdx < chLength + 2;) {
		iSensorData = (int)buffer[iDataIdx++];

		if ((iSensorData < 0) ||
			(iSensorData >= (SENSOR_MAX - 1))) {
			pr_err("[SSP]: %s - Mcu data frame error %d\n",
				__func__, iSensorData);
			goto exit;
		}

		data->get_sensor_data[iSensorData]((char *)buffer,
			&iDataIdx, &(fsb[iSensorData]));
	}

	fsb[PRESSURE_SENSOR].pressure[0] -= data->iPressureCal;

exit:
	ssp_dbg("[SSP]: %s Result\n"
		"[SSP]: accel %d,%d,%d\n"
		"[SSP]: gyro %d,%d,%d\n"
		"[SSP]: mag %d,%d,%d\n"
		"[SSP]: baro %d,%d\n", __func__,

		fsb[ACCELEROMETER_SENSOR].x, fsb[ACCELEROMETER_SENSOR].y,
		fsb[ACCELEROMETER_SENSOR].z, fsb[GYROSCOPE_SENSOR].x,
		fsb[GYROSCOPE_SENSOR].y, fsb[GYROSCOPE_SENSOR].z,
		fsb[GEOMAGNETIC_SENSOR].cal_x, fsb[GEOMAGNETIC_SENSOR].cal_y,
		fsb[GEOMAGNETIC_SENSOR].cal_z, fsb[PRESSURE_SENSOR].pressure[0],
		fsb[PRESSURE_SENSOR].pressure[1]);

	count = snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		fsb[ACCELEROMETER_SENSOR].x, fsb[ACCELEROMETER_SENSOR].y,
		fsb[ACCELEROMETER_SENSOR].z, fsb[GYROSCOPE_SENSOR].x,
		fsb[GYROSCOPE_SENSOR].y, fsb[GYROSCOPE_SENSOR].z,
		fsb[GEOMAGNETIC_SENSOR].cal_x, fsb[GEOMAGNETIC_SENSOR].cal_y,
		fsb[GEOMAGNETIC_SENSOR].cal_z, fsb[PRESSURE_SENSOR].pressure[0],
		fsb[PRESSURE_SENSOR].pressure[1]);

	kfree(fsb);

	return count;
}

ssize_t mcu_fota_rotate_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP] %s: %d\n", __func__, data->lpm_rotation_info);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->lpm_rotation_info);
}

unsigned short ssp_get_discharing_adc(void)
{
#if 0
	u16 adc = -1;
	int iRet = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	int buffer_length = 0;

	msg->cmd = MSG2SSP_AP_BATT_DISCHG_ADC;
	msg->length = 2;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &adc;
	msg->free_buffer = 0;
#if 0
	iRet = ssp_spi_sync(sensorhub_data, msg, 1000);
#else
	iRet = ssp_send_command(sensorhub_data, msg->cmd, msg->options, 1000,
		msg->buffer, msg->length, &(msg->buffer), &buffer_length, msg->data);
#endif
	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);
		iRet = ERROR;
	}
	pr_info("[SSP]%s, %d\n", __func__, adc);

	kfree(msg);
	return adc;
#else
    return 0;
#endif
}
EXPORT_SYMBOL_GPL(ssp_get_discharing_adc);

int ssp_charging_motion(struct ssp_data *data, int iEnable)
{
	u8 uBuf[2] = {0, 0};

	if (iEnable == 1) {
		pr_info("[SSP]%s, enable smart alert motion\n", __func__);
		send_instruction(data, ADD_LIBRARY,
			SMART_ALERT_MOTION, uBuf, 2);
	} else {
		pr_info("[SSP]%s, disable smart alert motion\n", __func__);
		send_instruction(data, REMOVE_LIBRARY,
			SMART_ALERT_MOTION, uBuf, 2);
	}

	return 0;
}

int ssp_charging_rotation(struct ssp_data *data, int iEnable)
{
	u8 uBuf[2] = {0, 0};

	if (iEnable == 1) {
		pr_info("[SSP]%s, enable LPM rotation\n", __func__);
		send_instruction(data, ADD_LIBRARY,
			LPM_AUTO_ROTATION, uBuf, 2);
	} else {
		pr_info("[SSP]%s, disable LPM rotation\n", __func__);
		send_instruction(data, REMOVE_LIBRARY,
			LPM_AUTO_ROTATION, uBuf, 2);
	}

	return 0;
}

int ssp_parse_motion(struct ssp_data *data, char *dataframe, int start, int end)
{
	int length = end - start;
	char *buf = dataframe + start;

	if (length > 5) {
		pr_err("[SSP]%s, invaild data length %d\n", __func__, length);
		return FAIL;
	}

	if ((buf[0] == 1) && (buf[1] == 1)) {
		data->lpm_int_mode = buf[2];
		if (buf[2] == SMART_ALERT_MOTION)
			pr_info("[SSP]: %s - LP MODE WAKEUP\n", __func__);
		else if (buf[2] == LPM_AUTO_ROTATION)
			data->lpm_rotation_info = buf[3];
		queue_work(data->lpm_motion_wq, &data->work_lpm_motion);
		//report_key_event(data);
		return SUCCESS;
	}

	return FAIL;
}

static void lpm_motion_work_func(struct work_struct *work)
{
	struct ssp_data *data =
		container_of(work, struct ssp_data, work_lpm_motion);

	if (data->lpm_int_mode == SMART_ALERT_MOTION) {
		input_event(data->motion_input_dev, EV_KEY, KEY_HOMEPAGE, 1);
		input_sync(data->motion_input_dev);
		ssp_charging_motion(data, 0);

		msleep(20);

		input_event(data->motion_input_dev, EV_KEY, KEY_HOMEPAGE, 0);
		input_sync(data->motion_input_dev);
		ssp_charging_motion(data, 1);
	} else if (data->lpm_int_mode == LPM_AUTO_ROTATION) {
		if ((data->lpm_rotation_info >= -1)
				&& (data->lpm_rotation_info <= 3)) {
			pr_info("[SSP]: %s - lpm_rotation_info : %d\n",
				__func__, data->lpm_rotation_info);
			input_event(data->motion_input_dev,
				EV_ABS, ABS_X, data->lpm_rotation_info);
			input_sync(data->motion_input_dev);
		}
	}

	wake_lock_timeout(&data->ssp_wake_lock, 1 * HZ);

}

int initialize_lpm_motion(struct ssp_data *data)
{
	data->lpm_motion_wq =
		create_singlethread_workqueue("ssp_lpm_motion_wq");
	if (!data->lpm_motion_wq)
		return ERROR;

	INIT_WORK(&data->work_lpm_motion, lpm_motion_work_func);
	return SUCCESS;
}
