/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
//#include <linux/iio/buffer_impl.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>
#include <linux/slab.h>

#include "ssp.h"
#include "ssp_iio.h"
//#include "ssp_cmd_define.h"
//#include "ssp_data.h"
#include "ssp_iio.h"
#ifdef USE_SCONTEXT
#include "ssp_scontext.h"
#endif

#define IIO_CHANNEL             -1
#define IIO_SCAN_INDEX          3
#define IIO_SIGN                's'
#define IIO_UNSIGN              'u'
#define IIO_SHIFT               0

#define META_EVENT              0
#define META_TIMESTAMP          0

#define PROX_AVG_READ_NUM       80
enum
{
	PROX_RAW_NUM = 0,
	PROX_RAW_MIN,
	PROX_RAW_SUM,
	PROX_RAW_MAX,
	PROX_RAW_DATA_SIZE,
};
#ifdef USE_SCONTEXT
#define SCONTEXT_DATA_LEN       56
#define SCONTEXT_HEADER_LEN     8
#define SCONTEXT_LENGTH_LEN		4
#define SCONTEXT_START_LEN		2
#define SCONTEXT_END_LEN		2
#endif
#define RESET_REASON_KERNEL_RESET            0x01
#define RESET_REASON_MCU_CRASHED             0x02
#define RESET_REASON_SYSFS_REQUEST           0x03

static int ssp_preenable(struct iio_dev *indio_dev)
{
	return 0;
}

static int ssp_predisable(struct iio_dev *indio_dev)
{
	return 0;
}

static const struct iio_buffer_setup_ops ssp_iio_ring_setup_ops = {
	.preenable = &ssp_preenable,
	.predisable = &ssp_predisable,
};

static int ssp_iio_configure_ring(struct iio_dev *indio_dev)
{
	struct iio_buffer *ring;

	ring = iio_kfifo_allocate();
	if (!ring) {
		return -ENOMEM;
	}

	ring->scan_timestamp = true;
	ring->bytes_per_datum = 8;
	indio_dev->buffer = ring;
	indio_dev->setup_ops = &ssp_iio_ring_setup_ops;
	indio_dev->modes |= INDIO_BUFFER_SOFTWARE;

	return 0;
}

static void ssp_iio_push_buffers(struct iio_dev *indio_dev, u64 timestamp,
                                 char *data, int data_len)
{
	char buf[data_len + sizeof(timestamp)];

	if (!indio_dev || !data) {
		return;
	}

	memcpy(buf, data, data_len);
	memcpy(buf + data_len, &timestamp, sizeof(timestamp));
	mutex_lock(&indio_dev->mlock);
	iio_push_to_buffers(indio_dev, buf);
	mutex_unlock(&indio_dev->mlock);
}

#ifdef CONFIG_SENSORS_SSP_PROXIMITY
static void report_prox_raw_data(struct ssp_data *data, int type,
                                 struct sensor_value *proxrawdata)
{
	if (data->prox_raw_avg[PROX_RAW_NUM]++ >= PROX_AVG_READ_NUM) {
		data->prox_raw_avg[PROX_RAW_SUM] /= PROX_AVG_READ_NUM;
		data->buf[type].prox_raw[1] = (u16)data->prox_raw_avg[1];
		data->buf[type].prox_raw[2] = (u16)data->prox_raw_avg[2];
		data->buf[type].prox_raw[3] = (u16)data->prox_raw_avg[3];

		data->prox_raw_avg[PROX_RAW_NUM] = 0;
		data->prox_raw_avg[PROX_RAW_MIN] = 0;
		data->prox_raw_avg[PROX_RAW_SUM] = 0;
		data->prox_raw_avg[PROX_RAW_MAX] = 0;
	} else {
		data->prox_raw_avg[PROX_RAW_SUM] += proxrawdata->prox_raw[0];

		if (data->prox_raw_avg[PROX_RAW_NUM] == 1) {
			data->prox_raw_avg[PROX_RAW_MIN] = proxrawdata->prox_raw[0];
		} else if (proxrawdata->prox_raw[0] < data->prox_raw_avg[PROX_RAW_MIN]) {
			data->prox_raw_avg[PROX_RAW_MIN] = proxrawdata->prox_raw[0];
		}

		if (proxrawdata->prox_raw[0] > data->prox_raw_avg[PROX_RAW_MAX]) {
			data->prox_raw_avg[PROX_RAW_MAX] = proxrawdata->prox_raw[0];
		}
	}

	data->buf[type].prox_raw[0] = proxrawdata->prox_raw[0];
}

static void report_prox_cal_data(struct ssp_data *data, int type,
                                 struct sensor_value *p_cal_data)
{
	data->prox_thresh[0] = p_cal_data->prox_cal[0];
	data->prox_thresh[1] = p_cal_data->prox_cal[1];
	ssp_info("prox thresh %u %u", data->prox_thresh[0], data->prox_thresh[1]);

	proximity_calibration_off(data);
}
#endif

void report_sensor_data(struct ssp_data *data, int type,
                        struct sensor_value *event)
{
	int i;
	static int log_count = 0;

	for(i=0;i<HRM_ECG_LIB_DATA_SIZE;i++)
	{
		data->buf[HRM_ECG_LIB_SENSOR].ecg_data_value[i] = event->ecg_data_value[i];
	}

	ssp_iio_push_buffers(data->indio_devs[type], event->timestamp,
	                     (char *)&data->buf[HRM_ECG_LIB_SENSOR], data->info[type].report_data_len);

#if 1//def SSP_DEBUG_LOG
	log_count++;
	if(log_count > 300)
		log_count = 0;
	else
		return;

	pr_info("[SSP]ecg_lib_data, %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %lld %lld %lld %lld\n",
		event->ecg_data_value[0],
		event->ecg_data_value[1],
		event->ecg_data_value[2],
		event->ecg_data_value[3],
		event->ecg_data_value[4],
		event->ecg_data_value[5],
		event->ecg_data_value[6],
		event->ecg_data_value[7],
		event->ecg_data_value[8],
		event->ecg_data_value[9],
		event->ecg_data_value[10],
		event->ecg_data_value[11],
		event->ecg_data_value[12],
		event->ecg_data_value[13],
		event->ecg_data_value[14],
		event->ecg_data_value[15],
		event->ecg_data_value[16],
		event->ecg_data_value[17],
		event->ecg_data_value[18],
		event->ecg_data_value[19],
		event->ecg_data_value[20],
		event->ecg_data_value[21],
		event->ecg_data_value[22],
		event->ecg_data_value[23],
		event->ecg_data_value[24],
		event->ecg_data_value[25],
		event->ecg_data_value[26],
		event->ecg_data_value[27],
		event->ecg_data_value[28],
		event->ecg_data_value[29],
		event->ecg_data_value[30],
		event->ecg_data_value[31],
		((event->timestamp & 0xffff000000000000) >> 48),
		((event->timestamp & 0x0000ffff00000000) >> 32),
		((event->timestamp & 0x00000000ffff0000) >> 16),
		(event->timestamp & 0x000000000000ffff));
#endif
}

#ifdef USE_SCONTEXT
void report_scontext_data(struct ssp_data *data, char *data_buf, u32 length)
{
	char buf[SCONTEXT_HEADER_LEN + SCONTEXT_DATA_LEN] = {0, };
	u16 start, end;
	u64 timestamp;

	ssp_scontext_log(__func__, data_buf, length);

	start = 0;
	memcpy(buf, &length, SCONTEXT_LENGTH_LEN);
	timestamp = get_current_timestamp();

	while (start < length) {
		if (start + SCONTEXT_DATA_LEN < length) {
			end = start + SCONTEXT_DATA_LEN - 1;
		} else {
			memset(buf + SCONTEXT_HEADER_LEN, 0, SCONTEXT_DATA_LEN);
			end = length - 1;
		}

		memcpy(buf + SCONTEXT_LENGTH_LEN, &start, SCONTEXT_START_LEN);
		memcpy(buf + SCONTEXT_LENGTH_LEN + SCONTEXT_START_LEN, &end, SCONTEXT_END_LEN);
		memcpy(buf + SCONTEXT_HEADER_LEN, data_buf + start, end - start + 1);

/*
        ssp_infof("[%d, %d, %d] 0x%x 0x%x 0x%x 0x%x// 0x%x 0x%x// 0x%x 0x%x",
                length, start, end,
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);


        ssp_infof("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);


        ssp_infof("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x, //0x%llx",
                buf[16], buf[17], buf[18], buf[19], buf[20], buf[21], buf[22], buf[23], timestamp);
*/
		ssp_iio_push_buffers(data->indio_devs[SENSOR_TYPE_SCONTEXT], timestamp,
		                     buf, data->info[SENSOR_TYPE_SCONTEXT].report_data_len);

		start = end + 1;
	}
}

void report_scontext_notice_data(struct ssp_data *data, char notice)
{
	char notice_buf[4] = {0x02, 0x01, 0x00, 0x00};
	int len = 3;

	notice_buf[2] = notice;
	if (notice == SCONTEXT_AP_STATUS_RESET) {
		len = 4;
		if (data->is_reset_from_sysfs == true) {
			notice_buf[3] = RESET_REASON_SYSFS_REQUEST;
			data->is_reset_from_sysfs = false;
		} else if (data->is_reset_from_kernel == true) {
			notice_buf[3] = RESET_REASON_KERNEL_RESET;
			data->is_reset_from_kernel = false;
		} else {
			notice_buf[3] = RESET_REASON_MCU_CRASHED;
		}
	}

	report_scontext_data(data, notice_buf, len);

	if (notice == SCONTEXT_AP_STATUS_WAKEUP) {
		ssp_infof("wake up");
	} else if (notice == SCONTEXT_AP_STATUS_SLEEP) {
		ssp_infof("sleep");
	} else if (notice == SCONTEXT_AP_STATUS_RESET) {
		ssp_infof("reset");
	} else {
		ssp_errf("invalid notice(0x%x)", notice);
	}
}
#endif
static void *init_indio_device(struct device *dev, struct ssp_data *data,
                               const struct iio_info *info,
                               const struct iio_chan_spec *channels,
                               const char *device_name)
{
	struct iio_dev *indio_dev;
	int ret = 0;

	indio_dev = iio_device_alloc(0);
	if (!indio_dev) {
		goto err_alloc;
	}

	indio_dev->name = device_name;
	indio_dev->dev.parent = dev;
	indio_dev->info = info;
	indio_dev->channels = channels;
	indio_dev->num_channels = 1;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	ret = ssp_iio_configure_ring(indio_dev);
	if (ret) {
		goto err_config_ring;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		goto err_register_device;
	}

	return indio_dev;

err_register_device:
	ssp_err("fail to register %s device", device_name);
	iio_kfifo_free(indio_dev->buffer);
err_config_ring:
	ssp_err("failed to configure %s buffer\n", indio_dev->name);
	iio_device_unregister(indio_dev);
err_alloc:
	ssp_err("fail to allocate memory for iio %s device", device_name);
	return NULL;
}

static const struct iio_info indio_info = {
	.driver_module = THIS_MODULE,
};

#define ECG_IIO_BIT        64
#define ECG_IIO_REPEAT_CNT 17
int initialize_indio_dev(struct device *dev, struct ssp_data *data)
{
	int timestamp_len = 0;
	int type;
	int realbits_size = 0;
	int repeat_size = 0;

	for (type = 0; type < SENSOR_TYPE_ECG_IIO; type++) {
		if (!data->info[type].enable || (data->info[type].report_data_len == 0)) {
			continue;
		}

		timestamp_len = sizeof(data->buf[type].timestamp);

		realbits_size = (data->info[type].report_data_len+timestamp_len) * BITS_PER_BYTE;
		repeat_size = 1;

		while ((realbits_size / repeat_size > 255) && (realbits_size % repeat_size == 0))
			repeat_size++;

		realbits_size /= repeat_size;

		data->indio_channels[type].type = IIO_TIMESTAMP;
		data->indio_channels[type].channel = IIO_CHANNEL;
		data->indio_channels[type].scan_index = IIO_SCAN_INDEX;
		data->indio_channels[type].scan_type.sign = IIO_UNSIGN;
		data->indio_channels[type].scan_type.realbits = ECG_IIO_BIT;
		data->indio_channels[type].scan_type.storagebits = ECG_IIO_BIT;
		data->indio_channels[type].scan_type.shift = IIO_SHIFT;
		data->indio_channels[type].scan_type.repeat = ECG_IIO_REPEAT_CNT;

		data->indio_devs[type]
		        = (struct iio_dev *)init_indio_device(dev, data,
		                                              &indio_info, &data->indio_channels[type],
		                                              data->info[type].name);

		if (!data->indio_devs[type]) {
			ssp_err("fail to init %s iio dev", data->info[type].name);
			remove_indio_dev(data);
			return ERROR;
		}
	}

	return SUCCESS;
}

void remove_indio_dev(struct ssp_data *data)
{
	int type;

	for (type = SENSOR_TYPE_ECG_IIO - 1; type >= 0; type--) {
		if (data->indio_devs[type]) {
			iio_device_unregister(data->indio_devs[type]);
		}
	}
}

