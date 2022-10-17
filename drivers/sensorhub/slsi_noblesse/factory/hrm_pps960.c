/*
 * Copyright (c)2013 Maxim Integrated Products, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include "../ssp.h"
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define VENDOR		"TI"
#define CHIP_ID		"PPS960"
#define MODULE_NAME	"AFE4920"

#define EOL_DATA_FILE_PATH "/csa/sensor/hrm_eol_data"

static ssize_t hrm_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t hrm_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

static ssize_t hrm_module_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MODULE_NAME);
}

static ssize_t hrm_led_g_ir_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
  int iRet = 0;
	char *buffer = NULL;
	int buffer_length = 0;

  int32_t ubuffer[3] = {0, };
  int32_t g_led=0, ir=0, r_led=0;
  struct ssp_data *data = dev_get_drvdata(dev);
  struct ssp_msg *msg;

  msg = kzalloc(sizeof(*msg), GFP_KERNEL);
  if (msg == NULL) {
    pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
      __func__);
    goto exit;
  }
  msg->cmd = HRM_G_IR_LED;
  msg->length = 12;
  msg->options = AP2HUB_READ;
  msg->buffer = (char *)&ubuffer;
  msg->free_buffer = 0;
#if 0
  iRet = ssp_ipc_sync(data, msg, 10000);
#else
			iRet = ssp_send_command(data, msg->cmd, msg->options, 10000,
															msg->buffer, msg->length, &buffer, &buffer_length, msg->data);
#endif

  if (iRet != SUCCESS) {
    pr_err("[SSP] %s - hrm green led Timeout!!\n", __func__);
    goto exit;
  }
	memcpy(&ubuffer, buffer, buffer_length);
  g_led = ubuffer[0];
  ir = ubuffer[1];
  r_led = ubuffer[2];

  if ((g_led>2050000)&&(ir>2050000)&&(r_led>2050000))
  {
    iRet = 1;
  }
  else
  {
    iRet = 0;
  }

  ssp_dbg("[SSP] : %s, %d, %d, %d Ret:%d\n", __func__, g_led, ir, r_led, iRet);


exit:
  return snprintf(buf, PAGE_SIZE, "%d\n", iRet);
}

static ssize_t hrm_led_b_r_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
  int iRet = 0;
	char *buffer = NULL;
	int buffer_length = 0;

  int32_t ubuffer[2] = {0, };
  int32_t b_led=0, r_led=0;
  struct ssp_data *data = dev_get_drvdata(dev);
  struct ssp_msg *msg;

  msg = kzalloc(sizeof(*msg), GFP_KERNEL);
  if (msg == NULL) {
    pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
      __func__);
    goto exit;
  }
  msg->cmd = HRM_B_R_LED;
  msg->length = 8;
  msg->options = AP2HUB_READ;
  msg->buffer = (char *)&ubuffer;
  msg->free_buffer = 0;
#if 0
  iRet = ssp_ipc_sync(data, msg, 10000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 10000,
													msg->buffer, msg->length, &buffer, &buffer_length, msg->data);
#endif

  if (iRet != SUCCESS) {
    pr_err("[SSP] %s - hrm blue & red led Timeout!!\n", __func__);
    goto exit;
  }

	memcpy(&ubuffer, buffer, buffer_length);
  b_led = ubuffer[0];
  r_led = ubuffer[1];

  if ((b_led>2050000)&&(r_led>2050000))
  {
    iRet = 1;
  }
  else
  {
    iRet = 0;
  }

  ssp_dbg("[SSP] : %s, %d, %d Ret:%d\n", __func__, b_led, r_led, iRet);


exit:
  return snprintf(buf, PAGE_SIZE, "%d\n", iRet);
}

static ssize_t hrm_noise_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *buffer = NULL;
	int buffer_length = 0;
  int iRet = 0;

  int32_t ubuffer[4] = {0, };
  int32_t pd1=0, pd2=0, pd3=0, pd4=0;
  struct ssp_data *data = dev_get_drvdata(dev);
  struct ssp_msg *msg;

  msg = kzalloc(sizeof(*msg), GFP_KERNEL);
  if (msg == NULL) {
    pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
      __func__);
    goto exit;
  }
  msg->cmd = HRM_NOISE;
  msg->length = 16;
  msg->options = AP2HUB_READ;
  msg->buffer = (char *)&ubuffer;
  msg->free_buffer = 0;
#if 0
  iRet = ssp_ipc_sync(data, msg, 10000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 10000,
													msg->buffer, msg->length, &buffer, &buffer_length, msg->data);
#endif

  if (iRet != SUCCESS) {
    pr_err("[SSP] %s - hrm noise Timeout!!\n", __func__);
    goto exit;
  }

	memcpy(&ubuffer, buffer, buffer_length);
  pd1 = ubuffer[0];
  pd2 = ubuffer[1];
  pd3 = ubuffer[2];
  pd4 = ubuffer[3];

  if ((pd1>2050000)&&(pd2>2050000)&&(pd3>2050000)&&(pd4>2050000))
  {
    iRet = 1;
  }
  else
  {
    iRet = 0;
  }

  ssp_dbg("[SSP] : %s, %d, %d, %d, %d Ret:%d\n", __func__, pd1, pd2, pd3, pd4, iRet);

exit:
  return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d\n", iRet, pd1, pd2, pd3, pd4);
}

static int hrm_open_eol_data(struct ssp_data *data)
{
	int iRet = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;
	u32 eol_data[HRM_EOL_DATA_SIZE];

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(EOL_DATA_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);
		pr_info("[SSP] %s : file open fail %d\n", __func__, iRet);
		memset(data->hrmcal, 0x00, sizeof(data->hrmcal));

		return iRet;
	}

	iRet = vfs_read(cal_filp, (char *)&eol_data, HRM_EOL_DATA_SIZE * sizeof(u32), &cal_filp->f_pos);

	if (iRet != HRM_EOL_DATA_SIZE  * sizeof(u32))
		iRet = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	pr_info("[SSP] %s: [%d %d %d %d %d %d %d %d]\n", __func__,
		eol_data[0], eol_data[1], eol_data[2], eol_data[3],
		eol_data[4], eol_data[5], eol_data[6], eol_data[7]);
	pr_info("[SSP] %s: [%d %d %d %d %d %d %d %d]\n", __func__,
		eol_data[8], eol_data[9], eol_data[10], eol_data[11],
		eol_data[12], eol_data[13], eol_data[14], eol_data[15]);
	pr_info("[SSP] %s: [%d %d %d %d %d %d %d %d]\n", __func__,
		eol_data[16], eol_data[17], eol_data[18], eol_data[19],
		eol_data[20], eol_data[21], eol_data[22], eol_data[23]);

	memcpy(data->hrmcal, eol_data, sizeof(eol_data));

	return iRet;
}

static int save_hrm_eol_data(struct ssp_data *data)
{
	int iRet = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(EOL_DATA_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("[SSP]: %s - Can't open osc_reg_value file\n", __func__);
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);
		return -EIO;
	}

	iRet = vfs_write(cal_filp, (char *)&data->buf[HRM_RAW_FAC_SENSOR], HRM_EOL_DATA_SIZE * sizeof(u32), &cal_filp->f_pos);

	if (iRet != HRM_EOL_DATA_SIZE * sizeof(u32)) {
		pr_err("[SSP]: %s - Can't write hrm osc_reg_value to file\n",
			__func__);
		iRet = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return iRet;
}


int pps960_set_hrm_calibration(struct ssp_data *data)
{
	int i, iRet = 0;
	struct ssp_msg *msg;

	if (!(data->uSensorState & (1 << HRM_RAW_SENSOR))) {
		for (i = 0; i < HRM_EOL_DATA_SIZE; i++)
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[i] = 0;

		pr_info("[SSP]: %s - Skip this function!!!"\
			", hrm sensor is not connected(0x%llx)\n",
			__func__, data->uSensorState);
		return iRet;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_SET_HRM_OSC_REG;
	msg->length = 4 * HRM_EOL_DATA_SIZE;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(4 * HRM_EOL_DATA_SIZE, GFP_KERNEL);

	msg->free_buffer = 1;
	memcpy(msg->buffer,
		&data->hrmcal[0], 4 * HRM_EOL_DATA_SIZE);

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

	pr_info("[SSP] %s: Set hrm cal data %d, %d, %d, %d, %d, %d, %d, %d",
		__func__, data->hrmcal[0], data->hrmcal[1], data->hrmcal[2],
		data->hrmcal[3], data->hrmcal[4], data->hrmcal[5],
		data->hrmcal[6], data->hrmcal[7]);
	pr_info("[SSP] %s: %d, %d, %d, %d, %d, %d, %d, %d\n", __func__,
		data->hrmcal[8], data->hrmcal[9], data->hrmcal[10],
		data->hrmcal[11], data->hrmcal[12], data->hrmcal[13],
		data->hrmcal[14], data->hrmcal[15]);
	pr_info("[SSP] %s: %d, %d, %d, %d, %d, %d, %d ,%d\n", __func__,
		data->hrmcal[16], data->hrmcal[17], data->hrmcal[18],
		data->hrmcal[19], data->hrmcal[20], data->hrmcal[21],
		data->hrmcal[22], data->hrmcal[23]);

	kfree(msg);
	return iRet;
}

int pps960_hrm_open_calibration(struct ssp_data *data)
{
	int iRet = 0;

	hrm_open_eol_data(data);

	return iRet;
}

static ssize_t hrm_lib_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *buffer = NULL;
	int buffer_length = 0;
	char ubuffer[10]  = {0, };
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		goto exit;
	}
	msg->cmd = HRM_LIB_VERSION_INFO;
	msg->length = HRM_LIB_VERSION_INFO_LENGTH;
	msg->options = AP2HUB_READ;
	msg->buffer = (char*)&ubuffer;
	msg->free_buffer = 0;
#if 0
	iRet = ssp_ipc_sync(data, msg, 3000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
													msg->buffer, msg->length, &buffer, &buffer_length, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s - hrm lib version Timeout!!\n", __func__);
		goto exit;
	}
	memcpy(&ubuffer, buffer, buffer_length);

	kfree(msg);
	return snprintf(buf, PAGE_SIZE, "%x %x %x %x %x %x %x %x %x %x\n",
			buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
			buffer[5], buffer[6], buffer[7], buffer[8], buffer[9]);
exit:
	return snprintf(buf, PAGE_SIZE, "%d\n", iRet);
}

static ssize_t hrm_eol_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&data->sysfs_op_mtx);
	memcpy(data->hrmcal, data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data,
		sizeof(data->hrmcal));

	save_hrm_eol_data(data);
	set_hrm_calibration(data);

	count = snprintf(buf, PAGE_SIZE,
		"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[0],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[1],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[2],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[3],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[4],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[5],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[6],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[7],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[8],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[9],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[10],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[11],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[12],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[13],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[14],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[15],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[16],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[17],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[18],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[19],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[20],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[21],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[22],
			data->buf[HRM_RAW_FAC_SENSOR].hrm_eol_data[23]);
	mutex_unlock(&data->sysfs_op_mtx);

	return count;
}

static ssize_t hrm_eol_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int iRet;
	int64_t dEnable;
	struct ssp_data *data = dev_get_drvdata(dev);

	iRet = kstrtoll(buf, 10, &dEnable);
	if (iRet < 0)
		return iRet;

	mutex_lock(&data->sysfs_op_mtx);
	if (dEnable)
		atomic_set(&data->eol_enable, 1);
	else
		atomic_set(&data->eol_enable, 0);
	mutex_unlock(&data->sysfs_op_mtx);

	return size;
}

static ssize_t hrm_raw_data_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		data->buf[HRM_RAW_SENSOR].hrm_raw_value1,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value2,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value3,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value4,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value5,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value6,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value7,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value8,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value9,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value10,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value11,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value12,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value13,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value14,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value15,
		data->buf[HRM_RAW_SENSOR].hrm_raw_value16);
}

static ssize_t hrm_lib_data_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
		data->buf[HRM_LIB_SENSOR].hr,
		data->buf[HRM_LIB_SENSOR].rri,
		data->buf[HRM_LIB_SENSOR].snr);
}

static ssize_t hrm_eol_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int iRet = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;
	u32 eol_data[HRM_EOL_DATA_SIZE];

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(EOL_DATA_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);

		memset(eol_data, 0, sizeof(eol_data));
		goto exit;
	}

	iRet = vfs_read(cal_filp, (char *)&eol_data, HRM_EOL_DATA_SIZE * sizeof(u32), &cal_filp->f_pos);

	if (iRet != HRM_EOL_DATA_SIZE * sizeof(u32))
		iRet = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

exit:
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d "\
		"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
		eol_data[0], eol_data[1], eol_data[2], eol_data[3],
		eol_data[4], eol_data[5], eol_data[6], eol_data[7],
		eol_data[8], eol_data[9], eol_data[10], eol_data[11],
		eol_data[12], eol_data[13], eol_data[14], eol_data[15],
		eol_data[16], eol_data[17], eol_data[18], eol_data[19],
		eol_data[20], eol_data[21], eol_data[22], eol_data[23]);
}

static ssize_t hrm_ir_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *buffer = NULL;
	int buffer_length = 0;
	int32_t ubuffer	= 0;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		goto exit;
	}
	msg->cmd = HRM_IR_LEVEL_THRESHOLD;
	msg->length = HRM_IR_LEVEL_THRESHOLD_LENGTH;
	msg->options = AP2HUB_READ;
	msg->buffer = (char*)&ubuffer;
	msg->free_buffer = 0;
#if 0
	iRet = ssp_ipc_sync(data, msg, 3000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
													msg->buffer, msg->length, &buffer, &buffer_length, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s - hrm ir threshold Timeout!!\n", __func__);
		goto exit;
	}
	memcpy(&ubuffer, buffer, buffer_length);
	pr_info("[SSP] %s - %d\n", __func__, ubuffer);

	kfree(msg);
	return snprintf(buf, PAGE_SIZE, "%d\n", ubuffer);
exit:
	return snprintf(buf, PAGE_SIZE, "%d\n", iRet);
}

static ssize_t hrm_dqa_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *rbuf = NULL;
	int rbuf_length = 0;
	int buffer[HRM_DQA_LENGTH]  = {0, };
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		goto exit;
	}
	msg->cmd = HRM_DQA;
	msg->length = HRM_DQA_LENGTH * sizeof(int);
	msg->options = AP2HUB_READ;
	msg->buffer = (char*)&buffer;
	msg->free_buffer = 0;
#if 0
	iRet = ssp_ipc_sync(data, msg, 3000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
		msg->buffer, msg->length, &rbuf, &rbuf_length, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s - hrm dqa Timeout!!\n", __func__);
		goto exit;
	}
	memcpy(&buffer, rbuf, rbuf_length);
	kfree(msg);
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
			buffer[5], buffer[6], buffer[7], buffer[8], buffer[9],
			buffer[10], buffer[11], buffer[12], buffer[13], buffer[14],
			buffer[15], buffer[16], buffer[17], buffer[18], buffer[19], buffer[20]);
exit:
	return snprintf(buf, PAGE_SIZE, "%d\n", iRet);
}

static ssize_t hrm_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *rbuf = NULL;
	int rbuf_length = 0;
	int iRet = 1;

	int32_t buffer[6]	= {0, };
	struct ssp_data *data = dev_get_drvdata(dev);
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
			__func__);
		goto exit;
	}
	msg->cmd = HRM_FACTORY;
	msg->length = 24;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *)&buffer;
	msg->free_buffer = 0;
#if 0
	iRet = ssp_ipc_sync(data, msg, 3000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
													msg->buffer, msg->length, &rbuf, &rbuf_length, msg->data);
#endif

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s - hrm ir threshold Timeout!!\n", __func__);
		goto exit;
	}
	memcpy(&buffer, rbuf, rbuf_length);
	pr_info("[SSP] %s- gain=0x%x ambgain=0x%x ir=%d red=%d green=%d amb=%d\n",
		__func__, buffer[0], buffer[1], buffer[2],
		buffer[3], buffer[4], buffer[5]);

	if ((buffer[2] > 0 && buffer[2] < 4194304)
		&& (buffer[3] > 0 && buffer[3] < 4194304)
		&& (buffer[4] > 0 && buffer[4] < 4194304)
		&& (buffer[5] > 0 && buffer[5] < 4194304)) {
		if (buffer[2] == 0 && buffer[3] == 0 &&
			buffer[4] == 0 && buffer[5] == 0)
			iRet = 0;
		else
			iRet = 1;
	} else
		iRet = 0;

exit:
	return snprintf(buf, PAGE_SIZE, "%d\n", iRet);
}

static ssize_t ecg_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&data->sysfs_op_mtx);
	memcpy(data->hrm_ecg_fac, data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data,
		sizeof(data->hrm_ecg_fac));

	count = snprintf(buf, PAGE_SIZE,
		"%d %d %d %d %d %d %d %d %d %d\n",
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[0],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[1],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[2],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[3],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[4],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[5],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[6],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[7],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[8],
			data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[9]);
	mutex_unlock(&data->sysfs_op_mtx);

	return count;

}

static ssize_t ecg_raw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	ssize_t count;

	mutex_lock(&data->sysfs_op_mtx);
	memcpy(data->hrm_ecg_lib, data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data,
		sizeof(data->hrm_ecg_lib));

	count = snprintf(buf, PAGE_SIZE,
		"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[0],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[1],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[2],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[3],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[4],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[5],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[6],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[7],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[8],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[9],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[10],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[11],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[12],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[13],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[14],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[15],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[16],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[17],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[18],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[19],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[20],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[21],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[22],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[23],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[24],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[25],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[26],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[27],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[28],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[29],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[30],
			data->buf[HRM_ECG_LIB_SENSOR].hrm_ecg_lib_data[31]);

	mutex_unlock(&data->sysfs_op_mtx);

	return count;

}

static ssize_t ecg_lead_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	ssize_t count;
	//s32 lead_off;
	//s32 ecg_det;
	int iRet = 0;
	char chTempBuf = 0;
	struct ssp_msg *msg = NULL;
	int read_buf[2] = {0,};
	int buffer_length = 0;
	char* read_buf_char = kzalloc(8, GFP_KERNEL);

	//Init variable
	read_buf[0]  = 15;
	read_buf[1]  = 1;
	//ecg_det = 1;

	//lead_off = data->buf[HRM_ECG_FAC_SENSOR].hrm_ecg_fac_data[0];
	//data->ecg_con_value = gpio_get_value(data->ecg_con_det);
	//ecg_det = data->ecg_con_value;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		kfree(read_buf_char);
		kfree(msg);
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}

	msg->cmd = HRM_ECG_DATA;
	msg->length = 8;  /*4byte: ecg_lead_off, 4byte: ecg_det_pin*/
	msg->options = AP2HUB_READ;
	msg->data = 0;
	msg->buffer = &chTempBuf;
	msg->free_buffer = 0;

#if 0
	iRet = ssp_ipc_sync(data, msg, 3000);
#else
	iRet = ssp_send_command(data, msg->cmd, msg->options, 3000,
												 msg->buffer, msg->length, &read_buf_char, &buffer_length, msg->data);
#endif

	if (iRet != SUCCESS) {
	  pr_err("[SSP] %s - ecg_lead_off Timeout!!\n", __func__);
	  kfree(read_buf_char);
	  kfree(msg);
	  return 0;
	}

	memcpy((char*)read_buf, read_buf_char, 8);

	//pr_info("[SSP] %s, ecg lead off: %d %d det_pin %d ret:%d\n", __func__, read_buf[0], read_buf[1], ecg_det, iRet);
	pr_info("[SSP] %s, ecg lead off: %d %d ret:%d\n", __func__, read_buf[0], read_buf[1], iRet);

//	if(data->ecg_con_det == 0)
//	{
		/*Not Rev0.2 & 0.3*/
		count = snprintf(buf, PAGE_SIZE, "%d %d\n", read_buf[0], read_buf[1]);
//	}
//	else
//	{
//		/*Rev0.2 & 0.3*/
//		count = snprintf(buf, PAGE_SIZE, "%d %d\n", read_buf[0], ecg_det);
//	}

	kfree(read_buf_char);
	kfree(msg);
	return count;
}

#if 0
static ssize_t ecg_raw_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int iRet = 0;
	char chTempBuf = 0;
	char *buffer = NULL;
	int buffer_length = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct ssp_msg *msg;
	int64_t commnad;

	if (kstrtoll(buf, 10, &commnad) < 0)
		return -EINVAL;
	chTempBuf = (char)commnad;

	//if (sysfs_streq(buf, "1"))
	//	ssp_dbg("[SSP]: %s - on\n", __func__);
	//else if (sysfs_streq(buf, "0"))
	//	ssp_dbg("[SSP]: %s - off\n", __func__);
	//else if (sysfs_streq(buf, "2")) {
		ssp_dbg("[SSP]: %s - Command %d\n", __func__, chTempBuf);

		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		if (msg == NULL) {
			pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
			return -ENOMEM;
		}

		msg->cmd = HRM_ECG_DATA;
		msg->length = 1;
		msg->options = AP2HUB_WRITE;
		msg->data = chTempBuf;
		msg->buffer = &chTempBuf;
		msg->free_buffer = 0;

#if 0
		iRet = ssp_ipc_sync(data, msg, 3000);
#else
		iRet = ssp_send_command(data, msg->cmd, msg->options, 0,
													 msg->buffer, msg->length, &buffer, &buffer_length, msg->data);
#endif

		if (iRet != SUCCESS) {
			pr_err("[SSP]: %s - Timeout!!\n", __func__);
		}
		else
		{
			ssp_dbg("[SSP]: %s factory test success!\n", __func__);
		}
//	} else {
//		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
//		return -EINVAL;
//	}

	return size;
}
#endif

static DEVICE_ATTR(name, S_IRUGO, hrm_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, hrm_vendor_show, NULL);
static DEVICE_ATTR(hrm_lib_ver, S_IRUGO, hrm_lib_version_show, NULL);
static DEVICE_ATTR(hrm_eol, S_IRUGO | S_IWUSR | S_IWGRP, hrm_eol_show, hrm_eol_store);
static DEVICE_ATTR(hrm_raw, S_IRUGO, hrm_raw_data_read, NULL);
static DEVICE_ATTR(hrm_lib, S_IRUGO, hrm_lib_data_read, NULL);
static DEVICE_ATTR(hrm_eol_data, S_IRUGO, hrm_eol_data_show, NULL);
static DEVICE_ATTR(hrm_ir_threshold, S_IRUGO, hrm_ir_threshold_show, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, hrm_selftest_show, NULL);
static DEVICE_ATTR(module, S_IRUGO, hrm_module_show, NULL);
static DEVICE_ATTR(hrm_led_g_ir, S_IRUGO, hrm_led_g_ir_show, NULL);
static DEVICE_ATTR(hrm_led_b_r, S_IRUGO, hrm_led_b_r_show, NULL);
static DEVICE_ATTR(hrm_noise, S_IRUGO, hrm_noise_show, NULL);
static DEVICE_ATTR(hrm_dqa, S_IRUGO, hrm_dqa_show, NULL);
static DEVICE_ATTR(ecg_selftest, S_IRUGO, ecg_selftest_show, NULL);
static DEVICE_ATTR(ecg_raw_data, S_IRUGO, ecg_raw_show, NULL);
static DEVICE_ATTR(ecg_lead_off, S_IRUGO, ecg_lead_show, NULL);

static struct device_attribute *hrm_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_hrm_lib_ver,
	&dev_attr_hrm_eol,
	&dev_attr_hrm_raw,
	&dev_attr_hrm_lib,
	&dev_attr_hrm_eol_data,
	&dev_attr_hrm_ir_threshold,
	&dev_attr_selftest,
	&dev_attr_module,
	&dev_attr_hrm_led_g_ir,
	&dev_attr_hrm_led_b_r,
	&dev_attr_hrm_noise,
	&dev_attr_hrm_dqa,
	&dev_attr_ecg_selftest,
	&dev_attr_ecg_raw_data,
	&dev_attr_ecg_lead_off,
	NULL,
};

void initialize_pps960_hrm_factorytest(struct ssp_data *data)
{
	sensors_register(data->hrm_device, data, hrm_attrs,
		"hrm_sensor");
}

void remove_pps960_hrm_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->hrm_device, hrm_attrs);
}
