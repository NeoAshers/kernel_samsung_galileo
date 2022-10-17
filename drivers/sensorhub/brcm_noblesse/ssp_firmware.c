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

#ifdef CONFIG_SENSORHUB_SPU
#include <linux/sec_sysup.h>
#include <linux/spu-verify.h>

struct spu_data {
	char spuFlag;
	size_t spu_patch_sz;
	unsigned char *spu_buffer;
	unsigned int version;
};
static struct spu_data spuData;
#endif

#undef SSP_FIRMWARE_REVISION_BCM

#define SSP_FIRMWARE_REVISION_BCM   20080400

#undef WOM_TARGET_REVISION_BCM

#define WOM_TARGET_REVISION_BCM		18050910

unsigned int get_module_rev2(void)
{
	return SSP_FIRMWARE_REVISION_BCM;
}

#ifdef CONFIG_SENSORHUB_SPU
static void spu_polling_func(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work, struct ssp_data, spu_work);

	pr_info("[SSP_SPU] : %s \n", __func__);
    open_spu();

	cancel_delayed_work_sync(&data->spu_work);
}

int initialize_spu_work(struct ssp_data *data)
{
	INIT_DELAYED_WORK(&data->spu_work, spu_polling_func);
	pr_info("[SSP_SPU] : %s finished\n", __func__);

	return SUCCESS;
}

static int check_spu_version(void)
{
	unsigned int version_spu = 0, version_kernel = 0;

	version_spu = sec_sysup_get_spu_firmware_version("SENSORHUB");
	if((signed int)version_spu < 0){
		pr_info("[SSP_SPU] %s file to open(%d)%d\n", __func__, (unsigned int)version_spu, -EINVAL);
		return -EINVAL;
	}
	else{
		version_kernel = get_module_rev2();
		pr_info("[SSP_SPU] %s spu version:%d kernel version:%d\n", __func__, version_spu, version_kernel);
		if(version_spu <= version_kernel)
			return -EINVAL;
	}
	spuData.version = version_spu;
	return 0;
}

static int download_image_from_spu(void)
{
	int ret;
	loff_t fsize = 0;
	unsigned char *buffer = NULL;
	mm_segment_t old_fs;
	struct file *filp = NULL;
	int offset;
	bool firm_valid_check;

	firm_valid_check = sec_sysup_check_spu_firmware_valid("SENSORHUB");
	if(firm_valid_check == false)
	{
		pr_info("[SSP_SPU] %s spu firmware not valid\n", __func__);
		return -EINVAL;
	}

	fsize = sec_sysup_get_firmware_size("SENSORHUB");
	if(fsize > 0x400000 || fsize < 0)
	{
		pr_err("[SSP_SPU] %s spu firmware size is too big or small\n", __func__);
		return -EINVAL;
	}
	spuData.spu_patch_sz = fsize;

	buffer = kzalloc(fsize + SEC_SYSUP_HEADER_SIZE + SPU_METADATA_SIZE(SENSORHUB), GFP_KERNEL);
	if(buffer == NULL)
	{
		pr_err("[SSP_SPU] %s, failed to alloc memory for spu\n", __func__);
		return -ENOMEM;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(SEC_SYSUP_SPU_PATH, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("[SSP_SPU] %s file open fail\n", __func__);
		set_fs(old_fs);
		if(buffer != NULL) kfree(buffer);
		return PTR_ERR(filp);
	}

	offset = sec_sysup_get_firmware_offset("SENSORHUB");
	pr_info("[SSP_SPU] %s version: size:%lld offset:%x\n", __func__, fsize, offset);
	vfs_llseek(filp, offset, SEEK_SET);

	ret = vfs_read(filp, buffer, fsize + SEC_SYSUP_HEADER_SIZE + SPU_METADATA_SIZE(SENSORHUB), &filp->f_pos);

	filp_close(filp, NULL);
	set_fs(old_fs);

	ret = spu_firmware_signature_verify("SENSORHUB", buffer, fsize + SEC_SYSUP_HEADER_SIZE + SPU_METADATA_SIZE(SENSORHUB));
	if (ret < 0){
		pr_err("[SSP_SPU] %s :spu_firmware_signature_verify failed with %d\n", __func__, ret);
		if(buffer != NULL) kfree(buffer);
		return -ret;
	}
	pr_info("[SSP_SPU] %s end buffer[0]:%d buffer[1]:%d\n", __func__, buffer[SEC_SYSUP_HEADER_SIZE], buffer[SEC_SYSUP_HEADER_SIZE+1]);

	spuData.spu_buffer = kzalloc(fsize, GFP_KERNEL);
	memcpy_toio(spuData.spu_buffer, buffer+SEC_SYSUP_HEADER_SIZE, (size_t)fsize);

	if(buffer != NULL) kfree(buffer);
	return 0;
}

void open_spu(void)
{
	if(check_spu_version() == 0){
        int ret;

		ret = download_image_from_spu();
		pr_info("[SSP_SPU]: %d %d\n", (int)spuData.spu_patch_sz, ret);
		if(ret == 0 && spuData.spu_buffer != NULL && spuData.spu_patch_sz != 0){
			spuData.spuFlag = true;
		}
		if(spuData.spu_buffer == NULL){
			pr_info("[SSP_SPU] null pointer\n");
		}
	}
}
void close_spu(void)
{
	if(spuData.spu_buffer != NULL) kfree(spuData.spu_buffer);
	spuData.spu_patch_sz = 0;
	//spuData.spuFlag = false;
}

bool get_spu_flag(void)
{
	return spuData.spuFlag;
}

unsigned char * get_spu_buffer(void)
{
	return spuData.spu_buffer;
}

size_t get_spu_size(void)
{
	return spuData.spu_patch_sz;
}

void check_spu_firmware_match(unsigned int uCurFirmRev)
{
	if(spuData.spuFlag){
		spuData.spuFlag = false;
		if(uCurFirmRev == spuData.version){
			pr_info("[SSP_SPU] version matched (%d : %d)!!\n", spuData.version, uCurFirmRev);
		}
		else {
			/* TODO : need to add recovery logic */
			pr_info("[SSP_SPU] version mis-matched (%d : %d)!!\n", spuData.version, uCurFirmRev);
		}
	}
}
#endif

unsigned int get_module_rev(struct ssp_data *data)
{
	if (data->bWOMode)
		return WOM_TARGET_REVISION_BCM;
	else
		return SSP_FIRMWARE_REVISION_BCM;
}

void toggle_mcu_reset(struct ssp_data *data)
{
	gpio_set_value_cansleep(data->rst, 0);

	usleep_range(1000, 1200);

	gpio_set_value_cansleep(data->rst, 1);

	msleep(50);

	if (!gpio_get_value(data->mcu_int2))
		pr_err("[SSP]: SH has entered bootloader in %s !!!!!",
			__func__);
}

#if SSP_STATUS_MONITOR
void toggle_mcu_hw_reset(struct ssp_data *data)
{
	gpio_set_value_cansleep(data->rst, 0);
	usleep_range(1000, 1200);
	pr_err("[SSP]:%s !!", __func__);

	gpio_set_value_cansleep(data->ap_int, 0); /* MCU request */

	if (data->reg_hub) {
		int ret = regulator_disable(data->reg_hub);
		if (ret)
			pr_err("[SSP] Failed to disable reg_hub regulator(%d)\n",
				ret);
		msleep(400);
		ret = regulator_enable(data->reg_hub);
		if (ret) {
			pr_err("[SSP] Failed to enable reg_hub regulator(%d)\n",
				ret);
		}
		usleep_range(1000, 1200);
	} else
		pr_err("[SSP] No VCC regulator even hw_reset is called SW reset excuted!");

	gpio_set_value_cansleep(data->rst, 1);

	if (data->reg_hub)
		msleep(1200);
	else
		msleep(50);

	gpio_set_value_cansleep(data->ap_int, 1); /*MCU request */

	if (!gpio_get_value(data->mcu_int2))
		pr_err("[SSP]: SH has entered bootloader in %s !!!!!",
			__func__);
}
#endif
