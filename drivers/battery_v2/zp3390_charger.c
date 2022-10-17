/*
 *  zp3390_charger.c
 *  Samsung zp3390 Charger Driver
 *
 *  Copyright (C) 2015 Samsung Electronics
 * Yeongmi Ha <yeongmi86.ha@samsung.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/errno.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/firmware.h>
#include <linux/muic/muic.h>
#include <linux/muic/muic_notifier.h>
#include <linux/alarmtimer.h>
#include <linux/wakelock.h>
#include <../drivers/battery_v2/include/sec_charging_common.h>
#include <../drivers/battery_v2/include/charger/zp3390_charger.h>
#include <../drivers/battery_v2/include/sec_battery.h>

/* Vout stabilization time is about 1.5sec after Vrect ocured. */
#define VOUT_STABLILZATION_TIME		1500

#define WPC_AUTH_DELAY_TIME			25
#define UNKNOWN_TX_ID				0xFF

#define ENABLE 1
#define DISABLE 0
#define CMD_CNT 3

#define ZP3390_I_OUT_OFFSET		12

#if defined(CONFIG_WIRELESS_FIRMWARE_UPDATE)
#define ZP3390_FW_FILENAME	"zp3390.mng.bin"
#endif

//bool sleep_mode = false;

static enum power_supply_property sec_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
#if defined(CONFIG_WIRELESS_FIRMWARE_UPDATE)
	POWER_SUPPLY_PROP_MANUFACTURER,
#endif
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL,
	POWER_SUPPLY_PROP_CHARGE_POWERED_OTG_CONTROL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_ENERGY_AVG,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_TEMP,
};

#if defined(CONFIG_WIRELESS_FIRMWARE_UPDATE)
static int zp3390_otp_update = 0;
static u8 adc_cal = 0;
#endif

unsigned int batt_booting_chk = 1;
struct device *wpc_device;

extern unsigned int lpcharge;

static int
zp3390_wpc_auth_delayed_work(struct zp3390_charger_data *charger, int sec);
static int zp3390_ap_battery_monitor(struct zp3390_charger_data *charger, int mode);
static int zp3390_full_charge(struct zp3390_charger_data *charger);
static void zp3390_wpc_request_tx_id(struct zp3390_charger_data *charger, int cnt);

static int zp3390_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	struct zp3390_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	struct i2c_msg msg[2];
	u8 wbuf[2];
	u8 rbuf[2];

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = wbuf;

	wbuf[0] = (reg & 0xFF00) >> 8;
	wbuf[1] = (reg & 0xFF);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = rbuf;

	mutex_lock(&charger->io_lock);
	ret = i2c_transfer(client->adapter, msg, 2);
	mutex_unlock(&charger->io_lock);
	if (ret < 0)
	{
		dev_err(&client->dev, "%s read err reg(0x%x) ret(%d)\n",
			__func__, reg, ret);
		return -1;
	}
	*val = rbuf[0];

#if defined(CONFIG_WIRELESS_FIRMWARE_UPDATE)
	if(!zp3390_otp_update)
#endif
	{
		pr_debug("%s reg = 0x%x, data = 0x%x\n", __func__, reg, *val);
	}

	return ret;
}

static int zp3390_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	struct zp3390_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	mutex_lock(&charger->io_lock);
	ret = i2c_master_send(client, data, 3);
	mutex_unlock(&charger->io_lock);
	if (ret < 3) {
		dev_err(&client->dev, "%s write err reg(0x%x) ret(%d)\n",
			__func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	pr_debug("%s reg = 0x%x, data = 0x%x \n", __func__, reg, val);

	return 0;
}

static s32 write_data(struct i2c_client *client, u8 *data, u16 length)
{
	struct zp3390_charger_data *charger = i2c_get_clientdata(client);
	s32 ret;
	int count = 0;
	mutex_lock(&charger->io_lock);
retry:
	ret = i2c_master_send(client , data , length);
	if (ret < 0) {
		usleep_range(1 * 1000, 1 * 1000);

		if (++count < 8)
			goto retry;

		mutex_unlock(&charger->io_lock);
		return ret;
	}

	usleep_range(ZP3390_DELAY_FOR_POST_TRANSCATION, ZP3390_DELAY_FOR_POST_TRANSCATION);
	mutex_unlock(&charger->io_lock);
	return length;
}

static int zp3390_write_reg_wdata(struct i2c_client *client, u16 reg, u16 val)
{
	int ret;
	u8 pkt[4];

	pkt[0] = (reg >> 8)&0xff;
	pkt[1] = (reg) & 0xff;
	pkt[2] = (val >> 8)&0xff;
	pkt[3] = (val) & 0xff;

	ret = write_data(client, pkt, 4);
	if(ret < 0)
	{
		dev_err(&client->dev, "%s: i2c write error, reg: 0x%x, ret: %d\n", __func__, reg, ret);
		return I2C_FAIL;
	}
	pr_info("%s reg = 0x%x, data = 0x%x \n", __func__, reg, val);
	return I2C_SUCCESS;
}

static int zp3390_write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret;
	int count = 0;
	unsigned char data[2] = { reg >> 8, reg & 0xff};

	ret = write_data(client, data, 2);
	return I2C_SUCCESS;
}

static inline s32 write_firmware_data(struct i2c_client *client,
	u16 addr, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
	u8 pkt[66]; /* max packet */

	pkt[0] = (addr) & 0xff; /* reg addr */
	pkt[1] = (addr >> 8)&0xff;
	memcpy((u8 *)&pkt[2], values, length);

	ret = write_data(client, pkt, length + 2);
	return ret;
}
static inline s32 read_firmware_data(struct i2c_client *client,
	u16 addr, u8 *values, u16 length)
{
	struct zp3390_charger_data *charger = i2c_get_clientdata(client);
	s32 ret;

	mutex_lock(&charger->io_lock);
	/* select register*/
	ret = i2c_master_send(client , (u8 *)&addr , 2);
	if (ret < 0)
	{
		mutex_unlock(&charger->io_lock);
		return ret;
	}

	/* for setup tx transaction. */
	usleep_range(1 * 1000, 1 * 1000);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
	{
		mutex_unlock(&charger->io_lock);
		return ret;
	}

	usleep_range(ZP3390_DELAY_FOR_POST_TRANSCATION, ZP3390_DELAY_FOR_POST_TRANSCATION);
	mutex_unlock(&charger->io_lock);
	return length;
}

static int zp3390_reg_update(struct i2c_client *client, u16 reg, u8 val, u8 mask)
{
	struct zp3390_charger_data *charger = i2c_get_clientdata(client);
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };
	u8 data2;
	int ret;

	ret = zp3390_reg_read(client, reg, &data2);
	if (ret >= 0) {
		u8 old_val = data2 & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		data[2] = new_val;

		mutex_lock(&charger->io_lock);
		ret = i2c_master_send(client, data, 3);
		mutex_unlock(&charger->io_lock);
		if (ret < 3) {
			dev_err(&client->dev, "%s: i2c write error, reg: 0x%x, ret: %d\n",
				__func__, reg, ret);
			return ret < 0 ? ret : -EIO;
		}
	}
	zp3390_reg_read(client, reg, &data2);

	pr_debug("%s reg = 0x%x, data = 0x%x, mask = 0x%x\n", __func__, reg, val, mask);

	return ret;
}

enum power_control {
	ZP3390_POWER_OFF,
	ZP3390_POWER_ON,
	ZP3390_POWER_ON_SEQUENCE,
};
static bool zp3390_power_control(struct zp3390_charger_data *charger, u8 ctl)
{
/*
	struct i2c_client *client = charger->client;

	pr_info("[charger] %s, %d\n", __func__, ctl);
	if (ctl == POWER_OFF) {
		msleep(50);
	}
	else if (ctl == POWER_ON) {
		msleep(50);
	}
*/
	return true;
}

#if defined(CONFIG_WIRELESS_FIRMWARE_UPDATE)
#define MAX_FW_PATH 255
#define FW_UPDATE_RETRY_CNT 3
#define TC_SECTOR_SZ		8
#define TC_SECTOR_SZ_WRITE		64
#define TC_SECTOR_SZ_READ		8
int zp3390_firmware_update(struct zp3390_charger_data *charger)
{
	struct i2c_client *client = charger->client;
	u16 flash_addr;
	u8 *verify_data;
	int retry_cnt = 0;
	int i;
	int page_sz = 64;
	u32 size = 32*1024;
	u16 chip_code;
	int fuzing_udelay = 8000;
	u8 *firmware_data = NULL;

	char fw_path[MAX_FW_PATH+1];
	struct file *fp = NULL;
	long fsize = 0, nread = 0;

    mm_segment_t old_fs = get_fs();
    set_fs(KERNEL_DS);

	verify_data = kzalloc(size, GFP_KERNEL);
	if (verify_data == NULL) {
		pr_err("cannot alloc verify buffer\n");
		return false;
	}

	snprintf(fw_path, MAX_FW_PATH, "/sdcard/%s", ZP3390_FW_FILENAME);
	fp = filp_open(fw_path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("file %s open error\n", fw_path);
		goto err_open;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;

	if(fsize != info->cap_info.ic_fw_size) {
		pr_err("invalid fw size!!\n");
		goto err_open;
	}

	firmware_data = kzalloc((size_t)fsize, GFP_KERNEL);
	if (!firmware_data) {
		pr_err("failed to alloc buffer for fw\n");
		goto err_alloc;
	}

	nread = vfs_read(fp, (char __user *)firmware_data, fsize, &fp->f_pos);
	if (nread != fsize) {
		goto err_fw_size;
	}

	filp_close(fp, current->files);
    set_fs(old_fs);
	pr_info("ums fw is loaded!!\n");

	zp3390_power_control(charger, ZP3390_POWER_OFF);
retry_upgrade:
	zp3390_power_control(charger, ZP3390_POWER_ON);

	if (zp3390_write_reg_wdata(client, 0x00c0, 0x0100) != I2C_SUCCESS){
		pr_err("power sequence error (vendor cmd enable)\n");
		goto fail_upgrade;
	}

	usleep_range(10, 10);

	if (zp3390_reg_read(client, 0x00cc, (u8 *)&chip_code) < 0) {
		pr_err("failed to read chip code\n");
		goto fail_upgrade;
	}

	pr_info("chip code = 0x%x\n", chip_code);

	usleep_range(10, 10);

	if (zp3390_write_cmd(client, 0x04c0) != I2C_SUCCESS){
		pr_err("power sequence error (intn clear)\n");
		goto fail_upgrade;
	}

	usleep_range(10, 10);

	if (zp3390_write_reg_wdata(client, 0x02c0, 0x01000) != I2C_SUCCESS){
		pr_err("power sequence error (nvm init)\n");
		goto fail_upgrade;
	}

	usleep_range(5 * 1000, 5 * 1000);

	pr_info("init flash\n");

	if (zp3390_write_reg_wdata(client, 0x03c0, 0x0100) != I2C_SUCCESS) {
		pr_err("failed to write nvm vpp on\n");
		goto fail_upgrade;
	}

	if (zp3390_write_reg_wdata(client, 0x04c1, 0x0100) != I2C_SUCCESS){
		pr_err("failed to write nvm wp disable\n");
		goto fail_upgrade;
	}

	if (zp3390_write_cmd(client, 0xD001) != I2C_SUCCESS) {
		pr_err("failed to init flash\n");
		goto fail_upgrade;
	}

	// Mass Erase
	//====================================================
	if (zp3390_write_cmd(client, 0xDF01) != I2C_SUCCESS) {
		pr_err("failed to mass erase\n");
		goto fail_upgrade;
	}

	msleep(100);

	// Mass Erase End
	//====================================================

	if (zp3390_write_reg_wdata(client, 0xDE01, 0x0100) != I2C_SUCCESS) {
		pr_err("failed to enter upgrade mode\n");
		goto fail_upgrade;
	}

	usleep_range(1000, 1000);

	if (zp3390_write_reg_wdata(client, 0xD301, 0x0800) != I2C_SUCCESS) {
		pr_err("failed to init upgrade mode\n");
		goto fail_upgrade;
	}

	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			if (write_firmware_data(client,
				0x01D1,
				(u8 *)&firmware_data[flash_addr],TC_SECTOR_SZ) < 0) {
				pr_err("error : write zinitix tc firmare\n");
				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
			usleep_range(100, 100);
		}

		usleep_range(fuzing_udelay, fuzing_udelay);	/*for fuzing delay*/
	}

	if (zp3390_write_reg_wdata(client, 0x03c0, 0x0000) != I2C_SUCCESS) {
		pr_err("nvm write vpp off\n");
		goto fail_upgrade;
	}

	if (zp3390_write_reg_wdata(client, 0x04c1, 0x0000) != I2C_SUCCESS){
		pr_err("nvm wp enable\n");
		goto fail_upgrade;
	}

	pr_info("init flash\n");

	if (zp3390_write_cmd(client, 0xD001) != I2C_SUCCESS) {
		pr_info("failed to init flash\n");
		goto fail_upgrade;
	}

	pr_info("read firmware data\n");

	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz/TC_SECTOR_SZ; i++) {
			if (read_firmware_data(client,
				0x01D2,
				(u8*)&verify_data[flash_addr], TC_SECTOR_SZ) < 0) {
				input_err(true, &client->dev, "Failed to read firmare\n");

				goto fail_upgrade;
			}

			flash_addr += TC_SECTOR_SZ;
		}
	}

	/* verify */
	pr_info(true, &client->dev, "verify firmware data\n");
	if (memcmp((u8 *)&firmware_data[0], (u8 *)&verify_data[0], size) == 0) {
		pr_info(true, &client->dev, "upgrade finished\n");

		zp3390_power_control(charger, ZP3390_POWER_OFF);
		zp3390_power_control(charger, ZP3390_POWER_ON);
		msleep(50);

		if (verify_data){
			pr_info("kfree\n");
			kfree(verify_data);
			verify_data = NULL;
		}

		kfree(firmware_data);
		return true;
	}

fail_upgrade:
	zp3390_power_control(charger, ZP3390_POWER_OFF);
	msleep(50);

	if (retry_cnt++ < FW_UPDATE_RETRY_CNT) {
		pr_err("upgrade failed : so retry... (%d)\n", retry_cnt);
		goto retry_upgrade;
	}

	if (verify_data){
		pr_info("kfree\n");
		kfree(verify_data);
	}
	if (firmware_data)
		kfree(firmware_data);

	pr_info("Failed to upgrade\n");

	if (fp != NULL) {
		err_fw_size:
			kfree(firmware_data);
		err_alloc:
			filp_close(fp, NULL);
		err_open:
			set_fs(old_fs);
	}
	return false;
}
#endif

static int zp3390_is_on_pad(struct zp3390_charger_data *charger)
{
	int ret;
	int wpc_det = charger->pdata->wpc_det;
	ret = (gpio_get_value(wpc_det) == 0) ? WPC_ON_PAD : WPC_OFF_PAD;
	return ret;
}

static int zp3390_get_adc(struct zp3390_charger_data *charger, int adc_type)
{
	int ret = 0;
	u8 data[2] = {0,};

	switch (adc_type) {
		case ZP3390_ADC_VOUT:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_VOUT_L_REG, &data[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_VOUT_H_REG, &data[1]);
			if(ret >= 0 ) {
				//data[1] &= 0x0f;
				ret = (data[0] | (data[1] << 8));
			} else
				ret = 0;
			break;
		case ZP3390_ADC_VRECT:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_VRECT_L_REG, &data[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_VRECT_H_REG, &data[1]);
			if(ret >= 0 ) {
				//data[1] &= 0x0f;
				ret = (data[0] | (data[1] << 8));
			} else
				ret = 0;
			break;
		case ZP3390_ADC_TX_ISENSE:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_TX_ISENSE_L_REG, &data[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_TX_ISENSE_H_REG, &data[1]);
			if(ret >= 0 ) {
				data[1] &= 0x0f;
				ret = (data[0] | (data[1] << 8)); // need to check
			} else
				ret = 0;
			break;
		case ZP3390_ADC_RX_IOUT:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_RX_IOUT_L_REG, &data[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_RX_IOUT_H_REG, &data[1]);
			if(ret >= 0 ) {
				ret = (data[0] | (data[1] << 8));
			} else
				ret = 0;
			break;
		case ZP3390_ADC_DIE_TEMP:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_DIE_TEMP_L_REG, &data[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_DIE_TEMP_H_REG, &data[1]);
			if(ret >= 0 ) {
				data[1] &= 0x0f;
				ret = (data[0] | (data[1] << 8)); // need to check
			} else
				ret = 0;
			break;

		case ZP3390_ADC_ALLIGN_X:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_ALLIGN_X_REG, &data[0]);
			if(ret >= 0 ) {
				ret = data[0]; // need to check
			} else
				ret = 0;
			break;

		case ZP3390_ADC_ALLIGN_Y:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_ALLIGN_Y_REG, &data[0]);
			if(ret >= 0 ) {
				ret = data[0]; // need to check
			} else
				ret = 0;
			break;
		case ZP3390_ADC_OP_FRQ:
			ret = zp3390_reg_read(charger->client, ZP3390_OP_FREQ_L_REG, &data[0]);
			if(ret < 0 ) {
				ret = 0;
				return ret;
			}
			ret = zp3390_reg_read(charger->client, ZP3390_OP_FREQ_H_REG, &data[1]);
			if(ret >= 0 )
				ret = (data[0] | (data[1] << 8));
			else
				ret = 0;
			break;
		case ZP3390_ADC_VBAT_RAW:
			ret = zp3390_reg_read(charger->client, ZP3390_VBAT_L_REG, &data[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_VBAT_H_REG, &data[1]);
			if(ret >= 0 ) {
				//data[1] &= 0x0f;
				ret = (data[0] | (data[1] << 8)); // need to check
			} else
				ret = 0;
			break;
		case ZP3390_ADC_VBAT:
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_VBAT_L_REG, &data[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_ADC_VBAT_H_REG, &data[1]);
			if(ret >= 0 ) {
				data[1] &= 0x0f;
				ret = (data[0] | (data[1] << 8)); // need to check
			} else
				ret = 0;
			break;
		default:
			break;
	}

	return ret;
}

static void zp3390_fod_set(struct zp3390_charger_data *charger)
{
	int i = 0;

	pr_info("%s \n", __func__);
	if(charger->pdata->fod_data_check) {
		for(i=0; i< ZP3390_NUM_FOD_REG; i++)
			zp3390_reg_write(charger->client, ZP3390_WPC_FOD_0A_REG+i, charger->pdata->fod_data[i]);
	}
}

static void zp3390_set_cmd_reg(struct zp3390_charger_data *charger, u8 val, u8 mask)
{
	u8 temp = 0;
	int ret = 0, i = 0;

	do {
		pr_info("%s \n", __func__);
		ret = zp3390_reg_update(charger->client, ZP3390_COMMAND_REG, val, mask); // command
		if(ret >= 0) {
			msleep(250);
			ret = zp3390_reg_read(charger->client, ZP3390_COMMAND_REG, &temp); // check out set bit exists
			if(ret < 0 || i > 3 )
				break;
		}
		i++;
	}while(temp != 0);
}

void zp3390_send_eop(struct zp3390_charger_data *charger, int healt_mode)
{
	int i = 0;
	int ret = 0;

	switch(healt_mode) {
		case POWER_SUPPLY_HEALTH_OVERHEAT:
		case POWER_SUPPLY_HEALTH_OVERHEATLIMIT:
		case POWER_SUPPLY_HEALTH_COLD:
			if(charger->pdata->cable_type == SEC_WIRELESS_PAD_PMA) {
				pr_info("%s pma mode \n", __func__);
				for(i = 0; i < CMD_CNT; i++) {
					ret = zp3390_reg_write(charger->client,
						ZP3390_END_POWER_TRANSFER_REG, ZP3390_EPT_END_OF_CHG);
					if(ret >= 0) {
						zp3390_set_cmd_reg(charger,
							ZP3390_CMD_SEND_EOP_MASK, ZP3390_CMD_SEND_EOP_MASK);
						msleep(250);
					} else
						break;
				}
			} else {
				pr_info("%s wpc mode \n", __func__);
				for(i = 0; i < CMD_CNT; i++) {
					ret = zp3390_reg_write(charger->client,
						ZP3390_END_POWER_TRANSFER_REG, ZP3390_EPT_OVER_TEMP);
					if(ret >= 0) {
						zp3390_set_cmd_reg(charger,
							ZP3390_CMD_SEND_EOP_MASK, ZP3390_CMD_SEND_EOP_MASK);
						msleep(250);
					} else
						break;
				}
			}
			break;
		case POWER_SUPPLY_HEALTH_UNDERVOLTAGE:
#if 0
			pr_info("%s ept-reconfigure \n", __func__);
			ret = zp3390_reg_write(charger->client, ZP3390_END_POWER_TRANSFER_REG, ZP3390_EPT_RECONFIG);
			if(ret >= 0) {
				zp3390_set_cmd_reg(charger, ZP3390_CMD_SEND_EOP_MASK, ZP3390_CMD_SEND_EOP_MASK);
				msleep(250);
			}
#endif
			break;
		default:
			break;
	}
}

int zp3390_send_cs100(struct zp3390_charger_data *charger)
{
	int i = 0;
	int ret = 0;

	for(i = 0; i < CMD_CNT; i++) {
		ret = zp3390_reg_write(charger->client, ZP3390_CHG_STATUS_REG, 100);
		if(ret >= 0) {
			zp3390_set_cmd_reg(charger, ZP3390_CMD_SEND_CHG_STS_MASK, ZP3390_CMD_SEND_CHG_STS_MASK);
			msleep(250);
			ret = 1;
		} else {
			ret = -1;
			break;
		}
	}
	return ret;
}

void zp3390_send_packet(struct zp3390_charger_data *charger, u8 header, u8 rx_data_com, u8 *data_val, int data_size)
{
	int ret;
	int i;
	ret = zp3390_reg_write(charger->client, ZP3390_PACKET_HEADER, header);
	ret = zp3390_reg_write(charger->client, ZP3390_RX_DATA_COMMAND, rx_data_com);

	for(i = 0; i< data_size; i++) {
		ret = zp3390_reg_write(charger->client, ZP3390_RX_DATA_VALUE0 + i, data_val[i]);
	}
	zp3390_set_cmd_reg(charger, ZP3390_CMD_SEND_RX_DATA_MASK, ZP3390_CMD_SEND_RX_DATA_MASK);
}

static bool zp3390_check_chg_in_status(struct zp3390_charger_data *charger)
{
	union power_supply_propval value;
	bool ret = false;

	psy_do_property(charger->pdata->charger_name, get,
		POWER_SUPPLY_PROP_POWER_NOW, value);

	if (value.intval == ACOK_INPUT)
		ret = true;

	return ret;
}

static bool zp3390_wait_chg_in_ok(struct zp3390_charger_data *charger, int retry)
{
	bool chg_in_ok;
	do {
		if (zp3390_is_on_pad(charger) == WPC_OFF_PAD) {
			pr_info("%s pad is off!\n", __func__);
			break;
		}

		chg_in_ok = zp3390_check_chg_in_status(charger);

		pr_info("%s chg_in_ok = %d vout = %d retry = %d\n",
			__func__, chg_in_ok, charger->vout_status, retry);

		if (chg_in_ok)
			return true;
		msleep(300);

		retry--;
	} while(retry > 0);

	return false;
}

static void zp3390_rx_ic_reset(struct zp3390_charger_data *charger)
{
	int ping_durationg =
		(charger->pdata->ping_duration * 2) - (charger->pdata->ping_duration / 4);

	pr_info("%s ping_durationg = %d\n", __func__, ping_durationg);

	if (gpio_is_valid(charger->pdata->wpc_en)) {
		wake_lock(&charger->wpc_wake_lock);

		/* rx_ic reset need 5 pings when wpc_en high status */
		gpio_direction_output(charger->pdata->wpc_en, 1);
		msleep(ping_durationg);
		gpio_direction_output(charger->pdata->wpc_en, 0);
		wake_unlock(&charger->wpc_wake_lock);

		charger->charge_mode = ZP3390_CHARGE_MODE_NONE;
		zp3390_wait_chg_in_ok(charger, 8);
	}
}

static void zp3390_power_hold_mode_set(struct zp3390_charger_data *charger, int set)
{
	int i = 0;

	if (charger->support_power_hold == 0) {
		pr_info("%s power hold not supported pad!\n", __func__);
		return;
	}

	pr_info("%s set = %d power_hold_mode = %d tx_id = %x\n", __func__,
		set, charger->power_hold_mode, charger->tx_id);

	wake_lock(&charger->wpc_wake_lock);

	if (set) {
		if (charger->power_hold_mode == 0) {
			charger->power_hold_mode = 1;
			for (i = 0; i < 3; i++) {
				zp3390_reg_write(charger->client, ZP3390_PACKET_HEADER, 0x28);
				zp3390_reg_write(charger->client, ZP3390_RX_DATA_COMMAND, 0x18);
				zp3390_reg_write(charger->client, ZP3390_RX_DATA_VALUE0, 0x01);
				zp3390_reg_write(charger->client, ZP3390_COMMAND_REG, 0x01);
				msleep(200);
			}
		}
	} else {
		if (charger->power_hold_mode == 1) {
			charger->power_hold_mode = 0;
			if (gpio_is_valid(charger->pdata->wpc_en)) {

				/* power hold mode exit need 2 pings when wpc_en high status */
				gpio_direction_output(charger->pdata->wpc_en, 1);
				msleep(charger->pdata->ping_duration * 2);
				gpio_direction_output(charger->pdata->wpc_en, 0);

				if (charger->wc_w_state == WPC_OFF_PAD) {
					pr_err("%s wpc detached!!\n", __func__);
					wake_unlock(&charger->wpc_wake_lock);
					return;
				}

				if (zp3390_wait_chg_in_ok(charger, 10) == true) {
					/* for stable ap mode wirte after acok inserted */
					msleep(300);
					charger->charge_mode = ZP3390_CHARGE_MODE_NONE;

					if (zp3390_check_chg_in_status(charger) == 0) {
						zp3390_rx_ic_reset(charger);
						pr_err("%s chg_in err!\n", __func__);
					}
				} else {
					zp3390_rx_ic_reset(charger);
				}
			} else {
				pr_err("%s wpc_en is invalid gpio!!\n", __func__);
			}
		}
	}

	wake_unlock(&charger->wpc_wake_lock);
	pr_info("%s end\n", __func__);
}

static int zp3390_get_firmware_version(struct zp3390_charger_data *charger, int firm_mode)
{
	int version = -1;
	int ret;
	u8 otp_fw_major[2] = {0,};
	u8 otp_fw_minor[2] = {0,};

	if (zp3390_is_on_pad(charger) == WPC_OFF_PAD)
		return version;

	switch (firm_mode) {
		case ZP3390_RX_FIRMWARE:
		case ZP3390_TX_FIRMWARE:
			ret = zp3390_reg_read(charger->client, ZP3390_FW_MAJOR_REV_L_REG, &otp_fw_major[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_FW_MAJOR_REV_H_REG, &otp_fw_major[1]);
			if (ret >= 0) {
				version =  otp_fw_major[0] | (otp_fw_major[1] << 8);
			}
			pr_debug("%s rx major firmware version 0x%x \n", __func__, version);

			ret = zp3390_reg_read(charger->client, ZP3390_FW_MINOR_REV_L_REG, &otp_fw_minor[0]);
			ret = zp3390_reg_read(charger->client, ZP3390_FW_MINOR_REV_H_REG, &otp_fw_minor[1]);
			if (ret >= 0) {
				version =  otp_fw_minor[0] | (otp_fw_minor[1] << 8);
			}
			pr_debug("%s rx minor firmware version 0x%x \n", __func__, version);
			break;
		default:
			pr_err("%s Wrong firmware mode \n", __func__);
			version = -1;
			break;
	}

	return version;
}

static void zp3390_wpc_detach_check(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_detach_chk_work.work);
	int cnt = 0;
	bool is_charging;
	union power_supply_propval value;

	while (cnt < 1) {
		if (zp3390_is_on_pad(charger) == WPC_ON_PAD) {
			is_charging = zp3390_check_chg_in_status(charger);
			if ((charger->power_hold_mode == 0) && (is_charging == 0)) {
				pr_info("%s toggle wpc_en for acok glitch WA\n", __func__);
				zp3390_rx_ic_reset(charger);
			}
		} else {
			psy_do_property(charger->pdata->battery_name, get,
				POWER_SUPPLY_PROP_ONLINE, value);
			pr_info("%s pad off!! cable_type(%d)\n", __func__, value.intval);

			if (is_wireless_type(value.intval)) {
				pr_info("%s send wpc_det_work\n", __func__);
				queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
			}
			break;
		}
		cnt++;
	}
}

static void zp3390_wpc_power_hold_check(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, power_hold_chk_work.work);

	if ((charger->power_hold_mode == 1) &&
		zp3390_check_chg_in_status(charger)) {
		pr_err("%s abnormal chg status!! go to vout off or phm!!\n", __func__);
		charger->power_hold_mode = 0;
		zp3390_full_charge(charger);
	}

	wake_unlock(&charger->power_hold_chk_lock);
}

static void zp3390_wpc_wait_vout(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_wait_vout_work.work);
	union power_supply_propval value;
	static int cnt = 0;

	if (charger->wc_w_state == WPC_OFF_PAD) {
		wake_unlock(&charger->wpc_wait_vout_lock);
		cnt = 0;
		return;
	}

	psy_do_property(charger->pdata->charger_name, get,
		POWER_SUPPLY_PROP_POWER_NOW, value);

	pr_info("%s ACOK = %d cnt = %d\n", __func__, value.intval, cnt);

	if (value.intval == ACOK_INPUT) {
		cnt = 0;
		charger->charge_mode = ZP3390_CHARGE_MODE_NONE;
		zp3390_ap_battery_monitor(charger, WPC_MODE_NORMAL);

		/* for stable charge acok insertd -> 0.5 sec delay -> charge on */
		msleep(500);

		if (charger->wc_w_state == WPC_OFF_PAD) {
			wake_unlock(&charger->wpc_wait_vout_lock);
			return;
		}

		value.intval = SEC_WIRELESS_PAD_WPC;
		psy_do_property(charger->pdata->charger_name, set,
			POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);

		/* if acok is inserted */
		charger->pdata->cable_type = ZP3390_PAD_MODE_WPC;
		value.intval = SEC_WIRELESS_PAD_WPC;
		psy_do_property(charger->pdata->wireless_name, set,
				POWER_SUPPLY_PROP_ONLINE, value);
	} else {
		if (cnt < 30) {
			cnt++;
			queue_delayed_work(charger->wqueue,
				&charger->wpc_wait_vout_work, msecs_to_jiffies(100));
			return;
		} else {
			pr_err("%s ACOK is not inserted!!\n", __func__);
			cnt = 0;
		}
	}

	wake_unlock(&charger->wpc_wait_vout_lock);
}

static void zp3390_wpc_init(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_init_work.work);
	union power_supply_propval acok;
	int ret;
	int fw_ver;

	wake_lock(&charger->wpc_wake_lock);

	psy_do_property(charger->pdata->charger_name, get,
		POWER_SUPPLY_PROP_POWER_NOW, acok);

	pr_info("%s wpc sts val:%x acok:%d\n", __func__, charger->wc_w_state, acok.intval);

	if (charger->wc_w_state == WPC_ON_PAD) {
		if (acok.intval == 0) {
			if (charger->pdata->can_vbat_monitoring) {
				/* VRECT TARGET VOLTAGE = 5449mV(0x0427) */
				zp3390_reg_write(charger->client, ZP3390_VRECT_TARGET_VOL_L, 0X27);
				zp3390_reg_write(charger->client, ZP3390_VRECT_TARGET_VOL_H, 0X04);
			} else {
				zp3390_reg_write(charger->client, ZP3390_VRECT_MARGIN_REG, 126);
				zp3390_reg_update(charger->client, 0x81, 0x80, 0x80); //set 7th bit in 0x81 to power on
			}
			zp3390_reg_update(charger->client, ZP3390_COMMAND_REG,
				ZP3390_CMD_TOGGLE_LDO_MASK, ZP3390_CMD_TOGGLE_LDO_MASK);
		}

		/* check again firmare version support vbat monitoring */
		if (!charger->pdata->otp_firmware_ver) {
			fw_ver = zp3390_get_firmware_version(charger, ZP3390_RX_FIRMWARE);
			if (fw_ver > 0) {
				pr_debug("%s rx major firmware version 0x%x\n", __func__, fw_ver);
				charger->pdata->otp_firmware_ver = fw_ver;
			}
		}
		queue_delayed_work(charger->wqueue,
				&charger->wpc_wait_vout_work,
				msecs_to_jiffies(100));
		schedule_delayed_work(&charger->wpc_isr_work, 0);

		ret = enable_irq_wake(charger->client->irq);
		if (ret < 0)
			pr_err("%s: Failed to Enable Wakeup Source(%d)\n", __func__, ret);
	}

	if (charger->wc_w_state == WPC_ON_PAD)
		zp3390_wpc_auth_delayed_work(charger, WPC_AUTH_DELAY_TIME);

	wake_unlock(&charger->wpc_wake_lock);
}

static int zp3390_temperature_check(struct zp3390_charger_data *charger)
{
	if (charger->temperature >= charger->pdata->tx_off_high_temp) {
		/* send TX watchdog command in high temperature */
		pr_err("%s:HIGH TX OFF TEMP:%d\n", __func__, charger->temperature);
		zp3390_reg_write(charger->client, ZP3390_PACKET_HEADER, 0x18);
		zp3390_reg_write(charger->client, ZP3390_RX_DATA_COMMAND, 0xe7);
		zp3390_reg_write(charger->client, ZP3390_COMMAND_REG, 0x01);
		zp3390_reg_write(charger->client, ZP3390_PACKET_HEADER, 0x18);
		zp3390_reg_write(charger->client, ZP3390_RX_DATA_COMMAND, 0xe7);
		zp3390_reg_write(charger->client, ZP3390_COMMAND_REG, 0x01);
	}

	return 0;
}

static int zp3390_monitor_work(struct zp3390_charger_data *charger)
{
	int vrect;
	int vout;
	int freq;
	int temp;
	int iout;
	int vbat;
	u8 ap_mode, vbat_monitor;
	int capacity;
	union power_supply_propval value = {0, };

	if (zp3390_is_on_pad(charger) == WPC_ON_PAD) {
		value.intval = SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE;
		psy_do_property(charger->pdata->fuelgauge_name, get,
			POWER_SUPPLY_PROP_CAPACITY, value);
		capacity = value.intval;

		vrect = zp3390_get_adc(charger, ZP3390_ADC_VRECT);
		vout = zp3390_get_adc(charger, ZP3390_ADC_VOUT);
		freq = zp3390_get_adc(charger, ZP3390_ADC_OP_FRQ);
		temp = zp3390_get_adc(charger, ZP3390_ADC_DIE_TEMP);
		iout = zp3390_get_adc(charger, ZP3390_ADC_RX_IOUT);
		vbat = zp3390_get_adc(charger, ZP3390_ADC_VBAT_RAW);

		zp3390_reg_read(charger->client, ZP3390_AP_BATTERY_MODE, &ap_mode);
		zp3390_reg_read(charger->client, ZP3390_WPC_FLAG_REG, &vbat_monitor);

		pr_info("%s vrect(%dmV) vout(%dmV) iout(%dmA) vbat(%dmV) ap_mode(0x%02x) SOC(%d) "
			"pwr_hold(%d) chg_type(%d) chg_mode(%d) "
			"fw(%x) vbat_monitor(%x) tx_id(%x) freq(%d) temp(%d)\n",
			__func__, vrect, vout, iout, vbat, ap_mode, capacity,
			charger->power_hold_mode, charger->charger_type, charger->charge_mode,
			charger->pdata->otp_firmware_ver, vbat_monitor,
			charger->tx_id, freq, temp);
	} else {
		pr_info("%s pad off!\n", __func__);
	}

	return 0;
}

static int zp3390_monitor_wdt_kick(struct zp3390_charger_data *charger)
{
	if ((charger->pdata->watchdog_test == false) &&
		(charger->wc_w_state == WPC_ON_PAD)) {
		zp3390_reg_update(charger->client, ZP3390_COMMAND_REG,
			ZP3390_CMD_SS_WATCHDOG_MASK, ZP3390_CMD_SS_WATCHDOG_MASK);
	}
	return 0;
}

static int zp3390_write_ap_mode(struct zp3390_charger_data *charger, int ap_mode)
{
	u8 data, i;
	int ret = -1;

	if (charger->curr_measure_mode == 1) {
		pr_info("%s current_measure_mode. skip ap mode setting!\n", __func__);
		return 0;
	}

	if (charger->need_margin == 1) {
		charger->need_margin = 0;
		msleep(150);
	}

	for (i = 0; i < 3; i++) {
		zp3390_reg_write(charger->client, ZP3390_AP_BATTERY_MODE, ap_mode);
		zp3390_reg_read(charger->client, ZP3390_AP_BATTERY_MODE, &data);

		if (ap_mode == data) {
			ret = 0;
			goto end;
		}
		msleep(100);
	}
end:
	pr_info("%s ap_mode %x read data = %x\n",
		__func__, ap_mode, data);
	return ret;
}

static int
zp3390_compatible_cc_charge(struct zp3390_charger_data *charger, bool prev_chk)
{
	int ret = 0;

	if (charger->store_mode) {
		ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_BATT_MODE);
	}

	if (prev_chk == false) {
		ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_CC_MODE);
	} else {
		if (charger->charge_mode != ZP3390_CHARGE_MODE_CC)
			ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_CC_MODE);
	}
	return ret;
}

static int
zp3390_compatible_cv_charge(struct zp3390_charger_data *charger, bool prev_chk)
{
	int ret = 0;

	if (prev_chk == false) {
		ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_CC_CV_MODE);
	} else {
		if (charger->charge_mode != ZP3390_CHARGE_MODE_CV)
			ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_CC_CV_MODE);
	}
	return ret;
}

static int
zp3390_incompatible_cc_charge(struct zp3390_charger_data *charger, bool prev_chk)
{
	int ret = 0;

	if (prev_chk == false) {
		ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_INCOMPATIBLE_CC_MODE);
	} else {
		if (charger->charge_mode != ZP3390_CHARGE_MODE_CC)
			ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_INCOMPATIBLE_CC_MODE);
	}
	return ret;
}

static int
zp3390_incompatible_cv_charge(struct zp3390_charger_data *charger, bool prev_chk)
{
	int ret = 0;

	if (prev_chk == false) {
		ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_INCOMPATIBLE_CC_CV_MODE);
	} else {
		if (charger->charge_mode != ZP3390_CHARGE_MODE_CV)
			ret = zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_INCOMPATIBLE_CC_CV_MODE);
	}
	return ret;
}

static int zp3390_power_hold_full_charge(struct zp3390_charger_data *charger)
{
	pr_info("%s\n", __func__);
	zp3390_power_hold_mode_set(charger, 1);
	return 0;
}

static int zp3390_power_hold_re_charge(struct zp3390_charger_data *charger)
{
	pr_info("%s\n", __func__);
	zp3390_power_hold_mode_set(charger, 0);
	return 0;
}

static int zp3390_normal_full_charge(struct zp3390_charger_data *charger)
{
	pr_info("%s\n", __func__);
	wake_lock(&charger->wpc_wake_lock);
	zp3390_ap_battery_monitor(charger, WPC_MODE_VOUT_OFF);
	msleep(200);
	zp3390_reg_update(charger->client, ZP3390_COMMAND_REG,
		ZP3390_CMD_TOGGLE_LDO_MASK, ZP3390_CMD_TOGGLE_LDO_MASK);
	charger->power_hold_mode = 1;
	wake_unlock(&charger->wpc_wake_lock);
	return 0;
}

static int zp3390_normal_re_charge(struct zp3390_charger_data *charger)
{
	int i, vrect;

	pr_info("%s\n", __func__);
	wake_lock(&charger->wpc_wake_lock);

	zp3390_reg_update(charger->client, ZP3390_COMMAND_REG,
		ZP3390_CMD_TOGGLE_LDO_MASK, ZP3390_CMD_TOGGLE_LDO_MASK);
	zp3390_ap_battery_monitor(charger, WPC_MODE_IDT);

	if (zp3390_wait_chg_in_ok(charger, 10) == true) {
		for (i = 0; i < 15; i++) {
			vrect = zp3390_get_adc(charger, ZP3390_ADC_VRECT);
			pr_info("%s %d vrect = %d\n", __func__, i, vrect);

			/* IDT mode raise Vrect to 5.5V
			 * If CEP 10 is adjusted, IDT mode raise Vrect to 4.95V
			 * therefore, considering set marging wait Vrect until over 4.9V
			 */
			if (vrect > 4900)
				break;

			msleep(100);
		}
	}
	charger->power_hold_mode = 0;

	pr_info("%s end\n", __func__);
	wake_unlock(&charger->wpc_wake_lock);
	return 0;
}

static int zp3390_cc_charge(struct zp3390_charger_data *charger, bool prev_chk)
{
	int ret = -1;
	enum zp3390_charger_pad_type type = charger->charger_type;

	if (charger->charge_cb_func[type]->cc_chg) {
		ret = charger->charge_cb_func[type]->cc_chg(charger, prev_chk);
		charger->charge_mode = ZP3390_CHARGE_MODE_CC;
	} else
		pr_err("%s invalid func\n", __func__);
	return ret;
}

static int zp3390_cv_charge(struct zp3390_charger_data *charger, bool prev_chk)
{
	int ret = -1;
	enum zp3390_charger_pad_type type = charger->charger_type;

	if (charger->charge_cb_func[type]->cv_chg) {
		ret = charger->charge_cb_func[type]->cv_chg(charger, prev_chk);
		charger->charge_mode = ZP3390_CHARGE_MODE_CV;
	} else
		pr_err("%s invalid func\n", __func__);
	return ret;
}

static int zp3390_full_charge(struct zp3390_charger_data *charger)
{
	int ret = -1;
	int i;
	enum zp3390_charger_pad_type type = charger->charger_type;
	union power_supply_propval value = {0, };

	pr_info("%s type = %d charger->charge_mode = %d\n",
		__func__, type, charger->charge_mode);

	if (charger->tx_id == 0) {
		for (i = 0; i < 50; i++) {
			if (charger->tx_id != 0) {
				break;
			}
			pr_info("%s %d tx_id = %d\n", __func__, i, charger->tx_id);
			msleep(100);
		}
	}

	mutex_lock(&charger->charger_lock);

	if (charger->charge_cb_func[type]->full_chg)  {
		ret = charger->charge_cb_func[type]->full_chg(charger);
		charger->charge_mode = ZP3390_CHARGE_MODE_OFF;
		charger->pdata->is_charging = 0;
		value.intval = SEC_WIRELESS_PAD_NONE;
		psy_do_property(charger->pdata->charger_name, set,
			POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);
	} else
		pr_err("%s invalid func\n", __func__);

	mutex_unlock(&charger->charger_lock);
	return ret;
}

static int zp3390_re_charge(struct zp3390_charger_data *charger)
{
	int ret = -1;
	bool chg_in_ok;
	union power_supply_propval value = {0, };
	enum zp3390_charger_pad_type type = charger->charger_type;

	chg_in_ok = zp3390_check_chg_in_status(charger);

	if (chg_in_ok) {
		pr_info("%s skip. already charge started!\n", __func__);
		return 0;
	}

	pr_info("%s type = %d charger->charge_mode = %d\n",
		__func__, type, charger->charge_mode);

	mutex_lock(&charger->charger_lock);

	if (charger->charge_cb_func[type]->re_chg) {
		charger->charge_mode = ZP3390_CHARGE_MODE_NONE;
		ret = charger->charge_cb_func[type]->re_chg(charger);

		if (zp3390_is_on_pad(charger) == WPC_ON_PAD) {
			charger->pdata->is_charging = 1;
			charger->need_margin = 1;
			value.intval = SEC_WIRELESS_PAD_WPC;
			psy_do_property(charger->pdata->charger_name, set,
				POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);

			zp3390_wpc_request_tx_id(charger, 3);
		}
	} else
		pr_err("%s invalid func\n", __func__);

	mutex_unlock(&charger->charger_lock);
	return ret;
}

static void zp3390_cc_cv_charge_select(
	struct zp3390_charger_data *charger, int capacity, bool prev_check)
{
	if (capacity >= charger->pdata->cc_cv_threshold) {
		zp3390_cv_charge(charger, prev_check);
	} else {
		/* to avoid setting cc mode when once cv mode entered and not detached */
		if ((prev_check == true) && (charger->charge_mode == ZP3390_CHARGE_MODE_CV)) {
			pr_info("%s forcely set cv mode for temp capacity drop\n", __func__);
			zp3390_cv_charge(charger, prev_check);
		} else {
			zp3390_cc_charge(charger, prev_check);
		}
	}
}

static int zp3390_ap_battery_monitor(struct zp3390_charger_data *charger, int mode)
{
	union power_supply_propval value = {0, };
	int capacity;
	u8 ap_battery_mode;
	int ret = 0;
	struct power_supply *batt_psy = NULL;

	batt_psy = get_power_supply_by_name(charger->pdata->battery_name);
	if (batt_psy == NULL)
		return 0;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_EXT_PROP_BOOT_BOOST_CHARGE, value);

	if ((value.intval == 1) && (mode != WPC_MODE_BOOT)) {
		pr_info("%s return by boot boost charge!!\n", __func__);
		return ret;
	}

	value.intval = SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE;
	psy_do_property(charger->pdata->fuelgauge_name, get,
		POWER_SUPPLY_PROP_CAPACITY, value);
	capacity = value.intval;

	ret = zp3390_reg_read(charger->client, ZP3390_AP_BATTERY_MODE,
		&ap_battery_mode);

	switch (mode) {
	case WPC_MODE_BOOT:
		zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_BOOT_MODE);
		charger->charge_mode = ZP3390_CHARGE_MODE_NONE;
		break;
	case WPC_MODE_IDT:
		zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_IDT_MODE);
		charger->charge_mode = ZP3390_CHARGE_MODE_NONE;
		break;
	case WPC_MODE_VOUT_OFF:
		zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_INCOMPATIBLE_PHP_MODE);
		break;
	case WPC_MODE_ATTACH:
		zp3390_cc_cv_charge_select(charger, capacity, false);
		break;
	case WPC_MODE_SWELL_ENTER:
	case WPC_MODE_SWELL_EXIT:
	case WPC_MODE_WAKEUP:
		if (ap_battery_mode == ZP3390_AP_BATTERY_IDT_MODE)
			zp3390_cc_cv_charge_select(charger, capacity, false);

		zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_WAKEUP_MODE);
		if (mode != WPC_MODE_WAKEUP)
			msleep(200);
		break;
	case WPC_MODE_NORMAL:
		if (charger->pdata->otp_firmware_ver == 0x125) {
			if (charger->wpc_wakeup_wa == 0)
				zp3390_cc_cv_charge_select(charger, capacity, true);
		} else {
			zp3390_cc_cv_charge_select(charger, capacity, true);
		}
		break;
	default:
		break;
	};

	ret = zp3390_reg_read(charger->client, ZP3390_AP_BATTERY_MODE,
		&ap_battery_mode);

	if (ret < 0 || ap_battery_mode == 0xFF) {
		ret = -1;
		pr_err("%s ret = %d ap_battery_mode = 0x%x\n",
			__func__, ret, ap_battery_mode);
	}

	return ret;
}

static int zp3390_power_hold_mode_monitor(struct zp3390_charger_data *charger)
{
	union power_supply_propval value = {0, };
	int health;
	int status;
	int chg_mode;
	int swelling_mode;
	int slate_mode;

	wake_lock(&charger->wpc_wake_lock);

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_HEALTH, value);
	health = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_STATUS, value);
	status = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_CHARGE_NOW, value);
	chg_mode = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_EXT_PROP_SWELLING_MODE, value);
	swelling_mode = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_EXT_PROP_SLATE_MODE, value);
	slate_mode = value.intval;

	if (charger->wc_w_state == WPC_OFF_PAD) {
		pr_err("%s exit by pad off!!\n", __func__);
		goto end;
	}

	pr_info("%s health(%d %d) sts(%d %d) chg_mode(%d %d) swell(%d %d)\n",
		__func__,
		charger->battery_health, health,
		charger->battery_status, status,
		charger->battery_chg_mode, chg_mode,
		charger->swelling_mode, swelling_mode);

	if ((chg_mode == SEC_BATTERY_CHARGING_NONE) &&
		(charger->battery_chg_mode == SEC_BATTERY_CHARGING_2ND)){
		zp3390_full_charge(charger);
	} else if (((health == POWER_SUPPLY_HEALTH_OVERHEAT) ||
		(health == POWER_SUPPLY_HEALTH_COLD)) &&
		(charger->battery_health == POWER_SUPPLY_HEALTH_GOOD)) {
		zp3390_full_charge(charger);
	} else if ((charger->battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING) &&
		(status == POWER_SUPPLY_STATUS_CHARGING)) {
		psy_do_property(charger->pdata->battery_name, get,
			POWER_SUPPLY_EXT_PROP_COOL_DOWN_MODE, value);
		if (value.intval == 0)
			zp3390_re_charge(charger);
	}

	if (charger->swelling_mode != swelling_mode) {
		if ((charger->swelling_mode != SWELLING_MODE_FULL) &&
			(swelling_mode == SWELLING_MODE_FULL)) {
			zp3390_full_charge(charger);
		}
	}

	if (charger->slate_mode != slate_mode) {
		if (slate_mode == 1) {
			zp3390_full_charge(charger);
		}
	}

end:
	charger->battery_health = health;
	charger->battery_status = status;
	charger->battery_chg_mode = chg_mode;
	charger->swelling_mode = swelling_mode;
	charger->slate_mode = slate_mode;

	wake_unlock(&charger->wpc_wake_lock);

	return 0;
}

static int zp3390_chg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct zp3390_charger_data *charger =
		power_supply_get_drvdata(psy);
	int ret;
	enum power_supply_ext_property ext_psp = psp;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			ret = zp3390_get_firmware_version(charger, ZP3390_RX_FIRMWARE);
			pr_info("%s rx major firmware version 0x%x \n", __func__, ret);

			if (ret >= 0)
				val->intval = 1;
			else
				val->intval = 0;

			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
		case POWER_SUPPLY_PROP_HEALTH:
		case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
		case POWER_SUPPLY_PROP_CHARGE_POWERED_OTG_CONTROL:
			return -ENODATA;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
				break;
		case POWER_SUPPLY_PROP_ONLINE:
			pr_debug("%s cable_type =%d \n ", __func__, charger->pdata->cable_type);
			val->intval = charger->pdata->cable_type;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = zp3390_is_on_pad(charger);
			pr_debug("%s is on chg pad = %d \n ", __func__, val->intval);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = charger->temperature;
			break;
		case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
			val->intval = charger->pdata->charging_mode;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if(charger->pdata->ic_on_mode || (charger->wc_w_state == WPC_ON_PAD)) {
				val->intval = zp3390_get_adc(charger, ZP3390_ADC_RX_IOUT);
			} else
				val->intval = 0;
			break;
		case POWER_SUPPLY_PROP_ENERGY_NOW: /* vout */
			if(charger->pdata->ic_on_mode || (charger->wc_w_state == WPC_ON_PAD)) {
				val->intval = zp3390_get_adc(charger, ZP3390_ADC_VOUT);
			} else
				val->intval = 0;
			break;

		case POWER_SUPPLY_PROP_ENERGY_AVG: /* vrect */
			if(charger->pdata->ic_on_mode || (charger->wc_w_state == WPC_ON_PAD)) {
				val->intval = zp3390_get_adc(charger, ZP3390_ADC_VRECT);
			} else
				val->intval = 0;
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			pr_info("%s charger->pdata->is_charging = %d\n",
				__func__, charger->pdata->is_charging);
			val->intval = charger->pdata->is_charging;
			break;
		case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
			switch (ext_psp) {
				case POWER_SUPPLY_EXT_PROP_MONITOR_WORK:
					zp3390_monitor_work(charger);
					val->intval = 0;
					break;
				default:
					break;
			}
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int zp3390_chg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct zp3390_charger_data *charger =
		power_supply_get_drvdata(psy);
	enum power_supply_ext_property ext_psp = psp;
	int vout_status = 0;

	pr_debug("%s: psp=%d val=%d\n", __func__, psp, val->intval);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if(val->intval == POWER_SUPPLY_STATUS_FULL) {
				pr_info("%s set green led \n", __func__);
				charger->pdata->cs100_status = zp3390_send_cs100(charger);
				charger->pdata->cs100_status = zp3390_send_cs100(charger);
			}
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			if (zp3390_is_on_pad(charger) == WPC_ON_PAD) {
				msleep(250);
				pr_info("%s: Charger interrupt occured during lpm \n", __func__);
				queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
			}
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			break;
		case POWER_SUPPLY_PROP_TEMP:
			charger->temperature = val->intval;
			zp3390_temperature_check(charger);
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			break;
		case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
			if(val->intval) {
				charger->pdata->ic_on_mode = true;
			} else {
				charger->pdata->ic_on_mode = false;
			}
			break;
		case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			break;
		case POWER_SUPPLY_PROP_ENERGY_NOW:
			break;
		case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
			switch (ext_psp) {
				case POWER_SUPPLY_EXT_PROP_WDT_KICK:
					zp3390_monitor_wdt_kick(charger);
					break;
				case POWER_SUPPLY_EXT_PROP_WIRELESS_CHARGE_MONITOR:
					zp3390_ap_battery_monitor(charger, val->intval);
					zp3390_power_hold_mode_monitor(charger);
					break;
				case POWER_SUPPLY_EXT_PROP_IS_RECHARGE:
					vout_status = zp3390_check_chg_in_status(charger);
					pr_info("%s pwr_hold = %d vout_status = %d intval = %d\n",
						__func__, charger->power_hold_mode,
						vout_status, val->intval);
					if ((vout_status == 0) && (val->intval == 1)) {
						wake_lock(&charger->wpc_wake_lock);
						zp3390_re_charge(charger);
						wake_unlock(&charger->wpc_wake_lock);
					}
					break;
				case POWER_SUPPLY_EXT_PROP_WPC_VOUT_OFF:
					vout_status = zp3390_check_chg_in_status(charger);
					if (vout_status == 1) {
						wake_lock(&charger->wpc_wake_lock);
						zp3390_full_charge(charger);
						wake_unlock(&charger->wpc_wake_lock);
					}
					break;
				case POWER_SUPPLY_EXT_PROP_UPDATE_BATTERY_DATA:
					break;
				case POWER_SUPPLY_EXT_PROP_STORE_MODE:
					charger->store_mode = val->intval;
					break;
				default:
					break;
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static void zp3390_wpc_opfq_work(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_opfq_work.work);

	u16 op_fq;
	u8 pad_mode;
	union power_supply_propval value;

	zp3390_reg_read(charger->client, ZP3390_SYS_OP_MODE_REG, &pad_mode);
	if (pad_mode == ZP3390_PAD_MODE_WPC) {
		op_fq = zp3390_get_adc(charger, ZP3390_ADC_OP_FRQ);
			pr_info("%s: Operating FQ %dkHz(0x%x)\n", __func__, op_fq, op_fq);
		if (op_fq > 190) { /* wpc threshold 190kHz */
				pr_info("%s: Reset M0\n",__func__);
				zp3390_reg_write(charger->client, 0x3040, 0x80); /*restart M0 */

				charger->pdata->opfq_cnt++;
				if (charger->pdata->opfq_cnt <= CMD_CNT) {
				queue_delayed_work(charger->wqueue,
					&charger->wpc_opfq_work, msecs_to_jiffies(10000));
					return;
				}
			}
	} else if (pad_mode == ZP3390_PAD_MODE_PMA) {
			charger->pdata->cable_type = ZP3390_PAD_MODE_PMA;
			value.intval = SEC_WIRELESS_PAD_PMA;
			psy_do_property(charger->pdata->wireless_name,
				set, POWER_SUPPLY_PROP_ONLINE, value);
	}

	charger->pdata->opfq_cnt = 0;
	wake_unlock(&charger->wpc_opfq_lock);
}

static void zp3390_wpc_id_request_work(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_id_request_work.work);
	int pending_work = delayed_work_pending(&charger->wpc_auth_check_work);

	pr_info("%s pending_work = %d wpc_id_request_step = %d\n",
		__func__, pending_work, charger->wpc_id_request_step);

	if (charger->wpc_id_request_step > 1) {
		wake_unlock(&charger->wpc_id_request_lock);
		return;
	}

	if (zp3390_is_on_pad(charger) == WPC_ON_PAD) {
		if (!pending_work)
			zp3390_wpc_request_tx_id(charger, 3);
	}

	wake_unlock(&charger->wpc_id_request_lock);

	pending_work = delayed_work_pending(&charger->wpc_auth_check_work);

	pr_info("%s 2nd pending_work = %d wpc_id_request_step = %d\n",
		__func__, pending_work, charger->wpc_id_request_step);

	if (!pending_work &&
		(charger->wpc_id_request_step == 0) &&
		(zp3390_is_on_pad(charger) == WPC_ON_PAD)) {
		charger->wpc_id_request_step++;
		wake_lock_timeout(&charger->wpc_id_request_lock, 15 * HZ);
		queue_delayed_work(charger->wqueue,
			&charger->wpc_id_request_work, msecs_to_jiffies(10000));
	}
}

static void zp3390_wpc_wa_work(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_wakeup_wa_work.work);
	u8 data;
	zp3390_reg_read(charger->client, ZP3390_AP_BATTERY_MODE, &data);

	pr_info("%s ap_mode = %x\n", __func__, data);

	charger->wpc_wakeup_wa = 0;
	zp3390_ap_battery_monitor(charger, WPC_MODE_NORMAL);
	wake_unlock(&charger->wpc_wakeup_wa_lock);
}

static void zp3390_wpc_det_work(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_det_work.work);
	int wc_w_state, prev_wc_state;
	union power_supply_propval value;
	u8 pad_mode;
	u8 vrect;

	wake_lock(&charger->wpc_wake_lock);

	wc_w_state = zp3390_is_on_pad(charger);

	prev_wc_state = charger->wc_w_state;
	charger->wc_w_state = wc_w_state;

	if ((prev_wc_state == WPC_OFF_PAD) && (wc_w_state == WPC_ON_PAD)) {
		charger->pdata->vout_status = ZP3390_VOUT_5V;

		/* set fod value */
		if(charger->pdata->fod_data_check)
			zp3390_fod_set(charger);

		/* enable Mode Change INT */
		zp3390_reg_update(charger->client, ZP3390_INT_ENABLE_L_REG,
						ZP3390_STAT_MODE_CHANGE_MASK, ZP3390_STAT_MODE_CHANGE_MASK);

		/* read vrect adjust */
		zp3390_reg_read(charger->client, ZP3390_VRECT_MARGIN_REG, &vrect);

		pr_info("%s: wpc activated\n",__func__);

		/* read pad mode */
		zp3390_reg_read(charger->client, ZP3390_SYS_OP_MODE_REG, &pad_mode);
		if(pad_mode == ZP3390_SYS_MODE_PMA) {
			charger->pdata->cable_type = ZP3390_PAD_MODE_PMA;
			value.intval = SEC_WIRELESS_PAD_PMA;
			psy_do_property(charger->pdata->wireless_name, set,
					POWER_SUPPLY_PROP_ONLINE, value);
		} else {
			cancel_delayed_work(&charger->wpc_wait_vout_work);
			wake_lock_timeout(&charger->wpc_wait_vout_lock, HZ * 5);
			queue_delayed_work(charger->wqueue,
				&charger->wpc_wait_vout_work,
				msecs_to_jiffies(VOUT_STABLILZATION_TIME));

			wake_lock(&charger->wpc_opfq_lock);
			queue_delayed_work(charger->wqueue,
				&charger->wpc_opfq_work, msecs_to_jiffies(10000));

			/* TX FW 0707 does not send TX ID interrupt somtimes.
			 * If TX ID interrupt not occured, send TX ID request after 3sec.
			 * TX ID interrupt occured after 1 ~ 1.5c after PDETB low.
			 */
			charger->wpc_id_request_step = 0;
			wake_lock_timeout(&charger->wpc_id_request_lock, 5 * HZ);
			queue_delayed_work(charger->wqueue,
				&charger->wpc_id_request_work, msecs_to_jiffies(3000));

			zp3390_wpc_auth_delayed_work(charger, WPC_AUTH_DELAY_TIME);
		}
		charger->pdata->is_charging = 1;
	} else if ((prev_wc_state == WPC_ON_PAD) &&
		(wc_w_state == WPC_OFF_PAD)) {

		charger->pdata->cable_type = ZP3390_PAD_MODE_NONE;
		charger->pdata->is_charging = 0;
		charger->pdata->vout_status = ZP3390_VOUT_0V;
		charger->pdata->opfq_cnt = 0;
		charger->charge_mode = ZP3390_CHARGE_MODE_NONE;
		charger->power_hold_mode = 0;
		charger->support_power_hold = 0;
		charger->wpc_wakeup_wa = 0;
		charger->tx_id = 0x0;

		if (charger->incompatible_tx) {
			charger->incompatible_tx = 0;
			value.intval = 0;
			psy_do_property(charger->pdata->battery_name, set,
				POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);
		}

		value.intval = SEC_WIRELESS_PAD_NONE;
		psy_do_property(charger->pdata->wireless_name, set,
				POWER_SUPPLY_PROP_ONLINE, value);
		pr_info("%s: wpc deactivated, set V_INT as PD\n",__func__);

		cancel_delayed_work(&charger->wpc_isr_work);

		if(delayed_work_pending(&charger->wpc_opfq_work)) {
			wake_unlock(&charger->wpc_opfq_lock);
			cancel_delayed_work(&charger->wpc_opfq_work);
		}

		if(delayed_work_pending(&charger->wpc_wait_vout_work)) {
			wake_unlock(&charger->wpc_wait_vout_lock);
			cancel_delayed_work(&charger->wpc_wait_vout_work);
		}

		if (wake_lock_active(&charger->wpc_auth_check_lock))
			wake_unlock(&charger->wpc_auth_check_lock);

		if (delayed_work_pending(&charger->wpc_auth_check_work))
			cancel_delayed_work(&charger->wpc_auth_check_work);

		if (wake_lock_active(&charger->wpc_id_request_lock))
			wake_unlock(&charger->wpc_id_request_lock);

		if (delayed_work_pending(&charger->wpc_id_request_work))
			cancel_delayed_work(&charger->wpc_id_request_work);

		charger->wpc_id_request_step = 0;
	}

	pr_info("%s: w(%d to %d)\n", __func__, prev_wc_state, wc_w_state);
	wake_unlock(&charger->wpc_wake_lock);
}

static enum alarmtimer_restart zp3390_wpc_ph_mode_alarm(struct alarm *alarm, ktime_t now)
{
	struct zp3390_charger_data *charger = container_of(alarm,
				struct zp3390_charger_data, polling_alarm);

	union power_supply_propval value;

	value.intval = false;
	psy_do_property(charger->pdata->charger_name, set,
		POWER_SUPPLY_PROP_PRESENT, value);
	pr_info("%s: wpc ph mode ends.\n", __func__);
	wake_unlock(&charger->wpc_wake_lock);

	return ALARMTIMER_NORESTART;
}

static int zp3390_compatible_tx_check(struct zp3390_charger_data *charger, int id)
{
	int i;
	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
		pr_info("%s input id = %x table_id %x\n", __func__, id,
				charger->pdata->charger_type[i].compatible_tx_id);
		if (id == charger->pdata->charger_type[i].compatible_tx_id) {
			pr_info("%s id = %x %d\n", __func__, id,
				charger->pdata->charger_type[i].support_power_hold);
			charger->support_power_hold =
				charger->pdata->charger_type[i].support_power_hold;
			return 1;
		}
	}
	charger->support_power_hold = 0;
	return 0;
}

static void zp3390_wpc_request_tx_id(struct zp3390_charger_data *charger, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++) {
		pr_info("%s requst TX-ID to TX (cnt %d)\n", __func__, i);
		zp3390_reg_write(charger->client, ZP3390_PACKET_HEADER, ZP3390_HEADER_AFC_CONF); // using 2 bytes packet property
		zp3390_reg_write(charger->client, ZP3390_RX_DATA_COMMAND, ZP3390_RX_DATA_COM_REQ_TX_ID);
		zp3390_reg_write(charger->client, ZP3390_RX_DATA_VALUE0, 0x0);
		zp3390_reg_write(charger->client, ZP3390_COMMAND_REG, ZP3390_CMD_SEND_RX_DATA);

		if (cnt > 1)
			msleep(300);
	}
}

static void zp3390_curr_measure_work(struct work_struct *work)
{
	struct zp3390_charger_data *charger = container_of(work,
		struct zp3390_charger_data, curr_measure_work.work);
	union power_supply_propval value = {0,};
	int i = 0;
	int acok_cnt = 0;
	bool valid_acok = false;

	pr_info("%s\n", __func__);

	while (i < 100) {
		if (zp3390_is_on_pad(charger) == WPC_OFF_PAD) {
			pr_info("%s pad is off!\n", __func__);
			break;
		}

		if (zp3390_check_chg_in_status(charger) == true) {
			if (acok_cnt > 10) {
				pr_info("%s valid acok. go to measurement mode\n", __func__);
				valid_acok = true;
				break;
			}
			acok_cnt++;
		} else {
			acok_cnt = 0;
		}
		i++;
		msleep(100);
	}

	if (!valid_acok) {
		pr_err("%s acok not stable.\n", __func__);
		return;
	}

	zp3390_write_ap_mode(charger, ZP3390_AP_BATTERY_CURR_MEASURE_MODE);
	charger->curr_measure_mode = 1;

	psy_do_property(charger->pdata->charger_name, set,
		POWER_SUPPLY_EXT_PROP_FORCED_JIG_MODE, value);
}

static void zp3390_wpc_isr_work(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_isr_work.work);

	u8 cmd_data, val_data, data;
	union power_supply_propval value;

	if (charger->wc_w_state == WPC_OFF_PAD) {
		pr_info("%s wc_w_state is 0. exit wpc_isr_work.\n",__func__);
		return;
	}

	if (zp3390_check_chg_in_status(charger) == false) {
		pr_info("%s acok is 0. exit wpc_isr_work.\n",__func__);
		return;
	}

	wake_lock(&charger->wpc_wake_lock);

	zp3390_reg_read(charger->client, ZP3390_TX_DATA_COMMAND, &cmd_data);
	zp3390_reg_read(charger->client, ZP3390_TX_DATA_VALUE0, &val_data);

	pr_info("%s cmd(%x) val(%x)\n", __func__, cmd_data, val_data);
	charger->tx_id = val_data;

	charger->tx_id_checked = 0;

	if (cmd_data == ZP3390_TX_DATA_COM_TX_ID) {
		if (zp3390_compatible_tx_check(charger, val_data)) {
			pr_info("%s data = 0x%x, compatible TX pad\n", __func__, val_data);
			value.intval = 1;
			psy_do_property(charger->pdata->charger_name, set,
				POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);
			psy_do_property(charger->pdata->battery_name, set,
				POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);

			if (charger->support_power_hold)
				charger->charger_type = ZP3390_CHARGER_TYPE_COMPATIBLE;
			else
				charger->charger_type = ZP3390_CHARGER_TYPE_MULTI;

			value.intval = 0;
			charger->incompatible_tx = 0;
			psy_do_property(charger->pdata->battery_name, set,
				POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);

			charger->tx_id_checked = 1;
		} else if (val_data == ZP3390_FACTORY_MODE_TX_ID) {
			pr_info("%s id 0x%x current measure TX pad\n", __func__, val_data);
			charger->charger_type = ZP3390_CHARGER_TYPE_COMPATIBLE;
			schedule_delayed_work(&charger->curr_measure_work,
				msecs_to_jiffies(2000));
		} else {
			charger->charger_type = ZP3390_CHARGER_TYPE_INCOMPATIBLE;
			pr_info("%s incompatible TX pad\n", __func__);
		}

		zp3390_reg_read(charger->client, ZP3390_INT_L_REG, &data);
		if (data & ZP3390_STAT_TX_DATA_RECEIVED_MASK) {
			zp3390_reg_write(charger->client, ZP3390_INT_CLEAR_L_REG,
				ZP3390_INT_TX_DATA_RECEIVED);
		}

	} else if (cmd_data == 0x19) {
		pr_info("%s incompatible TX pad\n", __func__);
		charger->charger_type = ZP3390_CHARGER_TYPE_INCOMPATIBLE;

		zp3390_reg_read(charger->client, ZP3390_INT_L_REG, &data);
		if (data & ZP3390_STAT_TX_DATA_RECEIVED_MASK) {
			zp3390_reg_write(charger->client, ZP3390_INT_CLEAR_L_REG,
				ZP3390_INT_TX_DATA_RECEIVED);
		}
	}

	zp3390_wpc_request_tx_id(charger, 3);
	wake_unlock(&charger->wpc_wake_lock);

	zp3390_wpc_auth_delayed_work(charger, WPC_AUTH_DELAY_TIME);
}

static irqreturn_t zp3390_wpc_det_irq_thread(int irq, void *irq_data)
{
	struct zp3390_charger_data *charger = irq_data;

	pr_info("%s !\n",__func__);

	while(charger->wqueue == NULL) {
		pr_err("%s wait wqueue created!!\n", __func__);
		msleep(10);
	}

	queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);

	return IRQ_HANDLED;
}

static void zp3390_enable_vbat_monitoring(struct zp3390_charger_data *charger)
{
	if (charger->pdata->can_vbat_monitoring) {
		zp3390_reg_write(charger->client, ZP3390_VRECT_MARGIN_REG, 9);
		zp3390_reg_update(charger->client, ZP3390_WPC_FLAG_REG,
			ZP3390_VBAT_MONITORING_MODE, ZP3390_VBAT_MONITORING_MODE_MASK);
	}
}

static irqreturn_t zp3390_wpc_irq_thread(int irq, void *irq_data)
{
	struct zp3390_charger_data *charger = irq_data;
	int ret;
	u8 irq_src[2];
	u8 reg_data;

	wake_lock(&charger->wpc_wake_lock);
	mutex_lock(&charger->charger_lock);

	pr_info("%s\n", __func__);

	/* check again firmare version support vbat monitoring */
	if (!charger->pdata->otp_firmware_ver) {
		ret = zp3390_get_firmware_version(charger, ZP3390_RX_FIRMWARE);
		if (ret > 0) {
			pr_debug("%s rx major firmware version 0x%x\n", __func__, ret);
			charger->pdata->otp_firmware_ver = ret;
		}
	}

	zp3390_enable_vbat_monitoring(charger);

	ret = zp3390_reg_read(charger->client, ZP3390_INT_L_REG, &irq_src[0]);
	ret = zp3390_reg_read(charger->client, ZP3390_INT_H_REG, &irq_src[1]);

	if (ret < 0) {
		pr_err("%s: Failed to read interrupt source: %d\n",
			__func__, ret);
		wake_unlock(&charger->wpc_wake_lock);
		mutex_unlock(&charger->charger_lock);
		return IRQ_NONE;
	}

	pr_info("%s: interrupt source(0x%x)\n", __func__, irq_src[1] << 8 | irq_src[0]);

	if(irq_src[0] & ZP3390_STAT_MODE_CHANGE_MASK) {
		pr_info("%s MODE CHANGE IRQ ! \n", __func__);
		ret = zp3390_reg_read(charger->client, ZP3390_SYS_OP_MODE_REG, &reg_data);
	}

	if(irq_src[0] & ZP3390_STAT_VOUT_MASK) {
		pr_info("%s Vout IRQ ! \n", __func__);
		ret = zp3390_reg_read(charger->client, ZP3390_INT_STATUS_L_REG, &reg_data);
		if (reg_data & ZP3390_INT_STAT_VOUT)
			charger->vout_status = 1;
		else
			charger->vout_status = 0;
		pr_info("%s vout_status = %d\n", __func__, charger->vout_status);
	}

	if(irq_src[0] & ZP3390_STAT_TX_DATA_RECEIVED_MASK) {
		pr_info("%s TX RECIVED IRQ ! \n", __func__);
		if(!delayed_work_pending(&charger->wpc_isr_work) &&
			charger->pdata->cable_type != ZP3390_PAD_MODE_WPC_AFC )
			schedule_delayed_work(&charger->wpc_isr_work, msecs_to_jiffies(1000));
	}

	if(irq_src[1] & ZP3390_STAT_OVER_CURR_MASK) {
		pr_info("%s OVER CURRENT IRQ ! \n", __func__);
	}

	if(irq_src[1] & ZP3390_STAT_OVER_TEMP_MASK) {
		pr_info("%s OVER TEMP IRQ ! \n", __func__);
	}

	if(irq_src[1] & ZP3390_STAT_TX_CONNECT_MASK) {
		pr_info("%s TX CONNECT IRQ ! \n", __func__);
		charger->pdata->tx_status = SEC_TX_POWER_TRANSFER;
	}

	if ((irq_src[1] << 8 | irq_src[0]) != 0) {
		msleep(5);

		/* clear intterupt */
		zp3390_reg_write(charger->client, ZP3390_INT_CLEAR_L_REG, irq_src[0]);
		zp3390_reg_write(charger->client, ZP3390_INT_CLEAR_H_REG, irq_src[1]);
		zp3390_set_cmd_reg(charger, 0x20, ZP3390_CMD_CLEAR_INT_MASK); // command
	}
	/* debug */
	wake_unlock(&charger->wpc_wake_lock);
	mutex_unlock(&charger->charger_lock);

	return IRQ_HANDLED;
}

enum {
	WPC_FW_VER = 0,
	WPC_IBUCK,
	WPC_WATCHDOG,
	WPC_ADDR,
	WPC_SIZE,
	WPC_DATA,
};


static struct device_attribute zp3390_attributes[] = {
	SEC_WPC_ATTR(fw),
	SEC_WPC_ATTR(ibuck),
	SEC_WPC_ATTR(watchdog_test),
	SEC_WPC_ATTR(addr),
	SEC_WPC_ATTR(size),
	SEC_WPC_ATTR(data),
};

ssize_t sec_wpc_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct zp3390_charger_data *charger =
		power_supply_get_drvdata(psy);

	const ptrdiff_t offset = attr - zp3390_attributes;
	u8 data;
	int i, count = 0;

	switch (offset) {
		case WPC_FW_VER:
			{
				int ret =0;
				int version =0;

				ret = zp3390_get_firmware_version(charger, ZP3390_RX_FIRMWARE);

				if (ret >= 0) {
					version =  ret;
				}
				pr_info("%s rx major firmware version 0x%x \n", __func__, version);

				count = sprintf(buf, "%x\n", version);
			}
			break;
		case WPC_IBUCK:
			{
				int ibuck =0;
				ibuck = zp3390_get_adc(charger, ZP3390_ADC_RX_IOUT);
				ibuck -= ZP3390_I_OUT_OFFSET;
				pr_info("%s raw iout %dmA\n", __func__, ibuck);
				count = sprintf(buf, "%d\n", ibuck);
			}
			break;
		case WPC_WATCHDOG:
			pr_info("%s: watchdog test [%d] \n", __func__, charger->pdata->watchdog_test);
			count = sprintf(buf, "%d\n", charger->pdata->watchdog_test);
			break;
		case WPC_ADDR:
			count = sprintf(buf, "0x%x\n", charger->addr);
			break;
		case WPC_SIZE:
			count = sprintf(buf, "0x%x\n", charger->size);
			break;
		case WPC_DATA:
			if (charger->size == 0)
				charger->size = 1;

			for (i = 0; i < charger->size; i++) {
				if (zp3390_reg_read(charger->client, charger->addr+i, &data) < 0) {
					dev_info(charger->dev,
							"%s: read fail\n", __func__);
					count += sprintf(buf+count, "addr: 0x%x read fail\n", charger->addr+i);
					continue;
				}
				count += sprintf(buf+count, "addr: 0x%x, data: 0x%x\n", charger->addr+i,data);
			}
			break;
		default:
			break;
		}
	return count;
}

ssize_t sec_wpc_store_attrs(
					struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct zp3390_charger_data *charger =
		power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - zp3390_attributes;
	int x, ret = -EINVAL;;

	switch (offset) {
		case WPC_WATCHDOG:
			sscanf(buf, "%d\n", &x);
			if (x == 1)
				charger->pdata->watchdog_test = true;
			else if (x == 0)
				charger->pdata->watchdog_test = false;
			else
				pr_debug("%s: non valid input\n", __func__);
			pr_info("%s: watchdog test is set to %d\n", __func__, charger->pdata->watchdog_test);
			ret = count;
			break;
		case WPC_ADDR:
			if (sscanf(buf, "0x%x\n", &x) == 1) {
				charger->addr = x;
			}
			ret = count;
			break;
		case WPC_SIZE:
			if (sscanf(buf, "%d\n", &x) == 1) {
				charger->size = x;
			}
			ret = count;
			break;
		case WPC_DATA:
			if (sscanf(buf, "0x%x", &x) == 1) {
				u8 data = x;
				if (zp3390_reg_write(charger->client, charger->addr, data) < 0)
				{
					dev_info(charger->dev,
							"%s: addr: 0x%x write fail\n", __func__, charger->addr);
				}
			}
			ret = count;
			break;
		default:
			break;
	}
	return ret;
}

static int sec_wpc_create_attrs(struct device *dev)
{
	unsigned long i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(zp3390_attributes); i++) {
		rc = device_create_file(dev, &zp3390_attributes[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	while (i--)
		device_remove_file(dev, &zp3390_attributes[i]);
create_attrs_succeed:
	return rc;
}

static void zp3390_wpc_auth_check_work(struct work_struct *work)
{
	struct zp3390_charger_data *charger =
		container_of(work, struct zp3390_charger_data, wpc_auth_check_work.work);

	union power_supply_propval value;
	u8 cmd_data, val_data;

	pr_info("%s wc_w_state=%d\n", __func__, charger->wc_w_state);

	if (charger->wc_w_state == WPC_OFF_PAD) {
		wake_unlock(&charger->wpc_auth_check_lock);
		return;
	}

	if (zp3390_check_chg_in_status(charger) == false) {
		pr_err("%s acok is not valid!!\n", __func__);
		goto out;
	}

	if (charger->tx_id_checked == 0) {
		zp3390_reg_read(charger->client, ZP3390_TX_DATA_COMMAND, &cmd_data);
		zp3390_reg_read(charger->client, ZP3390_TX_DATA_VALUE0, &val_data);
		charger->tx_id = val_data;

		pr_info("%s cmd_data = %x val_data = %x\n", __func__, cmd_data, val_data);

		if (cmd_data == ZP3390_TX_DATA_COM_TX_ID) {
			if (zp3390_compatible_tx_check(charger, val_data)) {
				pr_info("%s data = 0x%x, compatible TX pad\n", __func__, val_data);
				value.intval = 1;
				psy_do_property(charger->pdata->charger_name, set,
					POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);
				psy_do_property(charger->pdata->battery_name, set,
					POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);

				if (charger->support_power_hold)
					charger->charger_type = ZP3390_CHARGER_TYPE_COMPATIBLE;
				else
					charger->charger_type = ZP3390_CHARGER_TYPE_MULTI;

				value.intval = 0;
				charger->incompatible_tx = 0;
				psy_do_property(charger->pdata->battery_name, set,
					POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);
			} else {
				value.intval = 1;
				charger->incompatible_tx = 1;
				psy_do_property(charger->pdata->battery_name, set,
					POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);
				charger->charger_type = ZP3390_CHARGER_TYPE_INCOMPATIBLE;
			}
		} else {
			value.intval = 1;
			charger->incompatible_tx = 1;
			psy_do_property(charger->pdata->battery_name, set,
				POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);
			charger->charger_type = ZP3390_CHARGER_TYPE_INCOMPATIBLE;
		}
	}


	charger->tx_id_checked = 0;
	zp3390_wpc_request_tx_id(charger, 3);
out:
	if (zp3390_compatible_tx_check(charger, charger->tx_id) == 0) {
		value.intval = 1;
		charger->incompatible_tx = 1;
		psy_do_property(charger->pdata->battery_name, set,
			POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);
		charger->charger_type = ZP3390_CHARGER_TYPE_INCOMPATIBLE;
		if (charger->tx_id == 0)
			charger->tx_id = UNKNOWN_TX_ID;
	}
	wake_unlock(&charger->wpc_auth_check_lock);
}

static int
zp3390_wpc_auth_delayed_work(struct zp3390_charger_data *charger, int sec)
{
	int delay = sec * 1000;
	int lock_timeout = sec + (sec / 2);

	pr_info("%s delay=%d lock_timeout = %d\n", __func__, delay, lock_timeout);

	cancel_delayed_work(&charger->wpc_auth_check_work);

	if (!wake_lock_active(&charger->wpc_auth_check_lock))
		wake_lock_timeout(&charger->wpc_auth_check_lock, lock_timeout * HZ);

	schedule_delayed_work(&charger->wpc_auth_check_work,
		msecs_to_jiffies(delay));
	return 0;
}

static struct zp3390_chg_type_desc charger_type_compatilbe = {
	.charger_type = ZP3390_CHARGER_TYPE_COMPATIBLE,
	.charger_type_name = "compatible",
	.cc_chg = zp3390_compatible_cc_charge,
	.cv_chg = zp3390_compatible_cv_charge,
	.full_chg = zp3390_power_hold_full_charge,
	.re_chg = zp3390_power_hold_re_charge,
};

static struct zp3390_chg_type_desc charger_type_incompatilbe = {
	.charger_type = ZP3390_CHARGER_TYPE_INCOMPATIBLE,
	.charger_type_name = "incompatible",
	.cc_chg = zp3390_incompatible_cc_charge,
	.cv_chg = zp3390_incompatible_cv_charge,
	.full_chg = zp3390_normal_full_charge,
	.re_chg = zp3390_normal_re_charge,
};

static struct zp3390_chg_type_desc charger_type_multi = {
	.charger_type = ZP3390_CHARGER_TYPE_MULTI,
	.charger_type_name = "multi",
	.cc_chg = zp3390_compatible_cc_charge,
	.cv_chg = zp3390_compatible_cv_charge,
	.full_chg = zp3390_normal_full_charge,
	.re_chg = zp3390_normal_re_charge,
};

static int zp3390_wpc_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
	struct zp3390_charger_data *charger =
		container_of(nb, struct zp3390_charger_data, wpc_nb);

	pr_info("%s action=%lu, attached_dev=%d\n", __func__, action, attached_dev);

	switch (attached_dev) {
	case ATTACHED_DEV_TA_MUIC:
		if (action == MUIC_NOTIFY_CMD_ATTACH) {
			zp3390_reg_update(charger->client,
				ZP3390_WPC_FLAG_REG, 0x0, ZP3390_WATCHDOG_DIS_MASK);
			zp3390_get_firmware_version(charger, ZP3390_RX_FIRMWARE);
			zp3390_get_firmware_version(charger, ZP3390_TX_FIRMWARE);
		}
		break;
	case ATTACHED_DEV_WIRELESS_TA_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH) {
			pr_info("%s MUIC_NOTIFY_CMD_DETACH\n", __func__);
			charger->is_recharge = 0;
			schedule_delayed_work(&charger->wpc_detach_chk_work,
				msecs_to_jiffies(400));
		} else if (action == MUIC_NOTIFY_CMD_ATTACH) {
			zp3390_enable_vbat_monitoring(charger);
		}
		break;
	case ATTACHED_DEV_UNDEFINED_CHARGING_MUIC:
		if (action == MUIC_NOTIFY_CMD_ATTACH) {
			if (zp3390_is_on_pad(charger) == WPC_ON_PAD) {
				if (charger->power_hold_mode == 1) {
					wake_lock(&charger->power_hold_chk_lock);
					schedule_delayed_work(&charger->power_hold_chk_work,
						msecs_to_jiffies(3000));
				}
				zp3390_enable_vbat_monitoring(charger);
			}
		} else if (action == MUIC_NOTIFY_CMD_DETACH) {
			pr_info("%s MUIC_NOTIFY_CMD_DETACH\n", __func__);
			charger->is_recharge = 0;
			schedule_delayed_work(&charger->wpc_detach_chk_work,
				msecs_to_jiffies(400));
		}
		break;
	default:
		break;
	}

	return 0;
}

static int zp3390_parse_dt(struct device *dev, zp3390_charger_platform_data_t *pdata)
{

	struct device_node *np = dev->of_node;
	const u32 *p;
	int len, ret, i;
	enum of_gpio_flags irq_gpio_flags;
	/*changes can be added later, when needed*/

	ret = of_property_read_string(np,
		"zp3390,charger_name", (char const **)&pdata->charger_name);
	if (ret) {
		pr_info("%s: Charger name is Empty\n", __func__);
		pdata->charger_name = "sec-charger";
	}

	ret = of_property_read_string(np,
		"zp3390,fuelgauge_name", (char const **)&pdata->fuelgauge_name);
	if (ret) {
		pr_info("%s: Fuelgauge name is Empty\n", __func__);
		pdata->fuelgauge_name = "sec-fuelgauge";
	}

	ret = of_property_read_string(np,
		"zp3390,battery_name", (char const **)&pdata->battery_name);
	if (ret) {
		pr_info("%s: battery_name is Empty\n", __func__);
		pdata->battery_name = "battery";
	}

	ret = of_property_read_string(np,
		"zp3390,wireless_name", (char const **)&pdata->wireless_name);
	if (ret) {
		pr_info("%s: wireless_name is Empty\n", __func__);
		pdata->wireless_name = "wireless";
	}

	ret = of_property_read_string(np, "zp3390,wireless_charger_name",
		(char const **)&pdata->wireless_charger_name);
	if (ret) {
		pr_info("%s: wireless_charger_name is Empty\n", __func__);
		pdata->wireless_charger_name = "wpc";
	}

	/* wpc_det */
	ret = pdata->wpc_det = of_get_named_gpio_flags(np, "zp3390,wpc_det",
			0, &irq_gpio_flags);
	if (ret < 0) {
		dev_err(dev, "%s : can't get wpc_det\r\n", __func__);
	} else {
		if (gpio_is_valid(pdata->wpc_det)) {
		    ret = gpio_request(pdata->wpc_det, "wpc_det");
		    if (ret) {
		        dev_err(dev, "%s : can't get wpc_det\r\n", __func__);
		    } else {
		    	ret = gpio_direction_input(pdata->wpc_det);
				if (ret) {
					dev_err(dev, "%s : wpc_det set input fail\r\n", __func__);
				}
				pdata->irq_wpc_det = gpio_to_irq(pdata->wpc_det);
				pr_info("%s wpc_det = 0x%x, irq_wpc_det = 0x%x (%d)\n", __func__,
					pdata->wpc_det, pdata->irq_wpc_det, pdata->irq_wpc_det);
		    }
		}
	}

	/* wpc_int */
	ret = pdata->wpc_int = of_get_named_gpio_flags(np, "zp3390,wpc_int",
			0, &irq_gpio_flags);
	if (ret < 0) {
		dev_err(dev, "%s : can't wpc_int\r\n", __FUNCTION__);
	} else {
		pdata->irq_wpc_int = gpio_to_irq(pdata->wpc_int);
		pr_info("%s wpc_int = 0x%x, irq_wpc_int = 0x%x \n",__func__,
			pdata->wpc_int, pdata->irq_wpc_int);
	}

	/* wpc_en */
	ret = pdata->wpc_en = of_get_named_gpio_flags(np, "zp3390,wpc_en",
			0, &irq_gpio_flags);
	if (ret < 0) {
		dev_err(dev, "%s : can't wpc_en\r\n", __FUNCTION__);
	} else {
		if (gpio_is_valid(pdata->wpc_en)) {
		    ret = gpio_request(pdata->wpc_en, "wpc_en");
		    if (ret) {
		        dev_err(dev, "%s : can't get wpc_en\r\n", __func__);
		    } else {
		    	gpio_direction_output(pdata->wpc_en, 0);
		    }
		}
	}

	ret = of_property_read_u32(np, "zp3390,cc_cv_threshold",
		&pdata->cc_cv_threshold);
	if (ret < 0) {
		pr_err("%s error reading cc_cv_threshold %d\n", __func__, ret);
		pdata->cc_cv_threshold = 88;
	}

	pdata->can_vbat_monitoring = of_property_read_bool(np, "zp3390,vbat-monitoring");

	p = of_get_property(np, "zp3390,charger_type", &len);
	if (p) {
		pdata->num_compatible_tx = len / sizeof(struct zp3390_charger_type);
		pdata->charger_type = kzalloc(len, GFP_KERNEL);

		ret = of_property_read_u32_array(np, "zp3390,charger_type",
				 (u32 *)pdata->charger_type, len/sizeof(u32));
		if (ret) {
			pr_err("%s: failed to read zp3390,charger_type: %d\n", __func__, ret);
			kfree(pdata->charger_type);
			pdata->charger_type = NULL;
			pdata->num_compatible_tx = 0;
		} else {
			for (i = 0; i < pdata->num_compatible_tx; i++)
				pr_info("charger_type tx_id = 0x%02x power_hold = %d\n",
					pdata->charger_type[i].compatible_tx_id,
					pdata->charger_type[i].support_power_hold);
		}
	}

	p = of_get_property(np, "zp3390,sec_mode_data", &len);
	if (p) {
		pdata->num_sec_mode = len / sizeof(struct zp3390_sec_mode_config_data);
		pdata->sec_mode_config_data = kzalloc(len, GFP_KERNEL);
		ret = of_property_read_u32_array(np, "zp3390,sec_mode_data",
				 (u32 *)pdata->sec_mode_config_data, len/sizeof(u32));
		if (ret) {
			pr_err("%s: failed to read zp3390,sec_mode_data: %d\n", __func__, ret);
			kfree(pdata->sec_mode_config_data);
			pdata->sec_mode_config_data = NULL;
			pdata->num_sec_mode = 0;
		}
		pr_err("%s: num_sec_mode : %d\n", __func__, pdata->num_sec_mode);
		for (len = 0; len < pdata->num_sec_mode; ++len)
			pr_err("mode %d : vrect:%d, vout:%d\n",
				len, pdata->sec_mode_config_data[len].vrect, pdata->sec_mode_config_data[len].vout);
	} else
		pr_err("%s: there is no zp3390,sec_mode_data\n", __func__);

	ret = of_property_read_u32(np, "zp3390,tx-off-high-temp",
		&pdata->tx_off_high_temp);
	if (ret) {
		pr_info("%s : TX-OFF-TEMP is Empty\n", __func__);
		pdata->tx_off_high_temp = INT_MAX;
	}

	ret = of_property_read_u32(np, "zp3390,ping_duration",
		&pdata->ping_duration);
	if (ret) {
		pr_info("%s : ping_duration Empty\n", __func__);
		pdata->ping_duration = 350;
	}

	return 0;
}

static const struct power_supply_desc wpc_power_supply_desc = {
	.name = "wpc",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = sec_charger_props,
	.num_properties = ARRAY_SIZE(sec_charger_props),
	.get_property = zp3390_chg_get_property,
	.set_property = zp3390_chg_set_property,
};

static int zp3390_charger_probe(
						struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct zp3390_charger_data *charger;
	zp3390_charger_platform_data_t *pdata = client->dev.platform_data;
	struct power_supply_config wpc_cfg = {};
	int ret = 0;

	dev_info(&client->dev,
		"%s: zp3390 Charger Driver Loading\n", __func__);

	pdata = devm_kzalloc(&client->dev, sizeof(zp3390_charger_platform_data_t), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	ret = zp3390_parse_dt(&client->dev, pdata);
	if (ret < 0)
		return ret;

	client->irq = pdata->irq_wpc_int;
	pdata->hw_rev_changed = 1;
	pdata->on_mst_wa=1;
	pdata->wpc_cv_call_vout = 5600;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (charger == NULL) {
		dev_err(&client->dev, "Memory is not enough.\n");
		ret = -ENOMEM;
		goto err_wpc_nomem;
	}
	charger->dev = &client->dev;

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);
	if (!ret) {
		ret = i2c_get_functionality(client->adapter);
		dev_err(charger->dev, "I2C functionality is not supported.\n");
		ret = -ENOSYS;
		goto err_i2cfunc_not_support;
	}
	charger->client = client;
	charger->pdata = pdata;

	i2c_set_clientdata(client, charger);

	charger->pdata->ic_on_mode = false;
	charger->pdata->cable_type = ZP3390_PAD_MODE_NONE;
	charger->pdata->is_charging = 0;

	charger->pdata->otp_firmware_result = ZP3390_FW_RESULT_DOWNLOADING;
	charger->pdata->tx_firmware_result = ZP3390_FW_RESULT_DOWNLOADING;
	charger->pdata->tx_status = 0;
	charger->pdata->cs100_status = 0;
	charger->pdata->vout_status = ZP3390_VOUT_0V;
	charger->pdata->opfq_cnt = 0;
	charger->pdata->watchdog_test = false;
	charger->pdata->charging_mode = SEC_INITIAL_MODE;
	charger->wc_w_state = zp3390_is_on_pad(charger);

	charger->power_hold_mode = 0;
	charger->support_power_hold = 0;
	charger->curr_measure_mode = 0;
	charger->charge_mode = ZP3390_CHARGE_MODE_NONE;
	charger->store_mode = 0;
	charger->wpc_id_request_step = 0;
	charger->tx_id_checked = 0;
	charger->tx_id = 0x0;

	mutex_init(&charger->io_lock);
	mutex_init(&charger->charger_lock);

	wake_lock_init(&charger->wpc_wake_lock, WAKE_LOCK_SUSPEND,
			"wpc_wakelock");
	wake_lock_init(&charger->wpc_vbat_monitoring_enable_lock, WAKE_LOCK_SUSPEND,
			"wpc_vbat_monitoring_enable_lock");
	wake_lock_init(&charger->wpc_auth_check_lock, WAKE_LOCK_SUSPEND,
			"wpc_auth_check_lock");

	charger->temperature = 250;

	/* wpc_det */
	if (charger->pdata->irq_wpc_det >= 0) {
		INIT_DELAYED_WORK(&charger->wpc_det_work, zp3390_wpc_det_work);
		INIT_DELAYED_WORK(&charger->wpc_opfq_work, zp3390_wpc_opfq_work);

		ret = request_threaded_irq(charger->pdata->irq_wpc_det,
				NULL, zp3390_wpc_det_irq_thread,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |
				IRQF_ONESHOT,
				"wpd-det-irq", charger);
		if (ret) {
			dev_err(&client->dev,
				"%s: Failed to Reqeust IRQ\n", __func__);
			goto err_supply_unreg;
		}

		ret = enable_irq_wake(charger->pdata->irq_wpc_det);
		if (ret < 0)
			dev_err(&client->dev, "%s: Failed to Enable Wakeup Source(%d)\n",
				__func__, ret);
	}

	/* wpc_irq */
	if (charger->client->irq) {
		zp3390_reg_update(charger->client, ZP3390_INT_ENABLE_L_REG,
			ZP3390_INT_STAT_VOUT, ZP3390_STAT_VOUT_MASK);

		INIT_DELAYED_WORK(&charger->wpc_isr_work, zp3390_wpc_isr_work);
		ret = gpio_request(pdata->wpc_int, "wpc-irq");
		if (ret) {
			pr_err("%s failed requesting gpio(%d)\n",
				__func__, pdata->wpc_int);
			goto err_supply_unreg;
		}
		ret = request_threaded_irq(charger->client->irq,
				NULL, zp3390_wpc_irq_thread,
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
				"wpc-irq", charger);
		if (ret) {
			dev_err(&client->dev,
				"%s: Failed to Reqeust IRQ\n", __func__);
			goto err_supply_unreg;
		}
	}
	INIT_DELAYED_WORK(&charger->curr_measure_work, zp3390_curr_measure_work);
	INIT_DELAYED_WORK(&charger->wpc_wait_vout_work, zp3390_wpc_wait_vout);
	INIT_DELAYED_WORK(&charger->wpc_init_work, zp3390_wpc_init);
	schedule_delayed_work(&charger->wpc_init_work, msecs_to_jiffies(1000));

	INIT_DELAYED_WORK(&charger->wpc_auth_check_work, zp3390_wpc_auth_check_work);
	INIT_DELAYED_WORK(&charger->wpc_detach_chk_work, zp3390_wpc_detach_check);
	INIT_DELAYED_WORK(&charger->power_hold_chk_work, zp3390_wpc_power_hold_check);
	INIT_DELAYED_WORK(&charger->wpc_id_request_work, zp3390_wpc_id_request_work);

	wpc_cfg.drv_data = charger;
	if (!wpc_cfg.drv_data) {
		dev_err(&client->dev,
			"%s: Failed to Register psy_chg\n", __func__);
		goto err_supply_unreg;
	}

	charger->psy_chg = power_supply_register(&client->dev,
		&wpc_power_supply_desc, &wpc_cfg);
	if (IS_ERR(charger->psy_chg)) {
		goto err_supply_unreg;
	}

	ret = sec_wpc_create_attrs(&charger->psy_chg->dev);
	if (ret) {
		dev_err(&client->dev,
			"%s: Failed to Register psy_chg\n", __func__);
		goto err_pdata_free;
	}


	charger->wqueue = create_singlethread_workqueue("zp3390_workqueue");
	if (!charger->wqueue) {
		pr_err("%s: Fail to Create Workqueue\n", __func__);
		goto err_pdata_free;
	}

	wake_lock_init(&charger->wpc_opfq_lock, WAKE_LOCK_SUSPEND,
		"wpc_opfq_lock");

	wake_lock_init(&charger->wpc_wait_vout_lock, WAKE_LOCK_SUSPEND,
		"wpc_wait_vout_lock");

	wake_lock_init(&charger->power_hold_chk_lock, WAKE_LOCK_SUSPEND,
		"power_hold_chk_lock");

	wake_lock_init(&charger->wpc_id_request_lock, WAKE_LOCK_SUSPEND,
		"wpc_id_request_lock");

	wake_lock_init(&charger->wpc_wakeup_wa_lock, WAKE_LOCK_SUSPEND,
		"wpc_wakeup_wa_lock");

	INIT_DELAYED_WORK(&charger->wpc_wakeup_wa_work, zp3390_wpc_wa_work);
	charger->wpc_wakeup_wa = 0;

	charger->last_poll_time = ktime_get_boottime();
	alarm_init(&charger->polling_alarm, ALARM_BOOTTIME,
		zp3390_wpc_ph_mode_alarm);

	charger->charge_cb_func[ZP3390_CHARGER_TYPE_COMPATIBLE] = &charger_type_compatilbe;
	charger->charge_cb_func[ZP3390_CHARGER_TYPE_INCOMPATIBLE] = &charger_type_incompatilbe;
	charger->charge_cb_func[ZP3390_CHARGER_TYPE_MULTI] = &charger_type_multi;

#if defined(CONFIG_MUIC_NOTIFIER)
	muic_notifier_register(&charger->wpc_nb, zp3390_wpc_handle_notification,
			       MUIC_NOTIFY_DEV_CHARGER);
#endif

	dev_info(&client->dev,
		"%s: zp3390 Charger Driver Loaded\n", __func__);

	//device_init_wakeup(charger->dev, 1);
	return 0;
err_pdata_free:
	power_supply_unregister(charger->psy_chg);
err_supply_unreg:
	wake_lock_destroy(&charger->wpc_wake_lock);
	wake_lock_destroy(&charger->wpc_vbat_monitoring_enable_lock);
	wake_lock_destroy(&charger->wpc_auth_check_lock);
	mutex_destroy(&charger->io_lock);
	mutex_destroy(&charger->charger_lock);
err_i2cfunc_not_support:
//irq_base_err:
	kfree(charger);
err_wpc_nomem:
	devm_kfree(&client->dev, pdata);
	return ret;
}

#if defined CONFIG_PM
static int zp3390_charger_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct zp3390_charger_data *charger = i2c_get_clientdata(i2c);

	pr_info("%s: mode[%d]\n", __func__, charger->pdata->charging_mode);

	if(!charger->pdata->hw_rev_changed) { /* this code is only temporary with non int_ext wpc_det */
		if (device_may_wakeup(charger->dev)){
			enable_irq_wake(charger->pdata->irq_wpc_int);
		}
		disable_irq(charger->pdata->irq_wpc_int);
	} else { /* this is for proper board */
		if (device_may_wakeup(charger->dev)){
			enable_irq_wake(charger->pdata->irq_wpc_int);
			enable_irq_wake(charger->pdata->irq_wpc_det);
		}
		disable_irq(charger->pdata->irq_wpc_int);
		disable_irq(charger->pdata->irq_wpc_det);
	}
	return 0;
}

static int zp3390_charger_resume(struct device *dev)
{
	int wc_w_state;
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct zp3390_charger_data *charger = i2c_get_clientdata(i2c);

	pr_info("%s: mode[%d] charger->charge_mode %d charger->pdata->otp_firmware_ver = %x\n",
		__func__, charger->pdata->charging_mode, charger->charge_mode,
		charger->pdata->otp_firmware_ver);

	charger->wpc_wakeup_wa = 0;

	if(!charger->pdata->hw_rev_changed) { /* this code is only temporary with non int_ext wpc_det */
		wc_w_state = gpio_get_value(charger->pdata->wpc_det);

		if(charger->wc_w_state != wc_w_state)
			queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
		if (device_may_wakeup(charger->dev)) {
			disable_irq_wake(charger->pdata->irq_wpc_int);
		}
		enable_irq(charger->pdata->irq_wpc_int);
	} else { /* this is for proper board */
		if (device_may_wakeup(charger->dev)) {
			disable_irq_wake(charger->pdata->irq_wpc_int);
			disable_irq_wake(charger->pdata->irq_wpc_det);
		}
		enable_irq(charger->pdata->irq_wpc_int);
		enable_irq(charger->pdata->irq_wpc_det);
	}

	if ((zp3390_is_on_pad(charger) == WPC_ON_PAD) &&
		(charger->power_hold_mode == 0)) {
		if (charger->charge_mode == ZP3390_CHARGE_MODE_CV) {
			if (charger->pdata->otp_firmware_ver == 0x125) {
				wake_lock(&charger->wpc_wakeup_wa_lock);
				charger->wpc_wakeup_wa = 1;
				zp3390_ap_battery_monitor(charger, WPC_MODE_BOOT);
				schedule_delayed_work(&charger->wpc_wakeup_wa_work, msecs_to_jiffies(1000));
			} else {
				pr_info("%s set wake-up mode(0x12)\n", __func__);
				zp3390_ap_battery_monitor(charger, WPC_MODE_WAKEUP);
			}
		}
	}
	return 0;
}
#else
#define zp3390_charger_suspend NULL
#define zp3390_charger_resume NULL
#endif

static void zp3390_charger_shutdown(struct i2c_client *client)
{
	struct zp3390_charger_data *charger = i2c_get_clientdata(client);

	pr_debug("%s \n", __func__);

	if (zp3390_is_on_pad(charger) == WPC_ON_PAD) {
		if (charger->power_hold_mode) {
			pr_info("%s exit power hold mode before poweroff\n", __func__);
			zp3390_re_charge(charger);
		}

		zp3390_ap_battery_monitor(charger, WPC_MODE_IDT);
	}
}


static const struct i2c_device_id zp3390_charger_id[] = {
	{"zp3390-charger", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, zp3390_charger_id);

static struct of_device_id zp3390_i2c_match_table[] = {
	{ .compatible = "zp3390,i2c"},
	{},
};

const struct dev_pm_ops zp3390_charger_pm = {
	.suspend = zp3390_charger_suspend,
	.resume = zp3390_charger_resume,
};

static struct i2c_driver zp3390_charger_driver = {
	.driver = {
		.name	= "zp3390-charger",
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm	= &zp3390_charger_pm,
#endif /* CONFIG_PM */
		.of_match_table = zp3390_i2c_match_table,
	},
	.shutdown	= zp3390_charger_shutdown,
	.probe	= zp3390_charger_probe,
	//.remove	= zp3390_charger_remove,
	.id_table	= zp3390_charger_id,
};

static int __init zp3390_charger_init(void)
{
	pr_debug("%s \n",__func__);
	return i2c_add_driver(&zp3390_charger_driver);
}

static void __exit zp3390_charger_exit(void)
{
	pr_debug("%s \n",__func__);
	i2c_del_driver(&zp3390_charger_driver);
}

module_init(zp3390_charger_init);
module_exit(zp3390_charger_exit);

MODULE_DESCRIPTION("Samsung zp3390 Charger Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");

