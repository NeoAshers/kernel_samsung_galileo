/*
 *  p9222_charger.c
 *  Samsung p9222 Charger Driver
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
#include <linux/types.h>
#include <../drivers/battery_v2/include/sec_charging_common.h>
#include <../drivers/battery_v2/include/charger/p9222_charger.h>
#include <../drivers/battery_v2/include/sec_battery.h>

#define P9222_D2D_ALIGN_GUIDE

/* Vout stabilization time is about 1.5sec after Vrect ocured. */
#define VOUT_STABLILZATION_TIME		1500

#define WPC_AUTH_DELAY_TIME			6
#define UNKNOWN_TX_ID				0xFF

#define ENABLE 1
#define DISABLE 0
#define CMD_CNT 3

#define P9222_I_OUT_OFFSET		12

static enum power_supply_property sec_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
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

extern unsigned int lpcharge;

static int
p9222_wpc_auth_delayed_work(struct p9222_charger_data *charger, int sec);
static int p9222_ap_battery_monitor(struct p9222_charger_data *charger, int mode);
static int p9222_full_charge(struct p9222_charger_data *charger);
static void p9222_wpc_request_tx_id(struct p9222_charger_data *charger, int cnt);
static int p9222_power_sharing_charge_chk(struct p9222_charger_data *charger, int tx_id);
static int p9222_normal_full_charge(struct p9222_charger_data *charger);
static int p9222_normal_re_charge(struct p9222_charger_data *charger);
static int p9222_multi_charger_chk(struct p9222_charger_data *charger);
static int p9222_get_vout_strength(struct p9222_charger_data *charger);
static void p9222_send_packet(struct p9222_charger_data *charger, u8 header, u8 rx_data_com, u8 data_val);
static int p9222_rx_ic_power_on_check(struct p9222_charger_data *charger);
static void p9222_wpc_send_tx_uno_mode_disable(struct p9222_charger_data *charger, int cnt);
static int p9222_power_hold_full_charge(struct p9222_charger_data *charger);
static int p9222_power_hold_re_charge(struct p9222_charger_data *charger);
static int p9222_write_ap_mode(struct p9222_charger_data *charger, int ap_mode);

static int p9222_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	struct p9222_charger_data *charger = i2c_get_clientdata(client);
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
	if (ret < 0) {
		pr_err( "%s err reg(0x%x) ret(%d)\n", __func__, reg, ret);
		return -1;
	}
	*val = rbuf[0];

#if defined(CONFIG_WIRELESS_FIRMWARE_UPDATE)
	if(!p9222_otp_update)
#endif
	{
		pr_debug("%s reg = 0x%x, data = 0x%x\n", __func__, reg, *val);
	}

	return ret;
}

static int p9222_reg_read_bytes(struct i2c_client *client, u16 reg, u16 *val)
{
	struct p9222_charger_data *charger = i2c_get_clientdata(client);
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
	msg[1].len = 2;
	msg[1].buf = rbuf;

	mutex_lock(&charger->io_lock);
	ret = i2c_transfer(client->adapter, msg, 2);
	mutex_unlock(&charger->io_lock);
	if (ret < 0) {
		pr_err( "%s err reg(0x%x) ret(%d)\n", __func__, reg, ret);
		return -1;
	}
	*val = (rbuf[1] << 8) | rbuf[0];

	pr_debug("%s reg = 0x%x, data = 0x%x\n", __func__, reg, *val);

	return ret;
}

static int p9222_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	struct p9222_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	mutex_lock(&charger->io_lock);
	ret = i2c_master_send(client, data, 3);
	mutex_unlock(&charger->io_lock);
	if (ret < 3) {
		pr_err("%s err reg(0x%x) ret(%d)\n", __func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	pr_debug("%s reg = 0x%x, data = 0x%x \n", __func__, reg, val);

	return 0;
}

static int p9222_reg_update(struct i2c_client *client, u16 reg, u8 val, u8 mask)
{
	struct p9222_charger_data *charger = i2c_get_clientdata(client);
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };
	u8 data2;
	int ret;

	ret = p9222_reg_read(client, reg, &data2);
	if (ret >= 0) {
		u8 old_val = data2 & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		data[2] = new_val;

		mutex_lock(&charger->io_lock);
		ret = i2c_master_send(client, data, 3);
		mutex_unlock(&charger->io_lock);
		if (ret < 3) {
			pr_err("%s: err reg: 0x%x, ret: %d\n", __func__, reg, ret);
			return ret < 0 ? ret : -EIO;
		}
	}
	p9222_reg_read(client, reg, &data2);

	pr_debug("%s reg = 0x%x, data = 0x%x, mask = 0x%x\n", __func__, reg, val, mask);

	return ret;
}

static int p9222_is_on_pad(struct p9222_charger_data *charger)
{
	int ret;
	int wpc_det = charger->pdata->wpc_det;
	ret = (gpio_get_value(wpc_det) == 0) ? WPC_ON_PAD : WPC_OFF_PAD;
	return ret;
}

static int p9222_get_adc(struct p9222_charger_data *charger, int adc_type)
{
	int ret = 0;
	u8 data[2] = {0,};
	u16 val = 0;

	switch (adc_type) {
		case P9222_ADC_VOUT:
			ret = p9222_reg_read_bytes(charger->client, P9222_ADC_VOUT_L_REG, &val);
			ret = (ret >= 0) ? val : 0;
			break;
		case P9222_ADC_VRECT:
			ret = p9222_reg_read_bytes(charger->client, P9222_ADC_VRECT_L_REG, &val);
			ret = (ret >= 0) ? val : 0;
			break;
		case P9222_ADC_TX_ISENSE:
			ret = p9222_reg_read_bytes(charger->client, P9222_ADC_TX_ISENSE_L_REG, &val);
			ret = (ret >= 0) ? (val & 0x0FFF) : 0;
			break;
		case P9222_ADC_RX_IOUT:
			ret = p9222_reg_read_bytes(charger->client, P9222_ADC_RX_IOUT_L_REG, &val);
			ret = (ret >= 0) ? val : 0;
			break;
		case P9222_ADC_DIE_TEMP:
			ret = p9222_reg_read_bytes(charger->client, P9222_ADC_DIE_TEMP_L_REG, &val);
			ret = (ret >= 0) ? val : 0;
			break;
		case P9222_ADC_ALLIGN_X:
			ret = p9222_reg_read(charger->client, P9222_ADC_ALLIGN_X_REG, &data[0]);
			if(ret >= 0 )
				ret = data[0]; // need to check
			else
				ret = 0;
			break;

		case P9222_ADC_ALLIGN_Y:
			ret = p9222_reg_read(charger->client, P9222_ADC_ALLIGN_Y_REG, &data[0]);
			if(ret >= 0 )
				ret = data[0]; // need to check
			else
				ret = 0;
			break;
		case P9222_ADC_OP_FRQ:
			ret = p9222_reg_read_bytes(charger->client, P9222_OP_FREQ_L_REG, &val);
			ret = (ret >= 0) ? val : 0;
			break;
		case P9222_ADC_VBAT_RAW:
			ret = p9222_reg_read_bytes(charger->client, P9222_VBAT_L_REG, &val);
			ret = (ret >= 0) ? val : 0;
			break;
		case P9222_ADC_VBAT:
			ret = p9222_reg_read_bytes(charger->client, P9222_ADC_VBAT_L_REG, &val);
			ret = (ret >= 0) ? (val & 0x0FFF) : 0;
			break;
		default:
			break;
	}

	return ret;
}

static void p9222_fod_set(struct p9222_charger_data *charger)
{
	int i = 0;

	pr_info("%s \n", __func__);
	if(charger->pdata->fod_data_check) {
		for(i=0; i< P9222_NUM_FOD_REG; i++)
			p9222_reg_write(charger->client, P9222_WPC_FOD_0A_REG+i, charger->pdata->fod_data[i]);
	}
}

static void p9222_set_cmd_reg(struct p9222_charger_data *charger, u8 val, u8 mask)
{
	u8 temp = 0;
	int ret = 0, i = 0;

	do {
		pr_info("%s \n", __func__);
		ret = p9222_reg_update(charger->client, P9222_COMMAND_REG, val, mask); // command
		if(ret >= 0) {
			msleep(250);
			ret = p9222_reg_read(charger->client, P9222_COMMAND_REG, &temp); // check out set bit exists
			if(ret < 0 || i > 3 )
				break;
		}
		i++;
	}while(temp != 0);
}

void p9222_send_eop(struct p9222_charger_data *charger, int health_mode)
{
	int i = 0;
	int ret = 0;

	pr_info("%s health_mode = %d\n", __func__, health_mode);

	switch(health_mode) {
		case POWER_SUPPLY_HEALTH_OVERHEAT:
		case POWER_SUPPLY_HEALTH_OVERHEATLIMIT:
		case POWER_SUPPLY_HEALTH_COLD:
			charger->ot_cnt++;
			if(charger->pdata->cable_type == SEC_WIRELESS_PAD_PMA) {
				pr_info("%s pma mode \n", __func__);
				for(i = 0; i < CMD_CNT; i++) {
					ret = p9222_reg_write(charger->client,
						P9222_END_POWER_TRANSFER_REG, P9222_EPT_END_OF_CHG);
					if(ret >= 0) {
						p9222_set_cmd_reg(charger,
							P9222_CMD_SEND_EOP_MASK, P9222_CMD_SEND_EOP_MASK);
						msleep(250);
					} else
						break;
				}
			} else {
				pr_info("%s wpc mode \n", __func__);
				for(i = 0; i < CMD_CNT; i++) {
					ret = p9222_reg_write(charger->client,
						P9222_END_POWER_TRANSFER_REG, P9222_EPT_OVER_TEMP);
					if(ret >= 0) {
						p9222_set_cmd_reg(charger,
							P9222_CMD_SEND_EOP_MASK, P9222_CMD_SEND_EOP_MASK);
						msleep(250);
					} else
						break;
				}
			}
			break;
		case POWER_SUPPLY_HEALTH_UNDERVOLTAGE:
			pr_info("%s ept-undervoltage \n", __func__);
			if (p9222_power_sharing_charge_chk(charger, charger->tx_id)) {
				p9222_wpc_send_tx_uno_mode_disable(charger, 30);
			} else {
				charger->under_voltage_cnt++;
				ret = p9222_reg_write(charger->client,
					P9222_END_POWER_TRANSFER_REG, P9222_EPT_OVER_TEMP);
				if(ret >= 0) {
					p9222_set_cmd_reg(charger,
						P9222_CMD_SEND_EOP_MASK, P9222_CMD_SEND_EOP_MASK);
					msleep(250);
				}
			}
			break;
		default:
			break;
	}
}

int p9222_send_cs100(struct p9222_charger_data *charger)
{
	int i = 0;
	int ret = 0;

	for(i = 0; i < 6; i++) {
		ret = p9222_reg_write(charger->client, P9222_CHG_STATUS_REG, 100);
		if(ret >= 0) {
			p9222_set_cmd_reg(charger, P9222_CMD_SEND_CHG_STS_MASK, P9222_CMD_SEND_CHG_STS_MASK);
			msleep(250);
			ret = 1;
		} else {
			ret = -1;
			break;
		}
	}
	return ret;
}

static void p9222_wpc_send_tx_uno_mode_disable(struct p9222_charger_data *charger, int cnt)
{
	int i;

	pr_info("%s (cnt %d)\n", __func__, cnt);
	wake_lock(&charger->unsafe_voltage_chk_lock);

	for (i = 0; i < cnt; i++) {

		p9222_send_packet(charger, P9222_HEADER_AFC_CONF, 0x19, 0xFF);
		if (cnt > 1) {
			msleep(100);
			if (p9222_rx_ic_power_on_check(charger) == 0) {
				get_monotonic_boottime(&charger->uno_off_ts);
				break;
			}
		}
	}
	wake_unlock(&charger->unsafe_voltage_chk_lock);
	charger->uno_off_cnt++;
	pr_info("%s stop at cnt %d)\n", __func__, i);
}

static int p9222_get_target_vout(struct p9222_charger_data *charger)
{
	int target_vout = charger->pdata->low_vout_level;

	if (charger->power_hold_mode == 1)
		return 4100;

	if (p9222_multi_charger_chk(charger) != 1)
		return target_vout;

	/* Duo pad(TX id 0x70 ~ 0x7F) apply CEP by itself.
	 * Thus output vout is lower than expected set by ap mode.
	 */
	if (charger->pdata->use_ap_mode_table) {
		u8 ap_mode;
		int vbat;
		int i, j;
		int compare_vout = 0;
		int batt_mode_margin;
		int table_vout = 0;

		vbat = p9222_get_adc(charger, P9222_ADC_VBAT_RAW);
		p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE, &ap_mode);

		for (i = 0; i < charger->pdata->num_ap_mode_table; i++) {
			if (ap_mode == charger->pdata->ap_mode_table_info[i].ap_mode) {
				batt_mode_margin = charger->pdata->ap_mode_table_info[i].batt_mode_margin;
				table_vout = charger->pdata->ap_mode_table_info[i].vout;

				if (batt_mode_margin) {
					compare_vout = table_vout - batt_mode_margin;
					if (vbat <= compare_vout) {
						target_vout = table_vout - 150;
					} else {
						target_vout = (vbat + batt_mode_margin) - 150;
					}
				} else {
					if ((table_vout - 160) < charger->pdata->low_vout_level)
						target_vout = table_vout - 160;
					else {
						target_vout = charger->pdata->low_vout_level;
						for (j = 0; j < charger->pdata->num_ex_low_vout; j++) {
							if (charger->pdata->ex_low_vout[j].tx_id == charger->tx_id)
								target_vout = charger->pdata->ex_low_vout[j].low_vout;
						}
					}
				}
				break;
			}
		}

		pr_debug("%s ap_mode(%x) table_vout(%d) target_vout(%d) compare_vout(%d) vbat(%d)\n",
			__func__, ap_mode, table_vout, target_vout, compare_vout, vbat);
	}

	return target_vout;
}

static int p9222_unsafe_vout_check(struct p9222_charger_data *charger)
{
	int vout;
	int target_vout;

	vout = p9222_get_adc(charger, P9222_ADC_VOUT);
	target_vout = p9222_get_target_vout(charger);

	pr_info("%s vout(%d) target_vout(%d)\n", __func__, vout, target_vout);
	if (vout < target_vout)
		return 1;
	return 0;
}

static int p9222_unsafe_charge_check(struct p9222_charger_data *charger)
{
	int ret;

	if (charger->align_check_start == 1) {
		return 0;
	}

	if (charger->power_hold_mode == 1) {
		return 0;
	}

	ret = p9222_unsafe_vout_check(charger);
	return ret;
}

static void p9222_send_packet(struct p9222_charger_data *charger, u8 header, u8 rx_data_com, u8 data_val)
{
	if (p9222_reg_write(charger->client, P9222_PACKET_HEADER, header) < 0)
		return;

	if (p9222_reg_write(charger->client, P9222_RX_DATA_COMMAND, rx_data_com) < 0)
		return;

	if (p9222_reg_write(charger->client, P9222_RX_DATA_VALUE0, data_val) < 0)
		return;

	p9222_reg_update(charger->client, P9222_COMMAND_REG,
		P9222_CMD_SEND_RX_DATA_MASK, P9222_CMD_SEND_RX_DATA_MASK);
}

void p9222_send_multi_packet(struct p9222_charger_data *charger, u8 header, u8 rx_data_com, u8 *data_val, int data_size)
{
	int ret;
	int i;
	ret = p9222_reg_write(charger->client, P9222_PACKET_HEADER, header);
	ret = p9222_reg_write(charger->client, P9222_RX_DATA_COMMAND, rx_data_com);

	for(i = 0; i< data_size; i++) {
		ret = p9222_reg_write(charger->client, P9222_RX_DATA_VALUE0 + i, data_val[i]);
	}
	p9222_set_cmd_reg(charger, P9222_CMD_SEND_RX_DATA_MASK, P9222_CMD_SEND_RX_DATA_MASK);
}

static bool p9222_check_chg_in_status(struct p9222_charger_data *charger)
{
	union power_supply_propval value;
	bool ret = false;

	psy_do_property(charger->pdata->charger_name, get,
		POWER_SUPPLY_PROP_POWER_NOW, value);

	if (value.intval == ACOK_INPUT)
		ret = true;

	return ret;
}

static bool p9222_wait_chg_in_ok(struct p9222_charger_data *charger, int retry)
{
	bool chg_in_ok;
	do {
		if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
			pr_info("%s pad is off!\n", __func__);
			break;
		}

		chg_in_ok = p9222_check_chg_in_status(charger);

		pr_info("%s chg_in_ok = %d vout = %d retry = %d\n",
			__func__, chg_in_ok, charger->vout_status, retry);

		if (chg_in_ok)
			return true;
		msleep(300);

		retry--;
	} while(retry > 0);

	return false;
}

static int p9222_get_wpc_en_toggle_time(struct p9222_charger_data *charger)
{
	int toggle_time = charger->pdata->ps_wpc_en_toggle_time;
	int i;

	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
		if (charger->pdata->charger_type[i].compatible_tx_id == charger->tx_id) {
			if (charger->pdata->charger_type[i].phm_free_toggle_time > 0) {
				toggle_time = charger->pdata->charger_type[i].phm_free_toggle_time;
				break;
			}
		}
	}

	pr_info("%s tx(%x) toggle_time(%d)\n", __func__, charger->tx_id, toggle_time);

	return toggle_time;
}

static void p9222_rx_ic_reset(struct p9222_charger_data *charger)
{
	int ping_durationg =
		(charger->pdata->ping_duration * 2) + (charger->pdata->ping_duration / 4);

	pr_info("%s ping_durationg = %d\n", __func__, ping_durationg);

	if (gpio_is_valid(charger->pdata->wpc_en)) {
		wake_lock(&charger->wpc_wake_lock);

		/* rx_ic reset need 5 pings when wpc_en high status */
		gpio_direction_output(charger->pdata->wpc_en, 1);
		msleep(ping_durationg);
		gpio_direction_output(charger->pdata->wpc_en, 0);
		wake_unlock(&charger->wpc_wake_lock);

		charger->charge_mode = P9222_CHARGE_MODE_NONE;
		p9222_wait_chg_in_ok(charger, 8);
	}
}

static int p9222_rx_ic_power_on_check(struct p9222_charger_data *charger)
{
	int vrect;
	vrect = p9222_get_adc(charger, P9222_ADC_VRECT);

	if (vrect > 0) {
		pr_info("%s rx ic on(vrect : %d)\n", __func__, vrect);
		return 1;
	}

	pr_info("%s rx ic off(vrect : %d)\n", __func__, vrect);
	return 0;
}

static int p9222_send_watchdog_err_packet(struct p9222_charger_data *charger)
{
	int i = 0;

	pr_info("%s\n", __func__);

	for (i = 0; i < 3; i++) {
		p9222_send_multi_packet(charger, 0x18, 0xE7, NULL, 0);
		msleep(200);

		if (p9222_rx_ic_power_on_check(charger) == 0) {
			break;
		}
	}
	return 0;
}


static int p9222_power_hold_mode_set(struct p9222_charger_data *charger, int set)
{
	int i = 0, j;
	int ping_durationg = p9222_get_wpc_en_toggle_time(charger);
	int ret = 0;
	bool acok;
	int vout;

	acok = p9222_check_chg_in_status(charger);

	pr_info("%s set = %d power_hold_mode = %d tx_id = %x acok(%d)\n", __func__,
		set, charger->power_hold_mode, charger->tx_id, acok);

	wake_lock(&charger->wpc_wake_lock);

	if (set) {
		vout = p9222_get_adc(charger, P9222_ADC_VOUT);
		if (vout > 0) {
			charger->power_hold_mode = 1;
			for (i = 0; i < 3; i++) {
				ret = -1;
				p9222_send_packet(charger, 0x28, 0x18, 0x01);
				msleep(200);

				if (p9222_rx_ic_power_on_check(charger) == 0) {
					for (j = 0; j < 3; j++) {
						if (p9222_check_chg_in_status(charger) == true)
							msleep(100);
						else
							break;
					}
					ret = 0;
					if (charger->phm_set_fail == 1)
						charger->phm_set_fail = 0;
					break;
				}
			}
		}
	} else {
		if (!acok) {
			if (gpio_is_valid(charger->pdata->wpc_en)) {
				for (i = 0; i < 2; i++) {
					/* power hold mode exit need 2 pings when wpc_en high status */
					gpio_direction_output(charger->pdata->wpc_en, 1);
					msleep(ping_durationg);
					gpio_direction_output(charger->pdata->wpc_en, 0);

					if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
						pr_err("%s wpc detached!!\n", __func__);
						wake_unlock(&charger->wpc_wake_lock);
						return ret;
					}

					/* Power transfer phase have to keep  over 5sec to Vout LDO turn on
					 * if Vrect is lower than target Vrect.
					 * Thus wait chg_in_ok over 5sec.
					 */
					if (p9222_wait_chg_in_ok(charger, 20) == true) {
						/* for stable ap mode wirte after acok inserted */
						msleep(300);
						charger->charge_mode = P9222_CHARGE_MODE_NONE;
						if (p9222_check_chg_in_status(charger) == 0) {
							p9222_rx_ic_reset(charger);
							pr_err("%s chg_in err!\n", __func__);
						}
					} else {
						if (p9222_power_sharing_charge_chk(charger, charger->tx_id))
							p9222_rx_ic_reset(charger);
					}

					if (p9222_check_chg_in_status(charger) == 1) {
						pr_info("%s phm free ok!(%d)\n", __func__, i);
						break;
					} else {
						pr_err("%s phm free fail!(%d)\n", __func__, i);
					}
				}
			} else {
				pr_err("%s wpc_en is invalid gpio!!\n", __func__);
			}
			charger->power_hold_mode = 0;
		}
	}

	wake_unlock(&charger->wpc_wake_lock);
	pr_info("%s end\n", __func__);

	return ret;
}

static int
p9222_wpc_det_irq_enable(struct p9222_charger_data *charger, bool en_irq)
{
	int ret = 0;

	pr_info("%s irq(%d) wakeup(%d) en_irq(%d) called from(%ps)\n",
		__func__, charger->irq_wpc_det_enabled,
		charger->irq_wpc_det_wakeup_enabled, en_irq,
		__builtin_return_address(0));

	if (en_irq) {
		if (charger->irq_wpc_det_wakeup_enabled == 0) {
			ret = enable_irq_wake(charger->pdata->irq_wpc_det);
			charger->irq_wpc_det_wakeup_enabled = 1;
		}

		if (charger->irq_wpc_det_enabled == 0) {
			enable_irq(charger->pdata->irq_wpc_det);
			charger->irq_wpc_det_enabled = 1;
		}

	} else {
		if (charger->irq_wpc_det_wakeup_enabled == 1) {
			disable_irq_wake(charger->pdata->irq_wpc_det);
			charger->irq_wpc_det_wakeup_enabled = 0;
		}
		if (charger->irq_wpc_det_enabled == 1) {
			disable_irq(charger->pdata->irq_wpc_det);
			charger->irq_wpc_det_enabled = 0;
		}
	}

	return ret;
}

static int p9222_get_firmware_version(struct p9222_charger_data *charger, int firm_mode)
{
	int version = -1;
	int ret;
	u8 otp_fw_major[2] = {0,};
	u8 otp_fw_minor[2] = {0,};
	u8 tx_fw_major[2] = {0,};
	u8 tx_fw_minor[2] = {0,};

	if (p9222_is_on_pad(charger) == WPC_OFF_PAD)
		return version;

	switch (firm_mode) {
		case P9222_RX_FIRMWARE:
			ret = p9222_reg_read(charger->client, P9222_OTP_FW_MAJOR_REV_L_REG, &otp_fw_major[0]);
			ret = p9222_reg_read(charger->client, P9222_OTP_FW_MAJOR_REV_H_REG, &otp_fw_major[1]);
			if (ret >= 0) {
				version =  otp_fw_major[0] | (otp_fw_major[1] << 8);
			}
			pr_debug("%s rx major firmware version 0x%x \n", __func__, version);

			ret = p9222_reg_read(charger->client, P9222_OTP_FW_MINOR_REV_L_REG, &otp_fw_minor[0]);
			ret = p9222_reg_read(charger->client, P9222_OTP_FW_MINOR_REV_H_REG, &otp_fw_minor[1]);
			if (ret >= 0) {
				version =  otp_fw_minor[0] | (otp_fw_minor[1] << 8);
			}
			pr_debug("%s rx minor firmware version 0x%x \n", __func__, version);
			break;
		case P9222_TX_FIRMWARE:
			ret = p9222_reg_read(charger->client, P9222_SRAM_FW_MAJOR_REV_L_REG, &tx_fw_major[0]);
			ret = p9222_reg_read(charger->client, P9222_SRAM_FW_MAJOR_REV_H_REG, &tx_fw_major[1]);
			if (ret >= 0) {
				version =  tx_fw_major[0] | (tx_fw_major[1] << 8);
			}
			pr_debug("%s tx major firmware version 0x%x \n", __func__, version);

			ret = p9222_reg_read(charger->client, P9222_SRAM_FW_MINOR_REV_L_REG, &tx_fw_minor[0]);
			ret = p9222_reg_read(charger->client, P9222_SRAM_FW_MINOR_REV_H_REG, &tx_fw_minor[1]);
			if (ret >= 0) {
				version =  tx_fw_minor[0] | (tx_fw_minor[1] << 8);
			}
			pr_debug("%s tx minor firmware version 0x%x \n", __func__, version);
			break;
		default:
			pr_err("%s Wrong firmware mode \n", __func__);
			version = -1;
			break;
	}

	return version;
}

static void p9222_wpc_power_hold_check(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, power_hold_chk_work.work);

	if ((charger->power_hold_mode == 1) &&
		p9222_check_chg_in_status(charger)) {
		pr_err("%s abnormal chg status!! go to vout off or phm!!\n", __func__);
		charger->power_hold_mode = 0;
		p9222_full_charge(charger);
	}

	wake_unlock(&charger->power_hold_chk_lock);
}

static void p9222_phm_free_check(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, phm_free_work.work);

	pr_info("%s", __func__);

	gpio_direction_output(charger->pdata->wpc_en, 0);
	wake_unlock(&charger->phm_free_lock);
}

static void p9222_cs100_work(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, cs100_work.work);

	pr_info("%s", __func__);

	if ((p9222_rx_ic_power_on_check(charger) ==0) && charger->power_hold_mode)
		p9222_power_hold_mode_set(charger, 0);

	charger->pdata->cs100_status = p9222_send_cs100(charger);
	wake_unlock(&charger->cs100_lock);
}

static void p9222_wpc_init(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, wpc_init_work.work);
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
				p9222_reg_write(charger->client, P9222_VRECT_TARGET_VOL_L, 0X27);
				p9222_reg_write(charger->client, P9222_VRECT_TARGET_VOL_H, 0X04);
			} else {
				p9222_reg_write(charger->client, P9222_VRECT_SET_REG, 126);
				p9222_reg_update(charger->client, 0x81, 0x80, 0x80); //set 7th bit in 0x81 to power on
			}
			p9222_reg_update(charger->client, P9222_COMMAND_REG,
				P9222_CMD_TOGGLE_LDO_MASK, P9222_CMD_TOGGLE_LDO_MASK);
		}

		/* check again firmare version support vbat monitoring */
		if (!charger->pdata->otp_firmware_ver) {
			fw_ver = p9222_get_firmware_version(charger, P9222_RX_FIRMWARE);
			if (fw_ver > 0) {
				pr_debug("%s rx major firmware version 0x%x\n", __func__, fw_ver);
				charger->pdata->otp_firmware_ver = fw_ver;
			}
		}

		ret = enable_irq_wake(charger->client->irq);
		if (ret < 0)
			pr_err("%s: Failed to Enable Wakeup Source(%d)\n", __func__, ret);
	}

	if (charger->wc_w_state == WPC_ON_PAD) {
		bool chg_in_ok;

		chg_in_ok = p9222_check_chg_in_status(charger);

		if (chg_in_ok) {
			charger->force_wpc_det_chk = true;
			queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
			p9222_wpc_det_irq_enable(charger, true);
		} else {
			p9222_wpc_det_irq_enable(charger, false);
		}
	} else {
		p9222_wpc_det_irq_enable(charger, false);
	}
	wake_unlock(&charger->wpc_wake_lock);
}

static int p9222_temperature_check(struct p9222_charger_data *charger)
{
	if (charger->temperature >= charger->pdata->tx_off_high_temp) {
		/* send TX watchdog command in high temperature */
		pr_err("%s:HIGH TX OFF TEMP:%d\n", __func__, charger->temperature);
		p9222_send_watchdog_err_packet(charger);
	}

	return 0;
}

static int p9222_low_current_detect_wa(struct p9222_charger_data *charger, int vout)
{
	struct timespec ts = {0, };
	long low_curr_sec;
	int i;
	union power_supply_propval value;

	if (charger->pdata->support_low_iout_detect == 0)
		return 0;

	if (charger->low_current_detected > 1)
		return 0;

	if (vout < 1000)
		return 0;

	if (charger->current_now < 10) {
		if (charger->low_current_detected == 0) {
			charger->low_current_detected = 1;
			get_monotonic_boottime(&charger->low_current_detect_time);
			return 0;
		} else {
			get_monotonic_boottime(&ts);
			low_curr_sec = ts.tv_sec - charger->low_current_detect_time.tv_sec;

			pr_info("%s low_curr_sec(%ld)\n", __func__, low_curr_sec);

			if (low_curr_sec > 300) {
				if (charger->low_current_detected == 1) {
					//charger->low_current_detected++;
					charger->low_iout_cnt++;
					pr_info("%s low iout event detected!!!\n", __func__);
					p9222_power_hold_full_charge(charger);

					value.intval = SEC_BAT_CHG_MODE_CHARGING_OFF;
					psy_do_property(charger->pdata->charger_name, set,
						POWER_SUPPLY_PROP_CHARGING_ENABLED, value);

					for (i = 0; i < 10; i++) {
						if (p9222_get_adc(charger, P9222_ADC_VOUT) < 1000) {
							break;
						} else {
							msleep(100);
						}
					}
					p9222_power_hold_re_charge(charger);

					p9222_write_ap_mode(charger, P9222_AP_BATTERY_WAKEUP_MODE);

					value.intval = SEC_BAT_CHG_MODE_CHARGING;
					psy_do_property(charger->pdata->charger_name, set,
						POWER_SUPPLY_PROP_CHARGING_ENABLED, value);
				}
			}
		}
	} else {
		charger->low_current_detected = 0;
	}

	return 0;
}

static int p9222_abnormal_acok_cnt_wa(struct p9222_charger_data *charger)
{
	union power_supply_propval value;
	int i;

	if (charger->pdata->support_low_iout_detect == 0)
		return 0;

	value.intval = DEBUG_INFO_ACOK_CNT;
	psy_do_property(charger->pdata->charger_name, get,
		POWER_SUPPLY_EXT_PROP_DEBUG_INFO, value);
	charger->curr_acok_cnt = value.intval;

	if ((charger->curr_acok_cnt > charger->prev_acok_cnt) &&
		(charger->curr_acok_cnt - charger->prev_acok_cnt > 60)) {
		charger->low_iout_cnt++;
		pr_info("%s curr(%d) prev(%d)\n",
			__func__, charger->curr_acok_cnt, charger->prev_acok_cnt);
		p9222_normal_full_charge(charger);

		value.intval = SEC_BAT_CHG_MODE_CHARGING_OFF;
		psy_do_property(charger->pdata->charger_name, set,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, value);

		for (i = 0; i < 10; i++) {
			if (p9222_get_adc(charger, P9222_ADC_VOUT) < 1000) {
				break;
			} else {
				msleep(100);
			}
		}
		p9222_normal_re_charge(charger);

		p9222_write_ap_mode(charger, P9222_AP_BATTERY_WAKEUP_MODE);

		value.intval = SEC_BAT_CHG_MODE_CHARGING;
		psy_do_property(charger->pdata->charger_name, set,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, value);
	}

	charger->prev_acok_cnt = charger->curr_acok_cnt;

	return 0;
}

static u64 p9222_get_err_status(struct p9222_charger_data *charger)
{
	u64 ret = (((u64)(charger->low_iout_cnt & 0xFF) << 32) |
		((charger->wtd_err_cnt & 0xFF) << 24) |
		((charger->under_voltage_cnt & 0xFF) << 16) |
		((charger->ot_cnt & 0xFF) << 8) |
		(charger->uno_off_cnt & 0xFF));

	return ret;
}

static int p9222_monitor_work(struct p9222_charger_data *charger)
{
	int vrect;
	int vout = 0;
	int freq = 0;
	int temp = 0;
	u8 ap_mode = 0, vbat_monitor = 0;
	u8 full_bridge = 0;
	int capacity;
	union power_supply_propval value = {0, };

	if (p9222_is_on_pad(charger) == WPC_ON_PAD) {
		value.intval = SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE;
		psy_do_property(charger->pdata->fuelgauge_name, get,
			POWER_SUPPLY_PROP_CAPACITY, value);
		capacity = value.intval;

		vrect = p9222_get_adc(charger, P9222_ADC_VRECT);

		/* If vrect output normally, other reg is read normllay. */
		if (vrect) {
			vout = p9222_get_adc(charger, P9222_ADC_VOUT);
			freq = p9222_get_adc(charger, P9222_ADC_OP_FRQ);
			temp = p9222_get_adc(charger, P9222_ADC_DIE_TEMP) / 100;
			charger->current_now = p9222_get_adc(charger, P9222_ADC_RX_IOUT);
			charger->voltage_now = p9222_get_adc(charger, P9222_ADC_VBAT_RAW);

			p9222_reg_read(charger->client, P9222_RECT_OPER_MODE, &full_bridge);
			full_bridge = (P9222_RECT_OPER_MASK & full_bridge);

			p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE, &ap_mode);
			p9222_reg_read(charger->client, P9222_WPC_FLAG_REG, &vbat_monitor);
		} else {
			charger->current_now = 0;
			charger->voltage_now = 0;
		}

		charger->err_status = p9222_get_err_status(charger);

		pr_info("%s vrect(%dmV) vout(%dmV) iout(%dmA) vbat(%dmV) ap_mode(0x%02x) SOC(%d) "
			"phm(%d) type(%d) chg_mode(%d) fw(%x) monitor(%x) "
			"tx_id(%x) freq(%d) temp(%d) full(%x) err(%llX) pdetb(%d) low_iout(%d)\n",
			__func__, vrect, vout, charger->current_now, charger->voltage_now, ap_mode, capacity,
			charger->power_hold_mode, charger->charger_type, charger->charge_mode,
			charger->pdata->otp_firmware_ver, vbat_monitor,
			charger->tx_id, freq, temp, full_bridge, charger->err_status,
			charger->pdetb_count, charger->low_current_detected);

		if (vrect > 0) {
			psy_do_property(charger->pdata->battery_name, get,
				POWER_SUPPLY_PROP_HEALTH, value);
			if(value.intval == POWER_SUPPLY_HEALTH_OVERHEAT ||
				value.intval == POWER_SUPPLY_HEALTH_OVERHEATLIMIT) {
				pr_info("%s ept-ot \n", __func__);
				p9222_send_eop(charger, value.intval);
			}

			psy_do_property(charger->pdata->battery_name, get,
				POWER_SUPPLY_PROP_STATUS, value);

			if ((value.intval == POWER_SUPPLY_STATUS_CHARGING) ||
				(value.intval == POWER_SUPPLY_STATUS_FULL)) {
				p9222_low_current_detect_wa(charger, vout);
				p9222_abnormal_acok_cnt_wa(charger);
			}
		}
	} else {
		pr_info("%s pad off!\n", __func__);
	}

	return 0;
}

static int p9222_monitor_wdt_kick(struct p9222_charger_data *charger)
{
	if ((charger->pdata->watchdog_test == false) &&
		(charger->wc_w_state == WPC_ON_PAD)) {
		p9222_reg_update(charger->client, P9222_COMMAND_REG,
			P9222_CMD_SS_WATCHDOG_MASK, P9222_CMD_SS_WATCHDOG_MASK);
	}
	return 0;
}

static int p9222_write_ap_mode(struct p9222_charger_data *charger, int ap_mode)
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
		p9222_reg_write(charger->client, P9222_AP_BATTERY_MODE, ap_mode);
		p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE, &data);

		if (ap_mode == data) {
			ret = 0;
			goto end;
		}
		msleep(100);
	}

end:
	if ((ap_mode == P9222_AP_BATTERY_BATT_MODE) &&
		(charger->pdata->enable_batt_mode_headroom_margin == 1)) {

		/* set 144mV Minimum headroom between VOUT and VBAT when in VBAT Monitoring mode
		 * 0x90 is 144mV in decimal number
		 */
		p9222_reg_read(charger->client, P9222_BATT_VOLTAGE_HEADROOM_L, &data);
		if (data != 0x90) {
			msleep(200);
			p9222_reg_write(charger->client, P9222_BATT_VOLTAGE_HEADROOM_L, 0x90);
			p9222_reg_read(charger->client, P9222_BATT_VOLTAGE_HEADROOM_L, &data);
			pr_info("%s P9222_BATT_VOLTAGE_HEADROOM_L = %x\n", __func__, data);

		}
	}

	pr_info("%s ap_mode %x read data = %x\n", __func__, ap_mode, data);
	return ret;
}

static int p9222_get_cc_ap_mode(struct p9222_charger_data *charger, int *ap_mode)
{
	int i;
	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
		if (charger->pdata->charger_type[i].compatible_tx_id == charger->tx_id) {
			*ap_mode = charger->pdata->charger_type[i].cc_ap_mode;
			return 0;
		}
	}
	return -1;
}

static int p9222_get_cv_ap_mode(struct p9222_charger_data *charger, int *ap_mode)
{
	int i;
	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
		if (charger->pdata->charger_type[i].compatible_tx_id == charger->tx_id) {
			*ap_mode = charger->pdata->charger_type[i].cv_ap_mode;
			return 0;
		}
	}
	return -1;
}

static int p9222_get_half_bridge_ap_mode(struct p9222_charger_data *charger, int *ap_mode)
{
	int i;
	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
		if (charger->pdata->charger_type[i].compatible_tx_id == charger->tx_id) {
			*ap_mode = charger->pdata->charger_type[i].half_bridge_ap_mode;
			return 0;
		}
	}
	return -1;
}

static int
p9222_compatible_cc_charge(struct p9222_charger_data *charger, bool prev_chk)
{
	int ret = 0;
	int ap_mode = P9222_AP_BATTERY_BATT_MODE;

	if (charger->store_mode) {
		ret = p9222_write_ap_mode(charger, P9222_AP_BATTERY_BATT_MODE);
		return ret;
	}

	p9222_get_cc_ap_mode(charger, &ap_mode);

	if (prev_chk == false) {
		ret = p9222_write_ap_mode(charger, ap_mode);
	} else {
		if (charger->charge_mode != P9222_CHARGE_MODE_CC)
			ret = p9222_write_ap_mode(charger, ap_mode);
	}

	return ret;
}

static int
p9222_compatible_cv_charge(struct p9222_charger_data *charger, bool prev_chk)
{
	int ret = 0;
	int ap_mode = P9222_AP_BATTERY_INCOMPATIBLE_CC_MODE;

	p9222_get_cv_ap_mode(charger, &ap_mode);

	if (prev_chk == false) {
		ret = p9222_write_ap_mode(charger, ap_mode);
	} else {
		if (charger->charge_mode != P9222_CHARGE_MODE_CV)
			ret = p9222_write_ap_mode(charger, ap_mode);
	}
	return ret;
}

static int
p9222_incompatible_cc_charge(struct p9222_charger_data *charger, bool prev_chk)
{
	int ret = 0;
	int ap_mode = P9222_AP_BATTERY_INCOMPATIBLE_CC_MODE;

	p9222_get_cc_ap_mode(charger, &ap_mode);
	pr_info("%s ap_mode = %x\n", __func__, ap_mode);

	if (prev_chk == false) {
		ret = p9222_write_ap_mode(charger, ap_mode);
	} else {
		if (charger->charge_mode != P9222_CHARGE_MODE_CC)
			ret = p9222_write_ap_mode(charger, ap_mode);
	}
	return ret;
}

static int
p9222_incompatible_cv_charge(struct p9222_charger_data *charger, bool prev_chk)
{
	int ret = 0;
	int ap_mode = P9222_AP_BATTERY_INCOMPATIBLE_CC_CV_MODE;

	p9222_get_cc_ap_mode(charger, &ap_mode);
	pr_info("%s ap_mode = %x\n", __func__, ap_mode);

	if (prev_chk == false) {
		ret = p9222_write_ap_mode(charger, ap_mode);
	} else {
		if (charger->charge_mode != P9222_CHARGE_MODE_CV)
			ret = p9222_write_ap_mode(charger, ap_mode);
	}
	return ret;
}

static int p9222_power_hold_full_charge(struct p9222_charger_data *charger)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = p9222_power_hold_mode_set(charger, 1);
	if (ret < 0) {
		if (p9222_multi_charger_chk(charger) == 1) {
			charger->phm_set_fail = 1;
			p9222_normal_full_charge(charger);
		}
	}

	return 0;
}

static int p9222_power_hold_re_charge(struct p9222_charger_data *charger)
{
	int rx_ic_power = p9222_rx_ic_power_on_check(charger);

	pr_info("%s phm_set_fail = %d rx_ic_power = %d\n",
		__func__, charger->phm_set_fail, rx_ic_power);

	if (charger->phm_set_fail && rx_ic_power) {
		if (p9222_multi_charger_chk(charger) == 1) {
			p9222_normal_re_charge(charger);
		}
	} else {
		p9222_power_hold_mode_set(charger, 0);
	}

	charger->phm_set_fail = 0;
	return 0;
}

static int p9222_normal_full_charge(struct p9222_charger_data *charger)
{
	int i;
	pr_info("%s\n", __func__);
	wake_lock(&charger->wpc_wake_lock);
	p9222_ap_battery_monitor(charger, WPC_MODE_VOUT_OFF);
	msleep(200);
	p9222_reg_update(charger->client, P9222_COMMAND_REG,
		P9222_CMD_TOGGLE_LDO_MASK, P9222_CMD_TOGGLE_LDO_MASK);

	for (i = 0; i < 3; i++) {
		if (p9222_check_chg_in_status(charger) == true)
			msleep(100);
		else
				break;
	}

	charger->power_hold_mode = 1;
	wake_unlock(&charger->wpc_wake_lock);
	return 0;
}

static int p9222_normal_re_charge(struct p9222_charger_data *charger)
{
	int i, vrect;

	pr_info("%s\n", __func__);
	wake_lock(&charger->wpc_wake_lock);

	p9222_reg_update(charger->client, P9222_COMMAND_REG,
		P9222_CMD_TOGGLE_LDO_MASK, P9222_CMD_TOGGLE_LDO_MASK);
	p9222_ap_battery_monitor(charger, WPC_MODE_IDT);

	if (p9222_wait_chg_in_ok(charger, 10) == true) {
		for (i = 0; i < 15; i++) {
			vrect = p9222_get_adc(charger, P9222_ADC_VRECT);
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

static int p9222_cc_charge(struct p9222_charger_data *charger, bool prev_chk)
{
	int ret = -1;
	enum p9222_charger_pad_type type = charger->charger_type;

	if (charger->charge_cb_func[type]->cc_chg) {
		ret = charger->charge_cb_func[type]->cc_chg(charger, prev_chk);
		charger->charge_mode = P9222_CHARGE_MODE_CC;
	} else
		pr_err("%s invalid func\n", __func__);
	return ret;
}

static int p9222_cv_charge(struct p9222_charger_data *charger, bool prev_chk)
{
	int ret = -1;
	enum p9222_charger_pad_type type = charger->charger_type;

	if (charger->charge_cb_func[type]->cv_chg) {
		ret = charger->charge_cb_func[type]->cv_chg(charger, prev_chk);
		charger->charge_mode = P9222_CHARGE_MODE_CV;
	} else
		pr_err("%s invalid func\n", __func__);
	return ret;
}

static int p9222_full_charge(struct p9222_charger_data *charger)
{
	int ret = -1;
	int i;
	enum p9222_charger_pad_type type = charger->charger_type;

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
		charger->charge_mode = P9222_CHARGE_MODE_OFF;
		charger->pdata->is_charging = 0;
	} else
		pr_err("%s invalid func\n", __func__);

	mutex_unlock(&charger->charger_lock);
	return ret;
}

static int p9222_re_charge(struct p9222_charger_data *charger)
{
	int ret = -1;
	bool chg_in_ok;
	enum p9222_charger_pad_type type = charger->charger_type;

	chg_in_ok = p9222_check_chg_in_status(charger);

	if (chg_in_ok) {
		pr_info("%s skip. already charge started!\n", __func__);
		return 0;
	}

	pr_info("%s type = %d charger->charge_mode = %d\n",
		__func__, type, charger->charge_mode);

	mutex_lock(&charger->charger_lock);

	if (charger->charge_cb_func[type]->re_chg) {
		charger->charge_mode = P9222_CHARGE_MODE_NONE;
		ret = charger->charge_cb_func[type]->re_chg(charger);

		if (p9222_is_on_pad(charger) == WPC_ON_PAD) {
			charger->pdata->is_charging = 1;
			charger->need_margin = 1;
		}
	} else
		pr_err("%s invalid func\n", __func__);

	mutex_unlock(&charger->charger_lock);
	return ret;
}

static void p9222_cc_cv_charge_select(
	struct p9222_charger_data *charger, int capacity, bool prev_check)
{
	if (capacity >= charger->pdata->cc_cv_threshold) {
		p9222_cv_charge(charger, prev_check);
	} else {
		/* to avoid setting cc mode when once cv mode entered and not detached */
		if ((prev_check == true) && (charger->charge_mode == P9222_CHARGE_MODE_CV)) {
			pr_info("%s forcely set cv mode for temp capacity drop\n", __func__);
			p9222_cv_charge(charger, prev_check);
		} else {
			p9222_cc_charge(charger, prev_check);
		}
	}
}

static int p9222_ap_battery_monitor(struct p9222_charger_data *charger, int mode)
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

	ret = p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE,
		&ap_battery_mode);

	switch (mode) {
	case WPC_MODE_BOOT:
		p9222_write_ap_mode(charger, charger->pdata->boot_ap_mode);
		charger->charge_mode = P9222_CHARGE_MODE_NONE;
		break;
	case WPC_MODE_IDT:
		p9222_write_ap_mode(charger, P9222_AP_BATTERY_IDT_MODE);
		charger->charge_mode = P9222_CHARGE_MODE_NONE;
		break;
	case WPC_MODE_VOUT_OFF:
		p9222_write_ap_mode(charger, P9222_AP_BATTERY_INCOMPATIBLE_PHP_MODE);
		break;
	case WPC_MODE_ATTACH:
		p9222_cc_cv_charge_select(charger, capacity, false);
		break;
	case WPC_MODE_SWELL_ENTER:
	case WPC_MODE_SWELL_EXIT:
	case WPC_MODE_WAKEUP:
		if (ap_battery_mode == P9222_AP_BATTERY_IDT_MODE)
			p9222_cc_cv_charge_select(charger, capacity, false);

		if (mode != WPC_MODE_WAKEUP)
			msleep(200);
		break;
	case WPC_MODE_NORMAL:
		if (charger->pdata->otp_firmware_ver == 0x125) {
			if (charger->wpc_wakeup_wa == 0)
				p9222_cc_cv_charge_select(charger, capacity, true);
		} else {
			p9222_cc_cv_charge_select(charger, capacity, true);
		}
		break;
	default:
		break;
	};

	ret = p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE,
		&ap_battery_mode);

	if (ret < 0 || ap_battery_mode == 0xFF) {
		ret = -1;
		pr_err("%s ret = %d ap_battery_mode = 0x%x\n",
			__func__, ret, ap_battery_mode);
	}

	return ret;
}

static int p9222_power_hold_mode_monitor(struct p9222_charger_data *charger)
{
	union power_supply_propval value = {0, };
	int health;
	int status;
	int chg_mode;
	int swelling_mode;
	int slate_mode;
	int vout;

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

	pr_info("%s health(%d %d) sts(%d %d) chg_mode(%d %d) swell(%d %d)\n",
		__func__,
		charger->battery_health, health,
		charger->battery_status, status,
		charger->battery_chg_mode, chg_mode,
		charger->swelling_mode, swelling_mode);

	if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
		pr_err("%s exit by pad off!!\n", __func__);
		goto end;
	}

	if ((chg_mode == SEC_BATTERY_CHARGING_NONE) &&
		(charger->battery_chg_mode == SEC_BATTERY_CHARGING_2ND)){
		p9222_full_charge(charger);
	} else if (((health == POWER_SUPPLY_HEALTH_OVERHEAT) ||
		(health == POWER_SUPPLY_HEALTH_COLD)) &&
		(charger->battery_health == POWER_SUPPLY_HEALTH_GOOD)) {
		p9222_full_charge(charger);
	} else if ((charger->battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING) &&
		(status == POWER_SUPPLY_STATUS_CHARGING)) {
		psy_do_property(charger->pdata->battery_name, get,
			POWER_SUPPLY_EXT_PROP_COOL_DOWN_MODE, value);
		if (value.intval == 0)
			p9222_re_charge(charger);
	}

	if (charger->swelling_mode != swelling_mode) {
		if ((charger->swelling_mode != SWELLING_MODE_FULL) &&
			(swelling_mode == SWELLING_MODE_FULL)) {
			p9222_full_charge(charger);
		}
	}

	if (charger->slate_mode != slate_mode) {
		if (slate_mode == 1) {
			p9222_full_charge(charger);
		}
	}

	if (p9222_power_sharing_charge_chk(charger, charger->tx_id) == 0) {
		if ((charger->power_hold_mode == 1) && p9222_check_chg_in_status(charger)) {
			pr_err("%s abnormal chg status!! go to vout off or phm!!\n", __func__);
			charger->power_hold_mode = 0;
			p9222_full_charge(charger);

			vout = p9222_get_adc(charger, P9222_ADC_VOUT);
			if (vout < 1000)
				charger->power_hold_mode = 1;
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

static int p9222_power_sharing_charge_chk(struct p9222_charger_data *charger, int tx_id)
{
	int i;
	int ret = 0;
	int pad_type;

	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
		if (charger->pdata->charger_type[i].compatible_tx_id == tx_id) {
			pad_type = charger->pdata->charger_type[i].tx_pad_type;
			ret = (pad_type == P9222_PAD_TYPE_D2D) ? 1 : 0;
			break;
		}
	}

	return ret;
}

static int p9222_multi_charger_chk(struct p9222_charger_data *charger)
{
	int i;
	int ret = 0;
	int pad_type;

	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
		if (charger->pdata->charger_type[i].compatible_tx_id == charger->tx_id) {
			pad_type = charger->pdata->charger_type[i].tx_pad_type;
			ret = (pad_type == P9222_PAD_TYPE_MULTI) ? 1 : 0;
			break;
		}
	}

	return ret;
}

static int p9222_chg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct p9222_charger_data *charger =
		power_supply_get_drvdata(psy);
	int ret;
	u8 ap_mode;

	enum power_supply_ext_property ext_psp = psp;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			ret = p9222_get_firmware_version(charger, P9222_RX_FIRMWARE);
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
			val->intval = p9222_is_on_pad(charger);
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
				val->intval = p9222_get_adc(charger, P9222_ADC_RX_IOUT);
			} else
				val->intval = 0;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			val->intval = charger->current_now;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_AVG:
			val->intval = charger->voltage_now;
			break;
		case POWER_SUPPLY_PROP_ENERGY_NOW: /* vout */
			if(charger->pdata->ic_on_mode || (charger->wc_w_state == WPC_ON_PAD)) {
				val->intval = p9222_get_adc(charger, P9222_ADC_VOUT);
			} else
				val->intval = 0;
			break;

		case POWER_SUPPLY_PROP_ENERGY_AVG: /* vrect */
			if(charger->pdata->ic_on_mode || (charger->wc_w_state == WPC_ON_PAD)) {
				val->intval = p9222_get_adc(charger, P9222_ADC_VRECT);
			} else
				val->intval = 0;
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			pr_info("%s charger->pdata->is_charging = %d\n",
				__func__, charger->pdata->is_charging);
			val->intval = charger->pdata->is_charging;
			break;
		case POWER_SUPPLY_PROP_WPC_FREQ_STRENGTH:
			if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
				val->intval = 0;
			} else {
				if ((charger->align_check_start == 0) && (charger->power_hold_mode == 0)) {
					charger->d2d_vout_strength = p9222_get_vout_strength(charger);
					val->intval = charger->d2d_vout_strength;
				} else {
					val->intval = charger->d2d_vout_strength;
				}
			}
			break;
		case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
			switch (ext_psp) {
				case POWER_SUPPLY_EXT_PROP_WIRELESS_OP_FREQ:
	   				val->intval = p9222_get_adc(charger, P9222_ADC_OP_FRQ);
	   				break;
				case POWER_SUPPLY_EXT_PROP_WIRELESS_OP_FREQ_STRENGTH:
					if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
						val->intval = 0;
					} else {
						if ((charger->align_check_start == 0) && (charger->power_hold_mode == 0)) {
							charger->d2d_vout_strength = p9222_get_vout_strength(charger);
							val->intval = charger->d2d_vout_strength;
						} else {
							val->intval = charger->d2d_vout_strength;
						}
					}
	   				break;
				case POWER_SUPPLY_EXT_PROP_MONITOR_WORK:
					p9222_monitor_work(charger);
					val->intval = 0;
					break;
				case POWER_SUPPLY_EXT_PROP_POWER_SHARING_CHARGE:
					val->intval = (p9222_power_sharing_charge_chk(charger, charger->tx_id));
					break;
				case POWER_SUPPLY_EXT_PROP_WIRELESS_TX_ID:
					val->intval = charger->tx_id;
					break;
				case POWER_SUPPLY_EXT_PROP_WPC_VOUT_OFF:
					val->intval = charger->power_hold_mode;
					break;
				case POWER_SUPPLY_EXT_PROP_DEBUG_INFO:
					if (val->intval == DEBUG_INFO_PDETB_CNT) {
						val->intval = charger->pdetb_count;
					} else if (val->intval == DEBUG_INFO_WIRELESS_STATUS) {
						if ((charger->power_hold_mode == 0) &&
							(p9222_is_on_pad(charger) == WPC_ON_PAD)){
							p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE, &ap_mode);
							val->intval = (int)ap_mode;
						} else
							val->intval = 0;
					} else if (val->intval == DEBUG_INFO_WIRELESS_ERR_STATUS) {
						charger->err_status = p9222_get_err_status(charger);
						val->int64val = (int64_t)charger->err_status;
					}
					break;
				case POWER_SUPPLY_EXT_PROP_DARKZONE_CHECK:
					if (p9222_power_sharing_charge_chk(charger, charger->tx_id))
						val->intval = 1;
					else if (p9222_multi_charger_chk(charger))
						val->intval = 1;
					else
						val->intval = 0;
					break;
				case POWER_SUPPLY_EXT_PROP_WPC_TARGET_VOUT:
					val->intval = p9222_get_target_vout(charger);
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

static int p9222_chg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct p9222_charger_data *charger =
		power_supply_get_drvdata(psy);
	enum power_supply_ext_property ext_psp = psp;
	int vout_status = 0;
	int vrect = 0;
	int ap_mode;
	union power_supply_propval value;

	pr_debug("%s: psp=%d val=%d\n", __func__, psp, val->intval);

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if(val->intval == POWER_SUPPLY_STATUS_FULL) {
				pr_info("%s set green led \n", __func__);
				wake_lock_timeout(&charger->cs100_lock, HZ * 3);
				queue_delayed_work(charger->wqueue,
					&charger->cs100_work, 0);
			}
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			if (p9222_is_on_pad(charger) == WPC_ON_PAD) {
				msleep(250);
				pr_info("%s: Charger interrupt occured during lpm \n", __func__);
				queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
			}
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			if (val->intval == POWER_SUPPLY_HEALTH_OVERHEAT ||
				val->intval == POWER_SUPPLY_HEALTH_OVERHEATLIMIT) {
				/* if temperature is already overheat status when put on pad,
				 * first set power hold mode.
				 * if power hold mode is failed, send ept-ot packet.
				 */
				vrect = p9222_get_adc(charger, P9222_ADC_VRECT);
				if (vrect > 0)
					p9222_power_hold_mode_set(charger, 1);

				vrect = p9222_get_adc(charger, P9222_ADC_VRECT);
				if (vrect > 0) {
					pr_info("%s ept-ot \n", __func__);
					p9222_send_eop(charger, val->intval);
				}
			} else if (val->intval == POWER_SUPPLY_HEALTH_UNDERVOLTAGE) {
				pr_info("%s ept-ot health(%d)\n", __func__, val->intval);
				p9222_send_eop(charger, val->intval);
			}
			break;
		case POWER_SUPPLY_PROP_TEMP:
			charger->temperature = val->intval;
			p9222_temperature_check(charger);
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			pr_info("%s: check wpc det and set online\n", __func__);
			charger->force_wpc_det_chk = true;
			queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
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
					p9222_monitor_wdt_kick(charger);
					break;
				case POWER_SUPPLY_EXT_PROP_WIRELESS_CHARGE_MONITOR:
					p9222_ap_battery_monitor(charger, val->intval);
					p9222_power_hold_mode_monitor(charger);
					break;
				case POWER_SUPPLY_EXT_PROP_IS_RECHARGE:
					vout_status = p9222_check_chg_in_status(charger);
					pr_info("%s pwr_hold = %d vout_status = %d intval = %d\n",
						__func__, charger->power_hold_mode,
						vout_status, val->intval);

					if ((charger->power_hold_mode == 1) && (vout_status == 1) && (val->intval == 1)) {
						pr_info("%s abnormal vout status. Before PHM free, Vout must be off\n", __func__);
						wake_lock(&charger->wpc_wake_lock);
						p9222_full_charge(charger);
						wake_unlock(&charger->wpc_wake_lock);

						vout_status = p9222_check_chg_in_status(charger);
					}
					if ((vout_status == 0) && (val->intval == 1)) {
						wake_lock(&charger->wpc_wake_lock);
						p9222_re_charge(charger);
						charger->low_current_detected = 0;
						wake_unlock(&charger->wpc_wake_lock);
					}
					break;
				case POWER_SUPPLY_EXT_PROP_WPC_VOUT_OFF:
					if (val->intval == SEC_VOUT_OFF_EXIT) {
						vout_status = p9222_check_chg_in_status(charger);
						if (vout_status == 1) {
							charger->pdata->is_charging = 1;
							charger->power_hold_mode = 0;

							if (p9222_power_sharing_charge_chk(charger, charger->tx_id))
								value.intval = SEC_WIRELESS_PAD_WPC_POWER_SHARE;
							else
								value.intval = SEC_WIRELESS_PAD_WPC;

							psy_do_property(charger->pdata->charger_name, set,
								POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);
						}
					} else if (val->intval == SEC_VOUT_OFF_ENTER) {
						vout_status = p9222_get_adc(charger, P9222_ADC_VOUT);
						if (vout_status > 0) {
							wake_lock(&charger->wpc_wake_lock);
							p9222_full_charge(charger);
							wake_unlock(&charger->wpc_wake_lock);
						}
					}
					break;
				case POWER_SUPPLY_EXT_PROP_UPDATE_BATTERY_DATA:
					break;
				case POWER_SUPPLY_EXT_PROP_STORE_MODE:
					charger->store_mode = val->intval;
					break;
				case POWER_SUPPLY_EXT_PROP_HALF_BRIDGE_MODE:
					p9222_get_half_bridge_ap_mode(charger, &ap_mode);
					p9222_write_ap_mode(charger, ap_mode);
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

static void p9222_wpc_opfq_work(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, wpc_opfq_work.work);

	u16 op_fq;
	u8 pad_mode;
	union power_supply_propval value;

	p9222_reg_read(charger->client, P9222_SYS_OP_MODE_REG, &pad_mode);
	if (pad_mode == P9222_PAD_MODE_WPC) {
		op_fq = p9222_get_adc(charger, P9222_ADC_OP_FRQ);
			pr_info("%s: Operating FQ %dkHz(0x%x)\n", __func__, op_fq, op_fq);
		if (op_fq > 190) { /* wpc threshold 190kHz */
				pr_info("%s: Reset M0\n",__func__);
				p9222_reg_write(charger->client, 0x3040, 0x80); /*restart M0 */

				charger->pdata->opfq_cnt++;
				if (charger->pdata->opfq_cnt <= CMD_CNT) {
				queue_delayed_work(charger->wqueue,
					&charger->wpc_opfq_work, msecs_to_jiffies(10000));
					return;
				}
			}
	} else if (pad_mode == P9222_PAD_MODE_PMA) {
			charger->pdata->cable_type = P9222_PAD_MODE_PMA;
			value.intval = SEC_WIRELESS_PAD_PMA;
			psy_do_property(charger->pdata->wireless_name,
				set, POWER_SUPPLY_PROP_ONLINE, value);
	}

	charger->pdata->opfq_cnt = 0;
	wake_unlock(&charger->wpc_opfq_lock);
}

static void p9222_wpc_id_request_work(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, wpc_id_request_work.work);

	pr_info("%s charger->power_hold_mode = %d\n", __func__, charger->power_hold_mode);

	if (p9222_is_on_pad(charger) == WPC_ON_PAD) {
		p9222_wpc_request_tx_id(charger, 3);

		if (charger->power_hold_mode == 0)
			p9222_wpc_auth_delayed_work(charger, WPC_AUTH_DELAY_TIME);
	}

	wake_unlock(&charger->wpc_id_request_lock);
}

static void p9222_wpc_wa_work(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, wpc_wakeup_wa_work.work);
	u8 data;
	p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE, &data);

	pr_info("%s ap_mode = %x\n", __func__, data);

	charger->wpc_wakeup_wa = 0;
	p9222_ap_battery_monitor(charger, WPC_MODE_NORMAL);
	wake_unlock(&charger->wpc_wakeup_wa_lock);
}

static int p9222_get_vout_strength(struct p9222_charger_data *charger)
{
	int vout, i, ret;

	ret = charger->d2d_vout_strength;
	vout = p9222_get_adc(charger, P9222_ADC_VOUT);

	for (i = 0; i < charger->pdata->num_op_freq_list; i++) {
		if (vout >= (charger->pdata->op_freq_info[i].d2d_vout_buf)) {
			ret = charger->pdata->op_freq_info[i].d2d_vout_strength;
			break;
		}
	}

	return ret;
}

static void p9222_wpc_align_check_work(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, align_check_work.work);
	int pos;
	int vout_strength = 0;
	int vout, vrect, freq;
	int sum = 0, avr, vrect_avr;
	int vrect_sum = 0;
	int i;
	long check_time;
	bool vrect_check = false;
	union power_supply_propval value;
	struct timespec ts = {0, };

	get_monotonic_boottime(&ts);
	check_time = ts.tv_sec - charger->align_check_star_time.tv_sec;

	vout = p9222_get_adc(charger, P9222_ADC_VOUT);
	vrect = p9222_get_adc(charger, P9222_ADC_VRECT);
	freq = p9222_get_adc(charger, P9222_ADC_OP_FRQ);

	pos = charger->d2d_align_chk_cnt % P9222_OP_FREQ_CHECK_BUF_NUM;
	charger->d2d_vout_buf[pos]= vout;
	charger->d2d_vrect_buf[pos]= vrect;

	for (i = 0; i < P9222_OP_FREQ_CHECK_BUF_NUM; i++) {
		sum += charger->d2d_vout_buf[i];
		vrect_sum += charger->d2d_vrect_buf[i];
	}
	avr = sum / P9222_OP_FREQ_CHECK_BUF_NUM;
	vrect_avr = vrect_sum / P9222_OP_FREQ_CHECK_BUF_NUM;
	charger->vout_avr = avr;

	if (charger->pdata->use_ap_mode_table) {
		if (p9222_unsafe_vout_check(charger) == 0)
		    vout_strength = 100;
		else
		    vout_strength = 0;
	} else {
		for (i = 0; i < charger->pdata->num_op_freq_list; i++) {
			if (charger->vout_avr >= (charger->pdata->op_freq_info[i].d2d_vout_buf)) {
				vout_strength = charger->pdata->op_freq_info[i].d2d_vout_strength;
				break;
			}
		}
	}

	if (vout < 100)
		charger->d2d_align_detach_cnt++;
	else
		charger->d2d_align_detach_cnt = 0;

	if ((vout_strength == 0) && (charger->d2d_align_chk_cnt < 15))
		vrect_check = true;

	if ((vout_strength != charger->d2d_vout_strength) && (charger->d2d_align_chk_cnt >= 7)) {
		if ((vrect_check) && (vout_strength == 0)) {
			pr_info("%s vrect(%d %d) cnt(%d) strength_threshold(%d) \n",
				__func__, vrect, vrect_avr, charger->d2d_align_chk_cnt,
				charger->pdata->op_freq_info[0].d2d_vout_buf);
			if (vrect_avr >= charger->pdata->op_freq_info[0].d2d_vout_buf + 100) {
				vout_strength = 100;
			} else {
				vout_strength = 0;
			}

			charger->d2d_vout_strength = vout_strength;
			value.intval = vout_strength;
			pr_info("%s vrect strength = %d\n", __func__, vout_strength);
			psy_do_property(charger->pdata->battery_name,
				set, POWER_SUPPLY_PROP_WPC_FREQ_STRENGTH, value);

		} else {
			charger->d2d_vout_strength = vout_strength;
			value.intval = vout_strength;
			pr_info("%s vout_strength = %d\n", __func__, vout_strength);
			psy_do_property(charger->pdata->battery_name,
				set, POWER_SUPPLY_PROP_WPC_FREQ_STRENGTH, value);
		}
	}

	if (charger->d2d_vout_strength == 100)
		charger->max_freq_strength_cnt++;
	else
		charger->max_freq_strength_cnt = 0;

	pr_info("%s vout(%d %d) avr(%d) cnt(%d %d) check_time(%ld) max_freq_cnt(%d) vrect(%d) freq(%d)\n",
		__func__, vout, vout_strength, avr, charger->d2d_align_chk_cnt,
		charger->d2d_align_detach_cnt, check_time, charger->max_freq_strength_cnt, vrect, freq);

	charger->d2d_align_chk_cnt++;

	if (((charger->d2d_align_detach_cnt > 10) && (check_time >= 5)) ||
		(charger->max_freq_strength_cnt >= 70) ||
		(check_time >= 65)) {

		charger->d2d_vout_strength = -100;
		value.intval = charger->d2d_vout_strength;
		psy_do_property(charger->pdata->battery_name,
			set, POWER_SUPPLY_PROP_WPC_FREQ_STRENGTH, value);

		pr_info("%s end (strength:%d)\n", __func__, charger->d2d_vout_strength);
		charger->align_check_start = 0;

		if (p9222_unsafe_charge_check(charger))
			p9222_wpc_send_tx_uno_mode_disable(charger, 5);
		wake_unlock(&charger->align_check_lock);
	} else {
		if (wake_lock_active(&charger->align_check_lock)) {
			queue_delayed_work(charger->wqueue,
				&charger->align_check_work, msecs_to_jiffies(100));
		} else {
			pr_info("%s end(lock is terminated)\n", __func__);
			charger->align_check_start = 0;
		}
	}
}

static void p9222_wpc_det_work(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, wpc_det_work.work);
	int wc_w_state, prev_wc_state;
	union power_supply_propval value;
	u8 pad_mode;
	u8 vrect;
	bool det_off_force = false;
	bool det_on_force = false;

	wake_lock(&charger->wpc_wake_lock);

	wc_w_state = p9222_is_on_pad(charger);

	prev_wc_state = charger->wc_w_state;
	charger->wc_w_state = wc_w_state;

	if (charger->force_wpc_det_chk) {
		charger->force_wpc_det_chk = false;

		if ((prev_wc_state == WPC_OFF_PAD) && (wc_w_state == WPC_OFF_PAD))
			det_off_force = true;
		else if ((prev_wc_state == WPC_ON_PAD) && (wc_w_state == WPC_ON_PAD))
			det_on_force = true;

	}

	pr_info("%s: w(%d to %d) force(off:%d  on:%d)\n",
		__func__, prev_wc_state, wc_w_state, det_off_force, det_on_force);

	if (((prev_wc_state == WPC_OFF_PAD) && (wc_w_state == WPC_ON_PAD)) || det_on_force) {
		charger->pdata->vout_status = P9222_VOUT_5V;

		if (det_on_force == false)
			charger->pdetb_count++;

		/* set fod value */
		if(charger->pdata->fod_data_check)
			p9222_fod_set(charger);

		/* enable Mode Change INT */
		p9222_reg_update(charger->client, P9222_INT_ENABLE_L_REG,
			P9222_STAT_MODE_CHANGE_MASK, P9222_STAT_MODE_CHANGE_MASK);

		/* read vrect adjust */
		p9222_reg_read(charger->client, P9222_VRECT_SET_REG, &vrect);

		pr_info("%s: wpc activated\n",__func__);

		/* read pad mode */
		p9222_reg_read(charger->client, P9222_SYS_OP_MODE_REG, &pad_mode);
		if(pad_mode == P9222_SYS_MODE_PMA) {
			charger->pdata->cable_type = P9222_PAD_MODE_PMA;
			value.intval = SEC_WIRELESS_PAD_PMA;
			psy_do_property(charger->pdata->wireless_name, set,
					POWER_SUPPLY_PROP_ONLINE, value);
		} else {
			wake_lock(&charger->wpc_opfq_lock);
			queue_delayed_work(charger->wqueue,
				&charger->wpc_opfq_work, msecs_to_jiffies(10000));
		}
		charger->pdata->is_charging = 1;
	} else if (((prev_wc_state == WPC_ON_PAD) &&
		(wc_w_state == WPC_OFF_PAD)) || det_off_force) {

		p9222_wpc_det_irq_enable(charger, false);

		/* Send last tx_id to battery to cound tx_id */
		value.intval = charger->tx_id;
		psy_do_property(charger->pdata->wireless_name,
			set, POWER_SUPPLY_PROP_AUTHENTIC, value);

		if (charger->support_power_hold == 1 && charger->power_hold_mode == 1) {
			if(!delayed_work_pending(&charger->phm_free_work)) {
				int ping_duration = p9222_get_wpc_en_toggle_time(charger);

				pr_info("%s set wpc_en high(%d)\n", __func__, ping_duration);
				gpio_direction_output(charger->pdata->wpc_en, 1);
				wake_lock_timeout(&charger->phm_free_lock, ping_duration * 2);
				queue_delayed_work(charger->wqueue,
					&charger->phm_free_work, msecs_to_jiffies(ping_duration));
			}
		}

		charger->pdata->cable_type = P9222_PAD_MODE_NONE;
		charger->pdata->is_charging = 0;
		charger->pdata->vout_status = P9222_VOUT_0V;
		charger->pdata->opfq_cnt = 0;
		charger->charge_mode = P9222_CHARGE_MODE_NONE;
		charger->power_hold_mode = 0;
		charger->support_power_hold = 0;
		charger->wpc_wakeup_wa = 0;
		charger->tx_id = 0x0;
		charger->phm_set_fail = 0;

		charger->current_now = 0;
		charger->voltage_now = 0;

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

		psy_do_property(charger->pdata->battery_name, get,
			POWER_SUPPLY_PROP_ONLINE, value);

		pr_info("%s cable_type = %d\n", __func__, value.intval);
		if (is_wireless_type(value.intval)) {
			value.intval = SEC_WIRELESS_PAD_NONE;
			psy_do_property(charger->pdata->charger_name, set,
				POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);
		}

		cancel_delayed_work(&charger->wpc_isr_work);

		if(delayed_work_pending(&charger->wpc_opfq_work)) {
			wake_unlock(&charger->wpc_opfq_lock);
			cancel_delayed_work(&charger->wpc_opfq_work);
		}

		if (wake_lock_active(&charger->wpc_auth_check_lock))
			wake_unlock(&charger->wpc_auth_check_lock);

		if (delayed_work_pending(&charger->wpc_auth_check_work))
			cancel_delayed_work(&charger->wpc_auth_check_work);

		if (wake_lock_active(&charger->wpc_id_request_lock))
			wake_unlock(&charger->wpc_id_request_lock);

		if (delayed_work_pending(&charger->wpc_id_request_work))
			cancel_delayed_work(&charger->wpc_id_request_work);
	}

	wake_unlock(&charger->wpc_wake_lock);
}

static enum alarmtimer_restart p9222_wpc_ph_mode_alarm(struct alarm *alarm, ktime_t now)
{
	struct p9222_charger_data *charger = container_of(alarm,
				struct p9222_charger_data, polling_alarm);

	union power_supply_propval value;

	value.intval = false;
	psy_do_property(charger->pdata->charger_name, set,
		POWER_SUPPLY_PROP_PRESENT, value);
	pr_info("%s: wpc ph mode ends.\n", __func__);
	wake_unlock(&charger->wpc_wake_lock);

	return ALARMTIMER_NORESTART;
}

static int p9222_compatible_tx_check(struct p9222_charger_data *charger, int id)
{
	int i;
	pr_info("%s input id = %x\n", __func__, id);

	for (i = 0; i < charger->pdata->num_compatible_tx; i++) {
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

static void p9222_wpc_send_rx_id(struct p9222_charger_data *charger, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++) {
		pr_info("%s send RX-ID(%x) to TX (cnt %d)\n", __func__, charger->pdata->rx_id, i);
		p9222_send_packet(charger, P9222_HEADER_AFC_CONF, P9222_RX_DATA_COM_RX_ID, (u8)charger->pdata->rx_id);

		if (cnt > 1)
			msleep(100);
	}
}

static void p9222_wpc_request_tx_id(struct p9222_charger_data *charger, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++) {
		pr_info("%s requst TX-ID to TX (cnt %d)\n", __func__, i);
		p9222_send_packet(charger, P9222_HEADER_AFC_CONF, P9222_RX_DATA_COM_REQ_TX_ID, 0x0);

		if (cnt > 1)
			msleep(300);
	}

	p9222_wpc_send_rx_id(charger, 3);
}

static void p9222_curr_measure_work(struct work_struct *work)
{
	struct p9222_charger_data *charger = container_of(work,
		struct p9222_charger_data, curr_measure_work.work);
	union power_supply_propval value = {0,};
	int i = 0;
	int acok_cnt = 0;
	bool valid_acok = false;

	pr_info("%s\n", __func__);

	while (i < 100) {
		if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
			pr_info("%s pad is off!\n", __func__);
			break;
		}

		if (p9222_check_chg_in_status(charger) == true) {
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

	p9222_write_ap_mode(charger, P9222_AP_BATTERY_CURR_MEASURE_MODE);
	charger->curr_measure_mode = 1;

	psy_do_property(charger->pdata->charger_name, set,
		POWER_SUPPLY_EXT_PROP_FORCED_JIG_MODE, value);
}

#if defined(P9222_D2D_ALIGN_GUIDE)
static void p9222_align_check_init(struct p9222_charger_data *charger)
{
	charger->d2d_align_chk_cnt = 0;
	charger->d2d_align_detach_cnt = 0;
	charger->d2d_vout_strength = -1;
	charger->vout_avr = 0;
	charger->max_freq_strength_cnt = 0;

	charger->align_check_star_time.tv_sec = 0;
	charger->align_check_star_time.tv_nsec = 0;

	memset(charger->d2d_vout_buf, 0, sizeof(charger->d2d_vout_buf));
	memset(charger->d2d_vrect_buf, 0, sizeof(charger->d2d_vrect_buf));

}
#endif

static bool p9222_is_tx_support_bt(struct p9222_charger_data *charger)
{
	int i;
	for (i = 0; i < charger->pdata->num_of_bt_support; i++) {
		if (charger->pdata->tx_id_with_bt_support[i] == charger->tx_id) {
			pr_info("%s: tx id(%d) is support\n", __func__, charger->tx_id);
			return true;
		}
	}
	return false;
}

static int p9222_get_device_id(struct p9222_charger_data *charger, u8 *device_id, int size)
{
	int i;
	union power_supply_propval value = {0};

	if(!p9222_is_tx_support_bt(charger)) {
		pr_err("%s: tx_id(%d) don't support bt\n", __func__, charger->tx_id);
		return -EINVAL;
	}

	psy_do_property(charger->pdata->battery_name, get,
			POWER_SUPPLY_EXT_PROP_DEVICE_ID, value);
	if(value.strval == NULL) {
		pr_err("%s: device_id is null\n", __func__);
		return -EFAULT;
	}

	pr_info("%s: device_id is (%s)\n", __func__, value.strval);

	for(i = 0; i < size; i++) {
		char hex[3] = {0};
		hex[0] = value.strval[2*i];
		hex[1] = value.strval[2*i + 1];
		if(sscanf(hex, "%2x", (u32*)&device_id[i]) != 1) {
			return -ENODATA;
		}
	}
	return 0;
}

static bool p9222_send_device_id(struct p9222_charger_data *charger)
{
	u8 device_id[32] = {0};
	if(p9222_get_device_id(charger, device_id, 6)) {
		pr_err("%s: can't get device id\n", __func__);
		return false;
	}
	p9222_send_multi_packet(charger, 0x58, device_id[1], &device_id[2], 4);
	pr_info("%s: head=%x, cmd=%x, data=%x%x%x%x\n",
		__func__,
		0x58, 							//head
		device_id[1], 						//cmd
		device_id[2],device_id[3],device_id[4],device_id[5]);	//data

	return true;
}

static void p9222_wpc_isr_work(struct work_struct *work)
{
	static int wpc_det_irq_count = 0;
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, wpc_isr_work.work);

	u8 cmd_data, val_data, data;
	union power_supply_propval value;
	struct timespec ts = {0, };
	long diff_ms = 0;

	if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
		pr_info("%s wc_w_state is 0. exit wpc_isr_work.\n",__func__);
		wake_unlock(&charger->wpc_tx_id_check_lock);
		return;
	}

	if (p9222_compatible_tx_check(charger, charger->tx_id)) {
		if (charger->power_hold_mode == 1) {
			pr_info("%s currently phm mode. skip tx_id check!!\n", __func__);
			wake_unlock(&charger->wpc_tx_id_check_lock);
			return;
		}
	}

	p9222_reg_read(charger->client, P9222_TX_DATA_COMMAND, &cmd_data);
	p9222_reg_read(charger->client, P9222_TX_DATA_VALUE0, &val_data);

	pr_info("%s cmd(%x) val(%x)\n", __func__, cmd_data, val_data);

	if (cmd_data == P9222_TX_DATA_COM_CHG_STOP) {
		pr_info("%s skip chg stop fsk msg!!\n", __func__);
		wake_unlock(&charger->wpc_tx_id_check_lock);
		return;
	}

	if(cmd_data == P9222_TX_DATA_COM_TRY_SEND) {
		if(val_data == P9222_TX_DATA_VAL_TRY_SEND_BT_ADDRESS) {
			if(!p9222_send_device_id(charger)) {
				pr_err("%s: fail to send bt address\n", __func__);
			}
			wake_unlock(&charger->wpc_tx_id_check_lock);
			return;
		}
	}

	if (cmd_data == P9222_TX_DATA_COM_TX_ID)
		charger->tx_id = val_data;

	charger->tx_id_checked = 0;

	if (cmd_data == P9222_TX_DATA_COM_TX_ID) {
		if (p9222_compatible_tx_check(charger, val_data)) {
			pr_info("%s data = 0x%x, compatible TX pad\n", __func__, val_data);
			value.intval = 1;
			psy_do_property(charger->pdata->charger_name, set,
				POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);
			psy_do_property(charger->pdata->battery_name, set,
				POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);

			if (charger->support_power_hold)
				charger->charger_type = P9222_CHARGER_TYPE_COMPATIBLE;
			else
				charger->charger_type = P9222_CHARGER_TYPE_MULTI;

			value.intval = 0;
			charger->incompatible_tx = 0;
			psy_do_property(charger->pdata->battery_name, set,
				POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);

			if (p9222_power_sharing_charge_chk(charger, charger->tx_id)) {
				get_monotonic_boottime(&ts);
				diff_ms = (((ts.tv_sec - charger->uno_off_ts.tv_sec) * 1000) +
					(ts.tv_nsec/1000000)) - (charger->uno_off_ts.tv_nsec/1000000);
				pr_info("%s diff(%ld)\n", __func__, diff_ms);

				if (((diff_ms <= 1500)) && (charger->uno_off_cnt > 0)) {
					wpc_det_irq_count = charger->pdetb_count;
					p9222_wpc_send_tx_uno_mode_disable(charger, 30);
					wake_unlock(&charger->wpc_tx_id_check_lock);
					return;
				}

#if defined(P9222_D2D_ALIGN_GUIDE)
				psy_do_property(charger->pdata->battery_name, get,
					POWER_SUPPLY_PROP_ONLINE, value);
				pr_info("%s cable_type(%d)\n", __func__, value.intval);

				if ((charger->align_check_start == 0) &&
					(value.intval != SEC_BATTERY_CABLE_WPC_POWER_SHARE)) {
					charger->align_check_start = 1;
					p9222_align_check_init(charger);
					get_monotonic_boottime(&charger->align_check_star_time);

					wake_lock_timeout(&charger->align_check_lock, HZ * 70);
					queue_delayed_work(charger->wqueue,
						&charger->align_check_work, msecs_to_jiffies(100));
				}
#endif
				value.intval = SEC_WIRELESS_PAD_WPC_POWER_SHARE;
				psy_do_property(charger->pdata->charger_name, set,
					POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);
				pr_info("%s power share!!\n", __func__);
			} else {
				if (p9222_multi_charger_chk(charger) == 1) {
					if (p9222_check_chg_in_status(charger) == true) {
						value.intval = SEC_WIRELESS_PAD_WPC;
						psy_do_property(charger->pdata->charger_name, set,
							POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);
					}
				} else {
					value.intval = SEC_WIRELESS_PAD_WPC;
					psy_do_property(charger->pdata->charger_name, set,
						POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);
				}
			}

			charger->tx_id_checked = 1;
			charger->low_current_detected = 0;
		} else if (val_data == P9222_FACTORY_MODE_TX_ID) {
			pr_info("%s id 0x%x current measure TX pad\n", __func__, val_data);
			charger->charger_type = P9222_CHARGER_TYPE_COMPATIBLE;
			schedule_delayed_work(&charger->curr_measure_work,
				msecs_to_jiffies(2000));
		} else {
			charger->charger_type = P9222_CHARGER_TYPE_INCOMPATIBLE;
			pr_info("%s incompatible TX pad\n", __func__);
		}

		p9222_reg_read(charger->client, P9222_INT_L_REG, &data);
		if (data & P9222_STAT_TX_DATA_RECEIVED_MASK) {
			p9222_reg_write(charger->client, P9222_INT_CLEAR_L_REG,
				P9222_INT_TX_DATA_RECEIVED);
		}

	} else if (cmd_data == 0x19) {
		pr_info("%s incompatible TX pad\n", __func__);
		charger->charger_type = P9222_CHARGER_TYPE_INCOMPATIBLE;

		p9222_reg_read(charger->client, P9222_INT_L_REG, &data);
		if (data & P9222_STAT_TX_DATA_RECEIVED_MASK) {
			p9222_reg_write(charger->client, P9222_INT_CLEAR_L_REG,
				P9222_INT_TX_DATA_RECEIVED);
		}
	} else if (cmd_data == P9222_TX_DATA_COM_RX_ID) {
		p9222_wpc_send_rx_id(charger, 3);
		wake_unlock(&charger->wpc_tx_id_check_lock);
		return;
	}
	p9222_wpc_request_tx_id(charger, 3);
	wake_unlock(&charger->wpc_tx_id_check_lock);

	p9222_wpc_auth_delayed_work(charger, WPC_AUTH_DELAY_TIME);
}

static irqreturn_t p9222_wpc_det_irq_thread(int irq, void *irq_data)
{
	struct p9222_charger_data *charger = irq_data;
	int on_pad = p9222_is_on_pad(charger);

	pr_info("%s on_pad(%d)\n",__func__, on_pad);

	while(charger->wqueue == NULL) {
		pr_err("%s wait wqueue created!!\n", __func__);
		msleep(10);
	}

	if (delayed_work_pending(&charger->wpc_det_work)) {
		pr_info("%s wpc_det_work pended. cancel and send workq!!\n", __func__);
		cancel_delayed_work(&charger->wpc_det_work);
	}

	if (on_pad == WPC_OFF_PAD) {
		queue_delayed_work(charger->wqueue,
			&charger->wpc_det_work, msecs_to_jiffies(40));
	} else {
		queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
	}

	return IRQ_HANDLED;
}

static void p9222_enable_vbat_monitoring(struct p9222_charger_data *charger)
{
	if (charger->pdata->can_vbat_monitoring) {
		p9222_reg_write(charger->client, P9222_VRECT_SET_REG, 9);
		p9222_reg_update(charger->client, P9222_WPC_FLAG_REG,
			P9222_VBAT_MONITORING_MODE, P9222_VBAT_MONITORING_MODE_MASK);
	}
}

static irqreturn_t p9222_wpc_irq_thread(int irq, void *irq_data)
{
	struct p9222_charger_data *charger = irq_data;
	int ret;
	u8 irq_src[2];
	u8 reg_data;
	u8 data1, data2;

	wake_lock(&charger->wpc_wake_lock);
	mutex_lock(&charger->charger_lock);

	pr_info("%s\n", __func__);

	/* check again firmare version support vbat monitoring */
	if (!charger->pdata->otp_firmware_ver) {
		ret = p9222_get_firmware_version(charger, P9222_RX_FIRMWARE);
		if (ret > 0) {
			pr_debug("%s rx major firmware version 0x%x\n", __func__, ret);
			charger->pdata->otp_firmware_ver = ret;
		}
	}

	p9222_enable_vbat_monitoring(charger);

	ret = p9222_reg_read(charger->client, P9222_INT_L_REG, &irq_src[0]);
	ret = p9222_reg_read(charger->client, P9222_INT_H_REG, &irq_src[1]);

	if (ret < 0) {
		pr_err("%s: Failed to read interrupt source: %d\n",
			__func__, ret);
		wake_unlock(&charger->wpc_wake_lock);
		mutex_unlock(&charger->charger_lock);
		return IRQ_NONE;
	}

	pr_info("%s: interrupt source(0x%x)\n", __func__, irq_src[1] << 8 | irq_src[0]);

	if(irq_src[0] & P9222_STAT_MODE_CHANGE_MASK) {
		pr_info("%s MODE CHANGE IRQ ! \n", __func__);
		ret = p9222_reg_read(charger->client, P9222_SYS_OP_MODE_REG, &reg_data);
	}

	if(irq_src[0] & P9222_STAT_VOUT_MASK) {
		pr_info("%s Vout IRQ ! \n", __func__);
		ret = p9222_reg_read(charger->client, P9222_INT_STATUS_L_REG, &reg_data);
		if (reg_data & P9222_INT_STAT_VOUT)
			charger->vout_status = 1;
		else
			charger->vout_status = 0;
		pr_info("%s vout_status = %d\n", __func__, charger->vout_status);
	}

	if(irq_src[0] & P9222_STAT_TX_DATA_RECEIVED_MASK) {
		pr_info("%s TX RECIVED IRQ ! \n", __func__);

		p9222_reg_read(charger->client, P9222_TX_DATA_COMMAND, &data1);
		p9222_reg_read(charger->client, P9222_TX_DATA_VALUE0, &data2);
		pr_info("%s P9222_TX_DATA_COMMAND = %x tx_id = %x\n", __func__, data1, data2);

		if(!delayed_work_pending(&charger->wpc_isr_work) &&
			charger->pdata->cable_type != P9222_PAD_MODE_WPC_AFC ) {
			wake_lock_timeout(&charger->wpc_tx_id_check_lock, HZ * 5);
			schedule_delayed_work(&charger->wpc_isr_work, msecs_to_jiffies(0));
		}
	}

	if(irq_src[1] & P9222_STAT_OVER_CURR_MASK) {
		pr_info("%s OVER CURRENT IRQ ! \n", __func__);
	}

	if(irq_src[1] & P9222_STAT_OVER_TEMP_MASK) {
		pr_info("%s OVER TEMP IRQ ! \n", __func__);
	}

	if(irq_src[1] & P9222_STAT_TX_CONNECT_MASK) {
		pr_info("%s TX CONNECT IRQ ! \n", __func__);
		charger->pdata->tx_status = SEC_TX_POWER_TRANSFER;
	}

	if ((irq_src[1] << 8 | irq_src[0]) != 0) {
		msleep(5);

		/* clear intterupt */
		p9222_reg_write(charger->client, P9222_INT_CLEAR_L_REG, irq_src[0]);
		p9222_reg_write(charger->client, P9222_INT_CLEAR_H_REG, irq_src[1]);
		p9222_set_cmd_reg(charger, 0x20, P9222_CMD_CLEAR_INT_MASK); // command
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


static struct device_attribute p9222_attributes[] = {
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
	struct p9222_charger_data *charger =
		power_supply_get_drvdata(psy);

	const ptrdiff_t offset = attr - p9222_attributes;
	u8 data;
	int i, count = 0;

	switch (offset) {
		case WPC_FW_VER:
			{
				int ret =0;
				int version =0;

				ret = p9222_get_firmware_version(charger, P9222_RX_FIRMWARE);

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
				ibuck = p9222_get_adc(charger, P9222_ADC_RX_IOUT);
				ibuck -= P9222_I_OUT_OFFSET;
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
				if (p9222_reg_read(charger->client, charger->addr+i, &data) < 0) {
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
	struct p9222_charger_data *charger =
		power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - p9222_attributes;
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
				if (p9222_reg_write(charger->client, charger->addr, data) < 0)
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

	for (i = 0; i < ARRAY_SIZE(p9222_attributes); i++) {
		rc = device_create_file(dev, &p9222_attributes[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	while (i--)
		device_remove_file(dev, &p9222_attributes[i]);
create_attrs_succeed:
	return rc;
}

static void p9222_wpc_auth_check_work(struct work_struct *work)
{
	struct p9222_charger_data *charger =
		container_of(work, struct p9222_charger_data, wpc_auth_check_work.work);

	union power_supply_propval value;
	u8 cmd_data, val_data;
	int ret;

	pr_info("%s wc_w_state=%d\n", __func__, charger->wc_w_state);

	if (p9222_is_on_pad(charger) == WPC_OFF_PAD) {
		wake_unlock(&charger->wpc_auth_check_lock);
		return;
	}

	if (p9222_check_chg_in_status(charger) == false) {
		pr_err("%s acok is not valid!!\n", __func__);
		goto out;
	}

	if (charger->tx_id_checked == 0) {
		p9222_reg_read(charger->client, P9222_TX_DATA_COMMAND, &cmd_data);
		p9222_reg_read(charger->client, P9222_TX_DATA_VALUE0, &val_data);

		if (cmd_data == P9222_TX_DATA_COM_TX_ID)
			charger->tx_id = val_data;

		pr_info("%s cmd_data = %x val_data = %x\n", __func__, cmd_data, val_data);

		if (cmd_data == P9222_TX_DATA_AICL_RESET)
			goto out;

		if (cmd_data == P9222_TX_DATA_COM_TX_ID) {
			if (p9222_compatible_tx_check(charger, val_data)) {
				pr_info("%s data = 0x%x, compatible TX pad\n", __func__, val_data);
				value.intval = 1;
				psy_do_property(charger->pdata->charger_name, set,
					POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);
				psy_do_property(charger->pdata->battery_name, set,
					POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX, value);

				if (charger->support_power_hold)
					charger->charger_type = P9222_CHARGER_TYPE_COMPATIBLE;
				else
					charger->charger_type = P9222_CHARGER_TYPE_MULTI;

				value.intval = 0;
				charger->incompatible_tx = 0;
				psy_do_property(charger->pdata->battery_name, set,
					POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);


				if (p9222_power_sharing_charge_chk(charger, charger->tx_id)) {
					psy_do_property(charger->pdata->battery_name, get,
						POWER_SUPPLY_PROP_ONLINE, value);
					pr_info("%s cable_type(%d)\n", __func__, value.intval);

					if ((charger->align_check_start == 0) &&
						(value.intval != SEC_BATTERY_CABLE_WPC_POWER_SHARE)) {
						value.intval = SEC_WIRELESS_PAD_WPC_POWER_SHARE;
						psy_do_property(charger->pdata->charger_name, set,
							POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);
						pr_info("%s power share!!\n", __func__);
					}
				}

			} else {
				value.intval = 1;
				charger->incompatible_tx = 1;
				psy_do_property(charger->pdata->battery_name, set,
					POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);
				charger->charger_type = P9222_CHARGER_TYPE_INCOMPATIBLE;
			}
		}
	}

	charger->tx_id_checked = 0;
	p9222_wpc_request_tx_id(charger, 3);
out:
	if (p9222_compatible_tx_check(charger, charger->tx_id) == 0) {
		if (charger->pdata->support_legacy_pad == 0) {
			charger->wtd_err_cnt++;
			p9222_send_watchdog_err_packet(charger);
		}

		value.intval = 1;
		charger->incompatible_tx = 1;
		psy_do_property(charger->pdata->battery_name, set,
			POWER_SUPPLY_EXT_PROP_INCOMPATIBLE_WPC, value);
		charger->charger_type = P9222_CHARGER_TYPE_INCOMPATIBLE;
		if (charger->tx_id == 0)
			charger->tx_id = UNKNOWN_TX_ID;
	}

	ret = p9222_reg_read(charger->client, P9222_AP_BATTERY_MODE, &val_data);
	if ((ret >= 0) && (val_data == P9222_AP_BATTERY_IDT_MODE))
		p9222_ap_battery_monitor(charger, WPC_MODE_ATTACH);

	wake_unlock(&charger->wpc_auth_check_lock);
}

static int
p9222_wpc_auth_delayed_work(struct p9222_charger_data *charger, int sec)
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

static struct p9222_chg_type_desc charger_type_compatilbe = {
	.charger_type = P9222_CHARGER_TYPE_COMPATIBLE,
	.charger_type_name = "compatible",
	.cc_chg = p9222_compatible_cc_charge,
	.cv_chg = p9222_compatible_cv_charge,
	.full_chg = p9222_power_hold_full_charge,
	.re_chg = p9222_power_hold_re_charge,
};

static struct p9222_chg_type_desc charger_type_incompatilbe = {
	.charger_type = P9222_CHARGER_TYPE_INCOMPATIBLE,
	.charger_type_name = "incompatible",
	.cc_chg = p9222_incompatible_cc_charge,
	.cv_chg = p9222_incompatible_cv_charge,
	.full_chg = p9222_power_hold_full_charge,
	.re_chg = p9222_power_hold_re_charge,
};

static struct p9222_chg_type_desc charger_type_multi = {
	.charger_type = P9222_CHARGER_TYPE_MULTI,
	.charger_type_name = "multi",
	.cc_chg = p9222_compatible_cc_charge,
	.cv_chg = p9222_compatible_cv_charge,
	.full_chg = p9222_normal_full_charge,
	.re_chg = p9222_normal_re_charge,
};

static int p9222_wpc_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
	struct p9222_charger_data *charger =
		container_of(nb, struct p9222_charger_data, wpc_nb);
	union power_supply_propval value = {0, };

	pr_info("%s action=%lu, attached_dev=%d\n", __func__, action, attached_dev);

	switch (attached_dev) {
	case ATTACHED_DEV_TA_MUIC:
		if (action == MUIC_NOTIFY_CMD_ATTACH) {
			p9222_reg_update(charger->client,
				P9222_WPC_FLAG_REG, 0x0, P9222_WATCHDOG_DIS_MASK);
			p9222_get_firmware_version(charger, P9222_RX_FIRMWARE);
			p9222_get_firmware_version(charger, P9222_TX_FIRMWARE);
		}
		break;
	case ATTACHED_DEV_WIRELESS_TA_MUIC:
	case ATTACHED_DEV_WIRELESS_POWER_SHARE_MUIC:
		if (action == MUIC_NOTIFY_CMD_DETACH) {
			pr_info("%s MUIC_NOTIFY_CMD_DETACH\n", __func__);
			charger->op_freq_offset = 0;
			p9222_power_hold_mode_monitor(charger);
		} else if (action == MUIC_NOTIFY_CMD_ATTACH) {
			queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
			p9222_wpc_det_irq_enable(charger, true);
			p9222_enable_vbat_monitoring(charger);

			wake_lock_timeout(&charger->wpc_id_request_lock, 5 * HZ);
			schedule_delayed_work(&charger->wpc_id_request_work, 0);

				if (p9222_power_sharing_charge_chk(charger, charger->tx_id)) {
					value.intval = SEC_WIRELESS_PAD_WPC_POWER_SHARE;
					psy_do_property(charger->pdata->charger_name, set,
						POWER_SUPPLY_EXT_PROP_WPC_ONLINE, value);

					pr_info("%s power share!!\n", __func__);
				}
		}
		break;
	default:
		break;
	}

	return 0;
}

static int p9222_parse_dt(struct device *dev, p9222_charger_platform_data_t *pdata)
{

	struct device_node *np = dev->of_node;
	const u32 *p;
	int len, ret, i;
	enum of_gpio_flags irq_gpio_flags;
	/*changes can be added later, when needed*/

	ret = of_property_read_string(np,
		"p9222,charger_name", (char const **)&pdata->charger_name);
	if (ret) {
		pr_info("%s: Charger name is Empty\n", __func__);
		pdata->charger_name = "sec-charger";
	}

	ret = of_property_read_string(np,
		"p9222,fuelgauge_name", (char const **)&pdata->fuelgauge_name);
	if (ret) {
		pr_info("%s: Fuelgauge name is Empty\n", __func__);
		pdata->fuelgauge_name = "sec-fuelgauge";
	}

	ret = of_property_read_string(np,
		"p9222,battery_name", (char const **)&pdata->battery_name);
	if (ret) {
		pr_info("%s: battery_name is Empty\n", __func__);
		pdata->battery_name = "battery";
	}

	ret = of_property_read_string(np,
		"p9222,wireless_name", (char const **)&pdata->wireless_name);
	if (ret) {
		pr_info("%s: wireless_name is Empty\n", __func__);
		pdata->wireless_name = "wireless";
	}

	ret = of_property_read_string(np, "p9222,wireless_charger_name",
		(char const **)&pdata->wireless_charger_name);
	if (ret) {
		pr_info("%s: wireless_charger_name is Empty\n", __func__);
		pdata->wireless_charger_name = "wpc";
	}

	/* wpc_det */
	ret = pdata->wpc_det = of_get_named_gpio_flags(np, "p9222,wpc_det",
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
	ret = pdata->wpc_int = of_get_named_gpio_flags(np, "p9222,wpc_int",
			0, &irq_gpio_flags);
	if (ret < 0) {
		dev_err(dev, "%s : can't wpc_int\r\n", __FUNCTION__);
	} else {
		pdata->irq_wpc_int = gpio_to_irq(pdata->wpc_int);
		pr_info("%s wpc_int = 0x%x, irq_wpc_int = 0x%x \n",__func__,
			pdata->wpc_int, pdata->irq_wpc_int);
	}

	/* wpc_en */
	ret = pdata->wpc_en = of_get_named_gpio_flags(np, "p9222,wpc_en",
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

	ret = of_property_read_u32(np, "p9222,cc_cv_threshold",
		&pdata->cc_cv_threshold);
	if (ret < 0) {
		pr_err("%s error reading cc_cv_threshold %d\n", __func__, ret);
		pdata->cc_cv_threshold = 88;
	}

	ret = of_property_read_u32(np, "p9222,rx_id",
		&pdata->rx_id);
	if (ret < 0) {
		pr_err("%s error reading rx_id %d\n", __func__, ret);
		pdata->rx_id = 0;
	}

	pdata->can_vbat_monitoring = of_property_read_bool(np, "p9222,vbat-monitoring");

	p = of_get_property(np, "p9222,tx_id_with_bt_support", &len);
	if(p){
		pdata->num_of_bt_support = len / sizeof(u32);
		pdata->tx_id_with_bt_support = kzalloc(len, GFP_KERNEL);
		if(pdata->tx_id_with_bt_support == NULL) {
			pr_err("%s: line(%d) Memory is not enough.\n", __func__, __LINE__);
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(np, "p9222,tx_id_with_bt_support",
				 (u32*)pdata->tx_id_with_bt_support, len/sizeof(u32));
		if(ret) {
			pr_err("%s: failed to read p9222,tx_id_with_bt_support: %d\n",
			                                                    __func__, ret);
			kfree(pdata->tx_id_with_bt_support);
			pdata->tx_id_with_bt_support = NULL;
			pdata->num_of_bt_support = 0;
		} else {
			for(i = 0; i < pdata->num_of_bt_support; i++) {
				pr_info("tx_id_with_bt_support is %x\n",
				                            pdata->tx_id_with_bt_support[i]);
			}
		}
	}

	p = of_get_property(np, "p9222,charger_type", &len);
	if (p) {
		pdata->num_compatible_tx = len / sizeof(struct p9222_charger_type);
		pdata->charger_type = kzalloc(len, GFP_KERNEL);
		if(pdata->charger_type == NULL) {
			pr_err("%s: line(%d) Memory is not enough.\n", __func__, __LINE__);
			return -ENOMEM;
		}
		ret = of_property_read_u32_array(np, "p9222,charger_type",
				 (u32 *)pdata->charger_type, len/sizeof(u32));
		if (ret) {
			pr_err("%s: failed to read p9222,charger_type: %d\n", __func__, ret);
			kfree(pdata->charger_type);
			pdata->charger_type = NULL;
			pdata->num_compatible_tx = 0;
		} else {
			for (i = 0; i < pdata->num_compatible_tx; i++)
				pr_info("charger_type tx_id = 0x%02x power_hold = %d tx_pad_type(%d) cc(%x) cv(%x) half(%x) phm_toggle(%d)\n",
					pdata->charger_type[i].compatible_tx_id,
					pdata->charger_type[i].support_power_hold,
					pdata->charger_type[i].tx_pad_type,
					pdata->charger_type[i].cc_ap_mode,
					pdata->charger_type[i].cv_ap_mode,
					pdata->charger_type[i].half_bridge_ap_mode,
					pdata->charger_type[i].phm_free_toggle_time);
		}
	}

	p = of_get_property(np, "p9222,ex_low_vout_level", &len);
	if (p) {
		pdata->num_ex_low_vout = len / sizeof(struct p9222_ex_low_vout_info);
		pdata->ex_low_vout = kzalloc(len, GFP_KERNEL);
		if(pdata->ex_low_vout == NULL) {
			pr_err("%s: line(%d) Memory is not enough.\n", __func__, __LINE__);
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(np, "p9222,ex_low_vout_level",
				 (u32 *)pdata->ex_low_vout, len/sizeof(u32));
		if (ret) {
			pr_err("%s: failed to read p9222,ex_low_vout_level: %d\n", __func__, ret);
			kfree(pdata->ex_low_vout);
			pdata->ex_low_vout = NULL;
			pdata->num_ex_low_vout = 0;
		} else {
			for (i = 0; i < pdata->num_ex_low_vout; i++)
				pr_info("ex_low_vout_level tx_id = 0x%02x vout = %d\n",
					pdata->ex_low_vout[i].tx_id,
					pdata->ex_low_vout[i].low_vout);
		}
	}

	p = of_get_property(np, "p9222,sec_mode_data", &len);
	if (p) {
		pdata->num_sec_mode = len / sizeof(struct p9222_sec_mode_config_data);
		pdata->sec_mode_config_data = kzalloc(len, GFP_KERNEL);
		if(pdata->sec_mode_config_data == NULL) {
			pr_err("%s: line(%d) Memory is not enough.\n", __func__, __LINE__);
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(np, "p9222,sec_mode_data",
				 (u32 *)pdata->sec_mode_config_data, len/sizeof(u32));
		if (ret) {
			pr_err("%s: failed to read p9222,sec_mode_data: %d\n", __func__, ret);
			kfree(pdata->sec_mode_config_data);
			pdata->sec_mode_config_data = NULL;
			pdata->num_sec_mode = 0;
		}
		pr_err("%s: num_sec_mode : %d\n", __func__, pdata->num_sec_mode);
		for (len = 0; len < pdata->num_sec_mode; ++len)
			pr_err("mode %d : vrect:%d, vout:%d\n",
				len, pdata->sec_mode_config_data[len].vrect, pdata->sec_mode_config_data[len].vout);
	} else
		pr_err("%s: there is no p9222,sec_mode_data\n", __func__);

	p = of_get_property(np, "p9222,op_freq", &len);
	if (p) {
		pdata->num_op_freq_list = len / sizeof(struct p9222_op_freq_info);
		pdata->op_freq_info = kzalloc(len, GFP_KERNEL);
		if(pdata->op_freq_info == NULL) {
			pr_err("%s: line(%d) Memory is not enough.\n", __func__, __LINE__);
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(np, "p9222,op_freq",
				 (u32 *)pdata->op_freq_info, len/sizeof(u32));
		if (ret) {
			pr_err("%s: failed to read p9222,op_freq: %d\n", __func__, ret);
			kfree(pdata->op_freq_info);
			pdata->op_freq_info = NULL;
			pdata->num_op_freq_list = 0;
		} else {
			for (i = 0; i < pdata->num_op_freq_list; i++)
				pr_info("op_freq_info op_freq = %d d2d_vout_strength(%d)\n",
					pdata->op_freq_info[i].d2d_vout_buf,
					pdata->op_freq_info[i].d2d_vout_strength);
		}
	}

	p = of_get_property(np, "p9222,ap_mode_table", &len);
	if (p) {
		pdata->num_ap_mode_table = len / sizeof(struct p9222_ap_table_info);
		pdata->ap_mode_table_info = kzalloc(len, GFP_KERNEL);
		if(pdata->ap_mode_table_info == NULL) {
			pr_err("%s: line(%d) Memory is not enough.\n", __func__, __LINE__);
			return -ENOMEM;
		}

		ret = of_property_read_u32_array(np, "p9222,ap_mode_table",
				 (u32 *)pdata->ap_mode_table_info, len/sizeof(u32));
		if (ret) {
			pr_err("%s: failed to read p9222,ap_mode_table: %d\n", __func__, ret);
			kfree(pdata->ap_mode_table_info);
			pdata->ap_mode_table_info = NULL;
			pdata->num_ap_mode_table = 0;
			pdata->use_ap_mode_table = false;
		} else {
			pdata->use_ap_mode_table = true;
			for (i = 0; i < pdata->num_ap_mode_table; i++)
				pr_info("ap_mode_table_info ap_mode = %x vout(%d) vrect(%d) margin(%d)\n",
					pdata->ap_mode_table_info[i].ap_mode,
					pdata->ap_mode_table_info[i].vout,
					pdata->ap_mode_table_info[i].vrect,
					pdata->ap_mode_table_info[i].batt_mode_margin);
		}
	} else {
		pdata->use_ap_mode_table = false;
	}

	ret = of_property_read_u32(np, "p9222,tx-off-high-temp",
		&pdata->tx_off_high_temp);
	if (ret) {
		pr_info("%s : TX-OFF-TEMP is Empty\n", __func__);
		pdata->tx_off_high_temp = INT_MAX;
	}

	ret = of_property_read_u32(np, "p9222,ping_duration",
		&pdata->ping_duration);
	if (ret) {
		pr_info("%s : ping_duration Empty\n", __func__);
		pdata->ping_duration = 350;
	}

	ret = of_property_read_u32(np, "p9222,ps_wpc_en_toggle_time",
		&pdata->ps_wpc_en_toggle_time);
	if (ret) {
		pr_info("%s : ps_wpc_en_toggle_time Empty\n", __func__);
		pdata->ps_wpc_en_toggle_time = 510;
	}

	ret = of_property_read_u32(np, "p9222,support_legacy_pad",
		&pdata->support_legacy_pad);
	if (ret) {
		pr_info("%s : support_legacy_pad Empty\n", __func__);
		pdata->support_legacy_pad = 0;
	}

	ret = of_property_read_u32(np, "p9222,support_low_iout_detect",
		&pdata->support_low_iout_detect);
	if (ret) {
		pr_info("%s : support_low_iout_detect Empty\n", __func__);
		pdata->support_low_iout_detect = 0;
	}

	ret = of_property_read_u32(np, "p9222,boot_ap_mode",
		&pdata->boot_ap_mode);
	if (ret)
		pdata->boot_ap_mode = P9222_AP_BATTERY_BOOT_MODE;
	pr_info("%s : boot_ap_mode(%x)\n", __func__, pdata->boot_ap_mode);

	ret = of_property_read_u32(np, "p9222,low_vout_level",
		&pdata->low_vout_level);
	if (ret) {
		pr_info("%s : low_vout_level Empty\n", __func__);
		pdata->low_vout_level = 4500;
	}

	ret = of_property_read_u32(np, "p9222,enable_batt_mode_headroom_margin",
		&pdata->enable_batt_mode_headroom_margin);
	if (ret)
		pdata->enable_batt_mode_headroom_margin = 1;
	pr_info("%s : enable_batt_mode_headroom_margin(%x)\n", __func__, pdata->enable_batt_mode_headroom_margin);

	return 0;
}

static const struct power_supply_desc wpc_power_supply_desc = {
	.name = "wpc",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = sec_charger_props,
	.num_properties = ARRAY_SIZE(sec_charger_props),
	.get_property = p9222_chg_get_property,
	.set_property = p9222_chg_set_property,
};

static int p9222_charger_probe(
						struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct p9222_charger_data *charger;
	p9222_charger_platform_data_t *pdata = client->dev.platform_data;
	struct power_supply_config wpc_cfg = {};
	int ret = 0;

	dev_info(&client->dev,
		"%s: p9222 Charger Driver Loading\n", __func__);

	pdata = devm_kzalloc(&client->dev, sizeof(p9222_charger_platform_data_t), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	ret = p9222_parse_dt(&client->dev, pdata);
	if (ret < 0)
		return ret;

	client->irq = pdata->irq_wpc_int;
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
	charger->pdata->cable_type = P9222_PAD_MODE_NONE;
	charger->pdata->is_charging = 0;

	charger->pdata->tx_firmware_result = P9222_FW_RESULT_DOWNLOADING;
	charger->pdata->tx_status = 0;
	charger->pdata->cs100_status = 0;
	charger->pdata->vout_status = P9222_VOUT_0V;
	charger->pdata->opfq_cnt = 0;
	charger->pdata->watchdog_test = false;
	charger->pdata->charging_mode = SEC_INITIAL_MODE;
	charger->wc_w_state = p9222_is_on_pad(charger);

	charger->power_hold_mode = 0;
	charger->support_power_hold = 0;
	charger->curr_measure_mode = 0;
	charger->charge_mode = P9222_CHARGE_MODE_NONE;
	charger->store_mode = 0;
	charger->tx_id_checked = 0;
	charger->tx_id = 0x0;
	charger->force_wpc_det_chk = false;
	charger->irq_wpc_det_enabled = 0;
	charger->pdetb_count = 0;

	charger->uno_off_cnt = 0;
	charger->under_voltage_cnt = 0;
	charger->ot_cnt = 0;
	charger->wtd_err_cnt = 0;

	charger->low_current_detected = 0;

	mutex_init(&charger->io_lock);
	mutex_init(&charger->charger_lock);

	wake_lock_init(&charger->wpc_wake_lock, WAKE_LOCK_SUSPEND,
			"wpc_wakelock");
	wake_lock_init(&charger->wpc_tx_id_check_lock, WAKE_LOCK_SUSPEND,
			"wpc_tx_id_check_lock");
	wake_lock_init(&charger->wpc_auth_check_lock, WAKE_LOCK_SUSPEND,
			"wpc_auth_check_lock");

	charger->temperature = 250;

	/* wpc_det */
	if (charger->pdata->irq_wpc_det >= 0) {
		INIT_DELAYED_WORK(&charger->wpc_det_work, p9222_wpc_det_work);
		INIT_DELAYED_WORK(&charger->wpc_opfq_work, p9222_wpc_opfq_work);

		ret = request_threaded_irq(charger->pdata->irq_wpc_det,
				NULL, p9222_wpc_det_irq_thread,
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

		charger->irq_wpc_det_enabled = 1;
		charger->irq_wpc_det_wakeup_enabled = 1;

	}

	/* wpc_irq */
	if (charger->client->irq) {
		p9222_reg_update(charger->client, P9222_INT_ENABLE_L_REG,
			P9222_INT_STAT_VOUT, P9222_STAT_VOUT_MASK);

		INIT_DELAYED_WORK(&charger->wpc_isr_work, p9222_wpc_isr_work);
		ret = gpio_request(pdata->wpc_int, "wpc-irq");
		if (ret) {
			pr_err("%s failed requesting gpio(%d)\n",
				__func__, pdata->wpc_int);
			goto err_supply_unreg;
		}
		ret = request_threaded_irq(charger->client->irq,
				NULL, p9222_wpc_irq_thread,
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
				"wpc-irq", charger);
		if (ret) {
			dev_err(&client->dev,
				"%s: Failed to Reqeust IRQ\n", __func__);
			goto err_supply_unreg;
		}
	}
	INIT_DELAYED_WORK(&charger->curr_measure_work, p9222_curr_measure_work);
	INIT_DELAYED_WORK(&charger->wpc_init_work, p9222_wpc_init);
	schedule_delayed_work(&charger->wpc_init_work, msecs_to_jiffies(1000));

	INIT_DELAYED_WORK(&charger->wpc_auth_check_work, p9222_wpc_auth_check_work);
	INIT_DELAYED_WORK(&charger->power_hold_chk_work, p9222_wpc_power_hold_check);
	INIT_DELAYED_WORK(&charger->wpc_id_request_work, p9222_wpc_id_request_work);

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


	charger->wqueue = create_singlethread_workqueue("p9222_workqueue");
	if (!charger->wqueue) {
		pr_err("%s: Fail to Create Workqueue\n", __func__);
		goto err_pdata_free;
	}

	wake_lock_init(&charger->wpc_opfq_lock, WAKE_LOCK_SUSPEND,
		"wpc_opfq_lock");

	wake_lock_init(&charger->power_hold_chk_lock, WAKE_LOCK_SUSPEND,
		"power_hold_chk_lock");

	wake_lock_init(&charger->wpc_id_request_lock, WAKE_LOCK_SUSPEND,
		"wpc_id_request_lock");

	wake_lock_init(&charger->wpc_wakeup_wa_lock, WAKE_LOCK_SUSPEND,
		"wpc_wakeup_wa_lock");

	INIT_DELAYED_WORK(&charger->wpc_wakeup_wa_work, p9222_wpc_wa_work);
	charger->wpc_wakeup_wa = 0;

	wake_lock_init(&charger->align_check_lock, WAKE_LOCK_SUSPEND,
		"align_check_lock");
	INIT_DELAYED_WORK(&charger->align_check_work, p9222_wpc_align_check_work);

	wake_lock_init(&charger->phm_free_lock, WAKE_LOCK_SUSPEND,
		"phm_free_lock");
	INIT_DELAYED_WORK(&charger->phm_free_work, p9222_phm_free_check);

	wake_lock_init(&charger->cs100_lock, WAKE_LOCK_SUSPEND,
		"cs100_lock");
	INIT_DELAYED_WORK(&charger->cs100_work, p9222_cs100_work);

	wake_lock_init(&charger->unsafe_voltage_chk_lock, WAKE_LOCK_SUSPEND,
		"unsafe_voltage_chk_lock");

	charger->last_poll_time = ktime_get_boottime();
	alarm_init(&charger->polling_alarm, ALARM_BOOTTIME,
		p9222_wpc_ph_mode_alarm);

	charger->charge_cb_func[P9222_CHARGER_TYPE_COMPATIBLE] = &charger_type_compatilbe;
	charger->charge_cb_func[P9222_CHARGER_TYPE_INCOMPATIBLE] = &charger_type_incompatilbe;
	charger->charge_cb_func[P9222_CHARGER_TYPE_MULTI] = &charger_type_multi;

#if defined(CONFIG_MUIC_NOTIFIER)
	muic_notifier_register(&charger->wpc_nb, p9222_wpc_handle_notification,
			       MUIC_NOTIFY_DEV_CHARGER);
#endif

	get_monotonic_boottime(&charger->uno_off_ts);

	dev_info(&client->dev,
		"%s: p9222 Charger Driver Loaded\n", __func__);

	//device_init_wakeup(charger->dev, 1);
	return 0;
err_pdata_free:
	power_supply_unregister(charger->psy_chg);
err_supply_unreg:
	wake_lock_destroy(&charger->wpc_wake_lock);
	wake_lock_destroy(&charger->wpc_tx_id_check_lock);
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
static int p9222_charger_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct p9222_charger_data *charger = i2c_get_clientdata(i2c);
	union power_supply_propval value;

	if (device_may_wakeup(charger->dev)){
		enable_irq_wake(charger->pdata->irq_wpc_int);
		enable_irq_wake(charger->pdata->irq_wpc_det);
	}

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_ONLINE, value);
	pr_info("%s cable_type = %d\n", __func__, value.intval);

	if (is_wireless_type(value.intval)) {
		p9222_wpc_det_irq_enable(charger, true);
	} else {
		p9222_wpc_det_irq_enable(charger, false);
	}

	enable_irq_wake(charger->client->irq);

	disable_irq(charger->pdata->irq_wpc_int);

	return 0;
}

static int p9222_charger_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct p9222_charger_data *charger = i2c_get_clientdata(i2c);

	charger->wpc_wakeup_wa = 0;

	if (device_may_wakeup(charger->dev)) {
		disable_irq_wake(charger->pdata->irq_wpc_int);
		disable_irq_wake(charger->pdata->irq_wpc_det);
	}
	enable_irq(charger->pdata->irq_wpc_int);

	disable_irq_wake(charger->client->irq);

	if ((p9222_is_on_pad(charger) == WPC_ON_PAD) &&
		(charger->power_hold_mode == 0)) {
		if (charger->charge_mode == P9222_CHARGE_MODE_CV) {
			if (charger->pdata->otp_firmware_ver == 0x125) {
				wake_lock(&charger->wpc_wakeup_wa_lock);
				charger->wpc_wakeup_wa = 1;
				p9222_ap_battery_monitor(charger, WPC_MODE_BOOT);
				schedule_delayed_work(&charger->wpc_wakeup_wa_work, msecs_to_jiffies(1000));
			}
		}
	}
	return 0;
}
#else
#define p9222_charger_suspend NULL
#define p9222_charger_resume NULL
#endif

static void p9222_charger_shutdown(struct i2c_client *client)
{
	struct p9222_charger_data *charger = i2c_get_clientdata(client);

	pr_debug("%s \n", __func__);

	if (p9222_is_on_pad(charger) == WPC_ON_PAD) {
		if (charger->power_hold_mode) {
			pr_info("%s exit power hold mode before poweroff\n", __func__);
			p9222_re_charge(charger);
		}

		p9222_ap_battery_monitor(charger, WPC_MODE_IDT);
	}
}


static const struct i2c_device_id p9222_charger_id[] = {
	{"p9222-charger", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, p9222_charger_id);

static struct of_device_id p9222_i2c_match_table[] = {
	{ .compatible = "p9222,i2c"},
	{},
};

const struct dev_pm_ops p9222_charger_pm = {
	.suspend = p9222_charger_suspend,
	.resume = p9222_charger_resume,
};

static struct i2c_driver p9222_charger_driver = {
	.driver = {
		.name	= "p9222-charger",
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm	= &p9222_charger_pm,
#endif /* CONFIG_PM */
		.of_match_table = p9222_i2c_match_table,
	},
	.shutdown	= p9222_charger_shutdown,
	.probe	= p9222_charger_probe,
	//.remove	= p9222_charger_remove,
	.id_table	= p9222_charger_id,
};

static int __init p9222_charger_init(void)
{
	pr_debug("%s \n",__func__);
	return i2c_add_driver(&p9222_charger_driver);
}

static void __exit p9222_charger_exit(void)
{
	pr_debug("%s \n",__func__);
	i2c_del_driver(&p9222_charger_driver);
}

module_init(p9222_charger_init);
module_exit(p9222_charger_exit);

MODULE_DESCRIPTION("Samsung p9222 Charger Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
