/*
 * zp3390_charger.h
 * Samsung zp3390 Charger Header
 *
 * Copyright (C) 2015 Samsung Electronics, Inc.
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

#ifndef __zp3390_CHARGER_H
#define __zp3390_CHARGER_H __FILE__

#include <linux/mfd/core.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/battery/sec_charging_common.h>

#define ZP3390_DELAY_FOR_TRANSCATION		50
#define ZP3390_DELAY_FOR_POST_TRANSCATION	10

/* REGISTER MAPS */
#define ZP3390_CHIP_ID_L_REG					0x00
#define ZP3390_CHIP_ID_H_REG					0x01
#define ZP3390_CHIP_REVISION_REG				0x02
#define ZP3390_CUSTOMER_ID_REG				0x03
#define ZP3390_FW_MAJOR_REV_L_REG		0x04
#define ZP3390_FW_MAJOR_REV_H_REG		0x05
#define ZP3390_FW_MINOR_REV_L_REG		0x06
#define ZP3390_FW_MINOR_REV_H_REG		0x07

#define ZP3390_INT_STATUS_L_REG				0x34
#define ZP3390_INT_STATUS_H_REG				0x35
#define ZP3390_INT_L_REG						0x36
#define ZP3390_INT_H_REG						0x37
#define ZP3390_INT_ENABLE_L_REG				0x38
#define ZP3390_INT_ENABLE_H_REG				0x39
#define ZP3390_CHG_STATUS_REG				0x3A
#define ZP3390_END_POWER_TRANSFER_REG		0x3B
#define ZP3390_ADC_VOUT_L_REG				0x3C
#define ZP3390_ADC_VOUT_H_REG				0x3D
#define ZP3390_VOUT_SET_REG					0x3E
#define ZP3390_VRECT_MARGIN_REG					0x3F
#define ZP3390_ADC_VRECT_L_REG				0x40
#define ZP3390_ADC_VRECT_H_REG				0x41
#define ZP3390_ADC_TX_ISENSE_L_REG			0x42
#define ZP3390_ADC_TX_ISENSE_H_REG			0x43
#define ZP3390_ADC_RX_IOUT_L_REG				0x44
#define ZP3390_ADC_RX_IOUT_H_REG				0x45
#define ZP3390_ADC_DIE_TEMP_L_REG			0x46
#define ZP3390_ADC_DIE_TEMP_H_REG			0x47
#define ZP3390_OP_FREQ_L_REG					0x48
#define ZP3390_OP_FREQ_H_REG					0x49
#define ZP3390_ILIM_SET_REG					0x4A
#define ZP3390_ADC_ALLIGN_X_REG				0x4B
#define ZP3390_ADC_ALLIGN_Y_REG				0x4C
#define ZP3390_SYS_OP_MODE_REG				0x4D
#define ZP3390_COMMAND_REG					0x4E

#define ZP3390_PACKET_HEADER					0x50
#define ZP3390_RX_DATA_COMMAND				0x51
#define ZP3390_RX_DATA_VALUE0				0x52
#define ZP3390_RX_DATA_VALUE1				0x53
#define ZP3390_RX_DATA_VALUE2				0x54
#define ZP3390_RX_DATA_VALUE3				0x55
#define ZP3390_INT_CLEAR_L_REG				0x56
#define ZP3390_INT_CLEAR_H_REG				0x57
#define ZP3390_TX_DATA_COMMAND				0x58
#define ZP3390_TX_DATA_VALUE0				0x59
#define ZP3390_TX_DATA_VALUE1				0x5A
#define ZP3390_TX_DATA_VALUE2				0x5B

#define ZP3390_WPC_FOD_0A_REG				0x68	// ~0x75

#define ZP3390_WPC_FLAG_REG					0x80

#define ZP3390_VBAT_L_REG					0x88
#define ZP3390_VBAT_H_REG					0x89
#define ZP3390_ADC_VBAT_L_REG				0x8A
#define ZP3390_ADC_VBAT_H_REG				0x8B

#define ZP3390_AP_BATTERY_MODE				0x9A
#define ZP3390_VRECT_TARGET_VOL_L			0xB0
#define ZP3390_VRECT_TARGET_VOL_H			0xB1

#define ZP3390_NUM_FOD_REG					12

#define ZP3390_AP_BATTERY_IDT_MODE					0x00
#define ZP3390_AP_BATTERY_BOOT_MODE					0x01
/* ZP3390_AP_BATTERY_BATT_MODE Vout = Vbat + 0.3V Vrect = Vout + 0x1V */
#define ZP3390_AP_BATTERY_BATT_MODE					0x02
#define ZP3390_AP_BATTERY_CC_CV_MODE					0x06
#define ZP3390_AP_BATTERY_INCOMPATIBLE_CC_CV_MODE	0x07
#define ZP3390_AP_BATTERY_CC_MODE					0x10
#define ZP3390_AP_BATTERY_INCOMPATIBLE_CC_MODE		0x11
#define ZP3390_AP_BATTERY_WAKEUP_MODE				0x12
#define ZP3390_AP_BATTERY_INCOMPATIBLE_PHP_MODE		0x13
#define ZP3390_AP_BATTERY_CURR_MEASURE_MODE			0xF0



/* Chip Revision and Font Register, Chip_Rev (0x02) */
#define ZP3390_CHIP_GRADE_MASK				0x0f
#define ZP3390_CHIP_REVISION_MASK			0xf0

/* Status Registers, Status_L (0x34), Status_H (0x35) */
#define ZP3390_STAT_VOUT_SHIFT				7
#define ZP3390_STAT_STAT_VRECT_SHIFT			6
#define ZP3390_STAT_MODE_CHANGE_SHIFT		5
#define ZP3390_STAT_TX_DATA_RECEIVED_SHIFT	4
#define ZP3390_STAT_OVER_TEMP_SHIFT			2
#define ZP3390_STAT_OVER_VOL_SHIFT			1
#define ZP3390_STAT_OVER_CURR_SHIFT			0
#define ZP3390_STAT_VOUT_MASK				(1 << ZP3390_STAT_VOUT_SHIFT)
#define ZP3390_STAT_STAT_VRECT_MASK			(1 << ZP3390_STAT_STAT_VRECT_SHIFT)
#define ZP3390_STAT_MODE_CHANGE_MASK			(1 << ZP3390_STAT_MODE_CHANGE_SHIFT)
#define ZP3390_STAT_TX_DATA_RECEIVED_MASK	(1 << ZP3390_STAT_TX_DATA_RECEIVED_SHIFT)
#define ZP3390_STAT_OVER_TEMP_MASK			(1 << ZP3390_STAT_OVER_TEMP_SHIFT)
#define ZP3390_STAT_OVER_VOL_MASK			(1 << ZP3390_STAT_OVER_VOL_SHIFT)
#define ZP3390_STAT_OVER_CURR_MASK			(1 << ZP3390_STAT_OVER_CURR_SHIFT)

#define ZP3390_STAT_TX_OVER_CURR_SHIFT		6
#define ZP3390_STAT_TX_OVER_TEMP_SHIFT		5
#define ZP3390_STAT_TX_FOD_SHIFT				4
#define ZP3390_STAT_TX_CONNECT_SHIFT			3

#define ZP3390_STAT_TX_OVER_CURR_MASK		(1 << ZP3390_STAT_TX_OVER_CURR_SHIFT)
#define ZP3390_STAT_TX_OVER_TEMP_MASK		(1 << ZP3390_STAT_TX_OVER_TEMP_SHIFT)
#define ZP3390_STAT_TX_FOD_MASK				(1 << ZP3390_STAT_TX_FOD_SHIFT)
#define ZP3390_STAT_TX_CONNECT_MASK			(1 << ZP3390_STAT_TX_CONNECT_SHIFT)

/* Interrupt Registers, INT_L (0x36), INT_H (0x37) */
#define ZP3390_INT_STAT_VOUT					ZP3390_STAT_VOUT_MASK
#define ZP3390_INT_STAT_VRECT				ZP3390_STAT_STAT_VRECT_MASK
#define ZP3390_INT_MODE_CHANGE				ZP3390_STAT_MODE_CHANGE_MASK
#define ZP3390_INT_TX_DATA_RECEIVED			ZP3390_STAT_TX_DATA_RECEIVED_MASK
#define ZP3390_INT_OVER_VOLT					ZP3390_STAT_OVER_VOL_MASK
#define ZP3390_INT_OVER_CURR					ZP3390_STAT_OVER_CURR_MASK

#define ZP3390_INT_OVER_TEMP					ZP3390_STAT_OVER_TEMP_MASK
#define ZP3390_INT_TX_OVER_CURR				ZP3390_STAT_TX_OVER_CURR_MASK
#define ZP3390_INT_TX_OVER_TEMP				ZP3390_STAT_TX_OVER_TEMP_MASK
#define ZP3390_INT_TX_FOD					ZP3390_STAT_TX_FOD_MASK
#define ZP3390_INT_TX_CONNECT				ZP3390_STAT_TX_CONNECT_MASK

/* End of Power Transfer Register, EPT (0x3B) (RX only) */
#define ZP3390_EPT_UNKNOWN					0
#define ZP3390_EPT_END_OF_CHG				1
#define ZP3390_EPT_INT_FAULT					2
#define ZP3390_EPT_OVER_TEMP					3
#define ZP3390_EPT_OVER_VOL					4
#define ZP3390_EPT_OVER_CURR					5
#define ZP3390_EPT_BATT_FAIL					6
#define ZP3390_EPT_RECONFIG					7

/* System Operating Mode Register,Sys_Op_Mode (0x4D) */
#define ZP3390_SYS_MODE_INIT					0
#define ZP3390_SYS_MODE_WPC					1
#define ZP3390_SYS_MODE_PMA					2
#define ZP3390_SYS_MODE_MISSING_BACK			3
#define ZP3390_SYS_MODE_TX					4
#define ZP3390_SYS_MODE_WPC_RX				5
#define ZP3390_SYS_MODE_PMA_RX				6
#define ZP3390_SYS_MODE_MISSING				7


/* Command Register, COM(0x4E) */
#define ZP3390_CMD_SS_WATCHDOG_SHIFT			7
#define ZP3390_CMD_CLEAR_INT_SHIFT			5
#define ZP3390_CMD_SEND_CHG_STS_SHIFT		4
#define ZP3390_CMD_SEND_EOP_SHIFT			3
#define ZP3390_CMD_SET_TX_MODE_SHIFT			2
#define ZP3390_CMD_TOGGLE_LDO_SHIFT			1
#define ZP3390_CMD_SEND_RX_DATA_SHIFT		0
#define ZP3390_CMD_SS_WATCHDOG_MASK			(1 << ZP3390_CMD_SS_WATCHDOG_SHIFT)
#define ZP3390_CMD_CLEAR_INT_MASK			(1 << ZP3390_CMD_CLEAR_INT_SHIFT)
#define ZP3390_CMD_SEND_CHG_STS_MASK			(1 << ZP3390_CMD_SEND_CHG_STS_SHIFT)
#define ZP3390_CMD_SEND_EOP_MASK				(1 << ZP3390_CMD_SEND_EOP_SHIFT)
#define ZP3390_CMD_SET_TX_MODE_MASK			(1 << ZP3390_CMD_SET_TX_MODE_SHIFT)
#define ZP3390_CMD_TOGGLE_LDO_MASK			(1 << ZP3390_CMD_TOGGLE_LDO_SHIFT)
#define ZP3390_CMD_SEND_RX_DATA_MASK			(1 << ZP3390_CMD_SEND_RX_DATA_SHIFT)

#define ZP3390_CMD_SEND_RX_DATA				ZP3390_CMD_SEND_RX_DATA_MASK

/* Proprietary Packet Header Register, PPP_Header(0x50) */
#define ZP3390_HEADER_SIGNAL_STRENGTH		0x01
#define ZP3390_HEADER_END_POWER_TRANSFER		0x02
#define ZP3390_HEADER_CONTROL_ERROR			0x03
#define ZP3390_HEADER_RECEIVED_POWER			0x04
#define ZP3390_HEADER_CHARGE_STATUS			0x05
#define ZP3390_HEADER_POWER_CTR_HOLD_OFF		0x06
#define ZP3390_HEADER_AFC_CONF				0x28
#define ZP3390_HEADER_CONFIGURATION			0x51
#define ZP3390_HEADER_IDENTIFICATION			0x71
#define ZP3390_HEADER_EXTENDED_IDENT			0x81

/* RX Data Command Register, RX Data_COM (0x51) */
#define ZP3390_RX_DATA_COM_UNKNOWN			0x00
#define ZP3390_RX_DATA_COM_REQ_TX_ID			0x01
#define ZP3390_RX_DATA_COM_CHG_STATUS		0x05
#define ZP3390_RX_DATA_COM_AFC_SET			0x06
#define ZP3390_RX_DATA_COM_AFC_DEBOUNCE		0x07
#define ZP3390_RX_DATA_COM_SID_TAG			0x08
#define ZP3390_RX_DATA_COM_SID_TOKEN			0x09
#define ZP3390_RX_DATA_COM_TX_STANBY			0x0A
#define ZP3390_RX_DATA_COM_LED_CTRL			0x0B
#define ZP3390_RX_DATA_COM_REQ_AFC			0x0C
#define ZP3390_RX_DATA_COM_FAN_CTRL			0x0D

/* TX Data Command Register, TX Data_COM (0x58) */
#define ZP3390_TX_DATA_COM_UNKNOWN			0x00
#define ZP3390_TX_DATA_COM_TX_ID				0x01
#define ZP3390_TX_DATA_COM_AFC_TX			0x02
#define ZP3390_TX_DATA_COM_ACK				0x03
#define ZP3390_TX_DATA_COM_NAK				0x04

/* Flag Register, (0x80) */
#define ZP3390_WATCHDOG_DIS_SHIFT			5
#define ZP3390_WATCHDOG_DIS_MASK			(1 << ZP3390_WATCHDOG_DIS_SHIFT)
#define ZP3390_VBAT_MONITORING_MODE_SHIFT	7
#define ZP3390_VBAT_MONITORING_MODE_MASK		(1 << ZP3390_VBAT_MONITORING_MODE_SHIFT)

#define ZP3390_VBAT_MONITORING_MODE			ZP3390_VBAT_MONITORING_MODE_MASK

/* END POWER TRANSFER CODES IN WPC */
#define ZP3390_EPT_CODE_UNKOWN				0x00
#define ZP3390_EPT_CODE_CHARGE_COMPLETE		0x01
#define ZP3390_EPT_CODE_INTERNAL_FAULT		0x02
#define ZP3390_EPT_CODE_OVER_TEMPERATURE		0x03
#define ZP3390_EPT_CODE_OVER_VOLTAGE			0x04
#define ZP3390_EPT_CODE_OVER_CURRENT			0x05
#define ZP3390_EPT_CODE_BATTERY_FAILURE		0x06
#define ZP3390_EPT_CODE_RECONFIGURE			0x07
#define ZP3390_EPT_CODE_NO_RESPONSE			0x08

#define ZP3390_POWER_MODE_MASK				(0x1 << 0)
#define ZP3390_SEND_USER_PKT_DONE_MASK		(0x1 << 7)
#define ZP3390_SEND_USER_PKT_ERR_MASK		(0x3 << 5)
#define ZP3390_SEND_ALIGN_MASK				(0x1 << 3)
#define ZP3390_SEND_EPT_CC_MASK				(0x1 << 0)
#define ZP3390_SEND_EOC_MASK					(0x1 << 0)

#define ZP3390_PTK_ERR_NO_ERR				0x00
#define ZP3390_PTK_ERR_ERR					0x01
#define ZP3390_PTK_ERR_ILLEGAL_HD			0x02
#define ZP3390_PTK_ERR_NO_DEF				0x03

#define ZP3390_VOUT_5V_VAL					0x0f
#define ZP3390_VOUT_6V_VAL					0x19
#define ZP3390_VOUT_7V_VAL					0x23
#define ZP3390_VOUT_8V_VAL					0x2d
#define ZP3390_VOUT_9V_VAL					0x37

#define ZP3390_VOUT_TO_VAL(x)				((x - 3500) / 100)

#define ZP3390_FW_RESULT_DOWNLOADING			2
#define ZP3390_FW_RESULT_PASS				1
#define ZP3390_FW_RESULT_FAIL				0

#define  VOUT_LOWER_4_2V        0x07
#define  VOUT_LOWER_4_3V        0x08
#define  VOUT_LOWER_4_4V        0x09
#define  VOUT_LOWER_4_5V        0x0A
#define  VOUT_LOWER_4_6V	0x0B
#define  VOUT_LOWER_4_7V	0x0C
#define  VOUT_LOWER_4_8V	0x0D


enum {
    ZP3390_EVENT_IRQ = 0,
    ZP3390_IRQS_NR,
};

enum {
    ZP3390_PAD_MODE_NONE = 0,
    ZP3390_PAD_MODE_WPC,
    ZP3390_PAD_MODE_WPC_AFC,
    ZP3390_PAD_MODE_PMA,
    ZP3390_PAD_MODE_TX,
};

/* vout settings */
enum {
    ZP3390_VOUT_0V = 0,
    ZP3390_VOUT_5V,
    ZP3390_VOUT_6V,
    ZP3390_VOUT_9V,
    ZP3390_VOUT_CV_CALL,
    ZP3390_VOUT_CC_CALL,
    ZP3390_VOUT_9V_STEP,
};

enum {
    ZP3390_ADC_VOUT = 0,
    ZP3390_ADC_VRECT,
    ZP3390_ADC_TX_ISENSE,
    ZP3390_ADC_RX_IOUT,
    ZP3390_ADC_DIE_TEMP,
    ZP3390_ADC_ALLIGN_X,
    ZP3390_ADC_ALLIGN_Y,
    ZP3390_ADC_OP_FRQ,
    ZP3390_ADC_VBAT_RAW,
    ZP3390_ADC_VBAT,
};

enum {
	ZP3390_END_SIG_STRENGTH = 0,
	ZP3390_END_POWER_TRANSFER,
	ZP3390_END_CTR_ERROR,
	ZP3390_END_RECEIVED_POWER,
	ZP3390_END_CHARGE_STATUS,
	ZP3390_POWER_CTR_HOLD_OFF,
	ZP3390_AFC_CONF_5V,
	ZP3390_AFC_CONF_9V,
	ZP3390_CONFIGURATION,
	ZP3390_IDENTIFICATION,
	ZP3390_EXTENDED_IDENT,
	ZP3390_LED_CONTROL_ON,
	ZP3390_LED_CONTROL_OFF,
	ZP3390_FAN_CONTROL_ON,
	ZP3390_FAN_CONTROL_OFF,
	ZP3390_REQUEST_AFC_TX,
};

enum {
	ZP3390_WATCHDOG = 0,
	ZP3390_EOP_HIGH_TEMP,
	ZP3390_VOUT_ON,
	ZP3390_VOUT_OFF,
};

enum zp3390_irq_source {
	TOP_INT = 0,
};

enum zp3390_chip_rev {
	ZP3390_A_GRADE_IC = 0,
	ZP3390_B_GRADE_IC,
	ZP3390_C_GRADE_IC,
	ZP3390_D_GRADE_IC,
};

enum zp3390_irq {
	ZP3390_IRQ_STAT_VOUT = 0,
	ZP3390_IRQ_STAT_VRECT,
	ZP3390_IRQ_MODE_CHANGE,
	ZP3390_IRQ_TX_DATA_RECEIVED,
	ZP3390_IRQ_OVER_VOLT,
	ZP3390_IRQ_OVER_CURR,
	ZP3390_IRQ_OVER_TEMP,
	ZP3390_IRQ_TX_OVER_CURR,
	ZP3390_IRQ_TX_OVER_TEMP,
	ZP3390_IRQ_TX_FOD,
	ZP3390_IRQ_TX_CONNECT,
	ZP3390_IRQ_NR,
};

struct zp3390_irq_data {
	int mask;
	enum zp3390_irq_source group;
};

enum zp3390_firmware_mode {
	ZP3390_RX_FIRMWARE = 0,
	ZP3390_TX_FIRMWARE,
};

enum zp3390_read_mode {
	ZP3390_IC_GRADE = 0,
	ZP3390_IC_VERSION,
	ZP3390_IC_VENDOR,
};

enum zp3390_headroom {
	ZP3390_HEADROOM_0 = 0,
	ZP3390_HEADROOM_1, /* 0.277V */
	ZP3390_HEADROOM_2, /* 0.497V */
	ZP3390_HEADROOM_3, /* 0.650V */
};

enum zp3390_charger_pad_type {
	ZP3390_CHARGER_TYPE_COMPATIBLE = 0,
	ZP3390_CHARGER_TYPE_INCOMPATIBLE,
	ZP3390_CHARGER_TYPE_MULTI,
	ZP3390_CHARGER_TYPE_NUM_MAX
};

enum zp3390_charge_mode_type {
	ZP3390_CHARGE_MODE_NONE = 0,
	ZP3390_CHARGE_MODE_CC,
	ZP3390_CHARGE_MODE_CV,
	ZP3390_CHARGE_MODE_OFF
};


enum sec_wpc_setting_mode {
	SEC_INITIAL_MODE =0,
	SEC_SWELLING_MODE,
	SEC_LOW_BATT_MODE,
	SEC_MID_BATT_MODE,
	SEC_HIGH_BATT_MODE,
	SEC_VERY_HIGH_BATT_MODE,
	SEC_FULL_NONE_MODE,
	SEC_FULL_NORMAL_MODE,
	SEC_SWELLING_LDU_MODE,
	SEC_LOW_BATT_LDU_MODE,
	SEC_FULL_NONE_LDU_MODE,
	SEC_FULL_NORMAL_LDU_MODE,
	SEC_INCOMPATIBLE_TX_MODE,
	SEC_VBAT_MONITORING_DISABLE_MODE,
	SEC_INCOMPATIBLE_TX_HIGH_BATT_MODE,
	SEC_INCOMPATIBLE_TX_FULL_MODE,
	SEC_WPC_MODE_NUM,
};

/* for mesurement current leakage */
#define ZP3390_FACTORY_MODE_TX_ID			0x5F

struct zp3390_ppp_info {
	u8 header;
	u8 rx_data_com;
	u8 data_val[4];
	int data_size;
};

#define DECLARE_IRQ(idx, _group, _mask)		\
	[(idx)] = { .group = (_group), .mask = (_mask) }
static const struct zp3390_irq_data zp3390_irqs[] = {
	DECLARE_IRQ(ZP3390_IRQ_STAT_VOUT,	TOP_INT, 1 << 0),
};

struct zp3390_sec_mode_config_data {
	int vrect;
	int vout;
};

struct zp3390_charger_type {
	int compatible_tx_id;
	int support_power_hold;
};

struct zp3390_chg_type_desc;

struct zp3390_charger_platform_data {
	char *wireless_charger_name;
	char *charger_name;
	char *fuelgauge_name;
	char *wireless_name;
	char *battery_name;

	int wpc_det;
	int irq_wpc_det;
	int wpc_int;
	int irq_wpc_int;
	u32 irq_gpio_flags;
	int wpc_en;
	int cc_cv_threshold; /* cc/cv threshold capacity level */
	int cs100_status;
	int vout_status;
	int wireless_cc_cv;
	int siop_level;
	int cable_type;
	bool default_voreg;
	int is_charging;
	int *fod_data;
	int fod_data_check;
	bool ic_on_mode;
	int hw_rev_changed; /* this is only for noble/zero2 */
	int on_mst_wa;		/* this is only for Zero2, There is zinitix leakage */
	int otp_firmware_result;
	int tx_firmware_result;
	int wc_ic_grade;
	int wc_ic_rev;
	int otp_firmware_ver;
	int tx_firmware_ver;
	int vout;
	int vrect;
	int tx_status;
	int wpc_cv_call_vout;
	int wpc_cc_call_vout;
	int opfq_cnt;
	bool watchdog_test;
	unsigned int charging_mode;

	bool can_vbat_monitoring;
	struct zp3390_sec_mode_config_data *sec_mode_config_data;
	int num_sec_mode;
	int num_compatible_tx;
	//int *compatible_tx_id;
	struct zp3390_charger_type *charger_type;

	int tx_off_high_temp;
	int ping_duration;
};

#define zp3390_charger_platform_data_t \
	struct zp3390_charger_platform_data

struct zp3390_charger_data {
	struct i2c_client				*client;
	struct device					*dev;
	zp3390_charger_platform_data_t 	*pdata;
	struct mutex io_lock;
	struct mutex charger_lock;
	const struct firmware *firm_data_bin;

	int wc_w_state;

	struct power_supply *psy_chg;
	struct wake_lock wpc_wake_lock;
	struct wake_lock wpc_vbat_monitoring_enable_lock;
	struct wake_lock wpc_opfq_lock;
	struct wake_lock wpc_auth_check_lock;
	struct workqueue_struct *wqueue;
	struct delayed_work	wpc_det_work;
	struct delayed_work	wpc_opfq_work;
	struct delayed_work	wpc_isr_work;
	struct delayed_work wpc_init_work;
	struct delayed_work wpc_auth_check_work;
	struct delayed_work	curr_measure_work;
	struct delayed_work wpc_detach_chk_work;

	struct wake_lock wpc_id_request_lock;
	struct delayed_work	wpc_id_request_work;
	int wpc_id_request_step;

	struct wake_lock power_hold_chk_lock;
	struct delayed_work power_hold_chk_work;

	struct wake_lock wpc_wait_vout_lock;
	struct delayed_work	wpc_wait_vout_work;

	struct alarm polling_alarm;
	ktime_t last_poll_time;

	struct	notifier_block wpc_nb;

	struct wake_lock wpc_wakeup_wa_lock;
	struct delayed_work	wpc_wakeup_wa_work;
	int wpc_wakeup_wa;

	u16 addr;
	int size;

	int temperature;

	/* sec_battery's battery health */
	int battery_health;

	/* sec_battery's battery status */
	int battery_status;

	/* sec_battery's charge mode */
	int battery_chg_mode;

	/* sec_battery's swelling mode */
	int swelling_mode;

	/* sec_battery's slate(TCT) mode */
	int slate_mode;

	/* sec_battery's store(LDU) mode */
	int store_mode;

	/* check is recharge status */
	int is_recharge;

	/* check power hold status */
	int power_hold_mode;

	/* if set 1, need more sleep marging before ap_mode setting */
	int need_margin;

	/* check power hold status */
	int support_power_hold;

	/* check incompatible tx pad, incompatible tx pad is limited chg currnet */
	int incompatible_tx;

	/* if tx id is 0xF0, zp3390 set to current mesurement mode */
	int curr_measure_mode;

	/* Vout(ACOK) status */
	int vout_status;

	/* TX PAD id */
	int tx_id;

	/* TX id checked twice, if tx id checked normarlly, skip second tx id check */
	int tx_id_checked;

	/* charge mode cc/cv mode */
	enum zp3390_charge_mode_type charge_mode;

	enum zp3390_charger_pad_type charger_type;
	struct zp3390_chg_type_desc* charge_cb_func[ZP3390_CHARGER_TYPE_NUM_MAX];
};

struct zp3390_chg_type_desc {
	enum zp3390_charger_pad_type charger_type;
	char* charger_type_name;
	int (*cc_chg)(struct zp3390_charger_data *charger, bool prev_chk);
	int (*cv_chg)(struct zp3390_charger_data *charger, bool prev_chk);
	int (*full_chg)(struct zp3390_charger_data *charger);
	int (*re_chg)(struct zp3390_charger_data *charger);
};

ssize_t sec_wpc_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sec_wpc_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define SEC_WPC_ATTR(_name)						\
{									\
	.attr = {.name = #_name, .mode = 0664},	\
	.show = sec_wpc_show_attrs,					\
	.store = sec_wpc_store_attrs,					\
}

#endif /* __zp3390_CHARGER_H */
