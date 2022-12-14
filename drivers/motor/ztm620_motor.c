/*
** =============================================================================
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** File:
**     ztm620_motor.c
**
** Description:
**     ZTM620 vibration motor ic driver
**
** =============================================================================
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/ztm620_motor.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/suspend.h>
#ifdef CONFIG_SEC_SYSFS
#include <linux/sec_sysfs.h>
#if defined(CONFIG_SEC_DEBUG)
#include <linux/sec_debug.h>
#endif

static struct device *sec_motor;
#endif

static struct ztm620_motor_data *g_Ztm620MotorData = NULL;
static bool is_suspend = false;
static unsigned char pm_noti_test = 0;

static int ztm620_motor_reg_read(struct ztm620_motor_data *pMotorData, unsigned char reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pMotorData->mpRegmap, reg, &val);

	if (ret < 0){
		dev_err(pMotorData->dev, "[VIB] %s 0x%x error (%d)\n",
					__func__, reg, ret);
		return ret;
	}
	else
		return val;
}

static int ztm620_motor_reg_write(struct ztm620_motor_data *pMotorData,
	unsigned char reg, unsigned char val)
{
	int ret;

	ret = regmap_write(pMotorData->mpRegmap, reg, val);
	if (ret < 0){
		dev_err(pMotorData->dev, "[VIB] %s 0x%x=0x%x error (%d)\n",
					__func__, reg, val, ret);
	}

	return ret;
}

static int ztm620_motor_set_bits(struct ztm620_motor_data *pMotorData,
	unsigned char reg, unsigned char mask, unsigned char val)
{
	int ret;
	ret = regmap_update_bits(pMotorData->mpRegmap, reg, mask, val);
	if (ret < 0){
		dev_err(pMotorData->dev,
			"[VIB] %s reg=%x, mask=0x%x, value=0x%x error (%d)\n",
			__FUNCTION__, reg, mask, val, ret);
	}

	return ret;
}

static int ztm620_motor_run(void) {
	int ret = -EINVAL;
	unsigned char val, strength;
	int freq_reg, freq_hz;
	struct ztm620_motor_data *pMotorData = g_Ztm620MotorData;
	struct ztm620_motor_platform_data *pMotorPdata;

	if (!pMotorData) {
		pr_err("[VIB] %s ztm620_motor_data NULL error\n", __func__);
		goto out;
	}
	else
		pMotorPdata = &pMotorData->msPlatData;

	/* intensity level cannot be 0 and also cannot exceed
	   the number of strength values defined in dts */
	if (pMotorData->intensity_level == 0
			|| pMotorData->intensity_level
				> pMotorPdata->count_strength
			|| pMotorData->intensity_level
				> pMotorPdata->count_frequency) {
		pr_err("[VIB] %s level error: level %d, str_cnt %d, freq_cnt %d",
				__func__, pMotorData->intensity_level,
				pMotorPdata->count_strength,
				pMotorPdata->count_frequency);
		goto out;
	}

	/* strength is calculated with the level-mapped value in dt
	   multiplying intensity value out of 10000 */
	strength = pMotorPdata->strength[pMotorData->intensity_level - 1]
			* pMotorData->intensity_value / 10000;

	/* frequency data from platform is used unless it's 0
	   which means to use frequency defined in device tree for each model */
	if (pMotorData->frequency)
		freq_hz = pMotorData->frequency;
	else
		freq_hz = pMotorPdata->frequency[pMotorData->intensity_level - 1];

	ret = ztm620_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("[VIB] %s SOFT_EN write fail (%d)\n", __func__, ret);
		goto out;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	if (pMotorPdata->overdrive_num) {
		val = (pMotorData->overdrive & MOTOR_REG_OVER_DRV_EN_MASK)
			<< OVER_DRV_SHIFT_EN;
		val |= (pMotorPdata->overdrive_num & MOTOR_REG_OVER_DRV_CNT_MASK)
			<< OVER_DRV_SHIFT_CNT;
		ret = ztm620_motor_reg_write(pMotorData, MOTOR_REG_OVER_DRV, val);
		if (ret < 0) {
			pr_err("[VIB] %s OVER_DRV 0x%02x write fail (%d)\n",
				__func__, MOTOR_REG_OVER_DRV, ret);
			goto out;
		}
	}

	ret = ztm620_motor_reg_write(pMotorData, MOTOR_REG_STRENGTH, strength);
	if (ret < 0) {
		pr_err("[VIB] %s STRENGTH 0x%02x write %d fail (%d)\n",
			__func__, MOTOR_REG_STRENGTH, strength, ret);
		goto out;
	}

	ret = ztm620_motor_set_bits(pMotorData, MOTOR_REG_MODE_01,
		1 << MODE_01_SHIFT_DRV_MODE,
		pMotorPdata->meLoop << MODE_01_SHIFT_DRV_MODE);
	if (ret < 0) {
		pr_err("[VIB] %s MODE_01 0x%02x write fail (%d)\n",
			__func__, MOTOR_REG_MODE_01, ret);
		goto out;
	}

	/* Driving Period = DRV_FREQ * 16 Clocks
	   -> DRV_FREQ = Driving Period / (16 Clocks * Clock Period)
			 Clock Frequency / (16 Clocks * Driving Frequency)
	   DRV_FREQ_H[7:0] = DRV_FREQ[15:8]
	   DRV_FREQ_L[7:0] = DRV_FREQ[7:0]	*/
	freq_reg = (MOTOR_CLK * 10) / (freq_hz * 16);

	ret = ztm620_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_H, freq_reg / 256);
	if (ret < 0) {
		pr_err("[VIB] %s DRV_FREQ_H 0x%02x write fail (%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_H, ret);
		goto out;
	}

	ret = ztm620_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_L, freq_reg % 256);
	if (ret < 0) {
		pr_err("[VIB] %s DRV_FREQ_L 0x%02x write fail (%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_L, ret);
		goto out;
	}

	ret = ztm620_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_H, freq_reg / 256);
	if (ret < 0) {
		pr_err("[VIB] %s RESO_FREQ_H 0x%02x write fail (%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_H, ret);
		goto out;
	}

	ret = ztm620_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_L, freq_reg % 256);
	if (ret < 0) {
		pr_err("[VIB] %s RESO_FREQ_L 0x%02x write fail (%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_L, ret);
		goto out;
	}

	ret = ztm620_motor_reg_write(pMotorData,
		MOTOR_REG_ADC_SAMPLING_TIME, pMotorPdata->adc_sampling_time);
	if (ret < 0) {
		pr_err("[VIB] %s ADC_SAMPLING_TIME 0x%02x write fail (%d)\n",
			__func__, MOTOR_REG_ADC_SAMPLING_TIME, ret);
		goto out;
	}

	if (!pMotorData->running) {
		if (pMotorData->gpio_en > 0) {
			gpio_set_value(pMotorData->gpio_en, 1);
		}

		ret = ztm620_motor_reg_write(pMotorData,
			MOTOR_REG_MODE_00, pMotorPdata->motor_start_data);
		if (ret < 0) {
			pr_err("[VIB] %s MODE_00 write fail (%d)\n", __func__, ret);
			goto out;
		}
		pMotorData->running = true;
	}

	/* -1 for ovd means overdrive requested to be off by platform */
	pr_info("[VIB] Start: level %d, value 0x%x(%d),"
		" freq 0x%x(%d.%d/%d.%d), ovd %d\n",
		pMotorData->intensity_level,
		strength, pMotorData->intensity_value,
		freq_reg, pMotorData->frequency / 10, pMotorData->frequency % 10,
		pMotorPdata->frequency[pMotorData->intensity_level - 1] / 10,
		pMotorPdata->frequency[pMotorData->intensity_level - 1] % 10,
		pMotorData->overdrive?pMotorPdata->overdrive_num:-1);

out:
	return ret;
}

static void vibrator_work_routine(struct work_struct *work)
{
	int err;
	struct ztm620_motor_data *pMotorData = container_of(work,
						       struct ztm620_motor_data,
						       vibrator_work);
	struct ztm620_motor_platform_data *pMotorPdata = &pMotorData->msPlatData;

	if (pMotorData == NULL) {
		pr_err("[VIB] %s ztm620_motor_data NULL error\n", __func__);
		return;
	}

	mutex_lock(&pMotorData->lock);
	if (is_suspend)	goto out;

	if (pMotorData->intensity_value) {
		err = ztm620_motor_run();
		if (err < 0) {
			pr_err("[VIB] %s motor run fail %d\n", __func__, err);
			goto out;
		}
	}
	else if (pMotorData->running) {
		err = ztm620_motor_reg_write(pMotorData, MOTOR_REG_MODE_00, pMotorPdata->motor_stop_data);
		if (err < 0) {
			pr_err("[VIB] %s MODE_00 write fail %d\n", __func__, err);
			goto out;
		}

		if (pMotorPdata->break_mode && pMotorPdata->break_delay)
			queue_work(system_long_wq, &pMotorData->delay_en_off);
		else {
			if (pMotorData->gpio_en > 0) {
				gpio_set_value(pMotorData->gpio_en, 0);
			}

			err = ztm620_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
			if (err < 0) {
				pr_err("[VIB] %s SOFT_EN write fail %d\n",
							__func__, err);
				goto out;
			}
		}
		pMotorData->running = false;
		pMotorData->last_motor_off = CURRENT_TIME;
		pr_info("[VIB] Stop\n");
	}

out:
	mutex_unlock(&pMotorData->lock);
}

#define BRAKE_DELAY_RETRY 	5
static void ztm620_motor_delay_en_off(struct work_struct *work)
{
	struct ztm620_motor_data *pMotorData = container_of(work,
						       struct ztm620_motor_data,
						       delay_en_off);
	struct ztm620_motor_platform_data *pMotorPdata = &pMotorData->msPlatData;
	struct timespec tm, elapsed;
	int retry = 0;
	int err;

	do {
		if (retry == 0)
			msleep(pMotorPdata->break_delay);
		else
			msleep(pMotorPdata->break_delay / BRAKE_DELAY_RETRY);

		tm = CURRENT_TIME;
		elapsed = timespec_sub(tm, pMotorData->last_motor_off);
	} while (elapsed.tv_nsec < pMotorPdata->break_delay * 1000000 && retry++ < BRAKE_DELAY_RETRY);

	if (retry >= BRAKE_DELAY_RETRY)
		dev_err(pMotorData->dev, "[VIB] %s brake delay time err (%d < %d)\n",
					__func__,
					(int)(elapsed.tv_nsec / 1000000),
					pMotorPdata->break_delay);

	if (!pMotorData->running) {
		mutex_lock(&pMotorData->lock);
		if (pMotorData->gpio_en > 0) {
			gpio_set_value(pMotorData->gpio_en, 0);
		}

		err = ztm620_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
		if (err < 0) {
			dev_err(pMotorData->dev, "[VIB] %s SOFT_EN write fail %d\n",
						__func__, err);
		}
		mutex_unlock(&pMotorData->lock);
	}
}

static int ztm620_motor_haptic_play(struct input_dev *input, void *data,
				struct ff_effect *effect)
{
	struct ztm620_motor_data *pMotorData = input_get_drvdata(input);

	/* using ff_runble effect */
	/* struct ff_rumble_effect {
	 __u16 strong_magnitude; // strong_magnitude[15:15] = 0 (Reserved)
				 // strong_magnitude[14:14]
				 //   = Overdrive : 0 (Off) or 1 (On)
				 // strong_magnitude[13:0]
				 //   = Intensity Value : 0 (Stop)
				 //     or 1 ~ 10000 (Intensity)
	 __u16 weak_magnitude;	 // weak_magnitude[15:13]
				 //   = Intensity Level : 1 ~ 5
				 // weak_magnitude[12:0]
				 //   = Frequency in 0.1Hz : 0 ~ 8191 (0 ~ 819.1 Hz)
	}; */

	if (sec_debug_get_debug_level()) {
		pr_info("[VIB] %s param 0x%X 0x%X", __func__,
				effect->u.rumble.strong_magnitude,
				effect->u.rumble.weak_magnitude);
	}

	pMotorData->overdrive = (effect->u.rumble.strong_magnitude & 0x4000) >> 14;
	pMotorData->intensity_value = effect->u.rumble.strong_magnitude & 0x3fff;
	pMotorData->intensity_level = (effect->u.rumble.weak_magnitude & 0xe000) >> 13;
	pMotorData->frequency = effect->u.rumble.weak_magnitude & 0x1fff;

	queue_work(system_highpri_wq, &pMotorData->vibrator_work);

	return 0;
}

static int dev_init_platform_data(struct ztm620_motor_data *pMotorData)
{
	int ret = 0, i;
	struct ztm620_motor_platform_data *pMotorPdata = &pMotorData->msPlatData;
	unsigned int value_tmp = 0;

	dev_info(pMotorData->dev, "[VIB] %s: register init\n", __func__);

	ret = ztm620_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("[VIB] %s SOFT_EN 0x%02x write fail (%d)\n",
					__func__, MOTOR_REG_SOFT_EN, ret);
		goto out;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	value_tmp = 0;
	value_tmp |= (pMotorPdata->meLoop << MODE_01_SHIFT_DRV_MODE );
	value_tmp |= ((pMotorPdata->break_mode ? 0x00 : 0x01) << MODE_01_SHIFT_NBREAK_EN);
	value_tmp |= (MODE_01_DEFAULT_PGA_BEMP_GAIN << MODE_01_SHIFT_PGA_BEMP_GAIN );
	ret = ztm620_motor_set_bits(pMotorData,
		MOTOR_REG_MODE_01, MOTOR_REG_MODE_01_MASK, value_tmp);
	if (ret < 0) {
		pr_err("[VIB] %s MODE_01 0x%02x write fail (%d)\n",
					__func__, MOTOR_REG_MODE_01, ret);
		goto out;
	}

	value_tmp = 0;
	value_tmp |= (pMotorPdata->motor_type << MODE_13_SHIFT_ERM_NLRA );
	ret = ztm620_motor_set_bits(pMotorData,
		MOTOR_REG_MODE_13, MOTOR_REG_MODE_13_MASK, value_tmp);
	if (ret < 0) {
		pr_err("[VIB] %s MODE_13 0x%02x write fail (%d)\n",
					__func__, MOTOR_REG_MODE_13, ret);
		goto out;
	}

	if (pMotorPdata->count_init_regs > 0) {
		for (i = 0; i < pMotorPdata->count_init_regs; i++) {
			ret = ztm620_motor_reg_write(pMotorData,
						pMotorPdata->init_regs[i].addr,
						pMotorPdata->init_regs[i].data);
			if (ret < 0) {
				pr_err("[VIB] %s init_regs 0x%02x write fail (%d)\n",
					__func__,
					pMotorPdata->init_regs[i].addr, ret);
				goto out;
			}
		}
	}

	ret = ztm620_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
	if (ret < 0) {
		pr_err("[VIB] %s SOFT_EN 0x%02x write fail (%d)\n",
					__func__, MOTOR_REG_SOFT_EN, ret);
		goto out;
	}

out:
	return ret;
}

static void ztm620_motor_trigger_init(struct work_struct *work)
{
	struct ztm620_motor_data *pMotorData = container_of(work,
						       struct ztm620_motor_data,
						       trigger_init);
	mutex_lock(&pMotorData->lock);
	dev_init_platform_data(pMotorData);
	mutex_unlock(&pMotorData->lock);
}

static int ztm620_motor_pm_notifier(struct notifier_block *notifier,
                                       unsigned long pm_event, void *v)
{
	int err;
	struct ztm620_motor_data *pMotorData = container_of(notifier,
						       struct ztm620_motor_data,
						       ztm620_motor_pm_nb);

	if (pMotorData == NULL) {
		pr_err("[VIB] %s ztm620_motor_data NULL error\n", __func__);
		goto out;
	}

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&pMotorData->lock);
		if (pMotorData->running)
			pr_warn("[VIB] %s: motor is running", __func__);
		else if (pm_noti_test) {
			pMotorData->intensity_value = 0;
			pr_info("[VIB] %s: test suspend stop", __func__);
		}
		else {
			is_suspend = true;
			mutex_unlock(&pMotorData->lock);
			break;
		}

		err = ztm620_motor_reg_write(pMotorData,
				MOTOR_REG_MODE_00, pMotorData->msPlatData.motor_stop_data);
		if (err < 0) {
			pr_err("[VIB] %s MODE_00 write fail %d", __func__, err);
			goto out_err;
		}

		if (pMotorData->gpio_en > 0)
			gpio_set_value(pMotorData->gpio_en, 0);

		err = ztm620_motor_reg_write(pMotorData,
				MOTOR_REG_SOFT_EN, SOFT_DISABLE);
		if (err < 0) {
			pr_err("[VIB] %s SOFT_EN write fail %d", __func__, err);
			goto out_err;
		}
		pr_info("[VIB] %s Stop", __func__);

		pMotorData->running = false;
		pMotorData->last_motor_off = CURRENT_TIME;
		is_suspend = true;
		mutex_unlock(&pMotorData->lock);

		break;
	case PM_POST_SUSPEND:
		mutex_lock(&pMotorData->lock);
		is_suspend = false;

		if (pMotorData->intensity_value)
			pr_warn("[VIB] %s: motor is to run", __func__);
		else if (pm_noti_test) {
			pMotorData->intensity_value = MAX_INTENSITY_VALUE;
			pr_info("[VIB] %s test resume run", __func__);
		}
		else {
			mutex_unlock(&pMotorData->lock);
			break;
		}

		err = ztm620_motor_run();
		if (err < 0) {
			pr_err("[VIB] %s motor run fail %d",
					__func__, err);
			goto out_err;
		}
		mutex_unlock(&pMotorData->lock);

	        break;
	}

	return NOTIFY_OK;
out_err:
	mutex_unlock(&pMotorData->lock);
out:
	return NOTIFY_BAD;
}

static int Haptics_init(struct ztm620_motor_data *pMotorData)
{
	int ret = 0;

	struct input_dev *input_dev;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(pMotorData->dev, "[VIB] unable to allocate input device \n");
		return -ENODEV;
	}
	input_dev->name = "ztm620_motor_haptic";
	input_dev->dev.parent = pMotorData->dev;
	input_set_capability(input_dev, EV_FF, FF_RUMBLE);
	ret = input_ff_create_memless(input_dev, NULL,
		ztm620_motor_haptic_play);
	if (ret < 0) {
		dev_err(pMotorData->dev, "[VIB] input_ff_create_memless() failed: %d\n", ret);
		goto err_free_input;
	}
	ret = input_register_device(input_dev);
	if (ret < 0) {
		dev_err(pMotorData->dev,
			"[VIB] couldn't register input device: %d\n",
			ret);
		goto err_destroy_ff;
	}
	input_set_drvdata(input_dev, pMotorData);

	INIT_WORK(&pMotorData->vibrator_work, vibrator_work_routine);
	INIT_WORK(&pMotorData->delay_en_off, ztm620_motor_delay_en_off);
	INIT_WORK(&pMotorData->trigger_init, ztm620_motor_trigger_init);
	wake_lock_init(&pMotorData->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&pMotorData->lock);

	pMotorData->ztm620_motor_pm_nb.notifier_call = ztm620_motor_pm_notifier;
	register_pm_notifier(&pMotorData->ztm620_motor_pm_nb);

	return 0;
err_destroy_ff:
	input_ff_destroy(input_dev);
err_free_input:
	input_free_device(input_dev);

	return ret;
}

#if defined(CONFIG_OF)
static int of_ztm620_motor_dt(struct i2c_client* client, struct ztm620_motor_platform_data *pdata)
{
	int err;
	const char *motor_type;
	const char *loop_type;
	int i;

	dev_info(&client->dev, "[VIB] %s: dt parsing start\n", __func__);

	err = of_property_read_string(client->dev.of_node,"ztm620_motor,motor-type", &motor_type);
	if (err < 0) {
		dev_err(&client->dev, "[VIB] %s: motor-type read fail(%d)\n",
				__func__, err);
		return -ENODEV;
	}
	if (!strcmp(motor_type, "LRA")) {
		pdata->motor_type = ACTUATOR_LRA;
		pdata->motor_start_data = MODE_00_I2C_LRA;
		pdata->motor_stop_data = MODE_00_STOP_LRA;
	} else if (!strcmp(motor_type, "ERM")) {
		pdata->motor_type = ACTUATOR_ERM;
		pdata->motor_start_data = MODE_00_I2C_ERM;
		pdata->motor_stop_data = MODE_00_STOP_ERM;
	} else {
		dev_err(&client->dev, "[VIB] %s Wrong motor type: %s\n",
				__func__, motor_type);
		return -ENODEV;
	}
	pr_info("[VIB] motor-type = %s\n", motor_type);

	err = of_property_read_string(client->dev.of_node,
			"ztm620_motor,loop-type", &loop_type);
	if (err < 0) {
		dev_err(&client->dev, "[VIB] %s: loop-type read fail(%d)\n",
				__func__, err);
		return -ENODEV;
	}
	if (!strcmp(loop_type, "open"))
		pdata->meLoop = OPEN_LOOP;
	else if (!strcmp(loop_type, "closed"))
		pdata->meLoop = CLOSED_LOOP;
	else {
		dev_err(&client->dev, "[VIB] %s: Wrong loop type: %s\n",
				__func__, loop_type);
		return -ENODEV;
	}
	pr_info("[VIB] loop-type = %s\n", loop_type);

	pdata->break_mode = of_property_read_bool(client->dev.of_node,
			"ztm620_motor,break-on");
	pr_info("[VIB] break-on = %d\n", (int)pdata->break_mode);

	err = of_property_read_u32(client->dev.of_node,
			"ztm620_motor,brake-delay-ms", &pdata->break_delay);
	if (err < 0) {
		pdata->break_delay = DEFAULT_BRAKE_DELAY;
		dev_warn(&client->dev, "[VIB] %s: brake-delay-ms read fail(%d) :%d\n",
				__func__, err, pdata->break_delay);
	} else
		pr_info("[VIB] brake-delay-ms = %d\n", pdata->break_delay);

	pdata->count_init_regs = of_property_count_u32_elems(client->dev.of_node,
			"ztm620_motor,regs-init");
	if (pdata->count_init_regs > 0) {
		pdata->init_regs = devm_kzalloc(&client->dev,
				sizeof(u32) * pdata->count_init_regs, GFP_KERNEL);
		err = of_property_read_u32_array(client->dev.of_node,
				"ztm620_motor,regs-init",
				(u32 *)pdata->init_regs, pdata->count_init_regs);
		if (err < 0) {
			dev_err(&client->dev, "[VIB] %s: regs-init read fail(%d)\n",
					__func__, err);
			return -ENODEV;
		}
		pdata->count_init_regs /= 2;
	}
	pr_info("[VIB] regs-init count = %d\n", pdata->count_init_regs);

	pdata->gpio_en = of_get_named_gpio(client->dev.of_node,
			"ztm620_motor,motor_en", 0);
	if (pdata->gpio_en < 0)
		pr_warn("[VIB] %s: motor_en read fail(%d)\n",
				__func__, pdata->gpio_en);
	else
		pr_info("[VIB] motor_en = %d\n", pdata->gpio_en);

	err = of_property_read_string(client->dev.of_node,
			"ztm620_motor,regulator-name", &pdata->regulator_name);
	if (err < 0)
		pr_warn("[VIB] %s: regulator-name read fail(%d)\n",
				__func__, err);
	else
		pr_info("[VIB] regulator-name = %s\n", pdata->regulator_name);

	err = of_property_read_u32(client->dev.of_node,
			"ztm620_motor,adc-sampling-time", &pdata->adc_sampling_time);
	if (err < 0) {
		pdata->adc_sampling_time = DEFAULT_ADC_SAMPLING_TIME;
		pr_warn("[VIB] %s: adc-sampling-time read fail(%d) :%d\n",
				__func__, err, pdata->adc_sampling_time);
	} else
		pr_info("[VIB] adc-sampling-time = %d\n", pdata->adc_sampling_time);

	err = of_property_read_u32(client->dev.of_node,
			"ztm620_motor,soft-en-delay-ms", &pdata->soft_en_delay);
	if (err < 0) {
		pdata->soft_en_delay = DEFAULT_SOFT_EN_DELAY;
		pr_warn("[VIB] %s: soft-en-delay-ms read fail(%d) :%d\n",
				__func__, err, pdata->soft_en_delay);
	} else
		pr_info("[VIB] soft-en-delay-ms = %d\n", pdata->soft_en_delay);

	/* array elements of strength are assigned for each intensity level
	   starting from level 1 and increasing */
	pdata->count_strength = of_property_count_u32_elems(client->dev.of_node,
			"ztm620_motor,strength");
	pr_info("[VIB] strength count = %d", pdata->count_strength);
	if (pdata->count_strength > 0) {
		pdata->strength = devm_kzalloc(&client->dev,
				sizeof(u32) * pdata->count_strength, GFP_KERNEL);
		err = of_property_read_u32_array(client->dev.of_node,
				"ztm620_motor,strength",
				(u32 *)pdata->strength, pdata->count_strength);
		if (err < 0) {
			dev_err(&client->dev, "[VIB] %s: strength read fail(%d)\n",
					__func__, err);
		}

		for (i = 0; i < pdata->count_strength; i++) {
			pr_info("[VIB] strength[%d] = %d\n", i, pdata->strength[i]);
		}
	}

	/* array elements of frequency are assigned for each intensity level
	   starting from level 1 and increasing */
	pdata->count_frequency = of_property_count_u32_elems(client->dev.of_node,
			"ztm620_motor,frequency");
	pr_info("[VIB] frequency count = %d", pdata->count_frequency);
	if (pdata->count_frequency > 0) {
		pdata->frequency = devm_kzalloc(&client->dev,
				sizeof(u32) * pdata->count_frequency, GFP_KERNEL);
		err = of_property_read_u32_array(client->dev.of_node,
				"ztm620_motor,frequency",
				(u32 *)pdata->frequency, pdata->count_frequency);
		if (err < 0) {
			dev_err(&client->dev, "[VIB] %s: frequency read fail(%d)\n",
					__func__, err);
			return -ENODEV;
		}

		for (i = 0; i < pdata->count_frequency; i++) {
			pr_info("[VIB] frequency[%d] = %d\n", i, pdata->frequency[i]);
		}
	}

	err = of_property_read_u32(client->dev.of_node,
			"ztm620_motor,overdrive-num", &pdata->overdrive_num);
	if (err < 0) {
		pr_warn("[VIB] %s: overdrive-num read fail(%d) :%d\n",
				__func__, err, pdata->overdrive_num);
	} else
		pr_info("[VIB] overdrive-num = %d\n", pdata->overdrive_num);

	dev_info(&client->dev, "[VIB] %s: dt parsing done\n", __func__);

	return 0;
}
#endif /* CONFIG_OF */

int ztm620_motor_reset_handler(void)
{
	if (!g_Ztm620MotorData)
		return -ENODEV;

	queue_work(system_highpri_wq, &g_Ztm620MotorData->trigger_init);

	return 0;
}

bool ztm620_motor_is_running(void)
{
	if (g_Ztm620MotorData)
		return g_Ztm620MotorData->running;
	else
		return false;
}

static struct regmap_config ztm620_motor_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static unsigned char ztm620_motor_debugfs_addr;

static ssize_t read_ztm620_motor_dump_regs(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	static char buf[PAGE_SIZE];

	unsigned int val;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_MODE_00);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_MODE_00\t(0x00) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_MODE_01);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_MODE_01\t(0x01) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_SOFT_EN);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SOFT_EN\t(0x10) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_STRENGTH);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_STRENGTH\t(0x11) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_MODE_13);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_MODE_13\t(0x13) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_OVER_DRV);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_OVER_DRV\t(0x14) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_START_STRENGTH);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_START_STRENGTH(0x19) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_SEARCH_DRV_RATIO1);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SEARCH_DRV_RATIO1(0x1A) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_SEARCH_DRV_RATIO2);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SEARCH_DRV_RATIO2(0x1B) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_SEARCH_DRV_RATIO3);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SEARCH_DRV_RATIO3(0x1C) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_DRV_FREQ_H);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_DRV_FREQ_L\t(0x1F) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_DRV_FREQ_L);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_DRV_FREQ_H\t(0x20) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_RESO_FREQ_H);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_RESO_FREQ_L\t(0x21) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_RESO_FREQ_L);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_RESO_FREQ_H\t(0x22) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_FUNC_ENABLE);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_FUNC_ENABLE\t(0x23) = 0x%x\n", val);
		val = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_OUTPUT);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_OUTPUT\t(0x24) = 0x%x\n", val);
	}

	if (ret >= 0) {
		if (copy_to_user(buffer, buf, ret)) {
			return -EFAULT;
		}
		*ppos += ret;
	}
	return ret;
}

static const struct file_operations ztm620_motor_dump_regs_fops = {
	.read = read_ztm620_motor_dump_regs,
};

static int ztm620_motor_debugfs_data_fops_get(void *data, u64 * val)
{
	*val = ztm620_motor_reg_read(g_Ztm620MotorData, ztm620_motor_debugfs_addr);

	return 0;
}

static int ztm620_motor_debugfs_data_fops_set(void *data, u64 val)
{
	int ret;

	ret = ztm620_motor_reg_write(g_Ztm620MotorData, ztm620_motor_debugfs_addr,
							(unsigned char)val);
	if (ret < 0)
		pr_err("[VIB] %s 0x%02x=0x%02x write fail(%d)\n",
					__func__, ztm620_motor_debugfs_addr,
					(unsigned int)val, ret);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ztm620_motor_debugfs_data_fops,
			ztm620_motor_debugfs_data_fops_get, ztm620_motor_debugfs_data_fops_set, "%llx\n");

static int ztm620_motor_debugfs_enable_get(void *data, u64 * val)
{
	int ret = 0;

	if (g_Ztm620MotorData->gpio_en > 0)
		*val = gpio_get_value(g_Ztm620MotorData->gpio_en);
	else {
		ret = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_SOFT_EN);
		if (ret < 0)
			return ret;
		else
			*val = ret;
	}

	return 0;
}

static int ztm620_motor_debugfs_enable_set(void *data, u64 val)
{
	int ret = 0;

	if (g_Ztm620MotorData->gpio_en > 0)
		gpio_set_value(g_Ztm620MotorData->gpio_en, val?1:0);
	else {
		ret = ztm620_motor_reg_write(g_Ztm620MotorData,
					MOTOR_REG_SOFT_EN,
					val?(SOFT_ENABLE):(SOFT_DISABLE));
		if (ret < 0)
			pr_err("[VIB] %s SOFT_EN 0x%02x=%d write fail (%d)\n",
					__func__, MOTOR_REG_SOFT_EN,
					val?(SOFT_ENABLE):(SOFT_DISABLE), ret);
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ztm620_motor_debugfs_enable,
			ztm620_motor_debugfs_enable_get, ztm620_motor_debugfs_enable_set, "%llx\n");

int ztm620_motor_debug_init(void)
{
	struct dentry *d;

	d = debugfs_create_dir("ztm620_motor", NULL);

	debugfs_create_file("dump_regs", S_IRUSR | S_IWUSR, d, NULL, &ztm620_motor_dump_regs_fops);
	debugfs_create_u8("addr", S_IRUSR | S_IWUSR, d, &ztm620_motor_debugfs_addr);
	debugfs_create_file("data", S_IRUSR | S_IWUSR, d, NULL, &ztm620_motor_debugfs_data_fops);
	debugfs_create_file("enable", S_IRUSR | S_IWUSR, d, NULL, &ztm620_motor_debugfs_enable);
	debugfs_create_u8("pm_noti_test", S_IRUSR | S_IWUSR, d, &pm_noti_test);

	return 0;
}

#ifdef CONFIG_SEC_SYSFS
static ssize_t ztm620_motor_check_i2c(struct device *dev, struct device_attribute *attr,
		char *buf) {
	int err;

	err = ztm620_motor_reg_read(g_Ztm620MotorData, MOTOR_REG_STRENGTH);
	if(err < 0)
		return snprintf(buf, 20, "NG");

	return snprintf(buf, 20, "OK");
}

static DEVICE_ATTR(check_i2c, S_IRUGO, ztm620_motor_check_i2c, NULL);

static struct attribute *sec_motor_attrs[] = {
	&dev_attr_check_i2c.attr,
	NULL
};
static const struct attribute_group sec_motor_attr_group = {
	.attrs = sec_motor_attrs,
};

static int sec_motor_init(void) {
	int err;

	sec_motor = sec_device_create(NULL, "motor");
	if (IS_ERR(sec_motor)) {
		pr_err("[VIB] failed to create sec_motor\n");
		return -ENODEV;
	}

	err = sysfs_create_group(&sec_motor->kobj, &sec_motor_attr_group);
	if (err < 0) {
		pr_err("[VIB] failed to create sec_motor_attr\n");
		return err;
	}

	return 0;
}
#endif

static int ztm620_motor_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct ztm620_motor_data *pMotorData;
	struct ztm620_motor_platform_data *pMotorPdata;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, "[VIB] %s:I2C check failed\n", __FUNCTION__);
		return -ENODEV;
	}

	/* platform_data init */
	if(client->dev.of_node) {
		pMotorPdata = devm_kzalloc(&client->dev,
				sizeof(struct ztm620_motor_platform_data), GFP_KERNEL);
		if (!pMotorPdata) {
			dev_err(&client->dev, "[VIB] unable to allocate pdata memory\n");
			return -ENOMEM;
		}
		err = of_ztm620_motor_dt(client, pMotorPdata);
		if (err < 0) {
			dev_err(&client->dev, "[VIB] fail to read DT %d\n", err);
			return -ENODEV;
		}
	} else
		pMotorPdata = dev_get_platdata(&client->dev);

	pMotorData = devm_kzalloc(&client->dev, sizeof(struct ztm620_motor_data), GFP_KERNEL);
	if (pMotorData == NULL){
		dev_err(&client->dev, "[VIB] %s:no memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	pMotorData->dev = &client->dev;
	pMotorData->mpRegmap = devm_regmap_init_i2c(client, &ztm620_motor_i2c_regmap);
	if (IS_ERR(pMotorData->mpRegmap)) {
		err = PTR_ERR(pMotorData->mpRegmap);
		dev_err(pMotorData->dev,
			"[VIB] %s:Failed to allocate register map: %d\n",__FUNCTION__,err);
		return err;
	}

	if (pMotorPdata->regulator_name) {
		pMotorData->regulator
			= devm_regulator_get(pMotorData->dev, pMotorPdata->regulator_name);
		if (IS_ERR(pMotorData->regulator)) {
			dev_err(pMotorData->dev, "[VIB] Failed to get moter power supply.\n");
			return -EFAULT;
		}
		err = regulator_set_voltage(pMotorData->regulator, MOTOR_VCC, MOTOR_VCC);
		if (err < 0)
			dev_err(pMotorData->dev, "[VIB] Failed to set moter power %duV. %d\n",
			MOTOR_VCC, err);
		err = regulator_enable(pMotorData->regulator);
		if (err < 0) {
			dev_err(pMotorData->dev, "[VIB] %s enable fail(%d)\n",
					pMotorPdata->regulator_name, err);
			return -EFAULT;
		} else
			pr_info("[VIB] %s: %s enable\n",
					__func__, pMotorPdata->regulator_name);
	}

	if (pMotorPdata->gpio_en > 0) {
		err = devm_gpio_request_one(pMotorData->dev, pMotorPdata->gpio_en,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motor");
		if (err < 0) {
			dev_err(pMotorData->dev ,"[VIB] gpio_en request fail %d\n", err);
			return -EFAULT;
		}
	}
	pMotorData->gpio_en = pMotorPdata->gpio_en;

	memcpy(&pMotorData->msPlatData, pMotorPdata, sizeof(struct ztm620_motor_platform_data));

	err = ztm620_motor_reg_read(pMotorData, MOTOR_REG_STRENGTH);
	if(err < 0){
		dev_err(pMotorData->dev,
			"[VIB] %s, i2c bus fail (%d)\n", __FUNCTION__, err);
		return -EFAULT;
	}else{
		dev_info(pMotorData->dev,
			"[VIB] %s, i2c check - Strength (0x%x)\n", __FUNCTION__, err);
		pMotorData->mnDeviceID = err;
	}

	err = dev_init_platform_data(pMotorData);
	if (err < 0){
		dev_err(pMotorData->dev, "[VIB] dev_init_platform failed. %d\n", err);
		return -EFAULT;
	}

	err = Haptics_init(pMotorData);
	if (err < 0){
		dev_err(pMotorData->dev, "[VIB] Haptics_init failed. %d\n", err);
		return -EFAULT;
	}

	g_Ztm620MotorData = pMotorData;
	i2c_set_clientdata(client, pMotorData);

	ztm620_motor_debug_init();
#ifdef CONFIG_SEC_SYSFS
	sec_motor_init();
#endif

	dev_info(pMotorData->dev, "[VIB] ztm620_motor probe succeeded\n");

	return 0;
}

void ztm620_motor_shutdown(struct i2c_client *client)
{
	struct ztm620_motor_data *pMotorData = i2c_get_clientdata(client);
	struct ztm620_motor_platform_data *pMotorPdata = &(pMotorData->msPlatData);
	int err;

	dev_info(pMotorData->dev, "[VIB] %s", __func__);

	if (pMotorData->regulator) {
		err = regulator_disable(pMotorData->regulator);
		if (err < 0)
			dev_err(pMotorData->dev, "[VIB] %s disable fail(%d)",
					pMotorPdata->regulator_name, err);
		else
			pr_info("[VIB] %s: %s disable\n",
					__func__, pMotorPdata->regulator_name);
	}
}

#if defined(CONFIG_OF)
static struct of_device_id haptic_dt_ids[] = {
	{ .compatible = "ztm620_motor" },
	{ },
};
MODULE_DEVICE_TABLE(of, haptic_dt_ids);
#endif /* CONFIG_OF */

static struct i2c_device_id ztm620_motor_id_table[] =
{
	{ HAPTICS_DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ztm620_motor_id_table);

static struct i2c_driver ztm620_motor_driver =
{
	.driver = {
		.name = HAPTICS_DEVICE_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = haptic_dt_ids,
#endif /* CONFIG_OF */
	},
	.id_table = ztm620_motor_id_table,
	.probe = ztm620_motor_probe,
	.shutdown = ztm620_motor_shutdown,
};

static int __init ztm620_motor_init(void)
{
	pr_info("[VIB] %s\n", __func__);
	return i2c_add_driver(&ztm620_motor_driver);
}

static void __exit ztm620_motor_exit(void)
{
	i2c_del_driver(&ztm620_motor_driver);
}

module_init(ztm620_motor_init);
module_exit(ztm620_motor_exit);

MODULE_AUTHOR("samsung");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
