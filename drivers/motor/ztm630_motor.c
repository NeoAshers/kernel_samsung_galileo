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
**     ztm630_motor.c
**
** Description:
**     ZTM630 vibration motor ic driver
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
#include <linux/ztm630_motor.h>
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

#define BRAKE_DELAY_RETRY 	5
#define MAX_FIFO_SIZE	0xff

static struct ztm630_motor_data *g_Ztm630MotorData = NULL;
static unsigned char pm_noti_test = 0;
static const struct regmap_config ztm630_motor_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static unsigned char ztm630_motor_debugfs_addr;

static int ztm630_motor_reg_read(struct ztm630_motor_data *pMotorData, unsigned char reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pMotorData->mpRegmap, reg, &val);

	if (ret < 0) {
		pr_err("%s:0x%x error(%d)\n", __func__, reg, ret);
		return ret;
	}
	else
		return val;
}

static int ztm630_motor_reg_write(struct ztm630_motor_data *pMotorData,
	unsigned char reg, unsigned char val)
{
	int ret;

	ret = regmap_write(pMotorData->mpRegmap, reg, val);
	if (ret < 0) {
		pr_err("%s:0x%x=0x%x error(%d)\n", __func__, reg, val, ret);
	}

	return ret;
}

static int ztm630_motor_raw_write(struct ztm630_motor_data *pMotorData,
	unsigned char reg, const unsigned char *val, int val_len)
{
	int ret;

	ret = regmap_raw_write(pMotorData->mpRegmap, reg, val, val_len);
	if (ret < 0) {
		pr_err("%s:0x%x error(%d)\n", __func__, reg, ret);
	}

	return ret;
}

static int ztm630_motor_set_bits(struct ztm630_motor_data *pMotorData,
	unsigned char reg, unsigned char mask, unsigned char val)
{
	int ret;
	ret = regmap_update_bits(pMotorData->mpRegmap, reg, mask, val);
	if (ret < 0) {
		pr_err("%s:reg=%x, mask=0x%x, value=0x%x error (%d)\n",
			__func__, reg, mask, val, ret);
	}

	return ret;
}

static int ztm630_motor_run(struct ztm630_motor_data *pMotorData)
{
	struct ztm630_motor_pdata *pMotorPdata = pMotorData->msPlatData;
	int freq_reg, freq_hz, ret = -EINVAL;
	unsigned char val, strength;

	if (!pMotorData) {
		pr_err("%s:ztm630_motor_data NULL error\n", __func__);
		return -EINVAL;
	}

	/* intensity level cannot be 0 and also cannot exceed
	   the number of strength values defined in dts */
	if (pMotorData->intensity_level == 0
			|| pMotorData->intensity_level
				> pMotorPdata->count_strength
			|| pMotorData->intensity_level
				> pMotorPdata->count_frequency) {
		pr_err("%s:level error:level %d str_cnt %d freq_cnt %d\n",
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

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN write fail (%d)\n", __func__, ret);
		goto out;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	if (pMotorPdata->overdrive_num) {
		val = (pMotorData->overdrive & MOTOR_REG_OVER_DRV_EN_MASK)
			<< OVER_DRV_SHIFT_EN;
		val |= (pMotorPdata->overdrive_num & MOTOR_REG_OVER_DRV_CNT_MASK)
			<< OVER_DRV_SHIFT_CNT;
		ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_OVER_DRV, val);
		if (ret < 0) {
			pr_err("%s:OVER_DRV 0x%02x write fail(%d)\n",
				__func__, MOTOR_REG_OVER_DRV, ret);
			goto out;
		}
	}

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_STRENGTH, strength);
	if (ret < 0) {
		pr_err("%s:STRENGTH 0x%02x write %d fail (%d)\n",
			__func__, MOTOR_REG_STRENGTH, strength, ret);
		goto out;
	}

	ret = ztm630_motor_set_bits(pMotorData, MOTOR_REG_MODE_01,
		1 << MODE_01_SHIFT_DRV_MODE,
		pMotorPdata->meLoop << MODE_01_SHIFT_DRV_MODE);
	if (ret < 0) {
		pr_err("%s:MODE_01 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_MODE_01, ret);
		goto out;
	}

	/* Driving Period = DRV_FREQ * 16 Clocks
	   -> DRV_FREQ = Driving Period / (16 Clocks * Clock Period)
			 Clock Frequency / (16 Clocks * Driving Frequency)
	   DRV_FREQ_H[7:0] = DRV_FREQ[15:8]
	   DRV_FREQ_L[7:0] = DRV_FREQ[7:0]	*/
	freq_reg = (MOTOR_CLK * 10) / (freq_hz << 4);

	ret = ztm630_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_H, (freq_reg >> 8));
	if (ret < 0) {
		pr_err("%s:DRV_FREQ_H 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_H, ret);
		goto out;
	}

	ret = ztm630_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_L, (freq_reg & 0xff));
	if (ret < 0) {
		pr_err("%sDRV_FREQ_L 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_L, ret);
		goto out;
	}

	ret = ztm630_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_H, (freq_reg >> 8));
	if (ret < 0) {
		pr_err("%s:RESO_FREQ_H 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_H, ret);
		goto out;
	}

	ret = ztm630_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_L, (freq_reg & 0xff));
	if (ret < 0) {
		pr_err("%s:RESO_FREQ_L 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_L, ret);
		goto out;
	}

	ret = ztm630_motor_reg_write(pMotorData,
		MOTOR_REG_ADC_SAMPLING_TIME, pMotorPdata->adc_sampling_time);
	if (ret < 0) {
		pr_err("%s:ADC_SAMPLING_TIME 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_ADC_SAMPLING_TIME, ret);
		goto out;
	}

	if (!pMotorData->running) {
		if (pMotorData->en_gpio > 0) {
			gpio_set_value(pMotorData->en_gpio, 1);
		}

		ret = ztm630_motor_reg_write(pMotorData,
			MOTOR_REG_MODE_00, pMotorPdata->motor_start_data);
		if (ret < 0) {
			pr_err("%s:MODE_00 write fail(%d)\n", __func__, ret);
			goto out;
		}
		pMotorData->running = true;
	}

	/* -1 for ovd means overdrive requested to be off by platform */
	pr_info("%s:level:%d value:0x%x(%d)"
		"freq:0x%x(%d.%d/%d.%d) ovd:%d\n",
		__func__, pMotorData->intensity_level,
		strength, pMotorData->intensity_value,
		freq_reg, pMotorData->frequency / 10, pMotorData->frequency % 10,
		pMotorPdata->frequency[pMotorData->intensity_level - 1] / 10,
		pMotorPdata->frequency[pMotorData->intensity_level - 1] % 10,
		pMotorData->overdrive?pMotorPdata->overdrive_num:-1);

out:
	return ret;
}

static int ztm630_motor_fifo_play(struct ztm630_motor_data *pMotorData, const char *data, const int data_size)
{
	struct ztm630_motor_pdata *pMotorPdata = pMotorData->msPlatData;
	int index, write_size, ret = -EINVAL;

	if (data_size == 0) {
		pr_err("%s:data_size is ZERO\n", __func__);
		return -EINVAL;
	}

	pr_info("%s\n", __func__);

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN write fail(%d)\n", __func__, ret);
		return ret;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_MODE_01, FIFO_ENABLE);
	if (ret < 0) {
		pr_err("%s:FIFO_ENABLE write fail(%d)\n", __func__, ret);
		return ret;
	}

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_FIFO_INTERVAL, data[0]);
	if (ret < 0) {
		pr_err("%s:FIFO_INTERVAL write fail(%d)\n", __func__, ret);
		return ret;
	}

	for (index = 1; index < data_size; index += write_size) {
		if ((data_size - index) > MAX_FIFO_SIZE)
			write_size = MAX_FIFO_SIZE;
		else
			write_size = data_size - index;

		ret = ztm630_motor_raw_write(pMotorData, MOTOR_REG_FIFO_DATA,
					&data[index], write_size);
		if (ret < 0) {
			pr_err("%s:FIFO_DATA write fail(%d)\n", __func__, ret);
			return ret;
		}
	}

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN write fail(%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

static void ztm630_motor_fifo_vib_work(struct work_struct *work)
{
	struct ztm630_motor_data *pMotorData = container_of(work,
						       struct ztm630_motor_data,
						       fifo_vib_work);
	int ret;


	if (pMotorData == NULL) {
		pr_err("%s:ztm630_motor_data NULL error\n", __func__);
		return;
	}

	mutex_lock(&pMotorData->lock);

	if (pMotorData->fifo_data_size == 0) {
		pr_err("%s:fifo_data_size is ZERO\n", __func__);
		goto out;
	}

	if (pMotorData->is_suspend) {
		pr_err("%s:is_suspend\n", __func__);
		goto out;
	}

	ret = ztm630_motor_fifo_play(pMotorData, pMotorData->fifo_data_addr,
				pMotorData->fifo_data_size);
	if (ret < 0)
		pr_err("%s:failed fifo_play(%d)\n", __func__, ret);

out:
	pMotorData->fifo_data_size = 0;
	mutex_unlock(&pMotorData->lock);

	return;
}

static void ztm630_motor_vibrator_work(struct work_struct *work)
{
	int err;
	struct ztm630_motor_data *pMotorData = container_of(work,
						       struct ztm630_motor_data,
						       vibrator_work);
	struct ztm630_motor_pdata *pMotorPdata = pMotorData->msPlatData;

	if (pMotorData == NULL) {
		pr_err("%s:ztm630_motor_data NULL error\n", __func__);
		return;
	}

	mutex_lock(&pMotorData->lock);
	if (pMotorData->is_suspend)
		goto out;

	if (pMotorData->intensity_value) {
		err = ztm630_motor_run(pMotorData);
		if (err < 0) {
			pr_err("%s:motor run fail(%d)\n", __func__, err);
			goto out;
		}
	} else if (pMotorData->running) {
		err = ztm630_motor_reg_write(pMotorData, MOTOR_REG_MODE_00, pMotorPdata->motor_stop_data);
		if (err < 0) {
			pr_err("%s:MODE_00 write fail(%d)\n", __func__, err);
			goto out;
		}

		if (pMotorPdata->break_mode && pMotorPdata->break_delay)
			queue_work(system_long_wq, &pMotorData->delay_en_off);
		else {
			if (pMotorData->en_gpio > 0) {
				gpio_set_value(pMotorData->en_gpio, 0);
			}

			err = ztm630_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
			if (err < 0) {
				pr_err("%s:SOFT_EN write fail(%d)\n",
							__func__, err);
				goto out;
			}
		}
		pMotorData->running = false;
		pMotorData->last_motor_off = CURRENT_TIME;
		pr_info("%s:Stop\n", __func__);
	}

out:
	mutex_unlock(&pMotorData->lock);
}

static void ztm630_motor_delay_en_off(struct work_struct *work)
{
	struct ztm630_motor_data *pMotorData = container_of(work,
						       struct ztm630_motor_data,
						       delay_en_off);
	struct ztm630_motor_pdata *pMotorPdata = pMotorData->msPlatData;
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
		pr_err("%s:brake delay time err(%d<%d)\n",
					__func__,
					(int)(elapsed.tv_nsec / 1000000),
					pMotorPdata->break_delay);

	if (!pMotorData->running) {
		mutex_lock(&pMotorData->lock);
		if (pMotorData->en_gpio > 0) {
			gpio_set_value(pMotorData->en_gpio, 0);
		}

		err = ztm630_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
		if (err < 0) {
			pr_err("%s:SOFT_EN write fail %d\n",
						__func__, err);
		}
		mutex_unlock(&pMotorData->lock);
	}
}

static int ztm630_motor_haptic_play(struct input_dev *input, void *data,
				struct ff_effect *effect)
{
	struct ztm630_motor_data *pMotorData = input_get_drvdata(input);

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
		pr_info("%s:[0x%04x][0x%04x]\n", __func__,
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

static int ztm630_motor_platform_init(struct ztm630_motor_data *pMotorData)
{
	struct ztm630_motor_pdata *pMotorPdata = pMotorData->msPlatData;
	int ret = 0, i;
	unsigned int value_tmp = 0;

	pr_info("%s\n", __func__);

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_SOFT_EN, ret);
		goto out;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	value_tmp = 0;
	value_tmp |= (pMotorPdata->meLoop << MODE_01_SHIFT_DRV_MODE );
	value_tmp |= ((pMotorPdata->break_mode ? 0x00 : 0x01) << MODE_01_SHIFT_NBREAK_EN);
	value_tmp |= (MODE_01_DEFAULT_PGA_BEMP_GAIN << MODE_01_SHIFT_PGA_BEMP_GAIN );
	ret = ztm630_motor_set_bits(pMotorData,
		MOTOR_REG_MODE_01, MOTOR_REG_MODE_01_MASK, value_tmp);
	if (ret < 0) {
		pr_err("%s:MODE_01 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_MODE_01, ret);
		goto out;
	}

	value_tmp = 0;
	value_tmp |= (pMotorPdata->motor_type << MODE_13_SHIFT_ERM_NLRA );
	ret = ztm630_motor_set_bits(pMotorData,
		MOTOR_REG_MODE_13, MOTOR_REG_MODE_13_MASK, value_tmp);
	if (ret < 0) {
		pr_err("%s:MODE_13 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_MODE_13, ret);
		goto out;
	}

	if (pMotorPdata->count_init_regs > 0) {
		for (i = 0; i < pMotorPdata->count_init_regs; i++) {
			ret = ztm630_motor_reg_write(pMotorData,
						pMotorPdata->init_regs[i].addr,
						pMotorPdata->init_regs[i].data);
			if (ret < 0) {
				pr_err("%s:init_regs 0x%02x write fail(%d)\n",
					__func__,
					pMotorPdata->init_regs[i].addr, ret);
				goto out;
			}
		}
	}

	ret = ztm630_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_SOFT_EN, ret);
		goto out;
	}

out:
	return ret;
}

static void ztm630_motor_trigger_init(struct work_struct *work)
{
	struct ztm630_motor_data *pMotorData = container_of(work,
						       struct ztm630_motor_data,
						       trigger_init);
	mutex_lock(&pMotorData->lock);
	ztm630_motor_platform_init(pMotorData);
	mutex_unlock(&pMotorData->lock);
}

static int ztm630_motor_pm_notifier(struct notifier_block *notifier,
                                       unsigned long pm_event, void *v)
{
	int err;
	struct ztm630_motor_data *pMotorData = container_of(notifier,
						       struct ztm630_motor_data,
						       ztm630_motor_pm_nb);
	struct ztm630_motor_pdata *pMotorPdata = pMotorData->msPlatData;

	if (pMotorData == NULL) {
		pr_err("%s:ztm630_motor_data NULL error\n", __func__);
		goto out;
	}

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&pMotorData->lock);
		if (pMotorData->running)
			pr_warn("%s:motor is running\n", __func__);
		else if (pm_noti_test) {
			pMotorData->intensity_value = 0;
			pr_info("%s:test suspend stop\n", __func__);
		} else {
			pMotorData->is_suspend = true;
			mutex_unlock(&pMotorData->lock);
			break;
		}

		err = ztm630_motor_reg_write(pMotorData,
				MOTOR_REG_MODE_00, pMotorPdata->motor_stop_data);
		if (err < 0) {
			pr_err("%s:MODE_00 write fail %d\n", __func__, err);
			goto out_err;
		}

		if (pMotorData->en_gpio > 0)
			gpio_set_value(pMotorData->en_gpio, 0);

		err = ztm630_motor_reg_write(pMotorData,
				MOTOR_REG_SOFT_EN, SOFT_DISABLE);
		if (err < 0) {
			pr_err("%s:SOFT_EN write fail %d", __func__, err);
			goto out_err;
		}
		pr_info("%s:Stop", __func__);

		pMotorData->running = false;
		pMotorData->last_motor_off = CURRENT_TIME;
		pMotorData->is_suspend = true;
		mutex_unlock(&pMotorData->lock);

		break;
	case PM_POST_SUSPEND:
		mutex_lock(&pMotorData->lock);
		pMotorData->is_suspend = false;

		if (pMotorData->intensity_value)
			pr_warn("%s:motor is to run\n", __func__);
		else if (pm_noti_test) {
			pMotorData->intensity_value = MAX_INTENSITY_VALUE;
			pr_info("%s:test resume run\n", __func__);
		} else {
			mutex_unlock(&pMotorData->lock);
			break;
		}

		err = ztm630_motor_run(pMotorData);
		if (err < 0) {
			pr_err("%s:motor run fail(%d)\n", __func__, err);
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

static int ztm630_motor_haptics_init(struct ztm630_motor_data *pMotorData)
{
	struct input_dev *input_dev;
	int ret = 0;

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s:unable to allocate input device\n", __func__);
		return -ENODEV;
	}
	input_dev->name = "ztm630_motor_haptic";
	input_dev->dev.parent = pMotorData->dev;
	input_set_capability(input_dev, EV_FF, FF_RUMBLE);
	ret = input_ff_create_memless(input_dev, NULL,
		ztm630_motor_haptic_play);
	if (ret < 0) {
		pr_err("%s:input_ff_create_memless() failed: %d\n", __func__, ret);
		goto err_free_input;
	}
	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%scouldn't register input device: %d\n", __func__, ret);
		goto err_destroy_ff;
	}
	input_set_drvdata(input_dev, pMotorData);

	INIT_WORK(&pMotorData->vibrator_work, ztm630_motor_vibrator_work);
	INIT_WORK(&pMotorData->delay_en_off, ztm630_motor_delay_en_off);
	INIT_WORK(&pMotorData->trigger_init, ztm630_motor_trigger_init);
	INIT_WORK(&pMotorData->fifo_vib_work, ztm630_motor_fifo_vib_work);
	wake_lock_init(&pMotorData->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&pMotorData->lock);

	pMotorData->ztm630_motor_pm_nb.notifier_call = ztm630_motor_pm_notifier;
	register_pm_notifier(&pMotorData->ztm630_motor_pm_nb);

	return 0;
err_destroy_ff:
	input_ff_destroy(input_dev);
err_free_input:
	input_free_device(input_dev);

	return ret;
}

int ztm630_motor_reset_handler(void)
{
	if (g_Ztm630MotorData == NULL) {
		pr_err("%s:g_Ztm630MotorData is NULL\n", __func__);
		return -ENODEV;
	}

	queue_work(system_highpri_wq, &g_Ztm630MotorData->trigger_init);

	return 0;
}

bool ztm630_motor_is_running(void)
{
	if (g_Ztm630MotorData == NULL)
		return false;

	return g_Ztm630MotorData->running;
}

static ssize_t read_ztm630_motor_dump_regs(struct file *file,
	char __user *buffer, size_t count, loff_t *ppos)
{
	static char buf[PAGE_SIZE];
	ssize_t ret = 0;
	unsigned int val;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (*ppos == 0) {
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_MODE_00);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_MODE_00\t(0x00) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_MODE_01);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_MODE_01\t(0x01) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_SOFT_EN);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SOFT_EN\t(0x10) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_STRENGTH);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_STRENGTH\t(0x11) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_MODE_13);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_MODE_13\t(0x13) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_OVER_DRV);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_OVER_DRV\t(0x14) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_START_STRENGTH);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_START_STRENGTH(0x19) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_SEARCH_DRV_RATIO1);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SEARCH_DRV_RATIO1(0x1A) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_SEARCH_DRV_RATIO2);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SEARCH_DRV_RATIO2(0x1B) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_SEARCH_DRV_RATIO3);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_SEARCH_DRV_RATIO3(0x1C) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_DRV_FREQ_H);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_DRV_FREQ_L\t(0x1F) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_DRV_FREQ_L);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_DRV_FREQ_H\t(0x20) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_RESO_FREQ_H);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_RESO_FREQ_L\t(0x21) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_RESO_FREQ_L);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_RESO_FREQ_H\t(0x22) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_FUNC_ENABLE);
		ret += snprintf(buf + ret, sizeof(buf) - ret,
			"MOTOR_REG_FUNC_ENABLE\t(0x23) = 0x%x\n", val);
		val = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_OUTPUT);
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

static const struct file_operations ztm630_motor_dump_regs_fops = {
	.read = read_ztm630_motor_dump_regs,
};

static int ztm630_motor_debugfs_data_fops_get(void *data, u64 * val)
{
	*val = ztm630_motor_reg_read(g_Ztm630MotorData, ztm630_motor_debugfs_addr);

	return 0;
}

static int ztm630_motor_debugfs_data_fops_set(void *data, u64 val)
{
	int ret;

	ret = ztm630_motor_reg_write(g_Ztm630MotorData, ztm630_motor_debugfs_addr,
							(unsigned char)val);
	if (ret < 0)
		pr_err("%s:0x%02x=0x%02x write fail(%d)\n",
					__func__, ztm630_motor_debugfs_addr,
					(unsigned int)val, ret);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ztm630_motor_debugfs_data_fops,
			ztm630_motor_debugfs_data_fops_get, ztm630_motor_debugfs_data_fops_set, "%llx\n");

static int ztm630_motor_debugfs_enable_get(void *data, u64 * val)
{
	int ret = 0;

	if (g_Ztm630MotorData->en_gpio > 0)
		*val = gpio_get_value(g_Ztm630MotorData->en_gpio);
	else {
		ret = ztm630_motor_reg_read(g_Ztm630MotorData, MOTOR_REG_SOFT_EN);
		if (ret < 0)
			return ret;
		else
			*val = ret;
	}

	return 0;
}

static int ztm630_motor_debugfs_enable_set(void *data, u64 val)
{
	int ret = 0;

	if (g_Ztm630MotorData->en_gpio > 0)
		gpio_set_value(g_Ztm630MotorData->en_gpio, val?1:0);
	else {
		ret = ztm630_motor_reg_write(g_Ztm630MotorData,
					MOTOR_REG_SOFT_EN,
					val?(SOFT_ENABLE):(SOFT_DISABLE));
		if (ret < 0)
			pr_err("%s:SOFT_EN 0x%02x=%d write fail (%d)\n",
					__func__, MOTOR_REG_SOFT_EN,
					val?(SOFT_ENABLE):(SOFT_DISABLE), ret);
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ztm630_motor_debugfs_enable,
			ztm630_motor_debugfs_enable_get, ztm630_motor_debugfs_enable_set, "%llx\n");

int ztm630_motor_debug_init(struct ztm630_motor_data *pMotorData)
{
	pMotorData->debug_d = debugfs_create_dir(HAPTICS_DEVICE_NAME, NULL);

	debugfs_create_file("dump_regs", S_IRUSR | S_IWUSR, pMotorData->debug_d, pMotorData, &ztm630_motor_dump_regs_fops);
	debugfs_create_file("data", S_IRUSR | S_IWUSR, pMotorData->debug_d, pMotorData, &ztm630_motor_debugfs_data_fops);
	debugfs_create_file("enable", S_IRUSR | S_IWUSR, pMotorData->debug_d, pMotorData, &ztm630_motor_debugfs_enable);
	debugfs_create_u8("addr", S_IRUSR | S_IWUSR, pMotorData->debug_d, &ztm630_motor_debugfs_addr);
	debugfs_create_u8("pm_noti_test", S_IRUSR | S_IWUSR, pMotorData->debug_d, &pm_noti_test);

	return 0;
}

#ifdef CONFIG_SEC_SYSFS
static ssize_t ztm630_motor_check_i2c(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct ztm630_motor_data *pMotorData = dev_get_drvdata(dev);
	int err;

	if (!pMotorData) {
		pr_err("%s:ztm630_motor_data NULL error\n", __func__);
		return -EINVAL;
	}

	err = ztm630_motor_reg_read(pMotorData, MOTOR_REG_STRENGTH);
	if (err < 0)
		return snprintf(buf, 20, "NG");

	return snprintf(buf, 20, "OK");
}
static DEVICE_ATTR(check_i2c, S_IRUGO, ztm630_motor_check_i2c, NULL);

#ifdef CONFIG_TIZEN_SEC_KERNEL_ENG
static ssize_t ztm630_motor_store_fifo_vib(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct ztm630_motor_data *pMotorData = dev_get_drvdata(dev);
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;
	char file_path[0xff];
	int ret, path_length;
	u8 *buff;

	ret = sscanf(buf, "%s%n", &file_path[0], &path_length);
	if ((ret != 1) || (path_length > 0xff)) {
		pr_err("%s:Invalid argument\n", __func__);
		return -EINVAL;
	}

	pr_info("%s:file_name[%s]\n", __func__, &file_path[0]);

	mutex_lock(&pMotorData->lock);
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(&file_path[0], O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		pr_err("%s:open error\n", __func__);
		goto open_fail;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;
	if (fsize < 1) {
		pr_err("%s:Invalid faile size(%ld)\n", __func__, fsize);
		goto alloc_fail;
	}

	buff = devm_kzalloc(pMotorData->dev, (size_t)fsize, GFP_KERNEL);
	if (!buff) {
		pr_err("%s:failed to alloc buffer for fw\n", __func__);
		goto alloc_fail;
	}

	nread = vfs_read(fp, (char __user *)buff, fsize, &fp->f_pos);
	if (nread != fsize) {
		pr_err("%s:Invalid read size(%ld/%ld)\n", __func__, nread, fsize);
		goto err;
	}

	pMotorData->fifo_data_addr = &buff[0];
	pMotorData->fifo_data_size = fsize;

	ret = ztm630_motor_fifo_play(pMotorData, &buff[0], fsize);
	if (ret < 0) {
		pr_err("%s:failed fifo_play(%d)\n", __func__, ret);
		goto err;
	}
err:
	devm_kfree(pMotorData->dev, buff);

alloc_fail:
	filp_close(fp, current->files);
open_fail:
	set_fs(old_fs);

	mutex_unlock(&pMotorData->lock);

	return size;
}

static DEVICE_ATTR(fifo_vib, S_IWUSR | S_IWGRP,
		NULL, ztm630_motor_store_fifo_vib);
#endif

static struct attribute *sec_motor_attrs[] = {
	&dev_attr_check_i2c.attr,
#ifdef CONFIG_TIZEN_SEC_KERNEL_ENG
	&dev_attr_fifo_vib.attr,
#endif
	NULL
};
static const struct attribute_group sec_motor_attr_group = {
	.attrs = sec_motor_attrs,
};

static int ztm630_motor_sec_init(struct ztm630_motor_data *pMotorData)
{
	int err;

	sec_motor = sec_device_create(NULL, "motor");
	if (IS_ERR(sec_motor)) {
		pr_err("%s:failed to create sec_motor\n", __func__);
		return -ENODEV;
	}
	dev_set_drvdata(sec_motor, pMotorData);

	err = sysfs_create_group(&sec_motor->kobj, &sec_motor_attr_group);
	if (err < 0) {
		pr_err("%s:failed to create sec_motor_attr\n", __func__);
		return err;
	}

	return 0;
}
#endif

static int ztm630_motor_parse_dt(struct i2c_client* client, struct ztm630_motor_pdata *pdata)
{
	const char *motor_type;
	const char *loop_type;
	int err, i;

	pr_info("%s\n", __func__);

	err = of_property_read_string(client->dev.of_node,"ztm630_motor,motor-type", &motor_type);
	if (err < 0) {
		pr_err("%s:motor-type read fail(%d)\n", __func__, err);
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
		pr_err("%s:Wrong motor type[%s]\n", __func__, motor_type);
		return -ENODEV;
	}
	pr_info("%s:motor-type = %s\n", __func__, motor_type);

	err = of_property_read_string(client->dev.of_node,
			"ztm630_motor,loop-type", &loop_type);
	if (err < 0) {
		pr_err("%s:loop-type read fail(%d)\n", __func__, err);
		return -ENODEV;
	}
	if (!strcmp(loop_type, "open"))
		pdata->meLoop = OPEN_LOOP;
	else if (!strcmp(loop_type, "closed"))
		pdata->meLoop = CLOSED_LOOP;
	else {
		pr_err("%s: Wrong loop type: %s\n", __func__, loop_type);
		return -ENODEV;
	}
	pr_info("%s:loop-type[%s]\n", __func__, loop_type);

	pdata->break_mode = of_property_read_bool(client->dev.of_node,
			"ztm630_motor,break-on");
	pr_info("%s:break-on[%d]\n", __func__, (int)pdata->break_mode);

	err = of_property_read_u32(client->dev.of_node,
			"ztm630_motor,brake-delay-ms", &pdata->break_delay);
	if (err < 0) {
		pdata->break_delay = DEFAULT_BRAKE_DELAY;
		dev_warn(&client->dev, "%s: brake-delay-ms read fail(%d) :%d\n",
				__func__, err, pdata->break_delay);
	} else
		pr_info("%s:brake-delay-ms[%d]\n", __func__, pdata->break_delay);

	pdata->count_init_regs = of_property_count_u32_elems(client->dev.of_node,
			"ztm630_motor,regs-init");
	if (pdata->count_init_regs > 0) {
		pdata->init_regs = devm_kzalloc(&client->dev,
				sizeof(u32) * pdata->count_init_regs, GFP_KERNEL);
		err = of_property_read_u32_array(client->dev.of_node,
				"ztm630_motor,regs-init",
				(u32 *)pdata->init_regs, pdata->count_init_regs);
		if (err < 0) {
			pr_err("%s:regs-init read fail(%d)\n", __func__, err);
			return -ENODEV;
		}
		pdata->count_init_regs /= 2;
	}
	pr_info("%s:regs-init count[%d]\n", __func__, pdata->count_init_regs);

	pdata->en_gpio = of_get_named_gpio(client->dev.of_node,
			"ztm630_motor,motor_en", 0);
	if (pdata->en_gpio < 0)
		pr_debug("%s: motor_en read fail(%d)\n",
				__func__, pdata->en_gpio);
	else
		pr_info("%s:motor_en[%d]\n", __func__, pdata->en_gpio);

	err = of_property_read_string(client->dev.of_node,
			"ztm630_motor,regulator-name", &pdata->regulator_name);
	if (err < 0)
		pr_warn("%s: regulator-name read fail(%d)\n",
				__func__, err);
	else
		pr_info("%s:regulator-name[%s]\n", __func__, pdata->regulator_name);

	err = of_property_read_u32(client->dev.of_node,
			"ztm630_motor,adc-sampling-time", &pdata->adc_sampling_time);
	if (err < 0) {
		pdata->adc_sampling_time = DEFAULT_ADC_SAMPLING_TIME;
		pr_warn("%s: adc-sampling-time read fail(%d) :%d\n",
				__func__, err, pdata->adc_sampling_time);
	} else
		pr_info("%s:adc-sampling-time[%d]\n", __func__, pdata->adc_sampling_time);

	err = of_property_read_u32(client->dev.of_node,
			"ztm630_motor,soft-en-delay-ms", &pdata->soft_en_delay);
	if (err < 0) {
		pdata->soft_en_delay = DEFAULT_SOFT_EN_DELAY;
		pr_warn("%s: soft-en-delay-ms read fail(%d) :%d\n",
				__func__, err, pdata->soft_en_delay);
	} else
		pr_info("%s:soft-en-delay-ms[%d]\n", __func__, pdata->soft_en_delay);

	/* array elements of strength are assigned for each intensity level
	   starting from level 1 and increasing */
	pdata->count_strength = of_property_count_u32_elems(client->dev.of_node,
			"ztm630_motor,strength");
	pr_info("%s:strength count[%d]\n", __func__, pdata->count_strength);
	if (pdata->count_strength > 0) {
		pdata->strength = devm_kzalloc(&client->dev,
				sizeof(u32) * pdata->count_strength, GFP_KERNEL);
		err = of_property_read_u32_array(client->dev.of_node,
				"ztm630_motor,strength",
				(u32 *)pdata->strength, pdata->count_strength);
		if (err < 0) {
			pr_err("%s:strength read fail(%d)\n", __func__, err);
		}

		for (i = 0; i < pdata->count_strength; i++) {
			pr_info("%s:strength[%d]=[%d]\n", __func__, i, pdata->strength[i]);
		}
	}

	/* array elements of frequency are assigned for each intensity level
	   starting from level 1 and increasing */
	pdata->count_frequency = of_property_count_u32_elems(client->dev.of_node,
			"ztm630_motor,frequency");
	pr_info("%s:frequency count[%d]", __func__, pdata->count_frequency);
	if (pdata->count_frequency > 0) {
		pdata->frequency = devm_kzalloc(&client->dev,
				sizeof(u32) * pdata->count_frequency, GFP_KERNEL);
		err = of_property_read_u32_array(client->dev.of_node,
				"ztm630_motor,frequency",
				(u32 *)pdata->frequency, pdata->count_frequency);
		if (err < 0) {
			pr_err("%s:frequency read fail(%d)\n", __func__, err);
			return -ENODEV;
		}

		for (i = 0; i < pdata->count_frequency; i++) {
			pr_info("%s:frequency[%d]=[%d]\n", __func__, i, pdata->frequency[i]);
		}
	}

	err = of_property_read_u32(client->dev.of_node,
			"ztm630_motor,overdrive-num", &pdata->overdrive_num);
	if (err < 0) {
		pr_debug("%s: overdrive-num read fail(%d) :%d\n",
				__func__, err, pdata->overdrive_num);
	} else
		pr_info("%s:overdrive-num[%d]\n", __func__, pdata->overdrive_num);

	return 0;
}

static int ztm630_motor_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct ztm630_motor_data *pMotorData;
	struct ztm630_motor_pdata *pMotorPdata;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_err("%s:I2C check failed\n", __func__);
		return -ENODEV;
	}

	/* platform_data init */
	if (client->dev.of_node) {
		pMotorPdata = devm_kzalloc(&client->dev,
				sizeof(struct ztm630_motor_pdata), GFP_KERNEL);
		if (!pMotorPdata) {
			pr_err("%s:unable to allocate pdata memory\n", __func__);
			return -ENOMEM;
		}
		err = ztm630_motor_parse_dt(client, pMotorPdata);
		if (err < 0) {
			pr_err("%s:fail to read DT %d\n", __func__, err);
			return -ENODEV;
		}
	}

	pMotorData = devm_kzalloc(&client->dev, sizeof(struct ztm630_motor_data), GFP_KERNEL);
	if (pMotorData == NULL) {
		pr_err("%s:no memory\n", __func__);
		return -ENOMEM;
	}

	pMotorData->client = client;
	pMotorData->dev = &client->dev;
	pMotorData->msPlatData = pMotorPdata;
	g_Ztm630MotorData = pMotorData;
	i2c_set_clientdata(client, pMotorData);

	pMotorData->mpRegmap = devm_regmap_init_i2c(client, &ztm630_motor_i2c_regmap);
	if (IS_ERR(pMotorData->mpRegmap)) {
		err = PTR_ERR(pMotorData->mpRegmap);
		pr_err("%s:Failed to allocate register map(%d)\n",__func__,err);
		return err;
	}

	if (pMotorPdata->regulator_name) {
		pMotorData->regulator
			= devm_regulator_get(pMotorData->dev, pMotorPdata->regulator_name);
		if (IS_ERR(pMotorData->regulator)) {
			pr_err("%s:Failed to get moter power supply.\n", __func__);
			return -EFAULT;
		}
		err = regulator_set_voltage(pMotorData->regulator, MOTOR_VCC, MOTOR_VCC);
		if (err < 0)
			pr_err("%s:Failed to set moter power %duV(%d)\n",
			__func__, MOTOR_VCC, err);
		err = regulator_enable(pMotorData->regulator);
		if (err < 0) {
			pr_err("%s:%s:enable fail(%d)\n", __func__,
					pMotorPdata->regulator_name, err);
			return -EFAULT;
		} else
			pr_info("%s:%s enable\n",
					__func__, pMotorPdata->regulator_name);
	}

	if (pMotorPdata->en_gpio > 0) {
		err = devm_gpio_request_one(pMotorData->dev, pMotorPdata->en_gpio,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motor");
		if (err < 0) {
			pr_err("%s:en_gpio request fail %d\n", __func__, err);
			return -EFAULT;
		}
	}
	pMotorData->en_gpio = pMotorPdata->en_gpio;

	err = ztm630_motor_reg_read(pMotorData, MOTOR_REG_ADC_SAMPLING_TIME);
	if (err < 0) {
		pr_err("%s:i2c bus fail(%d)\n", __func__, err);
		return -EFAULT;
	}
	pr_info("%s:ADC_SAMPLING_TIME(0x%x)\n", __func__, err);
	pMotorData->mnDeviceID = err;

	err = ztm630_motor_platform_init(pMotorData);
	if (err < 0) {
		pr_err("%s:dev_init_platform failed(%d)\n", __func__, err);
		return -EFAULT;
	}

	err = ztm630_motor_haptics_init(pMotorData);
	if (err < 0) {
		pr_err("%s:ztm630_motor_haptics_init failed(%d)\n", __func__, err);
		return -EFAULT;
	}

	ztm630_motor_debug_init(pMotorData);
#ifdef CONFIG_SEC_SYSFS
	ztm630_motor_sec_init(pMotorData);
#endif

	pr_info("%s:succeeded\n", __func__);

	return 0;
}

void ztm630_motor_shutdown(struct i2c_client *client)
{
	struct ztm630_motor_data *pMotorData = i2c_get_clientdata(client);
	struct ztm630_motor_pdata *pMotorPdata = pMotorData->msPlatData;
	int err;

	pr_info("%s", __func__);

	if (pMotorData->regulator) {
		err = regulator_disable(pMotorData->regulator);
		if (err < 0)
			pr_err("%s:%s:disable fail(%d)", __func__,
				pMotorPdata->regulator_name, err);
		else
			pr_info("%s:%s disable\n", __func__,
				pMotorPdata->regulator_name);
	}
}

#if defined(CONFIG_OF)
static struct of_device_id haptic_dt_ids[] = {
	{ .compatible = HAPTICS_DEVICE_NAME },
	{ },
};
MODULE_DEVICE_TABLE(of, haptic_dt_ids);
#endif /* CONFIG_OF */

static struct i2c_device_id ztm630_motor_id_table[] =
{
	{ HAPTICS_DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ztm630_motor_id_table);

static struct i2c_driver ztm630_motor_driver =
{
	.driver = {
		.name = HAPTICS_DEVICE_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = haptic_dt_ids,
#endif /* CONFIG_OF */
	},
	.id_table = ztm630_motor_id_table,
	.probe = ztm630_motor_probe,
	.shutdown = ztm630_motor_shutdown,
};

static int __init ztm630_motor_init(void)
{
	return i2c_add_driver(&ztm630_motor_driver);
}

static void __exit ztm630_motor_exit(void)
{
	i2c_del_driver(&ztm630_motor_driver);
}

module_init(ztm630_motor_init);
module_exit(ztm630_motor_exit);

MODULE_AUTHOR("samsung");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
