/*
 *  input_assistant.c
 *  for Samsung Electronics
 *
 *  Copyright (C) 2017 Samsung Electronics
 *  Sang-Min,Lee <lsmin.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/trm.h>
#include <linux/input/input-assistant.h>
#ifdef CONFIG_DISPLAY_EARLY_DPMS
#include <drm/tgm_drm.h>
#endif
#ifdef CONFIG_OF
#include <linux/of.h>
#endif

//#define ENABLE_DEBUG
#define GPIO_KEYS_DEV_NAME	"gpio_keys"
#define POWER_KEYS_DEV_NAME	"s2mpw02-power-keys"
#define TSP_DEV_NAME		"sec_touchscreen"
#define BEZEL_DEV_NAME		"tizen_detent"
#define TSP_FINGER_RELEASE	-1
#define MAX_TSP_EVENT		1000

#define WAKELOCK_TIME		HZ/10

const struct input_assistant_data *g_assistant_data;

#ifdef CONFIG_DISPLAY_EARLY_DPMS
static void input_assistant_send_early_dpms_event(void)
{
	const struct display_early_dpms_nb_event event = {
		.id = DISPLAY_EARLY_DPMS_ID_PRIMARY,
		.data = (void *)true,
	};

	display_early_dpms_nb_send_event (DISPLAY_EARLY_DPMS_MODE_SET,
					(void *)&event);
}
#endif

static void input_assistant_init_tsp(struct input_handle *handle)
{
	struct input_dev *dev = handle->dev;
	struct input_handler *handler = handle->handler;
	struct input_assistant_data *data = handler->private;
	struct input_assistant_tsp *tsp_data = &data->tsp_data;
	struct input_absinfo *id_info = &dev->absinfo[ABS_MT_TRACKING_ID];

	if (id_info->maximum == 0)
		tsp_data->max_slot = TSP_NUM_MAX;
	else
		tsp_data->max_slot = id_info->maximum;

	pr_debug("%s:[%s][%d]", __func__, dev->name, tsp_data->max_slot);
}

static void input_assistant_tsp_booster_work(struct work_struct *work)
{
	struct input_assistant_data *data = container_of(work,
						       struct input_assistant_data,
						       tsp_booster_work);
	struct input_assistant_tsp *tsp_data = &data->tsp_data;

	if (data == NULL) {
		pr_err("%s:data is NULL\n", __func__);
		return;
	}

	if (tsp_data->slot_cnt == 0)
		touch_booster_release();
	else if (tsp_data->slot_cnt == 1)
		touch_booster_press();

	return;
}

static void input_assistant_tsp_logger(struct input_assistant_tsp *tsp_data, __u16 code)
{
	tsp_data->in_data[tsp_data->last_slot].need_update = false;

	switch (code) {
	case ABS_MT_TRACKING_ID:
		if (tsp_data->in_data[tsp_data->last_slot].tracking_id == TSP_FINGER_RELEASE)
			pr_info("%s:[R][%d] x=%d, y=%d, z=%d, M=%d, m=%d, e=%d\n", __func__,
				tsp_data->in_data[tsp_data->last_slot].slot_num,
				tsp_data->in_data[tsp_data->last_slot].x,
				tsp_data->in_data[tsp_data->last_slot].y,
				tsp_data->in_data[tsp_data->last_slot].z,
				tsp_data->in_data[tsp_data->last_slot].wmajor,
				tsp_data->in_data[tsp_data->last_slot].wminor,
				tsp_data->event_cnt);
		break;
	case ABS_MT_SLOT:
		pr_info("%s:[P][%d] x=%d, y=%d, z=%d, M=%d, m=%d, e=%d\n", __func__,
			tsp_data->in_data[tsp_data->last_slot].slot_num,
			tsp_data->in_data[tsp_data->last_slot].x,
			tsp_data->in_data[tsp_data->last_slot].y,
			tsp_data->in_data[tsp_data->last_slot].z,
			tsp_data->in_data[tsp_data->last_slot].wmajor,
			tsp_data->in_data[tsp_data->last_slot].wminor,
			tsp_data->event_cnt);
		break;
	case ABS_MT_PALM:
		pr_info("%s:[%s][%d] Palm, e=%d\n", __func__,
			tsp_data->in_data[tsp_data->last_slot].palm ? "P":"R",
			tsp_data->in_data[tsp_data->last_slot].slot_num,
			tsp_data->event_cnt);
		break;
	default:
		pr_err("%s:Unknown code[0x%02x]\n", __func__, code);
		break;
	}
}

static void input_assistant_tsp_handler(struct input_handle *handle,
			       const struct input_value *vals, unsigned int count)
{
	struct input_handler *handler = handle->handler;
	struct input_assistant_data *data = handler->private;
	struct input_assistant_tsp *tsp_data = &data->tsp_data;
	int i;
	bool need_update = false;

	wake_lock_timeout(&data->wake_lock, WAKELOCK_TIME);

	for (i = 0; i < count; i++) {
		if (vals[i].type != EV_ABS)
			continue;
		switch (vals[i].code) {
		case ABS_MT_TRACKING_ID:
			tsp_data->in_data[tsp_data->last_slot].tracking_id = vals[i].value;
			tsp_data->in_data[tsp_data->last_slot].slot_num = tsp_data->last_slot;
			if (tsp_data->in_data[tsp_data->last_slot].tracking_id == TSP_FINGER_RELEASE) {
				input_assistant_tsp_logger(tsp_data, ABS_MT_TRACKING_ID);
				tsp_data->slot_cnt--;
			} else {
				tsp_data->in_data[tsp_data->last_slot].need_update = true;
				need_update = true;
				tsp_data->slot_cnt++;
			}
			queue_work(system_highpri_wq, &data->tsp_booster_work);
			break;
		case ABS_MT_SLOT:
			if (tsp_data->in_data[tsp_data->last_slot].need_update) {
				need_update = false;
				input_assistant_tsp_logger(tsp_data, ABS_MT_SLOT);
			}
			tsp_data->last_slot = vals[i].value;
			break;
		case ABS_MT_PALM:
			tsp_data->in_data[tsp_data->last_slot].palm = vals[i].value;
			input_assistant_tsp_logger(tsp_data, ABS_MT_PALM);
			break;
		case ABS_MT_POSITION_X:
			tsp_data->in_data[tsp_data->last_slot].x = vals[i].value;
			break;
		case ABS_MT_POSITION_Y:
			tsp_data->in_data[tsp_data->last_slot].y = vals[i].value;
			break;
		case ABS_MT_PRESSURE:
			tsp_data->in_data[tsp_data->last_slot].z = vals[i].value;
			break;
		case ABS_MT_WIDTH_MAJOR:
			tsp_data->in_data[tsp_data->last_slot].wmajor = vals[i].value;
			break;
		case ABS_MT_WIDTH_MINOR:
			tsp_data->in_data[tsp_data->last_slot].wminor = vals[i].value;
			break;
		default:
			pr_err("%s:Unknown code[0x%02x]\n", __func__, vals[i].code);
			break;
		}
	}

	if (need_update) {
		for (i = 0; i < tsp_data->max_slot; i++) {
			if (tsp_data->in_data[i].need_update)
				input_assistant_tsp_logger(tsp_data, ABS_MT_SLOT);
		}
	}

	if (tsp_data->event_cnt > MAX_TSP_EVENT)
		tsp_data->event_cnt = 0;
	else
		tsp_data->event_cnt++;
}

static void input_assistant_key_logger(const struct input_value *vals)
{
	switch (vals->code) {
	case KEY_POWER:
		pr_info("%s: [%s]KEY_POWER\n", __func__, vals->value ? "P":"R");
		break;
	case KEY_PHONE:
		pr_info("%s: [%s]KEY_PHONE\n", __func__, vals->value ? "P":"R");
		break;
	case KEY_BACK:
		pr_info("%s: [%s]KEY_BACK\n", __func__, vals->value ? "P":"R");
		break;
	case KEY_HOME:
		pr_info("%s: [%s]KEY_HOME\n", __func__, vals->value ? "P":"R");
		break;
	case KEY_VOLUMEDOWN:
		pr_info("%s: [%s]KEY_VOLUMEDOWN\n", __func__, vals->value ? "P":"R");
		break;
	case KEY_VOLUMEUP:
		pr_info("%s: [%s]KEY_VOLUMEUP\n", __func__, vals->value ? "P":"R");
		break;
	default:
		pr_info("%s: [%s]0x%02x\n", __func__, vals->value ? "P":"R", vals->code);
		break;
	}
}

#ifdef BACK_KEY_BOOSTER
static void input_assistant_key_booster_work(struct work_struct *work)
{
	back_key_booster_turn_on();
}
#endif

static void input_assistant_key_handler(struct input_handle *handle,
			       const struct input_value *vals, unsigned int count)
{
	struct input_handler *handler = handle->handler;
	struct input_assistant_data *data = handler->private;
#ifdef CONFIG_DISPLAY_EARLY_DPMS
	struct input_dev *input_dev = handle->dev;
	struct device *dev = &input_dev->dev;
	struct dev_pm_info *power = &dev->power;
#endif
	int i;

	wake_lock_timeout(&data->wake_lock, WAKELOCK_TIME);

	for (i = 0; i < count; i++) {
		if (vals[i].type != EV_KEY)
			continue;
#ifdef BACK_KEY_BOOSTER
		if (vals[i].code == KEY_BACK)
			queue_work(system_highpri_wq, &data->key_booster_work);
#endif
#ifdef CONFIG_DISPLAY_EARLY_DPMS
		if ((power->is_suspended) && (vals[i].value))
			input_assistant_send_early_dpms_event();
#endif
		input_assistant_key_logger(&vals[i]);
#ifdef CONFIG_EXYNOS_SNAPSHOT_CRASH_KEY
		exynos_ss_check_crash_key(vals[i].code, vals[i].value);
#endif
	}
}

#ifdef ROTARY_BOOSTER
static void input_assistant_bezel_booster_work(struct work_struct *work)
{
	rotary_booster_turn_on();
}
#endif

#define MAX_dir_str 3
static void input_assistant_bezel_handler(struct input_handle *handle,
			       const struct input_value *vals, unsigned int count)
{
	struct input_handler *handler = handle->handler;
	struct input_assistant_data *data = handler->private;
#ifdef CONFIG_DISPLAY_EARLY_DPMS
	struct input_dev *input_dev = handle->dev;
	struct device *dev = &input_dev->dev;
	struct dev_pm_info *power = &dev->power;
#endif
	int i, wheel = 0, x = 0, update = 0;
	const char*  dir_str[MAX_dir_str] = {
		"ccw",
		"none",
		"cw",
	};

	wake_lock_timeout(&data->wake_lock, WAKELOCK_TIME);

	for (i = 0; i < count; i++) {
		if (vals[i].type != EV_REL)
			continue;


		switch (vals[i].code) {
		case REL_WHEEL:
#ifdef CONFIG_DISPLAY_EARLY_DPMS
			if (power->is_suspended)
				input_assistant_send_early_dpms_event();
#endif
#ifdef ROTARY_BOOSTER
			queue_work(system_highpri_wq, &data->bezel_booster_work);
#endif
			wheel = vals[i].value+1;
			update = true;
			break;
		case REL_X:
			x = ~vals[i].value & 0x07;
			break;
		default:
			pr_err("%s:Unknown code[0x%02x]\n", __func__, vals[i].code);
			break;
		}
	}

	if (update) {
		if (((wheel) >= MAX_dir_str) || ((wheel) < 0))
			pr_err("%s: invalid state:[%d][%d]\n", __func__, x, wheel-1);
		else
			pr_info("%s: state:[%d][%s]\n", __func__, x, dir_str[wheel]);
	}
}

static void input_assistant_events(struct input_handle *handle,
			       const struct input_value *vals, unsigned int count)
{
	struct input_dev *dev = handle->dev;

	if (!strncmp(dev->name, TSP_DEV_NAME, strlen(TSP_DEV_NAME)))
		input_assistant_tsp_handler(handle, vals, count);
	else if (!strncmp(dev->name, GPIO_KEYS_DEV_NAME, strlen(GPIO_KEYS_DEV_NAME)))
		input_assistant_key_handler(handle, vals, count);
	else if (!strncmp(dev->name, POWER_KEYS_DEV_NAME, strlen(POWER_KEYS_DEV_NAME)))
		input_assistant_key_handler(handle, vals, count);
	else if (!strncmp(dev->name, BEZEL_DEV_NAME, strlen(BEZEL_DEV_NAME)))
		input_assistant_bezel_handler(handle, vals, count);
}

static bool input_assistant_match(struct input_handler *handler,
				struct input_dev *dev)
{
	struct input_assistant_data *data = handler->private;
	struct input_assistant_pdata *pdata = data->pdata;
	int i;

	for (i = 0; i < pdata->support_dev_num; i++) {
		if (!strncmp(dev->name, pdata->support_dev_name[i],
			strlen(pdata->support_dev_name[i])))
			return 1;
	}

	dev_dbg(&dev->dev, "%s: not support device[%s]\n", __func__, dev->name);

	return 0;
}

static int input_assistant_connect(struct input_handler *handler,
				struct input_dev *dev,
				const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "input_assistant";

	ret = input_register_handle(handle);
	if (ret) {
		dev_err(&dev->dev,
			"%s: Failed to register input assistant handler(%d)\n",
			__func__, ret);
		kfree(handle);
		return ret;
	}

	ret = input_open_device(handle);
	if (ret) {
		dev_err(&dev->dev,
			"%s: Failed to open input assistant device(%d)\n",
			__func__, ret);
		input_unregister_handle(handle);
		kfree(handle);
		return ret;
	}

	if (!strncmp(dev->name, TSP_DEV_NAME, strlen(TSP_DEV_NAME))) {
		input_assistant_init_tsp(handle);
	}

	dev_info(&dev->dev, "%s: connected %s.\n", __func__, dev->name);

	return 0;
}

static void input_assistant_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static void input_assistant_set_ids(struct input_device_id *ids, unsigned int type,
				 unsigned int code)
{
	switch (type) {
	case EV_KEY:
		ids->flags = INPUT_DEVICE_ID_MATCH_KEYBIT;
		__set_bit(code, ids->keybit);
		break;

	case EV_REL:
		ids->flags = INPUT_DEVICE_ID_MATCH_RELBIT;
		__set_bit(code, ids->relbit);
		break;

	case EV_ABS:
		ids->flags = INPUT_DEVICE_ID_MATCH_ABSBIT;
		__set_bit(code, ids->absbit);
		break;

	case EV_MSC:
		ids->flags = INPUT_DEVICE_ID_MATCH_MSCIT;
		__set_bit(code, ids->mscbit);
		break;

	case EV_SW:
		ids->flags = INPUT_DEVICE_ID_MATCH_SWBIT;
		__set_bit(code, ids->swbit);
		break;

	case EV_LED:
		ids->flags = INPUT_DEVICE_ID_MATCH_LEDBIT;
		__set_bit(code, ids->ledbit);
		break;

	case EV_SND:
		ids->flags = INPUT_DEVICE_ID_MATCH_SNDBIT;
		__set_bit(code, ids->sndbit);
		break;

	case EV_FF:
		ids->flags = INPUT_DEVICE_ID_MATCH_FFBIT;
		__set_bit(code, ids->ffbit);
		break;

	case EV_PWR:
		/* do nothing */
		break;

	default:
		pr_err("%s: unknown type %u (code %u)\n",
			__func__, type, code);
		return;
	}

	ids->flags |= INPUT_DEVICE_ID_MATCH_EVBIT;
	__set_bit(type, ids->evbit);
}

#ifdef CONFIG_OF
static int input_assistant_parse_dt(struct device *dev,
			struct input_assistant_pdata *pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	enum mkey_check_option option;
	unsigned int *map;
	unsigned int *type;
	const char *out_prop;
	int i, j, rc, map_size, type_size;

	rc = of_property_read_u32(np, "input_assistant,num_map",
			(unsigned int *)&pdata->num_map);
	if (rc) {
		dev_err(dev, "%s: failed to get num_map.\n", __func__);
		goto error;
	}

	rc = of_property_read_u32(np, "input_assistant,map_key",
			(unsigned int *)&option);
	if (rc) {
		dev_err(dev, "Unable to read %s\n", "input_assistant,map_key");
		goto error;
	}

	pdata->mmap = devm_kzalloc(dev,
			sizeof(struct input_assistant_mmap)*pdata->num_map, GFP_KERNEL);
	if (!pdata->mmap) {
		rc = -ENOMEM;
		goto error;
	}

	for (i = 0; i < pdata->num_map; i++) {
		rc = of_property_read_string_index(np, "input_assistant,map_codes", i, &out_prop);
		if (rc < 0) {
			dev_err(dev, "%s: failed to get %d map_codes string.[%s]\n", __func__, i, out_prop);
			goto error;

		}
		prop = of_find_property(np, out_prop, NULL);
		if (!prop) {
			dev_err(dev, "%s: failed to get %d map_codes property[%s]\n", __func__, i, out_prop);
			rc = -EINVAL;
			goto error;
		}
		map_size = prop->length / sizeof(unsigned int);
		pdata->mmap[i].num_mkey = map_size;
		map = devm_kzalloc(dev, sizeof(unsigned int)*map_size, GFP_KERNEL);
		if (!map) {
			dev_err(dev, "%s: Failed to allocate map memory.\n", __func__);
			rc = -ENOMEM;
			goto error;
		}

		rc = of_property_read_u32_array(np, out_prop, map, map_size);
		if (rc && (rc != -EINVAL)) {
			dev_err(dev, "%s: Unable to read %s array\n", __func__, out_prop);
			goto error;
		}

		rc = of_property_read_string_index(np, "input_assistant,map_types", i, &out_prop);
		if (rc < 0) {
			dev_err(dev, "%s: failed to get %d map_types string[%s]\n", __func__, i, out_prop);
			rc = -EINVAL;
			goto error;

		}
		prop = of_find_property(np, out_prop, NULL);
		if (!prop) {
			dev_err(dev, "%s: failed to get %d map_codes property[%s]\n", __func__, i, out_prop);
			rc = -EINVAL;
			goto error;
		}

		type_size = prop->length / sizeof(unsigned int);
		type = devm_kzalloc(dev, sizeof(unsigned int)*type_size, GFP_KERNEL);
		if (!type) {
			dev_err(dev, "%s: Failed to allocate key memory.\n", __func__);
			rc = -ENOMEM;
			goto error;
		}

		rc = of_property_read_u32_array(np, out_prop, type, type_size);
		if (rc && (rc != -EINVAL)) {
			dev_err(dev, "Unable to read %s\n", out_prop);
			goto error;
		}

		pdata->mmap[i].mkey_map = devm_kzalloc(dev,
					sizeof(struct input_assistant_mkey)*
					map_size, GFP_KERNEL);
		if (!pdata->mmap[i].mkey_map) {
			dev_err(dev, "%s: Failed to allocate memory\n", __func__);
			rc = -ENOMEM;
			goto error;
		}

		for (j = 0; j < map_size; j++) {
			pdata->mmap[i].mkey_map[j].type = type[j];
			pdata->mmap[i].mkey_map[j].code = map[j];
		}
	}

	rc = of_property_read_u32(np, "input_assistant,dev_num",
			(unsigned int *)&pdata->support_dev_num);
	if (rc) {
		dev_err(dev, "%s: failed to get support_dev_num.\n", __func__);
		goto error;
	}

	pdata->support_dev_name = devm_kzalloc(dev,
			pdata->support_dev_num * sizeof(char *), GFP_KERNEL);
	if (!pdata->support_dev_name) {
		rc = -ENOMEM;
		goto error;
	}

	for (i = 0; i < pdata->support_dev_num; i++) {
		rc = of_property_read_string_index(np, "input_assistant,dev_name_str", i, &out_prop);
		if (rc < 0) {
			dev_err(dev, "failed to get %d dev_name_str string\n", i);
			goto error;
		}
		pdata->support_dev_name[i] = (char *)out_prop;
	}

#ifdef ENABLE_DEBUG
	for (i = 0; i < pdata->num_map; i++) {

		dev_info(dev, "%s: pdata->mmap[%d].num_mkey=[%d]\n",
				__func__, i, pdata->mmap[i].num_mkey);
		for (j = 0; j < pdata->mmap[i].num_mkey; j++) {
			dev_info(dev,
				"%s: pdata->mmap[%d].mkey_map[%d].type=[%d]\n",
				__func__, i, j, pdata->mmap[i].mkey_map[j].type);
			dev_info(dev,
				"%s: pdata->mmap[%d].mkey_map[%d].code=[%d]\n",
				__func__, i, j, pdata->mmap[i].mkey_map[j].code);
			dev_info(dev,
				"%s: pdata->mmap[%d].mkey_map[%d].option=[%d]\n",
				__func__, i, j, pdata->mmap[i].mkey_map[j].option);
		}
	}

	dev_info(dev, "%s: pdata->support_dev_num=[%d]\n", __func__, pdata->support_dev_num);
	for (i = 0; i < pdata->support_dev_num; i++)
		dev_info(dev, "%s: pdata->support_dev_name[%d] = [%s]\n",
			__func__, i, pdata->support_dev_name[i]);
#endif
	return 0;
error:
	return rc;
}

static struct input_handler input_assistant_handler = {
	.events = input_assistant_events,
	.match = input_assistant_match,
	.connect = input_assistant_connect,
	.disconnect = input_assistant_disconnect,
	.name = "input_assistant",
};

static const struct of_device_id input_assistant_of_match[] = {
	{ .compatible = "input-assistant", },
	{ },
};
MODULE_DEVICE_TABLE(of, input_assistant_of_match);
#else
static int input_assistant_parse_dt(struct device *dev,
			struct input_assistant_pdata *pdata)
{
	dev_err(dev, "%s\n", __func__);
	return -ENODEV;
}
#endif

static int input_assistant_probe(struct platform_device *pdev)
{

	struct input_assistant_pdata *pdata;
	struct input_assistant_data *assistant_data;
	struct input_device_id *input_assistant_ids;
	int ret, i, j, k;
	int total_num_key = 0;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct input_assistant_pdata),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "%s: Failed to allocate pdata memory\n", __func__);
			ret = -ENOMEM;
			goto error_1;
		}
		ret = input_assistant_parse_dt(&pdev->dev, pdata);
		if (ret) {
			dev_err(&pdev->dev, "%s: Fail parse device tree.\n", __func__);
			ret = -EINVAL;
			goto error_1;
		}
	} else {
		pdata = pdev->dev.platform_data;
		if (!pdata) {
			dev_err(&pdev->dev, "%s: Fail input assistant platform data.\n", __func__);
			ret = -EINVAL;
			goto error_1;
		}
	}

	if (pdata->num_map == 0) {
		dev_err(&pdev->dev,
			"%s: No input platform data. num_mkey is NULL.\n", __func__);
		ret = -EINVAL;
		goto error_1;
	}

	assistant_data = kzalloc(sizeof(struct input_assistant_data), GFP_KERNEL);
	if (!assistant_data) {
		ret = -ENOMEM;
		goto error_1;
	}

	for (i = 0; i < pdata->num_map; i++)
		total_num_key += pdata->mmap[i].num_mkey;

	input_assistant_ids =
		kzalloc(sizeof(struct input_device_id[(total_num_key + 1)]),
			GFP_KERNEL);
	if (!input_assistant_ids) {
		dev_err(&pdev->dev, "Failed to allocate input_assistant_ids memory\n");
		ret = -ENOMEM;
		goto error_2;
	}
	memset(input_assistant_ids, 0x00, sizeof(struct input_device_id));

	for (i = 0, k = 0; i < pdata->num_map; i++) {
		for (j = 0; j < pdata->mmap[i].num_mkey; j++) {
			input_assistant_set_ids(&input_assistant_ids[k++],
					     pdata->mmap[i].mkey_map[j].type,
					     pdata->mmap[i].mkey_map[j].code);
		}
	}

	dev_set_drvdata(&pdev->dev, assistant_data);

	assistant_data->pdev = pdev;
	assistant_data->pdata = pdata;
	g_assistant_data = assistant_data;

	input_assistant_handler.private = assistant_data;
	input_assistant_handler.id_table = input_assistant_ids;

	wake_lock_init(&assistant_data->wake_lock,
			WAKE_LOCK_SUSPEND, "input_assistant_wake_lock");

	INIT_WORK(&assistant_data->tsp_booster_work, input_assistant_tsp_booster_work);
#ifdef BACK_KEY_BOOSTER
	INIT_WORK(&assistant_data->key_booster_work, input_assistant_key_booster_work);
#endif
#ifdef ROTARY_BOOSTER
	INIT_WORK(&assistant_data->bezel_booster_work, input_assistant_bezel_booster_work);
#endif
	ret = input_register_handler(&input_assistant_handler);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register input_assistant_handler\n");
		goto error_3;
	}

#ifdef CONFIG_DISPLAY_EARLY_DPMS
	device_set_early_complete(&pdev->dev, EARLY_COMP_SLAVE);
#endif


	dev_info(&pdev->dev, "%s: done.\n", __func__);

	return 0;

error_3:
	wake_lock_destroy(&assistant_data->wake_lock);
	kfree(input_assistant_ids);
error_2:
	kfree(assistant_data);
error_1:
	return ret;

}

static int input_assistant_remove(struct platform_device *dev)
{
	struct input_assistant_data *assistant_data = platform_get_drvdata(dev);

	wake_lock_destroy(&assistant_data->wake_lock);
	kfree(input_assistant_handler.id_table);
	input_unregister_handler(&input_assistant_handler);
	kfree(assistant_data);
	platform_set_drvdata(dev, NULL);

	return 0;
}

static struct platform_driver input_assistant_driver = {
	.probe = input_assistant_probe,
	.remove = input_assistant_remove,
	.driver = {
			.owner	= THIS_MODULE,
			.name = "input_assistant",
#ifdef CONFIG_OF
			.of_match_table = input_assistant_of_match,
#endif
		   },
};

static int __init input_assistant_init(void)
{
	return platform_driver_register(&input_assistant_driver);
}

static void __exit input_assistant_exit(void)
{
	platform_driver_unregister(&input_assistant_driver);
}

device_initcall(input_assistant_init);
module_exit(input_assistant_exit);

MODULE_AUTHOR("Sang-Min,Lee <lsmin.lee@samsung.com>");
MODULE_LICENSE("GPL");
