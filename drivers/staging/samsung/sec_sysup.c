/*
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *
 * Samsung TN debugging code
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/reboot.h>

#include <linux/sec_ext.h>
#include <linux/sec_sysfs.h>
//#include <linux/sec_class.h>

#if defined(CONFIG_TIZEN)
#include <linux/sec_sysup.h>
#include <linux/spu-verify.h>
#endif

#define SEC_EDTBO_FILENAME		"/spu/edtbo/edtbo.img"
#define SEC_PARAM_EXTRA_MAX		10240
#define SEC_PARAM_STR_MAX		2048

static unsigned long edtbo_ver = 0;
static unsigned long edtbo_offset = 0;
static unsigned long param_offset = 0;

#if defined(CONFIG_TIZEN)
static int sec_sysup_get_app_firmware_magic(const char *path, char *magic)
{
	char buf[5] = {0, };
	struct file *filp = NULL;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("--> %s) filp_open failed\n", __func__);
		set_fs(old_fs);
		return PTR_ERR(filp);
	}

	vfs_read(filp, buf, 4, &filp->f_pos);

	filp_close(filp, NULL);
	set_fs(old_fs);

	return snprintf(magic, 5, "%s", buf);
}

static int sec_sysup_get_spu_firmware_magic(const char *name, char *magic)
{
	int offset;
	char buf[5] = {0, };
	struct file *filp = NULL;
	mm_segment_t old_fs;

	offset = sec_sysup_get_firmware_offset(name);
	if (offset < 0)
		return offset;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(SEC_SYSUP_SPU_PATH, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("--> %s) filp_open failed\n", __func__);
		set_fs(old_fs);
		return PTR_ERR(filp);
	}

	vfs_llseek(filp, offset, SEEK_SET);
	vfs_read(filp, buf, 4, &filp->f_pos);

	filp_close(filp, NULL);
	set_fs(old_fs);

	return snprintf(magic, 5, "%s", buf);
}

static bool sec_sysup_check_app_firmware_valid(const char *path)
{
	char magic[5] = {0, };
	int ret;

	ret = sec_sysup_get_app_firmware_magic(path, magic);
	if (ret < 0)
		return false;

	if (strncmp(magic, SEC_SYSUP_MAGIC, 4)) {
		pr_err("%s: read app_magic invalid: %s\n", __func__, magic);
		return false;
	}

	return true;
}

bool sec_sysup_check_spu_firmware_valid(const char *name)
{
	char magic[5] = {0, };
	int ret;

	ret = sec_sysup_get_spu_firmware_magic(name, magic);
	if (ret < 0)
		return false;

	if (strncmp(magic, SEC_SYSUP_MAGIC, 4)) {
		pr_err("%s: read spu_magic invalid: %s\n", __func__, magic);
		return false;
	}

	return true;
}

static u32 sec_sysup_get_app_firmware_version(const char *path)
{
	char buf[9] = {0, };
	struct file *filp = NULL;
	mm_segment_t old_fs;
	u32 version;
	int ret;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("--> %s) filp_open failed\n", __func__);
		set_fs(old_fs);
		return PTR_ERR(filp);
	}

	vfs_llseek(filp, 4, SEEK_SET);
	vfs_read(filp, buf, 8, &filp->f_pos);

	filp_close(filp, NULL);
	set_fs(old_fs);

	ret = kstrtou32(buf, 10, &version);
	if (ret)
		return ret;

	return version;
}

u32 sec_sysup_get_spu_firmware_version(const char *name)
{
	int offset;
	char buf[9] = {0, };
	struct file *filp = NULL;
	mm_segment_t old_fs;
	u32 version;
	int ret;

	offset = sec_sysup_get_firmware_offset(name);
	if (offset < 0)
		return offset;

	offset += 4;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(SEC_SYSUP_SPU_PATH, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("--> %s) filp_open failed\n", __func__);
		set_fs(old_fs);
		return PTR_ERR(filp);
	}

	vfs_llseek(filp, offset, SEEK_SET);
	vfs_read(filp, buf, 8, &filp->f_pos);

	filp_close(filp, NULL);
	set_fs(old_fs);

	ret = kstrtou32(buf, 10, &version);
	if (ret)
		return ret;

	return version;
}

int sec_sysup_get_firmware_offset(const char *name)
{
	if (!strncmp(name, "TSP", 3))
		return SEC_SYSUP_TSP_OFFSET;
	else if (!strncmp(name, "SENSORHUB", 9))
		return SEC_SYSUP_SENSORHUB_OFFSET;
	else {
		pr_err("%s not support\n", name);
		return -ENOENT;
	}
}

int sec_sysup_get_firmware_size(const char *name)
{
	int offset;
	char buf[8] = {0, };
	loff_t fsize = 0;
	struct file *filp = NULL;
	mm_segment_t old_fs;

	offset = sec_sysup_get_firmware_offset(name);
	if (offset < 0)
		return offset;

	offset += 12;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(SEC_SYSUP_SPU_PATH, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("--> %s) filp_open failed\n", __func__);
		set_fs(old_fs);
		return PTR_ERR(filp);
	}

	vfs_llseek(filp, offset, SEEK_SET);
	vfs_read(filp, buf, 8, &filp->f_pos);

	filp_close(filp, NULL);
	set_fs(old_fs);

	fsize = simple_strtol(buf, NULL, 10);

	return fsize;
}
#endif

static int __init init_sysup_edtbo(char *str)
{
	char *endstr;
	edtbo_ver = simple_strtoul(str, &endstr, 10);

	if (*endstr != '@' || kstrtoul(endstr + 1, 0, &edtbo_offset)) {
		pr_err("%s: Input string is invalid.\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: Edtbo Version = %lu\n", __func__, edtbo_ver);

	return 0;
}

early_param("tizenboot.sysup.edtbo", init_sysup_edtbo);

static int __init init_sysup_param(char *str)
{
	if (kstrtoul(str, 0, &param_offset)) {
		pr_err("%s: Input string is invalid.\n", __func__);
		return -EINVAL;
	}

	return 0;
}

early_param("tizenboot.sysup.param", init_sysup_param);

#if defined(CONFIG_TIZEN)
static ssize_t sec_firmware_update(char *name, char *path)
{
	struct file *from, *to;
	mm_segment_t old_fs;
	loff_t fsize;
	int offset;
	char *buf = NULL;
	u32 spu_ver, app_ver;

	if (sec_sysup_check_app_firmware_valid(path) == false)
		return -EINVAL;

	if (sec_sysup_check_spu_firmware_valid(name) == false)
		goto write_fw;

	app_ver = sec_sysup_get_app_firmware_version(path);
	spu_ver = sec_sysup_get_spu_firmware_version(name);
	pr_err("%s: app_ver: %u, spu_ver: %u\n", __func__, app_ver, spu_ver);

	if (app_ver <= spu_ver)
		return spu_ver;

write_fw:
	offset = sec_sysup_get_firmware_offset(name);
	if (offset < 0)
		return offset;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	from = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(from)) {
		pr_err("%s: %s file open error\n", __func__, path);
		fsize = -ENOENT;
		goto fp_err_2;
	}

	fsize = i_size_read(file_inode(from));
	buf = kzalloc(fsize, GFP_KERNEL);
	if (!buf) {
		pr_err("%s: %s buf alloc error\n", __func__, path);
		fsize = -ENOENT;
		goto fp_err_1;
	}

	vfs_read(from, buf, fsize, &from->f_pos);
	if (spu_firmware_signature_verify(name, buf, fsize) < 0) {
		pr_err("%s: %s %s spu_firmware_signature_verify error\n", __func__, name, path);
		fsize = -EINVAL;
		goto fp_err_1;
	}
	vfs_llseek(from, 0, SEEK_SET);

	to = filp_open(SEC_SYSUP_SPU_PATH, O_RDWR | O_SYNC, 0);
	if (IS_ERR(to)) {
		pr_err("%s: %s file open error\n", __func__, SEC_SYSUP_SPU_PATH);
		fsize = -ENOENT;
		goto fp_err_1;
	}

	vfs_llseek(to, offset, SEEK_SET);
	vfs_write(to, buf, fsize, &to->f_pos);

	filp_close(to, NULL);
fp_err_1:
	filp_close(from, NULL);
fp_err_2:
	set_fs(old_fs);

	pr_err("%s: name: %s, path: %s, fsize: %lld\n", __func__, name, path, fsize);

	return fsize;
}

static ssize_t sec_tsp_firmware_update_store(struct kobject *kobj,
					   struct kobj_attribute *attr,
					   const char *buf, size_t size)
{
	char fw_path[256] = {0, };
	int len = 0;

	len = strlen(buf) + 1;
	if (len > 256)
		return -EINVAL;

	snprintf(fw_path, len, "%s", buf);
	pr_err("%s: fw_path: %s\n", __func__, fw_path);

	return sec_firmware_update("TSP", fw_path);
}

static ssize_t sec_sensorhub_firmware_update_store(struct kobject *kobj,
					   struct kobj_attribute *attr,
					   const char *buf, size_t size)
{
	char fw_path[256] = {0, };
	int len = 0;

	len = strlen(buf) + 1;
	if (len > 256)
		return -EINVAL;

	snprintf(fw_path, len, "%s", buf);
	pr_err("%s: fw_path: %s\n", __func__, fw_path);

	return sec_firmware_update("SENSORHUB", fw_path);
}
#endif

static ssize_t sec_edtbo_update_store(struct kobject *kobj,
					   struct kobj_attribute *attr,
					   const char *buf, size_t size)
{
	struct fiemap *pfiemap;
	struct fiemap_extent_info fieinfo = { 0, };
	struct file *fp;
	mm_segment_t old_fs;
	struct inode *inode;
	u64 len;
	int update, error;

	if (!edtbo_offset)
		return -EFAULT;

	error = sscanf(buf, "%d", &update);
	if (error < 0 || update != 1)
		return -EINVAL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(SEC_EDTBO_FILENAME, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		pr_err("%s: file open error\n", __func__);
		error = -ENOENT;
		goto fp_err;
	}

	inode = file_inode(fp);
	if (!inode->i_op->fiemap) {
		error = -EOPNOTSUPP;
		goto open_err;
	}

	pfiemap = kmalloc(SEC_PARAM_EXTRA_MAX, GFP_KERNEL | __GFP_ZERO);
	if (!pfiemap) {
		error = -ENOMEM;
		goto open_err;
	}

	pfiemap->fm_length = ULONG_MAX;
	pfiemap->fm_extent_count = (SEC_PARAM_EXTRA_MAX - offsetof(struct fiemap, fm_extents)) / sizeof(struct fiemap_extent);

	error = fiemap_check_ranges(inode->i_sb, pfiemap->fm_start, pfiemap->fm_length, &len);
	if (error)
		goto alloc_err;

	fieinfo.fi_flags = pfiemap->fm_flags;
	fieinfo.fi_extents_max = pfiemap->fm_extent_count;
	fieinfo.fi_extents_start = pfiemap->fm_extents;

	error = inode->i_op->fiemap(inode, &fieinfo, pfiemap->fm_start, len);
	if (error)
		goto alloc_err;

	pfiemap->fm_flags = fieinfo.fi_flags;
	pfiemap->fm_mapped_extents = fieinfo.fi_extents_mapped;

	if (!pfiemap->fm_mapped_extents) {
		error = -EFAULT;
		goto alloc_err;
	}

	sec_set_param_extra(edtbo_offset, pfiemap, SEC_PARAM_EXTRA_MAX);

	error = size;

alloc_err:
	if (pfiemap)
		kfree(pfiemap);
open_err:
	filp_close(fp, NULL);
fp_err:
	set_fs(old_fs);

	return error;
}

static ssize_t sec_edtbo_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	pr_info("%s: Edtbo Version = %lu\n", __func__, edtbo_ver);

	return sprintf(buf, "%lu\n", edtbo_ver);
}

#if defined(CONFIG_TIZEN)
static struct kobj_attribute sec_tsp_sysup_firmware_update_attr =
	__ATTR(tsp_firmware_update, 0220, NULL, sec_tsp_firmware_update_store);
static struct kobj_attribute sec_sensorhub_sysup_firmware_update_attr =
	__ATTR(sensorhub_firmware_update, 0220, NULL, sec_sensorhub_firmware_update_store);
#endif
static struct kobj_attribute sec_sysup_edtbo_update_attr =
	__ATTR(edtbo_update, 0220, NULL, sec_edtbo_update_store);

static struct kobj_attribute sec_sysup_edtbo_version_attr =
	__ATTR(edtbo_version, 0440, sec_edtbo_version_show, NULL);

static struct attribute *sec_sysup_attributes[] = {
#if defined(CONFIG_TIZEN)
	&sec_tsp_sysup_firmware_update_attr.attr,
	&sec_sensorhub_sysup_firmware_update_attr.attr,
#endif
	&sec_sysup_edtbo_update_attr.attr,
	&sec_sysup_edtbo_version_attr.attr,
	NULL,
};

static struct attribute_group sec_sysup_attr_group = {
	.attrs = sec_sysup_attributes,
};

/* reboot notifier call */
static int sec_sysup_reboot_notifier(struct notifier_block *this,
				unsigned long event, void *_cmd)
{
	if (event == SYS_RESTART && _cmd) {
		size_t cmdlen = strlen(_cmd);
		if (cmdlen >= 5 && cmdlen < SEC_PARAM_STR_MAX && !strncmp(_cmd, "param", 5))
			sec_set_param_str(param_offset, _cmd, cmdlen);
	}

	return NOTIFY_OK;
}

static struct notifier_block sec_sysup_reboot_nb = {
	.notifier_call = sec_sysup_reboot_notifier,
	.priority = INT_MAX,
};

static int __init sec_sysup_init(void)
{
	int ret = 0;
	struct device *dev;

	pr_info("%s: start\n", __func__);

	dev = sec_device_create(NULL, "sec_sysup");
	if(!dev)
		pr_err("%s : sec device create failed!\n", __func__);

	ret = sysfs_create_group(&dev->kobj, &sec_sysup_attr_group);
	if (ret)
		pr_err("%s : could not create sysfs node\n", __func__);

	register_reboot_notifier(&sec_sysup_reboot_nb);

	return 0;
}

static void __exit sec_sysup_exit(void)
{
	pr_info("%s: exit\n", __func__);
}

module_init(sec_sysup_init);
module_exit(sec_sysup_exit);
