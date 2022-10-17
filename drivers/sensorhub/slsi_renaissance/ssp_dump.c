/*
 *  Copyright (C) 2018, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/time.h>

#include "ssp.h"
#include "ssp_platform.h"
#include "ssp_dump.h"

#define SENSORHUB_DUMP_DIR				"/opt/usr/data/gnss"

#define SENSORHUB_DUMP_MAX				5

void write_ssp_dump_file(struct ssp_data *data, char *info, void *buf, int size)
{
	char file_path[64] = {0,};
	char info_contents[100] = {0,};
	char info_path[64] = {0,};
	struct file *filp;
	mm_segment_t old_fs;
	int ret, info_len;
	struct timeval cur_time;
	struct tm t;

	do_gettimeofday(&cur_time);
	time64_to_tm(cur_time.tv_sec, 0, &t);

	/* info file */
	//info_len = snprintf(info_contents, sizeof(info_contents), "%s-%02u", info, data->reset_type);
	info_len = snprintf(info_contents, sizeof(info_contents), "%s-%02u[%d-%d %d:%d:%d] Sensor:0x%llx, Lib: 0x%llx, RC:%u, CC:%u, DC:%u, TC:%u\n",
			info, data->reset_type, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec,
			data->uSensorState, data->uLibraryState, data->uResetCnt, data->uComFailCnt, data->uDumpCnt, data->uTimeOutCnt);

	ssp_infof("%s(%d)", info_contents, info_len);

	/* dump file */
	snprintf(file_path, sizeof(file_path), "%s/sensorhub-%d.dump", SENSORHUB_DUMP_DIR, data->dump_index);
	ssp_infof("%s", file_path);
	snprintf(info_path, sizeof(info_path), "%s/sensorhub-info-%d.txt", SENSORHUB_DUMP_DIR, data->dump_index);

	old_fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(file_path, O_CREAT | O_WRONLY, 0666);
	if (IS_ERR(filp)) {
		ssp_errf("open '%s' file fail: %d\n", file_path, (int)PTR_ERR(filp));
		goto out;
	}
	ret = vfs_write(filp, buf, size, &filp->f_pos);
	if (ret < 0) {
		ssp_errf("Can't write dump to file (%d)", ret);
		filp_close(filp, NULL);
		goto out;
	}
	filp_close(filp, NULL);

	filp = filp_open(info_path, O_CREAT | O_WRONLY, 0666);
	if (IS_ERR(filp)) {
		ssp_errf("open '%s' file fail: %d\n", info_path, (int)PTR_ERR(filp));
		goto out;
	}
	ret = vfs_write(filp, info_contents, info_len, &filp->f_pos);
	if (ret < 0) {
		ssp_errf("can't write dump info to file (%d)", ret);
	}
	filp_close(filp, NULL);

	data->dump_index = (data->dump_index+1) % SENSORHUB_DUMP_MAX;

out:
	set_fs(old_fs);
}
