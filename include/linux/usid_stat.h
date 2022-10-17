/* include/linux/usid_stat.h
 *
 * Copyright (C) 2015 SAMSUNG, Inc.
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
 *******************************************************************************
 *                                  HISTORY                                    *
 *******************************************************************************
 * ver   who                                         what                      *
 * ---- -------------------------------------------- ------------------------- *
 * 1.0   Junho Jang <vincent.jang@samsung.com>       <2015>                    *
 *                                                   Initial Release           *
 * ---- -------------------------------------------- ------------------------- *
 * 1.1   Hunsup Jung <hunsup.jung@samsung.com>       <2017.05.16>              *
 *                                                   Remove unnecessary code   *
 * ---- -------------------------------------------- ------------------------- *
 * 1.2   Hunsup Jung <hunsup.jung@samsung.com>       <2017.06.12>              *
 *                                                   Just release version 1.2  *
 * ---- -------------------------------------------- ------------------------- *
 * 1.3   Junho Jang <vincent.jang@samsung.com>       <2018>                    *
 *                                                   refactoring           *
 * ---- -------------------------------------------- ------------------------- *
 * 1.4   Junho Jang <vincent.jang@samsung.com>       <2020>                    *
 *                                                   add per-usid usage statistics *
 * ---- -------------------------------------------- ------------------------- *
 */

#ifndef __USID_STAT_H
#define __USID_STAT_H

struct usid_stat_traffic {
	u32 usid;
	uid_t uid;
	u32 sid;
	char comm[TASK_COMM_LEN];
	unsigned int snd;
	unsigned int rcv;
	unsigned int snd_count;
	unsigned int rcv_count;
};

#ifdef CONFIG_USID_STAT
int usid_stat_tcp_snd(pid_t pid, int size);
int usid_stat_tcp_rcv(pid_t pid, int size);
void usid_stat_get_traffic_emon(int type,
	struct usid_stat_traffic *tcp_traffic, size_t n);
void usid_stat_get_traffic_batr(
	struct usid_stat_traffic *tcp_traffic, size_t n);
#else
static inline int usid_stat_tcp_snd(pid, size) { return 0; }
static inline int usid_stat_tcp_rcv(pid, size) { return 0; }
static inline void usid_stat_get_traffic_emon(int type,
	struct usid_stat_traffic *tcp_traffic, size_t n) { return; };
static inline void usid_stat_get_traffic_batr(
	struct usid_stat_traffic *tcp_traffic, size_t n) { return; };
#endif

#endif /* __USID_STAT_H */
