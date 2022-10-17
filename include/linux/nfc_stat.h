/* include/linux/nfc_stat.h
 *
 * Copyright (C) 2020 SAMSUNG, Inc.
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

#ifndef __NFC_STAT_H
#define __NFC_STAT_H

#ifdef CONFIG_NFC_STAT
extern int nfc_stat_se_activate(pid_t pid);
extern int nfc_stat_se_deactivate(pid_t pid);
#else
static inline int nfc_stat_se_activate(pid_t pid) { return 0; }
static inline int nfc_stat_se_deactivate(pid_t pid) { return 0; }
#endif

#endif /* __NFC_STAT_H */
