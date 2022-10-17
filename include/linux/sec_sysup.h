/* sec_sysup.h
 *
 * Copyright (C) 2014 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef SEC_SYSUP_H
#define SEC_SYSUP_H

#define SEC_SYSUP_TSP_OFFSET				0
#define SEC_SYSUP_SENSORHUB_OFFSET			2097152

#define SEC_SYSUP_HEADER_SIZE				20

#define SEC_SYSUP_MAGIC						"TIZN"

#define SEC_SYSUP_SPU_PATH					CONFIG_SEC_SYSUP_SPU_PATH

extern bool sec_sysup_check_spu_firmware_valid(const char *name);
extern u32 sec_sysup_get_spu_firmware_version(const char *name);
extern int sec_sysup_get_firmware_offset(const char *name);
extern int sec_sysup_get_firmware_size(const char *name);

#endif /* SEC_SYSUP_H */
