# drivers/staging/samsung/Makefile
#
# Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
#		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

#if !defined (CONFIG_GALILEO) || !defined (CONFIG_PULSE)
# Samsung Battery
#obj-$(CONFIG_BATTERY_SAMSUNG)	+= sec_batt.o
#endif

obj-$(CONFIG_TIZEN)			+= sec_fota_bl.o
# Samsung Logging Feature
obj-$(CONFIG_SEC_EXT)		+= sec_log.o
# Samsung sec sysfs Feature
obj-$(CONFIG_SEC_SYSFS)		+= sec_sysfs.o
# Samsung Reset Feature
obj-$(CONFIG_SEC_REBOOT)	+= sec_reboot.o
# Samsung Debugging Feature
obj-$(CONFIG_SEC_DEBUG)		+= sec_debug.o sec_getlog.o sec_gaf.o
obj-$(CONFIG_SEC_DEBUG)		+= sec_debug_test.o
obj-$(CONFIG_SEC_DEBUG_EXTRA_INFO)	+= sec_debug_extra_info.o
obj-$(CONFIG_SEC_DEBUG_HW_PARAM)	+= sec_debug_hw_param.o
obj-$(CONFIG_SEC_DEBUG_RESET_REASON) += sec_debug_reset_reason.o
obj-$(CONFIG_SEC_DEBUG_LAST_KMSG)	+= sec_debug_last_kmsg.o

# Samsung Misc Feature
obj-$(CONFIG_SEC_MISC)         += sec_misc.o
# Samsung TN BSP Options
obj-$(CONFIG_ARGOS)		+= sec_argos.o
# Samsung System Update
obj-$(CONFIG_SEC_SYSUP)     += sec_sysup.o
obj-$(CONFIG_SEC_PARAM)		+= sec_param.o
# Samsung sec bsp Feature
obj-$(CONFIG_SEC_BSP)          += sec_bsp.o
obj-$(CONFIG_SEC_INITCALL_DEBUG)		+= sec_initcall_debug.o

obj-$(CONFIG_SEC_HOTPLUG)	+= sec_hotplug.o
obj-$(CONFIG_SEC_GSIM)		+= sec_gsim.o

# Samsung sec NAD Feature
obj-$(CONFIG_SEC_NAD)		+= sec_nad.o
