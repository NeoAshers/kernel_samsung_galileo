#
# Copyright (c) 2012 Samsung Electronics Co., Ltd.
#		http://www.samsung.com
#
# Licensed under GPLv2
#

ccflags-y := -Idrivers/gpu/drm/tgm

scaler-objs := scaler-core.o scaler-regs.o scaler-debug.o
obj-$(CONFIG_VIDEO_EXYNOS_SCALER)	+= scaler.o

ccflags-$(CONFIG_SCALER_NO_SOFTRST) += -DSC_NO_SOFTRST
