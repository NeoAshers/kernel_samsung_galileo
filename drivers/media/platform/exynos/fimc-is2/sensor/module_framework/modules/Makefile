obj-$(CONFIG_USE_DIRECT_IS_CONTROL) += fimc-is-device-module-base.o

obj-$(CONFIG_CAMERA_CIS_6B2_OBJ) += fimc-is-device-module-6b2.o
obj-$(CONFIG_CAMERA_CIS_5E2_OBJ) += fimc-is-device-module-5e2.o
obj-$(CONFIG_CAMERA_CIS_5E3_OBJ) += fimc-is-device-module-5e3.o
obj-$(CONFIG_CAMERA_CIS_4H5YC_OBJ) += fimc-is-device-module-4h5yc.o
obj-$(CONFIG_CAMERA_CIS_4H5_OBJ) += fimc-is-device-module-4h5.o
obj-$(CONFIG_CAMERA_CIS_2P2_OBJ) += fimc-is-device-module-2p2.o
obj-$(CONFIG_CAMERA_CIS_2P8_OBJ) += fimc-is-device-module-2p8.o
obj-$(CONFIG_CAMERA_CIS_3P3_OBJ) += fimc-is-device-module-3p3.o
obj-$(CONFIG_CAMERA_CIS_3L2_OBJ) += fimc-is-device-module-3l2.o
obj-$(CONFIG_CAMERA_CIS_3H1_OBJ) += fimc-is-device-module-3h1.o
obj-$(CONFIG_CAMERA_CIS_3H1_C3_OBJ) += fimc-is-device-module-3h1-c3.o
obj-$(CONFIG_CAMERA_CIS_4E6_OBJ) += fimc-is-device-module-4e6.o
obj-$(CONFIG_CAMERA_CIS_4E6_C2_OBJ) += fimc-is-device-module-4e6-c2.o
obj-$(CONFIG_CAMERA_CIS_3M2_OBJ) += fimc-is-device-module-3m2.o
obj-$(CONFIG_CAMERA_CIS_3M3_OBJ) += fimc-is-device-module-3m3.o
obj-$(CONFIG_CAMERA_CIS_2L1_OBJ) += fimc-is-device-module-2l1.o
obj-$(CONFIG_CAMERA_CIS_3P8_OBJ) += fimc-is-device-module-3p8.o
obj-$(CONFIG_CAMERA_CIS_3P8SP_OBJ) += fimc-is-device-module-3p8sp.o
obj-$(CONFIG_CAMERA_CIS_2P7SX_OBJ) += fimc-is-device-module-2p7sx.o
obj-$(CONFIG_CAMERA_CIS_2L2_OBJ) += fimc-is-device-module-2l2.o
obj-$(CONFIG_CAMERA_CIS_2L3_OBJ) += fimc-is-device-module-2l3.o
obj-$(CONFIG_CAMERA_CIS_IMX260_OBJ) += fimc-is-device-module-imx260.o
obj-$(CONFIG_CAMERA_CIS_2L7_OBJ) += fimc-is-device-module-2l7.o
obj-$(CONFIG_CAMERA_CIS_IMX320_OBJ) += fimc-is-device-module-imx320.o
obj-$(CONFIG_CAMERA_CIS_IMX320_C3_OBJ) += fimc-is-device-module-imx320-c3.o
obj-$(CONFIG_CAMERA_CIS_IMX333_OBJ) += fimc-is-device-module-imx333.o
obj-$(CONFIG_CAMERA_CIS_5E6_OBJ) += fimc-is-device-module-5e6.o
obj-$(CONFIG_CAMERA_CIS_5F1_OBJ) += fimc-is-device-module-5f1.o
obj-$(CONFIG_CAMERA_CIS_IMX219_OBJ) += fimc-is-device-module-imx219.o
obj-$(CONFIG_CAMERA_CIS_IMX258_OBJ) += fimc-is-device-module-imx258.o
obj-$(CONFIG_CAMERA_CIS_SR259_OBJ) += fimc-is-device-module-sr259.o
obj-$(CONFIG_CAMERA_CIS_IMX241_OBJ) += fimc-is-device-module-imx241.o
obj-$(CONFIG_CAMERA_CIS_VIRTUAL_OBJ) += fimc-is-device-module-virtual.o

EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-is2
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-is2/include
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-is2/vendor
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-is2/sensor/module_framework
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-is2/sensor
ifeq ($(CONFIG_COMPANION_DIRECT_USE),y)
EXTRA_CFLAGS += -Idrivers/media/platform/exynos/fimc-is2/vendor/mcd/preprocessor
endif
