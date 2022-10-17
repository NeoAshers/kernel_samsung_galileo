#
# Makefile for nanohub
#

obj-$(CONFIG_CONTEXTHUB) += nanohub.o
nanohub-$(CONFIG_CHRE_SENSORHUB_HAL) := main.o comms.o
nanohub-$(CONFIG_NANOHUB_SPI) += spi.o bl.o
nanohub-$(CONFIG_NANOHUB_I2C) += i2c.o bl.o
nanohub-$(CONFIG_CONTEXTHUB_IPC) += chub.o chub_ipc.o chub_log.o chub_dbg.o
