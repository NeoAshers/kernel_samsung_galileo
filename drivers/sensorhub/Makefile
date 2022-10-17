#
# Makefile for the sensor hub.
#

ifeq ($(CONFIG_SENSORS_SSP_BBD),y)
obj-$(CONFIG_GALILEO)  += brcm/
obj-$(CONFIG_PULSE)  += brcm_pulse/
obj-$(CONFIG_NOBLESSE)  += brcm_noblesse/
endif
ifeq ($(CONFIG_SENSORS_SSP_CHUB),y)
obj-$(CONFIG_RENAISSANCE_CHUB)  += slsi_renaissance/
obj-$(CONFIG_NOBLESSE_CHUB)  += slsi_noblesse/
endif