#
# Makefile for the input misc lps22hb driver.
#

lps22hb-core-y := lps22hb_core.o
lps22hb-core-$(CONFIG_INPUT_LPS22HB_I2C) += lps22hb_i2c.o
lps22hb-core-$(CONFIG_INPUT_LPS22HB_SPI) += lps22hb_spi.o

obj-$(CONFIG_INPUT_LPS22HB) += lps22hb-core.o
