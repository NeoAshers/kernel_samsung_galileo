obj-$(CONFIG_B43)		+= b43/
obj-$(CONFIG_B43LEGACY)		+= b43legacy/

obj-$(CONFIG_BRCMFMAC)	+= brcm80211/
obj-$(CONFIG_BRCMSMAC)	+= brcm80211/

ifeq ($(CONFIG_RENAISSANCE),y)
obj-$(CONFIG_BCM43012)  += bcmdhd_101_10/
obj-$(CONFIG_BCM43012)  += bcmdhd_101_10/dhd_custom_exynos.o
obj-$(CONFIG_BCM43012)  += bcmdhd_101_10/dhd_custom_memprealloc.o
else
ifeq ($(CONFIG_PULSE),y)
obj-$(CONFIG_BCM43012)  += bcmdhd_101_10/
obj-$(CONFIG_BCM43012)  += bcmdhd_101_10/dhd_custom_exynos.o
obj-$(CONFIG_BCM43012)  += bcmdhd_101_10/dhd_custom_memprealloc.o
else
obj-$(CONFIG_BCM43012)  += bcmdhd_1_77/
obj-$(CONFIG_BCM43012)  += bcmdhd_1_77/dhd_custom_exynos.o
obj-$(CONFIG_BCM43012)  += bcmdhd_1_77/dhd_custom_memprealloc.o
endif
endif

obj-$(CONFIG_BCM43013)  += bcmdhd_101_10/
obj-$(CONFIG_BCM43013)  += bcmdhd_101_10/dhd_custom_exynos.o
obj-$(CONFIG_BCM43013)  += bcmdhd_101_10/dhd_custom_memprealloc.o
