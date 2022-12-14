/*
 * Copyright (C) 2010 Trusted Logic S.A.
 * modifications copyright (C) 2015 NXP B.V.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#define SEC_NFC_DRIVER_NAME "sec-nfc"
#define PN5XX_MAGIC    0xE9
/*
 * PN5XX power control via ioctl
 * PN5XX_SET_PWR(0): power off
 * PN5XX_SET_PWR(1): power on
 * PN5XX_SET_PWR(2): reset and power on with firmware download enabled
 */
#define PWR_OFF 0
#define PWR_ON  1
#define PWR_FW  2
#define CLK_OFF 0
#define CLK_ON  1
#define PN5XX_SET_PWR    _IOW(PN5XX_MAGIC, 0x01, unsigned int)
/*
 * SPI Request NFCC to enable p61 power, only in param
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 */
#define P61_SET_SPI_PWR    _IOW(PN5XX_MAGIC, 0x02, unsigned int)
/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 *
*/
#define P61_GET_PWR_STATUS    _IOR(PN5XX_MAGIC, 0x03, unsigned int)
/* DWP side this ioctl will be called
 * level 1 = Wired access is enabled/ongoing
 * level 0 = Wired access is disalbed/stopped
*/
#define P61_SET_WIRED_ACCESS _IOW(PN5XX_MAGIC, 0x04, unsigned int)
/*
  NFC Init will call the ioctl to register the PID with the i2c driver
*/
#define PN5XX_SET_NFC_SERVICE_PID _IOW(PN5XX_MAGIC, 0x05, unsigned int)
/*
  NFC and SPI will call the ioctl to update the power scheme
*/
#define PN5XX_SET_POWER_SCHEME _IOW(PN5XX_MAGIC, 0x07, unsigned int)

/*
  SPI or DWP can call this ioctl to get the current * power state of P61
*/
#define PN5XX_SET_DWNLD_STATUS    _IOW(PN5XX_MAGIC, 0x09, unsigned int)
/*
  NFC will call the ioctl to release the dwp on/off protection
*/
#define PN5XX_REL_DWPONOFF_WAIT _IOW(PN5XX_MAGIC, 0x0A, unsigned int)
/*
  NFC and SPI will call the ioctl to get the i2c/spi bus access
*/
#define PN5XX_GET_ESE_ACCESS _IOW(PN5XX_MAGIC, 0x06, unsigned int)
#define PN5XX_REL_SVDD_WAIT _IOW(PN5XX_MAGIC, 0x08, unsigned int)

#ifdef CONFIG_NFC_FEATURE_SN100U
#define P61_UPDATE_WIRED_ACCESS_STATE _IOW(PN5XX_MAGIC, 0x0B, unsigned int)

#define NFC_I2C_LDO_ON  1
/*
  NFC_ON: Driver is being used by the NFC service
*/
#define PN5XX_STATE_NFC_ON         0x01
/*
  FW_DNLD: NFC_ON and FW download is going on
*/
#define PN5XX_STATE_FW_DNLD        0x02
/*
  FW_DNLD: NFC_ON and FW download is going on
*/
#define PN5XX_STATE_ESE_COLD_RESET_FROM_DRIVER        0x04

#endif

typedef enum p61_access_state{
    P61_STATE_INVALID = 0x0000,
    P61_STATE_IDLE = 0x0100, /* p61 is free to use */
    P61_STATE_WIRED = 0x0200,  /* p61 is being accessed by DWP (NFCC)*/
    P61_STATE_SPI = 0x0400, /* P61 is being accessed by SPI */
    P61_STATE_DWNLD = 0x0800, /* NFCC fw download is in progress */
    P61_STATE_SPI_PRIO = 0x1000, /*Start of p61 access by SPI on priority*/
    P61_STATE_SPI_PRIO_END = 0x2000,/*End of p61 access by SPI on priority*/
    P61_STATE_SPI_END = 0x4000,
    P61_STATE_JCP_DWNLD = 0x8000,	/* JCOP downlad in progress */
    P61_STATE_SPI_SVDD_SYNC_START = 0x0001, /*ESE_VDD Low req by SPI*/
    P61_STATE_SPI_SVDD_SYNC_END = 0x0002, /*ESE_VDD is Low by SPI*/
    P61_STATE_DWP_SVDD_SYNC_START = 0x0004, /*ESE_VDD  Low req by Nfc*/
    P61_STATE_DWP_SVDD_SYNC_END = 0x0008 /*ESE_VDD is Low by Nfc*/
}p61_access_state_t;

struct pn5xx_i2c_platform_data {
    unsigned int irq_gpio;
    unsigned int ven_gpio;
    unsigned int firm_gpio;
#ifndef CONFIG_NFC_FEATURE_SN100U
    unsigned int ese_pwr_gpio; /* gpio to give power to p61, only TEE should use this */
#endif
#ifdef CONFIG_NFC_FEATURE_SN100U
    unsigned int iso_rst_gpio;
#endif
    unsigned int clkreq_gpio;
    struct regulator *pvdd_reg;
    struct regulator *vbat_reg;
    struct regulator *pmuvcc_reg;
    struct regulator *sevdd_reg;
#ifdef CONFIG_NFC_FEATURE_SN100U
	struct msm_xo_voter *nfc_clock;
	struct work_struct work_nfc_clock;
	struct workqueue_struct *wq_clock;
	bool clock_state;
#endif
};

typedef enum chip_type_pwr_scheme{
    PN67T_PWR_SCHEME = 0x01,
    PN80T_LEGACY_PWR_SCHEME,
    PN80T_EXT_PMU_SCHEME,
}chip_pwr_scheme_t;

typedef enum jcop_dwnld_state{
    JCP_DWNLD_IDLE = P61_STATE_JCP_DWNLD,   /* jcop dwnld is ongoing*/
    JCP_DWNLD_INIT=0x8010,                         /* jcop dwonload init state*/
    JCP_DWNLD_START=0x8020,                        /* download started */
    JCP_SPI_DWNLD_COMPLETE=0x8040,                 /* jcop download complete in spi interface*/
    JCP_DWP_DWNLD_COMPLETE=0x8080,                 /* jcop download complete */
} jcop_dwnld_state_t;

extern int ese_spi_pinctrl(int enable);
#if defined(CONFIG_NFC_FEATURE_SN100U)
void pn5xx_register_ese_shutdown(void (*func)(void));
extern unsigned int lpcharge;
#endif


