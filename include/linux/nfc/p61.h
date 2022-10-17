/*
 * Copyright (C) 2012-2014 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifdef CONFIG_NFC_FEATURE_SN100U
#define DEFAULT_BUFFER_SIZE 780
#else
#define DEFAULT_BUFFER_SIZE 258
#endif

#define P61_MAGIC 0xEB

#if !defined(CONFIG_NFC_FEATURE_SN100U)
#define P61_SET_PWR _IOW(P61_MAGIC, 0x01, unsigned int)
#define P61_SET_DBG _IOW(P61_MAGIC, 0x02, unsigned int)
#define P61_SET_POLL _IOW(P61_MAGIC, 0x03, unsigned int)

/* To set SPI configurations like gpio, clks */
#define P61_SET_SPI_CONFIG _IO(P61_MAGIC, 0x04)
/* Set the baud rate of SPI master clock nonTZ */
#define P61_ENABLE_SPI_CLK _IO(P61_MAGIC, 0x05)
/* To disable spi core clock */
#define P61_DISABLE_SPI_CLK _IO(P61_MAGIC, 0x06)
/* only nonTZ +++++*/
/* Transmit data to the device and retrieve data from it simultaneously.*/
#define P61_RW_SPI_DATA _IOWR(P61_MAGIC, 0x07, unsigned int)
/* only nonTZ -----*/


/*
 * SPI Request NFCC to enable p61 power, only in param
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 */
#define P61_SET_SPM_PWR    _IOW(P61_MAGIC, 0x08, unsigned int)

/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 *
*/
#define P61_GET_SPM_STATUS    _IOR(P61_MAGIC, 0x09, unsigned int)
#define P61_GET_ESE_ACCESS    _IOW(P61_MAGIC, 0x0A, unsigned int)
#define P61_SET_DWNLD_STATUS    _IOW(P61_MAGIC, 0x0B, unsigned int)

#else //#if !defined(CONFIG_NFC_FEATURE_SN100U)

#define P61_SET_PWR				_IOW(P61_MAGIC, 0x01, int)
#define P61_SET_DBG				_IOW(P61_MAGIC, 0x02, int)
#define P61_SET_POLL			_IOW(P61_MAGIC, 0x03, int)

/* SPI Request NFCC to enable p61 power, only in param
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 */
#define P61_SET_SPM_PWR			_IOW(P61_MAGIC, 0x04, int)

/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 */
#define P61_GET_SPM_STATUS		_IOR(P61_MAGIC, 0x05, int)
#define P61_SET_THROUGHPUT		_IOW(P61_MAGIC, 0x06, int)
#define P61_GET_ESE_ACCESS		_IOW(P61_MAGIC, 0x07, int)
#define P61_SET_POWER_SCHEME	_IOW(P61_MAGIC, 0x08, int)
#define P61_SET_DWNLD_STATUS	_IOW(P61_MAGIC, 0x09, int)
#define P61_INHIBIT_PWR_CNTRL	_IOW(P61_MAGIC, 0x0A, int)
/* SPI can call this IOCTL to perform the eSE COLD_RESET
 * via NFC driver.
  */
#define ESE_PERFORM_COLD_RESET  _IOW(P61_MAGIC, 0x0C, int)

/* only nonTZ +++++*/
/* Transmit data to the device and retrieve data from it simultaneously.*/
#define P61_RW_SPI_DATA			_IOWR(P61_MAGIC, 0x0F, unsigned int)
/* only nonTZ -----*/
#endif //#if !defined(CONFIG_NFC_FEATURE_SN100U)


struct p61_ioctl_transfer {
    unsigned char *rx_buffer;
    unsigned char *tx_buffer;
    unsigned int len;
};

struct p61_spi_platform_data {
    unsigned int irq_gpio;
    unsigned int rst_gpio;
};

#ifdef CONFIG_NFC_FEATURE_SN100U
#define ESE_PINCTRL
#endif
#ifdef ESE_PINCTRL
enum ESE_SPI_PINCTRL {
	ESE_SPI_SLEEP,
	ESE_SPI_ACTIVE,
#ifdef CONFIG_NFC_FEATURE_SN100U
	ESE_SPI_LPM,
#endif
	ESE_SPI_PCTRL_CNT,
};
#endif
