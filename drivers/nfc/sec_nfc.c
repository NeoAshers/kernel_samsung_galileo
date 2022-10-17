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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "sec_nfc.h"
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/wakelock.h>
#include <linux/completion.h>
#ifdef CONFIG_SLEEP_MONITOR
#include <linux/power/sleep_monitor.h>
#endif
#include <linux/nfc_stat.h>

#define SIG_NFC 44
#define MAX_BUFFER_SIZE    512

#ifdef CONFIG_NFC_NXP_DEBUG
#define NFC_DEBUG           1
#else
#define NFC_DEBUG           0
#endif

#if !defined(CONFIG_NFC_FEATURE_SN100U)
#define FEATURE_PN80T
#else
#define FEATURE_SN100X
#define PM_MONITOR_WIRED_MODE
/* VEN is kept ON all the time if you define the macro VEN_ALWAYS_ON.
Used for SN100 usecases */
#define VEN_ALWAYS_ON
#endif

#define MODE_OFF    0
#define MODE_RUN    1
#define MODE_FW     2

/* Only pn548, pn547 and pn544 are supported */
//#define CHIP "pn544"
#define CHIP "sec-nfc"
#define DRIVER_CARD    "PN5xx NFC"
#define DRIVER_DESC    "NFC driver for PN5xx Family"

#ifndef CONFIG_OF
#define CONFIG_OF
#endif

#ifdef	CONFIG_NOBLESSE
extern unsigned int system_rev;
#define SEC_NFC_ESE_PWR_PIN_ERR 200
#endif


struct pn5xx_dev    {
    wait_queue_head_t    read_wq;
    struct mutex        read_mutex;
    struct i2c_client    *client;
    struct miscdevice   pn5xx_device;
    int        ven_gpio;
    int        firm_gpio;
    int        irq_gpio;
#ifdef CONFIG_NFC_FEATURE_SN100U
    int i2c_probe;
    int pvdd;
    struct regulator *nfc_pvdd;
    unsigned int iso_rst_gpio;
#endif
    int                 clkreq_gpio; // YS : Not used due to Xtal use
#ifdef FEATURE_PN80T
    struct regulator *pvdd_reg;
    struct regulator *vbat_reg;
    struct regulator *pmuvcc_reg;
    struct regulator *sevdd_reg;
#endif

#ifdef CONFIG_NFC_FEATURE_SN100U
    struct work_struct work_nfc_clock;
    struct workqueue_struct *wq_clock;
    bool clk_req_wake; // Need to discuss with NXP
    struct completion ese_comp;
    struct completion svdd_sync_comp;
    struct completion dwp_onoff_comp;
#endif

#ifdef FEATURE_PN80T
    unsigned int        ese_pwr_gpio;  /* gpio used by SPI to provide power to p61 via NFCC */
#endif
    struct mutex        p61_state_mutex; /* used to make p61_current_state flag secure */
    p61_access_state_t  p61_current_state; /* stores the current P61 state */
    bool                nfc_ven_enabled; /* stores the VEN pin state powered by Nfc */
    bool                spi_ven_enabled; /* stores the VEN pin state powered by Spi */
    atomic_t irq_enabled;
    atomic_t read_flag;
    bool cancel_read;
    struct wake_lock nfc_wake_lock;
    long                nfc_service_pid; /*used to signal the nfc the nfc service */
#ifdef FEATURE_SN100X
/* Bit value  Status           Remark
 * b0 : 1  -> NFC_ON           Driver Open should set the flag
 *      0     NFC_OFF          Driver release should reset this flag
 * b1 : 1  -> FWDNLD           If FWDNLD is going on.
 *      0     Normal operation
 * b2 : 1 -> ese_cold_reset sequence has been triggered from the SPI driver
 *      0 -> ese_cold_reset cmd has been written by the NFC HAL
 * bits b3 to b7 : Reserved for the future use.
 * NOTE: Driver probe function should reset b0-b2 flags.
 *       The value of b3-b7 flags is undetermined.
 **/
	uint8_t             state_flags;
	void (*ese_shutdown)(void);
#endif

};

static struct pn5xx_dev *pn5xx_dev;

#ifdef PM_MONITOR_WIRED_MODE
static p61_access_state_t g_wired_access_state = P61_STATE_INVALID;
#endif

#ifdef CONFIG_NFC_FEATURE_SN100U
static struct semaphore pn5xx_access_sema;
static struct completion ese_cold_reset_sema;
static int ese_cold_reset_status;

static struct semaphore dwp_onoff_release_sema;
static atomic_t s_Device_opened = ATOMIC_INIT(1);
#endif

static struct semaphore ese_access_sema;

#ifdef FEATURE_PN80T
static struct semaphore svdd_sync_onoff_sema;
static unsigned char dwp_onoff_wait;
static struct completion dwp_onoff_sema;
#endif

static unsigned char svdd_sync_wait;
static void release_ese_lock(p61_access_state_t  p61_current_state);
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout);
static unsigned char p61_trans_acc_on = 0;
static void p61_get_access_state(struct pn5xx_dev*, p61_access_state_t*);

#ifdef CONFIG_SLEEP_MONITOR
int nfc_is_flag;
EXPORT_SYMBOL(nfc_is_flag);
int nfc_sleep_monitor_cb(void *priv, unsigned int *raw_val,
  int check_level, int caller_type)
{
 int state = DEVICE_UNKNOWN;

 if (check_level == SLEEP_MONITOR_CHECK_SOFT) {
  if (nfc_is_flag)
   state = DEVICE_ON_ACTIVE1;
  else
   state = DEVICE_POWER_OFF;

 } else {
  /* TODO: HARD */
  state = DEVICE_UNKNOWN;
 }

 *raw_val = state;

 pr_debug("[NFC] nfc_is_flag[%d] state[%d]\n", nfc_is_flag, state);

 return state;
}

static struct sleep_monitor_ops nfc_sleep_monitor_ops = {
  .read_cb_func = nfc_sleep_monitor_cb,
 };
#endif

/**********************************************************
 * driver functions
 **********************************************************/
static ssize_t pn5xx_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn5xx_dev *pn5xx_dev = filp->private_data;
    char tmp[MAX_BUFFER_SIZE] = { 0, };
    int ret = 0;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    pr_debug("[NFC] %s : reading %zu bytes.\n", __func__, count);

    mutex_lock(&pn5xx_dev->read_mutex);

    if (!gpio_get_value(pn5xx_dev->irq_gpio)) {
        atomic_set(&pn5xx_dev->read_flag, 0);
        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            goto fail;
        }

#if NFC_DEBUG
    pr_info("[NFC] %s: wait_event_interruptible : in\n", __func__);
#endif
    if (!gpio_get_value(pn5xx_dev->irq_gpio))
        ret = wait_event_interruptible(pn5xx_dev->read_wq,
                atomic_read(&pn5xx_dev->read_flag));

#if NFC_DEBUG
        pr_info("[NFC] %s: h\n", __func__);
#endif

        if (pn5xx_dev->cancel_read) {
            pn5xx_dev->cancel_read = false;
            ret = -1;
            goto fail;
        }

        if (ret)
            goto fail;
    }

    /* Read data */
    ret = i2c_master_recv(pn5xx_dev->client, tmp, count);

#if NFC_DEBUG
    pr_info("[NFC] %s: pn5xx: i2c_master_recv\n", __func__);
#endif
#ifdef FEATURE_SN100X
    /* if the received response for COLD_RESET_COMMAND
     * Consume it in driver */
	if ((pn5xx_dev->state_flags & PN5XX_STATE_ESE_COLD_RESET_FROM_DRIVER) &&
			0x4F == tmp[0] && 0x1E == tmp[1]) {
		size_t rcount = (size_t)tmp[2];

        	/* Read data: No need to wait for the interrupt */
		ret = i2c_master_recv(pn5xx_dev->client, &tmp[3], rcount);
		if (ret == rcount) {
			pr_debug("[NFC] %s NxpNciR : len = 4 > %02X%02X%02X%02X\n", __func__, tmp[0],tmp[1],tmp[2],tmp[3]);
			ret = tmp[3];
		} else {
			pr_err("[NFC] %s : Failed to receive payload of the cold_rst_cmd\n",__func__);
			ret = -1;
		}

		if (pn5xx_dev->state_flags & PN5XX_STATE_NFC_ON) {
			usleep_range(5000, 6000);
			ese_cold_reset_status = ret;
			complete(&ese_cold_reset_sema);
			ret = 0x00;
		}

		pn5xx_dev->state_flags &= ~(PN5XX_STATE_ESE_COLD_RESET_FROM_DRIVER);
		mutex_unlock(&pn5xx_dev->read_mutex);
		pr_info("[NFC] %s : return status %d", __func__, ret);
		return ret;
	}
#endif

    mutex_unlock(&pn5xx_dev->read_mutex);
    if (ret < 0) {
        pr_err("[NFC] %s: i2c_master_recv returned %d\n", __func__, ret);
        return ret;
    }
    if (ret > count) {
        pr_err("[NFC] %s: received too many bytes from i2c (%d)\n",
            __func__, ret);
        return -EIO;
    }
    if (copy_to_user(buf, tmp, ret)) {
  	pr_err("[NFC] %s: failed to copy to user space\n", __func__);
        return -EFAULT;
    }
    return ret;

fail:
    mutex_unlock(&pn5xx_dev->read_mutex);
    return ret;
}

static ssize_t pn5xx_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
 struct pn5xx_dev *pn5xx_dev = filp->private_data;
 char tmp[MAX_BUFFER_SIZE] = {0, };
    int ret = 0, retry = 2;
#if NFC_DEBUG
    pr_info("[NFC] %s: pn5xx: + w\n", __func__);
#endif

    pn5xx_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    if (copy_from_user(tmp, buf, count)) {
        pr_err("[NFC] %s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    pr_debug("[NFC] %s : writing %zu bytes.\n", __func__, count);
    /* Write data */
    do {
            retry--;

            ret = i2c_master_send(pn5xx_dev->client, tmp, count);
            if (ret == count)
                break;
            usleep_range(6000, 10000); /* Retry, chip was in standby */
#if NFC_DEBUG
            pr_debug("[NFC] %s: retry = %d\n", __func__, retry);
#endif
    } while (retry);
#if NFC_DEBUG
    pr_info("[NFC] %s: pn5xx: - w\n", __func__);
#endif
    if (ret != count) {
        pr_err("[NFC] %s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }

    return ret;
}

#ifdef FEATURE_SN100X
static int get_pn5xx_lock(int timeout)
{
	unsigned long tempJ = msecs_to_jiffies(timeout);

	if (down_timeout(&pn5xx_access_sema, tempJ) != 0) {
		pr_err("[NFC] get_pn5xx_lock: timeout\n");
		return -EBUSY;
	}

#if NFC_DEBUG
	pr_info("[NFC] get_pn5xx_lock: return\n");
#endif
	return 0;
}

static void release_pn5xx_lock(void)
{
	up(&pn5xx_access_sema);
	return;
}

void pn5xx_register_ese_shutdown(void (*func)(void))
{
	if (!lpcharge && pn5xx_dev)
		pn5xx_dev->ese_shutdown = func;
}
#endif

/**********************************************************
 * Interrupt control and handler
 **********************************************************/

static irqreturn_t pn5xx_dev_irq_handler(int irq, void *dev_id)
{
    struct pn5xx_dev *pn5xx_dev = dev_id;

    if (!gpio_get_value(pn5xx_dev->irq_gpio)) {
#if NFC_DEBUG
        pr_err("[NFC] %s, irq_gpio = %d\n", __func__,
        gpio_get_value(pn5xx_dev->irq_gpio));
#endif
        return IRQ_HANDLED;
    }

    /* Wake up waiting readers */
    atomic_set(&pn5xx_dev->read_flag, 1);
    wake_up(&pn5xx_dev->read_wq);
#if NFC_DEBUG
    pr_info("[NFC] pn5xx : call\n");
#endif
    wake_lock_timeout(&pn5xx_dev->nfc_wake_lock, 2 * HZ);
    return IRQ_HANDLED;
}

/**********************************************************
 * private functions
 **********************************************************/
#ifdef FEATURE_SN100X
static void p61_update_access_state(struct pn5xx_dev *pn5xx_dev,
		enum p61_access_state current_state, bool set)
{
	if (current_state) {
		if (set) {
			if (pn5xx_dev->p61_current_state == P61_STATE_IDLE)
				pn5xx_dev->p61_current_state
						= P61_STATE_INVALID;
			pn5xx_dev->p61_current_state |= current_state;
		} else {
			pn5xx_dev->p61_current_state &= (unsigned int)(~current_state);
			if (!pn5xx_dev->p61_current_state)
				pn5xx_dev->p61_current_state = P61_STATE_IDLE;
		}
	}
	pr_info("[NFC] Exit current_state = 0x%x\n",
			pn5xx_dev->p61_current_state);
}
#elif defined (FEATURE_PN80T)
static void p61_update_access_state(struct pn5xx_dev *pn5xx_dev, p61_access_state_t current_state, bool set)
{
    //pr_err("[NFC] %s: Enter current_state = %x\n", __func__, pn5xx_dev->p61_current_state);
    if (current_state)
    {
        if(set){
            if(pn5xx_dev->p61_current_state == P61_STATE_IDLE)
            pn5xx_dev->p61_current_state = P61_STATE_INVALID;
            pn5xx_dev->p61_current_state |= current_state;
        }
        else{
            pn5xx_dev->p61_current_state ^= current_state;
            if(!pn5xx_dev->p61_current_state)
                pn5xx_dev->p61_current_state = P61_STATE_IDLE;
        }
    }
    //pr_err("[NFC] %s: Exit current_state = %x\n", __func__, pn5xx_dev->p61_current_state);
}
#endif

static void p61_get_access_state(struct pn5xx_dev *pn5xx_dev, p61_access_state_t *current_state)
{

    if (current_state == NULL) {
        pr_err("[NFC] %s : invalid state of p61_access_state_t current state  \n", __func__);
    } else {
        *current_state = pn5xx_dev->p61_current_state;
    }
}

static void p61_access_lock(struct pn5xx_dev *pn5xx_dev)
{
    //pr_err("[NFC] %s: Enter\n", __func__);
    mutex_lock(&pn5xx_dev->p61_state_mutex);
    //pr_info("[NFC] %s: Exit\n", __func__);
}

static void p61_access_unlock(struct pn5xx_dev *pn5xx_dev)
{
    //pr_info("[NFC] %s: Enter\n", __func__);
    mutex_unlock(&pn5xx_dev->p61_state_mutex);
    //pr_info("[NFC] %s: Exit\n", __func__);
}

#ifdef FEATURE_SN100X
long p61_cold_reset(void)
{
	long ret = 0;
	unsigned int loop=0x03;
	struct file filp;
	size_t retno = 0x00;
	int timeout = 2000; /* 2s timeout :NCI cmd timeout*/
	unsigned long tempJ = msecs_to_jiffies(timeout);
	uint8_t cmd_ese_cold_reset[] = {0x2F, 0x1E, 0x00};
	filp.private_data = pn5xx_dev;

	pr_debug("[NFC] %s: Enter", __func__);

	if (pn5xx_dev->state_flags & PN5XX_STATE_FW_DNLD) {
		/* If FW DNLD, Operation is not permitted */
		pr_err("[NFC] %s : Operation is not permitted during fwdnld\n", __func__);
		return -EPERM;
	}

	/* Lock the write command window mutex. This insures smooth handling of
	 *  --> NFC_ON called while Cold_reset_seq is being performed */
	if (get_pn5xx_lock(2000)) {// Max CMDWINDOW time of NFC_HAL
		pr_err("[NFC] %s:Timeout for write command window",__func__);
		return -EBUSY; /* Timer Expired */
	}

	/* pn5xx_dev_read() should return the rsp if JNI has requested the cold reset*/
	pn5xx_dev->state_flags |= PN5XX_STATE_ESE_COLD_RESET_FROM_DRIVER;

	init_completion(&ese_cold_reset_sema);

	/* write command to I2C line*/
	do {
		ret = i2c_master_send(pn5xx_dev->client, cmd_ese_cold_reset,
									sizeof(cmd_ese_cold_reset));
		if (ret == sizeof(cmd_ese_cold_reset)) {
			break;
		}

		loop--;
		usleep_range(5000, 6000);
	} while(loop);

	if (!loop && (ret != sizeof(cmd_ese_cold_reset))) {
		pr_err("[NFC] %s : i2c_master_send returned %ld\n", __func__, ret);
		/* Unlock the write command window mutex*/
		release_pn5xx_lock();
		return -EIO;
	}

	pr_info("[NFC] %s: NxpNciX: %ld > %02X%02X%02X \n", __func__, ret,
		cmd_ese_cold_reset[0], cmd_ese_cold_reset[1], cmd_ese_cold_reset[2]);

	if (pn5xx_dev->state_flags & PN5XX_STATE_NFC_ON) { /* NFC_ON */
		/* Read is pending from the NFC service which will complete the ese_cold_reset_sema */
		/* wait for the ese_cold_reset Semaphore */
		if(wait_for_completion_timeout(&ese_cold_reset_sema, tempJ) == 0)
			pr_err("[NFC] %s: Timeout", __func__);

		ret = ese_cold_reset_status;
	} else { /* NFC_OFF */
		/* call the pn5xx_dev_read() */
		retno = pn5xx_dev_read(&filp, NULL,3, 0);
		if (!retno)
			ret = retno; // Success case
		else
			ret = -EAGAIN; // Failure case
	}

	/* Unlock the write command window mutex*/
	release_pn5xx_lock();

	/* Return the status to the SPI Driver */
	pr_debug("[NFC] %s: exit, Status:%ld", __func__,ret);

	return ret;
}
EXPORT_SYMBOL(p61_cold_reset);
#endif

static int signal_handler(p61_access_state_t state, long nfc_pid)
{
    struct siginfo sinfo;
    pid_t pid;
    struct task_struct *task;
    int sigret = 0;
    int ret = 0;

#if NFC_DEBUG
    pr_info("[NFC] %s: Enter\n", __func__);
#endif

#ifdef FEATURE_SN100X
    pr_debug("[NFC] pid:%ld\n", nfc_pid);
    if (nfc_pid == 0) {
        pr_err("[NFC] nfc_pid is clear don't call.\n");
        return -EPERM;
    }
#endif

    memset(&sinfo, 0, sizeof(struct siginfo));
    sinfo.si_signo = SIG_NFC;
    sinfo.si_code = SI_QUEUE;
    sinfo.si_int = state;
    pid = nfc_pid;

    task = pid_task(find_vpid(pid), PIDTYPE_PID);
    if(task)
    {
        pr_info("[NFC] %s.\n", task->comm);
        sigret = send_sig_info(SIG_NFC, &sinfo, task);

        if(sigret < 0){
            pr_err("[NFC] send_sig_info failed..... sigret %d.\n", sigret);
            ret = -1;
        }
    }
    else
    {
        pr_err("[NFC] finding task from PID failed\r\n");
        ret = -1;
    }
#if NFC_DEBUG
    pr_info("[NFC] %s: Exit ret = %d\n", __func__, ret);
#endif
    return ret;
}

#ifdef FEATURE_SN100X
static void svdd_sync_onoff(long nfc_service_pid, enum p61_access_state origin)
{
	int timeout = 500; /*500 ms timeout*/
	unsigned long tempJ = msecs_to_jiffies(timeout);

#if NFC_DEBUG
	pr_info("[NFC] Enter %s nfc_service_pid: %ld\n", __func__, nfc_service_pid);
#endif
	if (nfc_service_pid) {
		if (signal_handler(origin, nfc_service_pid) == 0) {
			reinit_completion(&pn5xx_dev->svdd_sync_comp);
			svdd_sync_wait = 1;
			pr_info("[NFC] Waiting for svdd protection response");
			/*if (down_timeout(&svdd_sync_onoff_sema, tempJ) != 0)*/
			if (!wait_for_completion_timeout(&pn5xx_dev->svdd_sync_comp, tempJ))
				pr_err("[NFC] %s svdd wait protection: Timeout", __func__);

			pr_info("[NFC] svdd wait protection : released");
			svdd_sync_wait = 0;
		}
	}
#if NFC_DEBUG
	pr_info("[NFC] Exit %s\n", __func__);
#endif
}

static int release_svdd_wait(void)
{
	unsigned char i = 0;

#if NFC_DEBUG
	pr_info("[NFC] Enter %s\n", __func__);
#endif
	for (i = 0; i < 9; i++) {
		if (svdd_sync_wait) {
			complete(&pn5xx_dev->svdd_sync_comp);
			svdd_sync_wait = 0;
			break;
		}
		usleep_range(10000, 10100);
	}
#if NFC_DEBUG
	pr_info("[NFC] Exit %s\n", __func__);
#endif
	return 0;
}
#elif defined (FEATURE_PN80T)
static void svdd_sync_onoff(long nfc_service_pid, p61_access_state_t origin)
{
    int timeout = 100; //100 ms timeout
    unsigned long tempJ = msecs_to_jiffies(timeout);
    //pr_info("[NFC] %s: Enter nfc_service_pid: %ld\n", __func__, nfc_service_pid);

    if(nfc_service_pid)
    {
        if (0 == signal_handler(origin, nfc_service_pid))
        {
            svdd_sync_wait = 1;
            sema_init(&svdd_sync_onoff_sema, 0);
            pr_info("[NFC] Waiting for svdd protection response");
            if(down_timeout(&svdd_sync_onoff_sema, tempJ) != 0)
            {
                pr_err("[NFC] svdd wait protection: Timeout");
            }
            pr_info("[NFC] svdd wait protection : released");
            svdd_sync_wait = 0;
        }
    }
#if NFC_DEBUG
    pr_info("[NFC] %s: Exit\n", __func__);
#endif
}

static int release_svdd_wait(void)
{
#if NFC_DEBUG
    pr_info("[NFC] %s: Enter \n", __func__);
#endif

    if(svdd_sync_wait)
    {
        up(&svdd_sync_onoff_sema);
        svdd_sync_wait = 0;
    }

#if NFC_DEBUG
    pr_info("[NFC] %s: Exit\n", __func__);
#endif
    return 0;
}
#endif //#ifdef FEATURE_SN100X


#ifdef FEATURE_PN80T
static int pn5xx_enable(struct pn5xx_dev *dev, unsigned long arg)
{
    int r = 0;

    p61_access_state_t current_state;

    pr_info("[NFC] %s: nfc enable [%d]\n", __func__ , atomic_read(&dev->irq_enabled));
    p61_get_access_state(dev, &current_state);
    switch (arg) {
    case 1: /* power on */
        if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
            p61_update_access_state(dev, P61_STATE_IDLE, true);

        if (current_state & P61_STATE_DWNLD)
            p61_update_access_state(dev, P61_STATE_DWNLD, false);

        gpio_set_value(dev->firm_gpio, 0);

        dev->nfc_ven_enabled = true;
        if (dev->spi_ven_enabled == false) {
            gpio_set_value_cansleep(dev->ven_gpio, 1);
            usleep_range(4900, 5000);
        }

        if (atomic_read(&dev->irq_enabled) == 0) {
            atomic_set(&dev->irq_enabled, 1);
            enable_irq(dev->client->irq);

            r = enable_irq_wake(dev->client->irq);
            if(r <0)
            {
                pr_info("[NFC] %s : fail nfc enable_irq_wake!! [%d]\n" , __func__ , r);
            }
            else
            {
                pr_info("[NFC] %s : nfc enable_irq_wake!! [%d]\n" , __func__ , r);
            }
        }

        svdd_sync_wait = 0;
        dwp_onoff_wait = 0;

        pr_info("[NFC] %s power on, irq=%d\n", __func__,
            atomic_read(&dev->irq_enabled));
        break;
    case 2:
        if (current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO))
        {
            /* NFCC fw/download should not be allowed if p61 is used by SPI */
            pr_info("[NFC] %s NFCC should not be allowed to reset/FW download \n", __func__);
            /*p61_access_unlock(pn5xx_dev); redundant */
            return -EBUSY; /* Device or resource busy */
        }
        dev->nfc_ven_enabled = true;
        if (dev->spi_ven_enabled == false)
        {
        /* power on with firmware download (requires hw reset) */
            pr_info("[NFC] %s power on with firmware\n", __func__);
            if (gpio_is_valid(dev->firm_gpio)) {
            p61_update_access_state(dev, P61_STATE_DWNLD, true);
            gpio_set_value_cansleep(dev->ven_gpio, 1);
            gpio_set_value(dev->firm_gpio, 1);
            }
            usleep_range(4900, 5000);
            gpio_set_value_cansleep(dev->ven_gpio, 0);
            usleep_range(4900, 5000);
            gpio_set_value_cansleep(dev->ven_gpio, 1);
            usleep_range(4900, 5000);
            if (atomic_read(&dev->irq_enabled) == 0) {
                atomic_set(&dev->irq_enabled, 1);
                enable_irq(dev->client->irq);
                enable_irq_wake(dev->client->irq);
            }
            pr_info("[NFC] %s power on with firmware, irq=%d\n", __func__,
                    atomic_read(&dev->irq_enabled));

            pr_info("[NFC] %s: VEN(%d) FIRM(%d)\n", __func__,
                    gpio_get_value(dev->ven_gpio), gpio_get_value(dev->firm_gpio));
        }
        break;
    default:
        pr_err("[NFC] %s bad arg %lu\n", __func__, arg);
        p61_access_unlock(dev);
        return -EINVAL;
    }

    /* turn on the regulators */
    /* -- if the regulators were specified, they're required */
    if(dev->pvdd_reg != NULL)
    {
        r = regulator_enable(dev->pvdd_reg);
        if (r < 0){
        pr_err("[NFC] %s: not able to enable pvdd\n", __func__);
        return r;
        }
    }
    if(dev->vbat_reg != NULL)
    {
        r = regulator_enable(dev->vbat_reg);
        if (r < 0){
            pr_err("[NFC] %s: not able to enable vbat\n", __func__);
            goto enable_exit0;
        }
    }
    if(dev->pmuvcc_reg != NULL)
    {
        r = regulator_enable(dev->pmuvcc_reg);
        if (r < 0){
            pr_err("[NFC] %s: not able to enable pmuvcc\n", __func__);
            goto enable_exit1;
        }
    }
    if(dev->sevdd_reg != NULL)
    {
        r = regulator_enable(dev->sevdd_reg);
        if (r < 0){
            pr_err("[NFC] %s: not able to enable sevdd\n", __func__);
            goto enable_exit2;
        }
    }
    return 0;

enable_exit2:
    if(dev->pmuvcc_reg) regulator_disable(dev->pmuvcc_reg);
enable_exit1:
    if(dev->vbat_reg) regulator_disable(dev->vbat_reg);
enable_exit0:
    if(dev->pvdd_reg) regulator_disable(dev->pvdd_reg);

    return r;
}

static void pn5xx_disable(struct pn5xx_dev *dev)
{
    int r;

    /* power off */
    //pr_info("[NFC] %s nfc off irq_enable[%d]\n", __func__ , atomic_read(&dev->irq_enabled));

    if (atomic_read(&dev->irq_enabled) == 1) {
        atomic_set(&dev->irq_enabled, 0);
        r = disable_irq_wake(dev->client->irq);
        if(r <0)
        {
            pr_info("[NFC] %s : fail nfc disable_irq_wake!! [%d]\n" , __func__ , r);
        }
        else
        {
            //pr_info("[NFC] %s : nfc disable_irq_wake!! [%d]\n" , __func__ , r);
            disable_irq_nosync(dev->client->irq);
        }
    }

    if (gpio_is_valid(dev->firm_gpio))
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(dev, &current_state);

        if(current_state & P61_STATE_DWNLD)
            p61_update_access_state(dev, P61_STATE_DWNLD, false);
        if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0) {
            p61_update_access_state(dev, P61_STATE_IDLE, true);
        }
        gpio_set_value(dev->firm_gpio, 0);
    }

    pr_info("[NFC] %s: power off, irq(%d)\n", __func__,
            atomic_read(&dev->irq_enabled));

    dev->nfc_ven_enabled = false;
    /* Don't change Ven state if spi made it high */
    if (dev->spi_ven_enabled == false) {
        gpio_set_value_cansleep(dev->ven_gpio, 0);
        usleep_range(4900, 5000);
    }

    if(dev->sevdd_reg) regulator_disable(dev->sevdd_reg);
    if(dev->pmuvcc_reg) regulator_disable(dev->pmuvcc_reg);
    if(dev->vbat_reg) regulator_disable(dev->vbat_reg);
    if(dev->pvdd_reg) regulator_disable(dev->pvdd_reg);

}
#endif //FEATURE_PN80T

#ifdef FEATURE_SN100X
static void dwp_onoff(long nfc_service_pid, enum p61_access_state origin)
{
	int timeout = 500; /*500 ms timeout*/
	unsigned long tempJ = msecs_to_jiffies(timeout);

	if (nfc_service_pid) {
		if (signal_handler(origin, nfc_service_pid) == 0) {
			reinit_completion(&pn5xx_dev->dwp_onoff_comp);
			if (!wait_for_completion_timeout(&pn5xx_dev->dwp_onoff_comp, tempJ))
				pr_err("[NFC] %s wait protection: Timeout\n", __func__);

			pr_info("[NFC] wait protection : released\n");
		}
	}
}
static int release_dwpOnOff_wait(void)
{
	int timeout = 500; //500 ms timeout
	unsigned long tempJ = msecs_to_jiffies(timeout);

#if NFC_DEBUG
	pr_info("[NFC] enter %s\n", __func__);
#endif
	complete(&pn5xx_dev->dwp_onoff_comp);
	{
		if(down_timeout(&dwp_onoff_release_sema, tempJ) != 0)
		{
			pr_err("[NFC] %s Dwp On/off release wait protection: Timeout", __func__);
		}
		pr_info("[NFC] Dwp On/Off release wait protection : released");
	}
	return 0;
}
#elif defined (FEATURE_PN80T)
static void dwp_onoff(long nfc_service_pid, p61_access_state_t origin)
{
    int timeout = 100; //100 ms timeout
    unsigned long tempJ = msecs_to_jiffies(timeout);
    if(nfc_service_pid)
    {
        if (0 == signal_handler(origin, nfc_service_pid))
        {
            init_completion(&dwp_onoff_sema);
            dwp_onoff_wait = 1;
            if(wait_for_completion_timeout(&dwp_onoff_sema, tempJ) != 0)
            {
                pr_info("[NFC] Dwp On/off wait protection: Timeout");
            }
            pr_info("[NFC] Dwp On/Off wait protection : released");
            dwp_onoff_wait = 0;
        }
    }
}
static int release_dwpOnOff_wait(void)
{
    pr_info("[NFC] %s: Enter \n", __func__);
    if (dwp_onoff_wait) {
    complete(&dwp_onoff_sema);
        dwp_onoff_wait = 0;
    }
    return 0;
}
#endif //#ifdef FEATURE_SN100X

static int set_nfc_pid(unsigned long arg)
{
    //pr_info("[NFC] %s : The NFC Service PID is %ld\n", __func__, arg);
    pn5xx_dev->nfc_service_pid = arg;
    return 0;
}

static int pn5xx_dev_open(struct inode *inode, struct file *filp)
{
    struct pn5xx_dev *pn5xx_dev = container_of(filp->private_data,
                                               struct pn5xx_dev,
                                               pn5xx_device);

    //printk("[NFC] pn5xx_dev_open!!!!!!!!!!!!!!!!!!!!!!!\n");
#ifdef	CONFIG_NOBLESSE
    if(system_rev < 5){
        pr_err("[NFC] Old HW(%d) with SN 110 Chip so exiting\n",system_rev);
        return -(SEC_NFC_ESE_PWR_PIN_ERR);
    }
#endif

    filp->private_data = pn5xx_dev;
#ifdef FEATURE_SN100X
	if (!atomic_dec_and_test(&s_Device_opened)) {
		atomic_inc(&s_Device_opened);
		pr_err("[NFC] already opened!\n");
		return -EBUSY;
	}

	pn5xx_dev->state_flags |= PN5XX_STATE_NFC_ON;
#endif

    pr_info("[NFC] %s : %d,%d\n", __func__, imajor(inode), iminor(inode));

    // pn5xx_enable(pn5xx_dev, MODE_RUN);

    return 0;
}

static int pn5xx_dev_release(struct inode *inode, struct file *filp)
{
    // struct pn5xx_dev *pn5xx_dev = container_of(filp->private_data,
    //                                           struct pn5xx_dev,
    //                                           pn5xx_device);
    p61_access_state_t current_state = P61_STATE_INVALID;
    pr_info("[NFC] %s : closing %d,%d\n", __func__, imajor(inode), iminor(inode));
    p61_get_access_state(pn5xx_dev, &current_state);
    if((p61_trans_acc_on ==  1) && ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0))
        release_ese_lock(P61_STATE_WIRED);
#ifdef FEATURE_SN100X
	pn5xx_dev->state_flags = 0x00;
        atomic_inc(&s_Device_opened);
#endif

    return 0;
}

static int set_jcop_download_state(unsigned long arg)
{
    p61_access_state_t current_state = P61_STATE_INVALID;
    int ret = 0;

    p61_get_access_state(pn5xx_dev, &current_state);
    pr_info("[NFC] %s: PN5XX_SET_DWNLD_STATUS:JCOP Dwnld state arg = %ld\n",
            __func__, arg);
    if (arg == JCP_DWNLD_INIT) {
        if (pn5xx_dev->nfc_service_pid) {
            pr_info("[NFC] %s nfc service pid ---- %ld\n",
                    __func__, pn5xx_dev->nfc_service_pid);
            signal_handler(JCP_DWNLD_INIT,
                    pn5xx_dev->nfc_service_pid);
        } else {
            if (current_state & P61_STATE_JCP_DWNLD)
                ret = -EINVAL;
            else
                p61_update_access_state(pn5xx_dev,
                        P61_STATE_JCP_DWNLD, true);
        }
    } else if (arg == JCP_DWNLD_START) {
        if (current_state & P61_STATE_JCP_DWNLD)
            ret = -EINVAL;
        else
            p61_update_access_state(pn5xx_dev,
                    P61_STATE_JCP_DWNLD, true);
    } else if (arg == JCP_SPI_DWNLD_COMPLETE) {
        if (pn5xx_dev->nfc_service_pid) {
            signal_handler(JCP_DWP_DWNLD_COMPLETE,
                    pn5xx_dev->nfc_service_pid);
        }
        p61_update_access_state(pn5xx_dev, P61_STATE_JCP_DWNLD, false);
    } else if (arg == JCP_DWP_DWNLD_COMPLETE) {
        p61_update_access_state(pn5xx_dev, P61_STATE_JCP_DWNLD, false);
    } else {
        pr_info("[NFC] %s bad jcop download arg %lu\n", __func__, arg);
        p61_access_unlock(pn5xx_dev);
        return -EBADRQC; /* Invalid request code */
    }
    pr_info("[NFC] %s: PN5XX_SET_DWNLD_STATUS = %x", __func__, current_state);

    return ret;
}

static int pn5xx_set_pwr(struct pn5xx_dev *pdev, unsigned long arg)
{
	int ret = 0;
	enum p61_access_state current_state;

	p61_get_access_state(pdev, &current_state);
#ifdef FEATURE_SN100X
	switch (arg) {
	case 0: /* power off */
		if (atomic_read(&pdev->irq_enabled) == 1) {
			atomic_set(&pdev->irq_enabled, 0);
			disable_irq_wake(pdev->client->irq);
			disable_irq_nosync(pdev->client->irq);
		}
		if (current_state & P61_STATE_DWNLD)
			p61_update_access_state(pdev, P61_STATE_DWNLD, false);

		if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI
			|P61_STATE_SPI_PRIO)) == 0)
			p61_update_access_state(pdev, P61_STATE_IDLE, true);

		pr_info("[NFC] power off, irq=%d\n", atomic_read(&pdev->irq_enabled));
		gpio_set_value(pdev->firm_gpio, 0);

		pdev->nfc_ven_enabled = false;
		/* Don't change Ven state if spi made it high */
#ifndef VEN_ALWAYS_ON
		if (pdev->spi_ven_enabled == false)
			gpio_set_value_cansleep(pdev->ven_gpio, 0);
#endif
		usleep_range(4900, 5000);
		break;
	case 1: /* power on */
		if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
			p61_update_access_state(pdev, P61_STATE_IDLE, true);

		if (current_state & P61_STATE_DWNLD)
			p61_update_access_state(pdev, P61_STATE_DWNLD, false);

		gpio_set_value(pdev->firm_gpio, 0);

		pdev->nfc_ven_enabled = true;
#ifndef VEN_ALWAYS_ON
		if (pdev->spi_ven_enabled == false)
			gpio_set_value_cansleep(pdev->ven_gpio, 1);
#endif
		usleep_range(4900, 5000);
		if (atomic_read(&pdev->irq_enabled) == 0) {
			atomic_set(&pdev->irq_enabled, 1);
			enable_irq(pdev->client->irq);
			enable_irq_wake(pdev->client->irq);
		}
		svdd_sync_wait = 0;

		pr_info("[NFC] power on, irq=%d\n", atomic_read(&pdev->irq_enabled));
		break;
	case 2:
		if (current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
			/* NFCC fw/download should not be allowed if p61 is used by SPI */
			pr_err("[NFC] not be allowed to reset/FW download\n");
			return -EBUSY; /* Device or resource busy */
		}
		pdev->nfc_ven_enabled = true;
		if (pdev->spi_ven_enabled == false) {
			/* power on with firmware download (requires hw reset) */
			p61_update_access_state(pdev, P61_STATE_DWNLD, true);
			gpio_set_value_cansleep(pdev->ven_gpio, 1);
			gpio_set_value(pdev->firm_gpio, 1);
			usleep_range(4900, 5000);
			gpio_set_value_cansleep(pdev->ven_gpio, 0);
			usleep_range(14900, 15000);
			gpio_set_value_cansleep(pdev->ven_gpio, 1);
			usleep_range(4900, 5000);
			if (atomic_read(&pdev->irq_enabled) == 0) {
				atomic_set(&pdev->irq_enabled, 1);
				enable_irq(pdev->client->irq);
				enable_irq_wake(pdev->client->irq);
			}
			pr_info("[NFC] power on with firmware, irq=%d\n",
				atomic_read(&pdev->irq_enabled));
			pr_info("[NFC] VEN=%d FIRM=%d\n", gpio_get_value(pdev->ven_gpio),
				gpio_get_value(pdev->firm_gpio));
		}
		break;
	case 3:
		/*NFC Service called ISO-RST*/
		if(current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
			return -EPERM; /* Operation not permitted */
		}
		if(current_state & P61_STATE_WIRED) {
			p61_update_access_state(pn5xx_dev, P61_STATE_WIRED, false);
		}
#ifdef ISO_RST
		gpio_set_value(pn5xx_dev->iso_rst_gpio, 0);
		msleep(50);
		gpio_set_value(pn5xx_dev->iso_rst_gpio, 1);
		msleep(50);
		pr_info("[NFC] %s ISO RESET from DWP DONE\n", __func__);
#endif
		break;
	case 4:
		pr_info("[NFC] %s FW dwldioctl called from NFC \n", __func__);
		/*NFC Service called FW dwnld*/
		if (pn5xx_dev->firm_gpio) {
			p61_update_access_state(pn5xx_dev, P61_STATE_DWNLD, true);
			gpio_set_value(pn5xx_dev->firm_gpio, 1);
			pn5xx_dev->state_flags |= PN5XX_STATE_FW_DNLD;
			msleep(10);
		}
		break;
	case 5:
		msleep(10);
		gpio_set_value(pn5xx_dev->ven_gpio, 0);
		msleep(15);
		gpio_set_value(pn5xx_dev->ven_gpio, 1);
		msleep(10);
		pr_info("[NFC] %s VEN reset DONE >>>>>>>\n", __func__);
		break;
	case 6:
		if (pn5xx_dev->firm_gpio) {
			gpio_set_value(pn5xx_dev->firm_gpio, 0);
			pn5xx_dev->state_flags &= ~(PN5XX_STATE_FW_DNLD);
			p61_update_access_state(pn5xx_dev, P61_STATE_DWNLD, false);
		}
		pr_info("[NFC] %s FW GPIO set to 0x00 >>>>>>>\n", __func__);
		break;
	case 7:
		get_pn5xx_lock(2000);
		break;
	case 8:
		release_pn5xx_lock();
		break;
	default:
		pr_err("[NFC] bad arg %lu\n", arg);
		/* changed the p61 state to idle*/
		ret = -EINVAL;
	}

#elif defined (FEATURE_PN80T)

         if (arg == 2) {
             /* power on w/FW */
             pn5xx_enable(pn5xx_dev, arg);

 #ifdef CONFIG_SLEEP_MONITOR
             nfc_is_flag = 1;
             pr_info("[NFC] nfc nfc_is_flag set true!!\n");
 #endif

         } else if (arg == 1) {
             /* power on */
             pn5xx_enable(pn5xx_dev, arg);

 #ifdef CONFIG_SLEEP_MONITOR
             nfc_is_flag = 1;
             pr_info("[NFC] nfc nfc_is_flag set true!!\n");
 #endif

         } else if (arg == 0) {
             /* power off */
             pn5xx_disable(pn5xx_dev);

 #ifdef CONFIG_SLEEP_MONITOR
             nfc_is_flag = 0;
             pr_info("[NFC] nfc nfc_is_flag set false!!\n");
 #endif

         } else if (arg == 3) {
             pr_info("[NFC] %s: Read Cancel\n", __func__);
             pn5xx_dev->cancel_read = true;
             atomic_set(&pn5xx_dev->read_flag, 1);
             wake_up(&pn5xx_dev->read_wq);
             //break;
         } else {
             pr_err("[NFC] %s bad SET_PWR arg %lu\n", __func__, arg);
             p61_access_unlock(pn5xx_dev);
             return -EINVAL;
         }
         //break;
#endif
	return ret;
}

static int pn5xx_p61_set_spi_pwr(struct pn5xx_dev *pdev,
	unsigned long arg)
{
	int ret = 0;
	p61_access_state_t current_state = P61_STATE_INVALID;

	p61_get_access_state(pn5xx_dev, &current_state);
	pr_info("[NFC] PN61_SET_SPI_PWR cur=0x%x\n", current_state);
	switch (arg) {
	case 0: /*else if (arg == 0)*/
		pr_info("[NFC] power off ese PN80T\n");
		if (current_state & P61_STATE_SPI_PRIO) {
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI_PRIO, false);
			if (!(current_state & P61_STATE_JCP_DWNLD)) {
				if (pn5xx_dev->nfc_service_pid) {
					pr_info("[NFC] %s nfc service pid %ld\n", __func__, pn5xx_dev->nfc_service_pid);
					if (!(current_state & P61_STATE_WIRED)) {
						svdd_sync_onoff(pn5xx_dev->nfc_service_pid,
							P61_STATE_SPI_SVDD_SYNC_START|P61_STATE_SPI_PRIO_END);
					} else
						signal_handler(P61_STATE_SPI_PRIO_END, pn5xx_dev->nfc_service_pid);
				} else {
					pr_err("[NFC] invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
				}
			} else if (!(current_state & P61_STATE_WIRED)) {
				svdd_sync_onoff(pn5xx_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
			}
			pn5xx_dev->spi_ven_enabled = false;
#ifndef VEN_ALWAYS_ON
			if (!(current_state & P61_STATE_WIRED)) {
#ifdef FEATURE_PN80T
				gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
				ese_spi_pinctrl(0);/*for factory spi pinctrl*/
				msleep(60);
#endif
				svdd_sync_onoff(pn5xx_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
			}
			if (pn5xx_dev->nfc_ven_enabled == false) {
				gpio_set_value(pn5xx_dev->ven_gpio, 0);
				usleep_range(10000, 10100);
			}
#endif
		} else if (current_state & P61_STATE_SPI) {
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI, false);
			if (!(current_state & P61_STATE_WIRED) && !(current_state & P61_STATE_JCP_DWNLD)) {
				if (pn5xx_dev->nfc_service_pid) {
					pr_info("[NFC] %s nfc service pid ---- %ld\n", __func__, pn5xx_dev->nfc_service_pid);
					svdd_sync_onoff(pn5xx_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_START|P61_STATE_SPI_END);
				} else {
					pr_err("[NFC] %s invalid nfc svc pid %ld\n", __func__, pn5xx_dev->nfc_service_pid);
				}
#if !defined(VEN_ALWAYS_ON) && defined(FEATURE_PN80T)
				gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
				ese_spi_pinctrl(0);
				msleep(60);
#endif

				svdd_sync_onoff(pn5xx_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
			}
			/*If JCOP3.2 or 3.3 for handling triple mode protection signal NFC service */
			else {
				if (!(current_state & P61_STATE_JCP_DWNLD)) {
					if (pn5xx_dev->nfc_service_pid) {
						pr_info("[NFC] %s nfc svc pid %ld\n", __func__, pn5xx_dev->nfc_service_pid);
						svdd_sync_onoff(pn5xx_dev->nfc_service_pid,
							P61_STATE_SPI_SVDD_SYNC_START|P61_STATE_SPI_END);
					} else {
						pr_err("[NFC] invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
					}
				} else {
					svdd_sync_onoff(
						pn5xx_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_START);
				}
#ifndef VEN_ALWAYS_ON
#ifdef FEATURE_PN80T
				gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
				ese_spi_pinctrl(0);
				msleep(60);
#endif
				svdd_sync_onoff(pn5xx_dev->nfc_service_pid,
					P61_STATE_SPI_SVDD_SYNC_END);
				pr_info("[NFC] PN80T ese_pwr_gpio off %s", __func__);
#endif
			}
			pn5xx_dev->spi_ven_enabled = false;
#ifndef VEN_ALWAYS_ON
			if (pn5xx_dev->nfc_ven_enabled == false) {
				gpio_set_value(pn5xx_dev->ven_gpio, 0);
				usleep_range(10000, 10100);
			}
#endif
		} else {
			pr_err("[NFC] %s : PN61_SET_SPI_PWR - failed, current_state = %x \n",
	                         __func__, pn5xx_dev->p61_current_state);
			//p61_access_unlock(pn5xx_dev);
			ret = -EPERM; /* Operation not permitted */
		}

		break;
	case 1: /*if (arg == 1) */
		pr_info("[NFC] %s : PN61_SET_SPI_PWR - power on ese\n", __func__);
		if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO|P61_STATE_DWNLD)) == 0) {
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI, true);

			/*To handle triple mode protection signal NFC service when SPI session started */
			if (!(current_state & P61_STATE_JCP_DWNLD)) {
				if (pn5xx_dev->nfc_service_pid) {
					pr_info("[NFC] %s nfc service pid ---- %ld\n",
			                        __func__, pn5xx_dev->nfc_service_pid);
					/*signal_handler(P61_STATE_SPI, pn5xx_dev->nfc_service_pid);*/
					dwp_onoff(pn5xx_dev->nfc_service_pid, P61_STATE_SPI);
				} else {
					pr_err("[NFC] %s invalid nfc svc pid %ld\n", __func__, pn5xx_dev->nfc_service_pid);
				}
			}
			pn5xx_dev->spi_ven_enabled = true;
#ifndef VEN_ALWAYS_ON
			if (pn5xx_dev->nfc_ven_enabled == false) {
				/*provide power to NFCC if, NFC service not provided*/
				gpio_set_value(pn5xx_dev->ven_gpio, 1);
				usleep_range(10000, 10100);
			}
			/* pull the gpio to high once NFCC is power on*/
#ifdef FEATURE_PN80T
			gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
			ese_spi_pinctrl(1);
			usleep_range(10000, 10100);
#endif
#endif
		} else {
			pr_err("[NFC] %s : PN61_SET_SPI_PWR -  power on ese failed \n", __func__);
			//p61_access_unlock(pn5xx_dev);
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 2: /*	else if (arg == 2) */
		pr_err("[NFC] %s : PN61_SET_SPI_PWR - reset\n", __func__);
		if (current_state & (P61_STATE_IDLE|P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
			if (pn5xx_dev->spi_ven_enabled == false) {
				pn5xx_dev->spi_ven_enabled = true;
#ifndef VEN_ALWAYS_ON
				if (pn5xx_dev->nfc_ven_enabled == false) {
					/* provide power to NFCC if,	NFC service not provided */
					gpio_set_value(pn5xx_dev->ven_gpio, 1);
					usleep_range(10000, 10100);
				}
#endif
			}
			svdd_sync_onoff(pn5xx_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
#if !defined(VEN_ALWAYS_ON) && defined(FEATURE_PN80T)
			gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
			msleep(60);
#endif
			svdd_sync_onoff(pn5xx_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
#if !defined(VEN_ALWAYS_ON) && defined(FEATURE_PN80T)
			gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
			usleep_range(10000, 10100);
#endif
		} else {
			pr_err("[NFC] %s : PN61_SET_SPI_PWR - reset  failed \n", __func__);
			//p61_access_unlock(pn5xx_dev);
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 3: /*else if (arg == 3) */
		pr_err("[NFC] %s : PN61_SET_SPI_PWR - Prio Session Start power on ese\n", __func__);
		if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO|P61_STATE_DWNLD)) == 0) {
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI_PRIO, true);
#if defined (FEATURE_PN80T) || defined (FEATURE_SN100X)
			if (current_state & P61_STATE_WIRED)
#endif
			{
				if (pn5xx_dev->nfc_service_pid) {
					pr_info("[NFC] nfc service pid %s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
					/*signal_handler(P61_STATE_SPI_PRIO, pn5xx_dev->nfc_service_pid);*/
					dwp_onoff(pn5xx_dev->nfc_service_pid, P61_STATE_SPI_PRIO);
				} else {
					pr_err("[NFC] invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
				}
			}
			pn5xx_dev->spi_ven_enabled = true;
#ifndef VEN_ALWAYS_ON
			if (pn5xx_dev->nfc_ven_enabled == false) {
				/* provide power to NFCC if,	NFC service not provided */
				gpio_set_value(pn5xx_dev->ven_gpio, 1);
				usleep_range(10000, 10100);
			}
			/* pull the gpio to high once NFCC is power on*/
#ifdef FEATURE_PN80T
			gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
			ese_spi_pinctrl(1);
			usleep_range(10000, 10100);
#endif
#endif
		} else {
			pr_err("[NFC] %s: Prio Session Start power on ese failed 0x%X\n", __func__, current_state);
	                //p61_access_unlock(pn5xx_dev);
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 4: /*else if (arg == 4)*/
		if (current_state & P61_STATE_SPI_PRIO) {
			pr_err("[NFC] %s : PN61_SET_SPI_PWR - Prio Session Ending...\n", __func__);
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI_PRIO, false);
			/* after SPI prio timeout, the state is changing from SPI prio to SPI */
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI, true);
#if defined (FEATURE_PN80T) || defined (FEATURE_SN100X)
			if (current_state & P61_STATE_WIRED)
#endif
			{
				if (pn5xx_dev->nfc_service_pid) {
					pr_info("[NFC] %s nfc service pid  %ld", __func__, pn5xx_dev->nfc_service_pid);
					signal_handler(P61_STATE_SPI_PRIO_END, pn5xx_dev->nfc_service_pid);
				} else {
					pr_err("[NFC] invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
				}
			}
		} else {
			pr_err("[NFC] %s : PN61_SET_SPI_PWR - Prio Session End failed 0x%X\n", __func__, current_state);
	                //p61_access_unlock(pn5xx_dev);
			ret = -EBADRQC; /* Device or resource busy */
		}
		break;
	case 5:
		release_ese_lock(P61_STATE_SPI);
		break;
#ifdef FEATURE_SN100X
	case 6:
		/*SPI Service called ISO-RST*/
		if(current_state & P61_STATE_WIRED) {
			return -EPERM; /* Operation not permitted */
		}
		if(current_state & P61_STATE_SPI) {
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI, false);
		}else if(current_state & P61_STATE_SPI_PRIO) {
			p61_update_access_state(pn5xx_dev, P61_STATE_SPI_PRIO, false);
		}
#ifdef ISO_RST
		gpio_set_value(pn5xx_dev->iso_rst_gpio, 0);
		msleep(50);
		gpio_set_value(pn5xx_dev->iso_rst_gpio, 1);
		msleep(50);
		pr_info("[NFC] %s ISO RESET from SPI DONE\n", __func__);
#endif
		break;
#endif //#ifdef FEATURE_SN100X
	default:
		pr_err("[NFC] %s bad ese pwr arg %lu\n", __func__, arg);
     		//p61_access_unlock(pn5xx_dev);
		ret = -EBADRQC; /* Invalid request code */
	}

	return ret;
}

#ifdef PM_MONITOR_WIRED_MODE
static void pn5xx_update_wired_access_state(unsigned long state)
{
	if (state == 1) { // Wired mode is activated
		nfc_stat_se_activate(current->pid);
		g_wired_access_state = P61_STATE_WIRED;
	} else if (state == 0) { // Wired mode is closed.
		nfc_stat_se_deactivate(current->pid);
		g_wired_access_state = P61_STATE_IDLE;
	} else {
		g_wired_access_state = P61_STATE_INVALID;
	}

	pr_err("[NFC] %s: P61_UPDATE_WIRED_ACCESS_STATE currunt state=0x%X\n",
		__func__, g_wired_access_state);
}
#endif

static int pn5xx_p61_set_wired_access(struct pn5xx_dev *pdev, unsigned long arg)
{
	p61_access_state_t current_state = P61_STATE_INVALID;
	int ret = 0;

	p61_get_access_state(pn5xx_dev, &current_state);
	pr_info("[NFC] %s: P61_SET_WIRED_ACCESS currunt state=0x%X\n", __func__, current_state);
	switch (arg) {
	case 0: /*else if (arg == 0)*/
		pr_err("[NFC] %s : P61_SET_WIRED_ACCESS - disabling \n", __func__);
		if (current_state & P61_STATE_WIRED) {
			p61_update_access_state(pn5xx_dev, P61_STATE_WIRED, false);
		} else {
			pr_err("[NFC] %s : P61_SET_WIRED_ACCESS - failed, current_state = %x \n",
	                        __func__, pn5xx_dev->p61_current_state);
	                 ///p61_access_unlock(pn5xx_dev);
			ret = -EPERM; /* Operation not permitted */
		}
		break;
	case 1: /*	if (arg == 1)*/
		if (current_state) {
			pr_err("[NFC] %s : P61_SET_WIRED_ACCESS - enabling\n", __func__);
			p61_update_access_state(pn5xx_dev, P61_STATE_WIRED, true);
			if (current_state & P61_STATE_SPI_PRIO) {
				if (pn5xx_dev->nfc_service_pid) {
					pr_info("[NFC] %s nfc service pid  %ld", __func__, pn5xx_dev->nfc_service_pid);
					signal_handler(P61_STATE_SPI_PRIO, pn5xx_dev->nfc_service_pid);
				} else {
					pr_err("[NFC] invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
				}
			}
		} else {
	                pr_err("[NFC] %s : P61_SET_WIRED_ACCESS -  enabling failed \n", __func__);
			//p61_access_unlock(pn5xx_dev);
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 2: /*	else if(arg == 2)*/
		pr_info("[NFC] %s : P61 ESE POWER REQ LOW \n", __func__);
		break;
	case 3: /*	else if(arg == 3)*/
		pr_info("[NFC] %s : P61 ESE POWER REQ HIGH \n", __func__);
		break;
	case 4: /*else if(arg == 4)*/
		release_ese_lock(P61_STATE_WIRED);
		break;
	default: /*else*/
		pr_err("[NFC] %s P61_SET_WIRED_ACCESS - bad arg %lu\n", __func__, arg);
		p61_access_unlock(pn5xx_dev);
		ret = -EBADRQC; /* Invalid request code */
	}

	return ret;
}

long  pn5xx_dev_ioctl(struct file *filp, unsigned int cmd,
                unsigned long arg)
{
    /* struct pn5xx_dev *pn5xx_dev = filp->private_data; */

    //struct pn5xx_dev *pn5xx_dev =  container_of(filp->private_data,struct pn5xx_dev, pn5xx_device);
#ifdef	CONFIG_NOBLESSE
    if(system_rev < 5){
        pr_err("[NFC] Old HW with SN 110 Chip so exiting\n");
        return -(SEC_NFC_ESE_PWR_PIN_ERR);
    }
#endif

    /* Free pass autobahn area, not protected. Use it carefullly. START */
    switch(cmd)
    {
        case PN5XX_GET_ESE_ACCESS:
            return get_ese_lock(P61_STATE_WIRED, arg);
        break;
        case PN5XX_REL_SVDD_WAIT:
            return release_svdd_wait();
        break;
        case PN5XX_SET_NFC_SERVICE_PID:
            return set_nfc_pid(arg);
        break;
        case PN5XX_REL_DWPONOFF_WAIT:
            return release_dwpOnOff_wait();
        break;
#ifdef PM_MONITOR_WIRED_MODE
        case P61_UPDATE_WIRED_ACCESS_STATE:
            pr_err("[NFC] pn5xx_dev_ioctl call with P61_UPDATE_WIRED_ACCESS_STATE, arg = %ld", arg);
            pn5xx_update_wired_access_state(arg); //return update_wired_access_state();
            return 0;
        break;
#endif
        default:
        break;
    }
    /* Free pass autobahn area, not protected. Use it carefullly. END */

    p61_access_lock(pn5xx_dev);
    switch (cmd) {
    case PN5XX_SET_PWR:
	pn5xx_set_pwr(pn5xx_dev, arg);
        break;

    case P61_SET_SPI_PWR:
    {
    	int ret = 0;
	ret = pn5xx_p61_set_spi_pwr(pn5xx_dev, arg);

        if (ret < 0)
             p61_access_unlock(pn5xx_dev);
    }
    break;
   case P61_GET_PWR_STATUS:
    {
        p61_access_state_t  current_state = P61_STATE_INVALID;
	p61_get_access_state(pn5xx_dev, &current_state);
        pr_err("[NFC] %s: P61_GET_PWR_STATUS  = %x",__func__, current_state);
        put_user(current_state, (int __user *)arg);
    }
    break;
    case P61_SET_WIRED_ACCESS:
    {
	pn5xx_p61_set_wired_access(pn5xx_dev, arg);
    }
    break;
    case PN5XX_SET_DWNLD_STATUS:
    {
	int ret = 0;
        ret = set_jcop_download_state(arg);
        if (ret < 0) {
	    pr_err("[NFC] %s set_jcop_download_state failed \n", __func__);
            p61_access_unlock(pn5xx_dev);
	}
        break;
    }
    default:
        pr_err("[NFC] %s bad ioctl %u\n", __func__, cmd);
        p61_access_unlock(pn5xx_dev);
        pr_info("[NFC] %s :exit cmd = %u, arg = %ld\n", __func__, cmd, arg);
       return -EINVAL;
    }
    p61_access_unlock(pn5xx_dev);
    return 0;
}

EXPORT_SYMBOL(pn5xx_dev_ioctl);

#ifdef FEATURE_SN100X
int get_ese_lock(enum p61_access_state  p61_current_state, int timeout)
{
	unsigned long tempJ = msecs_to_jiffies(timeout);

	pr_info("[NFC] enter p61_current_state=0x%x, timeout=%d, jiffies=%lu\n",
		p61_current_state, timeout, tempJ);
	reinit_completion(&pn5xx_dev->ese_comp);

	if (p61_trans_acc_on) {
		if (!wait_for_completion_timeout(&pn5xx_dev->ese_comp, tempJ)) {
			pr_err("[NFC] %s timeout p61_current_state = %d\n", __func__, p61_current_state);
			return -EBUSY;
		}
	}

	p61_trans_acc_on = 1;
	pr_info("[NFC] exit p61_trans_acc_on =%d, timeout = %d\n",
		p61_trans_acc_on, timeout);
	return 0;
}
EXPORT_SYMBOL(get_ese_lock);

static void release_ese_lock(enum p61_access_state  p61_current_state)
{
	pr_info("[NFC] enter p61_current_state = (0x%x)\n",
			p61_current_state);
	p61_trans_acc_on = 0;
	complete(&pn5xx_dev->ese_comp);
	pr_info("[NFC] p61_trans_acc_on =%d exit\n", p61_trans_acc_on);
}

#elif defined (FEATURE_PN80T)
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout)
{
    unsigned long tempJ = msecs_to_jiffies(timeout);
    pr_info("[NFC] get_ese_lock: enter p61_current_state =(0x%x), timeout = %d, jiffies = %lu\n"
     , p61_current_state, timeout, tempJ);
    if(down_timeout(&ese_access_sema, tempJ) != 0)
    {
        pr_err("[NFC] get_ese_lock: timeout p61_current_state = %d\n", p61_current_state);
        return -EBUSY;
    }

    p61_trans_acc_on = 1;
    pr_info("[NFC] get_ese_lock: exit p61_trans_acc_on =%d, timeout = %d\n"
            , p61_trans_acc_on, timeout);
    return 0;
}
EXPORT_SYMBOL(get_ese_lock);

static void release_ese_lock(p61_access_state_t  p61_current_state)
{
    pr_info("[NFC] %s: enter p61_current_state = (0x%x)\n", __func__, p61_current_state);
    up(&ese_access_sema);
    p61_trans_acc_on = 0;
    pr_info("[NFC] %s: p61_trans_acc_on =%d exit\n", __func__, p61_trans_acc_on);
}
#endif // #ifdef FEATURE_SN100X

static const struct file_operations pn5xx_dev_fops = {
    .owner    = THIS_MODULE,
   // .llseek    = no_llseek,
    .read    = pn5xx_dev_read,
    .write    = pn5xx_dev_write,
    .open    = pn5xx_dev_open,
    .release  = pn5xx_dev_release,
    .compat_ioctl = pn5xx_dev_ioctl,
};

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static int pn5xx_get_pdata(struct device *dev,
                           struct pn5xx_i2c_platform_data *pdata)
{
    struct device_node *node;
    u32 flags;
    int val;

    /* make sure there is actually a device tree node */
    node = dev->of_node;
    if (!node)
        return -ENODEV;

    memset(pdata, 0, sizeof(*pdata));

    /* read the dev tree data */

    /* ven pin - enable's power to the chip - REQUIRED */
    val = of_get_named_gpio_flags(node, "sec-nfc,ven-gpio", 0, &flags);
    if (val >= 0) {
        pdata->ven_gpio = val;
        //pr_info("[NFC] %s : nfc ven gpio %d\n" , __func__ , pdata->ven_gpio);
    }
    else{
        dev_err(dev, "VEN GPIO error getting from OF node\n");
        return val;
    }

    /* firm pin - controls firmware download - OPTIONAL */
    val = of_get_named_gpio_flags(node, "sec-nfc,firm-gpio", 0, &flags);
    if (val >= 0) {
        pdata->firm_gpio = val;
        //pr_info("[NFC] %s : nfc firm_gpio %d\n" , __func__ , pdata->firm_gpio);
    }
    else {
        pdata->firm_gpio = -EINVAL;
        dev_warn(dev, "FIRM GPIO <OPTIONAL> error getting from OF node\n");
    }

    /* irq pin - data available irq - REQUIRED */
    val = of_get_named_gpio_flags(node, "sec-nfc,irq-gpio", 0, &flags);
    if (val >= 0) {
        pdata->irq_gpio = val;
        //pr_info("[NFC] %s : nfc irq_gpio %d\n" , __func__ , pdata->irq_gpio);
    }
    else {
        dev_err(dev, "IRQ GPIO error getting from OF node\n");
        return val;
    }

#ifdef FEATURE_PN80T
    /* ese-pwr pin - enable's power to the ese- REQUIRED */
    val = of_get_named_gpio_flags(node, "sec-nfc,ese-pwr", 0, &flags);
    if (val >= 0) {
        pdata->ese_pwr_gpio = val;

    }
    else {
        dev_err(dev, "ESE PWR GPIO error getting from OF node\n");
 /* working from REV0.3 in Solis */
        /* return val; */
    }
#endif

    return 0;
}
#else
static int pn5xx_get_pdata(struct device *dev,
                           struct pn5xx_i2c_platform_data *pdata)
{
    pdata = dev->platform_data;
    return 0;
}
#endif

#ifdef CONFIG_NFC_FEATURE_SN100U
static int pn5xx_regulator_onoff(struct device *dev,
		struct pn5xx_dev *pdev, int onoff)
{
	int rc = 0;
	struct regulator *regulator_nfc_pvdd = pdev->nfc_pvdd;

	if (!regulator_nfc_pvdd) {
		pr_err("[NFC] error: null regulator!\n");
		rc = -ENODEV;
		goto done;
	}

	pr_info("[NFC] onoff = %d\n", onoff);
	if (onoff == NFC_I2C_LDO_ON) {
		rc = regulator_set_load(regulator_nfc_pvdd, 300000);
		if (rc) {
			pr_err("[NFC] regulator_uwb_vdd set_load failed, rc=%d\n", rc);
			goto done;
		}
		rc = regulator_enable(regulator_nfc_pvdd);
		if (rc) {
			pr_err("[NFC] enable failed, rc=%d\n", rc);
			goto done;
		}
	} else {
		rc = regulator_disable(regulator_nfc_pvdd);
		if (rc) {
			pr_err("[NFC] disable failed, rc=%d\n", rc);
			goto done;
		}
	}
#if NFC_DEBUG
	pr_info("[NFC] success\n");
#endif
done:
	return rc;
}
#endif

/*
 *  pn5xx_probe
 */
#ifdef KERNEL_3_4_AND_OLDER
 static int __devinit pn5xx_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
#else
static int pn5xx_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
#endif
{

#ifdef CONFIG_NFC_FEATURE_SN100U
    int addr;
    char tmp[4] = {0x20, 0x00, 0x01, 0x01};
    int addrcnt;
    struct device *dev = &client->dev;
#endif
    int ret;
    struct pn5xx_i2c_platform_data *pdata;  // gpio values, from board file or DT
    struct pn5xx_i2c_platform_data tmp_pdata;
    /* struct pn5xx_dev *pn5xx_dev;            // internal device specific data */

#ifdef	CONFIG_NOBLESSE
    if(system_rev < 5){
        pr_err("[NFC] Old HW(%d) with SN 110 Chip so exiting\n",system_rev);
        return 0;
    }
#endif

#ifdef CONFIG_SLEEP_MONITOR
    sleep_monitor_register_ops(NULL, &nfc_sleep_monitor_ops,
    SLEEP_MONITOR_NFC);
#endif

    /* ---- retrieve the platform data ---- */
    /* If the dev.platform_data is NULL, then */
    /* attempt to read from the device tree */
    if(!client->dev.platform_data)
    {
        ret = pn5xx_get_pdata(&(client->dev), &tmp_pdata);
        if(ret){
            return ret;
        }

        pdata = &tmp_pdata;
    }
    else
    {
        pdata = client->dev.platform_data;
    }

    if (pdata == NULL) {
        pr_err("[NFC] %s : nfc probe fail\n", __func__);
        return  -ENODEV;
    }

    /* validate the the adapter has basic I2C functionality */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("[NFC] %s : nfc need I2C_FUNC_I2C\n", __func__);
        return -ENODEV;
    }

    /* reserve the GPIO pins */
    //_info("%s: nfc request irq_gpio %d\n", __func__, pdata->irq_gpio);
    ret = gpio_request(pdata->irq_gpio, "nfc_int");
    if (ret){
        pr_err("[NFC] %s :nfc not able to get GPIO irq_gpio\n", __func__);
        return  -ENODEV;
    }
    ret = gpio_to_irq(pdata->irq_gpio);
    if (ret < 0){
        pr_err("[NFC] %s :nfc not able to map GPIO irq_gpio to an IRQ\n", __func__);
        goto err_ven;
    }
    else{
        client->irq = ret;
    }

    //_info("%s: nfc request ven_gpio %d\n", __func__, pdata->ven_gpio);
    ret = gpio_request(pdata->ven_gpio, "nfc_ven");
    if (ret){
        pr_err("[NFC] %s :nfc not able to get GPIO ven_gpio\n", __func__);
        goto err_ven;
    }

    if (gpio_is_valid(pdata->firm_gpio)) {
        //_info("%s: nfc request firm_gpio %d\n", __func__, pdata->firm_gpio);
        ret = gpio_request(pdata->firm_gpio, "nfc_firm");
        if (ret){
            pr_err("[NFC] %s :nfc not able to get GPIO firm_gpio\n", __func__);
            goto err_firm;
        }
    }
#ifdef FEATURE_PN80T
    if (gpio_is_valid(pdata->ese_pwr_gpio)) {
        //_info("%s: nfc request ese_pwr_gpio %d\n", __func__, pdata->ese_pwr_gpio);
        ret = gpio_request(pdata->ese_pwr_gpio, "nfc_ese_pwr");
        if (ret){
            pr_err("[NFC] %s :not able to get GPIO ese_pwr_gpio\n", __func__);
            goto err_ese_pwr;
        }
    }
#endif

    /* allocate the pn5xx driver information structure */
    pn5xx_dev = kzalloc(sizeof(*pn5xx_dev), GFP_KERNEL);
    if (pn5xx_dev == NULL) {
        dev_err(&client->dev, "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    wake_lock_init(&pn5xx_dev->nfc_wake_lock,
            WAKE_LOCK_SUSPEND, "nfc_wake_lock");

    /* store the platform data in the driver info struct */
    pn5xx_dev->irq_gpio = pdata->irq_gpio;
    pn5xx_dev->ven_gpio  = pdata->ven_gpio;
    pn5xx_dev->firm_gpio  = pdata->firm_gpio;
#ifdef FEATURE_PN80T
    pn5xx_dev->ese_pwr_gpio  = pdata->ese_pwr_gpio;
#endif
    pn5xx_dev->p61_current_state = P61_STATE_IDLE;
    pn5xx_dev->nfc_ven_enabled = false;
    pn5xx_dev->spi_ven_enabled = false;

    pn5xx_dev->clkreq_gpio = pdata->clkreq_gpio;

#ifdef FEATURE_PN80T
    pn5xx_dev->pvdd_reg = pdata->pvdd_reg;
    pn5xx_dev->vbat_reg = pdata->vbat_reg;
    pn5xx_dev->pmuvcc_reg = pdata->vbat_reg;
    pn5xx_dev->sevdd_reg = pdata->sevdd_reg;
#endif

#ifdef CONFIG_NFC_FEATURE_SN100U
    p61_trans_acc_on = 0;
    init_completion(&pn5xx_dev->ese_comp);
    init_completion(&pn5xx_dev->svdd_sync_comp);
    init_completion(&pn5xx_dev->dwp_onoff_comp);
    sema_init(&dwp_onoff_release_sema, 0);

    if (of_get_property(dev->of_node, "sec-nfc,ldo_control", NULL)) {
        pn5xx_dev->nfc_pvdd = devm_regulator_get(dev, "sec-nfc,nfc_pvdd");
        if (!pn5xx_dev->nfc_pvdd) {
            pr_err("[NFC] get nfc_pvdd error\n");
            pn5xx_dev->nfc_pvdd = NULL;
        } else
            pr_info("[NFC] LDO nfc_pvdd: valid\n");
    } else {
        pn5xx_dev->pvdd = of_get_named_gpio(dev->of_node, "sec-nfc,pvdd-gpio", 0);
        if (pn5xx_dev->pvdd < 0) {
            pr_err("[NFC] pvdd-gpio is not set.");
            pn5xx_dev->pvdd = 0;
        }
    }
#endif

    pn5xx_dev->client = client;
#ifdef CONFIG_NFC_FEATURE_SN100U
    ret = pn5xx_regulator_onoff(&client->dev, pn5xx_dev, NFC_I2C_LDO_ON);
	if (ret < 0)
		pr_err("[NFC] regulator_on fail err: %d\n", ret);
	usleep_range(4500, 4600); /* spec : VDDIO high -> 4.5 ms -> VEN high*/

#endif
    /* finish configuring the I/O */
    ret = gpio_direction_input(pn5xx_dev->irq_gpio);
    if (ret < 0) {
        pr_err("[NFC] %s :nfc not able to set irq_gpio as input\n", __func__);
        goto err_exit;
    }

    ret = gpio_direction_output(pn5xx_dev->ven_gpio, 0);
    if (ret < 0) {
        pr_err("[NFC] %s : nfc not able to set ven_gpio as output\n", __func__);
        goto err_exit;
    }

    if (gpio_is_valid(pn5xx_dev->firm_gpio)) {
        ret = gpio_direction_output(pn5xx_dev->firm_gpio, 0);
        if (ret < 0) {
            pr_err("[NFC] %s : nfc not able to set firm_gpio as output\n",
            __func__);
            goto err_exit;
        }
    }
#ifndef CONFIG_NFC_FEATURE_SN100U
    if (gpio_is_valid(pn5xx_dev->ese_pwr_gpio)) {
        ret = gpio_direction_output(pn5xx_dev->ese_pwr_gpio, 0);
        if (ret < 0) {
            pr_err("[NFC] %s : not able to set ese_pwr gpio as output\n", __func__);
            goto err_ese_pwr;
        }
    }
#endif
#ifdef FEATURE_SN100X
	pn5xx_dev->state_flags = 0x00;
#endif
    /* init mutex and queues */
    init_waitqueue_head(&pn5xx_dev->read_wq);
    mutex_init(&pn5xx_dev->read_mutex);
#ifdef FEATURE_SN100X
	sema_init(&pn5xx_access_sema, 1);
#endif
    mutex_init(&pn5xx_dev->p61_state_mutex);
    sema_init(&ese_access_sema, 1);
   //spin_lock_init(&pn5xx_dev->irq_enabled_lock);

    /* register as a misc device - character based with one entry point */
    pn5xx_dev->pn5xx_device.minor = MISC_DYNAMIC_MINOR;
 //pn5xx_dev->pn5xx_device.name = CHIP;
 pn5xx_dev->pn5xx_device.name = "sec-nfc";
#if 0 //def	CONFIG_NOBLESSE
    if(system_rev < 5){
        pr_err("[NFC] Noblesse hw id(%d) is not support NFC\n", system_rev);
    } else {
        pn5xx_dev->pn5xx_device.fops = &pn5xx_dev_fops;
 	}
#else
    pn5xx_dev->pn5xx_device.fops = &pn5xx_dev_fops;
#endif

    ret = misc_register(&pn5xx_dev->pn5xx_device);
    if (ret) {
        pr_err("[NFC] %s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }
    /* request irq.  the irq is set whenever the chip has data available
    * for reading.  it is cleared when all data has been read.
    */
    //pr_info("[NFC] %s : nfc requesting IRQ %d\n", __func__, client->irq);
    //pr_info("[NFC] %s : nfc requesting IRQ %s\n", __func__, client->name);

    ret = request_irq(client->irq, pn5xx_dev_irq_handler,
              IRQF_TRIGGER_RISING, SEC_NFC_DRIVER_NAME/*client->name*/, pn5xx_dev);

    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }

    disable_irq_nosync(pn5xx_dev->client->irq);
    atomic_set(&pn5xx_dev->irq_enabled, 0);

    i2c_set_clientdata(client, pn5xx_dev);

#ifdef CONFIG_NFC_FEATURE_SN100U
	gpio_set_value(pn5xx_dev->ven_gpio, 1);
	gpio_set_value(pn5xx_dev->firm_gpio, 1); /* add firmware pin */
	usleep_range(4900, 5000);
	gpio_set_value(pn5xx_dev->ven_gpio, 0);
	usleep_range(14900, 15000);
	gpio_set_value(pn5xx_dev->ven_gpio, 1);
	usleep_range(4900, 5000);

	addr = 0x2B;
	client->addr = addr;
	addrcnt = 2;
	do {
		ret = i2c_master_send(client, tmp, 4);
		if (ret > 0) {
			pr_err("[NFC] i2c addr(0x%X), ret(%d)\n",
					client->addr, ret);
			pn5xx_dev->i2c_probe = ret;
			break;
		}
	} while (addrcnt--);

	if (ret <= 0) {
		pr_err("[NFC] ret(%d), i2c_probe(%d)\n", ret, pn5xx_dev->i2c_probe);
		client->addr = 0x2B;
	}
	gpio_set_value(pn5xx_dev->ven_gpio, 0);
	gpio_set_value(pn5xx_dev->firm_gpio, 0); /* add */

#ifdef VEN_ALWAYS_ON
	usleep_range(14900, 15000);
	gpio_set_value(pn5xx_dev->ven_gpio, 1);
	atomic_set(&pn5xx_dev->irq_enabled, 1);
	enable_irq(client->irq);
	enable_irq_wake(client->irq);
#endif

	if (ret < 0)
		pr_err("[NFC] fail to get i2c addr\n");
	else
		pr_info("[NFC] success, i2c_probe(%d)\n", pn5xx_dev->i2c_probe);
#endif
    printk("[NFC] nfc pn5xx_probe END!!!!!!!!!!!!!!!!\n");
    return 0;

err_request_irq_failed:
    misc_deregister(&pn5xx_dev->pn5xx_device);
    wake_lock_destroy(&pn5xx_dev->nfc_wake_lock);
err_misc_register:
    mutex_destroy(&pn5xx_dev->read_mutex);
    mutex_destroy(&pn5xx_dev->p61_state_mutex);
    kfree(pn5xx_dev);
err_exit:
    if (gpio_is_valid(pdata->clkreq_gpio))
        gpio_free(pdata->clkreq_gpio);
#ifndef CONFIG_NFC_FEATURE_SN100U
err_ese_pwr:
    if (gpio_is_valid(pdata->ese_pwr_gpio))
        gpio_free(pdata->ese_pwr_gpio);
#endif

err_firm:
    gpio_free(pdata->ven_gpio);
err_ven:
    gpio_free(pdata->irq_gpio);
    return ret;
}

#ifdef KERNEL_3_4_AND_OLDER
static int __devexit pn5xx_remove(struct i2c_client *client)
#else
static int pn5xx_remove(struct i2c_client *client)
#endif
{
    struct pn5xx_dev *pn5xx_dev;

    pr_info("[NFC] nfc %s\n", __func__);

#ifdef CONFIG_SLEEP_MONITOR
    sleep_monitor_unregister_ops(SLEEP_MONITOR_NFC);
#endif
    pn5xx_dev = i2c_get_clientdata(client);
    wake_lock_destroy(&pn5xx_dev->nfc_wake_lock);
    free_irq(client->irq, pn5xx_dev);
    misc_deregister(&pn5xx_dev->pn5xx_device);
    mutex_destroy(&pn5xx_dev->read_mutex);
    mutex_destroy(&pn5xx_dev->p61_state_mutex);
    gpio_free(pn5xx_dev->irq_gpio);
    gpio_free(pn5xx_dev->ven_gpio);
#ifdef FEATURE_PN80T
    if (gpio_is_valid(pn5xx_dev->ese_pwr_gpio))
        gpio_free(pn5xx_dev->ese_pwr_gpio);
#endif

    pn5xx_dev->p61_current_state = P61_STATE_INVALID;
    pn5xx_dev->nfc_ven_enabled = false;
    pn5xx_dev->spi_ven_enabled = false;

    if (gpio_is_valid(pn5xx_dev->firm_gpio))
        gpio_free(pn5xx_dev->firm_gpio);
    if (gpio_is_valid(pn5xx_dev->clkreq_gpio))
        gpio_free(pn5xx_dev->clkreq_gpio);

#ifdef FEATURE_PN80T
    regulator_put(pn5xx_dev->pvdd_reg);
    regulator_put(pn5xx_dev->vbat_reg);
    regulator_put(pn5xx_dev->pmuvcc_reg);
    regulator_put(pn5xx_dev->sevdd_reg);
#endif

 #ifdef FEATURE_SN100X
    if (pn5xx_dev->nfc_pvdd) {
        devm_regulator_put(pn5xx_dev->nfc_pvdd);
        pn5xx_dev->nfc_pvdd = NULL;
    }
#endif

    kfree(pn5xx_dev);

    return 0;
}

/*
 *
 */
#ifdef CONFIG_OF
static struct of_device_id pn5xx_dt_match[] = {
    { .compatible = "sec-nfc,i2c"},
    {},
};
MODULE_DEVICE_TABLE(of, pn5xx_dt_match);
#endif

static const struct i2c_device_id pn5xx_id[] = {
    { SEC_NFC_DRIVER_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, pn5xx_id);

static struct i2c_driver pn5xx_driver = {
    .probe        = pn5xx_probe,
    .id_table    = pn5xx_id,
#ifdef KERNEL_3_4_AND_OLDER
    .remove        = __devexit_p(pn5xx_remove),
#else
    .remove        = pn5xx_remove,
#endif
    .driver        = {
        .name    = SEC_NFC_DRIVER_NAME,
        .of_match_table = pn5xx_dt_match,
    },
};

/*
 * module load/unload record keeping
 */

static int __init pn5xx_dev_init(void)
{
    int ret = 0;
    pr_info("[NFC] nfc %s\n", __func__);
    //printk("[NFC] nfc pn5xx_dev_init!!!!!!!!!!!!\n");
    ret = i2c_add_driver(&pn5xx_driver);
    //printk("[NFC] nfc pn5xx_dev_init!! = [%d]\n" , ret);

    return ret;
}

static void __exit pn5xx_dev_exit(void)
{
    pr_info("[NFC] nfc %s\n", __func__);
    //printk("[NFC] pn5xx_dev_exit!!\n");
    i2c_del_driver(&pn5xx_driver);
}

module_init(pn5xx_dev_init);
module_exit(pn5xx_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

