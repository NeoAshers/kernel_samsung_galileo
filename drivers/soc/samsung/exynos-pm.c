/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/wakeup_reason.h>
#include <linux/gpio.h>
#include <linux/syscore_ops.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/psci.h>
#include <linux/debugfs.h>
#include <asm/cpuidle.h>
#include <asm/smp_plat.h>
#ifdef CONFIG_IRQ_HISTORY
#include <linux/power/irq_history.h>
#endif

#include <soc/samsung/exynos-pm.h>
#include <soc/samsung/exynos-pmu.h>
#include <soc/samsung/exynos-powermode.h>
#include "cal-if/cmucal.h"

#define WAKEUP_STAT_EINT                (1 << 0)
#define WAKEUP_STAT_RTC_ALARM           (1 << 1)
#define WAKEUP_STAT_GNSS				(BIT(26) |BIT(21))
#define WAKEUP_STAT_CP					(BIT(29) |BIT(25) | BIT(24) | BIT(20))
#define WAKEUP_STAT_CHUB                (1 << 2)
#define WAKEUP_STAT_VTS                 (1 << 0)

/*
 * PMU register offset
 */
#define EXYNOS_PMU_WAKEUP_STAT			0x0600
#define EXYNOS_PMU_EINT_WAKEUP_MASK		0x060C
#define BOOT_CPU			0
#define CPU_INFORM4			0x0870
#define EXYNOS_PMU_WAKEUP_STAT4			0x0640

#ifdef CONFIG_SLEEP_STAT
extern void	sleep_stat_exynos_update_slpm_wakeup(void);
#endif

extern u32 exynos_eint_to_pin_num(int eint);
#define EXYNOS_EINT_PEND(b, x)      ((b) + 0xA00 + (((x) >> 3) * 4))

struct exynos_pm_info {
	void __iomem *eint_base;		/* GPIO_ALIVE base to check wkup reason */
	void __iomem *gic_base;			/* GICD_ISPENDRn base to check wkup reason */
	unsigned int num_eint;			/* Total number of EINT sources */
	unsigned int num_gic;			/* Total number of GIC sources */
	bool is_early_wakeup;
	bool is_usbl2_suspend;
	unsigned int suspend_mode_idx;		/* power mode to be used in suspend scenario */
	unsigned int suspend_psci_idx;		/* psci index to be used in suspend scenario */
	u8 num_extra_stat;			/* Total number of extra wakeup_stat */
	unsigned int *extra_wakeup_stat;	/* Extra wakeup stat SFRs offset */

	unsigned int usbl2_suspend_available;
	unsigned int usbl2_suspend_mode_idx;		/* power mode to be used in suspend scenario */
	bool (*usb_is_connect)(void);
};
static struct exynos_pm_info *pm_info;

struct exynos_pm_dbg {
	u32 test_early_wakeup;
	u32 test_usbl2_suspend;
};
static struct exynos_pm_dbg *pm_dbg;

static void exynos_show_wakeup_reason_eint(void)
{
	int bit;
	int i, size;
	long unsigned int ext_int_pend;
	u64 eint_wakeup_mask;
	bool found = 0;
	unsigned int val = 0;

	exynos_pmu_read(EXYNOS_PMU_EINT_WAKEUP_MASK, &val);
	eint_wakeup_mask = val;

	for (i = 0, size = 8; i < pm_info->num_eint; i += size) {
		ext_int_pend =
			__raw_readl(EXYNOS_EINT_PEND(pm_info->eint_base, i));

		for_each_set_bit(bit, &ext_int_pend, size) {
			u32 gpio;
			int irq;

			if (eint_wakeup_mask & (1 << (i + bit)))
				continue;

			gpio = exynos_eint_to_pin_num(i + bit);
			irq = gpio_to_irq(gpio);

#ifdef CONFIG_SUSPEND
#ifdef CONFIG_IRQ_HISTORY
			add_irq_history(irq, NULL);
#else
			log_wakeup_reason(irq);
#endif
			//update_wakeup_reason_stats(irq, i + bit);
#endif
			found = 1;
		}
	}

	if (!found) {
		pr_info("%s Resume caused by unknown EINT\n", EXYNOS_PM_PREFIX);
#ifdef CONFIG_IRQ_HISTORY
		add_irq_history(0, "ETC");
#endif
	}
}

static void exynos_show_wakeup_registers(unsigned int wakeup_stat)
{
	int i, size;
	int extra_wakeup_stat;

	pr_info("WAKEUP_STAT:\n");
	pr_info("%s: 0x%08x\n", EXYNOS_PM_PREFIX, wakeup_stat);
	for (i = 0; i < pm_info->num_extra_stat; i++) {
		exynos_pmu_read(pm_info->extra_wakeup_stat[i], &extra_wakeup_stat);
		pr_info("%s 0x%08x\n", EXYNOS_PM_PREFIX, extra_wakeup_stat);
	}

	pr_info("EINT_PEND: ");
	for (i = 0, size = 8; i < pm_info->num_eint; i += size)
		pr_info("%s 0x%02x ", EXYNOS_PM_PREFIX, __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_base, i)));
}

static void exynos_show_wakeup_reason(bool sleep_abort)
{
	unsigned int wakeup_stat, wakeup_stat4;
	int i, size;

	if (sleep_abort) {
		pr_info("%s early wakeup! Dumping pending registers...\n", EXYNOS_PM_PREFIX);

		pr_info("EINT_PEND:\n");
		for (i = 0, size = 8; i < pm_info->num_eint; i += size)
			pr_info("0x%x\n", __raw_readl(EXYNOS_EINT_PEND(pm_info->eint_base, i)));

		pr_info("GIC_PEND:\n");
		for (i = 0; i < pm_info->num_gic; i++)
			pr_info("GICD_ISPENDR[%d] = 0x%x\n", i, __raw_readl(pm_info->gic_base + i*4));

		pr_info("%s done.\n", EXYNOS_PM_PREFIX);
		return ;
	}

	exynos_pmu_read(EXYNOS_PMU_WAKEUP_STAT, &wakeup_stat);
	exynos_show_wakeup_registers(wakeup_stat);

	exynos_pmu_read(EXYNOS_PMU_WAKEUP_STAT4, &wakeup_stat4);
#ifdef CONFIG_SLEEP_STAT
	sleep_stat_exynos_update_slpm_wakeup();
#endif

	if (wakeup_stat & WAKEUP_STAT_RTC_ALARM)
		pr_info("%s Resume caused by RTC alarm\n", EXYNOS_PM_PREFIX);
	else if (wakeup_stat & WAKEUP_STAT_EINT)
		exynos_show_wakeup_reason_eint();
#ifdef CONFIG_IRQ_HISTORY
	else if (wakeup_stat & WAKEUP_STAT_GNSS)
		add_irq_history(0, "GNSS");
	else if (wakeup_stat & WAKEUP_STAT_CP)
		add_irq_history(0, "CP");
	else {
		if (wakeup_stat4 & WAKEUP_STAT_CHUB)
			add_irq_history(0, "CHub");
		else {
			pr_info("%s Resume caused by wakeup_stat 0x%08x wakeup_stat4 0x%08x\n",
				EXYNOS_PM_PREFIX, wakeup_stat, wakeup_stat4);
			add_irq_history(0, "ETC");
		}
	}
#else
	else
		pr_info("%s Resume caused by wakeup_stat 0x%08x\n",
			EXYNOS_PM_PREFIX, wakeup_stat);
#endif
}

#ifdef CONFIG_CPU_IDLE
static DEFINE_RWLOCK(exynos_pm_notifier_lock);
static RAW_NOTIFIER_HEAD(exynos_pm_notifier_chain);

int exynos_pm_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&exynos_pm_notifier_lock, flags);
	ret = raw_notifier_chain_register(&exynos_pm_notifier_chain, nb);
	write_unlock_irqrestore(&exynos_pm_notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pm_register_notifier);

int exynos_pm_unregister_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&exynos_pm_notifier_lock, flags);
	ret = raw_notifier_chain_unregister(&exynos_pm_notifier_chain, nb);
	write_unlock_irqrestore(&exynos_pm_notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pm_unregister_notifier);

static int __exynos_pm_notify(enum exynos_pm_event event, int nr_to_call, int *nr_calls)
{
	int ret;

	ret = __raw_notifier_call_chain(&exynos_pm_notifier_chain, event, NULL,
		nr_to_call, nr_calls);

	return notifier_to_errno(ret);
}

int exynos_pm_notify(enum exynos_pm_event event)
{
	int nr_calls;
	int ret = 0;

	read_lock(&exynos_pm_notifier_lock);
	ret = __exynos_pm_notify(event, -1, &nr_calls);
	read_unlock(&exynos_pm_notifier_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pm_notify);
#endif /* CONFIG_CPU_IDLE */

#if defined(CONFIG_SOC_EXYNOS8895)
#define SLEEP_VTS_ON   9
#define SLEEP_AUD_ON   10
#endif

static int exynos_pm_syscore_suspend(void)
{
	if (!exynos_check_cp_status()) {
		pr_info("%s %s: sleep canceled by CP reset \n",
					EXYNOS_PM_PREFIX, __func__);
		return -EINVAL;
	}

	pm_info->is_usbl2_suspend = false;
	if (pm_info->usbl2_suspend_available) {
		if (!IS_ERR_OR_NULL(pm_info->usb_is_connect))
			pm_info->is_usbl2_suspend = pm_info->usb_is_connect();
	}

	if (pm_info->is_usbl2_suspend || pm_dbg->test_usbl2_suspend) {
		exynos_prepare_sys_powerdown(pm_info->usbl2_suspend_mode_idx);
		pr_info("%s %s: Enter Suspend scenario. usbl2_mode_idx = %d)\n",
				EXYNOS_PM_PREFIX,__func__, pm_info->usbl2_suspend_mode_idx);
	} else {
		exynos_prepare_sys_powerdown(pm_info->suspend_mode_idx);
		pr_info("%s %s: Enter Suspend scenario. suspend_mode_idx = %d)\n",
				EXYNOS_PM_PREFIX,__func__, pm_info->suspend_mode_idx);
	}

	return 0;
}

static void exynos_pm_syscore_resume(void)
{
	if (pm_info->is_usbl2_suspend || pm_dbg->test_usbl2_suspend)
		exynos_wakeup_sys_powerdown(pm_info->usbl2_suspend_mode_idx, pm_info->is_early_wakeup);
	else
		exynos_wakeup_sys_powerdown(pm_info->suspend_mode_idx, pm_info->is_early_wakeup);

	exynos_show_wakeup_reason(pm_info->is_early_wakeup);

	if (!pm_info->is_early_wakeup)
		pr_debug("%s %s: post sleep, preparing to return\n",
						EXYNOS_PM_PREFIX, __func__);
}

static struct syscore_ops exynos_pm_syscore_ops = {
	.suspend	= exynos_pm_syscore_suspend,
	.resume		= exynos_pm_syscore_resume,
};

#ifdef CONFIG_SEC_GPIO_DVS
extern void gpio_dvs_check_sleepgpio(void);
#endif

static int exynos_pm_enter(suspend_state_t state)
{
	unsigned int psci_index;
	unsigned int prev_mif = 0, post_mif = 0;
	unsigned int prev_req, off_count;

	psci_index = pm_info->suspend_psci_idx;

	/* Send an IPI if test_early_wakeup flag is set */
	if (pm_dbg->test_early_wakeup)
		arch_send_call_function_single_ipi(0);

#ifdef CONFIG_SEC_GPIO_DVS
		/************************ Caution !!! ****************************/
		/* This function must be located in appropriate SLEEP position
		 * in accordance with the specification of each BB vendor.
		 */
		/************************ Caution !!! ****************************/
		gpio_dvs_check_sleepgpio();
#endif

	prev_mif = acpm_get_mifdn_count();
	prev_req = acpm_get_mif_request();

	exynos_pmu_read(CPU_INFORM4, &off_count);
	pr_info("%s prev mif_count %d %d\n",EXYNOS_PM_PREFIX, prev_mif, off_count);
	/* This will also act as our return point when
	 * we resume as it saves its own register state and restores it
	 * during the resume. */
	pm_info->is_early_wakeup = (bool)arm_cpuidle_suspend(psci_index);

	pr_info("%s is_early_wakeup: %d\n", EXYNOS_PM_PREFIX, pm_info->is_early_wakeup);
	if (pm_info->is_early_wakeup)
		pr_info("%s %s: return to originator\n",
				EXYNOS_PM_PREFIX, __func__);

	post_mif = acpm_get_mifdn_count();
	exynos_pmu_read(CPU_INFORM4, &off_count);
	pr_info("%s post mif_count %d	%d\n",EXYNOS_PM_PREFIX, post_mif,off_count);

	if (post_mif == prev_mif)
		pr_info("%s MIF blocked. MIF request Mster was 0x%x\n",
				EXYNOS_PM_PREFIX, prev_req);
	else
		pr_info("%s MIF down. cur_count: %d, acc_count: %d\n",
				EXYNOS_PM_PREFIX, post_mif - prev_mif, post_mif);

	return pm_info->is_early_wakeup;
}

static const struct platform_suspend_ops exynos_pm_ops = {
	.enter		= exynos_pm_enter,
	.valid		= suspend_valid_only_mem,
};

int register_usb_is_connect(bool (*func)(void))
{
	if(func) {
		pm_info->usb_is_connect = func;
		pr_info("Registered usb_is_connect func\n");
		return 0;
	} else {
		pr_err("%s	:function pointer is NULL \n", __func__);
		return -ENXIO;
	}
}
EXPORT_SYMBOL_GPL(register_usb_is_connect);

bool is_test_usbl2_suspend_set(void)
{
	if (!pm_dbg)
		return false;

	return pm_dbg->test_usbl2_suspend;
}
EXPORT_SYMBOL_GPL(is_test_usbl2_suspend_set);

#ifdef CONFIG_DEBUG_FS
static void __init exynos_pm_debugfs_init(void)
{
	struct dentry *root, *d;

	root = debugfs_create_dir("exynos-pm", NULL);
	if (!root) {
		pr_err("%s %s: could't create debugfs dir\n", EXYNOS_PM_PREFIX, __func__);
		return;
	}

	d = debugfs_create_u32("test_early_wakeup", 0644, root, &pm_dbg->test_early_wakeup);
	if (!d) {
		pr_err("%s %s: could't create debugfs test_early_wakeup\n",
					EXYNOS_PM_PREFIX, __func__);
		return;
	}

	d = debugfs_create_u32("test_usbl2_suspend", 0644, root, &pm_dbg->test_usbl2_suspend);
	if (!d) {
		pr_err("%s %s: could't create debugfs test_usbl2_suspend\n",
					EXYNOS_PM_PREFIX, __func__);
		return;
	}
}
#endif

enum acpm_dvfs_id {
	dvfs_mif = ACPM_VCLK_TYPE,
	dvfs_int,
	dvfs_cpucl0,
	dvfs_g3d,
	dvfs_cam,
	dvfs_disp,
	dvfs_aud,
	dvs_cp,
};

static ssize_t show_asv_cpu_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	/* {ASV Tbl ver} {CPU ASV (high)} {CPU ASV (mid)} {CPU ASV (low)} {CPU IDS} */
	return sprintf(buf, "%d 0 0 %d %d\n",
			cal_asv_get_tablever(),
			cal_asv_get_grp(dvfs_cpucl0),
			cal_asv_get_ids_info(dvfs_cpucl0));
}

static ssize_t show_asv_gpu_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* {GPU ASV (high)} {GPU ASV (mid)} {GPU ASV (low)} {GPU IDS} */
	return sprintf(buf, "0 0 %d 0\n",
			cal_asv_get_grp(dvfs_g3d));
}

static DEVICE_ATTR(asv_cpu_info, 0664, show_asv_cpu_info, NULL);
static DEVICE_ATTR(asv_gpu_info, 0664, show_asv_gpu_info, NULL);

static __init int exynos_pm_drvinit(void)
{
	int ret;

	pm_info = kzalloc(sizeof(struct exynos_pm_info), GFP_KERNEL);
	if (pm_info == NULL) {
		pr_err("%s %s: failed to allocate memory for exynos_pm_info\n",
					EXYNOS_PM_PREFIX, __func__);
		BUG();
	}

	pm_dbg = kzalloc(sizeof(struct exynos_pm_dbg), GFP_KERNEL);
	if (pm_dbg == NULL) {
		pr_err("%s %s: failed to allocate memory for exynos_pm_dbg\n",
					EXYNOS_PM_PREFIX, __func__);
		BUG();
	}

	if (of_have_populated_dt()) {
		struct device_node *np;
		np = of_find_compatible_node(NULL, NULL, "samsung,exynos-pm");
		if (!np) {
			pr_err("%s %s: unabled to find compatible node (%s)\n",
					EXYNOS_PM_PREFIX, __func__, "samsung,exynos-pm");
			BUG();
		}

		pm_info->eint_base = of_iomap(np, 0);
		if (!pm_info->eint_base) {
			pr_err("%s %s: unabled to ioremap EINT base address\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		pm_info->gic_base = of_iomap(np, 1);
		if (!pm_info->gic_base) {
			pr_err("%s %s: unbaled to ioremap GIC base address\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "num-eint", &pm_info->num_eint);
		if (ret) {
			pr_err("%s %s: unabled to get the number of eint from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "num-gic", &pm_info->num_gic);
		if (ret) {
			pr_err("%s %s: unabled to get the number of gic from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "suspend_mode_idx", &pm_info->suspend_mode_idx);
		if (ret) {
			pr_err("%s %s: unabled to get suspend_mode_idx from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "suspend_psci_idx", &pm_info->suspend_psci_idx);
		if (ret) {
			pr_err("%s %s: unabled to get suspend_psci_idx from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "usbl2_suspend_available", &pm_info->usbl2_suspend_available);
		if (ret) {
			pr_info("%s %s: Not support usbl2_suspend mode\n",
					EXYNOS_PM_PREFIX, __func__);
		} else {
			ret = of_property_read_u32(np, "usbl2_suspend_mode_idx", &pm_info->usbl2_suspend_mode_idx);
			if (ret) {
				pr_err("%s %s: unabled to get usbl2_suspend_mode_idx from DT\n",
						EXYNOS_PM_PREFIX, __func__);
				BUG();
			}
		}

		ret = of_property_count_u32_elems(np, "extra_wakeup_stat");
		if (!ret) {
			pr_err("%s %s: unabled to get wakeup_stat value from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		} else {
			pm_info->num_extra_stat = ret;
			pm_info->extra_wakeup_stat = kzalloc(sizeof(unsigned int) * ret, GFP_KERNEL);
			of_property_read_u32_array(np, "extra_wakeup_stat", pm_info->extra_wakeup_stat, ret);
		}
	} else {
		pr_err("%s %s: failed to have populated device tree\n",
					EXYNOS_PM_PREFIX, __func__);
		BUG();
	}

	suspend_set_ops(&exynos_pm_ops);
	register_syscore_ops(&exynos_pm_syscore_ops);
#ifdef CONFIG_DEBUG_FS
	exynos_pm_debugfs_init();
#endif

	ret = sysfs_create_file(power_kobj, &dev_attr_asv_cpu_info.attr);
	if (ret) {
		pr_err("%s: failed to create exynos9110 asv cpu attribute file\n", __func__);
	}

	ret = sysfs_create_file(power_kobj, &dev_attr_asv_gpu_info.attr);
	if (ret) {
		pr_err("%s: failed to create exynos9110 asv gpu attribute file\n", __func__);
	}

	return 0;
}
arch_initcall(exynos_pm_drvinit);
