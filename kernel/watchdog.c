/*
 * Detect hard and soft lockups on a system
 *
 * started by Don Zickus, Copyright (C) 2010 Red Hat, Inc.
 *
 * Note: Most of this code is borrowed heavily from the original softlockup
 * detector, so thanks to Ingo for the initial implementation.
 * Some chunks also taken from the old x86-specific nmi watchdog code, thanks
 * to those contributors as well.
 */

#define pr_fmt(fmt) "NMI watchdog: " fmt

#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/smpboot.h>
#include <linux/sched/rt.h>
#include <linux/tick.h>
#include <linux/workqueue.h>
#include <linux/exynos-ss.h>

#include <asm/irq_regs.h>
#include <linux/kvm_para.h>
#include <linux/kthread.h>
#ifdef CONFIG_SEC_DEBUG
#include <linux/sec_debug.h>
#endif

static DEFINE_MUTEX(watchdog_proc_mutex);

#if defined(CONFIG_HAVE_NMI_WATCHDOG) || defined(CONFIG_HARDLOCKUP_DETECTOR) \
	|| defined(CONFIG_HARDLOCKUP_DETECTOR_OTHER_CPU)
unsigned long __read_mostly watchdog_enabled = SOFT_WATCHDOG_ENABLED|NMI_WATCHDOG_ENABLED;
#else
unsigned long __read_mostly watchdog_enabled = SOFT_WATCHDOG_ENABLED;
#endif
int __read_mostly nmi_watchdog_enabled;
int __read_mostly soft_watchdog_enabled;
int __read_mostly watchdog_user_enabled;
int __read_mostly watchdog_thresh = 10;

#ifdef CONFIG_SMP
int __read_mostly sysctl_softlockup_all_cpu_backtrace;
int __read_mostly sysctl_hardlockup_all_cpu_backtrace;
#endif
static struct cpumask watchdog_cpumask __read_mostly;
unsigned long *watchdog_cpumask_bits = cpumask_bits(&watchdog_cpumask);

/* Helper for online, unparked cpus. */
#define for_each_watchdog_cpu(cpu) \
	for_each_cpu_and((cpu), cpu_online_mask, &watchdog_cpumask)

atomic_t watchdog_park_in_progress = ATOMIC_INIT(0);

/*
 * The 'watchdog_running' variable is set to 1 when the watchdog threads
 * are registered/started and is set to 0 when the watchdog threads are
 * unregistered/stopped, so it is an indicator whether the threads exist.
 */
static int __read_mostly watchdog_running;
/*
 * If a subsystem has a need to deactivate the watchdog temporarily, it
 * can use the suspend/resume interface to achieve this. The content of
 * the 'watchdog_suspended' variable reflects this state. Existing threads
 * are parked/unparked by the lockup_detector_{suspend|resume} functions
 * (see comment blocks pertaining to those functions for further details).
 *
 * 'watchdog_suspended' also prevents threads from being registered/started
 * or unregistered/stopped via parameters in /proc/sys/kernel, so the state
 * of 'watchdog_running' cannot change while the watchdog is deactivated
 * temporarily (see related code in 'proc' handlers).
 */
static int __read_mostly watchdog_suspended;

static u64 __read_mostly sample_period;
static unsigned long __read_mostly hardlockup_thresh;

static DEFINE_PER_CPU(unsigned long, watchdog_touch_ts);
static DEFINE_PER_CPU(unsigned long, hardlockup_touch_ts);
static DEFINE_PER_CPU(struct task_struct *, softlockup_watchdog);
static DEFINE_PER_CPU(struct hrtimer, watchdog_hrtimer);
static DEFINE_PER_CPU(bool, softlockup_touch_sync);
static DEFINE_PER_CPU(bool, soft_watchdog_warn);
static DEFINE_PER_CPU(unsigned long, hrtimer_interrupts);
static DEFINE_PER_CPU(unsigned long, soft_lockup_hrtimer_cnt);
static DEFINE_PER_CPU(struct task_struct *, softlockup_task_ptr_saved);
static DEFINE_PER_CPU(unsigned long, hrtimer_interrupts_saved);

static unsigned long soft_lockup_nmi_warn;

unsigned int __read_mostly softlockup_panic =
			CONFIG_BOOTPARAM_SOFTLOCKUP_PANIC_VALUE;

static int __init softlockup_panic_setup(char *str)
{
	softlockup_panic = simple_strtoul(str, NULL, 0);

	return 1;
}
__setup("softlockup_panic=", softlockup_panic_setup);

static int __init nowatchdog_setup(char *str)
{
	watchdog_enabled = 0;
	return 1;
}
__setup("nowatchdog", nowatchdog_setup);

static int __init nosoftlockup_setup(char *str)
{
	watchdog_enabled &= ~SOFT_WATCHDOG_ENABLED;
	return 1;
}
__setup("nosoftlockup", nosoftlockup_setup);

#ifdef CONFIG_SMP
static int __init softlockup_all_cpu_backtrace_setup(char *str)
{
	sysctl_softlockup_all_cpu_backtrace =
		!!simple_strtol(str, NULL, 0);
	return 1;
}
__setup("softlockup_all_cpu_backtrace=", softlockup_all_cpu_backtrace_setup);
static int __init hardlockup_all_cpu_backtrace_setup(char *str)
{
	sysctl_hardlockup_all_cpu_backtrace =
		!!simple_strtol(str, NULL, 0);
	return 1;
}
__setup("hardlockup_all_cpu_backtrace=", hardlockup_all_cpu_backtrace_setup);
#endif

/*
 * Hard-lockup warnings should be triggered after just a few seconds. Soft-
 * lockups can have false positives under extreme conditions. So we generally
 * want a higher threshold for soft lockups than for hard lockups. So we couple
 * the thresholds with a factor: we make the soft threshold twice the amount of
 * time the hard threshold is.
 */
static int get_softlockup_thresh(void)
{
	return watchdog_thresh * 2;
}

/*
 * Returns seconds, approximately.  We don't need nanosecond
 * resolution, and we don't need to waste time with a big divide when
 * 2^30ns == 1.074s.
 */
static unsigned long get_timestamp(void)
{
	return running_clock() >> 30LL;  /* 2^30 ~= 10^9 */
}

static void set_sample_period(void)
{
	/*
	 * convert watchdog_thresh from seconds to ns
	 * the divide by 5 is to give hrtimer several chances (two
	 * or three with the current relation between the soft
	 * and hard thresholds) to increment before the
	 * hardlockup detector generates a warning
	 */
	sample_period = get_softlockup_thresh() * ((u64)NSEC_PER_SEC / 5);
	hardlockup_thresh = sample_period * 3 / NSEC_PER_SEC;
}

/* Commands for resetting the watchdog */
static void __touch_watchdog(void)
{
	__this_cpu_write(watchdog_touch_ts, get_timestamp());
	__this_cpu_write(hardlockup_touch_ts, get_timestamp());
}

/**
 * touch_softlockup_watchdog_sched - touch watchdog on scheduler stalls
 *
 * Call when the scheduler may have stalled for legitimate reasons
 * preventing the watchdog task from executing - e.g. the scheduler
 * entering idle state.  This should only be used for scheduler events.
 * Use touch_softlockup_watchdog() for everything else.
 */
void touch_softlockup_watchdog_sched(void)
{
	/*
	 * Preemption can be enabled.  It doesn't matter which CPU's timestamp
	 * gets zeroed here, so use the raw_ operation.
	 */
	raw_cpu_write(watchdog_touch_ts, 0);
}

void touch_softlockup_watchdog(void)
{
	touch_softlockup_watchdog_sched();
	wq_watchdog_touch(raw_smp_processor_id());
}
EXPORT_SYMBOL(touch_softlockup_watchdog);

void touch_all_softlockup_watchdogs(void)
{
	int cpu;

	/*
	 * this is done lockless
	 * do we care if a 0 races with a timestamp?
	 * all it means is the softlock check starts one cycle later
	 */
	for_each_watchdog_cpu(cpu)
		per_cpu(watchdog_touch_ts, cpu) = 0;
	wq_watchdog_touch(-1);
}

void touch_softlockup_watchdog_sync(void)
{
	__this_cpu_write(softlockup_touch_sync, true);
	__this_cpu_write(watchdog_touch_ts, 0);
}

#ifdef CONFIG_HARDLOCKUP_DETECTOR_OTHER_CPU
static void watchdog_check_hardlockup_other_cpu(void);
#else
static inline void watchdog_check_hardlockup_other_cpu(void) { return; }
#endif

static int is_softlockup(unsigned long touch_ts)
{
	unsigned long now = get_timestamp();

	if ((watchdog_enabled & SOFT_WATCHDOG_ENABLED) && watchdog_thresh){
		/* Warn about unreasonable delays. */
		if (time_after(now, touch_ts + get_softlockup_thresh()))
			return now - touch_ts;
	}
	return 0;
}

/* watchdog detector functions */
bool is_hardlockup(void)
{
	unsigned long hrint = __this_cpu_read(hrtimer_interrupts);

	if (__this_cpu_read(hrtimer_interrupts_saved) == hrint)
		return true;

	__this_cpu_write(hrtimer_interrupts_saved, hrint);
	return false;
}

static void watchdog_interrupt_count(void)
{
	__this_cpu_inc(hrtimer_interrupts);
}

#ifdef CONFIG_HARDLOCKUP_DETECTOR_OTHER_CPU
static int watchdog_nmi_enable(unsigned int cpu);
static void watchdog_nmi_disable(unsigned int cpu);
#else
/*
 * These two functions are mostly architecture specific
 * defining them as weak here.
 */
int __weak watchdog_nmi_enable(unsigned int cpu)
{
	return 0;
}
void __weak watchdog_nmi_disable(unsigned int cpu)
{
}
#endif
static int watchdog_enable_all_cpus(void);
static void watchdog_disable_all_cpus(void);

/* watchdog kicker functions */
static enum hrtimer_restart watchdog_timer_fn(struct hrtimer *hrtimer)
{
	unsigned long touch_ts = __this_cpu_read(watchdog_touch_ts);
	struct pt_regs *regs = get_irq_regs();
	int duration;
	int softlockup_all_cpu_backtrace = sysctl_softlockup_all_cpu_backtrace;

	/* try to enable log_kevent of exynos-snapshot if log_kevent was off because of rcu stall */
	exynos_ss_try_enable("log_kevent", NSEC_PER_SEC * 15);
	if (atomic_read(&watchdog_park_in_progress) != 0)
		return HRTIMER_NORESTART;

	/* kick the hardlockup detector */
	watchdog_interrupt_count();

	/* test for hardlockups on the next cpu */
	watchdog_check_hardlockup_other_cpu();

	/* kick the softlockup detector */
	wake_up_process(__this_cpu_read(softlockup_watchdog));

	/* .. and repeat */
	hrtimer_forward_now(hrtimer, ns_to_ktime(sample_period));

	if (touch_ts == 0) {
		if (unlikely(__this_cpu_read(softlockup_touch_sync))) {
			/*
			 * If the time stamp was touched atomically
			 * make sure the scheduler tick is up to date.
			 */
			__this_cpu_write(softlockup_touch_sync, false);
			sched_clock_tick();
		}

		/* Clear the guest paused flag on watchdog reset */
		kvm_check_and_clear_guest_paused();
		__touch_watchdog();
		return HRTIMER_RESTART;
	}

	/* check for a softlockup
	 * This is done by making sure a high priority task is
	 * being scheduled.  The task touches the watchdog to
	 * indicate it is getting cpu time.  If it hasn't then
	 * this is a good indication some task is hogging the cpu
	 */
	duration = is_softlockup(touch_ts);
	if (unlikely(duration)) {
		/*
		 * If a virtual machine is stopped by the host it can look to
		 * the watchdog like a soft lockup, check to see if the host
		 * stopped the vm before we issue the warning
		 */
		if (kvm_check_and_clear_guest_paused())
			return HRTIMER_RESTART;

		/* only warn once */
		if (__this_cpu_read(soft_watchdog_warn) == true) {
			/*
			 * When multiple processes are causing softlockups the
			 * softlockup detector only warns on the first one
			 * because the code relies on a full quiet cycle to
			 * re-arm.  The second process prevents the quiet cycle
			 * and never gets reported.  Use task pointers to detect
			 * this.
			 */
			if (__this_cpu_read(softlockup_task_ptr_saved) !=
			    current) {
				__this_cpu_write(soft_watchdog_warn, false);
				__touch_watchdog();
			}
			return HRTIMER_RESTART;
		}

		if (softlockup_all_cpu_backtrace) {
			/* Prevent multiple soft-lockup reports if one cpu is already
			 * engaged in dumping cpu back traces
			 */
			if (test_and_set_bit(0, &soft_lockup_nmi_warn)) {
				/* Someone else will report us. Let's give up */
				__this_cpu_write(soft_watchdog_warn, true);
				return HRTIMER_RESTART;
			}
		}

		pr_emerg("BUG: soft lockup - CPU#%d stuck for %us! [%s:%d]\n",
			smp_processor_id(), duration,
			current->comm, task_pid_nr(current));
		__this_cpu_write(softlockup_task_ptr_saved, current);
		print_modules();
		print_irqtrace_events(current);
		if (regs)
			show_regs(regs);
		else
			dump_stack();

		if (softlockup_all_cpu_backtrace) {
			/* Avoid generating two back traces for current
			 * given that one is already made above
			 */
			trigger_allbutself_cpu_backtrace();

			clear_bit(0, &soft_lockup_nmi_warn);
			/* Barrier to sync with other cpus */
			smp_mb__after_atomic();
		}

		add_taint(TAINT_SOFTLOCKUP, LOCKDEP_STILL_OK);
		if (softlockup_panic) {
#ifdef CONFIG_SEC_DEBUG_EXTRA_INFO
			if (regs) {
				sec_debug_set_extra_info_fault(WATCHDOG_FAULT, (unsigned long)regs->pc, regs);
				sec_debug_set_extra_info_backtrace(regs);
				sec_debug_set_extra_info_backtrace_cpu(regs, smp_processor_id());
			}
#endif
			panic("softlockup: hung tasks");
		}
		__this_cpu_write(soft_watchdog_warn, true);
	} else
		__this_cpu_write(soft_watchdog_warn, false);

	return HRTIMER_RESTART;
}

static void watchdog_set_prio(unsigned int policy, unsigned int prio)
{
	struct sched_param param = { .sched_priority = prio };

	sched_setscheduler(current, policy, &param);
}

static void watchdog_enable(unsigned int cpu)
{
	struct hrtimer *hrtimer = raw_cpu_ptr(&watchdog_hrtimer);

	/* kick off the timer for the hardlockup detector */
	hrtimer_init(hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer->function = watchdog_timer_fn;

	/* Enable the perf event */
	watchdog_nmi_enable(cpu);

	/* done here because hrtimer_start can only pin to smp_processor_id() */
	hrtimer_start(hrtimer, ns_to_ktime(sample_period),
		      HRTIMER_MODE_REL_PINNED);

	/* initialize timestamp */
	watchdog_set_prio(SCHED_FIFO, MAX_RT_PRIO - 1);
	__touch_watchdog();
}

static void watchdog_disable(unsigned int cpu)
{
	struct hrtimer *hrtimer = raw_cpu_ptr(&watchdog_hrtimer);

	watchdog_set_prio(SCHED_NORMAL, 0);
	hrtimer_cancel(hrtimer);
	/* disable the perf event */
	watchdog_nmi_disable(cpu);
}

static void watchdog_cleanup(unsigned int cpu, bool online)
{
	watchdog_disable(cpu);
}

static int watchdog_should_run(unsigned int cpu)
{
	return __this_cpu_read(hrtimer_interrupts) !=
		__this_cpu_read(soft_lockup_hrtimer_cnt);
}

/*
 * The watchdog thread function - touches the timestamp.
 *
 * It only runs once every sample_period seconds (4 seconds by
 * default) to reset the softlockup timestamp. If this gets delayed
 * for more than 2*watchdog_thresh seconds then the debug-printout
 * triggers in watchdog_timer_fn().
 */
static void watchdog(unsigned int cpu)
{
	__this_cpu_write(soft_lockup_hrtimer_cnt,
			 __this_cpu_read(hrtimer_interrupts));
	__touch_watchdog();

	/*
	 * watchdog_nmi_enable() clears the NMI_WATCHDOG_ENABLED bit in the
	 * failure path. Check for failures that can occur asynchronously -
	 * for example, when CPUs are on-lined - and shut down the hardware
	 * perf event on each CPU accordingly.
	 *
	 * The only non-obvious place this bit can be cleared is through
	 * watchdog_nmi_enable(), so a pr_info() is placed there.  Placing a
	 * pr_info here would be too noisy as it would result in a message
	 * every few seconds if the hardlockup was disabled but the softlockup
	 * enabled.
	 */
	if (!(watchdog_enabled & NMI_WATCHDOG_ENABLED))
		watchdog_nmi_disable(cpu);
}

static struct smp_hotplug_thread watchdog_threads = {
	.store			= &softlockup_watchdog,
	.thread_should_run	= watchdog_should_run,
	.thread_fn		= watchdog,
	.thread_comm		= "watchdog/%u",
	.setup			= watchdog_enable,
	.cleanup		= watchdog_cleanup,
	.park			= watchdog_disable,
	.unpark			= watchdog_enable,
};

/*
 * park all watchdog threads that are specified in 'watchdog_cpumask'
 *
 * This function returns an error if kthread_park() of a watchdog thread
 * fails. In this situation, the watchdog threads of some CPUs can already
 * be parked and the watchdog threads of other CPUs can still be runnable.
 * Callers are expected to handle this special condition as appropriate in
 * their context.
 *
 * This function may only be called in a context that is protected against
 * races with CPU hotplug - for example, via get_online_cpus().
 */
static int watchdog_park_threads(void)
{
	int cpu, ret = 0;

	atomic_set(&watchdog_park_in_progress, 1);

	for_each_watchdog_cpu(cpu) {
		ret = kthread_park(per_cpu(softlockup_watchdog, cpu));
		if (ret)
			break;
	}

	atomic_set(&watchdog_park_in_progress, 0);

	return ret;
}

/*
 * unpark all watchdog threads that are specified in 'watchdog_cpumask'
 *
 * This function may only be called in a context that is protected against
 * races with CPU hotplug - for example, via get_online_cpus().
 */
static void watchdog_unpark_threads(void)
{
	int cpu;

	for_each_watchdog_cpu(cpu)
		kthread_unpark(per_cpu(softlockup_watchdog, cpu));
}

/*
 * Suspend the hard and soft lockup detector by parking the watchdog threads.
 */
int lockup_detector_suspend(void)
{
	int ret = 0;

	get_online_cpus();
	mutex_lock(&watchdog_proc_mutex);
	/*
	 * Multiple suspend requests can be active in parallel (counted by
	 * the 'watchdog_suspended' variable). If the watchdog threads are
	 * running, the first caller takes care that they will be parked.
	 * The state of 'watchdog_running' cannot change while a suspend
	 * request is active (see related code in 'proc' handlers).
	 */
	if (watchdog_running && !watchdog_suspended)
		ret = watchdog_park_threads();

	if (ret == 0)
		watchdog_suspended++;
	else {
		watchdog_disable_all_cpus();
		pr_err("Failed to suspend lockup detectors, disabled\n");
		watchdog_enabled = 0;
	}

	mutex_unlock(&watchdog_proc_mutex);

	return ret;
}

/*
 * Resume the hard and soft lockup detector by unparking the watchdog threads.
 */
void lockup_detector_resume(void)
{
	mutex_lock(&watchdog_proc_mutex);

	watchdog_suspended--;
	/*
	 * The watchdog threads are unparked if they were previously running
	 * and if there is no more active suspend request.
	 */
	if (watchdog_running && !watchdog_suspended)
		watchdog_unpark_threads();

	mutex_unlock(&watchdog_proc_mutex);
	put_online_cpus();
}

static int update_watchdog_all_cpus(void)
{
	int ret;

	ret = watchdog_park_threads();
	if (ret)
		return ret;

	watchdog_unpark_threads();

	return 0;
}

static int watchdog_enable_all_cpus(void)
{
	int err = 0;

	if (!watchdog_running) {
		err = smpboot_register_percpu_thread_cpumask(&watchdog_threads,
							     &watchdog_cpumask);
		if (err)
			pr_err("Failed to create watchdog threads, disabled\n");
		else
			watchdog_running = 1;
	} else {
		/*
		 * Enable/disable the lockup detectors or
		 * change the sample period 'on the fly'.
		 */
		err = update_watchdog_all_cpus();

		if (err) {
			watchdog_disable_all_cpus();
			pr_err("Failed to update lockup detectors, disabled\n");
		}
	}

	if (err)
		watchdog_enabled = 0;

	return err;
}

static void watchdog_disable_all_cpus(void)
{
	if (watchdog_running) {
		watchdog_running = 0;
		smpboot_unregister_percpu_thread(&watchdog_threads);
	}
}

#ifdef CONFIG_SYSCTL

/*
 * Update the run state of the lockup detectors.
 */
static int proc_watchdog_update(void)
{
	int err = 0;

	/*
	 * Watchdog threads won't be started if they are already active.
	 * The 'watchdog_running' variable in watchdog_*_all_cpus() takes
	 * care of this. If those threads are already active, the sample
	 * period will be updated and the lockup detectors will be enabled
	 * or disabled 'on the fly'.
	 */
	if (watchdog_enabled && watchdog_thresh)
		err = watchdog_enable_all_cpus();
	else
		watchdog_disable_all_cpus();

	return err;

}

/*
 * common function for watchdog, nmi_watchdog and soft_watchdog parameter
 *
 * caller             | table->data points to | 'which' contains the flag(s)
 * -------------------|-----------------------|-----------------------------
 * proc_watchdog      | watchdog_user_enabled | NMI_WATCHDOG_ENABLED or'ed
 *                    |                       | with SOFT_WATCHDOG_ENABLED
 * -------------------|-----------------------|-----------------------------
 * proc_nmi_watchdog  | nmi_watchdog_enabled  | NMI_WATCHDOG_ENABLED
 * -------------------|-----------------------|-----------------------------
 * proc_soft_watchdog | soft_watchdog_enabled | SOFT_WATCHDOG_ENABLED
 */
static int proc_watchdog_common(int which, struct ctl_table *table, int write,
				void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int err, old, new;
	int *watchdog_param = (int *)table->data;

	get_online_cpus();
	mutex_lock(&watchdog_proc_mutex);

	if (watchdog_suspended) {
		/* no parameter changes allowed while watchdog is suspended */
		err = -EAGAIN;
		goto out;
	}

	/*
	 * If the parameter is being read return the state of the corresponding
	 * bit(s) in 'watchdog_enabled', else update 'watchdog_enabled' and the
	 * run state of the lockup detectors.
	 */
	if (!write) {
		*watchdog_param = (watchdog_enabled & which) != 0;
		err = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	} else {
		err = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
		if (err)
			goto out;

		/*
		 * There is a race window between fetching the current value
		 * from 'watchdog_enabled' and storing the new value. During
		 * this race window, watchdog_nmi_enable() can sneak in and
		 * clear the NMI_WATCHDOG_ENABLED bit in 'watchdog_enabled'.
		 * The 'cmpxchg' detects this race and the loop retries.
		 */
		do {
			old = watchdog_enabled;
			/*
			 * If the parameter value is not zero set the
			 * corresponding bit(s), else clear it(them).
			 */
			if (*watchdog_param)
				new = old | which;
			else
				new = old & ~which;
		} while (cmpxchg(&watchdog_enabled, old, new) != old);

		/*
		 * Update the run state of the lockup detectors. There is _no_
		 * need to check the value returned by proc_watchdog_update()
		 * and to restore the previous value of 'watchdog_enabled' as
		 * both lockup detectors are disabled if proc_watchdog_update()
		 * returns an error.
		 */
		if (old == new)
			goto out;

		err = proc_watchdog_update();
	}
out:
	mutex_unlock(&watchdog_proc_mutex);
	put_online_cpus();
	return err;
}

/*
 * /proc/sys/kernel/watchdog
 */
int proc_watchdog(struct ctl_table *table, int write,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return proc_watchdog_common(NMI_WATCHDOG_ENABLED|SOFT_WATCHDOG_ENABLED,
				    table, write, buffer, lenp, ppos);
}

/*
 * /proc/sys/kernel/nmi_watchdog
 */
int proc_nmi_watchdog(struct ctl_table *table, int write,
		      void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return proc_watchdog_common(NMI_WATCHDOG_ENABLED,
				    table, write, buffer, lenp, ppos);
}

/*
 * /proc/sys/kernel/soft_watchdog
 */
int proc_soft_watchdog(struct ctl_table *table, int write,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return proc_watchdog_common(SOFT_WATCHDOG_ENABLED,
				    table, write, buffer, lenp, ppos);
}

/*
 * /proc/sys/kernel/watchdog_thresh
 */
int proc_watchdog_thresh(struct ctl_table *table, int write,
			 void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int err, old, new;

	get_online_cpus();
	mutex_lock(&watchdog_proc_mutex);

	if (watchdog_suspended) {
		/* no parameter changes allowed while watchdog is suspended */
		err = -EAGAIN;
		goto out;
	}

	old = ACCESS_ONCE(watchdog_thresh);
	err = proc_dointvec_minmax(table, write, buffer, lenp, ppos);

	if (err || !write)
		goto out;

	/*
	 * Update the sample period. Restore on failure.
	 */
	new = ACCESS_ONCE(watchdog_thresh);
	if (old == new)
		goto out;

	set_sample_period();
	err = proc_watchdog_update();
	if (err) {
		watchdog_thresh = old;
		set_sample_period();
	}
out:
	mutex_unlock(&watchdog_proc_mutex);
	put_online_cpus();
	return err;
}

/*
 * The cpumask is the mask of possible cpus that the watchdog can run
 * on, not the mask of cpus it is actually running on.  This allows the
 * user to specify a mask that will include cpus that have not yet
 * been brought online, if desired.
 */
int proc_watchdog_cpumask(struct ctl_table *table, int write,
			  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int err;

	get_online_cpus();
	mutex_lock(&watchdog_proc_mutex);

	if (watchdog_suspended) {
		/* no parameter changes allowed while watchdog is suspended */
		err = -EAGAIN;
		goto out;
	}

	err = proc_do_large_bitmap(table, write, buffer, lenp, ppos);
	if (!err && write) {
		/* Remove impossible cpus to keep sysctl output cleaner. */
		cpumask_and(&watchdog_cpumask, &watchdog_cpumask,
			    cpu_possible_mask);

		if (watchdog_running) {
			/*
			 * Failure would be due to being unable to allocate
			 * a temporary cpumask, so we are likely not in a
			 * position to do much else to make things better.
			 */
			if (smpboot_update_cpumask_percpu_thread(
				    &watchdog_threads, &watchdog_cpumask) != 0)
				pr_err("cpumask update failed\n");
		}
	}
out:
	mutex_unlock(&watchdog_proc_mutex);
	put_online_cpus();
	return err;
}

#endif /* CONFIG_SYSCTL */

void __init lockup_detector_init(void)
{
	set_sample_period();

#ifdef CONFIG_NO_HZ_FULL
	if (tick_nohz_full_enabled()) {
		pr_info("Disabling watchdog on nohz_full cores by default\n");
		cpumask_copy(&watchdog_cpumask, housekeeping_mask);
	} else
		cpumask_copy(&watchdog_cpumask, cpu_possible_mask);
#else
	cpumask_copy(&watchdog_cpumask, cpu_possible_mask);
#endif

	if (watchdog_enabled)
		watchdog_enable_all_cpus();
}

#ifdef CONFIG_HARDLOCKUP_DETECTOR_OTHER_CPU
static DEFINE_PER_CPU(bool, hard_watchdog_warn);
static DEFINE_PER_CPU(bool, watchdog_nmi_touch);
static cpumask_t __read_mostly watchdog_cpus;
ATOMIC_NOTIFIER_HEAD(hardlockup_notifier_list);
EXPORT_SYMBOL(hardlockup_notifier_list);

/* boot commands */
/*
 * Should we panic when a soft-lockup or hard-lockup occurs:
 */
unsigned int __read_mostly hardlockup_panic =
			CONFIG_BOOTPARAM_HARDLOCKUP_PANIC_VALUE;
/*
 * We may not want to enable hard lockup detection by default in all cases,
 * for example when running the kernel as a guest on a hypervisor. In these
 * cases this function can be called to disable hard lockup detection. This
 * function should only be executed once by the boot processor before the
 * kernel command line parameters are parsed, because otherwise it is not
 * possible to override this in hardlockup_panic_setup().
 */
void hardlockup_detector_disable(void)
{
	watchdog_enabled &= ~NMI_WATCHDOG_ENABLED;
}

static int __init hardlockup_panic_setup(char *str)
{
	if (!strncmp(str, "panic", 5))
		hardlockup_panic = 1;
	else if (!strncmp(str, "nopanic", 7))
		hardlockup_panic = 0;
	else if (!strncmp(str, "0", 1))
		watchdog_enabled &= ~NMI_WATCHDOG_ENABLED;
	else if (!strncmp(str, "1", 1))
		watchdog_enabled |= NMI_WATCHDOG_ENABLED;
	return 1;
}
__setup("nmi_watchdog=", hardlockup_panic_setup);

static unsigned int watchdog_next_cpu(unsigned int cpu)
{
	cpumask_t cpus = watchdog_cpus;
	unsigned int next_cpu;

	next_cpu = cpumask_next(cpu, &cpus);
	if (next_cpu >= nr_cpu_ids)
		next_cpu = cpumask_first(&cpus);

	if (next_cpu == cpu)
		return nr_cpu_ids;

	return next_cpu;
}

static int is_hardlockup_other_cpu(unsigned int cpu)
{
	unsigned long hrint = per_cpu(hrtimer_interrupts, cpu);

	if (per_cpu(hrtimer_interrupts_saved, cpu) == hrint) {
		unsigned long now = get_timestamp();
		unsigned long touch_ts = per_cpu(hardlockup_touch_ts, cpu);

		if (time_after(now, touch_ts) &&
				(now - touch_ts >= hardlockup_thresh))
			return 1;
	}

	per_cpu(hrtimer_interrupts_saved, cpu) = hrint;
	return 0;
}

static void watchdog_check_hardlockup_other_cpu(void)
{
	unsigned int next_cpu;

	/*
	 * Test for hardlockups every 3 samples.  The sample period is
	 *  watchdog_thresh * 2 / 5, so 3 samples gets us back to slightly over
	 *  watchdog_thresh (over by 20%).
	 */
	if (__this_cpu_read(hrtimer_interrupts) % 3 != 0)
		return;

	/* check for a hardlockup on the next cpu */
	next_cpu = watchdog_next_cpu(smp_processor_id());
	if (next_cpu >= nr_cpu_ids)
		return;

	smp_rmb();

	if (per_cpu(watchdog_nmi_touch, next_cpu) == true) {
		per_cpu(watchdog_nmi_touch, next_cpu) = false;
		return;
	}

	if (is_hardlockup_other_cpu(next_cpu)) {
		/* only warn once */
		if (per_cpu(hard_watchdog_warn, next_cpu) == true)
			return;

		if (hardlockup_panic) {
			exynos_ss_set_hardlockup(hardlockup_panic);
			atomic_notifier_call_chain(&hardlockup_notifier_list, 0, (void *)&next_cpu);
			panic("Watchdog detected hard LOCKUP on cpu %u", next_cpu);
		} else {
			WARN(1, "Watchdog detected hard LOCKUP on cpu %u", next_cpu);
		}

		per_cpu(hard_watchdog_warn, next_cpu) = true;
	} else {
		per_cpu(hard_watchdog_warn, next_cpu) = false;
	}
}

void touch_nmi_watchdog(void)
{
	/*
	 * Using __raw here because some code paths have
	 * preemption enabled.  If preemption is enabled
	 * then interrupts should be enabled too, in which
	 * case we shouldn't have to worry about the watchdog
	 * going off.
	 */
	raw_cpu_write(watchdog_nmi_touch, true);
	touch_softlockup_watchdog();
}
EXPORT_SYMBOL(touch_nmi_watchdog);

static int watchdog_nmi_enable(unsigned int cpu)
{
	/*
	 * The new cpu will be marked online before the first hrtimer interrupt
	 * runs on it.  If another cpu tests for a hardlockup on the new cpu
	 * before it has run its first hrtimer, it will get a false positive.
	 * Touch the watchdog on the new cpu to delay the first check for at
	 * least 3 sampling periods to guarantee one hrtimer has run on the new
	 * cpu.
	 */
	per_cpu(watchdog_nmi_touch, cpu) = true;
	smp_wmb();
	cpumask_set_cpu(cpu, &watchdog_cpus);
	return 0;
}

static void watchdog_nmi_disable(unsigned int cpu)
{
	unsigned int next_cpu = watchdog_next_cpu(cpu);

	/*
	 * Offlining this cpu will cause the cpu before this one to start
	 * checking the one after this one.  If this cpu just finished checking
	 * the next cpu and updating hrtimer_interrupts_saved, and then the
	 * previous cpu checks it within one sample period, it will trigger a
	 * false positive.  Touch the watchdog on the next cpu to prevent it.
	 */
	if (next_cpu < nr_cpu_ids)
		per_cpu(watchdog_nmi_touch, next_cpu) = true;
	smp_wmb();
	cpumask_clear_cpu(cpu, &watchdog_cpus);
}
#endif
