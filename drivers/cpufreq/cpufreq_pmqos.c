#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <soc/samsung/exynos-powermode.h>

#include <linux/trm.h>
#if defined(CONFIG_SYSTEM_LOAD_ANALYZER)
#include <linux/load_analyzer.h>
#endif

#if defined(CONFIG_KSWAPD_WAKEUP_DEBUGS)
extern unsigned long kswapd_count;
extern unsigned long run_time;
extern unsigned long last_wake_time;
#endif

#if defined(CONFIG_GEM_ALLOC_DEBUGS)
extern char gem_alloc_buf[];
#endif

extern unsigned int period_check_idle;

#if defined(CONFIG_SLP_BUSY_LEVEL)
#define PERIOD_IDLE_CHECK period_check_idle
extern unsigned int get_cpu_idle_state(void);
static unsigned int is_idle = 0;
static unsigned int wrist_up = 0;
extern int update_time_in_state_diff(void);
extern struct time_in_state_busy_level_diff time_in_state_diff[MAX_CPU_STATE];
#endif


#define define_one_root_rw(_name)		\
static struct global_attr _name =		\
__ATTR(_name, 0600, show_##_name, store_##_name)

#define __ATTR_ROOT_RO(_name) { \
	.attr	= { .name = __stringify(_name), .mode = 0400 },	\
	.show	= show_##_name,					\
}

#define define_one_root_ro(_name)		\
static struct global_attr _name =		\
__ATTR_ROOT_RO(_name)

static int atoi(const char *str)
{
	int result = 0;
	int count = 0;
	if (str == NULL)
		return -1;
	while (str[count] && str[count] >= '0' && str[count] <= '9') {
		result = result * 10 + str[count] - '0';
		++count;
	}
	return result;
}

#include "cpufreq_pmqos_galileo.c"
#define DEVICE_NAME "GALILEO"

#if defined(TRM_INPUT_BOOSTER_EN)
#include "cpufreq_pmqos_input.c"
#endif

struct kobject *cpufreq_pmqos_kobject;
static DEFINE_MUTEX(pmqos_kobject_mutex);

struct pm_qos_lock_tag {
	int cpufreq_min_value;
	int cpufreq_max_value;
	int cpu_online_min_value;
	int cpu_online_max_value;
};

int min_cpu_freq, max_cpu_freq;

/* CPUFREQ */
static struct pm_qos_request cpufreq_min_qos_array[NUMBER_OF_LOCK];
static struct pm_qos_request cpufreq_max_qos_array[NUMBER_OF_LOCK];

#if defined(BUILD_ERROR)
static struct pm_qos_request cpu_gov_up_level_array[NUMBER_OF_LOCK];
static struct pm_qos_request cpu_freq_up_threshold_array[NUMBER_OF_LOCK];
#endif

/* CPU ONLINE */
static struct pm_qos_request cpu_online_min_qos_array[NUMBER_OF_LOCK];
static struct pm_qos_request cpu_online_max_qos_array[NUMBER_OF_LOCK];

/* BUS */
static struct pm_qos_request bus_mif_min_qos_array[NUMBER_OF_LOCK];
static struct pm_qos_request bus_mif_max_qos_array[NUMBER_OF_LOCK];

static struct pm_qos_request bus_int_min_qos_array[NUMBER_OF_LOCK];

/* CHECK LOCK STATE */
static struct pm_qos_lock_tag trm_pm_qos_lock[NUMBER_OF_LOCK];

static void pmqos_saving_data(int pmqos_type, int id, int value)
{
	switch (pmqos_type) {

	case PM_QOS_CLUSTER0_FREQ_MIN:
		trm_pm_qos_lock[id].cpufreq_min_value = value;
		break;

	case PM_QOS_CLUSTER0_FREQ_MAX:
		trm_pm_qos_lock[id].cpufreq_max_value = value;
		break;

	case PM_QOS_CPU_ONLINE_MIN:
		trm_pm_qos_lock[id].cpu_online_min_value = value;
		break;

	case PM_QOS_CPU_ONLINE_MAX:
		trm_pm_qos_lock[id].cpu_online_max_value = value;
		break;
	}
}

int set_pmqos_data(struct pm_qos_request *any_qos_array, int pmqos_type, const char *buf)
{
	int lock_id = KERNEL_RESERVED00, lock_value = 0;
	char *p2 = NULL;

	p2 = strstr(buf, "ID");

	if (p2 == NULL)
		p2 = strstr(buf, "id");

	if (p2 != NULL)
		lock_id = atoi(p2+2);
	else
		lock_id = KERNEL_RESERVED00;

	if (lock_id >= NUMBER_OF_LOCK) {
		pr_err("%s lock_id=%d is wrong", __FUNCTION__ ,lock_id);
		return -EINVAL;
	}

	if (strstr(buf, "-1")!=NULL)
		lock_value = -1;
	else
		lock_value = atoi(buf);

	pmqos_saving_data(pmqos_type, lock_id ,lock_value);

	printk(KERN_DEBUG "%s %s/%d id=%d value=%d\n", __FUNCTION__
		, get_current()->comm ,get_current()->pid ,lock_id ,lock_value);

	if (lock_value == -1) {
		if (pm_qos_request_active(&any_qos_array[lock_id]))
			pm_qos_remove_request(&any_qos_array[lock_id]);

	} else {
		if (!pm_qos_request_active(&any_qos_array[lock_id])) {
			pm_qos_add_request(&any_qos_array[lock_id]
				, pmqos_type, lock_value);
		} else
			pm_qos_update_request(&any_qos_array[lock_id], lock_value);
	}

	return 0;
}

static ssize_t show_cpufreq_max(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	max_cpu_freq = pm_qos_request(PM_QOS_CLUSTER0_FREQ_MAX);
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", max_cpu_freq);

	return ret;
}

static ssize_t store_cpufreq_max(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	int ret = 0;

	ret = set_pmqos_data(cpufreq_max_qos_array, PM_QOS_CLUSTER0_FREQ_MAX, buf);
	if (ret)
		return ret;

	return count;
}

static ssize_t show_cpufreq_min(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	min_cpu_freq = pm_qos_request(PM_QOS_CLUSTER0_FREQ_MIN);
	ret +=  snprintf(buf + ret, PAGE_SIZE - ret, "%d\n", min_cpu_freq);

	return ret;
}

static ssize_t store_cpufreq_min(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	int ret = 0;

	ret = set_pmqos_data(cpufreq_min_qos_array, PM_QOS_CLUSTER0_FREQ_MIN, buf);
	if (ret)
		return ret;

	return count;
}

static ssize_t show_pmqos_lock_state(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	unsigned int i, ret = 0;

	for (i = 0; i < NUMBER_OF_LOCK; i++) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"[%2d] CPU FREQ MIN:%7d MAX:%7d | ONLINE MIN:%d MAX:%d\n",
				i, trm_pm_qos_lock[i].cpufreq_min_value, trm_pm_qos_lock[i].cpufreq_max_value,
				trm_pm_qos_lock[i].cpu_online_min_value, trm_pm_qos_lock[i].cpu_online_max_value);
	}

	return ret;
}

static ssize_t store_pmqos_lock_state(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t show_cpu_online_max(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", pm_qos_request(PM_QOS_CPU_ONLINE_MAX));

	return ret;
}

static ssize_t __ref store_cpu_online_max(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	set_pmqos_data(cpu_online_max_qos_array, PM_QOS_CPU_ONLINE_MAX, buf);

	return count;
}

static ssize_t show_cpu_online_min(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", pm_qos_request(PM_QOS_CPU_ONLINE_MIN));

	return ret;
}

static ssize_t __ref store_cpu_online_min(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	set_pmqos_data(cpu_online_min_qos_array, PM_QOS_CPU_ONLINE_MIN, buf);

	return count;
}

/* BUS ++ */
static ssize_t show_bus_mif_freq_min(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", pm_qos_request(PM_QOS_BUS_THROUGHPUT));

	return ret;
}

static ssize_t store_bus_mif_freq_min(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	set_pmqos_data(bus_mif_min_qos_array, PM_QOS_BUS_THROUGHPUT, buf);

	return count;
}

static ssize_t show_bus_mif_freq_max(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", pm_qos_request(PM_QOS_BUS_THROUGHPUT_MAX));

	return ret;
}

static ssize_t store_bus_mif_freq_max(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	set_pmqos_data(bus_mif_max_qos_array, PM_QOS_BUS_THROUGHPUT_MAX, buf);

	return count;
}

static ssize_t show_bus_int_freq_min(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", pm_qos_request(PM_QOS_DEVICE_THROUGHPUT));

	return ret;
}

static ssize_t store_bus_int_freq_min(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	set_pmqos_data(bus_int_min_qos_array, PM_QOS_DEVICE_THROUGHPUT, buf);

	return count;
}
/* BUS -- */

static ssize_t show_device_name(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%s\n", DEVICE_NAME);

	return ret;
}

static ssize_t store_device_name(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	return count;
}
#if 0
static ssize_t show_cpuidle_w_aftr_en(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", cpuidle_get_sicd_en());

	return ret;
}

static ssize_t store_cpuidle_w_aftr_en(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	int input = atoi(buf);

	cpuidle_set_sicd_en(!!input);

	return count;
}

static ssize_t show_cpuidle_w_aftr_jig_check_en(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", cpuidle_get_sicd_jig_check_en());

	return ret;
}

static ssize_t store_cpuidle_w_aftr_jig_check_en(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	int input = atoi(buf);

	cpuidle_set_sicd_jig_check_en(!!input);

	return count;
}
#endif

#if defined(BUILD_ERROR)
static unsigned int cpu_gov_up_level_value = PM_QOS_CPU_GOV_UP_LEVEL_DEFAULT_VALUE;

unsigned int cpu_gov_get_up_level(void)
{
	cpu_gov_up_level_value = pm_qos_request(PM_QOS_CPU_GOV_UP_LEVEL);

	return cpu_gov_up_level_value;
}

static ssize_t show_cpu_gov_up_level(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", cpu_gov_get_up_level());

	return ret;
}

static ssize_t store_cpu_gov_up_level(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	set_pmqos_data(cpu_gov_up_level_array, PM_QOS_CPU_GOV_UP_LEVEL, buf);

	return count;
}

static unsigned int cpu_freq_up_threshold_value = PM_QOS_CPU_FREQ_UP_THRESHOLD_DEFAULT_VALUE;

unsigned int cpu_freq_get_threshold(void)
{
	cpu_freq_up_threshold_value = pm_qos_request(PM_QOS_CPU_FREQ_UP_THRESHOLD);

	return cpu_freq_up_threshold_value;
}

static ssize_t show_cpu_freq_up_threshold(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", cpu_freq_get_threshold());

	return ret;
}

static ssize_t store_cpu_freq_up_threshold(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	set_pmqos_data(cpu_freq_up_threshold_array, PM_QOS_CPU_FREQ_UP_THRESHOLD, buf);

	return count;
}
#endif

#if defined(CONFIG_SLP_PERFORMANCE_MONITOR)
char performance_log_req_str[64];

void update_performance_log_req(void)
{
	mutex_lock(&pmqos_kobject_mutex);
	if (!cpufreq_pmqos_kobject) {
		mutex_unlock(&pmqos_kobject_mutex);
		return;
	}
	sysfs_notify(cpufreq_pmqos_kobject, NULL, "performance_log_req");

#if defined(CONFIG_SLP_MINI_TRACER)
	kernel_mini_tracer("update_performance_log_req\n", TIME_ON | FLUSH_CACHE);
#endif
	mutex_unlock(&pmqos_kobject_mutex);
}

static ssize_t show_performance_log_req(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	unsigned int ret = 0;
	ret =  sprintf(buf, "%s\n", performance_log_req_str);

	return ret;
}

static ssize_t store_performance_log_req(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	if (strlen(buf) < sizeof(performance_log_req_str))
		strcpy(performance_log_req_str, buf);

	update_performance_log_req();

	return count;
}
#endif

#if defined(CONFIG_GEM_ALLOC_DEBUGS)
static ssize_t show_gem_alloc_info(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;
	ret = sprintf(buf, "%s\n", gem_alloc_buf);
	reset_gem_alloc_buf();
	return ret;
}
#endif

#if defined (CONFIG_SLP_BUSY_LEVEL)

void update_cpu_busy_level(int reason)
{
	unsigned long cur_time = jiffies;
	static unsigned long last_time = 0;
	char buf[128];
	int len = 0;
	int i = 0;
	int diff_time_boot = 0;

	mutex_lock(&pmqos_kobject_mutex);
	if (!cpufreq_pmqos_kobject) {
		mutex_unlock(&pmqos_kobject_mutex);
		return;
	}

	len += snprintf(buf + len, sizeof(buf) - len, "[B_LEVEL] Reason: ");
	switch (reason) {
		case BUSY_LEVEL_TRIGGER_CPU_LOAD:
			len += snprintf(buf + len, sizeof(buf) - len, "CPU ");
			break;
		case BUSY_LEVEL_TRIGGER_FRAME_DROP:
			len += snprintf(buf + len, sizeof(buf) - len, "FRAME ");
			break;
		case BUSY_LEVEL_TRIGGER_ALLOCSTALL:
			len += snprintf(buf + len, sizeof(buf) - len, "MEM ");
			break;
		case BUSY_LEVEL_TRIGGER_IOWAIT:
			len += snprintf(buf + len, sizeof(buf) - len, "IOWAIT ");
			break;
		default:
			len += snprintf(buf + len, sizeof(buf) - len, "MANUAL ");
	}

	update_time_in_state_diff();

	if (!last_time) {
		for (i = 0; time_in_state_diff[i].cpu_freq; i++) {
			if (time_in_state_diff[i].diff) diff_time_boot += time_in_state_diff[i].diff;
		}
		len += snprintf(buf + len, sizeof(buf) - len, "%u [", diff_time_boot);
		time_in_state_diff[0].time_diff_busy_occur = diff_time_boot;
	} else {
		time_in_state_diff[0].time_diff_busy_occur = jiffies_to_msecs(cur_time - last_time);
		len += snprintf(buf + len, sizeof(buf) - len, "%u [", time_in_state_diff[0].time_diff_busy_occur);
	}
	last_time = cur_time;

	for (i = 0; time_in_state_diff[i].cpu_freq; i++) {
		len += snprintf(buf + len, sizeof(buf) - len, "%d ", time_in_state_diff[i].diff);
	}
	len += snprintf(buf + len, sizeof(buf) - len, "]");

	pr_info("%s\n", buf);
	sysfs_notify(cpufreq_pmqos_kobject, NULL, "cpu_busy_level_value");
	mutex_unlock(&pmqos_kobject_mutex);

}

void check_idle_cpu_state(void)
{
	static unsigned long long begin = 0;
	unsigned int keep_time;

	if (begin == 0)
		begin = jiffies;

	keep_time = jiffies_to_msecs(jiffies - begin);
	if (keep_time >= PERIOD_IDLE_CHECK) {
		if(is_idle != get_cpu_idle_state()) {
			pr_info("[B_LEVEL] idle state %u -> %u (%u)\n", is_idle, !is_idle, keep_time);
			is_idle = !is_idle;
			begin = jiffies;
		}
	}
}

static ssize_t show_cpu_busy_level_value(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", la_get_cpu_busy_level());

	return ret;
}

static ssize_t store_cpu_busy_level_value(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	int input;

	input = atoi(buf);

	if ((input >= 0) && (input <= 9 )) {
		la_set_cpu_busy_level(input);
		update_cpu_busy_level(BUSY_LEVEL_TRIGGER_MANUAL);
	}

	return count;
}

static ssize_t show_is_device_idle(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", is_idle);

	return ret;
}

void store_wrist_up_state(void)
{
	pr_info("%s wrist up\n", __FUNCTION__);

	mutex_lock(&pmqos_kobject_mutex);
	if (!cpufreq_pmqos_kobject) {
		mutex_unlock(&pmqos_kobject_mutex);
		return;
	}

	wrist_up = 1;
	sysfs_notify(cpufreq_pmqos_kobject, NULL, "is_wrist_up");
	mutex_unlock(&pmqos_kobject_mutex);
}

void store_lcd_on_state(void)
{
	pr_info("%s lcd_on\n", __FUNCTION__);

	mutex_lock(&pmqos_kobject_mutex);
	if (!cpufreq_pmqos_kobject) {
		mutex_unlock(&pmqos_kobject_mutex);
		return;
	}

	if (wrist_up == 3) {
		wrist_up = 2;
		sysfs_notify(cpufreq_pmqos_kobject, NULL, "is_wrist_up");
	}
	mutex_unlock(&pmqos_kobject_mutex);
}

static ssize_t show_is_wrist_up(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%d\n", wrist_up);

	if (wrist_up == 1) wrist_up = 3;
	if (wrist_up == 2) wrist_up = 0;

	return ret;
}
#endif

#if defined(CONFIG_KSWAPD_WAKEUP_DEBUGS)
static ssize_t show_kswapd_wakeup_info(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%lu,%lu,%lu\n", kswapd_count,
		(unsigned long) jiffies_to_msecs(run_time),
		(unsigned long) jiffies_to_msecs(last_wake_time));

	return ret;
}
#endif

char trm_scen_simul_buf[128];
 static ssize_t show_trm_scen_simul(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	unsigned int ret = 0;

	ret = sprintf(buf, "%s\n", trm_scen_simul_buf);

	return ret;
}

static ssize_t store_trm_scen_simul(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	if (strlen(buf) < sizeof(trm_scen_simul_buf))
		strcpy(trm_scen_simul_buf, buf);

	sysfs_notify(cpufreq_pmqos_kobject, NULL, "trm_scen_simul");

	pr_info("%s\n", __FUNCTION__);
	mdelay(50);

	return count;
}


define_one_root_rw(device_name);

define_one_root_rw(cpufreq_max);
define_one_root_rw(cpufreq_min);
define_one_root_rw(cpu_online_max);
define_one_root_rw(cpu_online_min);
define_one_root_rw(pmqos_lock_state);

define_one_root_rw(bus_mif_freq_max);
define_one_root_rw(bus_mif_freq_min);
define_one_root_rw(bus_int_freq_min);
#if 0
define_one_root_rw(cpuidle_w_aftr_en);
define_one_root_rw(cpuidle_w_aftr_jig_check_en);
#endif
#if defined(BUILD_ERROR)
define_one_root_rw(cpu_gov_up_level);
define_one_root_rw(cpu_freq_up_threshold);
#endif

#if defined(CONFIG_SLP_PERFORMANCE_MONITOR)
define_one_root_rw(performance_log_req);
#endif

#if defined (CONFIG_SLP_BUSY_LEVEL)
define_one_root_rw(cpu_busy_level_value);
define_one_root_ro(is_device_idle);
define_one_root_ro(is_wrist_up);
#endif

#if defined(CONFIG_KSWAPD_WAKEUP_DEBUGS)
define_one_root_ro(kswapd_wakeup_info);
#endif

#if defined(CONFIG_GEM_ALLOC_DEBUGS)
define_one_root_ro(gem_alloc_info);
#endif

define_one_root_rw(trm_scen_simul);

static struct attribute *pmqos_attributes[] = {
	&cpufreq_min.attr,
	&cpufreq_max.attr,
	&cpu_online_min.attr,
	&cpu_online_max.attr,
	&pmqos_lock_state.attr,
	&bus_mif_freq_min.attr,
	&bus_mif_freq_max.attr,
	&bus_int_freq_min.attr,
#if defined(CONFIG_KSWAPD_WAKEUP_DEBUGS)
	&kswapd_wakeup_info.attr,
#endif
#if defined(CONFIG_GEM_ALLOC_DEBUGS)
	&gem_alloc_info.attr,
#endif
#if 0
	&cpuidle_w_aftr_en.attr,
	&cpuidle_w_aftr_jig_check_en.attr,
#endif
#if defined(BUILD_ERROR)
	&cpu_freq_up_threshold.attr,
	&cpu_gov_up_level.attr,
#endif
	&device_name.attr,
#if defined(CONFIG_SLP_PERFORMANCE_MONITOR)
	&performance_log_req.attr,
#endif
#if defined (CONFIG_SLP_BUSY_LEVEL)
	&cpu_busy_level_value.attr,
	&is_device_idle.attr,
	&is_wrist_up.attr,
#endif
	&trm_scen_simul.attr,
#if defined(TRM_TOUCH_BOOSTER_EN)
	&touch_boost_en.attr,
	&touch_boost_press.attr,
	&touch_boost_move.attr,
	&touch_boost_release.attr,
	&touch_cpu_online_min.attr,
#endif
	NULL
};

static struct attribute_group pmqos_attr_group = {
	.attrs = pmqos_attributes,
};

static int __ref cpu_online_min_qos_handler(struct notifier_block *b, unsigned long val, void *v)
{
	int cpu;
	int online_num_now = num_online_cpus();
	unsigned int turn_on_cpu_num, cpu_online_min;

	if (val == -1)
		goto success;

	cpu_online_min = min((unsigned int)pm_qos_request(PM_QOS_CPU_ONLINE_MAX), (unsigned int)val);

	if (cpu_online_min  <= online_num_now)
		goto success;

	turn_on_cpu_num = cpu_online_min -online_num_now;

	for_each_cpu_not(cpu, cpu_online_mask) {
		if (turn_on_cpu_num-- == 0)
			break;
		if (cpu == 0)
			continue;
		pr_info("CPU_UP %d\n", cpu);
		cpu_up(cpu);
	}
success:
	return NOTIFY_OK;
}

static int __ref cpu_online_max_qos_handler(struct notifier_block *b, unsigned long val, void *v)
{
	int cpu;
	int online_num_now = num_online_cpus();
	unsigned int turn_off_cpu_num, cpu_online_max;

	if (pm_qos_request(PM_QOS_CPU_ONLINE_MIN) > 1) {
		cpu_online_min_qos_handler(NULL, pm_qos_request(PM_QOS_CPU_ONLINE_MIN), NULL);
	}

	cpu_online_max = val;

	if (cpu_online_max  >= online_num_now)
		goto success;

	turn_off_cpu_num = online_num_now - cpu_online_max;

	for_each_online_cpu(cpu) {
		if (cpu == 0)
			continue;

		pr_info("CPU_DOWN %d\n", cpu);
		cpu_down(cpu);
		if (--turn_off_cpu_num == 0)
			break;
	}

success:
	return NOTIFY_OK;
}

static struct notifier_block cpu_online_min_qos_notifier = {
	.notifier_call = cpu_online_min_qos_handler,
};

static struct notifier_block cpu_online_max_qos_notifier = {
	.notifier_call = cpu_online_max_qos_handler,
};

static int __init cpufreq_pmqos_init(void)
{
	int err;

	cpufreq_pmqos_kobject= kobject_create_and_add("pmqos",
			cpufreq_global_kobject);
	if (!cpufreq_pmqos_kobject)
		return -ENOMEM;

	err = sysfs_create_group(cpufreq_pmqos_kobject, &pmqos_attr_group);
	if (err)
		kobject_put(cpufreq_pmqos_kobject);
	else
		kobject_uevent(cpufreq_pmqos_kobject, KOBJ_ADD);

	pm_qos_add_notifier(PM_QOS_CPU_ONLINE_MIN, &cpu_online_min_qos_notifier);
	pm_qos_add_notifier(PM_QOS_CPU_ONLINE_MAX, &cpu_online_max_qos_notifier);

#if defined(TRM_TOUCH_BOOSTER_EN)
	input_booster_init();
#endif

	return err;
}
late_initcall(cpufreq_pmqos_init);

MODULE_AUTHOR("Yong-U Baek <yu.baek@samsung.com>");
MODULE_DESCRIPTION("cpufreq_pmqos");
MODULE_LICENSE("GPL");
