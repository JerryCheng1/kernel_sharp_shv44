/* Proximity checker driver
 *
 * Copyright (C) 2016 SHARP CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <misc/proximity.h>

#define USE_WAKELOCK
#ifdef USE_WAKELOCK
//#include <linux/wakelock.h>
#endif /* USE_WAKELOCK */

/* --------------------------------------------------------------------------- */
/* COORDINATOR Qualcomm_CS1.1 BUILDERR MODIFY start */
/*#include <linux/wakelock.h>*/
#include <linux/ktime.h>
#include <linux/device.h>

/* A wake_lock prevents the system from entering suspend or other low power
 * states when active. If the type is set to WAKE_LOCK_SUSPEND, the wake_lock
 * prevents a full system suspend.
 */

enum {
	WAKE_LOCK_SUSPEND, /* Prevent suspend */
	WAKE_LOCK_TYPE_COUNT
};

struct wake_lock {
	struct wakeup_source ws;
};

static inline void wake_lock_init(struct wake_lock *lock, int type,
				  const char *name)
{
	wakeup_source_init(&lock->ws, name);
}

static inline void wake_lock_destroy(struct wake_lock *lock)
{
	wakeup_source_trash(&lock->ws);
}

static inline void wake_lock(struct wake_lock *lock)
{
	__pm_stay_awake(&lock->ws);
}

static inline void wake_lock_timeout(struct wake_lock *lock, long timeout)
{
	__pm_wakeup_event(&lock->ws, jiffies_to_msecs(timeout));
}

static inline void wake_unlock(struct wake_lock *lock)
{
	__pm_relax(&lock->ws);
}

static inline int wake_lock_active(struct wake_lock *lock)
{
	return lock->ws.active;
}
/* COORDINATOR Qualcomm_CS1.1 BUILDERR MODIFY end */


#define DEV_NAME "prxchk"
#define PRXCHK_REQUEST_TIMEOUT_MS (500)

/* debug log switch */
static int prxchk_dbg_func_log = 0;
static int prxchk_dbg_func_fin_log = 0;
static int prxchk_dbg_enable_log = 0;
static int prxchk_dbg_sensor_log = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(prxchk_dbg_func_log, int, 0600);
module_param(prxchk_dbg_func_fin_log, int, 0600);
module_param(prxchk_dbg_enable_log, int, 0600);
module_param(prxchk_dbg_sensor_log, int, 0600);
#endif  /* CONFIG_ANDROID_ENGINEERING */


#define FUNC_LOG() \
	if(prxchk_dbg_func_log == 1){ \
		printk(KERN_DEBUG "[%s] %s is called\n", DEV_NAME, __func__); \
	}

#define FUNC_FIN_LOG() \
	if(prxchk_dbg_func_fin_log == 1){ \
		printk(KERN_DEBUG "[%s] %s is finished\n", DEV_NAME, __func__); \
	}

#define DEBUG_LOG(format, ...) \
	if(prxchk_dbg_enable_log == 1){ \
		printk(KERN_DEBUG "[%s][%s] " format "\n", DEV_NAME, __func__, ## __VA_ARGS__); \
	}

#define DEBUG_SENSOR_LOG(format, ...) \
	if(prxchk_dbg_sensor_log == 1){ \
		printk(KERN_DEBUG "[%s][%s] " format "\n", DEV_NAME, __func__, ## __VA_ARGS__); \
	}

struct prxchk_data {
	struct device *dev;
	struct mutex lock;
#ifdef USE_WAKELOCK
	struct wake_lock wakelock;
#endif /* USE_WAKELOCK */
	atomic_t request;
	atomic_t value;
	struct completion value_written;
};

static struct prxchk_data *g_prxchk;


static ssize_t request_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct prxchk_data *prxchk = dev_get_drvdata(dev);
	if (NULL == prxchk) {
		pr_err("prxchk is null.\n");
		return -EINVAL;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&prxchk->request));
}

static ssize_t request_ack(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct prxchk_data *prxchk = dev_get_drvdata(dev);
	int val;

	if (NULL == prxchk) {
		pr_err("prxchk is null.\n");
		return -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	atomic_set(&prxchk->request, val);
	return count;
}
static DEVICE_ATTR(request, S_IRUSR | S_IWUSR, request_get, request_ack);

static ssize_t proximity_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct prxchk_data *prxchk = dev_get_drvdata(dev);
	if (NULL == prxchk) {
		pr_err("prxchk is null.\n");
		return -EINVAL;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&prxchk->value));
}

static ssize_t proximity_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct prxchk_data *prxchk = dev_get_drvdata(dev);
	int val;

	if (NULL == prxchk) {
		pr_err("prxchk is null.\n");
		return -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	if (val != SH_PROXIMITY_NEAR && val != SH_PROXIMITY_FAR)
		return -EINVAL;
	if (completion_done(&prxchk->value_written)) {
		return -ETIMEDOUT;
	}
	atomic_set(&prxchk->value, val);
	complete(&prxchk->value_written);
	return count;
}
static DEVICE_ATTR(proximity_value, S_IRUSR | S_IWUSR,
		proximity_value_show, proximity_value_store);

static struct attribute *attributes[] = {
	&dev_attr_request.attr,
	&dev_attr_proximity_value.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

//#define SW_PROXIMITY_TYPE_SYSFS
#ifdef SW_PROXIMITY_TYPE_SYSFS

#define PROXLIGHT_SENSOR_PATH_NAME "/sys/devices/virtual/input/input%d/name"
#define PROXLIGHT_SENSOR_PATH_PS_DATAREAD "/sys/devices/virtual/input/input%d/ps_dataread"
#define PROXIMITY_DEVICE_NAME       "proximity_sensor"
#define PROXIMITY_DISABLE_ERR       "-1"

static int ProxSensorIdx = 0;
static int ProxSensorIdxGetFlg = 0;

static int proxlib_get_sensor_index(void) {
    char devname[80],name[80];
    struct file *file;
    mm_segment_t old_fs;

    if(ProxSensorIdxGetFlg  == 0){
        for(ProxSensorIdx = 0; ProxSensorIdx < 30; ProxSensorIdx++){
            memset(name, 0x00, sizeof(devname));
            memset(name, 0x00, sizeof(name));
            
            sprintf(devname, PROXLIGHT_SENSOR_PATH_NAME, ProxSensorIdx);
            file = filp_open( devname, O_RDONLY, 0);
            if (IS_ERR_OR_NULL(file)) {
                pr_err("cannot open file");
                continue;
            }
            old_fs = get_fs();
            set_fs(get_ds());

            if( (file->f_op->read(file, name, sizeof(name), &file->f_pos)) < 0){
                name[0] = '\0';
            }else{
                name[strlen(PROXIMITY_DEVICE_NAME )] = '\0';
            }
            set_fs(old_fs);
            filp_close(file, NULL);
            if ( strcmp(name, PROXIMITY_DEVICE_NAME ) == 0 )
            {
                ProxSensorIdxGetFlg = 1;
                break;
            }else{
                continue;
            }
        }
    }
    return 0;
}

static int proximity_distance_sysfs(int *distance) {
    int ret;
    char value[4];
    char devname[80];
    struct file *file;
    mm_segment_t old_fs;

    memset(devname, 0x00, sizeof(devname));
    memset(value,0x00,sizeof(value));
    ret = proxlib_get_sensor_index();
    
    sprintf(devname, PROXLIGHT_SENSOR_PATH_PS_DATAREAD, ProxSensorIdx);
    file = filp_open(devname, O_RDONLY, 0);
    if (IS_ERR_OR_NULL(file)) {
        pr_err("cannot open file. err=%p \n", file);
        return -1;
    }
    old_fs = get_fs();
    set_fs(get_ds());

    ret = file->f_op->read(file, value, sizeof(value), &file->f_pos);
    set_fs(old_fs);
    filp_close(file, NULL);
    if( strcmp( value, PROXIMITY_DISABLE_ERR ) == 0 ){
        pr_err("ps distance read failed = %s \n", value);
        *distance = -1;
        return -1;
    }
    *distance = simple_strtoul( value, NULL, 10 ) ;
    return ret;
}
#endif /* SW_PROXIMITY_TYPE_SYSFS */


int PROX_dataread_func(int *read_data)
{
	struct prxchk_data *prxchk = g_prxchk;
	unsigned long timeout = msecs_to_jiffies(PRXCHK_REQUEST_TIMEOUT_MS);

#ifdef SW_PROXIMITY_TYPE_SYSFS
	return ( proximity_distance_sysfs(read_data) );
#else
	if (NULL == prxchk) {
		pr_err("prxchk is null.\n");
		return -EINVAL;
	}
	FUNC_LOG();

#ifdef USE_WAKELOCK
	wake_lock(&prxchk->wakelock);
#endif /* USE_WAKELOCK */
	mutex_lock(&prxchk->lock);
	atomic_set(&prxchk->request, 1);

	reinit_completion(&prxchk->value_written);

	/* request proximity value to user space process */
	sysfs_notify(&prxchk->dev->kobj, NULL, dev_attr_request.attr.name);

	/* wait until arriving proximity value or timeout */
	DEBUG_LOG("wait for arriving proximity value");
	if (wait_for_completion_timeout(&prxchk->value_written, timeout)) {
		*read_data = atomic_read(&prxchk->value);
	} else {
		/* timeout */
		*read_data = SH_PROXIMITY_FAR;
		DEBUG_LOG("timeout");
	}

	DEBUG_SENSOR_LOG("read_data = %d", *read_data);

	atomic_set(&prxchk->request, 0);
	mutex_unlock(&prxchk->lock);
#ifdef USE_WAKELOCK
	wake_unlock(&prxchk->wakelock);
#endif /* USE_WAKELOCK */

#endif /* SW_PROXIMITY_TYPE_SYSFS */

	FUNC_FIN_LOG();

	return 0;
}

int PROX_stateread_func(int *state_data, int *read_data)
{
	struct prxchk_data *prxchk = g_prxchk;

	if (NULL == prxchk) {
		pr_err("prxchk is null.\n");
		return -EINVAL;
	}
	FUNC_LOG();

	/* always return SH_PROXIMITY_DISABLE for backward compatibility */
	*state_data = SH_PROXIMITY_DISABLE;
	*read_data = -1;

	FUNC_FIN_LOG();

	return 0;
}

static int prxchk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct prxchk_data *prxchk;
	int rc = 0;

	FUNC_LOG();

	prxchk = devm_kzalloc(dev, sizeof(*prxchk), GFP_KERNEL);
	if (NULL == prxchk) {
		pr_err("failed to allocate memory for struct prxchk_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	g_prxchk = prxchk;
	prxchk->dev = dev;
	atomic_set(&prxchk->request, 0);
	atomic_set(&prxchk->value, SH_PROXIMITY_FAR);
	platform_set_drvdata(pdev, prxchk);

	mutex_init(&prxchk->lock);
#ifdef USE_WAKELOCK
	wake_lock_init(&prxchk->wakelock, WAKE_LOCK_SUSPEND, DEV_NAME);
#endif /* USE_WAKELOCK */

	init_completion(&prxchk->value_written);

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

exit:
	FUNC_FIN_LOG();

	return rc;
}

static int prxchk_remove(struct platform_device *pdev)
{
	struct prxchk_data *prxchk = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (NULL == prxchk) {
		pr_err("prxchk is null.\n");
		return -EINVAL;
	}

	FUNC_LOG();

	sysfs_remove_group(&dev->kobj, &attribute_group);
#ifdef USE_WAKELOCK
	wake_lock_destroy(&prxchk->wakelock);
#endif /* USE_WAKELOCK */
	mutex_destroy(&prxchk->lock);

	FUNC_FIN_LOG();

	return 0;
}

static struct platform_device prxchk_device = {
	.name = DEV_NAME,
	.id = -1,
};

static struct platform_driver prxchk_driver = {
	.driver = {
		.name = DEV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = prxchk_probe,
	.remove = prxchk_remove,
};

static int __init prxchk_init(void)
{
	int rc;

	rc = platform_device_register(&prxchk_device);
	if (rc) {
		pr_err("%s failed to register platform device\n", __func__);
		return -EINVAL;
	}

	rc = platform_driver_register(&prxchk_driver);
	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);

	return rc;
}

static void __exit prxchk_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&prxchk_driver);
	platform_device_unregister(&prxchk_device);
}

EXPORT_SYMBOL(PROX_dataread_func);
EXPORT_SYMBOL(PROX_stateread_func);

module_init(prxchk_init);
module_exit(prxchk_exit);

MODULE_DESCRIPTION("Proximity checker driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
