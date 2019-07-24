/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define TRILED_REG_TYPE			0x04
#define TRILED_REG_SUBTYPE		0x05
#define TRILED_REG_EN_CTL		0x46

/* TRILED_REG_EN_CTL */
#define TRILED_EN_CTL_MASK		GENMASK(7, 5)
#define TRILED_EN_CTL_MAX_BIT		7

#define TRILED_TYPE			0x19
#define TRILED_SUBTYPE_LED3H0L12	0x02
#define TRILED_SUBTYPE_LED2H0L12	0x03
#define TRILED_SUBTYPE_LED1H2L12	0x04

#define TRILED_NUM_MAX			3

#define PWM_PERIOD_DEFAULT_NS		1000000

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00016 */
#ifndef MAX
#define  MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif
#define QPNP_TRI_LED_RED	0
#define QPNP_TRI_LED_GREEN	1
#define QPNP_TRI_LED_BLUE	2
#endif /* CONFIG_LEDS_SHARP */

struct pwm_setting {
	u64	pre_period_ns;
	u64	period_ns;
	u64	duty_ns;
};

struct led_setting {
	u64			on_ms;
	u64			off_ms;
	enum led_brightness	brightness;
	bool			blink;
	bool			breath;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
	u32			*duty_pcts_buf;
	u16			duty_pcts_length;
	u16			ramp_step_ms;
	u16			pause_hi;
	u16			pause_lo;
	u8			lut_flags;
	u8			start_idx;
#endif /* CONFIG_LEDS_SHARP */
};

struct qpnp_led_dev {
	struct led_classdev	cdev;
	struct pwm_device	*pwm_dev;
	struct pwm_setting	pwm_setting;
	struct led_setting	led_setting;
	struct qpnp_tri_led_chip	*chip;
	struct mutex		lock;
	const char		*label;
	const char		*default_trigger;
	u8			id;
	bool			blinking;
	bool			breathing;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00016 */
	u32			calib;
#endif /* CONFIG_LEDS_SHARP */
};

struct qpnp_tri_led_chip {
	struct device		*dev;
	struct regmap		*regmap;
	struct qpnp_led_dev	*leds;
	struct nvmem_device	*pbs_nvmem;
	struct mutex		bus_lock;
	int			num_leds;
	u16			reg_base;
	u8			subtype;
	u8			bitmap;
};

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00016 */
static char qpnp_led_system_color[32];
static u32 calib_enabled = 0;
module_param_string(system_color, qpnp_led_system_color, sizeof(qpnp_led_system_color), 0);
module_param(calib_enabled, uint, S_IRUGO | S_IWUSR);

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
#define RAMP_STEP_MS_MAX (511)
#define START_IDX_MAX (PWM_STATE_PATTERN_LENGTH_MAX - 1)
#endif /* CONFIG_LEDS_SHARP */

static u32 qpnp_tri_led_calib_brightness(struct qpnp_led_dev *led)
{
	pr_debug("%s: in %s LED calib_enabled=%d\n", __func__, led->cdev.name, calib_enabled);
	if (led->led_setting.brightness && calib_enabled &&
		led->calib && !led->blinking && !led->breathing) {
		return MAX(led->led_setting.brightness * led->calib / LED_FULL, 1);
	} else {
		return led->cdev.brightness;
	}
}

static void qpnp_tri_led_calib_read_clrvari_param(struct qpnp_led_dev *led)
{
	struct device_node *root, *node;
	u32 color;
	char default_name[32];
	int rc = -EINVAL;

	pr_debug("%s: in %s LED node=%s\n", __func__, led->cdev.name, qpnp_led_system_color);

	root = of_find_node_by_name(NULL, "sharp,shled_leds_color_variation");
	if (!root) {
		pr_err("%s: could not find root node.\n", __func__);
		goto calib_read_error;
	}

	node = of_find_node_by_name(root, qpnp_led_system_color);
	if (!node) {
		pr_err("%s: could not find node %s, try default.\n", __func__, qpnp_led_system_color);
		rc = of_property_read_u32(root, "sharp,system-color-default", &color);
		if (rc < 0) {
			pr_err("%s: could not find default node.\n", __func__);
			goto calib_read_error;
		}
		snprintf(default_name, sizeof(default_name), "sharp,shled-leds-calib-%02x", color);
		pr_debug("%s: default_name=%s\n", __func__, default_name);
		node = of_find_node_by_name(root, default_name);
		if (!node) {
			pr_err("%s: could not find node %s.\n", __func__, default_name);
			goto calib_read_error;
		}
	}

	if (led->id == QPNP_TRI_LED_RED) {
		rc = of_property_read_u32(node, "red", &led->calib);
	} else if (led->id == QPNP_TRI_LED_GREEN) {
		rc = of_property_read_u32(node, "green", &led->calib);
	} else if (led->id == QPNP_TRI_LED_BLUE) {
		rc = of_property_read_u32(node, "blue", &led->calib);
	}
	pr_debug("%s: led->calib=%d.\n", __func__, led->calib);
	if (rc) {
		pr_err("%s: could not find calibration parameter param:%s led->id:%d.\n", __func__, qpnp_led_system_color, led->id);
		goto calib_read_error;
	}

	calib_enabled = 1;

	return;

calib_read_error:
	calib_enabled = 0;
	return;
}
#endif /* CONFIG_LEDS_SHARP */

static int qpnp_tri_led_read(struct qpnp_tri_led_chip *chip, u16 addr, u8 *val)
{
	int rc;
	unsigned int tmp;

	mutex_lock(&chip->bus_lock);
	rc = regmap_read(chip->regmap, chip->reg_base + addr, &tmp);
	if (rc < 0)
		dev_err(chip->dev, "Read addr 0x%x failed, rc=%d\n", addr, rc);
	else
		*val = (u8)tmp;
	mutex_unlock(&chip->bus_lock);

	return rc;
}

static int qpnp_tri_led_masked_write(struct qpnp_tri_led_chip *chip,
				u16 addr, u8 mask, u8 val)
{
	int rc;

	mutex_lock(&chip->bus_lock);
	rc = regmap_update_bits(chip->regmap, chip->reg_base + addr, mask, val);
	if (rc < 0)
		dev_err(chip->dev, "Update addr 0x%x to val 0x%x with mask 0x%x failed, rc=%d\n",
					addr, val, mask, rc);
	mutex_unlock(&chip->bus_lock);

	return rc;
}

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
static int leds_qti_led_to_pwm_state(struct pwm_state *pstate,
					struct led_setting *led_setting)
{

	if (led_setting->brightness == 0) {
		pr_debug("%s: led off request\n",__func__);
		return -EINVAL;
	}
	if (led_setting->duty_pcts_length == 0) {
		pr_debug("%s: not set\n",__func__);
		return -EINVAL;
	}
	if (led_setting->ramp_step_ms == 0) {
		pr_debug("%s: not set\n",__func__);
		return -EINVAL;
	}

	pstate->pattern_length = led_setting->duty_pcts_length;
	pstate->pattern = led_setting->duty_pcts_buf;

	pstate->ramp_step_ms = led_setting->ramp_step_ms;
	pstate->pause_hi     = led_setting->pause_hi;
	pstate->pause_lo     = led_setting->pause_lo;
	pstate->lut_flags    = led_setting->lut_flags;
	pstate->lo_idx       = led_setting->start_idx;
	return 0;
}
#endif /* CONFIG_LEDS_SHARP */

static int __tri_led_config_pwm(struct qpnp_led_dev *led,
				struct pwm_setting *pwm)
{
	struct pwm_state pstate;
	int rc;

	pwm_get_state(led->pwm_dev, &pstate);
	pstate.enabled = !!(pwm->duty_ns != 0);
	pstate.period = pwm->period_ns;
	pstate.duty_cycle = pwm->duty_ns;
	pstate.output_type = led->led_setting.breath ?
		PWM_OUTPUT_MODULATED : PWM_OUTPUT_FIXED;
	/* Use default pattern in PWM device */
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
	if (led->led_setting.breath) {
		pstate.pattern = NULL;
		pstate.pattern_length = 0;
		rc = leds_qti_led_to_pwm_state(&pstate, &(led->led_setting));
	}
#endif /* CONFIG_LEDS_SHARP */
	pstate.output_pattern = NULL;
	rc = pwm_apply_state(led->pwm_dev, &pstate);

	if (rc < 0)
		dev_err(led->chip->dev, "Apply PWM state for %s led failed, rc=%d\n",
					led->cdev.name, rc);

	return rc;
}

#define PBS_ENABLE	1
#define PBS_DISABLE	2
#define PBS_ARG		0x42
#define PBS_TRIG_CLR	0xE6
#define PBS_TRIG_SET	0xE5
static int __tri_led_set(struct qpnp_led_dev *led)
{
	int rc = 0;
	u8 val = 0, mask = 0, pbs_val;
	u8 prev_bitmap;

	rc = __tri_led_config_pwm(led, &led->pwm_setting);
	if (rc < 0) {
		dev_err(led->chip->dev, "Configure PWM for %s led failed, rc=%d\n",
					led->cdev.name, rc);
		return rc;
	}

	mask |= 1 << (TRILED_EN_CTL_MAX_BIT - led->id);

	if (led->pwm_setting.duty_ns == 0)
		val = 0;
	else
		val = mask;

	if (led->chip->subtype == TRILED_SUBTYPE_LED2H0L12 &&
		led->chip->pbs_nvmem) {
		/*
		 * Control BOB_CONFIG_EXT_CTRL2_FORCE_EN for HR_LED through
		 * PBS trigger. PBS trigger for enable happens if any one of
		 * LEDs are turned on. PBS trigger for disable happens only
		 * if both LEDs are turned off.
		 */

		prev_bitmap = led->chip->bitmap;
		if (val)
			led->chip->bitmap |= (1 << led->id);
		else
			led->chip->bitmap &= ~(1 << led->id);

		if (!(led->chip->bitmap & prev_bitmap)) {
			pbs_val = led->chip->bitmap ? PBS_ENABLE : PBS_DISABLE;
			rc = nvmem_device_write(led->chip->pbs_nvmem, PBS_ARG,
				1, &pbs_val);
			if (rc < 0) {
				dev_err(led->chip->dev, "Couldn't set PBS_ARG, rc=%d\n",
					rc);
				return rc;
			}

			pbs_val = 1;
			rc = nvmem_device_write(led->chip->pbs_nvmem,
				PBS_TRIG_CLR, 1, &pbs_val);
			if (rc < 0) {
				dev_err(led->chip->dev, "Couldn't set PBS_TRIG_CLR, rc=%d\n",
					rc);
				return rc;
			}

			pbs_val = 1;
			rc = nvmem_device_write(led->chip->pbs_nvmem,
				PBS_TRIG_SET, 1, &pbs_val);
			if (rc < 0) {
				dev_err(led->chip->dev, "Couldn't set PBS_TRIG_SET, rc=%d\n",
					rc);
				return rc;
			}
		}
	}

	rc = qpnp_tri_led_masked_write(led->chip, TRILED_REG_EN_CTL,
							mask, val);
	if (rc < 0)
		dev_err(led->chip->dev, "Update addr 0x%x failed, rc=%d\n",
					TRILED_REG_EN_CTL, rc);

	return rc;
}

static int qpnp_tri_led_set(struct qpnp_led_dev *led)
{
	u64 on_ms, off_ms, period_ns, duty_ns;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00016 */
	u32 brightness = qpnp_tri_led_calib_brightness(led);
#else /* CONFIG_LEDS_SHARP */
	enum led_brightness brightness = led->led_setting.brightness;
#endif /* CONFIG_LEDS_SHARP */
	int rc = 0;

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00016 */
	pr_debug("%s: brightness=%d calib_brightness=%d\n", __func__,
		led->led_setting.brightness, brightness);
#endif /* CONFIG_LEDS_SHARP */

	if (led->led_setting.blink) {
		on_ms = led->led_setting.on_ms;
		off_ms = led->led_setting.off_ms;

		duty_ns = on_ms * NSEC_PER_MSEC;
		period_ns = (on_ms + off_ms) * NSEC_PER_MSEC;

		if (period_ns < duty_ns && duty_ns != 0)
			period_ns = duty_ns + 1;
	} else {
		/* Use initial period if no blinking is required */
		period_ns = led->pwm_setting.pre_period_ns;

		if (brightness == LED_OFF)
			duty_ns = 0;

		duty_ns = period_ns * brightness;
		do_div(duty_ns, LED_FULL);

		if (period_ns < duty_ns && duty_ns != 0)
			period_ns = duty_ns + 1;
	}
	dev_dbg(led->chip->dev, "PWM settings for %s led: period = %lluns, duty = %lluns\n",
				led->cdev.name, period_ns, duty_ns);
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00016 */
	pr_debug("%s: duty_ns=%d period_ns=%d brightness=%d calib_enabled=%d calib=%d calib_brightness=%d\n",
		__func__, duty_ns, period_ns, led->led_setting.brightness, calib_enabled, led->calib, brightness);
#endif /* CONFIG_LEDS_SHARP */

	led->pwm_setting.duty_ns = duty_ns;
	led->pwm_setting.period_ns = period_ns;

	rc = __tri_led_set(led);
	if (rc < 0) {
		dev_err(led->chip->dev, "__tri_led_set %s failed, rc=%d\n",
				led->cdev.name, rc);
		return rc;
	}

	if (led->led_setting.blink) {
		led->cdev.brightness = LED_FULL;
		led->blinking = true;
		led->breathing = false;
	} else if (led->led_setting.breath) {
		led->cdev.brightness = LED_FULL;
		led->blinking = false;
		led->breathing = true;
	} else {
		led->cdev.brightness = led->led_setting.brightness;
		led->blinking = false;
		led->breathing = false;
	}

	return rc;
}

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00058 */
static void qpnp_tri_led_set_brightness(struct led_classdev *led_cdev,
#else /* CONFIG_LEDS_SHARP */
static int qpnp_tri_led_set_brightness(struct led_classdev *led_cdev,
#endif /* CONFIG_LEDS_SHARP */
		enum led_brightness brightness)
{
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);
	int rc = 0;

	mutex_lock(&led->lock);
	if (brightness > LED_FULL)
		brightness = LED_FULL;

	if (brightness == led->led_setting.brightness &&
			!led->blinking && !led->breathing) {
		mutex_unlock(&led->lock);
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00058 */
		return;
#else /* CONFIG_LEDS_SHARP */
		return 0;
#endif /* CONFIG_LEDS_SHARP */
	}

	led->led_setting.brightness = brightness;
	if (!!brightness)
		led->led_setting.off_ms = 0;
	else
		led->led_setting.on_ms = 0;
	led->led_setting.blink = false;
	led->led_setting.breath = false;

	rc = qpnp_tri_led_set(led);
	if (rc)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

	mutex_unlock(&led->lock);

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00058 */
	return;
#else /* CONFIG_LEDS_SHARP */
	return rc;
#endif /* CONFIG_LEDS_SHARP */
}

static enum led_brightness qpnp_tri_led_get_brightness(
			struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static int qpnp_tri_led_set_blink(struct led_classdev *led_cdev,
		unsigned long *on_ms, unsigned long *off_ms)
{
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);
	int rc = 0;

	mutex_lock(&led->lock);
	if (led->blinking && *on_ms == led->led_setting.on_ms &&
			*off_ms == led->led_setting.off_ms) {
		dev_dbg(led_cdev->dev, "Ignore, on/off setting is not changed: on %lums, off %lums\n",
						*on_ms, *off_ms);
		mutex_unlock(&led->lock);
		return 0;
	}

	if (*on_ms == 0) {
		led->led_setting.blink = false;
		led->led_setting.breath = false;
		led->led_setting.brightness = LED_OFF;
	} else if (*off_ms == 0) {
		led->led_setting.blink = false;
		led->led_setting.breath = false;
		led->led_setting.brightness = led->cdev.brightness;
	} else {
		led->led_setting.on_ms = *on_ms;
		led->led_setting.off_ms = *off_ms;
		led->led_setting.blink = true;
		led->led_setting.breath = false;
	}

	rc = qpnp_tri_led_set(led);
	if (rc)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

	mutex_unlock(&led->lock);
	return rc;
}

static ssize_t breath_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", led->led_setting.breath);
}

static ssize_t breath_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int rc;
	bool breath;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	rc = kstrtobool(buf, &breath);
	if (rc < 0)
		return rc;

	mutex_lock(&led->lock);
	if (led->breathing == breath)
		goto unlock;

	led->led_setting.blink = false;
	led->led_setting.breath = breath;
	led->led_setting.brightness = breath ? LED_FULL : LED_OFF;
	rc = qpnp_tri_led_set(led);
	if (rc < 0)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

unlock:
	mutex_unlock(&led->lock);
	return (rc < 0) ? rc : count;
}

static DEVICE_ATTR(breath, 0644, breath_show, breath_store);
static const struct attribute *breath_attrs[] = {
	&dev_attr_breath.attr,
	NULL
};

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
static ssize_t pause_lo_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	u16 pause_lo;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtou16(buf, 10, &pause_lo);
	if (ret)
		return ret;

	pr_debug("%s in id=%d, pause_lo=%d\n", __func__, led->id, pause_lo);
	switch (led->id) {
	case QPNP_TRI_LED_RED:
	case QPNP_TRI_LED_GREEN:
	case QPNP_TRI_LED_BLUE:
		mutex_lock(&led->lock);
		led->led_setting.pause_lo = pause_lo;
		mutex_unlock(&led->lock);
		break;
	default:
		dev_err(led->chip->dev,
			"Invalid LED id type for pause lo\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t pause_hi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	u16 pause_hi;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtou16(buf, 10, &pause_hi);
	if (ret)
		return ret;

	pr_debug("%s in id=%d, pause_hi=%d\n", __func__, led->id, pause_hi);
	switch (led->id) {
	case QPNP_TRI_LED_RED:
	case QPNP_TRI_LED_GREEN:
	case QPNP_TRI_LED_BLUE:
		mutex_lock(&led->lock);
		led->led_setting.pause_hi = pause_hi;
		mutex_unlock(&led->lock);
		break;
	default:
		dev_err(led->chip->dev,
			"Invalid LED id type for pause hi\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t start_idx_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	u8 start_idx;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtou8(buf, 10, &start_idx);
	if (ret)
		return ret;

	pr_debug("%s in id=%d, start_idx=%d\n", __func__, led->id, start_idx);
	if (start_idx > START_IDX_MAX) {
		start_idx = START_IDX_MAX;
	}

	switch (led->id) {
	case QPNP_TRI_LED_RED:
	case QPNP_TRI_LED_GREEN:
	case QPNP_TRI_LED_BLUE:
		mutex_lock(&led->lock);
		led->led_setting.start_idx = start_idx;
		mutex_unlock(&led->lock);
		break;
	default:
		dev_err(led->chip->dev,
			"Invalid LED id type for start idx\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t lut_flags_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	u8 lut_flags;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtou8(buf, 10, &lut_flags);
	if (ret)
		return ret;

	pr_debug("%s in id=%d, lut_flags=0x%02x\n", __func__, led->id, lut_flags);
	switch (led->id) {
	case QPNP_TRI_LED_RED:
	case QPNP_TRI_LED_GREEN:
	case QPNP_TRI_LED_BLUE:
		mutex_lock(&led->lock);
		led->led_setting.lut_flags = lut_flags;
		mutex_unlock(&led->lock);
		break;
	default:
		dev_err(led->chip->dev,
			"Invalid LED id type for lut flags\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t ramp_step_ms_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	u16 ramp_step_ms;

	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtou16(buf, 10, &ramp_step_ms);
	if (ret)
		return ret;

	if (ramp_step_ms > RAMP_STEP_MS_MAX) {
		pr_debug("%s:ramp_step_ms is over(%d)\n",__func__, ramp_step_ms);
		ramp_step_ms = RAMP_STEP_MS_MAX;
	}
	pr_debug("%s in id=%d, ramp_step_ms=%d\n", __func__, led->id, ramp_step_ms);
	switch (led->id) {
	case QPNP_TRI_LED_RED:
	case QPNP_TRI_LED_GREEN:
	case QPNP_TRI_LED_BLUE:
		mutex_lock(&led->lock);
		led->led_setting.ramp_step_ms = ramp_step_ms;
		mutex_unlock(&led->lock);
		break;
	default:
		dev_err(led->chip->dev,
			"Invalid LED id type for ramp step\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t duty_pcts_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int ret;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);
	char *buffer;
	int i = 0;
	int num_duty_pcts = 0;
	int max_duty_pcts;
	int value;

	switch (led->id) {
	case QPNP_TRI_LED_RED:
	case QPNP_TRI_LED_GREEN:
	case QPNP_TRI_LED_BLUE:
		max_duty_pcts = PWM_STATE_PATTERN_LENGTH_MAX;
		break;
	default:
		dev_err(led->chip->dev,
			"Invalid LED id type for ramp step\n");
		return -EINVAL;
	}

	mutex_lock(&led->lock);
	buffer = (char *)buf;

	for (i = 0; i < max_duty_pcts; i++) {
		if (buffer == NULL)
			break;
		ret = sscanf((const char *)buffer, "%u,%s", &value, buffer);
		led->led_setting.duty_pcts_buf[i] = value;
		num_duty_pcts++;
		if (ret <= 1)
			break;
	}

	if (num_duty_pcts > max_duty_pcts) {
		dev_err(led->chip->dev,
			"Number of duty pcts given exceeds max (%d)\n",
			max_duty_pcts);
		return -EINVAL;
	}

	pr_debug("%s:duty_pcts num=%d\n",__func__, num_duty_pcts);
	led->led_setting.duty_pcts_length = num_duty_pcts;
	mutex_unlock(&led->lock);

	return count;
}

static DEVICE_ATTR(pause_lo, 0664, NULL, pause_lo_store);
static DEVICE_ATTR(pause_hi, 0664, NULL, pause_hi_store);
static DEVICE_ATTR(start_idx, 0664, NULL, start_idx_store);
static DEVICE_ATTR(lut_flags, 0664, NULL, lut_flags_store);
static DEVICE_ATTR(ramp_step_ms, 0664, NULL, ramp_step_ms_store);
static DEVICE_ATTR(duty_pcts, 0664, NULL, duty_pcts_store);

static struct attribute *lpg_attrs[] = {
	&dev_attr_pause_lo.attr,
	&dev_attr_pause_hi.attr,
	&dev_attr_start_idx.attr,
	&dev_attr_lut_flags.attr,
	&dev_attr_ramp_step_ms.attr,
	&dev_attr_duty_pcts.attr,
	NULL
};

static const struct attribute_group lpg_attr_group = {
	.attrs = lpg_attrs,
};

static int qpnp_tri_led_setting_init(struct pwm_state *state,
					struct led_setting *led_setting)
{
	//duty_pcts
	led_setting->duty_pcts_length = state->pattern_length;
	memcpy(led_setting->duty_pcts_buf, state->pattern,
				sizeof(u32) * led_setting->duty_pcts_length);

	//ramp_step_ms
	led_setting->ramp_step_ms = state->ramp_step_ms;

	//start_idx
	led_setting->start_idx = state->lo_idx;

	//pause_hi
	state->pause_hi;
	//pause_lo
	state->pause_lo;

	//lut_flags
	state->lut_flags;

	return 0;
}
#endif /* CONFIG_LEDS_SHARP */

static int qpnp_tri_led_register(struct qpnp_tri_led_chip *chip)
{
	struct qpnp_led_dev *led;
	int rc, i, j;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
	struct led_setting *led_setting;
	struct pwm_chip *pwm_chip;
	struct pwm_state state;
#endif /* CONFIG_LEDS_SHARP */

	for (i = 0; i < chip->num_leds; i++) {
		led = &chip->leds[i];
		mutex_init(&led->lock);
		led->cdev.name = led->label;
		led->cdev.max_brightness = LED_FULL;
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00058 */
		led->cdev.brightness_set = qpnp_tri_led_set_brightness;
#else /* CONFIG_LEDS_SHARP */
		led->cdev.brightness_set_blocking = qpnp_tri_led_set_brightness;
#endif /* CONFIG_LEDS_SHARP */
		led->cdev.brightness_get = qpnp_tri_led_get_brightness;
		led->cdev.blink_set = qpnp_tri_led_set_blink;
		led->cdev.default_trigger = led->default_trigger;
		led->cdev.brightness = LED_OFF;
		led->cdev.flags |= LED_KEEP_TRIGGER;

		rc = devm_led_classdev_register(chip->dev, &led->cdev);
		if (rc < 0) {
			dev_err(chip->dev, "%s led class device registering failed, rc=%d\n",
							led->label, rc);
			goto err_out;
		}

		if (pwm_get_output_type_supported(led->pwm_dev)
				& PWM_OUTPUT_MODULATED) {
			rc = sysfs_create_files(&led->cdev.dev->kobj,
					breath_attrs);
			if (rc < 0) {
				dev_err(chip->dev, "Create breath file for %s led failed, rc=%d\n",
						led->label, rc);
				goto err_out;
			}
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
			if ((led->id == QPNP_TRI_LED_RED) ||
			    (led->id == QPNP_TRI_LED_GREEN) ||
			    (led->id == QPNP_TRI_LED_BLUE)) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc) {
					goto err_out;
				}

				led_setting = &led->led_setting;
				led_setting->duty_pcts_buf =
					devm_kzalloc(chip->dev,
					sizeof(u32) * PWM_STATE_PATTERN_LENGTH_MAX,
					GFP_KERNEL);
				if (!led_setting->duty_pcts_buf)
					goto err_out;

				pwm_chip = led->pwm_dev->chip;
				if (pwm_chip->ops->get_ramp_config) {
					rc = pwm_chip->ops->get_ramp_config(
							led->pwm_dev, &state);
					if (rc < 0) {
						goto err_out;
					}
					qpnp_tri_led_setting_init(&state, led_setting);
				}
			}
#endif /* CONFIG_LEDS_SHARP */
		}
	}

	return 0;

err_out:
	for (j = 0; j <= i; j++) {
		if (j < i)
			sysfs_remove_files(&chip->leds[j].cdev.dev->kobj,
					breath_attrs);
		mutex_destroy(&chip->leds[j].lock);
	}
	return rc;
}

static int qpnp_tri_led_hw_init(struct qpnp_tri_led_chip *chip)
{
	int rc = 0;
	u8 val;

	rc = qpnp_tri_led_read(chip, TRILED_REG_TYPE, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Read REG_TYPE failed, rc=%d\n", rc);
		return rc;
	}

	if (val != TRILED_TYPE) {
		dev_err(chip->dev, "invalid subtype(%d)\n", val);
		return -ENODEV;
	}

	rc = qpnp_tri_led_read(chip, TRILED_REG_SUBTYPE, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Read REG_SUBTYPE failed, rc=%d\n", rc);
		return rc;
	}

	chip->subtype = val;

	return 0;
}

static int qpnp_tri_led_parse_dt(struct qpnp_tri_led_chip *chip)
{
	struct device_node *node = chip->dev->of_node, *child_node;
	struct qpnp_led_dev *led;
	struct pwm_args pargs;
	const __be32 *addr;
	int rc = 0, id, i = 0;

	addr = of_get_address(chip->dev->of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(chip->dev, "Getting address failed\n");
		return -EINVAL;
	}
	chip->reg_base = be32_to_cpu(addr[0]);

	chip->num_leds = of_get_available_child_count(node);
	if (chip->num_leds == 0) {
		dev_err(chip->dev, "No led child node defined\n");
		return -ENODEV;
	}

	if (chip->num_leds > TRILED_NUM_MAX) {
		dev_err(chip->dev, "can't support %d leds(max %d)\n",
				chip->num_leds, TRILED_NUM_MAX);
		return -EINVAL;
	}

	if (of_find_property(chip->dev->of_node, "nvmem", NULL)) {
		chip->pbs_nvmem = devm_nvmem_device_get(chip->dev, "pbs_sdam");
		if (IS_ERR_OR_NULL(chip->pbs_nvmem)) {
			rc = PTR_ERR(chip->pbs_nvmem);
			if (rc != -EPROBE_DEFER) {
				dev_err(chip->dev, "Couldn't get nvmem device, rc=%d\n",
					rc);
				return -ENODEV;
			}
			chip->pbs_nvmem = NULL;
			return rc;
		}
	}

	chip->leds = devm_kcalloc(chip->dev, chip->num_leds,
			sizeof(struct qpnp_led_dev), GFP_KERNEL);
	if (!chip->leds)
		return -ENOMEM;

	for_each_available_child_of_node(node, child_node) {
		rc = of_property_read_u32(child_node, "led-sources", &id);
		if (rc) {
			dev_err(chip->dev, "Get led-sources failed, rc=%d\n",
							rc);
			return rc;
		}

		if (id >= TRILED_NUM_MAX) {
			dev_err(chip->dev, "only support 0~%d current source\n",
					TRILED_NUM_MAX - 1);
			return -EINVAL;
		}

		led = &chip->leds[i++];
		led->chip = chip;
		led->id = id;
		led->label =
			of_get_property(child_node, "label", NULL) ? :
							child_node->name;

		led->pwm_dev =
			devm_of_pwm_get(chip->dev, child_node, NULL);
		if (IS_ERR(led->pwm_dev)) {
			rc = PTR_ERR(led->pwm_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "Get pwm device for %s led failed, rc=%d\n",
							led->label, rc);
			return rc;
		}

		pwm_get_args(led->pwm_dev, &pargs);
		if (pargs.period == 0)
			led->pwm_setting.pre_period_ns = PWM_PERIOD_DEFAULT_NS;
		else
			led->pwm_setting.pre_period_ns = pargs.period;

		led->default_trigger = of_get_property(child_node,
				"linux,default-trigger", NULL);

#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00016 */
		qpnp_tri_led_calib_read_clrvari_param(led);
#endif /* CONFIG_LEDS_SHARP */
	}

	return rc;
}

static int qpnp_tri_led_probe(struct platform_device *pdev)
{
	struct qpnp_tri_led_chip *chip;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Getting regmap failed\n");
		return -EINVAL;
	}

	rc = qpnp_tri_led_parse_dt(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Devicetree properties parsing failed, rc=%d\n",
								rc);
		return rc;
	}

	mutex_init(&chip->bus_lock);

	rc = qpnp_tri_led_hw_init(chip);
	if (rc) {
		dev_err(chip->dev, "HW initialization failed, rc=%d\n", rc);
		goto destroy;
	}

	dev_set_drvdata(chip->dev, chip);
	rc = qpnp_tri_led_register(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Registering LED class devices failed, rc=%d\n",
								rc);
		goto destroy;
	}

	dev_dbg(chip->dev, "Tri-led module with subtype 0x%x is detected\n",
					chip->subtype);
	return 0;
destroy:
	mutex_destroy(&chip->bus_lock);
	dev_set_drvdata(chip->dev, NULL);

	return rc;
}

static int qpnp_tri_led_remove(struct platform_device *pdev)
{
	int i;
	struct qpnp_tri_led_chip *chip = dev_get_drvdata(&pdev->dev);
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
	struct qpnp_led_dev *led;
	struct led_setting *led_setting;
#endif /* CONFIG_LEDS_SHARP */

	mutex_destroy(&chip->bus_lock);
	for (i = 0; i < chip->num_leds; i++) {
#ifdef CONFIG_LEDS_SHARP /* CUST_ID_00036 */
		led = &chip->leds[i];
		if (pwm_get_output_type_supported(led->pwm_dev)){
			if ((led->id == QPNP_TRI_LED_RED) ||
			    (led->id == QPNP_TRI_LED_GREEN) ||
			    (led->id == QPNP_TRI_LED_BLUE)) {
				sysfs_remove_group(&chip->leds[i].cdev.dev->kobj,
							&lpg_attr_group);
				led_setting = &led->led_setting;
				if (!led_setting->duty_pcts_buf) {
					devm_kfree(&pdev->dev,
						led_setting->duty_pcts_buf);
				}
			}
		}
#endif /* CONFIG_LEDS_SHARP */

		sysfs_remove_files(&chip->leds[i].cdev.dev->kobj, breath_attrs);
		mutex_destroy(&chip->leds[i].lock);
	}
	dev_set_drvdata(chip->dev, NULL);
	return 0;
}

static const struct of_device_id qpnp_tri_led_of_match[] = {
	{ .compatible = "qcom,tri-led",},
	{ },
};

static struct platform_driver qpnp_tri_led_driver = {
	.driver		= {
		.name		= "qcom,tri-led",
		.of_match_table	= qpnp_tri_led_of_match,
	},
	.probe		= qpnp_tri_led_probe,
	.remove		= qpnp_tri_led_remove,
};
module_platform_driver(qpnp_tri_led_driver);

MODULE_DESCRIPTION("QTI TRI_LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:qpnp-tri-led");
