/* drivers/misc/shbatt/shbatt_qi_rx.c
 *
 * Copyright (C) 2018 SHARP CORPORATION All rights reserved.
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

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :                                                   |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/poll.h>
#include <linux/namei.h>
#include <linux/of.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#include "misc/shbatt_kerl.h"
#include "misc/shbatt_qi_rx.h"

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
#define QIRX_DEV_NAME						"shbatt_qi_rx"
#define QIRX_OF_DEV_NAME					"sharp,shbatt_qi_rx"

#define qirx_err(fmt, ...)		\
		pr_err("%s: " fmt,		\
		 __func__, ##__VA_ARGS__)	\

#define qirx_dbg(qirx, level, fmt, ...)			\
	do {		 				\
		if (*qirx->debug_mask & (level))	\
			pr_info("%s: " fmt,		\
			 __func__, ##__VA_ARGS__);	\
		else					\
			pr_debug("%s: " fmt,		\
			__func__, ##__VA_ARGS__);	\
	} while (0)

static const char * const qirx_state_strings[] = {
	"QIRX_IDLE",
	"QIRX_INHIBIT",
	"QIRX_SUSPEND",
	"QIRX_GUIDING",
	"QIRX_CHARGING",
	"QIRX_FULL",
	"QIRX_ERROR",
};

enum qirx_ept_code{
	EPT_CODE_UNKNOWN = 0x00,
	EPT_CODE_CHARGE_COMPLETE,
	EPT_CODE_INTERNAL_FAULT,
	EPT_CODE_OVER_TEMP,
	EPT_CODE_OVER_VOLTAGE,
	EPT_CODE_OVER_CURRENT,
	EPT_CODE_BATTERY_FAIL,
	EPT_CODE_RESERVED_1,
	EPT_CODE_NO_RESPONSE,
	EPT_CODE_RESERVED_2,
	EPT_CODE_NEGOTIATION_FAIL,
	EPT_CODE_RESTART_POWER_TRANSFER,
	//when the status is not EPT
	EPT_CODE_INVALID,
};

static const char * const qirx_ept_code_strings[] = {
	"EPT_CODE_UNKNOWN",
	"EPT_CODE_CHARGE_COMPLETE",
	"EPT_CODE_INTERNAL_FAULT",
	"EPT_CODE_OVER_TEMP",
	"EPT_CODE_OVER_VOLTAGE",
	"EPT_CODE_OVER_CURRENT",
	"EPT_CODE_BATTERY_FAIL",
	"EPT_CODE_RESERVED_1",
	"EPT_CODE_NO_RESPONSE",
	"EPT_CODE_RESERVED_2",
	"EPT_CODE_NEGOTIATION_FAIL",
	"EPT_CODE_RESTART_POWER_TRANSFER",
	"EPT_CODE_INVALID",
};

enum{
	QIRX_GPIO_PIN_VCC_EN = 0,
	QIRX_GPIO_PIN_RST,
	QIRX_GPIO_PIN_ADDDET_STATE,
	QIRX_GPIO_PIN_EN_N,
	QIRX_GPIO_PIN_INT_N,
	QIRX_GPIO_PIN_PG,
};

enum qirx_irqstat{
	QIRX_NONE_IRQ = 0,
	QIRX_QI_EPT_IRQ,
	QIRX_CHARGE_START_IRQ,
	QIRX_ERR_POSSET_CLR_IRQ,
	QIRX_ERR_POSSET_IRQ,
};

static const char * const qirx_irqstat_strings[] = {
	"QIRX_NONE_IRQ",
	"QIRX_QI_EPT_IRQ",
	"QIRX_CHARGE_START_IRQ",
	"QIRX_ERR_POSSET_CLR_IRQ",
	"QIRX_ERR_POSSET_IRQ",
};

struct qirx {
	struct	i2c_client *client;
	struct	device  *dev;
	struct	delayed_work qirx_rerun_aicl_loop_work;
	struct	workqueue_struct *wq;
	struct	work_struct sm_work;
	struct	work_struct int_n_irq_work;
	struct	work_struct guiding_work;
//	struct	work_struct start_periph_work;
	struct	hrtimer qirx_kick_sm_timer;
	struct	hrtimer	guiding_timer;
//	struct	hrtimer	qi_ept_count_timer;
	bool	sm_queued;
	bool	pg_state;
	bool	addet_state;
	bool	suspend_state;
//	struct	power_supply *qirx_psy;
//	struct	power_supply *main_psy;
	struct	notifier_block psy_nb;
	int	batt_psy_status;
	int	dc_psy_present;
	enum	qirx_state current_state;
	enum	qirx_ept_code ept_code;
	enum	qirx_irqstat irqstat;
	int		current_received_power; /* uW */
//	int		current_output_voltage; /* uV */
//	int		max_psns_reference_regulation; /* mA */
//	int		current_psns_reference_regulation; /* mA */
	unsigned int	qi_vcc_en_gpio;
	unsigned int	qi_rst_gpio;
	unsigned int	qi_addet_state_gpio;
	unsigned int	qi_en_gpio;
	unsigned int	qi_int_n_gpio;
	unsigned int	qi_pg_gpio;
	int	*debug_mask;

	struct	power_supply *wireless_psy;
};

static int qirx_debug_mask;
module_param_named(
    debug_mask, qirx_debug_mask, int, 0600
);

//QI OUTSET_FOR_EPP
#define QI_OUTSET_FOR_EPP_REG 0x02
#define QI_OUTSET_FOR_EPP_5V_VAL  0x80
#define QI_OUTSET_FOR_EPP_DEFAULT 0x05
//QI RSVD9 REG
#define QI_RSVD9_REG	0xA1
#define QI_RSVD9_VAL_DEFAULT	0xC0
#define QI_RSVD9_VAL	0xF0
//QI DPSET2 REG
#define QI_DPSET2_REG	0xAD
#define QI_DPSET2_VAL_DEFAULT	0x40
#define QI_DPSET2_VAL	0x4F

//QI ID NUM
enum qirx_qi_id{
	QIRX_ID_1912 = 0,
	QIRX_ID_2834,
	QIRX_ID_2574,
	QIRX_ID_2225,
	QIRX_ID_DCM,
	QIRX_ID_MAX,
};

//Qi manifacture code struct
struct mfc_code_config {
	u8	tx_id1;
	u8	tx_id2;
	u8	tx_conf1;
	u8	tx_conf2;
	u8	mfc_epp_data;
	u8	mfc_rsvd9_data;
	u8	mfc_dpset2_data;
};

//Qi manifacture code table
static struct mfc_code_config qi_id_table[QIRX_ID_MAX + 1] = {
	/* tx_id1, tx_id2, tx_conf1, tx_conf2, OUTSET_FOR_EPP DATA, RSVD9 DATA, DPSET2 DATA */
	{0x00, 0x32, 0x1E, 0x1E, QI_OUTSET_FOR_EPP_5V_VAL, QI_RSVD9_VAL,         QI_DPSET2_VAL         },//QI-ID 1912
	{0x28, 0x00, 0x14, 0x14, QI_OUTSET_FOR_EPP_5V_VAL, QI_RSVD9_VAL_DEFAULT, QI_DPSET2_VAL_DEFAULT },//QI-ID 2834
	{0x00, 0x5D, 0x10, 0x10, QI_OUTSET_FOR_EPP_DEFAULT,QI_RSVD9_VAL_DEFAULT, QI_DPSET2_VAL_DEFAULT },//Qi-ID 2574
	{0x00, 0x5E, 0x10, 0x10, QI_OUTSET_FOR_EPP_DEFAULT,QI_RSVD9_VAL_DEFAULT, QI_DPSET2_VAL_DEFAULT },//Qi-ID 2225
	{0x00, 0x27, 0x1E, 0x1E, QI_OUTSET_FOR_EPP_DEFAULT,QI_RSVD9_VAL_DEFAULT, QI_DPSET2_VAL_DEFAULT },//QI-ID DCM
	{0x00, 0x00, 0x00, 0x00, QI_OUTSET_FOR_EPP_DEFAULT,QI_RSVD9_VAL_DEFAULT, QI_DPSET2_VAL_DEFAULT },//Qi-ID Other
};

static struct power_supply *batt_psy;
static struct power_supply *dc_psy;
static struct i2c_client *the_client = NULL;
static struct qirx *the_qirx = NULL;
static int32_t int_n_irqNo;
static int32_t pg_irqNo;
static int32_t addet_irqNo;
//static int qi_ept_count = 0;
static int polling_count;
static int maximum_power_value;
static int before_maximum_power_value = 0;
static int32_t int_n_irqNo;
static int32_t pg_irqNo;
static int aicl_current_icl = 0;
static bool icl_settled = false;
static int calibration_count = 0;
static bool qirx_charging_flag_now = false;
static bool qirx_id_1912_det_flag = false;
static bool qirx_id_check_flag = false;
static int taper_aicl_const = 0;
static int pred_volt_const = 0;
static bool moni_mode_get_complete_flag = false;
static bool epp_mode_flag = false;
static int qirx_psns_stability_current = 0;
static int qirx_step_current = 0;
static bool qirx_fod_set_flag = false;
static int moni_mode_set_state = 0;

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE PROTO TYPE DECLARE :                               |*/
/*+-----------------------------------------------------------------------------+*/
/* attribute store */
static int32_t qirx_probe( struct i2c_client *client, const struct i2c_device_id *id);
static int32_t qirx_remove( struct i2c_client *client );
static int32_t qirx_suspend( struct device *dev );
static int32_t qirx_resume( struct device *dev );
static void    qirx_shutdown( struct i2c_client *client );
int qirx_get_received_power( struct i2c_client *client, union power_supply_propval *val);
int qirx_get_frequency( struct i2c_client *client, union power_supply_propval *val);
static int32_t qirx_write_regdata_single(struct i2c_client *client, unsigned short addr, unsigned short data_first);
static int32_t qirx_read_regdata(struct i2c_client *client, unsigned short addr, unsigned short *data);
int qirx_get_outset_voltage(struct i2c_client *client,union power_supply_propval *val);
int qirx_set_outset_voltage(struct i2c_client *client,const union power_supply_propval *val);
int qirx_qi_suspend_get_value(struct qirx *qirx, union power_supply_propval *val);
int qirx_qi_suspend_set_value(struct qirx *qirx, const union power_supply_propval *val);
int qirx_calculate_max_current_ua(struct i2c_client *client,union power_supply_propval *val);
int qirx_get_psns_current_icl(struct i2c_client *client,union power_supply_propval *val);
int qirx_get_received_maximum_power( struct i2c_client *client, union power_supply_propval *val);
int qirx_get_ept_code(struct i2c_client * client);
int qirx_is_power_good(void);
int qirx_is_addet(void);
static void qirx_set_state(enum qirx_state next_state);
static int __init qirx_init( void );
static void __exit qirx_exit( void );
/*+-----------------------------------------------------------------------------+*/
/*| @ FUNCTION TABLE PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
#ifdef CONFIG_OF
static struct of_device_id qirx_match_table[] = {
	{ .name = QIRX_DEV_NAME,},
	{ .compatible = QIRX_OF_DEV_NAME, },
	{ },
};
#else
#define qirx_match_table NULL;
#endif /* CONFIG_OF */

static const struct i2c_device_id qirx_i2c_id[] = {
	{ QIRX_DEV_NAME, 0 },
	{ }
};

static const struct dev_pm_ops qirx_ops = {
    .suspend     = qirx_suspend,
    .resume      = qirx_resume,
};

MODULE_DEVICE_TABLE(i2c, qirx_i2c_id);

static struct i2c_driver qirx_interface_driver = {
    .driver = {
        .name    = QIRX_DEV_NAME,
        .owner   = THIS_MODULE,
        .pm      = &qirx_ops,
        .of_match_table = qirx_match_table,
    },
    .probe       = qirx_probe,
    .remove      = qirx_remove,
    .shutdown    = qirx_shutdown,
    .id_table    = qirx_i2c_id,
};

static struct wake_lock qirx_irq_wake_lock;
static spinlock_t qirx_wake_spinlock;

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/

static enum power_supply_property qirx_props[] = {
	POWER_SUPPLY_PROP_QI_STATUS,
	POWER_SUPPLY_PROP_QI_EPT_CODE,
	POWER_SUPPLY_PROP_QI_SUSPEND,
	POWER_SUPPLY_PROP_QI_RECEIVED_POWER,
	POWER_SUPPLY_PROP_QI_FREQUENCY,
	POWER_SUPPLY_PROP_QI_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_QI_CURRENT_MAX,
	POWER_SUPPLY_PROP_QI_CURRENT_NOW,
#ifdef PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION
	POWER_SUPPLY_PROP_QI_INPUT_VOLTAGE_REGULATION,
#endif /* PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION */
	POWER_SUPPLY_PROP_QI_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_QI_MAXIMUM_POWER,
	POWER_SUPPLY_PROP_QI_ONLINE,
};

static int qirx_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_QI_SUSPEND:
#ifdef PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION
	case POWER_SUPPLY_PROP_QI_INPUT_VOLTAGE_REGULATION:
#endif /* PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION */
		return 1;
	default:
		break;
	}

	return 0;
}

static int qirx_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_QI_STATUS:
		val->strval = qirx_state_strings[the_qirx->current_state];
		break;
	case POWER_SUPPLY_PROP_QI_EPT_CODE:
		rc = qirx_get_ept_code(the_client);
		val->strval = qirx_ept_code_strings[the_qirx->ept_code];
		break;
	case POWER_SUPPLY_PROP_QI_SUSPEND:
		rc = qirx_qi_suspend_get_value(the_qirx, val);
		break;
	case POWER_SUPPLY_PROP_QI_RECEIVED_POWER:
		rc = qirx_get_received_power(the_client, val);
		break;
	case POWER_SUPPLY_PROP_QI_FREQUENCY:
		rc = qirx_get_frequency(the_client, val);
		break;
	case POWER_SUPPLY_PROP_QI_VOLTAGE_NOW:
		rc = qirx_get_outset_voltage(the_client, val);
		break;
	case POWER_SUPPLY_PROP_QI_CURRENT_MAX:
		rc = qirx_calculate_max_current_ua(the_client, val);
		break;
	case POWER_SUPPLY_PROP_QI_CURRENT_NOW:
		rc = qirx_get_psns_current_icl(the_client, val);
		break;
#ifdef PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION
	case POWER_SUPPLY_PROP_QI_INPUT_VOLTAGE_REGULATION:
		rc = qirx_get_outset_voltage(the_client, val);
		break;
#endif /* PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION */
	case POWER_SUPPLY_PROP_QI_VOLTAGE_MAX:
		rc = qirx_get_outset_voltage(the_client, val);
		break;
	case POWER_SUPPLY_PROP_QI_MAXIMUM_POWER:
		rc = qirx_get_received_maximum_power(the_client, val);
		break;
	case POWER_SUPPLY_PROP_QI_ONLINE:
		val->intval = (( the_qirx->pg_state == true ) ||
						(( the_qirx->current_state == QIRX_CHARGING ) || ( the_qirx->current_state == QIRX_FULL ))) ? 1 : 0;
		break;
	default:
		qirx_dbg(the_qirx, QIRX_DEBUG, "%d is not supported in qirx\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int qirx_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_QI_SUSPEND:
		rc = qirx_qi_suspend_set_value(the_qirx, val);
		break;
#ifdef PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION
	case POWER_SUPPLY_PROP_QI_INPUT_VOLTAGE_REGULATION:
		rc = qirx_set_outset_voltage(the_client, val);
		break;
#endif /* PM_SUPPORT_QI_INPUT_VOLTAGE_REGULATION */
	default:
		qirx_dbg(the_qirx, QIRX_DEBUG, "set prop %d is not supported\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

int qirx_set_gpio_no( struct qirx *qirx )
{
    struct device_node *np = qirx->dev->of_node;

	qirx->qi_vcc_en_gpio      = of_get_named_gpio(np, "qirx_vcc_en_gpio",     0);
	qirx->qi_rst_gpio         = of_get_named_gpio(np, "qirx_rst_gpio",        0);
	qirx->qi_addet_state_gpio = of_get_named_gpio(np, "qirx_addet_state_gpio",0);
	qirx->qi_en_gpio          = of_get_named_gpio(np, "qirx_en_gpio",         0);
	qirx->qi_int_n_gpio       = of_get_named_gpio(np, "qirx_int_n_gpio",      0);
	qirx->qi_pg_gpio          = of_get_named_gpio(np, "qirx_pg_gpio",         0);

    return 0;
}

int qirx_get_gpio_no(struct qirx *qirx, int gpio)
{
	int gpio_no = -1;

	switch(gpio){
	case QIRX_GPIO_PIN_VCC_EN:
		gpio_no = qirx->qi_vcc_en_gpio;
		break;
	case QIRX_GPIO_PIN_RST:
		gpio_no = qirx->qi_rst_gpio;
		break;
	case QIRX_GPIO_PIN_ADDDET_STATE:
		gpio_no = qirx->qi_addet_state_gpio;
		break;
	case QIRX_GPIO_PIN_EN_N:
		gpio_no = qirx->qi_en_gpio;
		break;
	case QIRX_GPIO_PIN_INT_N:
		gpio_no = qirx->qi_int_n_gpio;
		break;
	case QIRX_GPIO_PIN_PG:
		gpio_no = qirx->qi_pg_gpio;
		break;

	default:
		break;
	}

	return gpio_no;
}

int qirx_gpio_request(struct qirx *qirx, int gpio)
{
    int ret = 0;
    struct pinctrl_state *pin_state = NULL;
    struct pinctrl *pin;
    int gpio_no;
	struct	device *dev = qirx->dev;

    gpio_no = qirx_get_gpio_no(qirx, gpio);
    pin = devm_pinctrl_get(dev);

    switch(gpio){
	case QIRX_GPIO_PIN_VCC_EN:
		ret = gpio_request(gpio_no, "qirx_vcc_en");
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_vcc_en_active");
		break;
	case QIRX_GPIO_PIN_RST:
		ret = gpio_request(gpio_no, "qirx_reset");
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_rst_active");
		break;
	case QIRX_GPIO_PIN_ADDDET_STATE:
		ret = gpio_request(gpio_no, "qirx_addet");
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_addet_state_active");
		break;
	case QIRX_GPIO_PIN_EN_N:
		ret = gpio_request(gpio_no, "qirx_en");
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_en1_active");
		break;
	case QIRX_GPIO_PIN_INT_N:
		ret = gpio_request(gpio_no, "qirx_int_n");
		pin_state = pinctrl_lookup_state(pin, "sharp_qi_int_n_active");
		break;
	case QIRX_GPIO_PIN_PG:
		ret = gpio_request(gpio_no, "qirx_pg");
		pin_state = pinctrl_lookup_state(pin, "sharp_qi_pg_active");
		break;
	default:
		break;
	}

	if (ret < 0){
		qirx_err("gpio request error(gpio %d, ret=%d)", gpio, ret);
		return ret;
	}

	if(pin_state == NULL){
		qirx_err("Null error(gpio %d)", gpio);
		return -1;
	}

	ret = pinctrl_select_state(pin, pin_state);
	if(ret){
		qirx_err("gpio request error(gpio %d, ret=%d)", gpio, ret);
		return ret;
	}

    return ret;
}

int qirx_gpio_free(struct qirx *qirx, int gpio)
{
    int ret = 0;
    struct pinctrl_state *pin_state = NULL;
    struct pinctrl *pin;
    int gpio_no;
	struct	device *dev = qirx->dev;

    gpio_no = qirx_get_gpio_no(qirx, gpio);
    pin = devm_pinctrl_get(dev);

    switch(gpio){
	case QIRX_GPIO_PIN_VCC_EN:
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_vcc_en_suspend");
		gpio_free(gpio_no);
		break;
	case QIRX_GPIO_PIN_RST:
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_rst_suspend");
		gpio_free(gpio_no);
		break;
	case QIRX_GPIO_PIN_ADDDET_STATE:
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_addet_state_suspend");
		gpio_free(gpio_no);
		break;
	case QIRX_GPIO_PIN_EN_N:
		pin_state = pinctrl_lookup_state(pin, "pm8150b_qi_en1_suspend");
		gpio_free(gpio_no);
		break;
	case QIRX_GPIO_PIN_INT_N:
		pin_state = pinctrl_lookup_state(pin, "sharp_qi_int_n_suspend");
		gpio_free(gpio_no);
		break;
	case QIRX_GPIO_PIN_PG:
		pin_state = pinctrl_lookup_state(pin, "sharp_qi_pg_suspend");
		gpio_free(gpio_no);
		break;
	default:
		break;
	}

	if(pin_state == NULL){
		qirx_err("Null error(gpio %d)", gpio);
		return -1;
	}

	ret = pinctrl_select_state(pin, pin_state);
	if(ret){
		qirx_err("gpio_free error(gpio %d, ret=%d)", gpio, ret);
		return ret;
	}

	return ret;
}

int qirx_gpio_direction_output(struct qirx *qirx, int gpio, int data)
{
	int gpio_no;

	gpio_no = qirx_get_gpio_no(qirx, gpio);
	gpio_direction_output(gpio_no, data);
	return 0;
}

int qirx_gpio_direction_input(struct qirx *qirx, int gpio)
{
	int gpio_no;

	gpio_no = qirx_get_gpio_no(qirx, gpio);
	gpio_direction_input(gpio_no);
	return 0;
}

int qirx_gpio_set_value(struct qirx *qirx, int gpio, int data)
{
    int gpio_no;

    gpio_no = qirx_get_gpio_no(qirx, gpio);
    gpio_set_value(gpio_no, data);
    return 0;
}

int qirx_gpio_get_value(struct qirx *qirx, int gpio)
{
    int ret;
    int gpio_no;

    gpio_no = qirx_get_gpio_no(qirx, gpio);
    ret = gpio_get_value(gpio_no);
    return ret;
}

int qirx_qi_suspend_get_value(struct qirx *qirx, union power_supply_propval *val)
{
	int gpio_state = 0;
	int result = 0;
	bool qi_suspend_state = false;

	result = qirx_gpio_request(qirx, QIRX_GPIO_PIN_EN_N);
	if (result < 0){
		qirx_err("QIRX_GPIO_PIN_EN_N request Error.\n");
		goto ERROR;
	}

	gpio_state = qirx_gpio_get_value(qirx, QIRX_GPIO_PIN_EN_N);
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx pin_en_n gpio_state = %d\n", gpio_state);

	if(gpio_state > 0)
		qi_suspend_state = true;

ERROR:
    qirx_gpio_free(qirx, QIRX_GPIO_PIN_EN_N);

	val->intval = (int)qi_suspend_state;

	return result;
}

int qirx_qi_suspend_set_value(struct qirx *qirx, const union power_supply_propval *val)
{
	int result = 0;

	if(val->intval){
		result = qirx_gpio_request(qirx, QIRX_GPIO_PIN_EN_N);
		if (result < 0){
			qirx_err("QIRX_GPIO_PIN_EN_N request Error.\n");
		}else{
			qirx_gpio_set_value(qirx, QIRX_GPIO_PIN_EN_N, val->intval);
			qirx_dbg(qirx, QIRX_DEBUG, "qirx pin_en_n val = %d\n", val->intval);
		}
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_EN_N);

		qirx->suspend_state = true;

		if(the_qirx->current_state != QIRX_INHIBIT)
			qirx_set_state(QIRX_SUSPEND);

	}else{
		if(the_qirx->current_state == QIRX_CHARGING || the_qirx->current_state == QIRX_GUIDING || the_qirx->current_state == QIRX_FULL){
			qirx->suspend_state = false;
		}else{
			result = qirx_gpio_request(qirx, QIRX_GPIO_PIN_EN_N);
			if (result < 0){
				qirx_err("QIRX_GPIO_PIN_EN_N request Error.\n");
			}else{
				qirx_gpio_set_value(qirx, QIRX_GPIO_PIN_EN_N, val->intval);
				qirx_dbg(qirx, QIRX_DEBUG, "qirx pin_en_n val = %d\n", val->intval);
			}
			qirx_gpio_free(qirx, QIRX_GPIO_PIN_EN_N);

			qirx->suspend_state = false;
			if(the_qirx->addet_state || the_qirx->current_state == QIRX_INHIBIT){
				qirx_set_state(QIRX_INHIBIT);
			}else{
				qirx_set_state(QIRX_IDLE);
			}
		}
	}

	qirx_dbg(qirx, QIRX_DEBUG, "qirx->suspend_state = %d, set_state =%s\n", qirx->suspend_state, qirx_state_strings[the_qirx->current_state]);

	return result;
}

#define MONI_MODE_REG 0x52
#define MONI_MODE_OPERATION_MASK 0x01
#define MONI_MODE_OPERATION_BPP 0x00
#define MONI_MODE_OPERATION_EPP 0x01
int qirx_is_qi_epp_mode(struct i2c_client *client, bool *mode)
{
	int result = 0;
	unsigned short addr = 0x00;
	unsigned short data = 0x00;

	//MONI_MODE Read
	addr = MONI_MODE_REG;
	result = qirx_read_regdata(client, addr, &data);
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n", addr,data);

	if(result < 0)
		return result;

	if ( data & MONI_MODE_OPERATION_MASK )
		*mode = true;
	else
		*mode = false;

	return result;
}

#define EPT_CODE_REG 0x0E
#define NO_EPT_VAL 0x0C
int qirx_get_ept_code(struct i2c_client * client)
{
	int result = 0;
	unsigned short addr = 0x00;
	unsigned short data = 0x00;

	addr = EPT_CODE_REG;

	result = qirx_read_regdata(client, addr, &data);
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n", addr,data);

	if(result < 0){
		qirx_err("Failed, get EPT code. result = %d\n", result);
		return result;
	}

	if(data >= NO_EPT_VAL){
		the_qirx -> ept_code = EPT_CODE_INVALID;
	}else{
		the_qirx -> ept_code = data;
	}

	return result;
}

#define BPPSET_SET_REG   0x00
#define BPPSET_STATE_REG 0x01
#define EPPSET_SET_REG   0x02
#define EPPSET_STATE_REG 0x03
#define OUTSET_STATE_MASK 0x07
#define OUTSET_BIT_5P0V  0x00
#define OUTSET_BIT_5P3V  0x01
#define OUTSET_BIT_6P0V  0x02
#define OUTSET_BIT_7P0V  0x03
#define OUTSET_BIT_9P0V  0x04
#define OUTSET_BIT_10P0V 0x05
#define OUTSET_BIT_11P0V 0x06
#define OUTSET_BIT_12P0V 0x07
#define OUTSET_VALUE_5P0V  5000000
#define OUTSET_VALUE_5P3V  5300000
#define OUTSET_VALUE_6P0V  6000000
#define OUTSET_VALUE_7P0V  7000000
#define OUTSET_VALUE_9P0V  9000000
#define OUTSET_VALUE_10P0V 10000000
#define OUTSET_VALUE_11P0V 11000000
#define OUTSET_VALUE_12P0V 12000000
int qirx_get_outset_voltage(struct i2c_client *client, union power_supply_propval *val)
{
	int result = 0;
	unsigned short addr = 0x00;
	unsigned short data = 0x00;
	int temp = 0;
	bool mode = false;

	if(the_qirx->pg_state && moni_mode_set_state == 0){

		//Get moni mode
		if(!moni_mode_get_complete_flag){
			result = qirx_is_qi_epp_mode(client,&mode);
			if(result < 0){
				qirx_err("qirx_is_qi_epp_mode error.\n");
			}
		}else{
			mode = epp_mode_flag;
		}

		if(mode){
			addr = EPPSET_STATE_REG;
			result = qirx_read_regdata(client, addr, &data);
			temp = (int)(data & OUTSET_STATE_MASK);
		}else{
			addr = BPPSET_STATE_REG;
			result = qirx_read_regdata(client, addr, &data);
			temp = (int)(data & OUTSET_STATE_MASK);
		}

		qirx_dbg(the_qirx, QIRX_DEBUG, "result = %d, addr = 0x%02x, data = 0x%02x, mode = %d, temp = %d\n",result,addr,data,mode,temp);

		switch (temp) {
			case OUTSET_BIT_5P0V:
				moni_mode_set_state = OUTSET_VALUE_5P0V;
				break;
			case OUTSET_BIT_5P3V:
				moni_mode_set_state = OUTSET_VALUE_5P3V;
				break;
			case OUTSET_BIT_6P0V:
				moni_mode_set_state = OUTSET_VALUE_6P0V;
				break;
			case OUTSET_BIT_7P0V:
				moni_mode_set_state = OUTSET_VALUE_7P0V;
				break;
			case OUTSET_BIT_9P0V:
				moni_mode_set_state = OUTSET_VALUE_9P0V;
				break;
			case OUTSET_BIT_10P0V:
				moni_mode_set_state = OUTSET_VALUE_10P0V;
				break;
			case OUTSET_BIT_11P0V:
				moni_mode_set_state = OUTSET_VALUE_11P0V;
				break;
			case OUTSET_BIT_12P0V:
				moni_mode_set_state = OUTSET_VALUE_12P0V;
				break;
			default:
				break;
		}
	}

	val->intval = moni_mode_set_state;
	qirx_dbg(the_qirx, QIRX_DEBUG, "moni_mode_set_state =  = %d\n", moni_mode_set_state);

	return result;
}

int qirx_set_outset_voltage(struct i2c_client *client, const union power_supply_propval *val)
{
	int result = 0;
	unsigned short write_addr = 0x00;
	unsigned short write_data_first = 0x00;
	int temp = 0;
	bool mode = false;
	unsigned short data = 0x00;

	switch (val->intval) {
		case OUTSET_VALUE_5P0V:
			temp = OUTSET_BIT_5P0V;
			break;
		case OUTSET_VALUE_5P3V:
			temp = OUTSET_BIT_5P3V;
			break;
		case OUTSET_VALUE_6P0V:
			temp = OUTSET_BIT_6P0V;
			break;
		case OUTSET_VALUE_7P0V:
			temp = OUTSET_BIT_7P0V;
			break;
		case OUTSET_VALUE_9P0V:
			temp = OUTSET_BIT_9P0V;
			break;
		case OUTSET_VALUE_10P0V:
			temp = OUTSET_BIT_10P0V;
			break;
		case OUTSET_VALUE_11P0V:
			temp = OUTSET_BIT_11P0V;
			break;
		case OUTSET_VALUE_12P0V:
			temp = OUTSET_BIT_12P0V;
			break;
		default:
			temp = -1;
			break;
	}

	if(temp < 0){
		qirx_err("outset voltage set range error. temp = %d.\n", temp);
		return result;
	}

	//Get moni mode
	if(!moni_mode_get_complete_flag){
		result = qirx_is_qi_epp_mode(client,&mode);
		if(result < 0){
			qirx_err("qirx_is_qi_epp_mode error.\n");
		}
	}else{
		mode = epp_mode_flag;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "result = %d, mode = %d, temp = %d\n",result,mode,temp);

	if(mode)
		write_addr = EPPSET_SET_REG;	//0x02
	else
		write_addr = BPPSET_SET_REG;	//0x00

	write_data_first = temp;
	result = qirx_write_regdata_single(client, write_addr, write_data_first);

	qirx_dbg(the_qirx, QIRX_DEBUG, "result = %d, write_addr = 0x%02x, write_data_first = 0x%02x\n",result,write_addr,write_data_first);

	if(result < 0){
		qirx_err("outset_voltage write error.result = %d\n", result);
		return result;
	}

	//for moni mode set value check
	if(the_qirx->pg_state){
		result = qirx_read_regdata(client, write_addr, &data);
		qirx_dbg(the_qirx, QIRX_DEBUG, "read result = %d, addr = 0x%02x, data = 0x%02x, mode = %d, temp = %d\n",result,write_addr,data,mode,temp);

		if(temp == data){
			switch (temp) {
				case OUTSET_BIT_5P0V:
					moni_mode_set_state = OUTSET_VALUE_5P0V;
					break;
				case OUTSET_BIT_5P3V:
					moni_mode_set_state = OUTSET_VALUE_5P3V;
					break;
				case OUTSET_BIT_6P0V:
					moni_mode_set_state = OUTSET_VALUE_6P0V;
					break;
				case OUTSET_BIT_7P0V:
					moni_mode_set_state = OUTSET_VALUE_7P0V;
					break;
				case OUTSET_BIT_9P0V:
					moni_mode_set_state = OUTSET_VALUE_9P0V;
					break;
				case OUTSET_BIT_10P0V:
					moni_mode_set_state = OUTSET_VALUE_10P0V;
					break;
				case OUTSET_BIT_11P0V:
					moni_mode_set_state = OUTSET_VALUE_11P0V;
					break;
				case OUTSET_BIT_12P0V:
					moni_mode_set_state = OUTSET_VALUE_12P0V;
					break;
				default:
					break;
			}
			qirx_dbg(the_qirx, QIRX_DEBUG, "moni_mode_set_state = %d\n", moni_mode_set_state);
		}
	}

	return result;
}

int qirx_get_psns_current_icl(struct i2c_client *client,union power_supply_propval *val)
{
	int result = 0;
	union power_supply_propval pval;

	dc_psy = power_supply_get_by_name("dc");
	result = power_supply_get_property(dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &pval);

	if(result < 0){
		qirx_err("get error:POWER_SUPPLY_PROP_CURRENT_MAX\n");
		return result;
	}

	val->intval = pval.intval;

	return result;
}
#define SHOW_DIGIT	10000
#define VOLTAGE_DIV 1000000
#define MA_SHOW 1000000
#define CALCULATE_MAX_CURRENT_VAL_700MA	700000
int qirx_calculate_max_current_ua(struct i2c_client *client,union power_supply_propval *val)
{
	int result = 0;
	int received_power_val = 0;
	int outset_voltage_val = 0;
	double calculate_max_current_val = 0.0;
	union power_supply_propval pval = {0,};

	result = qirx_get_received_maximum_power(client, &pval);
	received_power_val = pval.intval;

	qirx_dbg(the_qirx, QIRX_DEBUG, "received_power = %d\n", received_power_val);

	result = qirx_get_outset_voltage(client, &pval);
	outset_voltage_val = pval.intval;

	qirx_dbg(the_qirx, QIRX_DEBUG, "outset_voltage = %d\n", outset_voltage_val);

	calculate_max_current_val = (double)(received_power_val/SHOW_DIGIT)/(double)(outset_voltage_val/VOLTAGE_DIV);
	val->intval = (int)(calculate_max_current_val * MA_SHOW);

	if(val->intval > CALCULATE_MAX_CURRENT_VAL_700MA){
		val->intval = CALCULATE_MAX_CURRENT_VAL_700MA;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "current_max = %d\n", val->intval);

	return result;
}

int qirx_reset(struct qirx *qirx)
{
	int gpio_state = 0;
	int ret = 0;

	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_RST);
	if (ret < 0){
		qirx_err("QIRX_GPIO_PIN_RST request Error.\n");
		goto ERROR;
	}

	gpio_state = qirx_gpio_get_value(qirx, QIRX_GPIO_PIN_RST);
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx rst gpio_state = %d\n",gpio_state);

	qirx_gpio_set_value(qirx, QIRX_GPIO_PIN_RST,0);	//Qi RST 0
	qirx_gpio_set_value(qirx, QIRX_GPIO_PIN_RST,1);	//Qi RST 1
	udelay(10);	//10us wait
	qirx_gpio_set_value(qirx, QIRX_GPIO_PIN_RST,0);	//Qi RST 0
	mdelay(1);	//1ms wait

	gpio_state = qirx_gpio_get_value(qirx, QIRX_GPIO_PIN_RST);
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx rst gpio_state = %d\n",gpio_state);

ERROR:
    qirx_gpio_free(qirx, QIRX_GPIO_PIN_RST);

	return ret;
}

int qirx_gpio_init(struct qirx *qirx)
{
	int ret = 0;

	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_VCC_EN);
	if (ret < 0){
		qirx_err("QIRX_GPIO_PIN_VCC_EN request Error.\n");
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_VCC_EN);
		return -ENODEV;
	}
	qirx_gpio_direction_output(qirx,QIRX_GPIO_PIN_VCC_EN,1);

	qirx_gpio_free(qirx, QIRX_GPIO_PIN_VCC_EN);

	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_RST);
	if (ret < 0){
		qirx_err("QIRX_GPIO_PIN_RST request Error.\n");
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_RST);
		return -ENODEV;
	}
	qirx_gpio_direction_output(qirx,QIRX_GPIO_PIN_RST,0);

	qirx_gpio_free(qirx, QIRX_GPIO_PIN_RST);

	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_ADDDET_STATE);
	if (ret < 0){
		qirx_err("QIRX_GPIO_PIN_ADDDET_STATE request Error.\n");
		qirx_gpio_free(qirx,QIRX_GPIO_PIN_ADDDET_STATE);
		return -ENODEV;
	}
	qirx_gpio_direction_input(qirx,QIRX_GPIO_PIN_ADDDET_STATE);

	qirx_gpio_free(qirx,QIRX_GPIO_PIN_ADDDET_STATE);

	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_EN_N);
	if (ret < 0){
		qirx_err("QIRX_GPIO_PIN_EN_N request Error.\n");
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_EN_N);
		return -ENODEV;
	}
	qirx_gpio_direction_output(qirx,QIRX_GPIO_PIN_EN_N,1);

	qirx_gpio_free(qirx, QIRX_GPIO_PIN_EN_N);

	return 0;
}

static const struct power_supply_desc wireless_psy_desc = {
	 .name = "wireless",
	 .type = POWER_SUPPLY_TYPE_WIRELESS,
	 .properties = qirx_props,
	 .num_properties = ARRAY_SIZE(qirx_props),
	 .get_property = qirx_get_prop,
	 .set_property = qirx_set_prop,
	 .property_is_writeable = qirx_prop_is_writeable,
};

static int qirx_init_psy(struct qirx *qirx)
{
	struct power_supply_config wireless_cfg = {};
	struct	device *dev = qirx->dev;
	int rc = 0;

	wireless_cfg.of_node = dev->of_node;
	wireless_cfg.drv_data = qirx;
	wireless_cfg.num_supplicants = 0;
	wireless_cfg.supplied_to = NULL;
	qirx->wireless_psy = devm_power_supply_register(dev,
						&wireless_psy_desc,
						&wireless_cfg);
	if (IS_ERR(qirx->wireless_psy)) {
		qirx_err("Couldn't register wireless power supply\n");
		return PTR_ERR(qirx->wireless_psy);
	}

	return rc;
}

static int32_t qirx_write_regdata_single(struct i2c_client *client, unsigned short addr, unsigned short data_first)
{
	int ret = 0;
	uint8_t write_buf[] = {0x00,0x00};

	write_buf[0] = addr;
	write_buf[1] = data_first;

	ret = i2c_master_send( client, write_buf, ARRAY_SIZE(write_buf) );

	qirx_dbg(the_qirx, QIRX_INFO, "addr  = 0x%02x\n",write_buf[0]);
	qirx_dbg(the_qirx, QIRX_INFO, "write data  = 0x%02x\n",write_buf[1]);
	qirx_dbg(the_qirx, QIRX_INFO, "write size  = 0x%02x\n",ARRAY_SIZE(write_buf));
	qirx_dbg(the_qirx, QIRX_INFO, "ret         = %d\n",ret);

	if(ret < 0){
		qirx_err("Write Error addr = 0x%02x , ret = %d\n",write_buf[0],ret);
		return ret;
	}

	return ret;
}

static int32_t qirx_write_regdata(struct i2c_client *client, unsigned short addr, unsigned short data_first, unsigned short data_second)
{
	int ret = 0;
	uint8_t write_buf[] = {0x00,0x00,0x00};

	write_buf[0] = addr;
	write_buf[1] = data_first;
	write_buf[2] = data_second;

	ret = i2c_master_send( client, write_buf, ARRAY_SIZE(write_buf) );

	qirx_dbg(the_qirx, QIRX_DEBUG, "addr  = 0x%02x\n",write_buf[0]);
	qirx_dbg(the_qirx, QIRX_DEBUG, "write data  = 0x%02x\n",write_buf[1]);
	qirx_dbg(the_qirx, QIRX_DEBUG, "write data2 = 0x%02x\n",write_buf[2]);
	qirx_dbg(the_qirx, QIRX_DEBUG, "write size  = 0x%02x\n",ARRAY_SIZE(write_buf));
	qirx_dbg(the_qirx, QIRX_DEBUG, "ret         = %d\n",ret);

	if(ret < 0){
		qirx_err("Write Error addr = 0x%02x , ret = %d\n",write_buf[0],ret);
		return ret;
	}

	return ret;
}

static int32_t qirx_read_regdata(struct i2c_client *client, unsigned short addr, unsigned short *data)
{
	int ret = 0;
	unsigned short val = 0x00;

	ret = i2c_master_send( client, (char *)&addr, 1 );
	if (ret < 0)
		qirx_err("Error %d qirx send to subaddress 0x%x\n", ret, addr);

	ret = i2c_master_recv( client, (char *)&val, 1 );
	if (ret < 0)
		qirx_err("Error %d qirx read to subaddress 0x%x\n", ret, addr);

	*data = val;

	return ret;
}

static int qirx_update_power_supply(struct power_supply *psy, int ua)
{
	int rc = 0;
	union power_supply_propval val;

	val.intval = ua;
	rc = power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val);

	if(rc){
		qirx_err("update error:POWER_SUPPLY_PROP_CURRENT_MAX\n");
	}

	return rc;
}

int qirx_gpio_to_irq(struct qirx *qirx, int gpio)
{
	int gpio_no = 0;
	int ret  = 0;

	gpio_no = qirx_get_gpio_no(qirx, gpio);
	ret = gpio_to_irq(gpio_no);
	return ret;
}

static void qirx_wake_lock_init(void)
{
	spin_lock_init(&qirx_wake_spinlock);
	wake_lock_init(&qirx_irq_wake_lock, WAKE_LOCK_SUSPEND, "qirx_irq_wake_lock");
	return;
}

static void qirx_wake_lock_destroy(void)
{
	if (wake_lock_active(&qirx_irq_wake_lock)){
		wake_unlock(&qirx_irq_wake_lock);
	}
	wake_lock_destroy(&qirx_irq_wake_lock);
	return;
}

static void qirx_wake_lock_start(struct wake_lock *wl)
{
	unsigned long flags;

	spin_lock_irqsave(&qirx_wake_spinlock, flags);

	if(strcmp(wl->ws.name,"qirx_irq_wake_lock")){
		if (!wake_lock_active(wl)){
			wake_lock(wl);
		}
	}

	spin_unlock_irqrestore(&qirx_wake_spinlock, flags);

	return;
}

static void qirx_wake_lock_end(struct wake_lock *wl)
{
	unsigned long flags;

	spin_lock_irqsave(&qirx_wake_spinlock, flags);

	if(strcmp(wl->ws.name,"qirx_irq_wake_lock")){
		if (wake_lock_active(wl)){
			wake_unlock(wl);
		}
	}

	spin_unlock_irqrestore(&qirx_wake_spinlock, flags);

	return;
}

#define QI_INTSTAT1_REG	0x12
#define QI_INTSTAT2_REG	0x13
#define QI_INTSTAT1_INT_PMA_EOC_QO_EPT_BIT	0x20
#define QI_INTSTAT1_INT_CHG_START_DET_BIT	0x01
#define QI_INTSTAT2_INT_ERR_POSSET_CLR_BIT	0x02
#define QI_INTSTAT2_INT_ERR_POSSET_BIT		0x01
static int qirx_read_irq(struct i2c_client *client)
{
	int result = 0;
	unsigned short addr = 0x00;
	unsigned short intstat1_data = 0x00;
	unsigned short intstat2_data = 0x00;
	int irq_stat = QIRX_NONE_IRQ;

	//QI_INTSTAT1_REG Read
	addr = QI_INTSTAT1_REG;
	result = qirx_read_regdata(client, addr, &intstat1_data);
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n",addr,intstat1_data);

	//QI_INTSTAT2_REG Read
	addr = QI_INTSTAT2_REG;
	result |= qirx_read_regdata(client, addr, &intstat2_data);
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n",addr,intstat2_data);

	if(result < 0){
		qirx_err("INTCLR1/2 IRQ read fail. result = %d\n", result);
		return irq_stat;
	}

	if(intstat1_data == QI_INTSTAT1_INT_PMA_EOC_QO_EPT_BIT && intstat2_data == 0x00){
		irq_stat = QIRX_QI_EPT_IRQ;
	} else if(intstat1_data == QI_INTSTAT1_INT_CHG_START_DET_BIT && intstat2_data == 0x00){
		irq_stat = QIRX_CHARGE_START_IRQ;
	} else if(intstat1_data == 0x00 && intstat2_data == QI_INTSTAT2_INT_ERR_POSSET_CLR_BIT){
		irq_stat = QIRX_ERR_POSSET_CLR_IRQ;
	} else if(intstat1_data == 0x00 && intstat2_data == QI_INTSTAT2_INT_ERR_POSSET_BIT){
		irq_stat = QIRX_ERR_POSSET_IRQ;
	} else{
		qirx_err("INTCLR1/2 IRQ BIT error.\n");
		qirx_err("INTCLR1: 0x%x, INTCLR2: 0x%x\n", intstat1_data, intstat2_data);
		irq_stat = QIRX_NONE_IRQ;
	}

	return irq_stat;
}

#define QI_INTCLR1_REG	0x14
#define QI_INTCLR2_REG	0x15
#define QI_INTCLR1_INT_CLR_PMA_EOC_QI_EPT_BIT		0x20
#define QI_INTCLR1_REG_INT_CLR_CHG_START_DET_BIT	0x01
#define QI_INTCLR2_REG_INT_CLR_ERR_POSSET_CLR_BIT	0x02
#define QI_INTCLR2_REG_INT_CLR_ERR_POSSET_BIT		0x01
static int qirx_clear_irq(struct i2c_client *client)
{
	unsigned short write_addr = 0x00;
	unsigned short write_data_first = 0x00;
	unsigned short write_data_second = 0x00;
	int result;

	//Qi RX IC INTCLR1/2 IRQ Enable
	write_addr = QI_INTCLR1_REG;	//0x14

	write_data_first = QI_INTCLR1_INT_CLR_PMA_EOC_QI_EPT_BIT | QI_INTCLR1_REG_INT_CLR_CHG_START_DET_BIT;	//0x14 write data
	write_data_second = QI_INTCLR2_REG_INT_CLR_ERR_POSSET_CLR_BIT | QI_INTCLR2_REG_INT_CLR_ERR_POSSET_BIT;	//0x15 write data

	result = qirx_write_regdata(client, write_addr, write_data_first, write_data_second);

	return result;
}

static void qirx_kick_sm(struct qirx *qirx, int ms)
{
	pm_stay_awake(qirx->dev);
	qirx->sm_queued = true;

	if(ms)
		hrtimer_start(&qirx->qirx_kick_sm_timer, ms_to_ktime(ms), HRTIMER_MODE_REL);
	else
		queue_work(qirx->wq, &qirx->sm_work);
}

#define GUIDING_WAITE_TIME 30000
static void qirx_set_state(enum qirx_state next_state)
{
	int ret = 0;

	qirx_dbg(the_qirx, QIRX_INFO, "%s -> %s\n", qirx_state_strings[the_qirx->current_state], qirx_state_strings[next_state]);

	the_qirx->current_state = next_state;

	switch (next_state){
	case QIRX_IDLE:
	case QIRX_SUSPEND:
	case QIRX_INHIBIT:
		the_qirx->irqstat = QIRX_NONE_IRQ;
		the_qirx->pg_state = false;
		break;
	case QIRX_CHARGING:
	case QIRX_FULL:
		ret = qirx_clear_irq(the_client);
		the_qirx->irqstat = QIRX_NONE_IRQ;
		if(ret < 0){
			qirx_err("qirx_clear_irq fail. result = %d\n", ret);
		}
		break;
	case QIRX_GUIDING:
		hrtimer_start(&the_qirx->guiding_timer, ms_to_ktime(GUIDING_WAITE_TIME), HRTIMER_MODE_REL);
		ret = qirx_clear_irq(the_client);
		if(ret < 0){
			qirx_err("qirx_clear_irq fail. result = %d\n", ret);
		}
		break;
	case QIRX_ERROR:
		ret = qirx_clear_irq(the_client);
		if(ret < 0){
			qirx_err("qirx_clear_irq fail. result = %d\n", ret);
		}
		break;
	default :
		qirx_err("set_state fail, No action for state: %s\n", qirx_state_strings[next_state]);
	}
	power_supply_changed(the_qirx->wireless_psy);
	qirx_kick_sm(the_qirx, 0);

}

static int qirx_mfc_reg_setting(unsigned short epp_data, unsigned short rsvd9_data, unsigned short dpset2_data)
{
	int ret = 0;

	pr_info("%s: epp_data = 0x%02x , rsvd9_data = 0x%02x dpset2_data = 0x%02x\n",__FUNCTION__, epp_data, rsvd9_data, dpset2_data);

	//QI_OUTSET_FOR_EPP set Value
	ret = qirx_write_regdata_single(the_client, QI_OUTSET_FOR_EPP_REG, epp_data);
	if(ret < 0){
		qirx_err("QI_OUTSET_FOR_EPP_REG write error.ret = %d\n", ret);
		return ret;
	}

	//QI RSVD9 REG set Value
	ret = qirx_write_regdata_single(the_client, QI_RSVD9_REG, rsvd9_data);
	if(ret < 0){
		qirx_err("QI_RSVD9_REG write error.result = %d\n", ret);
		return ret;
	}

	//QI DPSET2 REG set Value
	ret = qirx_write_regdata_single(the_client, QI_DPSET2_REG, dpset2_data);
	if(ret < 0){
		qirx_err("QI_DPSET2_REG write error.result = %d\n", ret);
		return ret;
	}

	return 0;
}

//For Read Manifacture Code
#define TX_ID1_REG 0x60
#define TX_ID2_REG 0x61
#define TX_CONF1_REG 0x62
#define TX_CONF2_REG 0x63
static int qirx_check_id()
{
	int ret = 0;
	int id = 0;
	unsigned short tx_id1_val = 0x00;
	unsigned short tx_id2_val = 0x00;
	unsigned short tx_conf1_val = 0x00;
	unsigned short tx_conf2_val = 0x00;

	ret = qirx_read_regdata(the_client, TX_ID1_REG, &tx_id1_val);
	ret |= qirx_read_regdata(the_client, TX_ID2_REG, &tx_id2_val);
	ret |= qirx_read_regdata(the_client, TX_CONF1_REG, &tx_conf1_val);
	ret |= qirx_read_regdata(the_client, TX_CONF2_REG, &tx_conf2_val);
	if(ret < 0){
		qirx_err("TX read failed, qirx read error.ret = %d\n", ret);
		return QIRX_ID_MAX;
	}

	pr_info("%s: addr = 0x%02x , data = 0x%02x\n",__FUNCTION__, TX_ID1_REG,tx_id1_val);
	pr_info("%s: addr = 0x%02x , data = 0x%02x\n",__FUNCTION__, TX_ID2_REG,tx_id2_val);
	pr_info("%s: addr = 0x%02x , data = 0x%02x\n",__FUNCTION__, TX_CONF1_REG,tx_conf1_val);
	pr_info("%s: addr = 0x%02x , data = 0x%02x\n",__FUNCTION__, TX_CONF2_REG,tx_conf2_val);

	while(id < QIRX_ID_MAX){
		if(tx_id1_val == qi_id_table[id].tx_id1 && tx_id2_val == qi_id_table[id].tx_id2
		&& tx_conf1_val == qi_id_table[id].tx_conf1 && tx_conf2_val == qi_id_table[id].tx_conf2){
			qirx_dbg(the_qirx, QIRX_DEBUG, "Matching Qi id found.\n");
			break;
		}
		id++;
	}

	if(id == QIRX_ID_MAX){
		qirx_dbg(the_qirx, QIRX_DEBUG, "Matching Qi id not found.\n");
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "id = %d.\n",id);

	return id;
}

//#define QI_EPT_COUNT_TIME 1000
#define QI_AICL_FIRST_POLLING_TIME			0		//0[ms]
#define QI_AICL_FAST_POLLING_TIME			500		//500[ms]
#define QI_AICL_SLOW_POLLING_TIME			2000	//2000[ms]
#define PSNS_REFERENCE_CURRENT				0		//PSNS Reference
//QI_GPIO_DAT_SEL3
#define QI_GPIO_DAT_SEL3_REG 0x97
#define QI_GPIO_DAT_SEL3_VAL 0x09
#define QI_GPIO_DAT_SEL3_VAL_PG_ASSERT 0x00
#define QI_TAPER_AICL_CONST_BPP			1
#define QI_TAPER_AICL_CONST_EPP			2
//MONI MODE CONSTANT VOLTAGE
#define QI_PRED_VOLTAGE_CONSTANT_5V		5
#define QI_PRED_VOLTAGE_CONSTANT_10V	10
//PSNS_STABILITY_CURRENT for MONI
#define QI_PSNS_MODE_EPP_CURRENT		200000
#define QI_PSNS_MODE_BPP_CURRENT		50000
#define PSNS_ADD_CURRENT_50MA			50000
#define PSNS_ADD_CURRENT_100MA			100000
//FOD1_EPP
#define QI_FOD1_EPP_SET_REG	0x22
#define QI_FOD1_EPP_SET_VAL	0x9D
#define QI_FOD1_EPP_SET_VAL_DEFAULT	0x00
static void qirx_sm(struct work_struct *w)
{
	int ret = 0;
	int qirx_id = 0;
	bool mode = false;

	the_qirx->sm_queued = false;

	qirx_dbg(the_qirx, QIRX_DEBUG, "current_state: %s.\n", qirx_state_strings[the_qirx->current_state]);
	qirx_dbg(the_qirx, QIRX_DEBUG, "irqstat: %s.\n", qirx_irqstat_strings[the_qirx->irqstat]);

	if(the_qirx->current_state != QIRX_CHARGING){
		qirx_charging_flag_now = false;
	}
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx_charging_flag_now = %d.\n", qirx_charging_flag_now);

	if(the_qirx->current_state != QIRX_CHARGING && the_qirx->current_state != QIRX_FULL ){
		qirx_id_1912_det_flag = false;
		qirx_id_check_flag = false;
		qirx_dbg(the_qirx, QIRX_DEBUG, "initialize qirx_id_1912_det_flag = %d\n",qirx_id_1912_det_flag);
		qirx_dbg(the_qirx, QIRX_DEBUG, "initialize qirx_id_check_flag = %d\n", qirx_id_check_flag);
	}

	/* When PG_ASSERT */
	if(the_qirx->pg_state){
		//QI_GPIO_DAT_SEL3 when PG_ASSETRT
		ret = qirx_write_regdata_single(the_client, QI_GPIO_DAT_SEL3_REG, QI_GPIO_DAT_SEL3_VAL_PG_ASSERT);
		if(ret < 0){
			qirx_err("set fail, QI_GPIO_DAT_SEL3_REG when PG_ASSETRT write error.ret = %d\n", ret);
		}

		//Get moni mode
		if(!moni_mode_get_complete_flag){
			ret = qirx_is_qi_epp_mode(the_client,&mode);
			if(ret < 0){
				qirx_err("qirx_is_qi_epp_mode error.\n");
			}else{
				moni_mode_get_complete_flag = true;
			}

			if(mode){
				//MODE:EPP
				qirx_dbg(the_qirx, QIRX_DEBUG, "MONI MODE = EPP\n");
				qirx_psns_stability_current = QI_PSNS_MODE_EPP_CURRENT;
				qirx_step_current = PSNS_ADD_CURRENT_100MA;
				pred_volt_const = QI_PRED_VOLTAGE_CONSTANT_10V;
				taper_aicl_const = QI_TAPER_AICL_CONST_EPP;
				epp_mode_flag = true;
			}else{
				//MODE:BPP
				qirx_dbg(the_qirx, QIRX_DEBUG, "MONI MODE = BPP\n");
				qirx_psns_stability_current = QI_PSNS_MODE_BPP_CURRENT;
				qirx_step_current = PSNS_ADD_CURRENT_50MA;
				pred_volt_const = QI_PRED_VOLTAGE_CONSTANT_5V;
				taper_aicl_const = QI_TAPER_AICL_CONST_BPP;
				epp_mode_flag = false;
			}
		}

		//Check Manifacture code
		if(!qirx_id_check_flag){
			qirx_id = qirx_check_id();
			qirx_id_check_flag = true;
			qirx_dbg(the_qirx, QIRX_DEBUG, "qirx_id_check_flag = %d\n",qirx_id_check_flag);

			switch(qirx_id){
			case QIRX_ID_1912:
				qirx_dbg(the_qirx, QIRX_DEBUG, "ID 1912 was detected.\n");
				ret = qirx_mfc_reg_setting(qi_id_table[qirx_id].mfc_epp_data, qi_id_table[qirx_id].mfc_rsvd9_data, qi_id_table[qirx_id].mfc_dpset2_data);
				if(ret < 0){
					qirx_err("MFC reg write error.ret = %d\n", ret);
				}
				qirx_id_1912_det_flag = true;
				qirx_dbg(the_qirx, QIRX_DEBUG, "qirx_id_1912_det_flag = %d\n", qirx_id_1912_det_flag);
				break;
			case QIRX_ID_2834:
				qirx_dbg(the_qirx, QIRX_DEBUG, "ID 2834 was detected.\n");
				ret = qirx_mfc_reg_setting(qi_id_table[qirx_id].mfc_epp_data, qi_id_table[qirx_id].mfc_rsvd9_data, qi_id_table[qirx_id].mfc_dpset2_data);
				if(ret < 0){
					qirx_err("MFC reg write error.ret = %d\n", ret);
				}
				pred_volt_const = QI_PRED_VOLTAGE_CONSTANT_5V;
				taper_aicl_const = QI_TAPER_AICL_CONST_BPP;
				break;
			case QIRX_ID_2574:
			case QIRX_ID_2225:
				if(qirx_id == QIRX_ID_2574)
					qirx_dbg(the_qirx, QIRX_DEBUG, "ID 2574 was detected.\n");
				else
					qirx_dbg(the_qirx, QIRX_DEBUG, "ID 2225 was detected.\n");

				ret = qirx_mfc_reg_setting(qi_id_table[qirx_id].mfc_epp_data, qi_id_table[qirx_id].mfc_rsvd9_data, qi_id_table[qirx_id].mfc_dpset2_data);
				if(ret < 0){
					qirx_err("MFC reg write error.ret = %d\n", ret);
				}
				//Setting as MODE:EPP
				qirx_psns_stability_current = QI_PSNS_MODE_EPP_CURRENT;
				qirx_step_current = PSNS_ADD_CURRENT_100MA;
				pred_volt_const = QI_PRED_VOLTAGE_CONSTANT_5V;
				taper_aicl_const = QI_TAPER_AICL_CONST_BPP;
				epp_mode_flag = true;
				break;
			case QIRX_ID_DCM:
				qirx_dbg(the_qirx, QIRX_DEBUG, "ID DCM was detected.\n");
				ret = qirx_mfc_reg_setting(qi_id_table[qirx_id].mfc_epp_data, qi_id_table[qirx_id].mfc_rsvd9_data, qi_id_table[qirx_id].mfc_dpset2_data);
				if(ret < 0){
					qirx_err("MFC reg write error.ret = %d\n", ret);
				}
				//QI_FOD1_EPP_SET_REG set Value
				ret = qirx_write_regdata_single(the_client, QI_FOD1_EPP_SET_REG, QI_FOD1_EPP_SET_VAL);
				if(ret < 0){
					qirx_err("QI_FOD1_EPP_SET_REG write error.ret = %d\n", ret);
				}
				qirx_fod_set_flag = true;
				break;
			default:
				qirx_dbg(the_qirx, QIRX_DEBUG, "Other Qi-ID was detected.\n");
				ret = qirx_mfc_reg_setting(qi_id_table[qirx_id].mfc_epp_data, qi_id_table[qirx_id].mfc_rsvd9_data, qi_id_table[qirx_id].mfc_dpset2_data);
				if(ret < 0){
					qirx_err("MFC reg write error.ret = %d\n", ret);
				}
			}
		}
	/* When PG_DEASSERT */
	}else if(!the_qirx->pg_state){
		//QI_GPIO_DAT_SEL3 when PG_DEASSERT
		ret = qirx_write_regdata_single(the_client, QI_GPIO_DAT_SEL3_REG, QI_GPIO_DAT_SEL3_VAL);
		if(ret < 0){
			qirx_err("set fail, QI_GPIO_DAT_SEL3_REG when PG_DEASSETRT write error.ret= %d\n", ret);
		}
		//Initialize moni mode val
		taper_aicl_const = 0;
		pred_volt_const = 0;
		moni_mode_get_complete_flag = false;
		moni_mode_set_state = 0;
		if(qirx_fod_set_flag){
			//QI_FOD1_EPP_SET_REG set Value
			ret = qirx_write_regdata_single(the_client, QI_FOD1_EPP_SET_REG, QI_FOD1_EPP_SET_VAL_DEFAULT);
			if(ret < 0){
				qirx_err("QI_FOD1_EPP_SET_REG write error.ret = %d\n", ret);
			}
			qirx_fod_set_flag = false;
		}
	}

	switch(the_qirx->current_state) {
	case QIRX_IDLE:
		calibration_count = 0;
		icl_settled = false;
		polling_count = 0;
		maximum_power_value = 0;
		before_maximum_power_value = 0;
		aicl_current_icl = PSNS_REFERENCE_CURRENT;

		if(!qirx_is_power_good()){
			the_qirx->pg_state = true;
		}
		else{
			the_qirx->pg_state = false;
		}

		if(the_qirx->addet_state){
			qirx_set_state(QIRX_INHIBIT);
		} else if(the_qirx->suspend_state){
			qirx_set_state(QIRX_SUSPEND);
		} else if(the_qirx->pg_state
			|| the_qirx->irqstat == QIRX_ERR_POSSET_CLR_IRQ){
			qirx_set_state(QIRX_CHARGING);
		} else if (the_qirx->irqstat == QIRX_ERR_POSSET_IRQ){
//			qi_ept_count++;
//			if(qi_ept_count == 1){
//				hrtimer_start(&the_qirx->qi_ept_count_timer, ms_to_ktime(QI_EPT_COUNT_TIME), HRTIMER_MODE_REL);
//			} else if(qi_ept_count == 3){
				qirx_set_state(QIRX_GUIDING);
//				qi_ept_count = 0;
//				hrtimer_cancel(&the_qirx->qi_ept_count_timer);
//			}
		}
		break;

	case QIRX_INHIBIT:
		if(!the_qirx->addet_state){
			qirx_set_state(QIRX_IDLE);
		}
		break;
	case QIRX_SUSPEND:
		if(the_qirx->addet_state){
			qirx_set_state(QIRX_INHIBIT);
		} else if(!the_qirx->suspend_state){
			qirx_set_state(QIRX_IDLE);
		}
		break;
	case QIRX_GUIDING:
		if(the_qirx->addet_state){
			qirx_set_state(QIRX_INHIBIT);
		} else if(the_qirx->suspend_state){
			qirx_set_state(QIRX_SUSPEND);
		} else if(the_qirx->pg_state
			|| the_qirx->irqstat == QIRX_ERR_POSSET_CLR_IRQ){
			qirx_set_state(QIRX_CHARGING);
		} else if (the_qirx->irqstat == QIRX_ERR_POSSET_IRQ){
			hrtimer_cancel(&the_qirx->guiding_timer);
			hrtimer_start(&the_qirx->guiding_timer, ms_to_ktime(GUIDING_WAITE_TIME), HRTIMER_MODE_REL);
		} else if (the_qirx->irqstat == QIRX_QI_EPT_IRQ){
			qirx_set_state(QIRX_ERROR);
		}
		break;

	case QIRX_CHARGING:
		if(the_qirx->addet_state){
			qirx_set_state(QIRX_INHIBIT);
		} else if(the_qirx->suspend_state){
			qirx_set_state(QIRX_SUSPEND);
		} else if(!the_qirx->pg_state){
			qirx_set_state(QIRX_IDLE);
		} else if(the_qirx->irqstat == QIRX_QI_EPT_IRQ){
			qirx_set_state(QIRX_ERROR);
		} else if(the_qirx->batt_psy_status == POWER_SUPPLY_STATUS_CHARGING){
			if(!qirx_charging_flag_now){
				//Start AICL
				qirx_dbg(the_qirx, QIRX_DEBUG, "regist qirx_rerun_aicl_loop_work.\n");
				schedule_delayed_work(&the_qirx->qirx_rerun_aicl_loop_work, msecs_to_jiffies(QI_AICL_FIRST_POLLING_TIME));
				qirx_charging_flag_now = true;
				qirx_dbg(the_qirx, QIRX_DEBUG, "qirx_charging_flag_now = %d.\n", qirx_charging_flag_now);
			}
		} else if(the_qirx->batt_psy_status == POWER_SUPPLY_STATUS_FULL){
			qirx_set_state(QIRX_FULL);
		}
		break;

	case QIRX_FULL:
		icl_settled = false;
		polling_count = 0;
		maximum_power_value = 0;
		before_maximum_power_value = 0;

		if(the_qirx->addet_state){
			qirx_set_state(QIRX_INHIBIT);
		} else if(the_qirx->suspend_state){
			qirx_set_state(QIRX_SUSPEND);
		} else if(!the_qirx->pg_state){
			qirx_set_state(QIRX_IDLE);
		} else if(the_qirx->irqstat == QIRX_QI_EPT_IRQ){
			qirx_set_state(QIRX_ERROR);
		} else if(the_qirx->batt_psy_status != POWER_SUPPLY_STATUS_FULL){
			qirx_set_state(QIRX_CHARGING);
		}
		break;

	case QIRX_ERROR:
		if(!qirx_is_addet()){
			//ERROR of EPT
			ret = qirx_get_ept_code(the_client);
			if(ret < 0){
				qirx_err("EPT code get failed. ret = %d.\n", ret);
			} else{
				qirx_err("EPT Error, error code = %s.\n", qirx_ept_code_strings[the_qirx->ept_code]);
				power_supply_changed(the_qirx->wireless_psy);
			}
		}else{
			qirx_err("EPT because of VBUS.\n");
		}
		qirx_set_state(QIRX_IDLE);
		break;

	default :
		qirx_err("sm fail, Unhandled state %d\n", qirx_irqstat_strings[the_qirx->irqstat]);
	}
	shbatt_api_battlog_qirx_charge_status(the_qirx->current_state);
}

static enum hrtimer_restart qirx_kick_sm_timeout(struct hrtimer *timer)
{
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx_kick_sm_timer timeout\n");
	queue_work(the_qirx->wq, &the_qirx->sm_work);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart guiding_timeout(struct hrtimer *timer)
{
	qirx_dbg(the_qirx, QIRX_DEBUG, "guiding_timer timeout\n");

	if(the_qirx->current_state == QIRX_GUIDING){
		qirx_set_state(QIRX_IDLE);
	}

	return HRTIMER_NORESTART;
}

//static enum hrtimer_restart qi_ept_count_timeout(struct hrtimer *timer)
//{
//	qirx_dbg(the_qirx, QIRX_DEBUG, "qi_ept_count_timer timeout\n");
//	qi_ept_count = 0;
//
//	return HRTIMER_NORESTART;
//}

static void qirx_int_n_irq( struct work_struct *w )
{
	int ret = 0;

	the_qirx->irqstat = qirx_read_irq(the_client);
	qirx_dbg(the_qirx, QIRX_DEBUG, "irqstat = %s\n", qirx_irqstat_strings[the_qirx->irqstat]);

	ret = qirx_clear_irq(the_client);
	if(ret < 0){
		qirx_err("qirx_clear_irq fail. result = %d\n", ret);
	}

	if(the_qirx->irqstat != QIRX_CHARGE_START_IRQ || the_qirx->current_state != QIRX_CHARGING){
		qirx_kick_sm(the_qirx, 0);
	}
}

static irqreturn_t qirx_irq_handler(int32_t irq, void *dev_id)
{
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx_irq_handler IRQ occur!\n");

	qirx_wake_lock_start(&qirx_irq_wake_lock);
	if( irq != int_n_irqNo ){
		qirx_wake_lock_end(&qirx_irq_wake_lock);
		qirx_err("irqNo %d is not qirx_irq_handler.\n", irq);
		return IRQ_NONE;
	}

	schedule_work(&the_qirx->int_n_irq_work);

	qirx_wake_lock_end(&qirx_irq_wake_lock);

	return IRQ_HANDLED;
}

int qirx_is_power_good(void)
{
	return qirx_gpio_get_value(the_qirx, QIRX_GPIO_PIN_PG);
}

static irqreturn_t power_good_irq_handler(int32_t irq, void *dev_id)
{

	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx power_good_irq_handler IRQ occur!\n");

	qirx_wake_lock_start(&qirx_irq_wake_lock);
	if( irq != pg_irqNo ){
		qirx_wake_lock_end(&qirx_irq_wake_lock);
		qirx_err("irqNo %d is not power_good_irq_handler.\n", irq);
		return IRQ_NONE;
	}

	if(!qirx_is_power_good()){
	// falling
		the_qirx->pg_state = true;
	} else{
	// rising
		the_qirx->pg_state = false;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx->pg = %d.\n", the_qirx->pg_state);

	if(!the_qirx->pg_state || the_qirx->current_state != QIRX_CHARGING){
		qirx_kick_sm(the_qirx, 0);
	}
	qirx_wake_lock_end(&qirx_irq_wake_lock);

	return IRQ_HANDLED;
}

int qirx_is_addet(void)
{
	return qirx_gpio_get_value(the_qirx, QIRX_GPIO_PIN_ADDDET_STATE);
}

static irqreturn_t addet_irq_handler(int32_t irq, void *dev_id)
{
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx addet_irq_handler IRQ occur!\n");

	qirx_wake_lock_start(&qirx_irq_wake_lock);
	if( irq != addet_irqNo ){
		qirx_wake_lock_end(&qirx_irq_wake_lock);
		qirx_err("irqNo %d is not addet_irq_handler.\n", irq);
		return IRQ_NONE;
	}

	if(qirx_is_addet()){
	// rising
		the_qirx->addet_state = true;
		qirx_set_state(QIRX_INHIBIT);
	} else{
	// falling
		the_qirx->addet_state = false;
		qirx_set_state(QIRX_IDLE);
	}

	qirx_wake_lock_end(&qirx_irq_wake_lock);

	return IRQ_HANDLED;
}

static int qirx_psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	union power_supply_propval val;
	int ret;

	if( (ptr != batt_psy && ptr != dc_psy) || evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	//batt_psy POWER_SUPPLY_PROP_STATUS
	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if(ret < 0){
		qirx_err("fail, batt_psy get property. ret = %d\n", ret);
		return ret;
	}
	the_qirx->batt_psy_status = val.intval;

	//dc_psy POWER_SUPPLY_PROP_PRESENT
	ret = power_supply_get_property(dc_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if(ret < 0){
		qirx_err("fail, dc_psy get property. ret = %d\n", ret);
		return ret;
	}
	the_qirx->dc_psy_present = val.intval;

	qirx_dbg(the_qirx, QIRX_DEBUG, "batt_psy POWER_SUPPLY_PROP_STATUS  = %d\n", the_qirx->batt_psy_status);
	qirx_dbg(the_qirx, QIRX_DEBUG, "  dc_psy POWER_SUPPLY_PROP_PRESENT = %d\n", the_qirx->dc_psy_present);

	if((the_qirx->current_state == QIRX_CHARGING
		|| the_qirx->current_state == QIRX_FULL)
		&& the_qirx->dc_psy_present == 1
		&& (the_qirx -> batt_psy_status == POWER_SUPPLY_STATUS_CHARGING
		|| the_qirx -> batt_psy_status == POWER_SUPPLY_STATUS_FULL) ){
		qirx_kick_sm(the_qirx, 0);
	}

	return 0;
}

#define QIRX_GPIO_PIN_INT_N_NAME "qirx_hostif_int_n"
#define QIRX_GPIO_PIN_PG_NAME "qirx_hostif_pg"
#define QIRX_GPIO_PIN_ADDET_STATE_NAME "qirx_hostif_addet"
static int32_t qirx_irq_init(struct qirx *qirx)
{
	int32_t ret = 0;

	/* INT_N IRQ Initialize */
	int_n_irqNo = qirx_gpio_to_irq(qirx, QIRX_GPIO_PIN_INT_N);
	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_INT_N);
	if (ret < 0){
		qirx_err("failed to QIRX_GPIO_PIN_INT_N request. ret=%d\n",ret);
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_INT_N);
		return -ENODEV;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx int_n_irqNo = %d\n",int_n_irqNo);
	ret = request_any_context_irq(int_n_irqNo, qirx_irq_handler, IRQF_TRIGGER_FALLING, QIRX_GPIO_PIN_INT_N_NAME, NULL);
	ret |= irq_set_irq_wake(int_n_irqNo, 1);
	if(ret < 0) {
		qirx_err("Failed request_any_context_irq. ret=%d\n", ret);
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_INT_N);
		return -ENODEV;
	}

	qirx_gpio_free(qirx, QIRX_GPIO_PIN_INT_N);

	/* PG IRQ Initialize */
	pg_irqNo = qirx_gpio_to_irq(qirx, QIRX_GPIO_PIN_PG);
	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_PG);
	if (ret < 0){
		qirx_err("failed to QIRX_GPIO_PIN_PG request. ret=%d\n",ret);
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_PG);
		return -ENODEV;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx pg_irqNo = %d\n",pg_irqNo);
	ret = request_any_context_irq(pg_irqNo, power_good_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, QIRX_GPIO_PIN_PG_NAME, NULL);
	ret |= irq_set_irq_wake(pg_irqNo, 1);
	if(ret < 0) {
		qirx_err("Failed request_any_context_irq. ret=%d\n", ret);
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_PG);
		return -ENODEV;
	}
	qirx_gpio_free(qirx, QIRX_GPIO_PIN_PG);

	/* ADDET IRQ Initialize */
	addet_irqNo = qirx_gpio_to_irq(qirx, QIRX_GPIO_PIN_ADDDET_STATE);
	ret = qirx_gpio_request(qirx, QIRX_GPIO_PIN_ADDDET_STATE);
	if (ret < 0){
		qirx_err("QIRX_GPIO_PIN_ADDDET_STATE request. Error.\n");
		qirx_gpio_free(qirx,QIRX_GPIO_PIN_ADDDET_STATE);
		return -ENODEV;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx addet_irqNo = %d\n",addet_irqNo);
	ret = request_any_context_irq(addet_irqNo, addet_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, QIRX_GPIO_PIN_ADDET_STATE_NAME, NULL);
	ret |= irq_set_irq_wake(addet_irqNo, 1);
	if(ret < 0) {
		qirx_err("Failed request_any_context_irq. ret=%d\n", ret);
		qirx_gpio_free(qirx,QIRX_GPIO_PIN_ADDDET_STATE);
		return -ENODEV;
	}
	qirx_gpio_free(qirx,QIRX_GPIO_PIN_ADDDET_STATE);

	return 0;
}

#define RP16VAL_B0_REG	0x53
#define RP16VAL_B1_REG	0x54
#define RX_CONF_B0_REG	0x7D
#define ONE_BYTE_SHIFT	8
#define MAXIMUMPOWER_MASK	0x3F
#define MAXIMUMPOWER_DIVIDE	2
#define RECEIVED_POWER_CONSTANT 32768
#define MV_SHOW 1000000
int qirx_get_received_power( struct i2c_client *client, union power_supply_propval *val)
{
	int result = 0;

	unsigned short received_power_upper_val = 0x00;
	unsigned short received_power_lower_val = 0x00;

	unsigned short received_power_marge_val = 0;
	int maximum_power_val = 0;
	double received_power_val = 0.0;

	union power_supply_propval pval = {0,};
	int received_maximum_power = 0;

	result = qirx_get_received_maximum_power(client, &pval);
	received_maximum_power = pval.intval;

	qirx_dbg(the_qirx, QIRX_DEBUG, "received_maximum_power = %d\n", received_maximum_power);

	result |= qirx_read_regdata(client, RP16VAL_B0_REG, &received_power_upper_val);
	result |= qirx_read_regdata(client, RP16VAL_B1_REG, &received_power_lower_val);

	if(result < 0)
	{
		qirx_err("get_received_power failed, qirx read error.result = %d\n", result);
		return result;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n", RP16VAL_B0_REG, received_power_upper_val);
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n", RP16VAL_B1_REG, received_power_lower_val);

	received_power_marge_val = (received_power_upper_val << ONE_BYTE_SHIFT) + received_power_lower_val;
	maximum_power_val = received_maximum_power / MV_SHOW;
	received_power_val = ((double)received_power_marge_val / (double)RECEIVED_POWER_CONSTANT) * (double)maximum_power_val;

	val->intval = (int)(received_power_val * SHOW_DIGIT);
	qirx_dbg(the_qirx, QIRX_DEBUG, "received_power = %05d\n", val->intval);

	return result;
}

#define RPFREQ_UPPER_REG	0x57
#define RPFREQ_LOWER_REG	0x58
#define RPFREQ_MASK	0x1F
#define RPFREQ_CONSTANT	8192
#define RPFREQ_DIVIDE	64

int qirx_get_frequency( struct i2c_client *client, union power_supply_propval *val)
{
	int result = 0;

	unsigned short rpfreq_upper_val = 0x00;
	unsigned short rpfreq_lower_val = 0x00;

	unsigned short rpfreq_marge_val = 0;
	double rpfreq_val = 0.0;

	result |= qirx_read_regdata(client, RPFREQ_UPPER_REG, &rpfreq_upper_val);
	result |= qirx_read_regdata(client, RPFREQ_LOWER_REG, &rpfreq_lower_val);

	if(result < 0)
	{
		qirx_err("qirx_get_frequency failed, qirx read error.result = %d\n", result);
		return result;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n", RPFREQ_UPPER_REG, rpfreq_upper_val);
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n", RPFREQ_LOWER_REG, rpfreq_lower_val);

	rpfreq_marge_val = ((rpfreq_upper_val & RPFREQ_MASK) << ONE_BYTE_SHIFT) + rpfreq_lower_val;

	if(rpfreq_marge_val != 0){
		rpfreq_val = (double)RPFREQ_CONSTANT / ((double)rpfreq_marge_val / (double)RPFREQ_DIVIDE);
	}

	val->intval = (int)(rpfreq_val * SHOW_DIGIT);
	qirx_dbg(the_qirx, QIRX_DEBUG, "frequency = %07d\n", val->intval);

	return result;
}

int qirx_get_received_maximum_power( struct i2c_client *client, union power_supply_propval *val)
{
	int result = 0;
	unsigned short rxconf_b0_val = 0x00;
	unsigned short maximum_power_val = 0;

	result = qirx_read_regdata(client, RX_CONF_B0_REG, &rxconf_b0_val);

	if(result < 0)
	{
		qirx_err("get_received_power failed, qirx read error.result = %d\n", result);
		return result;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n", RX_CONF_B0_REG, rxconf_b0_val);

	maximum_power_val = (rxconf_b0_val & MAXIMUMPOWER_MASK) / MAXIMUMPOWER_DIVIDE;

	val->intval = (int)(maximum_power_val * MV_SHOW);

	return result;
}

int qirx_get_batt_charger_status(void)
{
	int ret = 0;
	int read_data_int = 0;
	union power_supply_propval	val = {0,};

	if (batt_psy == NULL) {
		qirx_err("batt_psy = NULL \n");
		return read_data_int;
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_BATTERY_CHARGER_STATUS, &val);
	if (!ret)
		read_data_int = val.intval;

	return read_data_int;
}

int qirx_get_current_avg( struct i2c_client *client, union power_supply_propval *val)
{
	int result = 0;
	int read_data_int = 0;
	union power_supply_propval pval;

	if (batt_psy == NULL) {
		qirx_err("batt_psy = NULL \n");
		return read_data_int;
	}

	//TEST POWER_SUPPLY_PROP_CURRENT_AVG
	//result = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_AVG, &pval);
	result = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	if(result < 0){
		qirx_err("get error:POWER_SUPPLY_PROP_CURRENT_MAX\n");
		return result;
	}

	val->intval = pval.intval;

	return result;
}

#define QI_PRED_WAT_DEVIDE				100
#define QI_AICL_POLLING_MAX_COUNT		10
#define PSNS_STABILITY_CURRENT_ID_1912	50000
#define PSNS_STABILITY_CURRENT_FULL		200000
#define PSNS_SECONDLY_STABILITY_CURRENT	400000
#define QI_TAPER_CURRENT_DIFF			200000
#define QI_CALIBRATION_COUNT_MAX		20
static void qirx_rerun_psued_aicl( struct work_struct *w )
{
	enum power_supply_battery_charger_status chg_stat;
	union power_supply_propval pval ={0,};
	int aicl_surplus = 0;//+a
	int current_max_val = 0;
	int polling_timer = QI_AICL_FAST_POLLING_TIME;
	int result = 0;
	int received_power_val = 0;
	int pred_received_power_val = 0;
	int current_avg = 0;

	chg_stat = POWER_SUPPLY_BATTERY_CHG_STS_NONE;

	qirx_dbg(the_qirx, QIRX_DEBUG, "[S]%s\n",__FUNCTION__);

	//QI-ID 1912 process
	if(qirx_id_1912_det_flag){
		pr_info("%s: QI-ID 1912 detect. aicl polling end.\n",__FUNCTION__);
		aicl_current_icl = PSNS_STABILITY_CURRENT_ID_1912;
		result = qirx_update_power_supply(dc_psy,aicl_current_icl);
		if(result < 0){
			qirx_err("set error: qirx_update_power_supply\n");
		}
		qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
		return;
	}

	//FULL Check
	if(the_qirx->batt_psy_status == POWER_SUPPLY_STATUS_FULL){
		qirx_dbg(the_qirx, QIRX_INFO, "charging status FULL. aicl polling end.\n");
		polling_count = 0;
		maximum_power_value = 0;
		before_maximum_power_value = 0;
		aicl_current_icl = PSNS_STABILITY_CURRENT_FULL;
		result = qirx_update_power_supply(dc_psy,aicl_current_icl);
		if(result < 0){
			qirx_err("set error: qirx_update_power_supply\n");
		}
		qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
		return;
	}

	//TAPER check
	chg_stat = qirx_get_batt_charger_status();
	if(chg_stat < 0){
		qirx_err("get batt charger status error.\n");
	}
	if(chg_stat == POWER_SUPPLY_BATTERY_CHG_STS_TAPER){
		qirx_dbg(the_qirx, QIRX_INFO, "charging type TAPER.\n");
		if(aicl_current_icl < qirx_psns_stability_current){
			aicl_current_icl = qirx_psns_stability_current;
		}
		result = qirx_update_power_supply(dc_psy,aicl_current_icl);
		if(result < 0){
			qirx_err("set error: qirx_update_power_supply\n");
		}
		qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
		//icl_settled: True
		if(icl_settled == true){
			//Get current avg
			result = qirx_get_current_avg(the_client, &pval);
			if(result < 0){
				qirx_err("get error: qirx_get_current_avg\n");
			}
			current_avg = pval.intval;
			qirx_dbg(the_qirx, QIRX_DEBUG, "current_avg = %d\n", current_avg);
			qirx_dbg(the_qirx, QIRX_DEBUG, "aicl_current_icl = %d\n", aicl_current_icl);

			//Discharging
			if(current_avg < 0){
				//Go to Fast polling
				icl_settled = false;
				polling_count = 0;
				polling_timer = QI_AICL_FAST_POLLING_TIME;
				goto RERUN_AICL;
			}else{
			//Charging
				if((aicl_current_icl * taper_aicl_const) > (current_avg + QI_TAPER_CURRENT_DIFF)){
					if(aicl_current_icl > qirx_psns_stability_current){
						aicl_current_icl -= qirx_step_current;
					}
					qirx_dbg(the_qirx, QIRX_DEBUG, "aicl_current_icl = %d\n", aicl_current_icl);
				}
			}
		}else{
		//icl_settled: False
			//Go to RERUN_AICL
			qirx_dbg(the_qirx, QIRX_INFO, "ICL isn't settled .\n");
			goto RERUN_AICL;
		}
		//Go to Slow polling
		polling_count = 0;
		polling_timer = QI_AICL_SLOW_POLLING_TIME;
		goto LOOP;
	}

	//Qi status = charging?
	if( QIRX_CHARGING != the_qirx->current_state ){
		qirx_dbg(the_qirx, QIRX_INFO, "Not charging. aicl polling end.\n");
		icl_settled = false;
		polling_count = 0;
		maximum_power_value = 0;
		before_maximum_power_value = 0;
		calibration_count = 0;
		aicl_current_icl = PSNS_REFERENCE_CURRENT;
		result = qirx_update_power_supply(dc_psy,aicl_current_icl);
		if(result < 0){
			qirx_err("set error: qirx_update_power_supply\n");
		}
		qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
		return;
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "current status is CHARGING.\n");

	//FULLON check
	chg_stat = qirx_get_batt_charger_status();
	if(chg_stat < 0){
		qirx_err("get batt charger status error.\n");
	}

	qirx_dbg(the_qirx, QIRX_DEBUG, "chg_stat = %d\n", chg_stat);

	if(chg_stat != POWER_SUPPLY_BATTERY_CHG_STS_FULLON && icl_settled == true){
		//Go to Slow polling
		aicl_current_icl = qirx_psns_stability_current;
		result = qirx_update_power_supply(dc_psy, aicl_current_icl);
		if(result < 0){
			qirx_err("set error: qirx_update_power_supply\n");
		}
		qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
		polling_count = 0;
		polling_timer = QI_AICL_SLOW_POLLING_TIME;
		goto LOOP;
	}

RERUN_AICL:
	qirx_dbg(the_qirx, QIRX_INFO, "Fast Charging. polling count = %d\n", polling_count);
	//Set AICL 200mA or prev polling is Slow polling when aicl_current_icl < 200mA
	if(aicl_current_icl < qirx_psns_stability_current || ( polling_timer == QI_AICL_SLOW_POLLING_TIME)){
		aicl_current_icl = qirx_psns_stability_current;
		result = qirx_update_power_supply(dc_psy, aicl_current_icl);
		if(result < 0){
			qirx_err("set error: qirx_update_power_supply\n");
		}
		qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
	}

	qirx_dbg(the_qirx, QIRX_INFO, "Fast Charging. calibration count = %d\n", calibration_count);

	//Get received_power
	result = qirx_get_received_power(the_client, &pval);
	if(result < 0)
	{
		qirx_err("get_received_power failed, qirx read error.result = %d\n", result);
	}
	received_power_val = pval.intval;
	qirx_dbg(the_qirx, QIRX_DEBUG, "received_power_val = %d.\n", received_power_val);

	//Get maximum power check
	result = qirx_get_received_maximum_power(the_client, &pval);
	if(result < 0)
	{
		qirx_err("get_received_power failed, qirx read error.result = %d\n", result);
	}
	maximum_power_value = pval.intval;
	qirx_dbg(the_qirx, QIRX_INFO, "maximum_power_value = %d, before_maximum_power_value = %d\n", maximum_power_value, before_maximum_power_value);

	// Comparison of maximum power and before maximum power
	if(maximum_power_value != before_maximum_power_value){
		aicl_current_icl = qirx_psns_stability_current;
		result = qirx_update_power_supply(dc_psy, aicl_current_icl);
		if(result < 0){
			qirx_err("set error: qirx_update_power_supply\n");
		}
		qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);

		before_maximum_power_value = maximum_power_value;

		//Go to Fast polling
		icl_settled = false;
		polling_count = 0;
		polling_timer = QI_AICL_FAST_POLLING_TIME;

	}else if(maximum_power_value == before_maximum_power_value){

		if(epp_mode_flag){
			//Preparation AICL 400mA when AICL is 200mA && calibration_count is 0
			if((aicl_current_icl == qirx_psns_stability_current) && calibration_count == 0 ){
				//Next aicl_current_icl = 400mA
				qirx_dbg(the_qirx, QIRX_DEBUG, "Next aicl_current_icl.\n");
				aicl_current_icl = PSNS_SECONDLY_STABILITY_CURRENT;
			}

			//Maintain 400 mA for 10 sec.
			if((aicl_current_icl == PSNS_SECONDLY_STABILITY_CURRENT) && calibration_count < QI_CALIBRATION_COUNT_MAX ){

				aicl_current_icl = PSNS_SECONDLY_STABILITY_CURRENT;
				result = qirx_update_power_supply(dc_psy, aicl_current_icl);
				if(result < 0){
					qirx_err("set error: qirx_update_power_supply\n");
				}
				qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);

				calibration_count++;
				qirx_dbg(the_qirx, QIRX_DEBUG, "10s Polling Now...\n");
				qirx_dbg(the_qirx, QIRX_DEBUG, "calibration_count = %d\n", calibration_count);

				//Go to Fast polling
				polling_count = 0;
				polling_timer = QI_AICL_FAST_POLLING_TIME;
				goto LOOP;
			}
			else if((aicl_current_icl == PSNS_SECONDLY_STABILITY_CURRENT) && calibration_count == QI_CALIBRATION_COUNT_MAX ){
				qirx_dbg(the_qirx, QIRX_DEBUG, "10s Polling End.\n");
				qirx_dbg(the_qirx, QIRX_DEBUG, "calibration_count = %d\n", calibration_count);
			}
		}else{
			calibration_count = QI_CALIBRATION_COUNT_MAX;
		}

		//Get prediction received_power
		pred_received_power_val = ((pred_volt_const * aicl_current_icl)/QI_PRED_WAT_DEVIDE) + aicl_surplus;
		qirx_dbg(the_qirx, QIRX_INFO, "aicl_current_icl = %d\n", aicl_current_icl);
		qirx_dbg(the_qirx, QIRX_DEBUG, "pred_received_power_val = %d\n", pred_received_power_val);

		//Get current_max
		result = qirx_calculate_max_current_ua(the_client, &pval);
		if(result < 0){
			qirx_err("current_max_val failed, qirx read error.result = %d\n", result);
		}
		current_max_val = pval.intval;
		qirx_dbg(the_qirx, QIRX_INFO, "current_max_val = %d\n", current_max_val);

		qirx_dbg(the_qirx, QIRX_INFO, "received_power_val = %d, pred_received_power_val = %d\n", received_power_val, pred_received_power_val);

		// Comparison of received power and predicted received power
		if(received_power_val >= pred_received_power_val){
			if((aicl_current_icl + qirx_step_current) > current_max_val){
				qirx_dbg(the_qirx, QIRX_INFO, "current icl + %d mA > current max\n", qirx_step_current/1000 );
				//Go to Slow polling
				polling_count = 0;
				polling_timer = QI_AICL_SLOW_POLLING_TIME;
				icl_settled = true;
			}else{
				if(!(chg_stat == POWER_SUPPLY_BATTERY_CHG_STS_PAUSE_CHARGE)){
					aicl_current_icl += qirx_step_current;
					result = qirx_update_power_supply(dc_psy, aicl_current_icl);
					if(result < 0){
						qirx_err("set error: qirx_update_power_supply\n");
					}
					qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
					//Go to Fast polling
					polling_count = 0;
					polling_timer = QI_AICL_FAST_POLLING_TIME;
				}
			}
		}else if(received_power_val < pred_received_power_val){
			//Check of polling count
			if(polling_count < QI_AICL_POLLING_MAX_COUNT){
				//Go to Fast polling
				polling_count++;
				polling_timer = QI_AICL_FAST_POLLING_TIME;
			}else{	//Polling MAX
				qirx_dbg(the_qirx, QIRX_INFO, "polling max count.\n");
				if((aicl_current_icl - qirx_step_current) > qirx_psns_stability_current){
					aicl_current_icl -= qirx_step_current;
				}
				result = qirx_update_power_supply(dc_psy, aicl_current_icl);
				if(result < 0){
					qirx_err("set error: qirx_update_power_supply\n");
				}
				qirx_dbg(the_qirx, QIRX_INFO, "set aicl_current_icl = %d\n", aicl_current_icl);
				//Go to Slow polling
				polling_timer = QI_AICL_SLOW_POLLING_TIME;
				icl_settled = true;
				polling_count = 0;
			}
		}
	}

LOOP:
	qirx_dbg(the_qirx, QIRX_DEBUG, "Go to %d ms polling.\n", polling_timer);
	schedule_delayed_work(&the_qirx->qirx_rerun_aicl_loop_work, msecs_to_jiffies(polling_timer));
	qirx_dbg(the_qirx, QIRX_DEBUG, "[E]%s\n",__FUNCTION__);
}

//QI RX_ID REG
#define RX_ID_B0_REG	0x79	//Qi Major version & Minor Version register
#define RX_ID_B1_REG	0x7A	//Qi Manufacture Code Register
#define RX_ID_B2_REG	0x7B	//Qi Manufacture Code Register
#define RX_ID_B3_REG	0x7C	//Qi Manufacture Code Register
#define CHIP_ID_REG		0xDF	//REVISION register
//QI RX_ID VALUE
#define RX_ID_B0_VAL	0x12
#define RX_ID_B1_VAL	0x00
#define RX_ID_B2_VAL	0x27
#define RX_ID_B3_VAL	0x16
//QI RX_IC INT IN Init
#define QI_INTEN1_REG	0x10
#define QI_INTEN2_REG	0x11
#define QI_INTEN1_PMA_EOC_QI_EPT_BIT	0x20
#define QI_INTEN1_CHG_START_DET_BIT		0x01
#define QI_INTEN2_ERR_POSSET_CLR_BIT	0x02
#define QI_INTEN2_EN_ERR_POSSET_BIT		0x01
//QI NTC_SET REG
#define QI_NTC_SET_REG	0x0A
#define QI_NTC_SET_VAL	0x04
//QI GPOSEL1 REG
#define QI_GPOSEL1_REG	0x95
#define QI_GPOSEL1_VAL	0x1B
//QI RSVD1 REG
#define QI_RSVD1_REG	0x51
#define QI_RSVD1_VAL	0x04
//QI RSVD2 REG
#define QI_RSVD2_REG	0xA6
#define QI_RSVD2_VAL	0x00
//QI RSVD3 REG
#define QI_RSVD3_REG	0xB8
#define QI_RSVD3_VAL	0x02
//QI RSVD4 REG
#define QI_RSVD4_REG	0xA7
#define QI_RSVD4_VAL	0x01
//QI RSVD5 REG
#define QI_RSVD5_REG	0xAC
#define QI_RSVD5_VAL	0x80
//QI RSVD7 REG
#define QI_RSVD7_REG	0xB0
#define QI_RSVD7_VAL	0x30
//QI RSVD8 REG
#define QI_RSVD8_REG	0x29
#define QI_RSVD8_VAL	0xC2
//QI FOD3LL
#define QI_FOD3LL_REG	0x2A
#define QI_FOD3LL_VAL	0x02
//QI FOD3B
#define QI_FOD3B_REG	0x28
#define QI_FOD3B_VAL	0x81
//QI RX_CONF_B4 REG
#define QI_RX_CONF_B4_REG	0x7F
#define QI_RX_CONF_B4_VAL	0x0B
//QI ALIGN_DET_EN
#define QI_ALIGN_DET_EN	0x67
#define QI_ALIGN_DET_EN_WAKEUP	0x01
//QI_POS_GAP_LV_SET
#define QI_POS_GAP_LV_SET 0x6A
#define QI_POS_GAP_LV_SET_VAL 0x06
static int32_t qirx_probe( struct i2c_client *client, const struct i2c_device_id *id )
{
	int result = 0;
	unsigned short addr = 0x00;
	unsigned short data = 0x00;
	unsigned short chip_addr = 0x00;
	unsigned short chip_data = 0x00;
	unsigned short write_addr = 0x00;
	unsigned short write_data_first = 0x00;
	unsigned short write_data_second = 0x00;
	int qirx_id[] = {RX_ID_B0_VAL, RX_ID_B1_VAL, RX_ID_B2_VAL, RX_ID_B3_VAL};
	int qirx_id_prop[] = {0x00, 0x00, 0x00, 0x00};
	struct qirx *qirx;

	int_n_irqNo = -1;
	pg_irqNo = -1;
	addet_irqNo = -1;

	qirx_charging_flag_now = false;
	qirx_id_1912_det_flag = false;
	qirx_id_check_flag = false;
	taper_aicl_const = 0;
	pred_volt_const = 0;
	moni_mode_get_complete_flag = false;
	epp_mode_flag = false;
	qirx_fod_set_flag = false;

	qirx = devm_kzalloc(&client->dev, sizeof(*qirx),GFP_KERNEL);
	if (!qirx)
		return -ENOMEM;

	the_client = client;
	qirx->dev = &client->dev;
	the_qirx = qirx;
	aicl_current_icl = PSNS_REFERENCE_CURRENT;
	polling_count = 0;
	maximum_power_value = 0;
	qirx->debug_mask = &qirx_debug_mask;

	qirx_dbg(the_qirx, QIRX_DEBUG, "[S]%s start.\n",__FUNCTION__);

	result = qirx_set_gpio_no(qirx);
	result = qirx_gpio_init(qirx);
	qirx_dbg(the_qirx, QIRX_DEBUG, "qirx_gpio_init result = %d\n",result);


	//EN GPIO ASSERT
	result |= qirx_gpio_request(qirx, QIRX_GPIO_PIN_EN_N);
	if (result < 0){
		qirx_err("QIRX_GPIO_PIN_EN_N request Error.\n");
		qirx_gpio_free(qirx, QIRX_GPIO_PIN_EN_N);
		return -ENODEV;
	}
	qirx_gpio_direction_output(qirx,QIRX_GPIO_PIN_EN_N,0);

	qirx_gpio_free(qirx, QIRX_GPIO_PIN_EN_N);

	//Qi Rx IC Connect Check
	addr = RX_ID_B0_REG;
	result |= qirx_read_regdata(client, addr, &data);
	qirx_id_prop[0] = data;
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n",addr,data);

	addr = RX_ID_B1_REG;
	result |= qirx_read_regdata(client, addr, &data);
	qirx_id_prop[1] = data;
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n",addr,data);

	addr = RX_ID_B2_REG;
	result |= qirx_read_regdata(client, addr, &data);
	qirx_id_prop[2] = data;
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n",addr,data);

	addr = RX_ID_B3_REG;
	result |= qirx_read_regdata(client, addr, &data);
	qirx_id_prop[3] = data;
	qirx_dbg(the_qirx, QIRX_DEBUG, "addr = 0x%02x , data = 0x%02x\n",addr,data);

	chip_addr = CHIP_ID_REG;
	result |= qirx_read_regdata(client, chip_addr, &chip_data);
	qirx_dbg(the_qirx, QIRX_DEBUG, "chip_addr = 0x%02x , chip_data = 0x%02x\n",chip_addr, chip_data);

	if(result < 0){
		qirx_err("probe fail, qirx read error.result = %d\n", result);
		return result;
	}

	if(memcmp(qirx_id,qirx_id_prop,sizeof(qirx_id)) == 0 ){
		pr_info("%s: Qi Rx IC  connect successful. chip_id = 0x%02x\n",__FUNCTION__, chip_data);
	}else{
		qirx_err("probe fail, Qi Rx IC cannot connect.\n");
		return -EPROBE_DEFER;
	}

	//Qi Rx init psy
	result = qirx_init_psy(qirx);
	if (result < 0) {
		qirx_err("Couldn't initialize qirx psy result = %d\n", result);
	}

	//PSNS Reference Init
	if (batt_psy == NULL) {
		batt_psy = power_supply_get_by_name("battery");
	}
	if(batt_psy == NULL)
	{
		qirx_err("batt_psy is NULL.\n");
		return -ENOMEM;
	}

	if (dc_psy == NULL) {
		dc_psy = power_supply_get_by_name("dc");
	}
	if(dc_psy == NULL)
	{
		qirx_err("dc_psy is NULL.\n");
		return -ENOMEM;
	}
	result = qirx_update_power_supply(dc_psy,PSNS_REFERENCE_CURRENT);

	//Qi RST RESET
	result = qirx_reset(qirx);
	if (result < 0) {
		qirx_err("Couldn't qirx reset result = %d\n", result);
		return result;
	}

	//QI POS_GAP_LV_SET REG
	result = qirx_write_regdata_single(client, QI_POS_GAP_LV_SET, QI_POS_GAP_LV_SET_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_POS_GAP_LV_SET write error.result = %d\n", result);
		return result;
	}

	//QI ALIGN_DET_EN Enable
	result = qirx_write_regdata_single(client, QI_ALIGN_DET_EN, QI_ALIGN_DET_EN_WAKEUP);
	if(result < 0){
		qirx_err("probe fail, QI_ALIGN_DET_EN write error.result = %d\n", result);
		return result;
	}

	//Qi RX IC INTEN1/2 IRQ Enable
	write_addr = QI_INTEN1_REG;	//0x10
	write_data_first = QI_INTEN1_PMA_EOC_QI_EPT_BIT;	//0x10 write data
	write_data_second = QI_INTEN2_ERR_POSSET_CLR_BIT | QI_INTEN2_EN_ERR_POSSET_BIT;	//0x11 write data
	result = qirx_write_regdata(client, write_addr, write_data_first, write_data_second);
	if(result < 0){
		qirx_err("probe fail, INTEN1/2 IRQ write error.result = %d\n", result);
		return result;
	}

	//Qi RX IC INTCLR1/2 Enable
	result = qirx_clear_irq(client);
	if(result < 0){
		qirx_err("probe fail, INTCLR1/2 IRQ write error.result = %d\n", result);
		return result;
	}

	//QI NTC_SET REG Enable
	result = qirx_write_regdata_single(client, QI_NTC_SET_REG, QI_NTC_SET_VAL);
	if(result < 0){
		qirx_err("probe fail, NTC_SET write error.result = %d\n", result);
		return result;
	}

	//Qi RX IC GPOSEL1 Enable
	result = qirx_write_regdata_single(client, QI_GPOSEL1_REG, QI_GPOSEL1_VAL);
	if(result < 0){
		qirx_err("probe fail, GPOSEL1 write error.result = %d\n", result);
		return result;
	}

	//QI RSVD1 REG Enable
	result = qirx_write_regdata_single(client, QI_RSVD1_REG, QI_RSVD1_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD1_REG write error.result = %d\n", result);
		return result;
	}

	//QI RSVD2 REG Enable
	result = qirx_write_regdata_single(client, QI_RSVD2_REG, QI_RSVD2_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD2_REG write error.result = %d\n", result);
		return result;
	}

	//QI RSVD3 REG Enable
	result = qirx_write_regdata_single(client, QI_RSVD3_REG, QI_RSVD3_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD3_REG write error.result = %d\n", result);
		return result;
	}

	//QI RSVD4 REG Enable
	result = qirx_write_regdata_single(client, QI_RSVD4_REG, QI_RSVD4_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD4_REG write error.result = %d\n", result);
		return result;
	}

	//QI RSVD5 REG Enable
	result = qirx_write_regdata_single(client, QI_RSVD5_REG, QI_RSVD5_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD5_REG write error.result = %d\n", result);
		return result;
	}

	//QI RSVD6 = DPSET2

	//QI RSVD7 REG Enable
	result = qirx_write_regdata_single(client, QI_RSVD7_REG, QI_RSVD7_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD7_REG write error.result = %d\n", result);
		return result;
	}

	//QI RSVD8 REG Enable
	result = qirx_write_regdata_single(client, QI_RSVD8_REG, QI_RSVD8_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD8_REG write error.result = %d\n", result);
		return result;
	}

	//QI RSVD9 REG
	result = qirx_write_regdata_single(client, QI_RSVD9_REG, QI_RSVD9_VAL_DEFAULT);
	if(result < 0){
		qirx_err("probe fail, QI_RSVD9_REG write error.result = %d\n", result);
		return result;
	}

	//QI DPSET2 REG Enable
	result = qirx_write_regdata_single(client, QI_DPSET2_REG, QI_DPSET2_VAL_DEFAULT);
	if(result < 0){
		qirx_err("probe fail, QI_DPSET2_REG write error.result = %d\n", result);
		return result;
	}

	//QI FOD3LL REG Enable
	result = qirx_write_regdata_single(client, QI_FOD3LL_REG, QI_FOD3LL_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_FOD3LL_REG write error.result = %d\n", result);
		return result;
	}

	//QI FOD3B REG Enable
	result = qirx_write_regdata_single(client, QI_FOD3B_REG, QI_FOD3B_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_FOD3B_REG write error.result = %d\n", result);
		return result;
	}

	//QI RX_CONF_B4 REG Enable
	result = qirx_write_regdata_single(client, QI_RX_CONF_B4_REG, QI_RX_CONF_B4_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_RX_CONF_B4_REG write error.result = %d\n", result);
		return result;
	}

	//QI_GPIO_DAT_SEL3 Enable
	result = qirx_write_regdata_single(client, QI_GPIO_DAT_SEL3_REG, QI_GPIO_DAT_SEL3_VAL);
	if(result < 0){
		qirx_err("probe fail, QI_GPIO_DAT_SEL3_REG write error.result = %d\n", result);
		return result;
	}

	//QI_OUTSET_FOR_EPP set Default
	result = qirx_write_regdata_single(client, QI_OUTSET_FOR_EPP_REG, QI_OUTSET_FOR_EPP_DEFAULT);
	if(result < 0){
		qirx_err("probe fail, QI_OUTSET_FOR_EPP_REG write error.result = %d\n", result);
	}

	qirx_wake_lock_init();

	if(int_n_irqNo == -1 || pg_irqNo == -1 || addet_irqNo == -1){
		result = qirx_irq_init(qirx);
		if(result){
			qirx_err("Failed qirx_irq_init. ret=%x\n", result);
		}
	}

	//workqueue initialize
	qirx->wq = alloc_ordered_workqueue("qirx_wq", WQ_FREEZABLE | WQ_HIGHPRI);
	if (!qirx->wq) {
		result = -ENOMEM;
		qirx_err("probe fail, alloc_ordered_workqueue failed. ret = %d\n", result);
		return result;
	}

	INIT_WORK(&qirx->int_n_irq_work, qirx_int_n_irq);
	INIT_WORK(&qirx->sm_work, qirx_sm);
	INIT_DELAYED_WORK(&the_qirx->qirx_rerun_aicl_loop_work, qirx_rerun_psued_aicl);

	hrtimer_init(&qirx->qirx_kick_sm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	qirx->qirx_kick_sm_timer.function = qirx_kick_sm_timeout;
	hrtimer_init(&qirx->guiding_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	qirx->guiding_timer.function = guiding_timeout;
//	hrtimer_init(&qirx->qi_ept_count_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//	qirx->qi_ept_count_timer.function = qi_ept_count_timeout;

	qirx->psy_nb.notifier_call = qirx_psy_changed;
	result = power_supply_reg_notifier(&qirx->psy_nb);

	if(result < 0){
		qirx_err("probe fail, power_supply_reg_notifier failed. ret = %d\n", result);
	}

	qirx_psy_changed(&qirx->psy_nb, PSY_EVENT_PROP_CHANGED, batt_psy);

	qirx_dbg(the_qirx, QIRX_DEBUG, "[E]%s End.\n",__FUNCTION__);

	return 0;
}


static int32_t qirx_remove( struct i2c_client *client )
{
	qirx_wake_lock_destroy();
	cancel_delayed_work_sync(&the_qirx->qirx_rerun_aicl_loop_work);
	return 0;
}


static int32_t qirx_suspend( struct device *dev )
{
    return 0;
}


static int32_t qirx_resume( struct device *dev )
{
    return 0;
}


static void qirx_shutdown( struct i2c_client *client )
{
}

static int __init qirx_init( void )
{
	int ret;

	pr_debug("[S] qirx_init start.\n");

	ret = i2c_add_driver( &qirx_interface_driver );

	if(ret != 0){
		pr_err("%s: can't regist i2c driver ret = %d\n",__FUNCTION__, ret);
	}

	pr_debug("[E] qirx_init end\n");

	return ret;
}

static void __exit qirx_exit( void )
{
	pr_debug("[S] qirx_exit start.\n");

	i2c_del_driver( &qirx_interface_driver );

	pr_debug("[E] qirx_exit end\n");
}

module_init(qirx_init);
module_exit(qirx_exit);

MODULE_DESCRIPTION("SH Battery Qi Rx Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/
