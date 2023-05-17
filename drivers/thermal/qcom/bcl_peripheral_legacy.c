/*
 * Copyright (c) 2014-2019, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/spmi.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/msm_bcl_legacy.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>

#include "../thermal_core.h"

#define BCL_DRIVER_NAME       "bcl_peripheral_legacy"
#define BCL_VBAT_INT          "bcl-low-vbat-int"
#define BCL_IBAT_INT          "bcl-high-ibat-int"
#define BCL_MONITOR_EN        0x46
#define BCL_VBAT_MIN          0x58
#define BCL_IBAT_MAX          0x59
#define BCL_V_GAIN_BAT        0x60
#define BCL_I_GAIN_RSENSE     0x61
#define BCL_I_OFFSET_RSENSE   0x62
#define BCL_I_GAIN_BATFET     0x63
#define BCL_I_OFFSET_BATFET   0x64
#define BCL_I_SENSE_SRC       0x65
#define BCL_VBAT_MIN_CLR      0x66
#define BCL_IBAT_MAX_CLR      0x67
#define BCL_VBAT_LOW          0x68
#define BCL_IBAT_HIGH         0x69
#define BCL_READ_RETRY_LIMIT  3
#define VAL_CP_REG_BUF_LEN    3
#define VAL_REG_BUF_OFFSET    0
#define VAL_CP_REG_BUF_OFFSET 2
#define BCL_VBAT_NO_READING   127
#define BCL_CONSTANT_NUM      32

#define PON_SPARE_FULL_CURRENT		0x0
#define PON_SPARE_DERATED_CURRENT	0x1

#define READ_CONV_FACTOR(_node, _key, _val, _ret, _dest) do { \
		_ret = of_property_read_u32(_node, _key, &_val); \
		if (_ret) { \
			pr_err("Error reading key:%s. err:%d\n", _key, _ret); \
			return _ret; \
		} \
		_dest = _val; \
	} while (0)

#define READ_OPTIONAL_PROP(_node, _key, _val, _ret, _dest) do { \
		_ret = of_property_read_u32(_node, _key, &_val); \
		if (_ret && _ret != -EINVAL) { \
			pr_err("Error reading key:%s. err:%d\n", _key, _ret); \
			return _ret; \
		} else if (!_ret) { \
			_dest = _val; \
		} \
	} while (0)

struct bcl_peripheral_data {
	int                     irq_num;
	long int		trip_temp;
	int                     trip_val;
	int                     last_val;
	int                     scaling_factor;
	int                     offset_factor_num;
	int                     offset_factor_den;
	int                     offset;
	int                     gain_factor_num;
	int                     gain_factor_den;
	int                     gain;
	int                     inhibit_derating_ua;
	struct mutex            state_trans_lock;
	bool			irq_enabled;
	struct thermal_zone_of_device_ops ops;
	struct thermal_zone_device *tz_dev;
};

struct bcl_device {
	struct regmap			*regmap;
	uint16_t			fg_bcl_addr;
	uint16_t			pon_spare_addr;
	int					i_src;
	struct notifier_block		psy_nb;
	struct work_struct		soc_eval_work;
	struct bcl_peripheral_data	param[BCL_LEGACY_TYPE_MAX];
};

static struct bcl_device *bcl_perph;
static struct power_supply_desc bcl_psy_d;
static struct power_supply *bcl_psy;
static const char bcl_psy_name[] = "fg_adc";
static bool calibration_done;
static bool probe_done = false;

bool msm_bcl_is_legacy(void)
{
	return probe_done;
}

static int bcl_read_multi_register(int16_t reg_offset, uint8_t *data, int len)
{
	int ret = 0;

	if (!bcl_perph) {
		pr_err("BCL device not initialized\n");
		return -EINVAL;
	}
	ret = regmap_bulk_read(bcl_perph->regmap,
				(bcl_perph->fg_bcl_addr + reg_offset),
				data, len);
	if (ret < 0) {
		pr_err("Error reading register %d. err:%d", reg_offset, ret);
		return ret;
	}

	return ret;
}

static int bcl_write_general_register(int16_t reg_offset,
					uint16_t base, uint8_t data)
{
	int ret = 0;
	uint8_t *write_buf = &data;

	if (!bcl_perph) {
		pr_err("BCL device not initialized\n");
		return -EINVAL;
	}
	ret = regmap_write(bcl_perph->regmap, (base + reg_offset), *write_buf);
	if (ret < 0) {
		pr_err("Error reading register %d. err:%d", reg_offset, ret);
		return ret;
	}
	pr_debug("wrote 0x%02x to 0x%04x\n", data, base + reg_offset);

	return ret;
}

static int bcl_write_register(int16_t reg_offset, uint8_t data)
{
	return bcl_write_general_register(reg_offset,
			bcl_perph->fg_bcl_addr, data);
}

static void convert_vbat_to_adc_val(int *val)
{
	struct bcl_peripheral_data *perph_data = NULL;

	if (!bcl_perph)
		return;

	perph_data = &bcl_perph->param[BCL_LEGACY_LOW_VBAT];

	*val = ((*val * 100 * 1000) 
		/ (100 + (perph_data->gain_factor_num
		* perph_data->gain) * BCL_CONSTANT_NUM
		/ perph_data->gain_factor_den))
		/ perph_data->scaling_factor;
}

static void convert_adc_to_vbat_val(int *val)
{
	struct bcl_peripheral_data *perph_data = NULL;

	if (!bcl_perph)
		return;

	perph_data = &bcl_perph->param[BCL_LEGACY_LOW_VBAT];

	*val = (((*val + 2) * perph_data->scaling_factor)
		* (100 + (perph_data->gain_factor_num
		* perph_data->gain)
		* BCL_CONSTANT_NUM  / perph_data->gain_factor_den)
		/ 100) / 1000;
}

static void convert_ibat_to_adc_val(int *val)
{
	struct bcl_peripheral_data *perph_data = NULL;

	if (!bcl_perph)
		return;

	perph_data = &bcl_perph->param[BCL_LEGACY_HIGH_IBAT];

	*val = ((*val * 100 * 1000)
		/ (100 + (perph_data->gain_factor_num
		* perph_data->gain)
		* BCL_CONSTANT_NUM / perph_data->gain_factor_den)
		- (perph_data->offset_factor_num * perph_data->offset)
		/ perph_data->offset_factor_den)
		/  perph_data->scaling_factor;
}

static void convert_adc_to_ibat_val(int *val)
{
	struct bcl_peripheral_data *perph_data = NULL;

	if (!bcl_perph)
		return;

	perph_data = &bcl_perph->param[BCL_LEGACY_HIGH_IBAT];

	*val = ((*val * perph_data->scaling_factor
		+ (perph_data->offset_factor_num * perph_data->offset)
		/ perph_data->offset_factor_den)
		* (100 + (perph_data->gain_factor_num
		* perph_data->gain) * BCL_CONSTANT_NUM /
		perph_data->gain_factor_den) / 100) / 1000;
}

static int bcl_set_ibat(void *data, int low, int high)
{
	int ret = 0, ibat_ua, thresh_value;
	int8_t val = 0;
	struct bcl_peripheral_data *bat_data =
		(struct bcl_peripheral_data *)data;

	thresh_value = high;
	if (bat_data->trip_temp == thresh_value)
		return 0;

	mutex_lock(&bat_data->state_trans_lock);
	if (bat_data->irq_num && bat_data->irq_enabled) {
		disable_irq_nosync(bat_data->irq_num);
		bat_data->irq_enabled = false;
	}
	if (thresh_value == INT_MAX) {
		bat_data->trip_temp = thresh_value;
		goto set_trip_exit;
	}

	ibat_ua = thresh_value;
	convert_ibat_to_adc_val(&thresh_value);
	val = (int8_t)thresh_value;

	pr_debug("ibat high threshold:%d mA ADC:0x%02x\n",
			ibat_ua, val);

	ret = bcl_write_register(BCL_IBAT_HIGH, val);
	if (ret) {
		pr_err("Error accessing BCL peripheral. err:%d\n", ret);
		goto set_trip_exit;
	}
	bat_data->trip_temp = ibat_ua;

	if (bat_data->irq_num && !bat_data->irq_enabled) {
		enable_irq(bat_data->irq_num);
		bat_data->irq_enabled = true;
	}

	if (bcl_perph->param[BCL_LEGACY_HIGH_IBAT].inhibit_derating_ua == 0
			|| bcl_perph->pon_spare_addr == 0)
		goto set_trip_exit;

	ret = bcl_write_general_register(bcl_perph->pon_spare_addr,
			PON_SPARE_FULL_CURRENT, val);
	if (ret) {
		pr_debug("Error accessing PON register. err:%d\n", ret);
		goto set_trip_exit;
	}
	thresh_value = ibat_ua
		- bcl_perph->param[BCL_LEGACY_HIGH_IBAT].inhibit_derating_ua;
	convert_ibat_to_adc_val(&thresh_value);
	val = (int8_t)thresh_value;
	ret = bcl_write_general_register(bcl_perph->pon_spare_addr,
			PON_SPARE_DERATED_CURRENT, val);
	if (ret) {
		pr_debug("Error accessing PON register. err:%d\n", ret);
		goto set_trip_exit;
	}

set_trip_exit:
	mutex_unlock(&bat_data->state_trans_lock);

	return ret;
}

static int bcl_set_vbat(void *data, int low, int high)
{
	int ret = 0, vbat_uv, thresh_value;
	int8_t val = 0;
	struct bcl_peripheral_data *bat_data =
		(struct bcl_peripheral_data *)data;

	thresh_value = low;
	if (bat_data->trip_temp == thresh_value)
		return 0;

	mutex_lock(&bat_data->state_trans_lock);

	if (bat_data->irq_num && bat_data->irq_enabled) {
		disable_irq_nosync(bat_data->irq_num);
		bat_data->irq_enabled = false;
	}
	if (thresh_value == INT_MIN) {
		bat_data->trip_temp = thresh_value;
		goto set_trip_exit;
	}
	vbat_uv = thresh_value;
	convert_vbat_to_adc_val(&thresh_value);
	val = (int8_t)thresh_value;

	pr_debug("vbat low threshold:%d mv ADC:0x%02x\n",
			vbat_uv, val);

	ret = bcl_write_register(BCL_VBAT_LOW, val);
	if (ret) {
		pr_err("Error accessing BCL peripheral. err:%d\n", ret);
		goto set_trip_exit;
	}
	bat_data->trip_temp = vbat_uv;
	if (bat_data->irq_num && !bat_data->irq_enabled) {
		enable_irq(bat_data->irq_num);
		bat_data->irq_enabled = true;
	}

set_trip_exit:
	mutex_unlock(&bat_data->state_trans_lock);
	return ret;
}

static int bcl_clear_vbat_min(void)
{
	int ret  = 0;

	ret = bcl_write_register(BCL_VBAT_MIN_CLR, BIT(7));
	if (ret)
		pr_err("Error in clearing vbat min reg. err:%d", ret);

	return ret;
}

static int bcl_clear_ibat_max(void)
{
	int ret  = 0;

	ret = bcl_write_register(BCL_IBAT_MAX_CLR, BIT(7));
	if (ret)
		pr_err("Error in clearing ibat max reg. err:%d", ret);

	return ret;
}

static int bcl_read_ibat(void *data, int *adc_value)
{
	int ret = 0, timeout = 0;
	int8_t val[VAL_CP_REG_BUF_LEN] = {0};
	struct bcl_peripheral_data *bat_data =
		(struct bcl_peripheral_data *)data;

	*adc_value = (int)val[VAL_REG_BUF_OFFSET];
	do {
		ret = bcl_read_multi_register(BCL_IBAT_MAX, val,
			VAL_CP_REG_BUF_LEN);
		if (ret) {
			pr_err("BCL register read error. err:%d\n", ret);
			goto bcl_read_exit;
		}
	} while (val[VAL_REG_BUF_OFFSET] != val[VAL_CP_REG_BUF_OFFSET]
		&& timeout++ < BCL_READ_RETRY_LIMIT);
	if (val[VAL_REG_BUF_OFFSET] != val[VAL_CP_REG_BUF_OFFSET]) {
		ret = -ENODEV;
		*adc_value = bat_data->last_val;
		goto bcl_read_exit;
	}
	*adc_value = (int)val[VAL_REG_BUF_OFFSET];
	if (*adc_value == 0) {
		/*
		 * The sensor sometime can read a value 0 if there is
		 * consequtive reads
		 */
		*adc_value = bat_data->last_val;
	} else {
		convert_adc_to_ibat_val(adc_value);
		bat_data->last_val = *adc_value;
	}
	pr_debug("ibat:%d mA\n", bat_data->last_val);

bcl_read_exit:
	return ret;
}

static int bcl_read_ibat_and_clear(void *data, int *adc_value)
{
	int ret = 0;

	ret = bcl_read_ibat(data, adc_value);
	if (ret)
		return ret;
	return bcl_clear_ibat_max();
}

static int bcl_read_vbat(void *data, int *adc_value)
{
	int ret = 0, timeout = 0;
	int8_t val[VAL_CP_REG_BUF_LEN] = {0};
	struct bcl_peripheral_data *bat_data =
		(struct bcl_peripheral_data *)data;

	*adc_value = (int)val[VAL_REG_BUF_OFFSET];
	do {
		ret = bcl_read_multi_register(BCL_VBAT_MIN, val,
			VAL_CP_REG_BUF_LEN);
		if (ret) {
			pr_err("BCL register read error. err:%d\n", ret);
			goto bcl_read_exit;
		}
	} while (val[VAL_REG_BUF_OFFSET] != val[VAL_CP_REG_BUF_OFFSET]
		&& timeout++ < BCL_READ_RETRY_LIMIT);
	if (val[VAL_REG_BUF_OFFSET] != val[VAL_CP_REG_BUF_OFFSET]) {
		ret = -ENODEV;
		goto bcl_read_exit;
	}
	*adc_value = (int)val[VAL_REG_BUF_OFFSET];
	if (*adc_value == BCL_VBAT_NO_READING) {
		*adc_value = bat_data->last_val;
	} else {
		convert_adc_to_vbat_val(adc_value);
		bat_data->last_val = *adc_value;
	}
	pr_debug("vbat:%d mv\n", bat_data->last_val);

bcl_read_exit:
	return ret;
}

static int bcl_read_vbat_and_clear(void *data, int *adc_value)
{
	int ret;

	ret = bcl_read_vbat(data, adc_value);
	if (ret)
		return ret;
	return bcl_clear_vbat_min();
}

static irqreturn_t bcl_handle_ibat(int irq, void *data)
{
	struct bcl_peripheral_data *perph_data =
		(struct bcl_peripheral_data *)data;
	bool irq_enabled = false;

	mutex_lock(&perph_data->state_trans_lock);
	irq_enabled = perph_data->irq_enabled;
	mutex_unlock(&perph_data->state_trans_lock);

	if (irq_enabled)
		of_thermal_handle_trip(perph_data->tz_dev);

	return IRQ_HANDLED;
}

static irqreturn_t bcl_handle_vbat(int irq, void *data)
{
	struct bcl_peripheral_data *perph_data =
		(struct bcl_peripheral_data *)data;
	bool irq_enabled = false;

	mutex_lock(&perph_data->state_trans_lock);
	irq_enabled = perph_data->irq_enabled;
	mutex_unlock(&perph_data->state_trans_lock);

	if (irq_enabled)
		of_thermal_handle_trip(perph_data->tz_dev);

	return IRQ_HANDLED;
}

int msm_bcl_legacy_read(enum bcl_legacy_dev_type type, int *value)
{
	int ret = 0;

	if (!bcl_perph) {
		pr_debug("BCL device not initialized yet\n");
		return -EINVAL;
	}

	if (!value || type >= BCL_LEGACY_TYPE_MAX) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}

	if (type == BCL_LEGACY_HIGH_IBAT)
		ret = bcl_read_ibat(&bcl_perph->param[type], value);
	else if (type == BCL_LEGACY_LOW_VBAT)
		ret = bcl_read_vbat(&bcl_perph->param[type], value);

	if (ret) {
		pr_err("Error reading param%d. err: %d\n", type, ret);
		return ret;
	}

	return ret;
}

static int bcl_get_devicetree_data(struct platform_device *pdev)
{
	int ret = 0, temp_val = 0;
	char *key = NULL;
	const __be32 *prop = NULL;
	struct device_node *dev_node = pdev->dev.of_node;

	prop = of_get_address(dev_node, 0, NULL, NULL);
	if (prop) {
		bcl_perph->fg_bcl_addr = be32_to_cpu(*prop);
		pr_debug("fg_user_adc@%04x\n", bcl_perph->fg_bcl_addr);
	} else {
		dev_err(&pdev->dev, "No fg_user_adc registers found\n");
		return -ENODEV;
	}

	prop = of_get_address(dev_node, 1, NULL, NULL);
	if (prop) {
		bcl_perph->pon_spare_addr = be32_to_cpu(*prop);
		pr_debug("pon_spare@%04x\n", bcl_perph->pon_spare_addr);
	}

	/* Get VADC and IADC scaling factor */
	key = "qcom,vbat-scaling-factor";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_LOW_VBAT].scaling_factor);
	key = "qcom,vbat-gain-numerator";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_LOW_VBAT].gain_factor_num);
	key = "qcom,vbat-gain-denominator";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_LOW_VBAT].gain_factor_den);
	key = "qcom,ibat-scaling-factor";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_HIGH_IBAT].scaling_factor);
	key = "qcom,ibat-offset-numerator";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_HIGH_IBAT].offset_factor_num);
	key = "qcom,ibat-offset-denominator";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_HIGH_IBAT].offset_factor_den);
	key = "qcom,ibat-gain-numerator";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_HIGH_IBAT].gain_factor_num);
	key = "qcom,ibat-gain-denominator";
	READ_CONV_FACTOR(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_HIGH_IBAT].gain_factor_den);
	key = "qcom,inhibit-derating-ua";
	READ_OPTIONAL_PROP(dev_node, key, temp_val, ret,
		bcl_perph->param[BCL_LEGACY_HIGH_IBAT].
		inhibit_derating_ua);

	return ret;
}

static int bcl_calibrate(void)
{
	int ret = 0;
	int8_t i_src = 0, val = 0;

	ret = bcl_read_multi_register(BCL_I_SENSE_SRC, &i_src, 1);
	if (ret) {
		pr_err("Error reading current sense reg. err:%d\n", ret);
		goto bcl_cal_exit;
	}

	ret = bcl_read_multi_register((i_src & 0x01) ?
		BCL_I_GAIN_RSENSE : BCL_I_GAIN_BATFET, &val, 1);
	if (ret) {
		pr_err("Error reading %s current gain. err:%d\n",
			(i_src & 0x01) ? "rsense" : "batfet", ret);
		goto bcl_cal_exit;
	}
	bcl_perph->param[BCL_LEGACY_HIGH_IBAT].gain = val;
	ret = bcl_read_multi_register((i_src & 0x01) ?
		BCL_I_OFFSET_RSENSE : BCL_I_OFFSET_BATFET, &val, 1);
	if (ret) {
		pr_err("Error reading %s current offset. err:%d\n",
			(i_src & 0x01) ? "rsense" : "batfet", ret);
		goto bcl_cal_exit;
	}
	bcl_perph->param[BCL_LEGACY_HIGH_IBAT].offset = val;
	ret = bcl_read_multi_register(BCL_V_GAIN_BAT, &val, 1);
	if (ret) {
		pr_err("Error reading vbat offset. err:%d\n", ret);
		goto bcl_cal_exit;
	}
	bcl_perph->param[BCL_LEGACY_LOW_VBAT].gain = val;

	if (((i_src & 0x01) != bcl_perph->i_src)
		&& (bcl_perph->param[BCL_LEGACY_LOW_VBAT].irq_enabled &&
			bcl_perph->param[BCL_LEGACY_HIGH_IBAT].irq_enabled)) {
		bcl_set_vbat(&bcl_perph->param[BCL_LEGACY_LOW_VBAT],
				bcl_perph->param[BCL_LEGACY_LOW_VBAT].trip_val, 0);
		bcl_set_ibat(&bcl_perph->param[BCL_LEGACY_HIGH_IBAT], 0,
				bcl_perph->param[BCL_LEGACY_HIGH_IBAT].trip_val);
		bcl_perph->i_src = i_src;
	}

bcl_cal_exit:
	return ret;
}

static void power_supply_callback(struct power_supply *psy)
{
	static struct power_supply *bms_psy;
	int ret = 0;

	if (calibration_done)
		return;

	if (!bms_psy)
		bms_psy = power_supply_get_by_name("bms");
	if (bms_psy) {
		calibration_done = true;
		pr_debug("Recalibrate callback");
		ret = bcl_calibrate();
		if (ret)
			pr_err("Could not read calibration values. err:%d",
				ret);
	}
}

static int bcl_psy_get_property(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *val)
{
	return 0;
}
static int bcl_psy_set_property(struct power_supply *psy,
				enum power_supply_property prop,
				const union power_supply_propval *val)
{
	return -EINVAL;
}

static int bcl_set_soc(void *data, int low, int high)
{
	struct bcl_peripheral_data *bat_data =
		(struct bcl_peripheral_data *)data;

	if (low == bat_data->trip_temp)
		return 0;

	mutex_lock(&bat_data->state_trans_lock);
	pr_debug("low soc threshold:%d\n", low);
	bat_data->trip_temp = low;
	if (low == INT_MIN) {
		bat_data->irq_enabled = false;
		goto unlock_and_exit;
	}
	bat_data->irq_enabled = true;
	schedule_work(&bcl_perph->soc_eval_work);

unlock_and_exit:
	mutex_unlock(&bat_data->state_trans_lock);
	return 0;
}

static int bcl_read_soc(void *data, int *val)
{
	static struct power_supply *batt_psy;
	union power_supply_propval ret = {0,};
	int err = 0;

	*val = 100;
	if (!batt_psy)
		batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		err = power_supply_get_property(batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (err) {
			pr_err("battery percentage read error:%d\n",
				err);
			return err;
		}
		*val = ret.intval;
	}
	pr_debug("soc:%d\n", *val);

	return err;
}

static void bcl_evaluate_soc(struct work_struct *work)
{
	int battery_percentage;
	struct bcl_peripheral_data *perph_data =
		&bcl_perph->param[BCL_LEGACY_SOC_MONITOR];

	if (bcl_read_soc((void *)perph_data, &battery_percentage))
		return;

	mutex_lock(&perph_data->state_trans_lock);
	if (!perph_data->irq_enabled)
		goto eval_exit;
	if (battery_percentage > perph_data->trip_temp)
		goto eval_exit;

	perph_data->trip_val = battery_percentage;
	mutex_unlock(&perph_data->state_trans_lock);
	of_thermal_handle_trip(perph_data->tz_dev);

	return;
eval_exit:
	mutex_unlock(&perph_data->state_trans_lock);
}

static int battery_supply_callback(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct power_supply *psy = data;

	if (strcmp(psy->desc->name, "battery"))
		return NOTIFY_OK;
	schedule_work(&bcl_perph->soc_eval_work);

	return NOTIFY_OK;
}

static void bcl_fetch_trip(struct platform_device *pdev, const char *int_name,
		struct bcl_peripheral_data *data,
		irqreturn_t (*handle)(int, void *))
{
	int ret = 0, irq_num = 0;

	/*
	 * Allow flexibility for the HLOS to set the trip temperature for
	 * all the thresholds but handle the interrupt for only one vbat
	 * and ibat interrupt. The LMH-DCVSh will handle and mitigate for the
	 * rest of the ibat/vbat interrupts.
	 */
	if (!handle) {
		mutex_lock(&data->state_trans_lock);
		data->irq_num = 0;
		data->irq_enabled = false;
		mutex_unlock(&data->state_trans_lock);
		return;
	}

	irq_num = platform_get_irq_byname(pdev, int_name);
	if (irq_num) {
		mutex_lock(&data->state_trans_lock);
		ret = devm_request_threaded_irq(&pdev->dev,
				irq_num, NULL, handle,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				int_name, data);
		if (ret) {
			dev_err(&pdev->dev,
				"Error requesting trip irq. err:%d",
				ret);
			mutex_unlock(&data->state_trans_lock);
			return;
		}
		disable_irq_nosync(irq_num);
		data->irq_num = irq_num;
		data->irq_enabled = false;
		mutex_unlock(&data->state_trans_lock);
	}
}

static void bcl_probe_soc(struct platform_device *pdev)
{
	int ret = 0;
	struct bcl_peripheral_data *soc_data;

	soc_data = &bcl_perph->param[BCL_LEGACY_SOC_MONITOR];
	mutex_init(&soc_data->state_trans_lock);
	soc_data->ops.get_temp = bcl_read_soc;
	soc_data->ops.set_trips = bcl_set_soc;
	INIT_WORK(&bcl_perph->soc_eval_work, bcl_evaluate_soc);
	bcl_perph->psy_nb.notifier_call = battery_supply_callback;
	ret = power_supply_reg_notifier(&bcl_perph->psy_nb);
	if (ret < 0) {
		pr_err("Unable to register soc notifier. err:%d\n", ret);
		return;
	}
	soc_data->tz_dev = thermal_zone_of_sensor_register(&pdev->dev,
				BCL_LEGACY_SOC_MONITOR, soc_data, &soc_data->ops);
	if (IS_ERR(soc_data->tz_dev)) {
		pr_err("soc register failed. err:%ld\n",
				PTR_ERR(soc_data->tz_dev));
		return;
	}
	thermal_zone_device_update(soc_data->tz_dev, THERMAL_DEVICE_UP);
	schedule_work(&bcl_perph->soc_eval_work);
}

static void bcl_vbat_init(struct platform_device *pdev,
		struct bcl_peripheral_data *vbat)
{
	mutex_init(&vbat->state_trans_lock);
	bcl_fetch_trip(pdev, BCL_VBAT_INT, vbat, bcl_handle_vbat);
	vbat->ops.get_temp = bcl_read_vbat_and_clear;
	vbat->ops.set_trips = bcl_set_vbat;
	vbat->tz_dev = thermal_zone_of_sensor_register(&pdev->dev,
				BCL_LEGACY_LOW_VBAT, vbat, &vbat->ops);
	if (IS_ERR(vbat->tz_dev)) {
		pr_err("vbat register failed. err:%ld\n",
				PTR_ERR(vbat->tz_dev));
		return;
	}
	thermal_zone_device_update(vbat->tz_dev, THERMAL_DEVICE_UP);
}

static void bcl_ibat_init(struct platform_device *pdev,
		struct bcl_peripheral_data *ibat)
{
	mutex_init(&ibat->state_trans_lock);
	bcl_fetch_trip(pdev, BCL_IBAT_INT, ibat, bcl_handle_ibat);
	ibat->ops.get_temp = bcl_read_ibat_and_clear;
	ibat->ops.set_trips = bcl_set_ibat;
	ibat->tz_dev = thermal_zone_of_sensor_register(&pdev->dev,
				BCL_LEGACY_HIGH_IBAT, ibat, &ibat->ops);
	if (IS_ERR(ibat->tz_dev)) {
		pr_err("ibat register failed. err:%ld\n",
				PTR_ERR(ibat->tz_dev));
		return;
	}
	thermal_zone_device_update(ibat->tz_dev, THERMAL_DEVICE_UP);
}

static int bcl_remove(struct platform_device *pdev)
{
	int i = 0;

	for (; i < BCL_LEGACY_TYPE_MAX; i++) {
		if (!bcl_perph->param[i].tz_dev)
			continue;
		if (i == BCL_LEGACY_SOC_MONITOR) {
			power_supply_unreg_notifier(&bcl_perph->psy_nb);
			flush_work(&bcl_perph->soc_eval_work);
		}
		thermal_zone_of_sensor_unregister(&pdev->dev,
				bcl_perph->param[i].tz_dev);
	}
	bcl_perph = NULL;

	return 0;
}

static int bcl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct power_supply_config bcl_psy_cfg = {};

	bcl_perph = devm_kzalloc(&pdev->dev, sizeof(*bcl_perph), GFP_KERNEL);
	if (!bcl_perph)
		return -ENOMEM;

	bcl_perph->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!bcl_perph->regmap) {
		dev_err(&pdev->dev, "Couldn't get parent's regmap\n");
		return -EINVAL;
	}

	bcl_get_devicetree_data(pdev);
	bcl_ibat_init(pdev, &bcl_perph->param[BCL_LEGACY_HIGH_IBAT]);
	bcl_vbat_init(pdev, &bcl_perph->param[BCL_LEGACY_LOW_VBAT]);
	bcl_probe_soc(pdev);

	ret = bcl_calibrate();
	if (ret) {
		pr_debug("Could not read calibration values. err:%d",
			ret);
		goto bcl_probe_exit;
	}
	bcl_psy_d.name = bcl_psy_name;
	bcl_psy_d.type = POWER_SUPPLY_TYPE_BMS;
	bcl_psy_d.get_property = bcl_psy_get_property;
	bcl_psy_d.set_property = bcl_psy_set_property;
	bcl_psy_d.num_properties = 0;
	bcl_psy_d.external_power_changed = power_supply_callback;

	bcl_psy_cfg.num_supplicants = 0;
	bcl_psy_cfg.drv_data = bcl_perph;

	bcl_psy = devm_power_supply_register(&pdev->dev, &bcl_psy_d,
			&bcl_psy_cfg);
	if (IS_ERR(bcl_psy)) {
		pr_err("Unable to register bcl_psy rc = %ld\n",
		PTR_ERR(bcl_psy));
		return ret;
	}

	dev_set_drvdata(&pdev->dev, bcl_perph);
	ret = bcl_write_register(BCL_MONITOR_EN, BIT(7));
	if (ret) {
		pr_err("Error accessing BCL peripheral. err:%d\n", ret);
		goto bcl_probe_exit;
	}

	pr_info("BCL PMIC peripheral probed successfully\n");
	probe_done = true;
	return 0;

bcl_probe_exit:
	bcl_remove(pdev);
	return ret;
}

static const struct of_device_id bcl_match[] = {
	{	
		.compatible = "qcom,msm-bcl-legacy",
	},
	{},
};

static struct platform_driver bcl_driver = {
	.probe  = bcl_probe,
	.remove = bcl_remove,
	.driver = {
		.name           = BCL_DRIVER_NAME,
		.owner          = THIS_MODULE,
		.of_match_table = bcl_match,
	},
};

builtin_platform_driver(bcl_driver);
