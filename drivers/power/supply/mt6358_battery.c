// SPDX-License-Identifier: GPL-2.0
/*
 * Battery monitor for the fuel-gauge ADC of the MediaTek MT6358 PMIC.
 *
 * The PMIC measures battery voltage (through its auxiliary ADC) and battery
 * current (through a dedicated ADC across a sense resistor). MediaTek's own
 * gauge runs a large proprietary algorithm on top of the coulomb counter;
 * this driver only reads voltage and current and lets the ADC battery helper
 * estimate the state of charge from the open-circuit voltage curve of the
 * battery described in the device tree.
 */
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/math64.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "adc-battery-helper.h"

#define MT6358_FGADC_CON1		0xd0a
#define MT6358_FG_LATCHDATA_ST		BIT(15)
#define MT6358_FG_SW_CLEAR		BIT(3)
#define MT6358_FG_SW_READ_PRE		BIT(0)
#define MT6358_FGADC_CUR_CON0		0xd8a

/* One current ADC step is 381.47 uA across a 10 mOhm sense resistor */
#define MT6358_FG_NA_PER_LSB_10MOHM	381470
#define MT6358_FG_REF_SHUNT_UOHM	10000

/* Battery voltage readings from the auxiliary ADC are 15 bits wide */
#define MT6358_AUXADC_BITS		15

struct mt6358_battery {
	struct adc_battery_helper helper;	/* must be first */
	struct device *dev;
	struct regmap *regmap;
	struct iio_channel *vbat;
	u32 shunt_uohm;
	u32 gain_permille;
};

static int mt6358_battery_read_current(struct mt6358_battery *bat, int *ua)
{
	unsigned int val, raw;
	int ret;

	/* Latch the gauge's results into its readout registers */
	ret = regmap_write(bat->regmap, MT6358_FGADC_CON1, MT6358_FG_SW_READ_PRE);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(bat->regmap, MT6358_FGADC_CON1, val,
				       val & MT6358_FG_LATCHDATA_ST, 100, 50000);
	if (ret)
		return ret;

	ret = regmap_read(bat->regmap, MT6358_FGADC_CUR_CON0, &raw);

	/* Release the latch again, also when the read failed */
	regmap_update_bits(bat->regmap, MT6358_FGADC_CON1,
			   MT6358_FG_SW_CLEAR | MT6358_FG_SW_READ_PRE,
			   MT6358_FG_SW_CLEAR);
	regmap_read_poll_timeout(bat->regmap, MT6358_FGADC_CON1, val,
				 !(val & MT6358_FG_LATCHDATA_ST), 100, 50000);
	regmap_update_bits(bat->regmap, MT6358_FGADC_CON1, MT6358_FG_SW_CLEAR, 0);
	if (ret)
		return ret;

	/* Signed, positive while charging */
	*ua = div_s64((s64)(s16)raw * MT6358_FG_NA_PER_LSB_10MOHM *
		      (MT6358_FG_REF_SHUNT_UOHM / 1000) * bat->gain_permille,
		      (s64)bat->shunt_uohm * 1000);
	return 0;
}

static int mt6358_battery_get_voltage_and_current_now(struct power_supply *psy,
						       int *volt, int *curr)
{
	struct mt6358_battery *bat = power_supply_get_drvdata(psy);
	int raw, scale, scale2 = 0;
	int ret;

	/*
	 * The auxadc driver's scale is the full-scale voltage in mV (reference
	 * times divider ratio), without the division by the ADC range, so the
	 * IIO "processed" helpers cannot be used on it.
	 */
	ret = iio_read_channel_raw(bat->vbat, &raw);
	if (ret < 0)
		return ret;
	ret = iio_read_channel_scale(bat->vbat, &scale, &scale2);
	if (ret < 0)
		return ret;
	if (ret == IIO_VAL_FRACTIONAL && scale2)
		*volt = div_s64((s64)raw * scale * 1000, (s64)scale2 << MT6358_AUXADC_BITS);
	else
		*volt = div_s64((s64)raw * scale * 1000, 1 << MT6358_AUXADC_BITS);

	return mt6358_battery_read_current(bat, curr);
}

static const struct power_supply_desc mt6358_battery_desc = {
	.name		= "mt6358-battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= adc_battery_helper_get_property,
	.external_power_changed	= adc_battery_helper_external_power_changed,
	.properties	= adc_battery_helper_properties,
	.num_properties	= ADC_HELPER_NUM_PROPERTIES,
};

static int mt6358_battery_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	struct mt6358_battery *bat;
	struct mt6397_chip *pmic;
	struct power_supply *psy;

	bat = devm_kzalloc(dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = dev;
	pmic = dev_get_drvdata(dev->parent);
	if (!pmic || !pmic->regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");
	bat->regmap = pmic->regmap;

	bat->vbat = devm_iio_channel_get(dev, "vbat");
	if (IS_ERR(bat->vbat))
		return dev_err_probe(dev, PTR_ERR(bat->vbat), "no battery voltage channel\n");

	bat->shunt_uohm = MT6358_FG_REF_SHUNT_UOHM;
	device_property_read_u32(dev, "shunt-resistor-micro-ohms", &bat->shunt_uohm);
	if (!bat->shunt_uohm)
		return dev_err_probe(dev, -EINVAL, "bad shunt resistance\n");
	bat->gain_permille = 1000;
	device_property_read_u32(dev, "mediatek,current-gain-permille", &bat->gain_permille);

	platform_set_drvdata(pdev, bat);

	psy_cfg.drv_data = bat;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy = devm_power_supply_register(dev, &mt6358_battery_desc, &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "registering the power supply failed\n");

	return adc_battery_helper_init(&bat->helper, psy,
				       mt6358_battery_get_voltage_and_current_now, NULL);
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6358_battery_pm_ops, adc_battery_helper_suspend,
				adc_battery_helper_resume);

static const struct of_device_id mt6358_battery_of_match[] = {
	{ .compatible = "mediatek,mt6358-battery" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6358_battery_of_match);

static struct platform_driver mt6358_battery_driver = {
	.driver = {
		.name = "mt6358-battery",
		.of_match_table = mt6358_battery_of_match,
		.pm = pm_sleep_ptr(&mt6358_battery_pm_ops),
	},
	.probe = mt6358_battery_probe,
};
module_platform_driver(mt6358_battery_driver);

MODULE_DESCRIPTION("MediaTek MT6358 PMIC battery monitor");
MODULE_LICENSE("GPL");
