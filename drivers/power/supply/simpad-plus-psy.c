#define DEBUG

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>



#define VALID_MASK_DCIN 0x1

static char *startup_names[2] = { "start-adapter", "start-key" };
static char *bat_ce_names[2] = { "cen", "charge_high" };
typedef enum _startupCode {
	START_REBOOT = 0,
	START_ADAPTER = 1,
	START_KEY = 2,
	START_INVALID = 3
} startupCode;

static char *startup_code_text[4] = {
	"reboot",
	"dcin",
	"key",
	"unknown"
};

struct sbp_priv {
	struct platform_device *pdev;
	struct power_supply *ps_dcin;
	struct gpio_desc* gpio_dcin_valid;
	bool gpio_dcin_valid_active_low;
	struct gpio_desc* gpio_bat_ce[2];
	bool gpio_bat_ce_active_low[2];
	bool bat_ce[2];;
	struct notifier_block ps_dcin_nb;
	struct gpio_desc* gpio_startup_code[2];
	const struct of_device_id *of_id;
};

static startupCode get_startup(struct sbp_priv *priv)
{
	int val = gpiod_get_value(priv->gpio_startup_code[0]);
	if (gpiod_get_value(priv->gpio_startup_code[1]))
		val |= 2;
	return (startupCode)(val & 3);
}

static enum power_supply_property dcin_props_lb[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int lm_pmu_mains_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct sbp_priv *priv = power_supply_get_drvdata(psy);

	switch (psp)
	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = gpiod_get_value(priv->gpio_dcin_valid);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}


static int lm_pmu_notifier_call(struct notifier_block *nb,
		unsigned long val, void *v)
{
	struct power_supply *psy = v;

	if (strncmp(psy->desc->name, "ds2781-battery", 14) == 0) {
		pr_info( "Found %s to %d\n", psy->desc->name, (int)val);

	}
	return 0;
}


static ssize_t lm_pmu_show_startup(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct sbp_priv *priv = dev_get_drvdata(dev->parent);
	startupCode c = get_startup(priv);
	//dev_info(&priv->pdev->dev, "startup code = %d\n", c);
	if (strcmp(attr->attr.name, "startup_code") == 0) {
		return sprintf(buf, "%s\n", startup_code_text[c]);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_show_bat_ce(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct sbp_priv *priv = dev_get_drvdata(dev->parent);

	if (strcmp(attr->attr.name, "bat_charge_en") == 0) {
		return sprintf(buf, "%d\n", priv->bat_ce[0]);
	}
	if (strcmp(attr->attr.name, "bat_high_current") == 0) {
		return sprintf(buf, "%d\n", priv->bat_ce[1]);
	}
	return -EINVAL;
}

static ssize_t lm_pmu_set_bat_ce(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct sbp_priv *priv = dev_get_drvdata(dev->parent);
	bool enable = buf[0] == '1' ? 1 : 0;
	if (strcmp(attr->attr.name, "bat_charge_en") == 0) {
		gpiod_set_value(priv->gpio_bat_ce[0], enable);
		priv->bat_ce[0] = enable;
		return count;
	}
	else if (strcmp(attr->attr.name, "bat_high_current") == 0) {
		gpiod_set_value(priv->gpio_bat_ce[1], enable);
		priv->bat_ce[1] = enable;
		return count;
	}

	return -EINVAL;
}


static DEVICE_ATTR(bat_charge_en, 0644, lm_pmu_show_bat_ce, lm_pmu_set_bat_ce);
static DEVICE_ATTR(bat_high_current, 0644, lm_pmu_show_bat_ce, lm_pmu_set_bat_ce);
static DEVICE_ATTR(startup_code, S_IRUGO, lm_pmu_show_startup, NULL);

static struct attribute *simpad_sysfs_attr[] = {
	&dev_attr_bat_charge_en.attr,
	&dev_attr_bat_high_current.attr,
	&dev_attr_startup_code.attr,
	NULL,
};

static struct attribute *simpad2p_sysfs_attr[] = {
	&dev_attr_bat_charge_en.attr,
	&dev_attr_startup_code.attr,
	NULL,
};

static const struct attribute_group simpad_sysfs_attr_group = {
	.attrs = simpad_sysfs_attr,
};

static const struct attribute_group simpad2p_sysfs_attr_group = {
	.attrs = simpad2p_sysfs_attr,
};

/**********************************************************************
 *  Parse DT
 */

static int lm_pmu_dt(struct sbp_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	int n, num_gpio;

	priv->gpio_dcin_valid = devm_gpiod_get(dev, "dcin", GPIOD_IN);
	if (IS_ERR(priv->gpio_dcin_valid)) {
		dev_err(dev, "%s: Invalid GPIO dcin [%ld]\n", __func__, PTR_ERR(priv->gpio_dcin_valid));
		return PTR_ERR(priv->gpio_dcin_valid);
	}

	/* Charge gpio */
	num_gpio = gpiod_count(dev, "bat-ce");
	if (num_gpio > 2)
		num_gpio = 2;
	for (n = 0; n < num_gpio; n++) {
		priv->gpio_bat_ce[n] = devm_gpiod_get_index(dev, "bat-ce", n, GPIOD_OUT_LOW);
		if (IS_ERR(priv->gpio_bat_ce[n])) {
			dev_err(dev, "Unable to get %s gpio\n", bat_ce_names[n]);
			return -EINVAL;
		}
		priv->bat_ce[n] = true;
	}
	/* Startup codes */
	num_gpio = gpiod_count(dev, "startup");
	if (num_gpio != 2) {
		dev_err(dev, "%s: need 2 startup gpios, found %d\n", __func__, num_gpio);
		return -EINVAL;
	}
	for (n = 0; n < num_gpio; n++) {
		priv->gpio_startup_code[n] = devm_gpiod_get_index(dev, "startup", n, GPIOD_IN);
		if (IS_ERR(priv->gpio_startup_code[n])) {
			dev_err(dev, "%s: Unable to request startup-gpios pin %ld\n", __func__,
				PTR_ERR(priv->gpio_startup_code[n]));
			return -EINVAL;
		}
	}
	return 0;
}

static struct of_device_id simpad_plus_psy_dt_ids[] = {
	{ .compatible = "datarespons,simpad-plus-psy", .data = &simpad_sysfs_attr_group, },
	{ .compatible = "datarespons,simpad2p-psy", .data = &simpad2p_sysfs_attr_group, },
	{}
};
MODULE_DEVICE_TABLE(of, simpad_plus_psy_dt_ids);

static int simpad_plus_psy_probe(struct platform_device *pdev)
{
	struct sbp_priv *priv;
	int ret=0;
	struct power_supply_desc *psy_desc_dcin;
	struct power_supply_config psy_cfg = {};

	psy_desc_dcin = devm_kzalloc(&pdev->dev, sizeof(*psy_desc_dcin), GFP_KERNEL);
	if (!psy_desc_dcin)
		return -ENOMEM;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	psy_cfg.drv_data = priv;
	priv->pdev = pdev;
	priv->of_id = of_match_device(simpad_plus_psy_dt_ids, &pdev->dev);
	if (!priv->of_id) {
		dev_err(&pdev->dev, "No OF data available\n");
		return -EINVAL;
	}
	platform_set_drvdata(pdev, priv);

	ret = lm_pmu_dt(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: Failed to obtain platform data\n", __func__);
		goto cleanup;
	}

	psy_desc_dcin->name = "DCIN";
	psy_desc_dcin->type = POWER_SUPPLY_TYPE_MAINS;
	psy_desc_dcin->get_property = lm_pmu_mains_get_property;
	psy_desc_dcin->num_properties = ARRAY_SIZE(dcin_props_lb);
	psy_desc_dcin->properties = dcin_props_lb;


	priv->ps_dcin = power_supply_register_no_ws(&pdev->dev, psy_desc_dcin, &psy_cfg);
	if (IS_ERR(priv->ps_dcin)) {
		dev_err(&pdev->dev, "Unable to register MAINS PS\n");
	}
	else {
		priv->ps_dcin_nb.notifier_call = lm_pmu_notifier_call;
		power_supply_reg_notifier(&priv->ps_dcin_nb);
		ret = sysfs_create_group(&priv->ps_dcin->dev.kobj, priv->of_id->data);
	}

	return 0;

cleanup:
	return ret;
}

static int lm_pmu_sp_remove(struct platform_device *pdev)
{
	struct sbp_priv *priv = platform_get_drvdata(pdev);
	sysfs_remove_group(&priv->ps_dcin->dev.kobj, priv->of_id->data);
	return 0;
}

static void lm_pmu_sp_shutdown(struct platform_device *pdev)
{
}

static struct platform_driver simpad_plus_psy_driver = {
	.driver = {
		.name = "simpad-plus-psy",
		.owner = THIS_MODULE,
		.of_match_table = simpad_plus_psy_dt_ids,
	},
	.probe = simpad_plus_psy_probe,
	.remove = lm_pmu_sp_remove,
	.shutdown = lm_pmu_sp_shutdown,
};

module_platform_driver(simpad_plus_psy_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Laerdal Plus Simpad Power Supply");
