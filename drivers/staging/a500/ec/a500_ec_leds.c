/*
 * LEDs driver for Acer A50x tablets
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include "ec.h"

struct ec_led {
	struct led_classdev	cdev;
	struct work_struct	work;
	u8			new_state;
};

EC_REG_DATA(RESET_LEDS,		0x40,	100);
EC_REG_DATA(POWER_LED_ON,	0x42,	100);
EC_REG_DATA(CHARGE_LED_ON,	0x43,	100);
EC_REG_DATA(ANDROID_LEDS_OFF,	0x5A,	100);

static void ec_led_set(struct led_classdev *led_cdev,
		       enum led_brightness value)
{
	struct ec_led *led = container_of(led_cdev, struct ec_led, cdev);

	led->new_state = value;
	schedule_work(&led->work);
}

#define EC_LED(_color, _addr)						\
static struct ec_led ec_##_color##_led = {			\
	.cdev = {							\
		.name			= "power-button-" #_color,	\
		.brightness_set		= ec_led_set,			\
		.max_brightness		= 1,				\
		.flags			= LED_CORE_SUSPENDRESUME,	\
	},								\
};									\
static void ec_##_color##_led_work(struct work_struct *work)		\
{									\
	struct ec_led *led = container_of(work, struct ec_led, work);	\
									\
	if (led->new_state)						\
		ec_write_word_data(_addr, 0);				\
	else								\
		ec_write_word_data(RESET_LEDS, 0);			\
}

EC_LED(white, POWER_LED_ON);
EC_LED(orange, CHARGE_LED_ON);

static int ec_leds_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	INIT_WORK(&ec_white_led.work, ec_white_led_work);
	INIT_WORK(&ec_orange_led.work, ec_orange_led_work);

	ret = led_classdev_register(&pdev->dev, &ec_white_led.cdev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: Failed to register white led\n", __func__);
		return ret;
	}

	ret = led_classdev_register(&pdev->dev, &ec_orange_led.cdev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: Failed to register orange led\n", __func__);
		led_classdev_unregister(&ec_white_led.cdev);
		return ret;
	}

	if (of_property_read_bool(np, "leds-reset")) {
		ec_write_word_data(RESET_LEDS, 0);
		ec_write_word_data(ANDROID_LEDS_OFF, 0);
	}

	return 0;
}

static int ec_leds_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&ec_white_led.cdev);
	cancel_work_sync(&ec_white_led.work);

	led_classdev_unregister(&ec_orange_led.cdev);
	cancel_work_sync(&ec_orange_led.work);

	return 0;
}

static const struct of_device_id ec_leds_match[] = {
	{ .compatible = "acer,a50x-leds" },
	{ }
};
MODULE_DEVICE_TABLE(of, ec_leds_match);

static struct platform_driver ec_leds_driver = {
	.driver = {
		.name = "a50x-leds",
		.of_match_table = ec_leds_match,
		.owner = THIS_MODULE,
	},
	.probe = ec_leds_probe,
	.remove = ec_leds_remove,
};
module_platform_driver(ec_leds_driver);

MODULE_DESCRIPTION("Acer A50x EC LEDs driver");
MODULE_AUTHOR("Dmitry Osipenko <digetx@gmail.com>");
MODULE_LICENSE("GPL");
