/* SPDX-License-Identifier: GPL-2.0 */
#ifndef DLP3438_H
#define DLP3438_H

struct  dlp3438_suspend{
	struct notifier_block fb_notif;
	int(*suspend)(struct  dlp3438_suspend*);
	int(*resume)(struct  dlp3438_suspend*);
	struct mutex ops_lock;
};

struct dlp3438 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct dlp3438_suspend ds;

	struct gpio_desc *dlp_fanv_gpio;
	struct gpio_desc *dlp_fan_gpio;
	struct gpio_desc *led_power_gpio;
	struct gpio_desc *project_on_gpio;
	struct gpio_desc *host_vbus_gpio;

	//20191118 KentYu added for supporting 4G module
	struct gpio_desc *power_4g_gpio;
	struct gpio_desc *reg_4g_gpio;
	struct gpio_desc *reset_4g_gpio;

	unsigned char mode;
	unsigned int brightness;
	unsigned int max_level;
	unsigned char fan_mode;

	struct delayed_work delay_work;
};
#endif
