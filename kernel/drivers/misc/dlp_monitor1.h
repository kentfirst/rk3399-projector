/* SPDX-License-Identifier: GPL-2.0 */
#ifndef DLP_MONITOR1_H
#define DLP_MONITOR1_H

//#define SUPPORT_ATTR						//20200812 KentYu support setup menu call
//#define SUPPORT_DELAY						//20200812 KentYu support delay work
#define SUPPORT_SUSPEND						//20200812 KentYu support suspend & resume processing

#define DLPDET_HIGH 0x00000001
#define DLPDET_LOW  0x00000000


#if defined(SUPPORT_SUSPEND)					//20200812 KentYu support suspend & resume processing
struct  dlpmonitor1_suspend{
	struct notifier_block fb_notif;
	int(*suspend)(struct  dlpmonitor1_suspend*);
	int(*resume)(struct  dlpmonitor1_suspend*);
	struct mutex ops_lock;
};
#endif

struct dlpmonitor1 {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	#if defined(SUPPORT_SUSPEND)				//20200812 KentYu support suspend & resume processing
	struct dlpmonitor1_suspend ds;
	#endif

	struct gpio_desc *dlp_fanv_gpio;

	unsigned char temperature;
	unsigned char fanstatus;
	unsigned int fandet_gpio;

	#if defined(SUPPORT_DELAY)				//20200812 KentYu support delay work
	struct delayed_work delay_work;
	#endif

	struct iio_channel *chan;
	struct timer_list dlp_monitor1_timer;
};

/*
struct dlp_monitor1_pdata{
	//input
	unsigned int fandet_gpio;

	struct iio_channel *chan;
	struct timer_list dlp_monitor1_timer;
};
*/
#endif
