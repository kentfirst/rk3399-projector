/*
 * A driver for the Integrated Circuits ICS932S401
 * Copyright (C) 2008 IBM
 *
 * Author: Darrick J. Wong <darrick.wong@oracle.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/delay.h>
//#include <linux/clk.h>
#include <linux/gpio/consumer.h>
//#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/io.h>//KentYu
#include <linux/of.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/notifier.h>

#include "dlp3438.h"

#define INPUT_SOURCE			0x0005
#define IMAGE_ORIENTATION		0x0014
#define DLP3438_MAX_REGISTER		IMAGE_ORIENTATION

#define MAX_BRIGHTNESS_LEVEL 256
#define FAN_OFF_FACTOR 6 / 10 //20200726 by Kent

static const struct regmap_config dlp3438_regmap_config = {
	.name = "dlp3438",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DLP3438_MAX_REGISTER,
};

struct dlp3438 *dlp3438_info;

#if 1//20201027 KentYu for reporting temp per 5 second
#define TCN75_TIME	5000
#define TCN75_MAX	50000
static struct delayed_work my_queue_work;
static struct workqueue_struct *my_workqueue = NULL;
#endif

#if 1//20201030 KentYu for solving power on flash issue
static bool bResume=true;
#endif

/*
void vI2cDeinitDlp3438(void)
{
	dev_err(dlp3438_info->dev, "dlp3438 write0 = %d\n", regmap_write(dlp3438_info->regmap, IMAGE_ORIENTATION, 0x04));
	dev_info(dlp3438_info->dev, "dlp3438 i2c deinit\n");
}
*/

/*
int vDlp3438Suspend(void)
{
	int ret=0;

	dev_info(dlp3438_info->dev, "dlp3438 suspend\n");
//	dev_err(dlp3438_info->dev, "dlp3438 write0 = %d\n", regmap_write(dlp3438_info->regmap, IMAGE_ORIENTATION, 0x04));
//	dev_info(dlp3438_info->dev, "dlp3438 i2c deinit\n");

	dlp3438_info->project_on_gpio = devm_gpiod_get_optional(dlp3438_info->dev, "project-on", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438_info->project_on_gpio)) {
		ret = PTR_ERR(dlp3438_info->project_on_gpio);
		dev_err(dlp3438_info->dev, "failed to request dlp project on GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dlp3438_info->dev, "dlp3438 control project on high\n");
	if (dlp3438_info->project_on_gpio)
	{
		ret = gpiod_direction_output(dlp3438_info->project_on_gpio, 1);
		dev_info(dlp3438_info->dev, "dlp3438 control project on high %d\n", ret);
	}

	dlp3438_info->led_power_gpio = devm_gpiod_get_optional(dlp3438_info->dev, "led-power", GPIOD_OUT_LOW);
	if (IS_ERR(dlp3438_info->led_power_gpio)) {
		ret = PTR_ERR(dlp3438_info->led_power_gpio);
		dev_err(dlp3438_info->dev, "failed to request dlp led power GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dlp3438_info->dev, "dlp3438 control led power low\n");
	if (dlp3438_info->led_power_gpio)
	{
		ret = gpiod_direction_output(dlp3438_info->led_power_gpio, 0);
		dev_info(dlp3438_info->dev, "dlp3438 control led power low %d\n", ret);
	}
	msleep(10);

	//host vbus
	dlp3438_info->host_vbus_gpio = devm_gpiod_get_optional(dlp3438_info->dev, "host-vbus", GPIOD_OUT_LOW);
	if (IS_ERR(dlp3438_info->host_vbus_gpio)) {
		ret = PTR_ERR(dlp3438_info->host_vbus_gpio);
		dev_err(dlp3438_info->dev, "failed to request host vbus GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dlp3438_info->dev, "dlp3438 control host vbus low\n");
	if (dlp3438_info->host_vbus_gpio)
	{
		ret = gpiod_direction_output(dlp3438_info->host_vbus_gpio, 0);
		dev_info(dlp3438_info->dev, "dlp3438 control host vbus low %d\n", ret);
	}

	return ret;
}
*/

static inline int fb_notifier_callback(struct notifier_block *self,
				       unsigned long action, void *data)
{
	struct dlp3438_suspend *ds;
	struct fb_event *event = data;
	int blank_mode;
	int ret = 0;

	ds = container_of(self, struct dlp3438_suspend, fb_notif);
	mutex_lock(&ds->ops_lock);

	switch (action) {
	case FB_EARLY_EVENT_BLANK:
		blank_mode = *((int *)event->data);
		if (blank_mode != FB_BLANK_UNBLANK)
			ret = ds->suspend(ds);
		break;

	case FB_EVENT_BLANK:
		blank_mode = *((int *)event->data);
		if (blank_mode == FB_BLANK_UNBLANK)
			ds->resume(ds);
		break;

	default:
		break;
	}
	mutex_unlock(&ds->ops_lock);

	if (ret < 0)
		return ret;
	return NOTIFY_OK;
}

static inline int dlp3438_register_fb(struct dlp3438_suspend *ds)
{
        memset(&ds->fb_notif, 0, sizeof(ds->fb_notif));
        ds->fb_notif.notifier_call = fb_notifier_callback;
        mutex_init(&ds->ops_lock);

        return fb_register_client(&ds->fb_notif);
}

static inline void dlp3438_unregister_fb(struct dlp3438_suspend *ds)
{
        fb_unregister_client(&ds->fb_notif);
}


static int project_set_mode(struct dlp3438 *dlp3438)
{
	int ret;

	switch (dlp3438->mode) {
	case 1:
		ret = regmap_write(dlp3438->regmap, IMAGE_ORIENTATION, 0x00);
		break;

	case 2:
		ret = regmap_write(dlp3438->regmap, IMAGE_ORIENTATION, 0x02);
		break;

	case 3:
		ret = regmap_write(dlp3438->regmap, IMAGE_ORIENTATION, 0x04);
		break;

	case 4:
		ret = regmap_write(dlp3438->regmap, IMAGE_ORIENTATION, 0x06);
		break;

	default:
		break;
	}

	return ret;
}

static unsigned char dlp3438_external_read(struct dlp3438 *dlp3438, unsigned char reg)
{
	unsigned char aAddr = 0xeb;
	unsigned char rAddr = 0xed;
	unsigned int val = 0;
	unsigned char rData[]={0x0, 0x00, 0x00, 0x01, 0x01};

	rData[0] = reg;
	regmap_raw_write(dlp3438->regmap, aAddr, rData, sizeof(rData));
	regmap_read(dlp3438->regmap, rAddr, &val);

	return (unsigned char)val;
}

static void dlp3438_external_write(struct dlp3438 *dlp3438, unsigned char reg,
				   unsigned char val)
{
	unsigned char aAddr = 0xeb;
	unsigned char wAddr = 0xec;
	unsigned char wData[]={0x0, 0x00, 0x00, 0x01, 0x00};

	wData[0] = reg;
	regmap_raw_write(dlp3438->regmap, aAddr, wData, sizeof(wData));
	regmap_write(dlp3438->regmap, wAddr, (int)val);
}

static unsigned int dlp3438_read_led(struct dlp3438 *dlp3438, int ch)
{
	unsigned int val_l;
	unsigned int val_h;
	unsigned int value;
	unsigned char Data[6];

	switch (ch) {
	case 1:
		val_h = dlp3438_external_read(dlp3438, 0x03);
		val_l = dlp3438_external_read(dlp3438, 0x04);
		break;

	case 2:
		val_h = dlp3438_external_read(dlp3438, 0x05);
		val_l = dlp3438_external_read(dlp3438, 0x06);
		break;

	case 3:
		val_h = dlp3438_external_read(dlp3438, 0x07);
		val_l = dlp3438_external_read(dlp3438, 0x08);
		break;

	default:
		/*Read RGB LED Current (55h) */
		regmap_raw_read(dlp3438->regmap, 0x55, Data, 2);
		val_l = Data[0];
		val_h = Data[1];
		break;
	}
	value = ((val_h << 8) & 0x300) | (val_l & 0xff);

	return value;
}

static void dlp3438_write_led(struct dlp3438 *dlp3438, int ch, unsigned int val)
{
	unsigned char Data[6];
	unsigned int temp;

	switch (ch) {
	/* channel-1 */
	case 1:
		dlp3438_external_write(dlp3438, 0x03, (val >> 8) & 0x3);
		dlp3438_external_write(dlp3438, 0x04, val & 0xff);
		break;

	/* channel-2 */
	case 2:
		dlp3438_external_write(dlp3438, 0x05, (val >> 8) & 0x3);
		dlp3438_external_write(dlp3438, 0x06, val & 0xff);
		break;

	/* channel-3 */
	case 3:
		dlp3438_external_write(dlp3438, 0x07, (val >> 8) & 0x3);
		dlp3438_external_write(dlp3438, 0x08, val & 0xff);
		break;

	/* Write RGB LED Current (54h) */
	default:
		//RGB remap from (1023, 1023, 1023) to (530, 850, 680) max
		temp = val*530/1023;
		Data[0] = temp & 0xff;
		Data[1] = (temp >> 8) & 0x3;
		temp = val*850/1023;
		Data[2] = temp & 0xff;
		Data[3] = (temp >> 8) & 0x3;
		temp = val*680/1023;
		Data[4] = temp & 0xff;
		Data[5] = (temp >> 8) & 0x3;
		/*
		Data[0] = val & 0xff;
		Data[1] = (val >> 8) & 0x3;
		Data[2] = val & 0xff;
		Data[3] = (val >> 8) & 0x3;
		Data[4] = val & 0xff;
		Data[5] = (val >> 8) & 0x3;
		*/
		regmap_raw_write(dlp3438->regmap, 0x54, Data, 6);
		break;
	}
}

static ssize_t project_mode_show(struct device *device,
				 struct device_attribute *attr, char *buf)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(device);

	if ((dlp3438->mode > 4) || (dlp3438->mode < 1))
		dlp3438->mode = 0;

	return sprintf(buf, "%d\n", dlp3438->mode);
}

static ssize_t project_mode_store(struct device *device,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(device);
	unsigned char val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val > 4 || val < 1)
		val = 0;

	dlp3438->mode = val;
	project_set_mode(dlp3438);
	dev_info(dlp3438->dev, "set project mode :%d successfully\n", val);

	return count;
}

static ssize_t project_fan_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(device);
	bool gpio_value = false;

	if (dlp3438->dlp_fanv_gpio)
		gpio_value = gpiod_get_value(dlp3438->dlp_fanv_gpio);

	return sprintf(buf, "%d\n", gpio_value);
}

static ssize_t project_fan_store(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(device);
	unsigned char val;
	int ret;
	unsigned int max_level = dlp3438->max_level;
	unsigned int brightness_bar_val;
	static bool bFirstMachineOn=true;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val != 1)
		val = 0;

	if (dlp3438->dlp_fanv_gpio)
		gpiod_set_value(dlp3438->dlp_fanv_gpio, val);
	dev_info(dlp3438->dev, "set project fan: %d successfully\n", val);

	if(bFirstMachineOn)//for machine on first time
	{
		bFirstMachineOn = false;
		dlp3438->fan_mode = val;
		return count;
	}

	if (dlp3438->dlp_fanv_gpio && dlp3438->fan_mode != val)
	{
		dlp3438->fan_mode = val;

		if (val)//fan off to on
		{
			max_level = dlp3438->max_level * FAN_OFF_FACTOR;
			brightness_bar_val = (dlp3438->brightness + 1) * MAX_BRIGHTNESS_LEVEL / max_level;
			max_level = dlp3438->max_level;
		}
		else//fan on to off
		{
			brightness_bar_val = (dlp3438->brightness + 1) * MAX_BRIGHTNESS_LEVEL / max_level;
			max_level = dlp3438->max_level * FAN_OFF_FACTOR;
		}
		//dlp3438->brightness = (val * 1024) / MAX_BRIGHTNESS_LEVEL - 1;
		dlp3438->brightness = (brightness_bar_val * max_level) / MAX_BRIGHTNESS_LEVEL + 1;//jjj - 1;
		dlp3438_write_led(dlp3438, 0, dlp3438->brightness);
		dev_info(dlp3438->dev, "set project brightness after fan on/off:%d, %d successfully\n",
			 val, dlp3438->brightness);
	}

	return count;
}

static ssize_t project_brightness_show(struct device *device,
				       struct device_attribute *attr,
				       char *buf)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(device);
	unsigned int val;
	unsigned int max_level = dlp3438->max_level;

	if (dlp3438->dlp_fanv_gpio && !gpiod_get_value(dlp3438->dlp_fanv_gpio))
		max_level = dlp3438->max_level * FAN_OFF_FACTOR;//KentYu changed 202020726:7 / 10;

	dlp3438->brightness = dlp3438_read_led(dlp3438, 0) + 1;//1~1024?
	val = dlp3438->brightness * MAX_BRIGHTNESS_LEVEL / max_level;
	if (val > MAX_BRIGHTNESS_LEVEL)
		val = MAX_BRIGHTNESS_LEVEL;

	return sprintf(buf, "%d\n", val);
}

static ssize_t project_brightness_store(struct device *device,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(device);
	unsigned int val;
	int ret;
	unsigned int max_level = dlp3438->max_level;

	if (dlp3438->dlp_fanv_gpio && !gpiod_get_value(dlp3438->dlp_fanv_gpio))
		max_level = dlp3438->max_level * FAN_OFF_FACTOR;//KentYu changed 202020726:7 / 10;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	if (val > MAX_BRIGHTNESS_LEVEL)
		val = MAX_BRIGHTNESS_LEVEL;

	//dlp3438->brightness = (val * 1024) / MAX_BRIGHTNESS_LEVEL - 1;
	dlp3438->brightness = (val * max_level) / MAX_BRIGHTNESS_LEVEL - 1;
	dlp3438_write_led(dlp3438, 0, dlp3438->brightness);
	dev_info(dlp3438->dev, "set project brightness :%d, %d successfully\n",
		 val, dlp3438->brightness);

        return count;
}

static DEVICE_ATTR_RW(project_mode);
static DEVICE_ATTR_RW(project_fan);
static DEVICE_ATTR_RW(project_brightness);

static struct attribute *dlp3438_attr_attrs[] = {
	&dev_attr_project_mode.attr,
	&dev_attr_project_fan.attr,
	&dev_attr_project_brightness.attr,
	NULL,
};

static struct attribute_group dlp3438_attr_group = {
	.name = NULL,   /* we want them in the same directory */
	.attrs = dlp3438_attr_attrs,
};


static void dlp3438_detect_work(struct work_struct *work)
{
	struct dlp3438 *dlp3438 = container_of(work, struct dlp3438, delay_work.work);
	struct device *dev = dlp3438->dev;
	int ret;

	//led power
	//projector led on will reset projector settings!!! so need to reinit.
	if (dlp3438->led_power_gpio)
		gpiod_set_value(dlp3438->led_power_gpio, 1); 

	//project on
	//projector on doesn't influece projector status!!! so can be put up!!!
	if(bResume)//20201030 KentYu for solving power on flash issue
	{
		bResume = false;
		dev_err(dev, "dlp3438 project on\n");
		if (dlp3438->project_on_gpio)
			gpiod_set_value(dlp3438->project_on_gpio, 1);
		msleep(170);
	}

	//20201030 KentYu this delay must be set for I2C work correctly!!!!!
	//500->flash, 200->invert, 310~330->ok
	msleep(350);

	if (!dlp3438->max_level) {
		dlp3438->max_level = 1023;//jjj1024;  // max brightness level
		//dlp3438->max_level = dlp3438_read_led(dlp3438, 0) + 1; //default value is 532
		//dlp3438->max_level = 800;
	}
	dev_err(dev, "dlp3438 max brightness level = %d\n", dlp3438->max_level);

	ret = regmap_write(dlp3438->regmap, INPUT_SOURCE, 0x00);//20200531: select inputsource-external video
	dev_err(dev, "dlp3438 write input source = %d\n", ret);
	project_set_mode(dlp3438);

}

#if 1//20201027 KentYu for reporting temp per 5 second
extern int tcn75_read_temp(int *tcn75_temp);
void work_func(struct work_struct *work)
{
	int dlp_temp;
	
	tcn75_read_temp(&dlp_temp);
	dev_info(dlp3438_info->dev, "dlp temp=%d\n", dlp_temp);
	if(dlp_temp > TCN75_MAX)
	{
	        if (dlp3438_info->project_on_gpio)
	                gpiod_set_value(dlp3438_info->project_on_gpio, 0);//close projector
	}

	queue_delayed_work(my_workqueue, &my_queue_work, TCN75_TIME);
}
#endif

static int dlp3438_early_suspend(struct dlp3438_suspend *ds) {
        struct dlp3438 *dlp3438 = container_of(ds, struct dlp3438, ds);

        dev_info(dlp3438->dev, "dlp3438_early_suspend ---\n");

        if (dlp3438->dlp_fan_gpio)//20200810 by Kent
                gpiod_set_value(dlp3438->dlp_fan_gpio, 0);

	#if 1//20201030 KentYu for solving power on flash issue
	if (dlp3438->led_power_gpio)
		gpiod_set_value(dlp3438->led_power_gpio, 0);
	#endif

	#if 0//20201030 KentYu for solving power on flash issue
        if (dlp3438->project_on_gpio)
                gpiod_set_value(dlp3438->project_on_gpio, 0);
	#endif
 
        return 0;
}

static int dlp3438_early_resume(struct dlp3438_suspend *ds) {                                                                            
        struct dlp3438 *dlp3438 = container_of(ds, struct dlp3438, ds);

        dev_info(dlp3438->dev, "dlp3438_early_resume +++\n");
        if (dlp3438->dlp_fan_gpio)//20200810 by Kent
                gpiod_set_value(dlp3438->dlp_fan_gpio, 1);

	//if (dlp3438->project_on_gpio)
	//	gpiod_set_value(dlp3438->project_on_gpio, 1);

        schedule_delayed_work(&dlp3438->delay_work, HZ);

        return 0;
}

static int dlp3438_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct dlp3438 *dlp3438;
	int ret;

	dlp3438 = devm_kzalloc(dev, sizeof(*dlp3438), GFP_KERNEL);
	if (!dlp3438)
		return -ENOMEM;

	dlp3438->dev = dev;
	dlp3438->client = client;
	i2c_set_clientdata(client, dlp3438);

	dlp3438_info = dlp3438;

	ret = sysfs_create_group(&dev->kobj, &dlp3438_attr_group);
	if (ret)
		dev_err(dev, "failed to create sysfs group: %d\n", ret);


//dlp_v1
/*
	dlp3438->dlp_fan_gpio = devm_gpiod_get_optional(dev, "dlp-fan", GPIOD_OUT_LOW);
	if (IS_ERR(dlp3438->dlp_fan_gpio)) {
		ret = PTR_ERR(dlp3438->dlp_fan_gpio);
		dev_err(dev, "failed to request dlp fan GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control dlp fan low\n");
	if (dlp3438->dlp_fan_gpio)
	{
		ret = gpiod_direction_output(dlp3438->dlp_fan_gpio, 0);
		dev_info(dev, "dlp3438 control dlp fan low %d\n", ret);
	}

	dlp3438->led_power_gpio = devm_gpiod_get_optional(dev, "led-power", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->led_power_gpio)) {
		ret = PTR_ERR(dlp3438->led_power_gpio);
		dev_err(dev, "failed to request dlp led power GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control led power high\n");
	if (dlp3438->led_power_gpio)
	{
		ret = gpiod_direction_output(dlp3438->led_power_gpio, 1);
		dev_info(dev, "dlp3438 control led power high %d\n", ret);
	}

	dlp3438->project_on_gpio = devm_gpiod_get_optional(dev, "project-on", GPIOD_OUT_LOW);
	if (IS_ERR(dlp3438->project_on_gpio)) {
		ret = PTR_ERR(dlp3438->project_on_gpio);
		dev_err(dev, "failed to re//20191118 KentYu added for supporting 4G modulequest dlp project on GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control project on low\n");
	if (dlp3438->project_on_gpio)
	{
		ret = gpiod_direction_output(dlp3438->project_on_gpio, 0);
		dev_info(dev, "dlp3438 control project on %d\n", ret);
	}
*/

//dlp_v2				//KentYu added 20200516
	//host vbus
	dlp3438->host_vbus_gpio = devm_gpiod_get_optional(dev, "host-vbus", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->host_vbus_gpio)) {
		ret = PTR_ERR(dlp3438->host_vbus_gpio);
		dev_err(dev, "failed to request host vbus GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control host vbus high\n");
	if (dlp3438->host_vbus_gpio)
	{
		ret = gpiod_direction_output(dlp3438->host_vbus_gpio, 1);
		dev_info(dev, "dlp3438 control host vbus high %d\n", ret);
	}

	//dlp fanv
	dlp3438->dlp_fanv_gpio = devm_gpiod_get_optional(dev, "dlp-fanv", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->dlp_fanv_gpio)) {
		ret = PTR_ERR(dlp3438->dlp_fanv_gpio);
		dev_err(dev, "failed to request dlp fanv GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control dlp fanv low\n");
	if (dlp3438->dlp_fanv_gpio)
	{
		ret = gpiod_direction_output(dlp3438->dlp_fanv_gpio, 1);
		dev_info(dev, "dlp3438 control dlp fanv low %d\n", ret);
	}

	//dlp fan
	dlp3438->dlp_fan_gpio = devm_gpiod_get_optional(dev, "dlp-fan", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->dlp_fan_gpio)) {
		ret = PTR_ERR(dlp3438->dlp_fan_gpio);
		dev_err(dev, "failed to request dlp fan GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control dlp fan high\n");
	if (dlp3438->dlp_fan_gpio)
	{
		ret = gpiod_direction_output(dlp3438->dlp_fan_gpio, 1);
		dev_info(dev, "dlp3438 control dlp fan high %d\n", ret);
	}


	//led power off
	dlp3438->led_power_gpio = devm_gpiod_get_optional(dev, "led-power", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->led_power_gpio)) {
		ret = PTR_ERR(dlp3438->led_power_gpio);
		dev_err(dev, "failed to request dlp led power GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control led power high\n");
	if (dlp3438->led_power_gpio)
	{
		ret = gpiod_direction_output(dlp3438->led_power_gpio, 0);
		dev_info(dev, "dlp3438 control led power off %d\n", ret);
	}

	//project off
	dlp3438->project_on_gpio = devm_gpiod_get_optional(dev, "project-on", GPIOD_OUT_LOW);
	if (IS_ERR(dlp3438->project_on_gpio)) {
		ret = PTR_ERR(dlp3438->project_on_gpio);
		dev_err(dev, "failed to request dlp project on GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "dlp3438 control project on low\n");
	if (dlp3438->project_on_gpio)
	{
		ret = gpiod_direction_output(dlp3438->project_on_gpio, 0);
		dev_info(dev, "dlp3438 control project off %d\n", ret);
	}

//end

	//20191118 KentYu added for supporting 4G module
	#if 0
	//PWRKEY 1 (reversed)
	dlp3438->reg_4g_gpio = devm_gpiod_get_optional(dev, "reg-4g", GPIOD_OUT_LOW);
	if (IS_ERR(dlp3438->reg_4g_gpio)) {
		ret = PTR_ERR(dlp3438->reg_4g_gpio);
		dev_err(dev, "failed to request 4g reg GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "4g reg low\n");
	if (dlp3438->reg_4g_gpio)
	{
		ret = gpiod_direction_output(dlp3438->reg_4g_gpio, 0);
		dev_info(dev, "4g reg low %d\n", ret);

		msleep(100);

	}
	#endif


	dlp3438->regmap = devm_regmap_init_i2c(client, &dlp3438_regmap_config);
	if (IS_ERR(dlp3438->regmap)) {
		ret = PTR_ERR(dlp3438->regmap);
		dev_err(dev, "failed to initialize regmap: %d\n", ret);
		return ret;
	}

	dlp3438->mode = 3;
	dlp3438->max_level = 0;//20200724 by yb
	INIT_DELAYED_WORK(&dlp3438->delay_work, dlp3438_detect_work);
	schedule_delayed_work(&dlp3438->delay_work, 8 * HZ);

	#if 1//20201027 KentYu for reporting temp per 5 second
	my_workqueue= create_workqueue("5s");
	INIT_DELAYED_WORK(&my_queue_work,work_func);
	queue_delayed_work(my_workqueue, &my_queue_work, TCN75_TIME);
	#endif

	{
	phys_addr_t write_addr_bak = 0xff76037c;
	void __iomem *write_addr = ioremap(write_addr_bak, 4);
	dev_info(dev, "clockgate1=%x\n", readl(write_addr));
	writel(0x100000, write_addr);
	dev_info(dev, "clockgate2=%x\n", readl(write_addr));
	}

	//RESET 1(reversed)
	dlp3438->reset_4g_gpio = devm_gpiod_get_optional(dev, "reset-4g", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->reset_4g_gpio)) {
		ret = PTR_ERR(dlp3438->reset_4g_gpio);
		dev_err(dev, "failed to request 4g reset GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "4g reset low\n");
	if (dlp3438->reset_4g_gpio)
	{
		ret = gpiod_direction_output(dlp3438->reset_4g_gpio, 0);
		dev_info(dev, "4g reset off %d\n", ret);
		gpiod_set_value(dlp3438->reset_4g_gpio, 0);
	}

	msleep(100);

	//4G_PWR 1
	dlp3438->power_4g_gpio = devm_gpiod_get_optional(dev, "power-4g", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->power_4g_gpio)) {
		ret = PTR_ERR(dlp3438->power_4g_gpio);
		dev_err(dev, "failed to request 4g power GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "4g power high\n");
	if (dlp3438->power_4g_gpio)
	{
		ret = gpiod_direction_output(dlp3438->power_4g_gpio, 1);
		dev_info(dev, "4g power on %d\n", ret);
		gpiod_set_value(dlp3438->power_4g_gpio, 1);
	}

	msleep(100);

	//PWRKEY 0(reversed)
	dlp3438->reg_4g_gpio = devm_gpiod_get_optional(dev, "reg-4g", GPIOD_OUT_HIGH);
	if (IS_ERR(dlp3438->reg_4g_gpio)) {
		ret = PTR_ERR(dlp3438->reg_4g_gpio);
		dev_err(dev, "failed to request 4g reg GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dev, "4g reg high\n");	//a low pulse
	if (dlp3438->reg_4g_gpio)
	{
		ret = gpiod_direction_output(dlp3438->reg_4g_gpio, 1);
		dev_info(dev, "4g reg high %d\n", ret);

		//ssleep(5);
	}

	dlp3438->ds.resume = dlp3438_early_resume;
	dlp3438->ds.suspend = dlp3438_early_suspend;
	dlp3438_register_fb(&dlp3438->ds);

	return 0;
}

static int dlp3438_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dlp3438 *dlp3438 = dev_get_drvdata(dev);

	sysfs_remove_group(&dev->kobj, &dlp3438_attr_group);
	dlp3438_unregister_fb(&dlp3438->ds);

	#if 0
	//20191118 KentYu added for supporting 4G module
	dev_info(dev, "4g power low\n");
	if (dlp3438->power_4g_gpio)
	{
		ret = gpiod_direction_output(dlp3438->power_4g_gpio, 0);
		dev_info(dev, "4g power off %d\n", ret);
	}
	#endif

//	dev_err(dlp3438->dev, "dlp3438 write0 = %d\n", regmap_write(dlp3438->regmap, IMAGE_ORIENTATION, 0x04));

//	if(dlp3438_info)
//		kfree(dlp3438_info);

	return 0;
}

static void dlp3438_i2c_shutdown(struct i2c_client *client)
{
//	vDlp3438Suspend();
}

//#ifdef CONFIG_PM
#if 0
static int __maybe_unused dlp3438_i2c_suspend(struct device *dev)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(dev);

	dev_info(dlp3438->dev, "dlp3438_i2c_suspend ---\n");

	if (dlp3438->project_on_gpio)
		gpiod_set_value(dlp3438->project_on_gpio, 0);

	return 0;
}

static int __maybe_unused dlp3438_i2c_resume(struct device *dev)
{
	struct dlp3438 *dlp3438 = dev_get_drvdata(dev);

	dev_info(dlp3438->dev, "dlp3438_i2c_resume +++\n");

	schedule_delayed_work(&dlp3438->delay_work, HZ);

	return 0;
}

static const struct dev_pm_ops dlp3438_pm_ops = {
       SET_SYSTEM_SLEEP_PM_OPS(dlp3438_i2c_suspend, dlp3438_i2c_resume)
};

#define DLP3438_PM_OPS (&dlp3438_pm_ops)
#else
#define DLP3438_PM_OPS NULL
#endif

int (*suspend)(struct i2c_client *, pm_message_t mesg);
static const struct i2c_device_id dlp3438_i2c_table[] = {
	{ "dlp3438", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, dlp3438_i2c_table);

static const struct of_device_id dlp3438_of_match[] = {
	{ .compatible = "ti,dlp3438" },
	{}
};
MODULE_DEVICE_TABLE(of, dlp3438_of_match);

static struct i2c_driver dlp3438_i2c_driver = {
	.driver = {
		.name = "dlp3438",
		.of_match_table = dlp3438_of_match,
		.pm = DLP3438_PM_OPS,
	},
	.probe = dlp3438_i2c_probe,
	.remove = dlp3438_i2c_remove,
	.shutdown = dlp3438_i2c_shutdown,
	.id_table = dlp3438_i2c_table,
};
module_i2c_driver(dlp3438_i2c_driver);

MODULE_AUTHOR("Kent Yu <kent.yu@arrowasia.com>");
MODULE_DESCRIPTION("TI dlp3438 chip driver");
MODULE_LICENSE("GPL v1");
