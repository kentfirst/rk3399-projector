/* 
 * Copyright (C) 2014 Rockchip Corporation.
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

#include "dlp_monitor1.h"

/* Debug */
#if 0
#define DBG(x...) printk(x)
#else
#define DBG(x...) do { } while (0)
#endif

/*
struct dlp_monitor1_pdata *dlp_monitor1_info;

static void dlp_monitor1_timer_callback(unsigned long arg)
{
//	struct dlp_monitor1_pdata *headset = (struct dlp_monitor1_pdata *)(arg);

//out:
	return;
}

static int dlp_monitor1_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct dlp_monitor1_pdata *pdata;
	int ret;
	enum of_gpio_flags flags;

	pdata = kzalloc(sizeof(struct dlp_monitor1_pdata), GFP_KERNEL);
	if (pdata == NULL) {
		printk("%s failed to allocate driver data\n",__FUNCTION__);
		return -ENOMEM;
	}
	memset(pdata,0,sizeof(struct dlp_monitor1_pdata));

	dlp_monitor1_info = pdata;

	//fandet input
	ret = of_get_named_gpio_flags(node, "fandet_gpio", 0, &flags);
	if (ret < 0) {
		printk("%s() Can not read property fandet_gpio\n", __FUNCTION__);
		goto err;
	} else {
		pdata->fandet_gpio = ret;
		ret = devm_gpio_request(&pdev->dev, pdata->fandet_gpio, "fandet_gpio");
		if(ret < 0){
			printk("%s() devm_gpio_request fandet_gpio request ERROR\n", __FUNCTION__);
			goto err;
		}

		ret = gpio_direction_input(pdata->fandet_gpio); 
		if(ret < 0){
			printk("%s() gpio_direction_input fandet_gpio set ERROR\n", __FUNCTION__);
			goto err;
		}
	}

	//ledtemp adc
	pdata->chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(pdata->chan))
      	{
		pdata->chan = NULL;
		printk("%s() have not set adc chan\n", __FUNCTION__);
	}

	//initial timer
	if(pdata->chan != NULL)
	{
		setup_timer(&pdata->dlp_monitor1_timer, dlp_monitor1_timer_callback, (unsigned long)pdata);
		if(ret < 0)
		{
			goto err;
		}	

	}

	return 0;
err:
	kfree(pdata);
	return ret;
}

static int dlp_monitor1_remove(struct platform_device *pdev)
{
	if(dlp_monitor1_info)
		kfree(dlp_monitor1_info);
	return 0;
}

extern struct dlpmonitor1 *dlpmonitor1_info;
//extern int vdlpmonitor1Suspend(void);//jjj
static int dlp_monitor1_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;
	return 0;
	//host vbus
	dlpmonitor1_info->host_vbus_gpio = devm_gpiod_get_optional(dlpmonitor1_info->dev, "hostvbus", GPIOD_OUT_LOW);
	if (IS_ERR(dlpmonitor1_info->host_vbus_gpio)) {
		ret = PTR_ERR(dlpmonitor1_info->host_vbus_gpio);
		dev_err(dlpmonitor1_info->dev, "failed to request usb host vbus GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dlpmonitor1_info->dev, "usb control host vbus low\n");
	if (dlpmonitor1_info->host_vbus_gpio)
	{
		ret = gpiod_direction_output(dlpmonitor1_info->host_vbus_gpio, 0);
		dev_info(dlpmonitor1_info->dev, "usb control host vbus low %d\n", ret);
	}

	//project on 
	dlpmonitor1_info->project_on_gpio = devm_gpiod_get_optional(dlpmonitor1_info->dev, "projecton", GPIOD_OUT_HIGH);
	if (IS_ERR(dlpmonitor1_info->project_on_gpio)) {
		ret = PTR_ERR(dlpmonitor1_info->project_on_gpio);
		dev_err(dlpmonitor1_info->dev, "failed to request dlp project on GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dlpmonitor1_info->dev, "dlpmonitor1 control project on high\n");
	if (dlpmonitor1_info->project_on_gpio)
	{
		ret = gpiod_direction_output(dlpmonitor1_info->project_on_gpio, 1);
		dev_info(dlpmonitor1_info->dev, "dlpmonitor1 control project on high %d\n", ret);
	}

	//led power
	dlpmonitor1_info->led_power_gpio = devm_gpiod_get_optional(dlpmonitor1_info->dev, "ledpower", GPIOD_OUT_LOW);
	if (IS_ERR(dlpmonitor1_info->led_power_gpio)) {
		ret = PTR_ERR(dlpmonitor1_info->led_power_gpio);
		dev_err(dlpmonitor1_info->dev, "failed to request dlp led power GPIO: %d\n", ret);
		return ret;
	}

	dev_info(dlpmonitor1_info->dev, "dlpmonitor1 control led power low\n");
	if (dlpmonitor1_info->led_power_gpio)
	{
		ret = gpiod_direction_output(dlpmonitor1_info->led_power_gpio, 0);
		dev_info(dlpmonitor1_info->dev, "dlpmonitor1 control led power low %d\n", ret);
	}

	return 0;
}

static int dlp_monitor1_resume(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dlp_monitor1_of_match[] = {
        { .compatible = "dlp_monitor1", },
        {},
};
MODULE_DEVICE_TABLE(of, dlp_monitor1_of_match);

static struct platform_driver dlp_monitor1_driver = {
	.probe	= dlp_monitor1_probe,
	.remove = dlp_monitor1_remove,
	.resume = dlp_monitor1_resume,	
	.suspend = dlp_monitor1_suspend,	
	.driver	= {
		.name	= "dlp_monitor1",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(dlp_monitor1_of_match),		
	},
};

static int __init dlp_monitor1_init(void)
{
	platform_driver_register(&dlp_monitor1_driver);
	return 0;
}

static void __exit dlp_monitor1_exit(void)
{
	platform_driver_unregister(&dlp_monitor1_driver);
}

late_initcall(dlp_monitor1_init);

*/




#define DEST_REG			0x0005
#define IMAGE_ORIENTATION		0x0014
#define DLP3438_MAX_REGISTER		IMAGE_ORIENTATION

static const struct regmap_config dlp3438_regmap_config = {
	.name = "dlpmonitor1",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DLP3438_MAX_REGISTER,
};

struct dlpmonitor1 *dlpmonitor1_info;

#if defined(SUPPORT_SUSPEND)					//20200812 KentYu support suspend & resume processing
static inline int fb_notifier_callback(struct notifier_block *self,
				       unsigned long action, void *data)
{
	struct dlpmonitor1_suspend *ds;
	struct fb_event *event = data;
	int blank_mode;
	int ret = 0;

	ds = container_of(self, struct dlpmonitor1_suspend, fb_notif);
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

static inline int dlpmonitor1_register_fb(struct dlpmonitor1_suspend *ds)
{
        memset(&ds->fb_notif, 0, sizeof(ds->fb_notif));
        ds->fb_notif.notifier_call = fb_notifier_callback;
        mutex_init(&ds->ops_lock);

        return fb_register_client(&ds->fb_notif);
}

static inline void dlpmonitor1_unregister_fb(struct dlpmonitor1_suspend *ds)
{
        fb_unregister_client(&ds->fb_notif);
}
#endif

#if defined(SUPPORT_ATTR)					//20200812 KentYu support setup menu call
static ssize_t projector_temperature_show(struct device *device,
				 struct device_attribute *attr, char *buf)
{
	struct dlpmonitor1 *dlpmonitor1 = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dlpmonitor1->temperature);
}

static ssize_t projector_temperature_store(struct device *device,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
/*
	struct dlpmonitor1 *dlpmonitor1 = dev_get_drvdata(device);
	unsigned char val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	dlpmonitor1->temperature = val;
	dev_info(dlpmonitor1->dev, "set project temperature :%d successfully\n", val);
*/
	return count;
}

static ssize_t projector_fanstatus_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct dlpmonitor1 *dlpmonitor1 = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", dlpmonitor1->fanstatus);
}

static ssize_t projector_fanstatus_store(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
/*
	struct dlpmonitor1 *dlpmonitor1 = dev_get_drvdata(device);
	unsigned char val;
	int ret;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val != 1)
		val = 0;
*/
	return count;
}

static DEVICE_ATTR_RW(projector_temperature);
static DEVICE_ATTR_RW(projector_fanstatus);

static struct attribute *dlpmonitor1_attr_attrs[] = {
	&dev_attr_projector_temperature.attr,
	&dev_attr_projector_fanstatus.attr,
	NULL,
};

static struct attribute_group dlpmonitor1_attr_group = {
	.name = NULL,   /* we want them in the same directory */
	.attrs = dlpmonitor1_attr_attrs,
};
#endif

#if defined(SUPPORT_DELAY)					//20200812 KentYu support delay work
extern int tcn75_read_temp(int *tcn75_temp);//20201026
static void dlpmonitor1_detect_work(struct work_struct *work)
{
	int dlp_temp;
	
	tcn75_read_temp(&dlp_temp);
	dev_info(dlpmonitor1_info->dev, "dlp temp=%d\n", dlp_temp);
	schedule_delayed_work(&dlpmonitor1_info->delay_work, 6 * HZ);
}
#endif

#if defined(SUPPORT_SUSPEND)					//20200812 KentYu support suspend & resume processing
static int dlpmonitor1_early_suspend(struct dlpmonitor1_suspend *ds) {
        struct dlpmonitor1 *dlpmonitor1 = container_of(ds, struct dlpmonitor1, ds);

        dev_info(dlpmonitor1->dev, "dlpmonitor1_early_suspend ---\n");

        //if (dlpmonitor1->dlp_fan_gpio)//20200810 by Kent
        //        gpiod_set_value(dlpmonitor1->dlp_fan_gpio, 0);

        //if (dlpmonitor1->project_on_gpio)
        //        gpiod_set_value(dlpmonitor1->project_on_gpio, 0);

        return 0;
}

static int dlpmonitor1_early_resume(struct dlpmonitor1_suspend *ds) {                                                                            
        struct dlpmonitor1 *dlpmonitor1 = container_of(ds, struct dlpmonitor1, ds);

        dev_info(dlpmonitor1->dev, "dlpmonitor1_early_resume +++\n");

        //if (dlpmonitor1->dlp_fan_gpio)//20200810 by Kent
        //        gpiod_set_value(dlpmonitor1->dlp_fan_gpio, 1);

        //schedule_delayed_work(&dlpmonitor1->delay_work, HZ);

        return 0;
}
#endif

static int dlpmonitor1_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct dlpmonitor1 *dlpmonitor1;
	#if defined(SUPPORT_ATTR)				//20200812 KentYu support setup menu call
	int ret;
	#endif

	dev_info(dlpmonitor1_info->dev, "dlpmonitor1 probe1...\n");//20201024

	dlpmonitor1 = devm_kzalloc(dev, sizeof(*dlpmonitor1), GFP_KERNEL);
	if (!dlpmonitor1)
		return -ENOMEM;

	dlpmonitor1->dev = dev;
	dlpmonitor1->client = client;
	i2c_set_clientdata(client, dlpmonitor1);

	dlpmonitor1_info = dlpmonitor1;

	dev_info(dlpmonitor1_info->dev, "dlpmonitor1 probe2...\n");//20201024

	#if defined(SUPPORT_ATTR)				//20200812 KentYu support setup menu call
	ret = sysfs_create_group(&dev->kobj, &dlpmonitor1_attr_group);
	if (ret)
		dev_err(dev, "failed to create sysfs group: %d\n", ret);
	#endif

	#if defined(SUPPORT_DELAY)				//20200812 KentYu support delay work
	INIT_DELAYED_WORK(&dlpmonitor1->delay_work, dlpmonitor1_detect_work);
	schedule_delayed_work(&dlpmonitor1->delay_work, 6 * HZ);
	#endif

	#if defined(SUPPORT_SUSPEND)				//20200812 KentYu support suspend & resume processing
	dlpmonitor1->ds.resume = dlpmonitor1_early_resume;
	dlpmonitor1->ds.suspend = dlpmonitor1_early_suspend;
	dlpmonitor1_register_fb(&dlpmonitor1->ds);
	#endif

	return 0;
}

static int dlpmonitor1_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dlpmonitor1 *dlpmonitor1 = dev_get_drvdata(dev);

	#if defined(SUPPORT_ATTR)				//20200812 KentYu support setup menu call
	sysfs_remove_group(&dev->kobj, &dlpmonitor1_attr_group);
	#endif

	#if defined(SUPPORT_SUSPEND)				//20200812 KentYu support suspend & resume processing
	dlpmonitor1_unregister_fb(&dlpmonitor1->ds);
	#endif

	return 0;
}

static void dlpmonitor1_i2c_shutdown(struct i2c_client *client)
{
}

#define DLPMONITOR1_PM_OPS NULL

//int (*suspend)(struct i2c_client *, pm_message_t mesg);

static const struct i2c_device_id dlpmonitor1_i2c_table[] = {
	{ "dlpmonitor1", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, dlpmonitor1_i2c_table);

static const struct of_device_id dlpmonitor1_of_match[] = {
	{ .compatible = "ti,dlpmonitor1" },
	{}
};
MODULE_DEVICE_TABLE(of, dlpmonitor1_of_match);

static struct i2c_driver dlpmonitor1_i2c_driver = {
	.driver = {
		.name = "dlpmonitor1",
		.of_match_table = dlpmonitor1_of_match,
		.pm = DLPMONITOR1_PM_OPS,
	},
	.probe = dlpmonitor1_i2c_probe,
	.remove = dlpmonitor1_i2c_remove,
	.shutdown = dlpmonitor1_i2c_shutdown,
	.id_table = dlpmonitor1_i2c_table,
};
module_i2c_driver(dlpmonitor1_i2c_driver);

MODULE_AUTHOR("Kent Yu <kent.yu@arrowasia.com>");
MODULE_DESCRIPTION("DLP monitor1 Driver");
MODULE_LICENSE("GPL v1");
