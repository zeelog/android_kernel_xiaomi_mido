 /*Simple synchronous userspace interface to SPI devices
  *
  * Copyright (C) 2006 SWAPP
  *     Andrea Paterniani <a.paterniani@swapp-eng.it>
  * Copyright (C) 2007 David Brownell (simplification, cleanup)
  * Copyright (C) 2017 XiaoMi, Inc.
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
  * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
  */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define WAKELOCK_HOLD_TIME 2000 /* in ms */
#define GF_SPIDEV_NAME "goodix,fingerprint"
/*device name after register in charater*/
#define GF_DEV_NAME            "goodix_fp"
#define	GF_INPUT_NAME	    "gf3208"	/*"goodix_fp" */

#define	CHRD_DRIVER_NAME	"goodix_fp_spi"
#define	CLASS_NAME		    "goodix_fp"
#define N_SPI_MINORS		32	/* ... up to 256 */


struct gf_key_map key_map[] = {
	  {  "POWER",  KEY_POWER  },
	  {  "HOME" ,  KEY_HOME   },
	  {  "MENU" ,  KEY_MENU   },
	  {  "BACK" ,  KEY_BACK   },
	  {  "UP"   ,  KEY_UP     },
	  {  "DOWN" ,  KEY_DOWN   },
	  {  "LEFT" ,  KEY_LEFT   },
	  {  "RIGHT",  KEY_RIGHT  },
	  {  "FORCE",  KEY_F9     },
	  {  "CLICK",  KEY_F19    },
	  {  "CAMERA", KEY_CAMERA },
};

/*Global variables*/
/*static MODE g_mode = GF_IMAGE_MODE;*/
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf_dev gf;
static struct class *gf_class;
static struct wakeup_source ttw_ws;
static int gf_major = -1;
static int driver_init_partial(struct gf_dev *gf_dev);

static void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		pr_warn("IRQ has been enabled.\n");
	} else {
		enable_irq_wake(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq_wake(gf_dev->irq);
	} else {
		pr_warn("IRQ has been disabled.\n");
	}
}

#ifdef AP_CONTROL_CLK
static long spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int max_steps = 10;

	cur = clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available = clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;
		cur = clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;
		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
			 || ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}

static void spi_clock_set(struct gf_dev *gf_dev, int speed)
{
	long rate;
	int rc;

	rate = spi_clk_max_rate(gf_dev->core_clk, speed);
	if (rate < 0) {
		pr_info("%s: no match found for requested clock frequency:%d",
			__func__, speed);
		return;
	}

	rc = clk_set_rate(gf_dev->core_clk, rate);
}

static int gfspi_ioctl_clk_init(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	data->clk_enabled = 0;
	data->core_clk = clk_get(&data->spi->dev, "core_clk");
	if (IS_ERR_OR_NULL(data->core_clk)) {
		pr_err("%s: fail to get core_clk\n", __func__);
		return -EPERM;
	}
	data->iface_clk = clk_get(&data->spi->dev, "iface_clk");
	if (IS_ERR_OR_NULL(data->iface_clk)) {
		pr_err("%s: fail to get iface_clk\n", __func__);
		clk_put(data->core_clk);
		data->core_clk = NULL;
		return -ENOENT;
	}
	return 0;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *data)
{
	int err;

	pr_debug("%s: enter\n", __func__);

	if (data->clk_enabled)
		return 0;

	err = clk_prepare_enable(data->core_clk);
	if (err) {
		pr_err("%s: fail to enable core_clk\n", __func__);
		return -EPERM;
	}

	err = clk_prepare_enable(data->iface_clk);
	if (err) {
		pr_err("%s: fail to enable iface_clk\n", __func__);
		clk_disable_unprepare(data->core_clk);
		return -ENOENT;
	}

	data->clk_enabled = 1;

	return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	if (!data->clk_enabled)
		return 0;

	clk_disable_unprepare(data->core_clk);
	clk_disable_unprepare(data->iface_clk);
	data->clk_enabled = 0;

	return 0;
}

static int gfspi_ioctl_clk_uninit(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	if (data->clk_enabled)
		gfspi_ioctl_clk_disable(data);

	if (!IS_ERR_OR_NULL(data->core_clk)) {
		clk_put(data->core_clk);
		data->core_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(data->iface_clk)) {
		clk_put(data->iface_clk);
		data->iface_clk = NULL;
	}

	return 0;
}
#endif

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key = { 0 };
	int retval = 0;
		int i;
#ifdef AP_CONTROL_CLK
	unsigned int speed = 0;
#endif
	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval =
			 !access_ok(VERIFY_WRITE, (void __user *)arg,
			 _IOC_SIZE(cmd));
	if ((retval == 0) && (_IOC_DIR(cmd) & _IOC_WRITE))
		retval =
			 !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	if (gf_dev->device_available == 0) {
		if ((cmd == GF_IOC_POWER_ON) || (cmd == GF_IOC_POWER_OFF) || (cmd == GF_IOC_ENABLE_GPIO))
			pr_info("power cmd\n");
		else {
			pr_info("Sensor is power off currently. \n");
			return -ENODEV;
		}
	}

	switch (cmd) {
	case GF_IOC_ENABLE_GPIO:
		driver_init_partial(gf_dev);
		break;
	case GF_IOC_RELEASE_GPIO:
		gf_cleanup(gf_dev);
		break;
	case GF_IOC_DISABLE_IRQ:
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:
		gf_enable_irq(gf_dev);
		break;
	case GF_IOC_SETSPEED:
#ifdef AP_CONTROL_CLK
		retval = __get_user(speed, (u32 __user *) arg);
		if (retval == 0) {
			if (speed > 8 * 1000 * 1000) {
				pr_warn("Set speed:%d is larger than 8Mbps.\n",	speed);
			} else {
				spi_clock_set(gf_dev, speed);
			}
		} else {
			pr_warn("Failed to get speed from user. retval = %d\n",	retval);
		}
#else
		pr_info("This kernel doesn't support control clk in AP\n");
#endif
		break;
	case GF_IOC_RESET:
		gf_hw_reset(gf_dev, 3);
		break;
	case GF_IOC_COOLBOOT:
		gf_power_off(gf_dev);
		usleep_range(5000, 6000);
		gf_power_on(gf_dev);
		break;
	case GF_IOC_SENDKEY:
		if (copy_from_user
			 (&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			pr_warn("Failed to copy data from user space.\n");
			retval = -EFAULT;
			break;
		}

		for (i = 0; i < ARRAY_SIZE(key_map); i++) {
			if (key_map[i].val == gf_key.key) {
				if (KEY_CAMERA == gf_key.key) {
					printk("lihao send camera key!\n");
					input_report_key(gf_dev->input, KEY_SELECT, gf_key.value);
					input_sync(gf_dev->input);
				} else {
					input_report_key(gf_dev->input, gf_key.key, gf_key.value);
					input_sync(gf_dev->input);
				}
				break;
			}
		}

		if (i == ARRAY_SIZE(key_map)) {
			pr_warn("key %d not support yet \n", gf_key.key);
			retval = -EFAULT;
		}

		break;
	case GF_IOC_CLK_READY:
#ifdef AP_CONTROL_CLK
		gfspi_ioctl_clk_enable(gf_dev);
#else
		pr_info("Doesn't support control clock.\n");
#endif
		break;
	case GF_IOC_CLK_UNREADY:
#ifdef AP_CONTROL_CLK
		gfspi_ioctl_clk_disable(gf_dev);
#else
		pr_info("Doesn't support control clock.\n");
#endif
		break;
	case GF_IOC_PM_FBCABCK:
		__put_user(gf_dev->fb_black, (u8 __user *) arg);
		break;
	case GF_IOC_POWER_ON:
		if (gf_dev->device_available == 1)
			pr_info("Sensor has already powered-on.\n");
		else
			gf_power_on(gf_dev);
		gf_dev->device_available = 1;
		break;
	case GF_IOC_POWER_OFF:
		if (gf_dev->device_available == 0)
			pr_info("Sensor has already powered-off.\n");
		else
			gf_power_off(gf_dev);
		gf_dev->device_available = 0;
		break;
	default:
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long
gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{

#if defined(GF_NETLINK_ENABLE)
	struct gf_dev *gf_dev = &gf;
	char temp = GF_NET_EVENT_IRQ;
	__pm_wakeup_event(&ttw_ws, WAKELOCK_HOLD_TIME);
	sendnlmsg(&temp);
#elif defined (GF_FASYNC)
	struct gf_dev *gf_dev = &gf;
	if (gf_dev->async)
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif

	return IRQ_HANDLED;

}

static int driver_init_partial(struct gf_dev *gf_dev)
{
	int ret = 0;
	pr_warn("--------driver_init_partial start.--------\n");

	gf_dev->device_available = 1;

	if (gf_parse_dts(gf_dev))
		goto error;

	gf_dev->irq = gf_irq_num(gf_dev);
	ret = devm_request_threaded_irq(&gf_dev->spi->dev,
			gf_dev->irq,
			NULL,
			gf_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"gf", gf_dev);
	if (ret) {
		pr_err("Could not request irq %d\n", gpio_to_irq(gf_dev->irq_gpio));
		goto error;
	}
	if (!ret) {
		gf_enable_irq(gf_dev);
		gf_disable_irq(gf_dev);
	}
	gf_hw_reset(gf_dev, 360);

	return 0;

error:

	gf_cleanup(gf_dev);

	gf_dev->device_available = 0;

	return -EPERM;


}
static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			/*power the sensor*/
			gf_dev->device_available = 1;
		}
	}
	mutex_unlock(&device_list_lock);
	return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gf_dev = filp->private_data;
	int ret;

	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	gf_dev->users--;
	if (!gf_dev->users) {
		gf_disable_irq(gf_dev);
		devm_free_irq(&gf_dev->spi->dev, gf_dev->irq, gf_dev);

		/*power off the sensor*/
		gf_dev->device_available = 0;
		gf_power_off(gf_dev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
#ifdef GF_FASYNC
	.fasync = gf_fasync,
#endif
};

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct gf_dev *gf_dev = NULL;
	struct fb_event *evdata = NULL;
	unsigned int blank;
#if defined(GF_NETLINK_ENABLE)
		char temp = 0;
#endif

	if (val != FB_EARLY_EVENT_BLANK)
		return 0;

	evdata = data;
	gf_dev = container_of(nb, struct gf_dev, gf_notifier);
	if (evdata && evdata->data && gf_dev) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 1;
#if defined(GF_NETLINK_ENABLE)
				temp = GF_NET_EVENT_FB_BLACK;
				sendnlmsg(&temp);
#elif defined (GF_FASYNC)
				if (gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO,
					POLL_IN);
				}
#endif
			}
			break;
		case FB_BLANK_UNBLANK:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 0;
#if defined(GF_NETLINK_ENABLE)
				temp = GF_NET_EVENT_FB_UNBLACK;
				sendnlmsg(&temp);
#elif defined (GF_FASYNC)
				if (gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO,
						POLL_IN);
				}
#endif
			}
			break;
		default:
			pr_debug("%s defalut\n", __func__);
			break;
		}
	}

	return 0;
}

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
	int i;

	set_bit(EV_KEY, gf_dev->input->evbit);
	for (i = 0; i < ARRAY_SIZE(key_map); i++) {
		set_bit(key_map[i].val, gf_dev->input->keybit);
	}
	set_bit(KEY_SELECT, gf_dev->input->keybit);
	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input))
		pr_warn("Failed to register GF as input device.\n");
}


#if defined(USE_SPI_BUS)
static int gf_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_probe(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;
	int gf_notifier_init = 0;


	pr_warn("--------gf_probe start.--------\n");
	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);

#if defined(USE_SPI_BUS)
	gf_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
	gf_dev->spi = pdev;
#endif

	gf_dev->irq_gpio 		= 	-EINVAL;
	gf_dev->reset_gpio 	= 	-EINVAL;
	gf_dev->pwr_gpio 	= 	-EINVAL;
	gf_dev->device_available =  0;
	gf_dev->fb_black  =  0;
	gf_dev->irq_enabled = 0;
	gf_dev->fingerprint_pinctrl = NULL;

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf_dev->devt = MKDEV(gf_major, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				    gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		/*input device subsystem */
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			dev_dbg(&gf_dev->input->dev,
				"Faile to allocate input device.\n");
			status = -ENOMEM;
			goto error;
		}
#ifdef AP_CONTROL_CLK
		pr_info("Get the clk resource.\n");
		/* Enable spi clock */
		if (gfspi_ioctl_clk_init(gf_dev))
			goto gfspi_probe_clk_init_failed;


		if (gfspi_ioctl_clk_enable(gf_dev))
			goto gfspi_probe_clk_enable_failed;

		spi_clock_set(gf_dev, 4.8*1000*1000);
#endif

#ifdef CONFIG_FB
		gf_dev->gf_notifier.notifier_call = goodix_fb_state_chg_callback;
		gf_notifier_init = fb_register_client(&gf_dev->gf_notifier);
		if (gf_notifier_init)
			pr_err("%s: Fail to register fb notifier\n", __func__);
#endif

		gf_reg_key_kernel(gf_dev);

		wakeup_source_init(&ttw_ws, "ttw_ws");

	}

	pr_warn("--------gf_probe end---OK.--------\n");
	return status;

error:
	gf_cleanup(gf_dev);
	gf_dev->device_available = 0;
	if (gf_dev->devt != 0) {
		pr_info("Err: status = %d\n", status);
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);

#ifdef AP_CONTROL_CLK
gfspi_probe_clk_enable_failed:
		gfspi_ioctl_clk_uninit(gf_dev);
gfspi_probe_clk_init_failed:
#endif
		if (gf_dev->input != NULL)
			input_unregister_device(gf_dev->input);
	}

	return status;
}

#if defined(USE_SPI_BUS)
static int gf_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_remove(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;

	wakeup_source_trash(&ttw_ws);

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq)
		free_irq(gf_dev->irq, gf_dev);

	if (gf_dev->input != NULL) {
		input_unregister_device(gf_dev->input);
		input_free_device(gf_dev->input);
	}

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	if (gf_dev->users == 0)
		kfree(gf_dev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct of_device_id gx_match_table[] = {
	{.compatible = GF_SPIDEV_NAME,},
	{},
};

#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver = {
#endif
	.driver = {
		   .name = GF_DEV_NAME,
		   .owner = THIS_MODULE,
#if defined(USE_SPI_BUS)

#endif
		   .of_match_table = gx_match_table,
		   },
	.probe = gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int status;
	pr_warn("--------gf_init start.--------\n");
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	/* Dynamically allocate a major */
	gf_major = register_chrdev(0, CHRD_DRIVER_NAME, &gf_fops);
	if (gf_major < 0) {
		pr_warn("Failed to register char device!\n");
		return gf_major;
	}
	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(gf_major, gf_driver.driver.name);
		pr_warn("Failed to create class.\n");
		return PTR_ERR(gf_class);
	}
#if defined(USE_PLATFORM_BUS)
	status = platform_driver_register(&gf_driver);
#elif defined(USE_SPI_BUS)
	status = spi_register_driver(&gf_driver);
#endif
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(gf_major, gf_driver.driver.name);
		pr_warn("Failed to register SPI driver.\n");
	}

#ifdef GF_NETLINK_ENABLE
	netlink_init();
#endif
	pr_info(" status = 0x%x\n", status);

	pr_warn("--------gf_init end---OK.--------\n");
	return 0;
}

module_init(gf_init);

static void __exit gf_exit(void)
{
#ifdef CONFIG_FB
	struct gf_dev *gf_dev = &gf;
#endif

#ifdef CONFIG_FB
	if (fb_unregister_client(&gf_dev->gf_notifier))
		pr_err("%s: Fail to unregister fb notifier\n", __func__);
#endif

#ifdef GF_NETLINK_ENABLE
	netlink_exit();
#endif
#if defined(USE_PLATFORM_BUS)
	platform_driver_unregister(&gf_driver);
#elif defined(USE_SPI_BUS)
	spi_unregister_driver(&gf_driver);
#endif
	class_destroy(gf_class);
	if (gf_major >= 0)
		unregister_chrdev(gf_major, gf_driver.driver.name);
}

module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");
