#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/time64.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define CONFIG "/tmp/gpio_trigger"

struct gpio_trigger_data {
	unsigned int		gpio_pin;
	unsigned long       timeout;
	struct hrtimer      timer;
	ktime_t             kt;
	u64  		        ts;
	volatile bool		timer_inited;
};

static struct gpio_trigger_data* pdev_data;

static enum hrtimer_restart timer_callback(struct hrtimer *timer)
{
	/* set the next expire time */
	pdev_data->ts = ktime_get_real_ns();
	pdev_data->ts = pdev_data->ts % pdev_data->timeout;
	if (pdev_data->ts < (pdev_data->timeout / 2)) {
		pdev_data->kt = ktime_set(0, pdev_data->timeout - pdev_data->ts);
	} else {
		pdev_data->kt = ktime_set(0, pdev_data->timeout * 2 - pdev_data->ts);
	}
	hrtimer_forward(timer, timer->base->get_time(), pdev_data->kt);
	gpio_set_value(pdev_data->gpio_pin, 1); /* set the signal */
	udelay(300);
	gpio_set_value(pdev_data->gpio_pin, 0);
	return HRTIMER_RESTART;
}

static int set_mode(struct gpio_trigger_data *pdev_data)
{
	if (!pdev_data->timer_inited) {
		pdev_data->timer_inited = true;
		/* setup timer interval to 1000 msecs */
		pdev_data->ts = ktime_get_real_ns();;
		pdev_data->ts = pdev_data->ts % pdev_data->timeout;
		if (pdev_data->ts < (pdev_data->timeout / 2)) {
			pdev_data->kt = ktime_set(0, pdev_data->timeout - pdev_data->ts);
		} else {
			pdev_data->kt = ktime_set(0, pdev_data->timeout * 2 - pdev_data->ts);
		}
    	hrtimer_init(&pdev_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    	hrtimer_start(&pdev_data->timer, pdev_data->kt, HRTIMER_MODE_REL);
		pdev_data->timer.function = timer_callback;
	}
	return 0;
}

/* module init
*/
static int __init gpio_trigger_init(void)
{
	int err;
	struct file *fp;
	mm_segment_t fs;
	loff_t pos = 0;
	char buf[32] = "";
	unsigned int hz = 0;

	printk("%s\n", __FUNCTION__);

	pdev_data = kzalloc(sizeof(struct gpio_trigger_data), GFP_KERNEL);
	if (!pdev_data) {
		return -ENOMEM;
	}

	fp = filp_open(CONFIG, O_RDONLY, 0644);
	if (IS_ERR(fp)) {
		printk("can not open %s\n", CONFIG);
		return -1;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read(fp, buf, sizeof(buf), &pos);
	filp_close(fp, NULL);
	set_fs(fs);
	sscanf(buf, "%u %u", &pdev_data->gpio_pin, &hz);
	printk("use GPIO: %d, Freq: %d Hz\n", pdev_data->gpio_pin, hz);

	pdev_data->timeout = 1000000000ul / hz;
	/* GPIO setup */
	if (gpio_is_valid(pdev_data->gpio_pin)) {
		err = gpio_request_one(pdev_data->gpio_pin, GPIOF_DIR_OUT|GPIOF_EXPORT, "gpio_pps");
		if (err < 0) {
			printk("%s: unable to request gpio (%d)\n", __func__, err);
			return err;
		}
	} else {
		printk("%s: gpio(%d) is invalid\n", __func__, pdev_data->gpio_pin);
		return -1;
	}

	set_mode(pdev_data);
	return 0;
}


/* module fini
*/
static void __exit gpio_trigger_exit(void)
{
	printk("%s\n", __FUNCTION__);
	if (pdev_data) {
		if (pdev_data->timer_inited) {
			pdev_data->timer_inited = false;
			hrtimer_cancel(&pdev_data->timer);
		}
		gpio_free(pdev_data->gpio_pin);
	}
}


module_init(gpio_trigger_init);
module_exit(gpio_trigger_exit);

MODULE_DESCRIPTION("TSARI GPIO Trigger Driver");
MODULE_AUTHOR("liangwang@tsari.stinghua.edu.cn");
MODULE_LICENSE("GPL");