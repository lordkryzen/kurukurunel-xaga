// SPDX-License-Identifier: GPL-2.0
/* Native xaga /dev/irtx ioctl and packed-waveform ABI on the MediaTek PWM provider.
 * Retains native DT routing, 26 MHz clock, mode and duty/cycle interpretation.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/dma-mapping.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <mt-plat/mtk_pwm.h>
#include <mt-plat/mtk_pwm_hal.h>

#define IRTX_CLOCK 26000000ULL
#define IRTX_MAX_WORDS 65536U
#define IRTX_MAX_US 2000000U
#define IRTX_SET_CARRIER _IOW('R', 0, unsigned int)
#define IRTX_GET_SOLUTION _IOR('R', 1, unsigned int)
#define IRTX_SET_DUTY _IOW('R', 2, unsigned int)
#define IRTX_SET_LED _IOW('R', 10, unsigned int)
extern bool mtk_pwm_ir_ready(void);

struct irtx_device {
	struct device *dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *idle, *active;
	struct regulator *supply;
	struct miscdevice misc;
	struct mutex lock;
	atomic_t opened;
	u32 channel, invert, carrier, cycle, duty;
	bool powered;
};

static int irtx_period(u32 carrier, u32 cycle, u32 duty, u32 *ticks)
{
	u64 value;
	if (!carrier || !cycle || !duty || duty > cycle)
		return -EINVAL;
	value = DIV_ROUND_CLOSEST_ULL(IRTX_CLOCK * duty, (u64)carrier * cycle);
	if (!value || value > 65536)
		return -ERANGE;
	*ticks = value;
	return 0;
}

static int irtx_frame_size(size_t count, u32 *words)
{
	/* Last u32 is native userspace's duration trailer; it is not DMA data. */
	if (count < 2 * sizeof(u32) || count % sizeof(u32))
		return -EINVAL;
	if (count / sizeof(u32) - 1 > IRTX_MAX_WORDS)
		return -EMSGSIZE;
	*words = count / sizeof(u32) - 1;
	return 0;
}

static int irtx_power(struct irtx_device *ir, bool enable)
{
	int ret;
	if (enable) {
		if (ir->supply && !ir->powered) {
			ret = regulator_enable(ir->supply);
			if (ret) return ret;
			ir->powered = true;
		}
		ret = pinctrl_select_state(ir->pinctrl, ir->active);
		if (ret && ir->powered) {
			if (!regulator_disable(ir->supply)) ir->powered = false;
		}
		return ret;
	}
	ret = pinctrl_select_state(ir->pinctrl, ir->idle);
	if (ir->powered) {
		int err = regulator_disable(ir->supply);
		if (!err) ir->powered = false;
		if (!ret) ret = err;
	}
	return ret;
}

static int irtx_open(struct inode *inode, struct file *file)
{
	struct irtx_device *ir = container_of(file->private_data, struct irtx_device, misc);
	if (atomic_cmpxchg(&ir->opened, 0, 1)) return -EBUSY;
	file->private_data = ir;
	return nonseekable_open(inode, file);
}

static int irtx_release(struct inode *inode, struct file *file)
{
	struct irtx_device *ir = file->private_data;
	mutex_lock(&ir->lock);
	mt_pwm_disable(ir->channel, false);
	irtx_power(ir, false);
	mutex_unlock(&ir->lock);
	atomic_set(&ir->opened, 0);
	return 0;
}

static ssize_t irtx_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct irtx_device *ir = file->private_data;
	struct pwm_spec_config conf = {
		.mode = PWM_MODE_MEMORY, .clk_div = CLK_DIV1,
		.clk_src = PWM_CLK_NEW_MODE_BLOCK, .pmic_pad = false,
		.PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31,
		.PWM_MODE_MEMORY_REGS.WAVE_NUM = 1,
	};
	u32 words, ticks, duration, i;
	dma_addr_t dma;
	u32 *wave;
	int ret, off_ret;
	if (!count) return 0;
	ret = irtx_frame_size(count, &words);
	if (ret) return ret;
	mutex_lock(&ir->lock);
	ret = irtx_period(ir->carrier, ir->cycle, ir->duty, &ticks);
	if (ret) goto unlock;
	wave = dma_alloc_coherent(ir->dev, count, &dma, GFP_KERNEL | GFP_DMA);
	if (!wave) { ret = -ENOMEM; goto unlock; }
	if (copy_from_user(wave, buf, count)) { ret = -EFAULT; goto free; }
	duration = wave[words];
	if (!duration) {
		/* Native trailer is normally populated; derive a nonzero bounded wait. */
		u64 us = DIV_ROUND_UP_ULL((u64)words * 32 * ticks * 1000000, IRTX_CLOCK);
		if (!us || us > IRTX_MAX_US) { ret = -EINVAL; goto free; }
		duration = us;
	}
	if (duration > IRTX_MAX_US) { ret = -EINVAL; goto free; }
	if (ir->invert) for (i = 0; i < words; i++) wave[i] = ~wave[i];
	conf.pwm_no = ir->channel;
	conf.PWM_MODE_MEMORY_REGS.HDURATION = ticks - 1;
	conf.PWM_MODE_MEMORY_REGS.LDURATION = ticks - 1;
	conf.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = dma;
	conf.PWM_MODE_MEMORY_REGS.BUF0_SIZE = words - 1;
	ret = irtx_power(ir, true);
	if (ret) { irtx_power(ir, false); goto free; }
	ret = pwm_set_spec_config(&conf);
	if (!ret) {
		usleep_range(duration, duration + 100);
		ret = count;
	}
	/* PWM must stop before the coherent buffer can be freed, even on error. */
	mt_pwm_disable(ir->channel, false);
	off_ret = irtx_power(ir, false);
	if (ret >= 0 && off_ret) ret = off_ret;
	dev_info(ir->dev, "IR frame: carrier=%u words=%u time_us=%u ret=%d\n",
		 ir->carrier, words, duration, ret);
free:
	dma_free_coherent(ir->dev, count, wave, dma);
unlock:
	mutex_unlock(&ir->lock);
	return ret;
}

static long irtx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct irtx_device *ir = file->private_data;
	u32 val, ticks, cycle, duty;
	int ret = 0;
	if (cmd == IRTX_GET_SOLUTION) return put_user(1U, (u32 __user *)arg);
	if (cmd != IRTX_SET_CARRIER && cmd != IRTX_SET_DUTY && cmd != IRTX_SET_LED)
		return -ENOTTY;
	if (copy_from_user(&val, (void __user *)arg, sizeof(val))) return -EFAULT;
	mutex_lock(&ir->lock);
	switch (cmd) {
	case IRTX_SET_CARRIER:
		ret = irtx_period(val, ir->cycle, ir->duty, &ticks);
		if (!ret) ir->carrier = val;
		break;
	case IRTX_SET_DUTY:
		cycle = val & 0xffff; duty = val >> 16;
		ret = irtx_period(ir->carrier, cycle, duty, &ticks);
		if (!ret) { ir->cycle = cycle; ir->duty = duty; }
		break;
	case IRTX_SET_LED:
		ret = irtx_power(ir, !!(val & 0xf));
		break;
	}
	mutex_unlock(&ir->lock);
	return ret;
}

#ifdef CONFIG_COMPAT
static long irtx_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return irtx_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif
static const struct file_operations irtx_fops = {
	.owner = THIS_MODULE, .open = irtx_open, .release = irtx_release,
	.write = irtx_write, .unlocked_ioctl = irtx_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = irtx_compat_ioctl,
#endif
};

static int irtx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct irtx_device *ir;
	const char *supply_name;
	int ret;
	if (!mtk_pwm_ir_ready()) return -EPROBE_DEFER;
	ir = devm_kzalloc(dev, sizeof(*ir), GFP_KERNEL);
	if (!ir) return -ENOMEM;
	ir->dev = dev; ir->carrier = 38009; ir->cycle = 3; ir->duty = 1;
	mutex_init(&ir->lock); atomic_set(&ir->opened, 0);
	ret = of_property_read_u32(dev->of_node, "pwm_ch", &ir->channel);
	if (ret) return ret;
	/* Current native PWM block exposes six channels, numbered0..5. */
	if (ir->channel >= 6) return -EINVAL;
	of_property_read_u32(dev->of_node, "pwm_data_invert", &ir->invert);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) return ret;
	ir->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ir->pinctrl)) return PTR_ERR(ir->pinctrl);
	ir->idle = pinctrl_lookup_state(ir->pinctrl, "irtx_gpio_led_default");
	if (IS_ERR(ir->idle)) return PTR_ERR(ir->idle);
	ir->active = pinctrl_lookup_state(ir->pinctrl, "irtx_gpio_led_set");
	if (IS_ERR(ir->active)) return PTR_ERR(ir->active);
	/* MediaTek's native DT uses a supply-name string, not a phandle. */
	ret = of_property_read_string(dev->of_node, "pwm-supply", &supply_name);
	if (!ret) {
		ir->supply = devm_regulator_get(dev, supply_name);
		if (IS_ERR(ir->supply)) return PTR_ERR(ir->supply);
		ret = regulator_set_voltage(ir->supply, 2800000, 2800000);
		if (ret) return ret;
	} else if (of_find_property(dev->of_node, "pwm-supply", NULL)) {
		return ret;
	}
	ret = irtx_power(ir, false);
	if (ret) return ret;
	ir->misc.minor = MISC_DYNAMIC_MINOR;
	ir->misc.name = "irtx"; ir->misc.fops = &irtx_fops; ir->misc.parent = dev;
	platform_set_drvdata(pdev, ir);
	ret = misc_register(&ir->misc);
	if (ret) return ret;
	dev_info(dev, "Native IR ready: PWM%u invert=%u DMA32\n", ir->channel, ir->invert);
	return 0;
}

static void irtx_remove(struct platform_device *pdev)
{
	struct irtx_device *ir = platform_get_drvdata(pdev);
	misc_deregister(&ir->misc);
	mt_pwm_disable(ir->channel, false);
	irtx_power(ir, false);
}
static const struct of_device_id irtx_match[] = {
	{ .compatible = "mediatek,irtx-pwm" }, {},
};
MODULE_DEVICE_TABLE(of, irtx_match);
static struct platform_driver irtx_driver = {
	.probe = irtx_probe, .remove = irtx_remove,
	.driver = { .name = "mt_irtx", .of_match_table = irtx_match,
		.suppress_bind_attrs = true },
};
module_platform_driver(irtx_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Native MediaTek PWM consumer IR transmitter");
