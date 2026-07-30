// SPDX-License-Identifier: GPL-2.0-only
/*
 * Userspace interface for the Rockchip PWM v4 wave-table generator
 */

#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pwm-rockchip.h>
#include <linux/pwm-wave.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define PWM_WAVE_TABLE_SIZE	0x300
#define PWM_WAVE_MAX_CLOCK_HZ	1000000000ULL

struct rockchip_pwm_wave {
	struct device *dev;
	struct miscdevice miscdev;
	struct pwm_device *pwm;
	struct pwm_state pwm_state;
	struct pwm_wave_config config;
	/* Serialize configuration changes and complete-table submissions. */
	struct mutex lock;
	atomic_t opened;
	u64 table[PWM_WAVE_TABLE_SIZE];
	bool configured;
	bool wave_enabled;
	int id;
};

static DEFINE_IDA(pwm_wave_ida);

static int rockchip_pwm_wave_set_enabled(struct rockchip_pwm_wave *wave,
					 bool enable)
{
	struct rockchip_pwm_wave_config config = {
		.enable = enable,
		.clk_rate = wave->config.clock_rate_hz,
	};
	int ret;

	if (wave->wave_enabled == enable)
		return 0;

	ret = rockchip_pwm_set_wave(wave->pwm, &config);
	if (!ret)
		wave->wave_enabled = enable;

	return ret;
}

static int rockchip_pwm_wave_stop(struct rockchip_pwm_wave *wave)
{
	int ret;

	if (wave->pwm_state.enabled) {
		wave->pwm_state.enabled = false;
		ret = pwm_apply_state(wave->pwm, &wave->pwm_state);
		if (ret) {
			wave->pwm_state.enabled = true;
			return ret;
		}
	}

	return rockchip_pwm_wave_set_enabled(wave, false);
}

static bool rockchip_pwm_wave_valid_config(const struct pwm_wave_config *config)
{
	if (!config->period_ns || config->period_ns > U32_MAX)
		return false;
	if (!config->clock_rate_hz ||
	    config->clock_rate_hz > PWM_WAVE_MAX_CLOCK_HZ)
		return false;
	if (config->repeat > U16_MAX)
		return false;
	if (config->flags & ~PWM_WAVE_FLAG_16BIT)
		return false;

	return !memchr_inv(config->reserved, 0, sizeof(config->reserved));
}

static int rockchip_pwm_wave_validate_table(struct rockchip_pwm_wave *wave,
					    const u32 *table,
					    unsigned int len)
{
	u64 max_ticks = wave->config.flags & PWM_WAVE_FLAG_16BIT ?
			U16_MAX : U8_MAX;
	unsigned int i;

	for (i = 0; i < len; i++) {
		u64 ticks;

		if (table[i] > wave->config.period_ns)
			return -ERANGE;

		ticks = DIV_ROUND_CLOSEST_ULL(wave->config.clock_rate_hz *
					      table[i], NSEC_PER_SEC);
		if ((table[i] && !ticks) || ticks > max_ticks)
			return -ERANGE;
	}

	return 0;
}

static int rockchip_pwm_wave_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct rockchip_pwm_wave *wave =
		container_of(miscdev, struct rockchip_pwm_wave, miscdev);

	if (atomic_cmpxchg(&wave->opened, 0, 1))
		return -EBUSY;

	file->private_data = wave;
	return nonseekable_open(inode, file);
}

static int rockchip_pwm_wave_release(struct inode *inode, struct file *file)
{
	struct rockchip_pwm_wave *wave = file->private_data;

	atomic_set(&wave->opened, 0);
	return 0;
}

static ssize_t rockchip_pwm_wave_write(struct file *file,
				       const char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct rockchip_pwm_wave *wave = file->private_data;
	struct rockchip_pwm_wave_table duty_table;
	struct rockchip_pwm_wave_config wave_config;
	unsigned int max_entries;
	unsigned int entries;
	u32 *table;
	int ret;
	unsigned int i;

	if (!count || count % sizeof(*table))
		return -EINVAL;
	if (count > PWM_WAVE_TABLE_SIZE * sizeof(*table))
		return -EMSGSIZE;

	entries = count / sizeof(*table);
	table = memdup_user(buffer, count);
	if (IS_ERR(table))
		return PTR_ERR(table);

	mutex_lock(&wave->lock);

	if (!wave->configured) {
		ret = -EINVAL;
		goto out_unlock;
	}

	max_entries = wave->config.flags & PWM_WAVE_FLAG_16BIT ?
		      PWM_WAVE_TABLE_SIZE / 2 : PWM_WAVE_TABLE_SIZE;
	if (entries > max_entries) {
		ret = -EMSGSIZE;
		goto out_unlock;
	}

	ret = rockchip_pwm_wave_validate_table(wave, table, entries);
	if (ret)
		goto out_unlock;

	ret = rockchip_pwm_wave_stop(wave);
	if (ret)
		goto out_unlock;

	for (i = 0; i < entries; i++)
		wave->table[i] = table[i];

	duty_table.offset = 0;
	duty_table.len = entries;
	duty_table.table = wave->table;

	memset(&wave_config, 0, sizeof(wave_config));
	wave_config.duty_table = &duty_table;
	wave_config.enable = true;
	wave_config.duty_en = true;
	wave_config.irq_en = false;
	wave_config.clk_rate = wave->config.clock_rate_hz;
	wave_config.rpt = wave->config.repeat;
	wave_config.width_mode =
		wave->config.flags & PWM_WAVE_FLAG_16BIT ?
		PWM_WAVE_TABLE_16BITS_WIDTH : PWM_WAVE_TABLE_8BITS_WIDTH;
	wave_config.update_mode = PWM_WAVE_INCREASING;
	wave_config.duty_max = entries - 1;

	ret = rockchip_pwm_set_wave(wave->pwm, &wave_config);
	if (ret)
		goto out_unlock;
	wave->wave_enabled = true;

	wave->pwm_state.enabled = true;
	ret = pwm_apply_state(wave->pwm, &wave->pwm_state);
	if (ret) {
		wave->pwm_state.enabled = false;
		rockchip_pwm_wave_set_enabled(wave, false);
		goto out_unlock;
	}

	ret = count;

out_unlock:
	mutex_unlock(&wave->lock);
	kfree(table);
	return ret;
}

static long rockchip_pwm_wave_ioctl(struct file *file, unsigned int cmd,
				    unsigned long arg)
{
	struct rockchip_pwm_wave *wave = file->private_data;
	void __user *argp = (void __user *)arg;
	struct pwm_wave_config config;
	struct pwm_wave_caps caps = {
		.abi_version = PWM_WAVE_ABI_VERSION,
		.max_entries_8bit = PWM_WAVE_TABLE_SIZE,
		.max_entries_16bit = PWM_WAVE_TABLE_SIZE / 2,
		.supported_flags = PWM_WAVE_FLAG_16BIT,
	};
	int ret;

	switch (cmd) {
	case PWM_WAVE_IOC_GET_CAPS:
		if (copy_to_user(argp, &caps, sizeof(caps)))
			return -EFAULT;
		return 0;

	case PWM_WAVE_IOC_GET_CONFIG:
		mutex_lock(&wave->lock);
		config = wave->config;
		mutex_unlock(&wave->lock);
		if (copy_to_user(argp, &config, sizeof(config)))
			return -EFAULT;
		return 0;

	case PWM_WAVE_IOC_SET_CONFIG:
		if (copy_from_user(&config, argp, sizeof(config)))
			return -EFAULT;
		if (!rockchip_pwm_wave_valid_config(&config))
			return -EINVAL;

		mutex_lock(&wave->lock);
		ret = rockchip_pwm_wave_stop(wave);
		if (ret)
			goto out_unlock;

		wave->pwm_state.period = config.period_ns;
		wave->pwm_state.duty_cycle = 0;
		ret = pwm_apply_state(wave->pwm, &wave->pwm_state);
		if (!ret) {
			wave->config = config;
			wave->configured = true;
		}
out_unlock:
		mutex_unlock(&wave->lock);
		return ret;

	case PWM_WAVE_IOC_STOP:
		mutex_lock(&wave->lock);
		ret = rockchip_pwm_wave_stop(wave);
		mutex_unlock(&wave->lock);
		return ret;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations rockchip_pwm_wave_fops = {
	.owner = THIS_MODULE,
	.open = rockchip_pwm_wave_open,
	.release = rockchip_pwm_wave_release,
	.write = rockchip_pwm_wave_write,
	.unlocked_ioctl = rockchip_pwm_wave_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.llseek = no_llseek,
};

static int rockchip_pwm_wave_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_pwm_wave *wave;
	int ret;

	wave = devm_kzalloc(dev, sizeof(*wave), GFP_KERNEL);
	if (!wave)
		return -ENOMEM;

	wave->dev = dev;
	mutex_init(&wave->lock);
	atomic_set(&wave->opened, 0);

	wave->pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(wave->pwm))
		return dev_err_probe(dev, PTR_ERR(wave->pwm),
				     "failed to request PWM\n");

	pwm_init_state(wave->pwm, &wave->pwm_state);
	wave->pwm_state.duty_cycle = 0;
	wave->pwm_state.polarity = PWM_POLARITY_NORMAL;
	wave->pwm_state.enabled = false;
	ret = pwm_apply_state(wave->pwm, &wave->pwm_state);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure PWM\n");

	wave->id = ida_alloc(&pwm_wave_ida, GFP_KERNEL);
	if (wave->id < 0)
		return wave->id;

	wave->miscdev.minor = MISC_DYNAMIC_MINOR;
	wave->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "pwm-wave-%d",
					    wave->id);
	if (!wave->miscdev.name) {
		ret = -ENOMEM;
		goto err_free_id;
	}
	wave->miscdev.fops = &rockchip_pwm_wave_fops;
	wave->miscdev.parent = dev;

	ret = misc_register(&wave->miscdev);
	if (ret)
		goto err_free_id;

	platform_set_drvdata(pdev, wave);
	dev_info(dev, "registered /dev/%s with %u wave entries\n",
		 wave->miscdev.name, PWM_WAVE_TABLE_SIZE);

	return 0;

err_free_id:
	ida_free(&pwm_wave_ida, wave->id);
	return ret;
}

static int rockchip_pwm_wave_remove(struct platform_device *pdev)
{
	struct rockchip_pwm_wave *wave = platform_get_drvdata(pdev);

	misc_deregister(&wave->miscdev);
	mutex_lock(&wave->lock);
	rockchip_pwm_wave_stop(wave);
	mutex_unlock(&wave->lock);
	ida_free(&pwm_wave_ida, wave->id);
	mutex_destroy(&wave->lock);

	return 0;
}

static const struct of_device_id rockchip_pwm_wave_of_match[] = {
	{ .compatible = "rockchip,pwm-wave" },
	{ }
};
MODULE_DEVICE_TABLE(of, rockchip_pwm_wave_of_match);

static struct platform_driver rockchip_pwm_wave_driver = {
	.probe = rockchip_pwm_wave_probe,
	.remove = rockchip_pwm_wave_remove,
	.driver = {
		.name = "rockchip-pwm-wave",
		.of_match_table = rockchip_pwm_wave_of_match,
	},
};
module_platform_driver(rockchip_pwm_wave_driver);

MODULE_DESCRIPTION("Userspace interface for Rockchip PWM wave tables");
MODULE_LICENSE("GPL");
