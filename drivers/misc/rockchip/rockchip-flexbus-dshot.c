// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Rockchip Flexbus DShot protocol driver
 *
 * Copyright (C) 2026 Rockchip Electronics Co., Ltd.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <dt-bindings/mfd/rockchip-flexbus.h>
#include <linux/mfd/rockchip-flexbus.h>

#define RK_DSHOT_DEFAULT_RATE		600000
#define RK_DSHOT_MIN_RATE		150000
#define RK_DSHOT_MAX_RATE		1200000
#define RK_DSHOT_SAMPLES_PER_BIT	8
#define RK_DSHOT_FRAME_BITS		16
#define RK_DSHOT_CHANNELS		4
#define RK_DSHOT_ZERO_HIGH		3
#define RK_DSHOT_ONE_HIGH		6
#define RK_DSHOT_DMA_LEN		64
#define RK_DSHOT_TIMEOUT_MS		100

#define RK_DSHOT_IOCTL_BASE		'D'
#define RK_DSHOT_IOC_SET_RATE		_IOW(RK_DSHOT_IOCTL_BASE, 0x00, __u32)
#define RK_DSHOT_IOC_SET_TELEMETRY	_IOW(RK_DSHOT_IOCTL_BASE, 0x01, __u32)
#define RK_DSHOT_IOC_SEND_CMD		_IOW(RK_DSHOT_IOCTL_BASE, 0x02, __u32)
#define RK_DSHOT_IOC_SEND_FRAME		_IOW(RK_DSHOT_IOCTL_BASE, 0x03, struct rk_dshot_frame)

#define RK_DSHOT_ERR_ISR		(FLEXBUS_DMA_TIMEOUT_ISR | FLEXBUS_DMA_ERR_ISR | \
					 FLEXBUS_TX_UDF_ISR | FLEXBUS_TX_OVF_ISR)
#define RK_DSHOT_ISR			(RK_DSHOT_ERR_ISR | FLEXBUS_TX_DONE_ISR)

enum rk_dshot_result {
	RK_DSHOT_DONE = 0,
	RK_DSHOT_ERR,
};

struct rk_dshot_frame {
	__u16 value[RK_DSHOT_CHANNELS];
};

struct rk_flexbus_dshot {
	struct device *dev;
	struct rockchip_flexbus *fb;
	struct miscdevice miscdev;
	struct mutex lock;
	struct completion completion;
	enum rk_dshot_result result;
	u8 *tx_buf;
	dma_addr_t tx_dma;
	u32 rate;
	u16 last_value[RK_DSHOT_CHANNELS];
	bool telemetry;
};

static u16 rk_dshot_make_frame(u16 value, bool telemetry)
{
	u16 packet = (value << 1) | telemetry;
	u16 csum = 0;
	u16 csum_data = packet;
	int i;

	for (i = 0; i < 3; i++) {
		csum ^= csum_data;
		csum_data >>= 4;
	}

	return (packet << 4) | (csum & 0xf);
}

static void rk_dshot_set_sample(u8 *buf, unsigned int index, unsigned int channel,
				bool high)
{
	u8 mask = BIT(channel);

	if (!high)
		return;

	if (index & 1)
		mask <<= 4;

	buf[index / 2] |= mask;
}

static void rk_dshot_encode(struct rk_flexbus_dshot *dshot, const u16 value[RK_DSHOT_CHANNELS])
{
	u16 frame[RK_DSHOT_CHANNELS];
	unsigned int sample = 0;
	int bit, channel, i, high_time;

	memset(dshot->tx_buf, 0, RK_DSHOT_DMA_LEN);
	for (channel = 0; channel < RK_DSHOT_CHANNELS; channel++)
		frame[channel] = rk_dshot_make_frame(value[channel], dshot->telemetry);

	for (bit = RK_DSHOT_FRAME_BITS - 1; bit >= 0; bit--) {
		for (i = 0; i < RK_DSHOT_SAMPLES_PER_BIT; i++) {
			for (channel = 0; channel < RK_DSHOT_CHANNELS; channel++) {
				high_time = (frame[channel] & BIT(bit)) ?
					    RK_DSHOT_ONE_HIGH : RK_DSHOT_ZERO_HIGH;
				rk_dshot_set_sample(dshot->tx_buf, sample, channel,
						    i < high_time);
			}
			sample++;
		}
	}
}

static int rk_dshot_set_rate(struct rk_flexbus_dshot *dshot, u32 rate)
{
	int ret;

	if (rate < RK_DSHOT_MIN_RATE || rate > RK_DSHOT_MAX_RATE)
		return -EINVAL;

	ret = clk_set_rate(dshot->fb->clks[0].clk,
			   rate * RK_DSHOT_SAMPLES_PER_BIT * 2);
	if (ret)
		return ret;

	dshot->rate = rate;

	return 0;
}

static int rk_dshot_xmit_locked(struct rk_flexbus_dshot *dshot,
				const u16 value[RK_DSHOT_CHANNELS])
{
	struct rockchip_flexbus *fb = dshot->fb;
	u32 num = RK_DSHOT_FRAME_BITS * RK_DSHOT_SAMPLES_PER_BIT;
	int ret = 0;

	reinit_completion(&dshot->completion);
	dshot->result = RK_DSHOT_ERR;

	rk_dshot_encode(dshot, value);

	rockchip_flexbus_writel(fb, FLEXBUS_ICR, RK_DSHOT_ISR);
	rockchip_flexbus_writel(fb, FLEXBUS_COM_CTL, FLEXBUS_TX_ONLY);
	rockchip_flexbus_writel(fb, FLEXBUS_TX_NUM, num);
	rockchip_flexbus_writel(fb, FLEXBUS_TXWAT_START, 16);
	rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_ADDR0, dshot->tx_dma >> 2);
	rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_LEN0, RK_DSHOT_DMA_LEN);
	rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_ENR);

	if (!wait_for_completion_timeout(&dshot->completion,
					 msecs_to_jiffies(RK_DSHOT_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (dshot->result != RK_DSHOT_DONE)
		ret = -EIO;

	rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_DIS);
	memcpy(dshot->last_value, value, sizeof(dshot->last_value));

	return ret;
}

static int rk_dshot_xmit(struct rk_flexbus_dshot *dshot,
			 const u16 value[RK_DSHOT_CHANNELS])
{
	int ret;
	int i;

	for (i = 0; i < RK_DSHOT_CHANNELS; i++)
		if (value[i] > 0x7ff)
			return -EINVAL;

	mutex_lock(&dshot->lock);
	ret = rk_dshot_xmit_locked(dshot, value);
	mutex_unlock(&dshot->lock);

	return ret;
}

static void rk_dshot_irq_handler(struct rockchip_flexbus *fb, u32 isr)
{
	struct rk_flexbus_dshot *dshot = fb->fb0_data;

	if (fb->opmode0 != ROCKCHIP_FLEXBUS0_OPMODE_DSHOT || !dshot)
		return;

	rockchip_flexbus_writel(fb, FLEXBUS_ICR, isr & RK_DSHOT_ISR);

	if (isr & RK_DSHOT_ERR_ISR) {
		dshot->result = RK_DSHOT_ERR;
		if (isr & FLEXBUS_DMA_TIMEOUT_ISR)
			dev_err_ratelimited(dshot->dev, "dma timeout\n");
		if (isr & FLEXBUS_DMA_ERR_ISR)
			dev_err_ratelimited(dshot->dev, "dma error\n");
		if (isr & FLEXBUS_TX_UDF_ISR)
			dev_err_ratelimited(dshot->dev, "tx underflow\n");
		if (isr & FLEXBUS_TX_OVF_ISR)
			dev_err_ratelimited(dshot->dev, "tx overflow\n");
		complete(&dshot->completion);
		return;
	}

	if (isr & FLEXBUS_TX_DONE_ISR) {
		dshot->result = RK_DSHOT_DONE;
		complete(&dshot->completion);
	}
}

static ssize_t rate_hz_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rk_flexbus_dshot *dshot = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", dshot->rate);
}

static ssize_t rate_hz_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct rk_flexbus_dshot *dshot = dev_get_drvdata(dev);
	u32 rate;
	int ret;

	ret = kstrtou32(buf, 0, &rate);
	if (ret)
		return ret;

	mutex_lock(&dshot->lock);
	ret = rk_dshot_set_rate(dshot, rate);
	mutex_unlock(&dshot->lock);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(rate_hz);

static ssize_t telemetry_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rk_flexbus_dshot *dshot = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", dshot->telemetry);
}

static ssize_t telemetry_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct rk_flexbus_dshot *dshot = dev_get_drvdata(dev);
	bool telemetry;
	int ret;

	ret = kstrtobool(buf, &telemetry);
	if (ret)
		return ret;

	mutex_lock(&dshot->lock);
	dshot->telemetry = telemetry;
	mutex_unlock(&dshot->lock);

	return count;
}
static DEVICE_ATTR_RW(telemetry);

static ssize_t last_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rk_flexbus_dshot *dshot = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u %u %u %u\n",
			  dshot->last_value[0], dshot->last_value[1],
			  dshot->last_value[2], dshot->last_value[3]);
}
static DEVICE_ATTR_RO(last_value);

static ssize_t channels_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", RK_DSHOT_CHANNELS);
}
static DEVICE_ATTR_RO(channels);

static struct attribute *rk_dshot_attrs[] = {
	&dev_attr_rate_hz.attr,
	&dev_attr_telemetry.attr,
	&dev_attr_last_value.attr,
	&dev_attr_channels.attr,
	NULL,
};
ATTRIBUTE_GROUPS(rk_dshot);

static int rk_dshot_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct rk_flexbus_dshot *dshot;

	dshot = container_of(miscdev, struct rk_flexbus_dshot, miscdev);
	file->private_data = dshot;

	return 0;
}

static int rk_dshot_broadcast(u16 value[RK_DSHOT_CHANNELS], u32 raw)
{
	int i;

	if (raw > 0x7ff)
		return -EINVAL;

	for (i = 0; i < RK_DSHOT_CHANNELS; i++)
		value[i] = raw;

	return 0;
}

static int rk_dshot_parse_text(char *buf, u16 value[RK_DSHOT_CHANNELS])
{
	char *token;
	u32 raw;
	int ret;
	int count = 0;

	while ((token = strsep(&buf, " \t\n")) != NULL) {
		if (!*token)
			continue;

		if (count >= RK_DSHOT_CHANNELS)
			return -EINVAL;

		ret = kstrtou32(token, 0, &raw);
		if (ret)
			return ret;
		if (raw > 0x7ff)
			return -EINVAL;

		value[count++] = raw;
	}

	if (count == 1)
		return rk_dshot_broadcast(value, value[0]);

	return count == RK_DSHOT_CHANNELS ? 0 : -EINVAL;
}

static ssize_t rk_dshot_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct rk_flexbus_dshot *dshot = file->private_data;
	u16 value[RK_DSHOT_CHANNELS];
	char tmp[64];
	int ret;

	if (!count)
		return 0;

	if (count < sizeof(tmp)) {
		if (copy_from_user(tmp, buf, count))
			return -EFAULT;
		tmp[count] = '\0';
		ret = rk_dshot_parse_text(tmp, value);
		if (!ret)
			goto send;
	}

	if (count == sizeof(u16)) {
		u16 raw;

		if (copy_from_user(&raw, buf, sizeof(raw)))
			return -EFAULT;
		ret = rk_dshot_broadcast(value, raw);
		if (ret)
			return ret;
	} else if (count == sizeof(u32)) {
		u32 raw;

		if (copy_from_user(&raw, buf, sizeof(raw)))
			return -EFAULT;
		ret = rk_dshot_broadcast(value, raw);
		if (ret)
			return ret;
	} else if (count == sizeof(value)) {
		if (copy_from_user(value, buf, sizeof(value)))
			return -EFAULT;
	} else {
		return -EINVAL;
	}

send:
	ret = rk_dshot_xmit(dshot, value);
	if (ret)
		return ret;

	return count;
}

static long rk_dshot_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk_flexbus_dshot *dshot = file->private_data;
	struct rk_dshot_frame frame;
	u16 values[RK_DSHOT_CHANNELS];
	u32 raw;
	int ret;

	switch (cmd) {
	case RK_DSHOT_IOC_SET_RATE:
		if (copy_from_user(&raw, (void __user *)arg, sizeof(raw)))
			return -EFAULT;
		mutex_lock(&dshot->lock);
		ret = rk_dshot_set_rate(dshot, raw);
		mutex_unlock(&dshot->lock);
		return ret;
	case RK_DSHOT_IOC_SET_TELEMETRY:
		if (copy_from_user(&raw, (void __user *)arg, sizeof(raw)))
			return -EFAULT;
		mutex_lock(&dshot->lock);
		dshot->telemetry = !!raw;
		mutex_unlock(&dshot->lock);
		return 0;
	case RK_DSHOT_IOC_SEND_CMD:
		if (copy_from_user(&raw, (void __user *)arg, sizeof(raw)))
			return -EFAULT;
		ret = rk_dshot_broadcast(values, raw);
		if (ret)
			return ret;
		return rk_dshot_xmit(dshot, values);
	case RK_DSHOT_IOC_SEND_FRAME:
		if (copy_from_user(&frame, (void __user *)arg, sizeof(frame)))
			return -EFAULT;
		memcpy(values, frame.value, sizeof(values));
		return rk_dshot_xmit(dshot, values);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations rk_dshot_fops = {
	.owner = THIS_MODULE,
	.open = rk_dshot_open,
	.write = rk_dshot_write,
	.unlocked_ioctl = rk_dshot_ioctl,
	.llseek = no_llseek,
};

static int rk_dshot_hw_init(struct rk_flexbus_dshot *dshot)
{
	struct rockchip_flexbus *fb = dshot->fb;
	u32 tx_ctl = fb->dfs_reg->dfs_4bit | FLEXBUS_TX_CTL_MSB;
	int ret;

	fb->config->grf_config(fb, false, false, false);

	rockchip_flexbus_writel(fb, FLEXBUS_ENR, 0xffff0000);
	rockchip_flexbus_writel(fb, FLEXBUS_FREE_SCLK, 0x30000);
	rockchip_flexbus_writel(fb, FLEXBUS_CSN_CFG, 0x30000);
	rockchip_flexbus_writel(fb, FLEXBUS_SLAVE_MODE, 0);
	rockchip_flexbus_writel(fb, FLEXBUS_REMAP, 0);
	rockchip_flexbus_writel(fb, FLEXBUS_TX_CTL, tx_ctl);
	rockchip_flexbus_writel(fb, FLEXBUS_IMR, RK_DSHOT_ISR);

	ret = rk_dshot_set_rate(dshot, dshot->rate);
	if (ret)
		return ret;

	return 0;
}

static int rk_dshot_probe(struct platform_device *pdev)
{
	struct rockchip_flexbus *fb = dev_get_drvdata(pdev->dev.parent);
	struct rk_flexbus_dshot *dshot;
	int ret;

	if (fb->opmode0 != ROCKCHIP_FLEXBUS0_OPMODE_DSHOT ||
	    fb->opmode1 != ROCKCHIP_FLEXBUS1_OPMODE_NULL) {
		dev_err(&pdev->dev, "flexbus opmode mismatch, fb0=%u fb1=%u\n",
			fb->opmode0, fb->opmode1);
		return -ENODEV;
	}

	if (!fb->dfs_reg->dfs_4bit)
		return dev_err_probe(&pdev->dev, -EINVAL, "4-bit dfs is unsupported\n");

	dshot = devm_kzalloc(&pdev->dev, sizeof(*dshot), GFP_KERNEL);
	if (!dshot)
		return -ENOMEM;

	dshot->dev = &pdev->dev;
	dshot->fb = fb;
	dshot->rate = RK_DSHOT_DEFAULT_RATE;
	mutex_init(&dshot->lock);
	init_completion(&dshot->completion);
	platform_set_drvdata(pdev, dshot);

	device_property_read_u32(&pdev->dev, "rockchip,dshot-rate", &dshot->rate);
	dshot->telemetry = device_property_read_bool(&pdev->dev, "rockchip,telemetry");

	dshot->tx_buf = dmam_alloc_coherent(dshot->dev, RK_DSHOT_DMA_LEN, &dshot->tx_dma,
					    GFP_KERNEL | __GFP_ZERO);
	if (!dshot->tx_buf) {
		ret = -ENOMEM;
		goto err_mutex;
	}

	rockchip_flexbus_set_fb0(fb, dshot, rk_dshot_irq_handler);

	ret = rk_dshot_hw_init(dshot);
	if (ret)
		goto err_fb0;

	dshot->miscdev.minor = MISC_DYNAMIC_MINOR;
	dshot->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
					     "rk-flexbus-dshot");
	if (!dshot->miscdev.name) {
		ret = -ENOMEM;
		goto err_fb0;
	}
	dshot->miscdev.fops = &rk_dshot_fops;
	dshot->miscdev.parent = &pdev->dev;
	dshot->miscdev.groups = rk_dshot_groups;

	ret = misc_register(&dshot->miscdev);
	if (ret)
		goto err_fb0;
	dev_set_drvdata(dshot->miscdev.this_device, dshot);

	dev_info(&pdev->dev, "channels=%u rate=%u telemetry=%u\n",
		 RK_DSHOT_CHANNELS, dshot->rate, dshot->telemetry);

	return 0;

err_fb0:
	rockchip_flexbus_set_fb0(fb, NULL, NULL);
err_mutex:
	mutex_destroy(&dshot->lock);

	return ret;
}

static int rk_dshot_remove(struct platform_device *pdev)
{
	struct rk_flexbus_dshot *dshot = platform_get_drvdata(pdev);

	misc_deregister(&dshot->miscdev);
	rockchip_flexbus_set_fb0(dshot->fb, NULL, NULL);
	mutex_destroy(&dshot->lock);

	return 0;
}

static const struct of_device_id rk_dshot_of_match[] = {
	{ .compatible = "rockchip,rk3506-flexbus-dshot" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rk_dshot_of_match);

static struct platform_driver rk_dshot_driver = {
	.probe = rk_dshot_probe,
	.remove = rk_dshot_remove,
	.driver = {
		.name = "rockchip-flexbus-dshot",
		.of_match_table = rk_dshot_of_match,
	},
};
module_platform_driver(rk_dshot_driver);

MODULE_DESCRIPTION("Rockchip Flexbus DShot protocol driver");
MODULE_LICENSE("GPL");
