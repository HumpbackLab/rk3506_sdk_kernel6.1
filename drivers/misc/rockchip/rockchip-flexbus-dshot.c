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
#define RK_DSHOT_MIN_SAMPLES_PER_BIT	16
#define RK_DSHOT_MAX_SAMPLES_PER_BIT	128
#define RK_DSHOT_FRAME_BITS		16
#define RK_DSHOT_CHANNELS		4
#define RK_DSHOT_DEFAULT_SAMPLES_PER_BIT	32
#define RK_DSHOT_MAX_TX_SAMPLES		(RK_DSHOT_FRAME_BITS * RK_DSHOT_MAX_SAMPLES_PER_BIT)
#define RK_DSHOT_MAX_DMA_LEN		round_up(DIV_ROUND_UP(RK_DSHOT_MAX_TX_SAMPLES, 2), 0x40)
#define RK_DSHOT_TIMEOUT_MS		100
#define RK_DSHOT_PASSTHROUGH_MAX_SAMPLES	524288
#define RK_DSHOT_PASSTHROUGH_MAX_DMA_LEN	\
	round_up(DIV_ROUND_UP(RK_DSHOT_PASSTHROUGH_MAX_SAMPLES, 2), 0x40)
#define RK_DSHOT_PASSTHROUGH_MIN_SAMPLE_RATE	100000
#define RK_DSHOT_PASSTHROUGH_MAX_SAMPLE_RATE	10000000
#define RK_DSHOT_PASSTHROUGH_DEFAULT_TIMEOUT_MS	100

#define RK_DSHOT_IOCTL_BASE		'D'
#define RK_DSHOT_IOC_SET_RATE		_IOW(RK_DSHOT_IOCTL_BASE, 0x00, __u32)
#define RK_DSHOT_IOC_SET_TELEMETRY	_IOW(RK_DSHOT_IOCTL_BASE, 0x01, __u32)
#define RK_DSHOT_IOC_SEND_CMD		_IOW(RK_DSHOT_IOCTL_BASE, 0x02, __u32)
#define RK_DSHOT_IOC_SEND_FRAME		_IOW(RK_DSHOT_IOCTL_BASE, 0x03, struct rk_dshot_frame)
#define RK_DSHOT_IOC_PASSTHROUGH_XFER	_IOWR(RK_DSHOT_IOCTL_BASE, 0x04, \
					      struct rk_dshot_passthrough_xfer)

#define RK_DSHOT_PASSTHROUGH_F_TX_INVERT	BIT(0)
#define RK_DSHOT_PASSTHROUGH_F_RX_INVERT	BIT(1)

#define RK_DSHOT_ERR_ISR		(FLEXBUS_DMA_TIMEOUT_ISR | FLEXBUS_DMA_ERR_ISR | \
					 FLEXBUS_TX_UDF_ISR | FLEXBUS_TX_OVF_ISR)
#define RK_DSHOT_ISR			(RK_DSHOT_ERR_ISR | FLEXBUS_TX_DONE_ISR)
#define RK_DSHOT_PASSTHROUGH_ERR_ISR	(RK_DSHOT_ERR_ISR | FLEXBUS_RX_UDF_ISR | \
					 FLEXBUS_RX_OVF_ISR)
#define RK_DSHOT_PASSTHROUGH_ISR	(RK_DSHOT_PASSTHROUGH_ERR_ISR | \
					 FLEXBUS_TX_DONE_ISR | FLEXBUS_RX_DONE_ISR)

enum rk_dshot_result {
	RK_DSHOT_DONE = 0,
	RK_DSHOT_ERR,
};

struct rk_dshot_frame {
	__u16 value[RK_DSHOT_CHANNELS];
};

struct rk_dshot_passthrough_xfer {
	__u64 tx_buf;
	__u64 rx_buf;
	__u32 tx_samples;
	__u32 rx_samples;
	__u32 sample_rate;
	__u32 channel;
	__u32 timeout_ms;
	__u32 flags;
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
	u8 *rx_buf;
	dma_addr_t rx_dma;
	u32 rate;
	u32 actual_rate;
	u32 tx_clk_rate;
	u32 samples_per_bit;
	u32 tx_samples;
	u32 dma_len;
	u32 passthrough_actual_sample_rate;
	bool telemetry;
	bool inverted;
	bool passthrough_rx_reversed;
	bool passthrough_active;
	bool passthrough_wait_tx;
	bool passthrough_wait_rx;
	bool passthrough_tx_done;
	bool passthrough_rx_done;
};

static int rk_dshot_hw_init(struct rk_flexbus_dshot *dshot);

static u16 rk_dshot_make_frame(u16 value, bool telemetry, bool inverted)
{
	u16 packet = (value << 1) | telemetry;
	u16 csum = 0;
	u16 csum_data = packet;
	int i;

	for (i = 0; i < 3; i++) {
		csum ^= csum_data;
		csum_data >>= 4;
	}

	if (inverted)
		csum = ~csum;

	return (packet << 4) | (csum & 0xf);
}

static void rk_dshot_set_sample(u8 *buf, unsigned int index, unsigned int channel, bool high)
{
	u8 mask = BIT(channel);

	/* FLEXBUS_TX_CTL_MSB sends the eight nibbles in each DMA word in reverse order. */
	index ^= 7;
	if (index & 1)
		mask <<= 4;

	if (high)
		buf[index / 2] |= mask;
	else
		buf[index / 2] &= ~mask;
}

static bool rk_dshot_get_sample(const u8 *buf, unsigned int index, unsigned int channel)
{
	u8 sample = buf[index / 2];

	if (index & 1)
		sample >>= 4;

	return sample & BIT(channel);
}

static void rk_dshot_encode(struct rk_flexbus_dshot *dshot, const u16 value[RK_DSHOT_CHANNELS])
{
	u16 frame[RK_DSHOT_CHANNELS];
	unsigned int sample = 0;
	int bit, channel, i, high_time;

	memset(dshot->tx_buf, dshot->inverted ? 0xff : 0x00, dshot->dma_len);
	for (channel = 0; channel < RK_DSHOT_CHANNELS; channel++)
		frame[channel] = rk_dshot_make_frame(value[channel], dshot->telemetry,
						     dshot->inverted);

	for (bit = RK_DSHOT_FRAME_BITS - 1; bit >= 0; bit--) {
		for (i = 0; i < dshot->samples_per_bit; i++) {
			for (channel = 0; channel < RK_DSHOT_CHANNELS; channel++) {
				high_time = (frame[channel] & BIT(bit)) ?
					    DIV_ROUND_CLOSEST(dshot->samples_per_bit * 3, 4) :
					    DIV_ROUND_CLOSEST(dshot->samples_per_bit * 3, 8);
				rk_dshot_set_sample(dshot->tx_buf, sample, channel,
						    dshot->inverted ? i >= high_time : i < high_time);
			}
			sample++;
		}
	}
}

static void rk_dshot_update_timing(struct rk_flexbus_dshot *dshot,
				   u32 rate, u32 tx_clk_rate, u32 samples_per_bit)
{
	dshot->rate = rate;
	dshot->tx_clk_rate = tx_clk_rate;
	dshot->samples_per_bit = samples_per_bit;
	dshot->actual_rate = DIV_ROUND_CLOSEST(tx_clk_rate, samples_per_bit * 2);
	dshot->tx_samples = RK_DSHOT_FRAME_BITS * samples_per_bit;
	dshot->dma_len = round_up(DIV_ROUND_UP(dshot->tx_samples, 2), 0x40);
}

static int rk_dshot_set_rate(struct rk_flexbus_dshot *dshot, u32 rate)
{
	u32 best_samples = RK_DSHOT_DEFAULT_SAMPLES_PER_BIT;
	u32 best_clk_rate = rate * best_samples * 2;
	u32 best_error = U32_MAX;
	u32 samples;
	int ret;

	if (rate < RK_DSHOT_MIN_RATE || rate > RK_DSHOT_MAX_RATE)
		return -EINVAL;

	for (samples = RK_DSHOT_MIN_SAMPLES_PER_BIT;
	     samples <= RK_DSHOT_MAX_SAMPLES_PER_BIT; samples++) {
		unsigned long target = rate * samples * 2;
		long rounded = clk_round_rate(dshot->fb->clks[0].clk, target);
		u32 actual_rate;
		u32 error;

		if (rounded <= 0)
			continue;

		actual_rate = DIV_ROUND_CLOSEST_ULL(rounded, samples * 2);
		error = abs((int)actual_rate - (int)rate);
		if (error < best_error ||
		    (error == best_error &&
		     abs((int)samples - RK_DSHOT_DEFAULT_SAMPLES_PER_BIT) <
		     abs((int)best_samples - RK_DSHOT_DEFAULT_SAMPLES_PER_BIT))) {
			best_error = error;
			best_samples = samples;
			best_clk_rate = rounded;
		}
	}

	ret = clk_set_rate(dshot->fb->clks[0].clk, best_clk_rate);
	if (ret)
		return ret;

	rk_dshot_update_timing(dshot, rate, clk_get_rate(dshot->fb->clks[0].clk),
			       best_samples);

	dev_dbg(dshot->dev, "rate=%u actual=%u tx_clk=%u samples_per_bit=%u error=%d\n",
		dshot->rate, dshot->actual_rate, dshot->tx_clk_rate,
		dshot->samples_per_bit, (int)dshot->actual_rate - (int)dshot->rate);

	return 0;
}

static int rk_dshot_set_passthrough_sample_rate(struct rk_flexbus_dshot *dshot, u32 sample_rate)
{
	struct rockchip_flexbus *fb = dshot->fb;
	unsigned long target = sample_rate * 2;
	long tx_rate;
	int ret;

	if (sample_rate < RK_DSHOT_PASSTHROUGH_MIN_SAMPLE_RATE ||
	    sample_rate > RK_DSHOT_PASSTHROUGH_MAX_SAMPLE_RATE)
		return -EINVAL;

	tx_rate = clk_round_rate(fb->clks[0].clk, target);
	if (tx_rate <= 0)
		return -EINVAL;

	ret = clk_set_rate(fb->clks[0].clk, tx_rate);
	if (ret)
		return ret;

	if (fb->num_clks > 1) {
		long rx_rate = clk_round_rate(fb->clks[1].clk, target);

		if (rx_rate <= 0)
			return -EINVAL;

		ret = clk_set_rate(fb->clks[1].clk, rx_rate);
		if (ret)
			return ret;
	}

	dshot->passthrough_actual_sample_rate =
		clk_get_rate(fb->clks[fb->num_clks > 1 ? 1 : 0].clk) / 2;

	return 0;
}

static int rk_dshot_xmit_buf_locked(struct rk_flexbus_dshot *dshot)
{
	struct rockchip_flexbus *fb = dshot->fb;
	int ret = 0;

	reinit_completion(&dshot->completion);
	dshot->result = RK_DSHOT_ERR;

	rockchip_flexbus_writel(fb, FLEXBUS_ICR, RK_DSHOT_ISR);
	rockchip_flexbus_writel(fb, FLEXBUS_COM_CTL, FLEXBUS_TX_ONLY);
	rockchip_flexbus_writel(fb, FLEXBUS_TX_NUM, dshot->tx_samples);
	rockchip_flexbus_writel(fb, FLEXBUS_TXWAT_START, 16);
	rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_ADDR0, dshot->tx_dma >> 2);
	rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_LEN0, dshot->dma_len);
	rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_DIS);
	rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_ENR);

	if (!wait_for_completion_timeout(&dshot->completion,
					 msecs_to_jiffies(RK_DSHOT_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	else if (dshot->result != RK_DSHOT_DONE)
		ret = -EIO;

	return ret;
}

static int rk_dshot_xmit_locked(struct rk_flexbus_dshot *dshot,
				const u16 value[RK_DSHOT_CHANNELS])
{
	int ret;

	rk_dshot_encode(dshot, value);
	ret = rk_dshot_xmit_buf_locked(dshot);

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

static void rk_dshot_prepare_passthrough_locked(struct rk_flexbus_dshot *dshot,
						bool wait_tx, bool wait_rx)
{
	reinit_completion(&dshot->completion);
	dshot->result = RK_DSHOT_ERR;
	dshot->passthrough_active = true;
	dshot->passthrough_wait_tx = wait_tx;
	dshot->passthrough_wait_rx = wait_rx;
	dshot->passthrough_tx_done = !wait_tx;
	dshot->passthrough_rx_done = !wait_rx;
}

static int rk_dshot_wait_passthrough_locked(struct rk_flexbus_dshot *dshot, u32 timeout_ms)
{
	unsigned long timeout = msecs_to_jiffies(timeout_ms);

	if (!wait_for_completion_timeout(&dshot->completion, timeout))
		return -ETIMEDOUT;
	if (dshot->result != RK_DSHOT_DONE)
		return -EIO;

	return 0;
}

static void rk_dshot_finish_passthrough_locked(struct rk_flexbus_dshot *dshot)
{
	dshot->passthrough_active = false;
	dshot->passthrough_wait_tx = false;
	dshot->passthrough_wait_rx = false;
}

static void rk_dshot_pack_passthrough_tx(struct rk_flexbus_dshot *dshot,
					 const u8 *samples, u32 sample_count,
					 u32 channel, u32 dma_len, u32 flags)
{
	u32 i;

	memset(dshot->tx_buf, 0x00, dma_len);

	for (i = 0; i < sample_count; i++)
		rk_dshot_set_sample(dshot->tx_buf, i, channel,
				    (flags & RK_DSHOT_PASSTHROUGH_F_TX_INVERT) ?
				    !samples[i] : !!samples[i]);
}

static void rk_dshot_unpack_passthrough_rx(struct rk_flexbus_dshot *dshot,
					   u8 *samples, u32 sample_count, u32 channel,
					   u32 flags)
{
	u32 rx_channel = dshot->passthrough_rx_reversed ?
			(RK_DSHOT_CHANNELS - 1 - channel) : channel;
	u32 i;

	for (i = 0; i < sample_count; i++)
		samples[i] = (rk_dshot_get_sample(dshot->rx_buf, i, rx_channel) ^
			      !!(flags & RK_DSHOT_PASSTHROUGH_F_RX_INVERT)) ? 1 : 0;
}

static int rk_dshot_passthrough_xfer_locked(struct rk_flexbus_dshot *dshot,
					    const struct rk_dshot_passthrough_xfer *xfer,
					    const u8 *tx_samples, u8 *rx_samples)
{
	struct rockchip_flexbus *fb = dshot->fb;
	u32 timeout_ms = xfer->timeout_ms ?: RK_DSHOT_PASSTHROUGH_DEFAULT_TIMEOUT_MS;
	u32 tx_dma_len = round_up(DIV_ROUND_UP(xfer->tx_samples, 2), 0x40);
	u32 rx_dma_len = round_up(DIV_ROUND_UP(xfer->rx_samples, 2), 0x40);
	u32 tx_ctl = fb->dfs_reg->dfs_4bit | FLEXBUS_TX_CTL_MSB;
	u32 rx_ctl = fb->dfs_reg->dfs_4bit | FLEXBUS_RX_CTL_MSB;
	int ret;

	if (xfer->channel >= RK_DSHOT_CHANNELS)
		return -EINVAL;
	if (!xfer->tx_samples && !xfer->rx_samples)
		return -EINVAL;
	if (xfer->tx_samples > RK_DSHOT_PASSTHROUGH_MAX_SAMPLES ||
	    xfer->rx_samples > RK_DSHOT_PASSTHROUGH_MAX_SAMPLES)
		return -EINVAL;
	if (tx_dma_len > RK_DSHOT_PASSTHROUGH_MAX_DMA_LEN ||
	    rx_dma_len > RK_DSHOT_PASSTHROUGH_MAX_DMA_LEN)
		return -EINVAL;

	ret = rk_dshot_set_passthrough_sample_rate(dshot, xfer->sample_rate);
	if (ret) {
		rk_dshot_hw_init(dshot);
		return ret;
	}

	rockchip_flexbus_writel(fb, FLEXBUS_ICR, RK_DSHOT_PASSTHROUGH_ISR);
	rockchip_flexbus_writel(fb, FLEXBUS_IMR, RK_DSHOT_PASSTHROUGH_ISR);
	rockchip_flexbus_writel(fb, FLEXBUS_ENR, 0xffff0000);
	rockchip_flexbus_writel(fb, FLEXBUS_FREE_SCLK, FLEXBUS_RX_FREE_MODE);
	rockchip_flexbus_writel(fb, FLEXBUS_SLAVE_MODE, 0);
	rockchip_flexbus_writel(fb, FLEXBUS_REMAP, 0);
	rockchip_flexbus_writel(fb, FLEXBUS_TX_CTL, tx_ctl);
	rockchip_flexbus_writel(fb, FLEXBUS_RX_CTL, rx_ctl);
	fb->config->grf_config(fb, false, false, false);

	if (xfer->tx_samples && xfer->rx_samples) {
		rk_dshot_pack_passthrough_tx(dshot, tx_samples, xfer->tx_samples,
					     xfer->channel, tx_dma_len, xfer->flags);
		memset(dshot->rx_buf, 0, rx_dma_len);

		rockchip_flexbus_writel(fb, FLEXBUS_COM_CTL, FLEXBUS_TX_AND_RX);
		rockchip_flexbus_writel(fb, FLEXBUS_TX_NUM, xfer->tx_samples);
		rockchip_flexbus_writel(fb, FLEXBUS_RX_NUM, xfer->rx_samples);
		rockchip_flexbus_writel(fb, FLEXBUS_TXWAT_START, 16);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_ADDR0, dshot->tx_dma >> 2);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_DST_ADDR0, dshot->rx_dma >> 2);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_LEN0, tx_dma_len);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_DST_LEN0, rx_dma_len);
		rk_dshot_prepare_passthrough_locked(dshot, true, true);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_RX_DIS | FLEXBUS_TX_DIS);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_RX_ENR | FLEXBUS_TX_ENR);

		ret = rk_dshot_wait_passthrough_locked(dshot, timeout_ms);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_RX_DIS | FLEXBUS_TX_DIS);
		rk_dshot_finish_passthrough_locked(dshot);
		if (ret)
			goto restore;

		rk_dshot_unpack_passthrough_rx(dshot, rx_samples, xfer->rx_samples,
					       xfer->channel, xfer->flags);
		goto restore;
	}

	if (xfer->tx_samples) {
		rk_dshot_pack_passthrough_tx(dshot, tx_samples, xfer->tx_samples,
					     xfer->channel, tx_dma_len, xfer->flags);

		rockchip_flexbus_writel(fb, FLEXBUS_COM_CTL, FLEXBUS_TX_ONLY);
		rockchip_flexbus_writel(fb, FLEXBUS_TX_NUM, xfer->tx_samples);
		rockchip_flexbus_writel(fb, FLEXBUS_TXWAT_START, 16);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_ADDR0, dshot->tx_dma >> 2);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_SRC_LEN0, tx_dma_len);
		rk_dshot_prepare_passthrough_locked(dshot, true, false);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_DIS);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_ENR);

		ret = rk_dshot_wait_passthrough_locked(dshot, timeout_ms);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_DIS);
		rk_dshot_finish_passthrough_locked(dshot);
		if (ret)
			goto restore;
	}

	if (xfer->rx_samples) {
		memset(dshot->rx_buf, 0, rx_dma_len);

		rockchip_flexbus_writel(fb, FLEXBUS_COM_CTL, FLEXBUS_RX_ONLY);
		rockchip_flexbus_writel(fb, FLEXBUS_RX_NUM, xfer->rx_samples);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_DST_ADDR0, dshot->rx_dma >> 2);
		rockchip_flexbus_writel(fb, FLEXBUS_DMA_DST_LEN0, rx_dma_len);
		rk_dshot_prepare_passthrough_locked(dshot, false, true);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_RX_DIS);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_RX_ENR);

		ret = rk_dshot_wait_passthrough_locked(dshot, timeout_ms);
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_RX_DIS);
		rk_dshot_finish_passthrough_locked(dshot);
		if (ret)
			goto restore;

		rk_dshot_unpack_passthrough_rx(dshot, rx_samples, xfer->rx_samples,
					       xfer->channel, xfer->flags);
	}

restore:
	rockchip_flexbus_writel(fb, FLEXBUS_ENR, 0xffff0000);
	rk_dshot_hw_init(dshot);

	return ret;
}

static void rk_dshot_irq_handler(struct rockchip_flexbus *fb, u32 isr)
{
	struct rk_flexbus_dshot *dshot = fb->fb0_data;

	if (fb->opmode0 != ROCKCHIP_FLEXBUS0_OPMODE_DSHOT || !dshot)
		return;

	if (dshot->passthrough_active) {
		rockchip_flexbus_writel(fb, FLEXBUS_ICR, isr & RK_DSHOT_PASSTHROUGH_ISR);

		if (isr & RK_DSHOT_PASSTHROUGH_ERR_ISR) {
			rockchip_flexbus_writel(fb, FLEXBUS_ENR, 0xffff0000);
			dshot->result = RK_DSHOT_ERR;
			dev_err_ratelimited(dshot->dev, "passthrough error isr=0x%08x\n", isr);
			complete(&dshot->completion);
			return;
		}

		if (isr & FLEXBUS_TX_DONE_ISR)
			dshot->passthrough_tx_done = true;
		if (isr & FLEXBUS_RX_DONE_ISR)
			dshot->passthrough_rx_done = true;

		if (dshot->passthrough_tx_done && dshot->passthrough_rx_done) {
			dshot->result = RK_DSHOT_DONE;
			complete(&dshot->completion);
		}
		return;
	}

	rockchip_flexbus_writel(fb, FLEXBUS_ICR, isr & RK_DSHOT_ISR);

	if (isr & FLEXBUS_TX_DONE_ISR) {
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_DIS);
		dshot->result = RK_DSHOT_DONE;
		complete(&dshot->completion);
		return;
	}

	if (isr & RK_DSHOT_ERR_ISR) {
		rockchip_flexbus_writel(fb, FLEXBUS_ENR, FLEXBUS_TX_DIS);
		dshot->result = RK_DSHOT_ERR;
		dev_err_ratelimited(dshot->dev, "transfer error isr=0x%08x\n", isr);
		if (isr & FLEXBUS_DMA_TIMEOUT_ISR)
			dev_err_ratelimited(dshot->dev, "dma timeout\n");
		if (isr & FLEXBUS_DMA_ERR_ISR)
			dev_err_ratelimited(dshot->dev, "dma error\n");
		if (isr & FLEXBUS_TX_UDF_ISR)
			dev_err_ratelimited(dshot->dev, "tx underflow\n");
		if (isr & FLEXBUS_TX_OVF_ISR)
			dev_err_ratelimited(dshot->dev, "tx overflow\n");
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

static struct attribute *rk_dshot_attrs[] = {
	&dev_attr_rate_hz.attr,
	&dev_attr_telemetry.attr,
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

static long rk_dshot_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk_flexbus_dshot *dshot = file->private_data;
	struct rk_dshot_frame frame;
	struct rk_dshot_passthrough_xfer xfer;
	u16 values[RK_DSHOT_CHANNELS];
	u8 *tx_samples = NULL;
	u8 *rx_samples = NULL;
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
	case RK_DSHOT_IOC_PASSTHROUGH_XFER:
		if (copy_from_user(&xfer, (void __user *)arg, sizeof(xfer)))
			return -EFAULT;
		if (xfer.flags & ~(RK_DSHOT_PASSTHROUGH_F_TX_INVERT |
				   RK_DSHOT_PASSTHROUGH_F_RX_INVERT))
			return -EINVAL;
		if (xfer.channel >= RK_DSHOT_CHANNELS ||
		    xfer.tx_samples > RK_DSHOT_PASSTHROUGH_MAX_SAMPLES ||
		    xfer.rx_samples > RK_DSHOT_PASSTHROUGH_MAX_SAMPLES ||
		    (!xfer.tx_samples && !xfer.rx_samples))
			return -EINVAL;
		if (xfer.tx_samples) {
			tx_samples = memdup_user(u64_to_user_ptr(xfer.tx_buf), xfer.tx_samples);
			if (IS_ERR(tx_samples))
				return PTR_ERR(tx_samples);
		}
		if (xfer.rx_samples) {
			rx_samples = kzalloc(xfer.rx_samples, GFP_KERNEL);
			if (!rx_samples) {
				kfree(tx_samples);
				return -ENOMEM;
			}
		}

		mutex_lock(&dshot->lock);
		ret = rk_dshot_passthrough_xfer_locked(dshot, &xfer, tx_samples, rx_samples);
		if (!ret)
			xfer.sample_rate = dshot->passthrough_actual_sample_rate;
		mutex_unlock(&dshot->lock);

		if (!ret && xfer.rx_samples &&
		    copy_to_user(u64_to_user_ptr(xfer.rx_buf), rx_samples, xfer.rx_samples))
			ret = -EFAULT;
		if (!ret && copy_to_user((void __user *)arg, &xfer, sizeof(xfer)))
			ret = -EFAULT;

		kfree(rx_samples);
		kfree(tx_samples);
		return ret;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations rk_dshot_fops = {
	.owner = THIS_MODULE,
	.open = rk_dshot_open,
	.unlocked_ioctl = rk_dshot_ioctl,
	.llseek = no_llseek,
};

static int rk_dshot_hw_init(struct rk_flexbus_dshot *dshot)
{
	struct rockchip_flexbus *fb = dshot->fb;
	u32 tx_ctl = fb->dfs_reg->dfs_4bit | FLEXBUS_TX_CTL_MSB;

	fb->config->grf_config(fb, false, false, false);

	rockchip_flexbus_writel(fb, FLEXBUS_ENR, 0xffff0000);
	rockchip_flexbus_writel(fb, FLEXBUS_FREE_SCLK, 0x30000);
	rockchip_flexbus_writel(fb, FLEXBUS_CSN_CFG, 0x30000);
	rockchip_flexbus_writel(fb, FLEXBUS_SLAVE_MODE, 0);
	rockchip_flexbus_writel(fb, FLEXBUS_REMAP, 0);
	rockchip_flexbus_writel(fb, FLEXBUS_TX_CTL, tx_ctl);
	rockchip_flexbus_writel(fb, FLEXBUS_IMR, RK_DSHOT_ISR);

	return rk_dshot_set_rate(dshot, dshot->rate);
}

static int rk_dshot_parse_polarity(struct device *dev, struct rk_flexbus_dshot *dshot)
{
	const char *polarity;
	int ret;

	dshot->inverted = true;

	ret = device_property_read_string(dev, "rockchip,dshot-polarity", &polarity);
	if (!ret) {
		if (!strcmp(polarity, "normal")) {
			dshot->inverted = false;
			return 0;
		}
		if (!strcmp(polarity, "invert") || !strcmp(polarity, "inverted")) {
			dshot->inverted = true;
			return 0;
		}

		return dev_err_probe(dev, -EINVAL,
				     "invalid rockchip,dshot-polarity: %s\n", polarity);
	}

	return 0;
}

static int rk_dshot_probe(struct platform_device *pdev)
{
	struct rockchip_flexbus *fb = dev_get_drvdata(pdev->dev.parent);
	struct rk_flexbus_dshot *dshot;
	int ret;

	if (fb->opmode0 != ROCKCHIP_FLEXBUS0_OPMODE_DSHOT ||
	    (fb->opmode1 != ROCKCHIP_FLEXBUS1_OPMODE_NULL &&
	     fb->opmode1 != ROCKCHIP_FLEXBUS1_OPMODE_ADC)) {
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
	dshot->passthrough_rx_reversed =
		device_property_read_bool(&pdev->dev, "rockchip,passthrough-rx-reversed");
	ret = rk_dshot_parse_polarity(&pdev->dev, dshot);
	if (ret)
		goto err_mutex;

	dshot->tx_buf = dmam_alloc_coherent(dshot->dev, RK_DSHOT_PASSTHROUGH_MAX_DMA_LEN,
					    &dshot->tx_dma,
					    GFP_KERNEL | __GFP_ZERO);
	if (!dshot->tx_buf) {
		ret = -ENOMEM;
		goto err_mutex;
	}
	dshot->rx_buf = dmam_alloc_coherent(dshot->dev, RK_DSHOT_PASSTHROUGH_MAX_DMA_LEN,
					    &dshot->rx_dma,
					    GFP_KERNEL | __GFP_ZERO);
	if (!dshot->rx_buf) {
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

	dev_info(&pdev->dev,
		 "channels=%u rate=%u actual=%u tx_clk=%u samples_per_bit=%u telemetry=%u polarity=%s\n",
		 RK_DSHOT_CHANNELS, dshot->rate, dshot->actual_rate,
		 dshot->tx_clk_rate, dshot->samples_per_bit, dshot->telemetry,
		 dshot->inverted ? "inverted" : "normal");

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
