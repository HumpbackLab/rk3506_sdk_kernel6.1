// SPDX-License-Identifier: GPL-2.0-only
/* Silan SC7U22 six-axis IMU IIO driver. */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <asm/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>

#define SC7U22_REG_WHO_AM_I		0x01
#define SC7U22_REG_COM_CONF		0x04
#define SC7U22_REG_INT1_OUT_SEL1	0x05
#define SC7U22_REG_INT1_OUT_SEL2	0x06
#define SC7U22_REG_FIFO_CFG0		0x1c
#define SC7U22_REG_FIFO_CFG1		0x1d
#define SC7U22_REG_FIFO_CFG2		0x1e
#define SC7U22_REG_FIFO_STAT0		0x1f
#define SC7U22_REG_FIFO_DATA		0x21
#define SC7U22_REG_ACC_CONF		0x40
#define SC7U22_REG_ACC_RANGE		0x41
#define SC7U22_REG_GYR_CONF		0x42
#define SC7U22_REG_GYR_RANGE		0x43
#define SC7U22_REG_FIFO_DOWNS		0x45
#define SC7U22_REG_SOFT_RST		0x4a
#define SC7U22_REG_PWR_CTRL		0x7d
#define SC7U22_REG_SEG_SEL		0x7f

#define SC7U22_WHO_AM_I			0x6a
#define SC7U22_SPI_READ			BIT(7)
#define SC7U22_SPI_MAX_HZ		10000000
#define SC7U22_COM_DEFAULT		(BIT(6) | BIT(4))
#define SC7U22_INT1_WTM			BIT(7)

#define SC7U22_FIFO_ACC_EN		BIT(2)
#define SC7U22_FIFO_GYR_EN		BIT(1)
#define SC7U22_FIFO_MODE_BYPASS		0x00
#define SC7U22_FIFO_MODE_STREAM		BIT(5)
#define SC7U22_FIFO_COUNT_HIGH_MASK	GENMASK(3, 0)
#define SC7U22_FIFO_OVERFLOW		BIT(4)
#define SC7U22_FIFO_EMPTY		BIT(6)

#define SC7U22_PWR_ALL			(BIT(3) | BIT(2) | BIT(1))
#define SC7U22_ACC_CONF_1600HZ		0x8c
#define SC7U22_ACC_RANGE_16G		0x03
#define SC7U22_GYR_CONF_1600HZ		0x8c
#define SC7U22_GYR_RANGE_2000DPS	0x00
#define SC7U22_FIFO_DOWNS_FULL_RATE	0x88
#define SC7U22_SOFT_RESET		0xa5

/* FIFO counts 16-bit words; one headerless six-axis frame is six words. */
#define SC7U22_WORDS_PER_FRAME		6
#define SC7U22_BYTES_PER_FRAME		12
#define SC7U22_WATERMARK_FRAMES		2
/* WTM is true only when FIFO_words > FTH, hence 12 words uses FTH=11. */
#define SC7U22_FIFO_FTH			11
#define SC7U22_SAMPLE_PERIOD_NS		625000LL
#define SC7U22_MAX_BURST_FRAMES		32
#define SC7U22_MAX_DRAIN_LOOPS		16
#define SC7U22_WATCHDOG_MS		10
#define SC7U22_REINIT_ERROR_LIMIT	3

enum sc7u22_scan_index {
	SC7U22_SCAN_ACCEL_X,
	SC7U22_SCAN_ACCEL_Y,
	SC7U22_SCAN_ACCEL_Z,
	SC7U22_SCAN_GYRO_X,
	SC7U22_SCAN_GYRO_Y,
	SC7U22_SCAN_GYRO_Z,
	SC7U22_SCAN_TIMESTAMP,
};

struct sc7u22_stats {
	atomic64_t irq_count;
	atomic64_t fifo_reads;
	atomic64_t frames_pushed;
	atomic64_t fifo_overflow;
	atomic64_t fifo_empty_irq;
	atomic64_t fifo_partial_frame;
	atomic64_t fifo_spurious_wtm;
	atomic64_t spi_errors;
	atomic64_t reinitializations;
	atomic64_t nonempty_before_reset;
	atomic64_t max_words_seen;
	atomic64_t max_frames_per_irq;
	atomic64_t drain_loop_limit_hit;
	atomic64_t missed_irq;
	atomic64_t irq_latency_total_ns;
	atomic64_t irq_latency_min_ns;
	atomic64_t irq_latency_max_ns;
	atomic64_t irq_latency_hist[7];
	atomic64_t irq_thread_total_ns;
	atomic64_t irq_thread_min_ns;
	atomic64_t irq_thread_max_ns;
};

struct sc7u22_scan {
	__le16 channels[6];
	s64 timestamp;
};

struct sc7u22_data {
	struct spi_device *spi;
	struct iio_dev *indio_dev;
	/* Serializes all SPI traffic, configuration, and FIFO draining. */
	struct mutex lock;
	struct delayed_work watchdog_work;
	struct regulator_bulk_data supplies[2];
	struct sc7u22_stats stats;
	u8 rx_buf[SC7U22_MAX_BURST_FRAMES * SC7U22_BYTES_PER_FRAME];
	s64 wtm_irq_ts_ns;
	unsigned int max_burst_frames;
	unsigned int consecutive_errors;
	bool buffer_enabled;
};

static void sc7u22_update_max(atomic64_t *value, s64 sample)
{
	s64 old = atomic64_read(value);

	while (sample > old && !atomic64_try_cmpxchg(value, &old, sample))
		;
}

static void sc7u22_update_min(atomic64_t *value, s64 sample)
{
	s64 old = atomic64_read(value);

	while (sample < old && !atomic64_try_cmpxchg(value, &old, sample))
		;
}

static void sc7u22_record_latency_histogram(struct sc7u22_data *st,
					    s64 latency_ns)
{
	static const s64 limits_ns[] = {
		50000, 100000, 200000, 400000, 625000, 1250000,
	};
	unsigned int bucket;

	for (bucket = 0; bucket < ARRAY_SIZE(limits_ns); bucket++)
		if (latency_ns < limits_ns[bucket])
			break;
	atomic64_inc(&st->stats.irq_latency_hist[bucket]);
}

static int sc7u22_write_reg(struct sc7u22_data *st, u8 reg, u8 value)
{
	u8 tx[2] = { reg & ~SC7U22_SPI_READ, value };
	int ret = spi_write(st->spi, tx, sizeof(tx));

	if (ret)
		atomic64_inc(&st->stats.spi_errors);
	return ret;
}

static int sc7u22_read(struct sc7u22_data *st, u8 reg, void *data,
		       unsigned int len)
{
	u8 command = reg | SC7U22_SPI_READ;
	int ret = spi_write_then_read(st->spi, &command, 1, data, len);

	if (ret)
		atomic64_inc(&st->stats.spi_errors);
	return ret;
}

static int sc7u22_read_reg(struct sc7u22_data *st, u8 reg, u8 *value)
{
	return sc7u22_read(st, reg, value, 1);
}

static int sc7u22_fifo_count(struct sc7u22_data *st, u8 *status,
			     unsigned int *words)
{
	u8 value[2];
	int ret;

	ret = sc7u22_read(st, SC7U22_REG_FIFO_STAT0, value, sizeof(value));
	if (ret)
		return ret;
	*status = value[0];
	*words = ((value[0] & SC7U22_FIFO_COUNT_HIGH_MASK) << 8) | value[1];
	sc7u22_update_max(&st->stats.max_words_seen, *words);
	return 0;
}

static int sc7u22_fifo_cycle_mode(struct sc7u22_data *st)
{
	int ret;

	/* Required after every completed FIFO/Stream read by the datasheet. */
	ret = sc7u22_write_reg(st, SC7U22_REG_FIFO_CFG1,
			       SC7U22_FIFO_MODE_BYPASS);
	if (ret)
		return ret;
	return sc7u22_write_reg(st, SC7U22_REG_FIFO_CFG1,
				SC7U22_FIFO_MODE_STREAM |
				((SC7U22_FIFO_FTH >> 8) & 0x07));
}

static int sc7u22_soft_reset(struct sc7u22_data *st)
{
	int ret;

	ret = sc7u22_write_reg(st, SC7U22_REG_SEG_SEL, 0);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_COM_CONF, SC7U22_COM_DEFAULT);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_SOFT_RST, SC7U22_SOFT_RESET);
	if (ret)
		return ret;
	usleep_range(1000, 1500);
	ret = sc7u22_write_reg(st, SC7U22_REG_SOFT_RST, SC7U22_SOFT_RESET);
	if (ret)
		return ret;
	msleep(200);
	ret = sc7u22_write_reg(st, SC7U22_REG_SEG_SEL, 0);
	if (ret)
		return ret;
	return sc7u22_write_reg(st, SC7U22_REG_COM_CONF, SC7U22_COM_DEFAULT);
}

static int sc7u22_stop_locked(struct sc7u22_data *st)
{
	int ret, error = 0;

	ret = sc7u22_write_reg(st, SC7U22_REG_INT1_OUT_SEL1, 0);
	if (ret)
		error = ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_INT1_OUT_SEL2, 0);
	if (ret && !error)
		error = ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_FIFO_CFG1,
			       SC7U22_FIFO_MODE_BYPASS);
	if (ret && !error)
		error = ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_FIFO_CFG0, 0);
	if (ret && !error)
		error = ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_PWR_CTRL, 0);
	if (ret && !error)
		error = ret;
	return error;
}

static int sc7u22_start_locked(struct sc7u22_data *st)
{
	static const struct {
		u8 reg;
		u8 value;
	} verify[] = {
		{ SC7U22_REG_COM_CONF, SC7U22_COM_DEFAULT },
		{ SC7U22_REG_ACC_CONF, SC7U22_ACC_CONF_1600HZ },
		{ SC7U22_REG_ACC_RANGE, SC7U22_ACC_RANGE_16G },
		{ SC7U22_REG_GYR_CONF, SC7U22_GYR_CONF_1600HZ },
		{ SC7U22_REG_GYR_RANGE, SC7U22_GYR_RANGE_2000DPS },
		{ SC7U22_REG_FIFO_DOWNS, SC7U22_FIFO_DOWNS_FULL_RATE },
		{ SC7U22_REG_FIFO_CFG0, SC7U22_FIFO_ACC_EN |
					SC7U22_FIFO_GYR_EN },
		{ SC7U22_REG_FIFO_CFG1, SC7U22_FIFO_MODE_STREAM },
		{ SC7U22_REG_FIFO_CFG2, SC7U22_FIFO_FTH },
		{ SC7U22_REG_INT1_OUT_SEL1, SC7U22_INT1_WTM },
	};
	u8 status;
	u8 value;
	unsigned int words;
	unsigned int i;
	int ret;

	ret = sc7u22_write_reg(st, SC7U22_REG_INT1_OUT_SEL1, 0);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_INT1_OUT_SEL2, 0);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_PWR_CTRL, 0);
	if (ret)
		return ret;
	usleep_range(1000, 1500);

	ret = sc7u22_write_reg(st, SC7U22_REG_ACC_RANGE,
			       SC7U22_ACC_RANGE_16G);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_GYR_RANGE,
			       SC7U22_GYR_RANGE_2000DPS);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_ACC_CONF,
			       SC7U22_ACC_CONF_1600HZ);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_GYR_CONF,
			       SC7U22_GYR_CONF_1600HZ);
	if (ret)
		return ret;
	usleep_range(2000, 2500);

	ret = sc7u22_write_reg(st, SC7U22_REG_PWR_CTRL, SC7U22_PWR_ALL);
	if (ret)
		return ret;
	msleep(60);
	ret = sc7u22_write_reg(st, SC7U22_REG_FIFO_DOWNS,
			       SC7U22_FIFO_DOWNS_FULL_RATE);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_FIFO_CFG0,
			       SC7U22_FIFO_ACC_EN | SC7U22_FIFO_GYR_EN);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_FIFO_CFG2,
			       SC7U22_FIFO_FTH & 0xff);
	if (ret)
		return ret;
	ret = sc7u22_fifo_cycle_mode(st);
	if (ret)
		return ret;
	ret = sc7u22_fifo_count(st, &status, &words);
	if (ret)
		return ret;
	ret = sc7u22_write_reg(st, SC7U22_REG_INT1_OUT_SEL1,
			       SC7U22_INT1_WTM);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(verify); i++) {
		ret = sc7u22_read_reg(st, verify[i].reg, &value);
		if (ret)
			return ret;
		if (value != verify[i].value) {
			dev_err(&st->spi->dev,
				"register 0x%02x read 0x%02x, expected 0x%02x\n",
				verify[i].reg, value, verify[i].value);
			return -EIO;
		}
	}

	return 0;
}

static void sc7u22_push_frames(struct sc7u22_data *st, unsigned int frames,
			       s64 first_timestamp, unsigned int frame_offset)
{
	struct sc7u22_scan scan = { };
	unsigned int i, axis;

	for (i = 0; i < frames; i++) {
		const u8 *frame = st->rx_buf + i * SC7U22_BYTES_PER_FRAME;
		s64 timestamp = first_timestamp +
			(frame_offset + i) * SC7U22_SAMPLE_PERIOD_NS;

		for (axis = 0; axis < ARRAY_SIZE(scan.channels); axis++) {
			u16 sample = get_unaligned_be16(frame + axis * 2);

			scan.channels[axis] = cpu_to_le16(sample);
		}
		iio_push_to_buffers_with_timestamp(st->indio_dev, &scan, timestamp);
	}
	atomic64_add(frames, &st->stats.frames_pushed);
}

static int sc7u22_reinitialize_locked(struct sc7u22_data *st)
{
	int ret;

	atomic64_inc(&st->stats.reinitializations);
	ret = sc7u22_soft_reset(st);
	if (!ret)
		ret = sc7u22_start_locked(st);
	if (!ret)
		st->consecutive_errors = 0;
	return ret;
}

static void sc7u22_handle_io_error_locked(struct sc7u22_data *st)
{
	st->consecutive_errors++;
	if (st->consecutive_errors >= SC7U22_REINIT_ERROR_LIMIT)
		sc7u22_reinitialize_locked(st);
	else
		sc7u22_fifo_cycle_mode(st);
}

static int sc7u22_drain_fifo_locked(struct sc7u22_data *st, s64 anchor_ts,
				    bool from_irq)
{
	unsigned int initial_frames, frame_offset = 0;
	unsigned int frames, words, loops;
	s64 first_timestamp;
	u8 status;
	int ret, partial_retries = 0;
	bool read_any = false;

	ret = sc7u22_fifo_count(st, &status, &words);
	if (ret)
		goto io_error;
	if (status & SC7U22_FIFO_OVERFLOW) {
		atomic64_inc(&st->stats.fifo_overflow);
		return sc7u22_fifo_cycle_mode(st);
	}
	if (from_irq && words < SC7U22_WATERMARK_FRAMES *
				SC7U22_WORDS_PER_FRAME) {
		if (!words || (status & SC7U22_FIFO_EMPTY))
			atomic64_inc(&st->stats.fifo_empty_irq);
		atomic64_inc(&st->stats.fifo_spurious_wtm);
		return 0;
	}

	initial_frames = words / SC7U22_WORDS_PER_FRAME;
	if (!initial_frames)
		return 0;
	if (from_irq)
		first_timestamp = anchor_ts - SC7U22_SAMPLE_PERIOD_NS;
	else
		first_timestamp = anchor_ts -
			(initial_frames - 1) * SC7U22_SAMPLE_PERIOD_NS;

	for (loops = 0; loops < SC7U22_MAX_DRAIN_LOOPS; loops++) {
		ret = sc7u22_fifo_count(st, &status, &words);
		if (ret)
			goto io_error;
		if (status & SC7U22_FIFO_OVERFLOW) {
			atomic64_inc(&st->stats.fifo_overflow);
			break;
		}
		frames = words / SC7U22_WORDS_PER_FRAME;
		if (!frames) {
			if (words && partial_retries++ < 2) {
				usleep_range(40, 80);
				continue;
			}
			if (words)
				atomic64_inc(&st->stats.fifo_partial_frame);
			break;
		}
		frames = min(frames, st->max_burst_frames);
		ret = sc7u22_read(st, SC7U22_REG_FIFO_DATA, st->rx_buf,
				  frames * SC7U22_BYTES_PER_FRAME);
		if (ret)
			goto io_error;
		read_any = true;
		atomic64_inc(&st->stats.fifo_reads);
		sc7u22_push_frames(st, frames, first_timestamp, frame_offset);
		frame_offset += frames;
	}
	if (loops == SC7U22_MAX_DRAIN_LOOPS)
		atomic64_inc(&st->stats.drain_loop_limit_hit);
	sc7u22_update_max(&st->stats.max_frames_per_irq, frame_offset);

	ret = sc7u22_fifo_count(st, &status, &words);
	if (ret)
		goto io_error;
	if (words)
		atomic64_inc(&st->stats.nonempty_before_reset);
	if (read_any || words || (status & SC7U22_FIFO_OVERFLOW)) {
		ret = sc7u22_fifo_cycle_mode(st);
		if (ret)
			goto io_error;
	}
	st->consecutive_errors = 0;
	return 0;

io_error:
	sc7u22_handle_io_error_locked(st);
	return ret;
}

static irqreturn_t sc7u22_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct sc7u22_data *st = iio_priv(indio_dev);

	WRITE_ONCE(st->wtm_irq_ts_ns, iio_get_time_ns(indio_dev));
	atomic64_inc(&st->stats.irq_count);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t sc7u22_irq_thread(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct sc7u22_data *st = iio_priv(indio_dev);
	s64 irq_ts = READ_ONCE(st->wtm_irq_ts_ns);
	s64 start = iio_get_time_ns(indio_dev);
	s64 latency = start - irq_ts;
	s64 duration;

	atomic64_add(latency, &st->stats.irq_latency_total_ns);
	sc7u22_update_min(&st->stats.irq_latency_min_ns, latency);
	sc7u22_update_max(&st->stats.irq_latency_max_ns, latency);
	sc7u22_record_latency_histogram(st, latency);
	mutex_lock(&st->lock);
	if (st->buffer_enabled)
		sc7u22_drain_fifo_locked(st, irq_ts, true);
	mutex_unlock(&st->lock);
	duration = iio_get_time_ns(indio_dev) - start;
	atomic64_add(duration, &st->stats.irq_thread_total_ns);
	sc7u22_update_min(&st->stats.irq_thread_min_ns, duration);
	sc7u22_update_max(&st->stats.irq_thread_max_ns, duration);
	return IRQ_HANDLED;
}

static void sc7u22_watchdog_work(struct work_struct *work)
{
	struct sc7u22_data *st = container_of(to_delayed_work(work),
						    struct sc7u22_data,
						    watchdog_work);
	u8 status;
	s64 now;
	unsigned int words;
	int ret;

	mutex_lock(&st->lock);
	if (!st->buffer_enabled)
		goto unlock;
	ret = sc7u22_fifo_count(st, &status, &words);
	if (ret) {
		sc7u22_handle_io_error_locked(st);
	} else {
		now = iio_get_time_ns(st->indio_dev);
		/*
		 * Do not race a normal IRQ whose thread is waiting for this mutex.
		 * A genuinely missed edge leaves FIFO above WTM without any recent
		 * hard-IRQ timestamp; one watchdog period is far below FIFO capacity.
		 */
		if ((status & SC7U22_FIFO_OVERFLOW) ||
		    (words >= SC7U22_WATERMARK_FRAMES *
			      SC7U22_WORDS_PER_FRAME &&
		     now - READ_ONCE(st->wtm_irq_ts_ns) >=
			      SC7U22_WATCHDOG_MS * NSEC_PER_MSEC)) {
			atomic64_inc(&st->stats.missed_irq);
			sc7u22_drain_fifo_locked(st, now, false);
		}
	}
	if (st->buffer_enabled)
		schedule_delayed_work(&st->watchdog_work,
				      msecs_to_jiffies(SC7U22_WATCHDOG_MS));
unlock:
	mutex_unlock(&st->lock);
}

static int sc7u22_buffer_postenable(struct iio_dev *indio_dev)
{
	struct sc7u22_data *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = sc7u22_start_locked(st);
	if (!ret) {
		st->buffer_enabled = true;
		enable_irq(st->spi->irq);
		schedule_delayed_work(&st->watchdog_work,
				      msecs_to_jiffies(SC7U22_WATCHDOG_MS));
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int sc7u22_buffer_predisable(struct iio_dev *indio_dev)
{
	struct sc7u22_data *st = iio_priv(indio_dev);

	WRITE_ONCE(st->buffer_enabled, false);
	disable_irq(st->spi->irq);
	cancel_delayed_work_sync(&st->watchdog_work);
	mutex_lock(&st->lock);
	sc7u22_stop_locked(st);
	mutex_unlock(&st->lock);
	return 0;
}

static const struct iio_buffer_setup_ops sc7u22_buffer_ops = {
	.postenable = sc7u22_buffer_postenable,
	.predisable = sc7u22_buffer_predisable,
};

static int sc7u22_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *channel,
			    int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 1600;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		if (channel->type == IIO_ACCEL)
			*val2 = 4788403; /* 16 g * 9.80665 / 32768 */
		else if (channel->type == IIO_ANGL_VEL)
			*val2 = 1065264; /* rad/s per LSB at 2000 dps */
		else
			return -EINVAL;
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return -EINVAL;
	}
}

static ssize_t diagnostics_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sc7u22_data *st = iio_priv(dev_to_iio_dev(dev));
	s64 irqs = atomic64_read(&st->stats.irq_count);

	return sysfs_emit(buf,
		"irq_count=%lld\nfifo_reads=%lld\nframes_pushed=%lld\n"
		"fifo_overflow=%lld\nfifo_empty_irq=%lld\n"
		"fifo_partial_frame=%lld\nfifo_spurious_wtm=%lld\n"
		"spi_errors=%lld\nreinitializations=%lld\n"
		"nonempty_before_reset=%lld\nmax_words_seen=%lld\n"
		"max_frames_per_irq=%lld\ndrain_loop_limit_hit=%lld\n"
		"missed_irq=%lld\nirq_latency_min_ns=%lld\n"
		"irq_latency_avg_ns=%lld\nirq_latency_max_ns=%lld\n"
		"irq_latency_hist_lt50us=%lld\n"
		"irq_latency_hist_50_100us=%lld\n"
		"irq_latency_hist_100_200us=%lld\n"
		"irq_latency_hist_200_400us=%lld\n"
		"irq_latency_hist_400_625us=%lld\n"
		"irq_latency_hist_625_1250us=%lld\n"
		"irq_latency_hist_ge1250us=%lld\n"
		"irq_thread_min_ns=%lld\nirq_thread_avg_ns=%lld\n"
		"irq_thread_max_ns=%lld\n",
		irqs, atomic64_read(&st->stats.fifo_reads),
		atomic64_read(&st->stats.frames_pushed),
		atomic64_read(&st->stats.fifo_overflow),
		atomic64_read(&st->stats.fifo_empty_irq),
		atomic64_read(&st->stats.fifo_partial_frame),
		atomic64_read(&st->stats.fifo_spurious_wtm),
		atomic64_read(&st->stats.spi_errors),
		atomic64_read(&st->stats.reinitializations),
		atomic64_read(&st->stats.nonempty_before_reset),
		atomic64_read(&st->stats.max_words_seen),
		atomic64_read(&st->stats.max_frames_per_irq),
		atomic64_read(&st->stats.drain_loop_limit_hit),
		atomic64_read(&st->stats.missed_irq),
		irqs ? atomic64_read(&st->stats.irq_latency_min_ns) : 0,
		irqs ? div64_s64(atomic64_read(&st->stats.irq_latency_total_ns),
				     irqs) : 0,
		atomic64_read(&st->stats.irq_latency_max_ns),
		atomic64_read(&st->stats.irq_latency_hist[0]),
		atomic64_read(&st->stats.irq_latency_hist[1]),
		atomic64_read(&st->stats.irq_latency_hist[2]),
		atomic64_read(&st->stats.irq_latency_hist[3]),
		atomic64_read(&st->stats.irq_latency_hist[4]),
		atomic64_read(&st->stats.irq_latency_hist[5]),
		atomic64_read(&st->stats.irq_latency_hist[6]),
		irqs ? atomic64_read(&st->stats.irq_thread_min_ns) : 0,
		irqs ? div64_s64(atomic64_read(&st->stats.irq_thread_total_ns),
				     irqs) : 0,
		atomic64_read(&st->stats.irq_thread_max_ns));
}

static IIO_DEVICE_ATTR_RO(diagnostics, 0);

static struct attribute *sc7u22_attributes[] = {
	&iio_dev_attr_diagnostics.dev_attr.attr,
	NULL,
};

static const struct attribute_group sc7u22_attribute_group = {
	.attrs = sc7u22_attributes,
};

static int sc7u22_debugfs_reg_access(struct iio_dev *indio_dev,
				     unsigned int reg, unsigned int writeval,
				      unsigned int *readval)
{
	struct sc7u22_data *st = iio_priv(indio_dev);
	u8 value;
	int ret;

	if (!readval || reg > U8_MAX)
		return -EINVAL;
	mutex_lock(&st->lock);
	ret = sc7u22_read_reg(st, reg, &value);
	mutex_unlock(&st->lock);
	if (!ret)
		*readval = value;
	return ret;
}

static const struct iio_info sc7u22_info = {
	.read_raw = sc7u22_read_raw,
	.attrs = &sc7u22_attribute_group,
	.debugfs_reg_access = sc7u22_debugfs_reg_access,
};

#define SC7U22_CHANNEL(_type, _axis, _index) { \
	.type = (_type), \
	.modified = 1, \
	.channel2 = IIO_MOD_ ## _axis, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | \
				    BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = (_index), \
	.scan_type = { \
		.sign = 's', .realbits = 16, .storagebits = 16, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec sc7u22_channels[] = {
	SC7U22_CHANNEL(IIO_ACCEL, X, SC7U22_SCAN_ACCEL_X),
	SC7U22_CHANNEL(IIO_ACCEL, Y, SC7U22_SCAN_ACCEL_Y),
	SC7U22_CHANNEL(IIO_ACCEL, Z, SC7U22_SCAN_ACCEL_Z),
	SC7U22_CHANNEL(IIO_ANGL_VEL, X, SC7U22_SCAN_GYRO_X),
	SC7U22_CHANNEL(IIO_ANGL_VEL, Y, SC7U22_SCAN_GYRO_Y),
	SC7U22_CHANNEL(IIO_ANGL_VEL, Z, SC7U22_SCAN_GYRO_Z),
	IIO_CHAN_SOFT_TIMESTAMP(SC7U22_SCAN_TIMESTAMP),
};

static const unsigned long sc7u22_scan_masks[] = {
	GENMASK(SC7U22_SCAN_GYRO_Z, SC7U22_SCAN_ACCEL_X),
	0,
};

static void sc7u22_regulators_disable(void *data)
{
	struct sc7u22_data *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->supplies), st->supplies);
}

static int sc7u22_probe(struct spi_device *spi)
{
	struct sc7u22_data *st;
	struct iio_dev *indio_dev;
	size_t max_message, max_transfer;
	u8 whoami = 0;
	int attempt, ret;

	if (spi->irq <= 0)
		return dev_err_probe(&spi->dev, -EINVAL,
				     "FIFO watermark IRQ is required\n");
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	st->spi = spi;
	st->indio_dev = indio_dev;
	max_message = spi_max_message_size(spi);
	max_transfer = spi_max_transfer_size(spi);
	if (max_message > 1)
		max_transfer = min(max_transfer, max_message - 1);
	else
		max_transfer = 0;
	st->max_burst_frames = min_t(size_t, SC7U22_MAX_BURST_FRAMES,
				     max_transfer /
					     SC7U22_BYTES_PER_FRAME);
	if (!st->max_burst_frames)
		return dev_err_probe(&spi->dev, -EINVAL,
				     "SPI controller transfer limit is too small\n");
	atomic64_set(&st->stats.irq_latency_min_ns, S64_MAX);
	atomic64_set(&st->stats.irq_thread_min_ns, S64_MAX);
	mutex_init(&st->lock);
	INIT_DELAYED_WORK(&st->watchdog_work, sc7u22_watchdog_work);
	st->supplies[0].supply = "vdd";
	st->supplies[1].supply = "vddio";

	ret = devm_regulator_bulk_get(&spi->dev, ARRAY_SIZE(st->supplies),
				      st->supplies);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to get regulators\n");
	ret = regulator_bulk_enable(ARRAY_SIZE(st->supplies), st->supplies);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to enable regulators\n");
	ret = devm_add_action_or_reset(&spi->dev, sc7u22_regulators_disable, st);
	if (ret)
		return ret;

	spi->max_speed_hz = min_t(u32, spi->max_speed_hz, SC7U22_SPI_MAX_HZ);
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to setup SPI\n");

	for (attempt = 0; attempt < 5; attempt++) {
		ret = sc7u22_write_reg(st, SC7U22_REG_SEG_SEL, 0);
		if (!ret)
			ret = sc7u22_read_reg(st, SC7U22_REG_WHO_AM_I, &whoami);
		if (!ret && whoami == SC7U22_WHO_AM_I)
			break;
		usleep_range(10000, 11000);
	}
	if (attempt == 5)
		return dev_err_probe(&spi->dev, ret ?: -ENODEV,
				     "unexpected WHO_AM_I 0x%02x\n", whoami);
	ret = sc7u22_soft_reset(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to reset device\n");
	ret = sc7u22_read_reg(st, SC7U22_REG_WHO_AM_I, &whoami);
	if (ret || whoami != SC7U22_WHO_AM_I)
		return dev_err_probe(&spi->dev, ret ?: -ENODEV,
				     "device failed reset, WHO_AM_I 0x%02x\n",
				     whoami);

	indio_dev->name = "sc7u22";
	indio_dev->info = &sc7u22_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = sc7u22_channels;
	indio_dev->num_channels = ARRAY_SIZE(sc7u22_channels);
	indio_dev->available_scan_masks = sc7u22_scan_masks;
	ret = devm_iio_kfifo_buffer_setup(&spi->dev, indio_dev,
					  &sc7u22_buffer_ops);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to setup IIO buffer\n");
	ret = devm_request_threaded_irq(&spi->dev, spi->irq,
					sc7u22_irq_handler, sc7u22_irq_thread,
					 IRQF_ONESHOT | IRQF_NO_AUTOEN,
					 dev_name(&spi->dev), indio_dev);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to request IRQ\n");
	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to register IIO device\n");

	spi_set_drvdata(spi, indio_dev);
	dev_info(&spi->dev, "SC7U22 IIO IMU at %u Hz SPI, IRQ %d\n",
		 spi->max_speed_hz, spi->irq);
	return 0;
}

static const struct of_device_id sc7u22_of_match[] = {
	{ .compatible = "silan,sc7u22" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc7u22_of_match);

static const struct spi_device_id sc7u22_ids[] = {
	{ "sc7u22", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, sc7u22_ids);

static struct spi_driver sc7u22_driver = {
	.driver = {
		.name = "sc7u22",
		.of_match_table = sc7u22_of_match,
	},
	.probe = sc7u22_probe,
	.id_table = sc7u22_ids,
};
module_spi_driver(sc7u22_driver);

MODULE_AUTHOR("ncerzzk <huangcmzzk@gmail.com>");
MODULE_DESCRIPTION("Silan SC7U22 six-axis IIO IMU driver");
MODULE_LICENSE("GPL");
