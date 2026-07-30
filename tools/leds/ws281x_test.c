// SPDX-License-Identifier: GPL-2.0-only
/*
 * Userspace WS2812/SK6812 encoder for /dev/pwm-wave-N.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/pwm-wave.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define WS281X_PERIOD_NS	1250U
#define WS281X_RESET_NS		300000U
#define WS281X_CLOCK_HZ		50000000U

struct ws281x_protocol {
	uint32_t t0h_ns;
	uint32_t t1h_ns;
	unsigned int channels;
};

static int parse_protocol(const char *name, struct ws281x_protocol *protocol)
{
	if (!strcmp(name, "ws2812")) {
		protocol->t0h_ns = 400;
		protocol->t1h_ns = 800;
		protocol->channels = 3;
	} else if (!strcmp(name, "sk6812-rgb")) {
		protocol->t0h_ns = 300;
		protocol->t1h_ns = 600;
		protocol->channels = 3;
	} else if (!strcmp(name, "sk6812-rgbw")) {
		protocol->t0h_ns = 300;
		protocol->t1h_ns = 600;
		protocol->channels = 4;
	} else {
		return -1;
	}

	return 0;
}

static void encode_byte(uint32_t *wave, unsigned int *offset, uint8_t value,
			const struct ws281x_protocol *protocol)
{
	int bit;

	for (bit = 7; bit >= 0; bit--)
		wave[(*offset)++] = value & (1U << bit) ?
				    protocol->t1h_ns : protocol->t0h_ns;
}

int main(int argc, char **argv)
{
	static const unsigned int order[] = { 1, 0, 2, 3 };
	struct pwm_wave_config config = {
		.period_ns = WS281X_PERIOD_NS,
		.clock_rate_hz = WS281X_CLOCK_HZ,
	};
	struct ws281x_protocol protocol;
	struct pwm_wave_caps caps;
	unsigned int reset_entries;
	unsigned int wave_entries;
	unsigned int max_pixels;
	unsigned int offset = 0;
	unsigned long count;
	uint32_t *wave;
	char *end;
	ssize_t size;
	unsigned int i;
	int fd;
	int ret = EXIT_FAILURE;

	if (argc != 4) {
		fprintf(stderr, "usage: %s /dev/pwm-wave-N ", argv[0]);
		fprintf(stderr,
			"<ws2812|sk6812-rgb|sk6812-rgbw> <led-count>\n");
		return EXIT_FAILURE;
	}

	count = strtoul(argv[3], &end, 0);
	if (*end || !count || count > UINT32_MAX ||
	    parse_protocol(argv[2], &protocol)) {
		fprintf(stderr, "invalid protocol or LED count\n");
		return EXIT_FAILURE;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("open");
		return EXIT_FAILURE;
	}

	if (ioctl(fd, PWM_WAVE_IOC_GET_CAPS, &caps) < 0) {
		perror("PWM_WAVE_IOC_GET_CAPS");
		goto out_close;
	}
	if (caps.abi_version != PWM_WAVE_ABI_VERSION) {
		fprintf(stderr, "unsupported PWM wave ABI %u\n",
			caps.abi_version);
		goto out_close;
	}

	reset_entries = (WS281X_RESET_NS + WS281X_PERIOD_NS - 1) /
			WS281X_PERIOD_NS;
	max_pixels = (caps.max_entries_8bit - reset_entries) /
		     (protocol.channels * 8);
	if (count > max_pixels) {
		fprintf(stderr, "LED count exceeds PWM table capacity %u\n",
			max_pixels);
		goto out_close;
	}

	wave_entries = count * protocol.channels * 8 + reset_entries;
	wave = calloc(wave_entries, sizeof(*wave));
	if (!wave) {
		perror("calloc");
		goto out_close;
	}

	for (i = 0; i < count; i++) {
		uint32_t pixel = 0;
		uint8_t colors[4];
		unsigned int channel;

		if (i == 0)
			pixel = 0x00ff0000;
		else if (i == 1)
			pixel = 0x0000ff00;
		else if (i == 2)
			pixel = 0x000000ff;
		else if (i == 3 && protocol.channels == 4)
			pixel = 0xff000000;

		colors[0] = (pixel >> 16) & 0xff;
		colors[1] = (pixel >> 8) & 0xff;
		colors[2] = pixel & 0xff;
		colors[3] = (pixel >> 24) & 0xff;

		for (channel = 0; channel < protocol.channels; channel++)
			encode_byte(wave, &offset, colors[order[channel]],
				    &protocol);
	}

	if (ioctl(fd, PWM_WAVE_IOC_SET_CONFIG, &config) < 0) {
		perror("PWM_WAVE_IOC_SET_CONFIG");
		goto out_free;
	}

	size = wave_entries * sizeof(*wave);
	if (write(fd, wave, size) != size) {
		if (errno)
			perror("write");
		else
			fprintf(stderr, "short PWM wave write\n");
		goto out_free;
	}

	printf("ABI %u, submitted %u entries for %lu pixels (capacity %u)\n",
	       caps.abi_version, wave_entries, count, max_pixels);
	ret = EXIT_SUCCESS;

out_free:
	free(wave);
out_close:
	close(fd);
	return ret;
}
