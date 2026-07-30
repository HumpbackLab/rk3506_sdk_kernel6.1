# SPDX-License-Identifier: GPL-2.0
# Makefile for LEDs tools

CFLAGS = -Wall -Wextra -g -D__EXPORTED_HEADERS__ \
	 -I../../include/uapi -I../../include

all: uledmon led_hw_brightness_mon ws281x_test
%: %.c
	$(CC) $(CFLAGS) -o $@ $^

clean:
	$(RM) uledmon led_hw_brightness_mon ws281x_test

.PHONY: all clean
