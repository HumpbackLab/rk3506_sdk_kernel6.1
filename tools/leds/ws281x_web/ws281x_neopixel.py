# SPDX-License-Identifier: MIT
"""Adafruit NeoPixel backend for the Linux /dev/pwm-wave-N ABI."""

import fcntl
import os
import struct
import sys
import types

import adafruit_pixelbuf

# NeoPixel imports Blinka's pin modules even though this backend bypasses all
# pin handling. Provide the tiny import surface it needs, which also avoids
# copying architecture-specific Blinka extensions into the target rootfs.
if "board" not in sys.modules:
    sys.modules["board"] = types.ModuleType("board")
if "microcontroller" not in sys.modules:
    microcontroller_stub = types.ModuleType("microcontroller")
    microcontroller_stub.Pin = object
    sys.modules["microcontroller"] = microcontroller_stub
if "digitalio" not in sys.modules:
    digitalio_stub = types.ModuleType("digitalio")
    digitalio_stub.DigitalInOut = object
    digitalio_stub.Direction = types.SimpleNamespace(OUTPUT=1)
    sys.modules["digitalio"] = digitalio_stub

# Adafruit's default neopixel_write backend rejects unknown Linux boards while
# importing. This backend never calls it because _transmit() is overridden.
if "neopixel_write" not in sys.modules:
    neopixel_write_stub = types.ModuleType("neopixel_write")
    neopixel_write_stub.neopixel_write = lambda _pin, _buffer: None
    sys.modules["neopixel_write"] = neopixel_write_stub

import neopixel


WS281X_PERIOD_NS = 1250
WS281X_RESET_NS = 300000
WS281X_CLOCK_HZ = 50000000
PWM_WAVE_ABI_VERSION = 1

_IOC_WRITE = 1
_IOC_READ = 2


def _ioc(direction, ioctl_type, number, size):
    return (
        (direction << 30)
        | (size << 16)
        | (ord(ioctl_type) << 8)
        | number
    )


PWM_WAVE_IOC_GET_CAPS = _ioc(_IOC_READ, "P", 0x00, 32)
PWM_WAVE_IOC_SET_CONFIG = _ioc(_IOC_WRITE, "P", 0x01, 40)

PROTOCOLS = {
    "ws2812": (400, 800, "GRB"),
    "sk6812-rgb": (300, 600, "GRB"),
    "sk6812-rgbw": (300, 600, "GRBW"),
}


class WS281xNeoPixel(neopixel.NeoPixel):
    """NeoPixel sequence whose ``show()`` submits a userspace-encoded wave."""

    def __init__(
        self,
        device,
        n,
        *,
        protocol="ws2812",
        pixel_order=None,
        brightness=1.0,
        auto_write=False,
    ):
        if protocol not in PROTOCOLS:
            raise ValueError(f"unsupported protocol: {protocol}")

        t0h_ns, t1h_ns, default_order = PROTOCOLS[protocol]
        pixel_order = (pixel_order or default_order).upper()
        channels = 4 if protocol == "sk6812-rgbw" else 3

        if len(pixel_order) != channels or set(pixel_order) != set("RGBW"[:channels]):
            expected = "RGBW" if channels == 4 else "RGB"
            raise ValueError(f"pixel_order must be a permutation of {expected}")

        self._device_path = device
        self._protocol = protocol
        self._wire_order = pixel_order
        self._t0h_ns = t0h_ns
        self._t1h_ns = t1h_ns
        self._reset_entries = (
            WS281X_RESET_NS + WS281X_PERIOD_NS - 1
        ) // WS281X_PERIOD_NS
        self.pixel_channels = channels
        self._fd = os.open(device, os.O_RDWR | os.O_CLOEXEC)

        try:
            caps_buffer = bytearray(32)
            fcntl.ioctl(self._fd, PWM_WAVE_IOC_GET_CAPS, caps_buffer, True)
            abi, max_entries, _, _ = struct.unpack_from("=4I", caps_buffer)
            maximum = (max_entries - self._reset_entries) // (channels * 8)
            if abi != PWM_WAVE_ABI_VERSION:
                raise RuntimeError(f"unsupported PWM wave ABI version {abi}")
            if not 1 <= n <= maximum:
                raise ValueError(f"LED count must be between 1 and {maximum}")

            config = struct.pack(
                "=QQII4I",
                WS281X_PERIOD_NS,
                WS281X_CLOCK_HZ,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            fcntl.ioctl(self._fd, PWM_WAVE_IOC_SET_CONFIG, config)

            # Bypass NeoPixel.__init__ because the kernel owns the physical pin.
            self._power = None
            self.pin = None
            adafruit_pixelbuf.PixelBuf.__init__(
                self,
                n,
                brightness=brightness,
                byteorder=pixel_order,
                auto_write=auto_write,
            )
        except Exception:
            os.close(self._fd)
            self._fd = -1
            raise

    def _transmit(self, buffer):
        wave = []

        for value in buffer:
            for bit in range(7, -1, -1):
                wave.append(
                    self._t1h_ns if value & (1 << bit) else self._t0h_ns
                )
        wave.extend([0] * self._reset_entries)

        frame = struct.pack(f"={len(wave)}I", *wave)
        written = os.write(self._fd, frame)
        if written != len(frame):
            raise OSError(f"short PWM wave write: {written}/{len(frame)}")

    def deinit(self):
        if getattr(self, "_fd", -1) < 0:
            return

        try:
            self.fill(0)
            self.show()
        finally:
            os.close(self._fd)
            self._fd = -1
