#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Web-controlled WS281x/SK6812 effects using Adafruit NeoPixel."""

import argparse
import colorsys
import json
import math
import mimetypes
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parent
VENDOR = ROOT / "vendor"
if VENDOR.is_dir():
    sys.path.insert(0, str(VENDOR))

try:
    from ws281x_neopixel import PROTOCOLS, WS281xNeoPixel
except ImportError as error:
    raise SystemExit(
        "Adafruit NeoPixel is not installed. Run: "
        "python3 -m pip install --no-deps -r requirements.txt"
    ) from error

def parse_color(value):
    value = value.lstrip("#")
    if len(value) != 6:
        raise ValueError("color must use #RRGGBB format")
    return tuple(int(value[index : index + 2], 16) for index in (0, 2, 4))


class EffectController:
    def __init__(self, device, protocol, count, order, brightness):
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.phase = 0.0
        self.last_time = time.monotonic()
        self.state = {
            "device": device,
            "protocol": protocol,
            "count": count,
            "order": order,
            "effect": "solid",
            "color": "#ff4000",
            "brightness": brightness,
            "speed": 1.0,
        }
        self.pixels = self._new_pixels()
        self.thread = threading.Thread(target=self._animate, daemon=True)
        self.thread.start()

    def _new_pixels(self):
        return WS281xNeoPixel(
            self.state["device"],
            self.state["count"],
            protocol=self.state["protocol"],
            pixel_order=self.state["order"],
            brightness=self.state["brightness"],
            auto_write=False,
        )

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def update(self, changes):
        allowed = {
            "protocol",
            "count",
            "order",
            "effect",
            "color",
            "brightness",
            "speed",
        }
        unknown = set(changes) - allowed
        if unknown:
            raise ValueError(f"unknown settings: {', '.join(sorted(unknown))}")

        with self.lock:
            old_state = dict(self.state)
            old_hardware = (
                self.state["protocol"],
                self.state["count"],
                self.state["order"],
            )

            if "protocol" in changes:
                if changes["protocol"] not in PROTOCOLS:
                    raise ValueError("unsupported protocol")
                self.state["protocol"] = changes["protocol"]
            if "count" in changes:
                self.state["count"] = int(changes["count"])
            if "order" in changes:
                self.state["order"] = str(changes["order"]).upper()
            if "effect" in changes:
                if changes["effect"] not in {
                    "solid",
                    "rainbow",
                    "chase",
                    "breathe",
                    "off",
                }:
                    raise ValueError("unsupported effect")
                self.state["effect"] = changes["effect"]
            if "color" in changes:
                parse_color(changes["color"])
                self.state["color"] = changes["color"].lower()
            if "brightness" in changes:
                brightness = float(changes["brightness"])
                if not 0.0 <= brightness <= 1.0:
                    raise ValueError("brightness must be between 0 and 1")
                self.state["brightness"] = brightness
            if "speed" in changes:
                speed = float(changes["speed"])
                if not 0.05 <= speed <= 10.0:
                    raise ValueError("speed must be between 0.05 and 10")
                self.state["speed"] = speed

            new_hardware = (
                self.state["protocol"],
                self.state["count"],
                self.state["order"],
            )
            if new_hardware != old_hardware:
                self.pixels.deinit()
                try:
                    self.pixels = self._new_pixels()
                except Exception:
                    self.state = old_state
                    self.pixels = self._new_pixels()
                    raise
            else:
                self.pixels.brightness = self.state["brightness"]

            self.phase = 0.0
            return dict(self.state)

    def _render(self, now):
        elapsed = min(now - self.last_time, 0.25)
        self.last_time = now
        self.phase += elapsed * self.state["speed"]

        effect = self.state["effect"]
        color = parse_color(self.state["color"])
        count = len(self.pixels)
        channels = self.pixels.pixel_channels

        if effect == "off":
            self.pixels.fill((0,) * channels)
        elif effect == "solid":
            self.pixels.fill(color + ((0,) if channels == 4 else ()))
        elif effect == "rainbow":
            for index in range(count):
                hue = (index / count + self.phase * 0.15) % 1.0
                rgb = tuple(
                    round(component * 255)
                    for component in colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                )
                self.pixels[index] = rgb + (
                    (0,) if channels == 4 else ()
                )
        elif effect == "chase":
            self.pixels.fill((0,) * channels)
            head = int(self.phase * 8) % count
            for tail in range(min(4, count)):
                scale = 1.0 - tail / 4
                rgb = tuple(round(component * scale) for component in color)
                self.pixels[(head - tail) % count] = rgb + (
                    (0,) if channels == 4 else ()
                )
        elif effect == "breathe":
            scale = (math.sin(self.phase * math.tau) + 1.0) / 2.0
            rgb = tuple(round(component * scale) for component in color)
            self.pixels.fill(rgb + ((0,) if channels == 4 else ()))

        self.pixels.show()

    def _animate(self):
        while not self.stop_event.wait(1 / 30):
            try:
                with self.lock:
                    self._render(time.monotonic())
            except OSError as error:
                print(f"WS281x output error: {error}", flush=True)
                self.stop_event.wait(0.5)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        with self.lock:
            self.pixels.deinit()


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "WS281xWeb/1.0"

    def _json(self, status, payload):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/state":
            self._json(200, self.server.controller.snapshot())
            return

        if path == "/":
            path = "/static/index.html"
        target = (ROOT / path.lstrip("/")).resolve()
        static_root = (ROOT / "static").resolve()
        if static_root not in target.parents or not target.is_file():
            self.send_error(404)
            return

        body = target.read_bytes()
        self.send_response(200)
        self.send_header(
            "Content-Type",
            mimetypes.guess_type(target.name)[0] or "application/octet-stream",
        )
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if urlparse(self.path).path != "/api/state":
            self.send_error(404)
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 65536:
                raise ValueError("invalid request size")
            changes = json.loads(self.rfile.read(length))
            state = self.server.controller.update(changes)
            self._json(200, state)
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self._json(400, {"error": str(error)})
        except OSError as error:
            self._json(500, {"error": str(error)})

    def log_message(self, message, *args):
        print(f"{self.client_address[0]} - {message % args}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/pwm-wave-0")
    parser.add_argument("--protocol", choices=PROTOCOLS, default="ws2812")
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--order", default=None)
    parser.add_argument("--brightness", type=float, default=0.25)
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    default_order = PROTOCOLS[args.protocol][2]
    controller = EffectController(
        args.device,
        args.protocol,
        args.count,
        (args.order or default_order).upper(),
        args.brightness,
    )
    server = ThreadingHTTPServer((args.listen, args.port), RequestHandler)
    server.controller = controller

    def stop_server(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)

    print(f"WS281x web control: http://{args.listen}:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        controller.close()


if __name__ == "__main__":
    main()
