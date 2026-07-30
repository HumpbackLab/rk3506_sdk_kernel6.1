# WS281x Web Control

Web-controlled WS2812/SK6812 effects using the Adafruit NeoPixel sequence API.
The userspace adapter encodes the LED protocol into duty durations and submits
them to the generic PowerFin `/dev/pwm-wave-N` interface.

## Install

Install the Adafruit package and its dependencies:

```sh
python3 -m pip install --no-deps -r requirements.txt
```

The Buildroot image includes Python 3 but currently does not include pip. The
dependencies can also be installed into a staging directory on the build host
and copied into the target root filesystem:

```sh
python3 -m pip install --no-deps --target ./vendor -r requirements.txt
```

`--no-deps` is intentional: the adapter uses the official NeoPixel and
PixelBuf sequence implementation but replaces its pin writer, so the
architecture-specific Adafruit Blinka GPIO dependencies are unnecessary.

Then copy this directory, including `vendor`, to the target. `app.py`
automatically loads the bundled modules from `vendor`.

## Run

WS2812 RGB, eight pixels:

```sh
python3 app.py --device /dev/pwm-wave-0 --protocol ws2812 --count 8
```

SK6812 RGBW, twelve pixels:

```sh
python3 app.py --device /dev/pwm-wave-0 --protocol sk6812-rgbw \
  --count 12 --order GRBW
```

Open `http://<board-ip>:8080/`. The service listens on all interfaces by
default and has no authentication, so it should only be exposed on a trusted
network.
