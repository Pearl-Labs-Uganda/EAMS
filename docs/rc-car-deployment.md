# RC Car - Jetson Orin Nano Deployment

This deployment guide is for the Jetson Orin Nano port of the RC car server.
The old Raspberry Pi/pigpio deployment path no longer applies to the runtime
code in this repository.

## Runtime Architecture

The app still uses the same high-level design:

1. Flask + WebSocket server serves `static/` and accepts control commands.
2. `SensorThread` samples ultrasonics, IR inputs, and MPU6050 telemetry.
3. `MotorThread` is the only code that touches motor pins and owns the safety
   layer: deadman, stall latch, slew limiting, zero-cross coast, and duty cap.

The hardware backend is now `hardware.py`, using `Jetson.GPIO` and `smbus2`.

## Install

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip i2c-tools
sudo usermod -aG gpio,i2c $USER
```

Log out and back in after changing groups.

```bash
cd ~/rccar
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Pin Numbering

`config.py` uses Jetson.GPIO `BOARD` numbering. These are physical 40-pin
header positions, not Raspberry Pi BCM GPIO numbers.

| Function | BOARD pin |
|---|---:|
| ENA left PWM | 33 |
| ENB right PWM | 32 |
| IN1 left | 22 |
| IN2 left | 18 |
| IN3 right | 24 |
| IN4 right | 13 |
| US front TRIG/ECHO | 16 / 26 |
| US rear TRIG/ECHO | 11 / 15 |
| IR 0..5 | 29, 31, 36, 35, 37, 40 |
| MPU6050 SDA/SCL | 3 / 5 |

Ultrasonic ECHO and any 5 V IR outputs must be level shifted to 3.3 V before
they touch the Jetson header.

## PWM

BOARD pins 33 and 32 must be configured as PWM outputs with Jetson-IO / pinmux
tooling for your JetPack image. If PWM is unavailable or unstable on the header,
use an external PCA9685 PWM board and adapt `hardware.py` for it.

## IMU

The MPU6050 should appear at `0x68`:

```bash
i2cdetect -y 1
```

`STALL_GUARD_ENABLED` is `True` by default. If the IMU is absent, the car will
stall-cut because it cannot prove motion. Only disable the stall guard for
wheels-off-ground tests.

## Run Manually

```bash
cd ~/rccar
.venv/bin/python server.py
```

Open:

```text
http://<jetson-ip>:8080
```

Dry-run without hardware:

```bash
python server.py --dry-run
```

## systemd

Edit `rccar.service` if your username is not `jetson`.

```bash
sudo cp ~/rccar/rccar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rccar
journalctl -u rccar -f
```

## First Hardware Test

1. Put the car on blocks.
2. Confirm the L298N 5 V regulator jumper is removed.
3. Confirm all 5 V sensor outputs are level shifted.
4. Confirm I2C sees the MPU6050 at `0x68`.
5. Confirm PWM appears on BOARD pins 33 and 32.
6. Take control in the browser and command a small movement.
7. Close the browser tab and confirm the deadman stops the wheels within 300 ms.
8. Verify left/right and forward/reverse mapping before floor driving.
