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
.venv/bin/pip install -r rccar_requirements.txt
```

## Pin Numbering

`config.py` uses Jetson.GPIO `BOARD` numbering. These are physical 40-pin
header positions, not Raspberry Pi BCM GPIO numbers.

| Function | BOARD pin |
|---|---:|
| ENA left PWM | 15 |
| ENB right PWM | 33 |
| IN1 left | 18 |
| IN2 left | 22 |
| IN3 right | 24 |
| IN4 right | 26 |
| US front TRIG/ECHO | 32 / 11 |
| US rear TRIG/ECHO | 16 / 13 |
| IR 0..5 | 29, 31, 36, 37, 12, 38 |
| MPU6050 SDA/SCL | 3 / 5 |

Ultrasonic ECHO and any 5 V IR outputs must be level shifted to 3.3 V before
they touch the Jetson header.

## PWM

BOARD pins **15 and 33** must be configured as PWM outputs with Jetson-IO /
pinmux tooling for your JetPack image:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

Reboot afterwards. Two things to get right in that tool:

- **Leave PWM DISABLED for pin 32.** It offers PWM on this board, but it carries
  `US_FRONT_TRIG`. A pin muxed to the PWM controller ignores GPIO writes, and the
  failure is silent — the front ultrasonic simply reads nothing.
- **Leave SPI disabled.** Pins 24 and 26 are SPI chip-selects by default and now
  carry IN3/IN4. If SPI claims them, the direction writes are swallowed.

Verify with `pwm_bench.py` (motor battery disconnected — it holds IN1–IN4 low so
the H-bridge outputs stay off):

```bash
cd ~/rccar && .venv/bin/python pwm_bench.py
# in a second terminal:
sudo cat /sys/kernel/debug/pwm     # both chips enabled, nonzero duty
```

If PWM is unavailable or unstable on the header, use an external PCA9685 PWM
board and adapt `hardware.py` for it.

## IMU

Fitted and working as of 3 Aug 2026. The MPU6050 appears at `0x68` on **bus 7**:

```bash
i2cdetect -y 7
```

**Bus 7, not bus 1.** On the Orin Nano the 40-pin header's SDA/SCL (physical
pins 3/5) is `/dev/i2c-7`. Bus 1 is the Raspberry Pi / Jetson Nano number and is
repeated by most tutorials; it is wrong for this board. `config.I2C_BUS = 7`.
The failure mode is deceptive: opening the wrong bus succeeds and only the first
register write fails, so it presents as a dead sensor rather than a config error.
`ls /dev/i2c-*` lists the buses that actually exist.

The service user must be in the `i2c` group (see Install above) or
`/dev/i2c-7` is unreadable and the IMU appears absent for a different reason.

`STALL_GUARD_ENABLED` is now `True` in `config.py`, restoring stall protection.
If the IMU stops responding, the car will stall-cut after 1 s because it cannot
prove motion — that is the guard working, not a bug. Note that the guard reads
CHASSIS rotation from the gyro, so on blocks it may stall-cut even with the
wheels spinning normally; validate the threshold on the floor.

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

Edit `rccar.service` if your username is not `jetson`, or if the repo is not at
`/home/jetson/rccar`. The unit runs as an unprivileged user, so that user must
be in the `gpio` and `i2c` groups (see Install above) — this is the usual reason
a server that works under `sudo` fails under systemd.

**Do not `enable` the service until wheels-off-ground testing has passed.**
`enable` makes it start at boot; until direction mapping is verified, an
unattended server owning the motor pins after a power cut is not what you want.

Install and test, without enabling at boot:

```bash
sudo cp ~/rccar/rccar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start rccar
journalctl -u rccar -f
```

Once you trust it, add boot-time start:

```bash
sudo systemctl enable rccar
```

Useful afterwards:

```bash
sudo systemctl status rccar
sudo systemctl restart rccar
sudo systemctl stop rccar
sudo systemctl disable rccar     # stop starting at boot
```

### What `Restart=on-failure` means for this car

The deadman is a loop *inside* the Python process. If the process dies, the
deadman dies with it, but the L298N does not know that and the PWM pins hold
their last duty. With `RestartSec=2` that is up to two seconds of uncommanded
driving before the replacement process starts and runs `_all_stop()`.

This is not an argument for removing the restart — an unattended car is better
off with a server that comes back. It is an argument for keeping the car on
blocks until the service has proven stable, and for knowing what you are looking
at if it ever happens.

### AP mode

If `NETWORK_MODE = "ap"`, the server should come up after the access point
exists. Add to `[Unit]`:

```ini
After=network-online.target hostapd.service
Wants=network-online.target
```

## First Hardware Test

1. Put the car on blocks.
2. Confirm the L298N 5 V regulator jumper is removed.
3. Confirm all 5 V sensor outputs are level shifted.
4. Confirm `i2cdetect -y 7` sees the MPU6050 at `0x68`.
5. Confirm PWM appears on BOARD pins 15 and 33 (`pwm_bench.py`, battery off).
6. Take control in the browser and command a small movement.
7. Close the browser tab and confirm the deadman stops the wheels within 300 ms.
8. Verify left/right and forward/reverse mapping before floor driving.