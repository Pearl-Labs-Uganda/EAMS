# RC Car - Jetson Orin Nano Control + Telemetry

4-wheel differential-drive RC car using a Jetson Orin Nano, one L298N, browser
manual control, and live telemetry over a WebSocket.

## Important Safety Note

Two TT motors are paralleled per L298N channel. Stall current can exceed the
L298N's realistic thermal limits, so the server-side motor safety layer is not
optional: duty cap, slew limit, zero-cross coast, deadman, and stall cutoff all
live in `motors.py` and are tuned in `config.py`.

Keep the car on blocks until the pin map, PWM output, direction mapping,
deadman, and stall guard have all been verified.

## Jetson Dependencies

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip i2c-tools
sudo usermod -aG gpio,i2c $USER
```

Log out and back in after adding groups.

Create the app environment:

```bash
cd ~/rccar
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## PWM Setup

`config.py` uses BOARD pins 33 and 32 for L298N `ENA` and `ENB`. These must be
configured as PWM-capable header pins on the Jetson before motor speed control
will work. Use NVIDIA Jetson-IO / pinmux tooling for your JetPack image.

If you cannot get stable PWM on those pins, use an external PCA9685 PWM board
instead of driving L298N enable pins directly from Jetson.GPIO.

## Wiring

Pin numbers below are Jetson.GPIO `BOARD` numbers: physical 40-pin header
positions.

| Function | Jetson BOARD pin | Direction | Notes |
|---|---:|---|---|
| ENA left PWM | 33 | out/PWM | Jetson pinmux must enable PWM |
| ENB right PWM | 32 | out/PWM | Jetson pinmux must enable PWM |
| IN1 left | 22 | out | L298N direction |
| IN2 left | 18 | out | L298N direction |
| IN3 right | 24 | out | L298N direction |
| IN4 right | 13 | out | L298N direction |
| US front TRIG | 16 | out | |
| US front ECHO | 26 | in | 5 V to 3.3 V divider required |
| US rear TRIG | 11 | out | |
| US rear ECHO | 15 | in | 5 V to 3.3 V divider required |
| IR 0..5 | 29,31,36,35,37,40 | in | Dividers required if modules output 5 V |
| MPU6050 SDA/SCL | 3/5 | I2C | Expect `0x68` on bus 1 |

## Run

Laptop dry-run:

```bash
python server.py --dry-run
```

On Jetson:

```bash
cd ~/rccar
.venv/bin/python server.py
```

Open `http://<jetson-ip>:8080`.

## First-Run Checklist

1. Put the car on blocks with wheels free.
2. Confirm L298N 5 V regulator jumper is removed.
3. Confirm all 5 V sensor outputs are level shifted before reaching the Jetson.
4. Confirm `i2cdetect -y 1` shows the MPU6050 at `0x68`.
5. Verify PWM appears on BOARD pins 33 and 32.
6. Verify the deadman: take control, command motion, close the tab, and confirm wheels stop within 300 ms.
7. Verify direction mapping before the car touches the floor.

## systemd

Edit `rccar.service` if your Jetson username is not `jetson`, then:

```bash
sudo cp ~/rccar/rccar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rccar
journalctl -u rccar -f
```
