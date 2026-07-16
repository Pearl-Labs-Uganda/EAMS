# RC Car — Manual Control + Telemetry

4-wheel differential drive (TT motors, one L298N), Pi Zero 2 W, browser control
over a single WebSocket. Display + manual control only. No autonomy.

**Read this first:** two motors are paralleled per L298N channel. Stall current
(~2–3 A/ch) exceeds the L298N's realistic rating — **the software is the thermal
protection**. All guards live in `motors.py`, tuned in `config.py`. Do not raise
`DUTY_CAP` (0.55) until stall current is actually measured (see Open Items).

## Wiring

### Motors / L298N
| L298N | Connects to |
|---|---|
| OUT1/OUT2 | Left pair (2 motors in parallel) |
| OUT3/OUT4 | Right pair (2 motors in parallel) |
| +12V | 3S pack + (direct) |
| GND | Star ground at pack negative |
| +5V (logic) | LM2596 5 V rail — **onboard 5 V jumper REMOVED** |
| ENA / ENB | GPIO 12 / 13 |
| IN1 IN2 IN3 IN4 | GPIO 5, 6, 16, 26 |

Star ground at the pack negative. Motor return current must **not** pass
through the Pi's ground pins.

### Sensors (BCM pins)
| Sensor | Pin(s) | Divider? |
|---|---|---|
| US front TRIG/ECHO | 23 / 24 | **ECHO: yes, 5 V→3.3 V** |
| US rear TRIG/ECHO | 27 / 22 | **ECHO: yes, 5 V→3.3 V** |
| IR 0–5 | 4, 17, 25, 20, 21, 7 | **Assumed 5 V modules → yes, all six** (unconfirmed — see Open Items) |
| MPU6050 SDA/SCL | 2 / 3 | No (3.3 V I²C) |

Divider values: **1 kΩ (top) / 2 kΩ (bottom)** gives 5 V → 3.33 V. Any ratio
near 1:2 with total ≥ 2 kΩ is fine.

## Deploy

**On the Pi, once:**
```
sudo apt update
sudo apt install -y pigpio python3-pigpio python3-flask python3-venv
sudo systemctl enable --now pigpiod
sudo raspi-config nonint do_i2c 0      # enable I2C for the MPU6050
```
Verify IMU: `i2cdetect -y 1` → expect `0x68`.
`pigpiod` must run with **default flags** (no `-t 0`).

**Transfer from dev machine** (do `ssh-copy-id pi@raspberrypi.local` first):
```
rsync -avz --delete ./rccar/ pi@raspberrypi.local:/home/pi/rccar/
```
(`raspberrypi.local` needs mDNS; fall back to the IP.)

**Python deps:**
```
ssh pi@raspberrypi.local
cd ~/rccar
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```
`--system-site-packages` matters — it lets the venv see apt's `pigpio`.

**Run manually (always do this first):**
```
cd ~/rccar && .venv/bin/python server.py
```
Wheels off the ground. Open `http://<pi-ip>:8080`.

**Test with no hardware** (laptop): `python3 server.py --dry-run`

**Install as a service (only after manual run works):**
```
sudo cp ~/rccar/rccar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rccar
journalctl -u rccar -f
```

## First-run checklist
1. Car on blocks, wheels free.
2. Confirm L298N 5 V jumper is **removed**.
3. Confirm dividers present on both ECHO pins.
4. Logic rail only (motor rail disconnected) — web app loads, telemetry streams.
5. Connect motor rail. Verify deadman: close the browser tab → wheels stop within 300 ms.
6. Verify direction mapping before it ever touches the floor.

## Networking
`NETWORK_MODE` in `config.py`. Default `"lan"` (Pi joins your WiFi).
For AP mode: `sudo apt install hostapd dnsmasq`, copy `setup/hostapd.conf` to
`/etc/hostapd/`, `setup/dnsmasq.conf` to `/etc/dnsmasq.d/`, give wlan0 a static
192.168.4.1, enable both services, set `NETWORK_MODE = "ap"`. Left **off** by default.

If telemetry stutters over WiFi, set `TELEMETRY_HZ = 10` in `config.py`
**before** touching the 20 Hz control rate.

## Open items — verify, don't assume
1. **Stall current is unmeasured.** `DUTY_CAP = 0.55` is an estimate. Measure:
   hold one wheel, motor on 12 V, ammeter inline. If per-motor stall > ~1.5 A,
   lower the cap. It's one obvious constant in `config.py`.
2. **L298N CSA/CSB** may be grounded on your clone. If exposed, an overcurrent
   auto-cut can be added later — `motors.py`'s loop is structured so a
   current-sense check drops in beside the stall check.
3. **IR logic voltage unconfirmed.** Wiring table assumes 5 V (dividers on all
   six). If they're 3.3 V modules, dividers can be omitted.
4. **Flyback diodes** on the L298N clone: assumed populated; visually check.
5. **LM2596 clone rating:** expected load < 1 A (Pi ~600 mA peak + sensors),
   probably fine — measure anyway.
6. **3.3 V GPIO → L298N inputs** is marginal (V_IH ≈ 2.3 V). Usually works. If
   direction control is erratic, suspect this before the code.
