# RC Car — Codebase to Running Server on the Pi

**Date:** 16 July 2026
**Distilled from:** RC Car Project — Progress Report & Replication Guide

---

## 1. Where we were

A written requirements document, and nothing else. No code, no Pi configured.

Target: a 4-wheel differential-drive car controlled from a browser, with live sensor telemetry. Pi Zero 2 W as the brain, one L298N driving four TT motors (two paralleled per channel), everything over WiFi.

The governing constraint: **two motors per L298N channel means stall current (~2–3 A) exceeds the chip's realistic 2 A rating.** There is one L298N by decision. Therefore the software *is* the thermal protection — duty cap, slew limit, zero-cross coast, deadman, and stall cutoff are all mandatory, all server-side, all inside the motor thread. The client must be incapable of bypassing any of them.

## 2. Where we got to

Server running on real hardware. Web app loads, WebSocket streams telemetry, and **the safety layer is verified working** — deadman trips on client silence, stall latches and clears on RESET.

| Component | Status |
|---|---|
| Codebase generated from requirements | ✅ |
| `--dry-run` full-stack test on laptop | ✅ |
| pigpio built from source, daemon running as a service | ✅ |
| Code transferred to `/home/eams-pi/rccar` | ✅ |
| venv + dependencies installed | ✅ |
| I²C enabled | ✅ |
| Server up, web app loads, WebSocket connects | ✅ |
| Deadman / stall latch / reset verified | ✅ |
| MPU6050 responding | ⚠ **not yet** — see §5 |
| systemd unit installed | ⬜ not yet |
| Motors ever powered | ⬜ not yet |

### Platforms
- **Pi:** Raspberry Pi OS Trixie (Debian 13), kernel 6.18, Python 3.13, user `eams-pi`, `172.20.10.2` (phone hotspot).
- **Dev machine:** Linux laptop, `~/Downloads/iso/pi/rccar/`, `172.20.10.3`.

### The three threads (what you're deploying)
1. **Web/socket** — serves `static/` on :8080, owns one WebSocket (commands down, telemetry up).
2. **Sensor** — 20 Hz tick into a shared dict under a lock. Ultrasonics fire *alternately* (front on even ticks, rear on odd) to avoid cross-talk. IMU raw over I²C.
3. **Motor** — 50 Hz loop, the only code touching motor pins. Per tick, in order: deadman → stall cutoff → slew limit → zero-cross coast → **duty cap (55 %, applied last)**.

---

## 3. How to replicate

### 3.1 Problem: pigpio is gone from apt on Trixie

The planned `sudo apt install -y pigpio python3-pigpio` fails:
```
Error: Package 'pigpio' has no installation candidate
```
Trixie (Debian 13) dropped the package. **Fix: build from source.** (Alternative was reflashing Bookworm Legacy.)

```bash
sudo apt install -y python3-venv python3-pip git build-essential
cd /tmp
git clone https://github.com/joan2937/pigpio
cd pigpio
make
sudo make install
```
*Objective:* compile and install `libpigpio.so`, the `pigpiod` daemon, and the `pigs` CLI.

`make install` **will fail at the last step** with:
```
ModuleNotFoundError: No module named 'distutils'
```
**This does not matter.** That step installs the *Python* module via `distutils`, removed in Python 3.12+. The daemon and libraries are already in place; the Python client comes from PyPI in §3.3 instead.

```bash
sudo ldconfig
```
*Objective:* refresh the linker cache so the fresh `.so` files in `/usr/local/lib` are found.

### 3.2 pigpiod as a service

The apt package ships a unit; a source build doesn't. Write one:
```bash
sudo tee /etc/systemd/system/pigpiod.service << 'EOF'
[Unit]
Description=pigpio daemon
After=network.target

[Service]
Type=forking
ExecStart=/usr/local/bin/pigpiod
ExecStop=/bin/systemctl kill pigpiod

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now pigpiod
```
*Objective:* run `pigpiod` at boot with **default flags**. Do **not** add `-t 0` — it switches the clock source from PCM to PWM.

Verify:
```bash
systemctl status pigpiod   # → active (running)
pigs t                     # → a tick count, e.g. 3214257916
```
`pigs t` asks the daemon for its tick. A number back proves it's alive and reachable.

### 3.3 Transfer the code

Two wrong turns worth avoiding:
- `eams-pi@172.20.10.2:/home/pi/rccar/` → **fails**, the user is `eams-pi`, so `/home/pi` doesn't exist.
- Running rsync from the wrong directory → **fails**, `change_dir ".../rccar"`.

Working command, run **from the directory containing `rccar/`**:
```bash
rsync -avz --delete rccar/ eams-pi@172.20.10.2:/home/eams-pi/rccar/
```
*Flags:* `-a` preserve structure/permissions, `-v` verbose, `-z` compress, `--delete` mirror exactly. Trailing `/` on `rccar/` means "the contents of", not "the folder itself". 14 files.

### 3.4 Python environment

```bash
cd ~/rccar
python3 -m venv .venv
.venv/bin/pip install flask flask-sock pigpio
```
*Objective:* isolated env + the three deps. The pigpio **Python client** comes from PyPI (1.78) — this replaces what failed in §3.1.

Note: `--system-site-packages` from the original plan is **not needed** — there's no apt `python3-pigpio` to see.

Installed: flask 3.1.3, flask-sock 0.7.0, pigpio 1.78.

### 3.5 Enable I²C

```bash
sudo raspi-config nonint do_i2c 0     # 0 = on
sudo apt install -y i2c-tools
i2cdetect -y 1                        # expect 0x68
```
*Objective:* bring up the I²C bus for the MPU6050 and confirm the chip answers.

### 3.6 First run — wheels off the ground

```bash
cd ~/rccar && .venv/bin/python server.py
```
Open `http://172.20.10.2:8080` — **`http://`, explicitly.**

---

## 4. What the logs mean

| Log line | Meaning | Verdict |
|---|---|---|
| `Running on http://172.20.10.2:8080` + 200s for `/`, `app.js`, `style.css` | Server up, app served | ✅ |
| `client connected` | WebSocket handshake worked, telemetry streaming | ✅ |
| `DEADMAN tripped: no command for 300 ms` | Client went quiet (tab backgrounded, disconnect) | ✅ **expected — this is the safety net working** |
| `STALL cut … awaiting client reset` / `stall latch cleared by client reset` | Duty commanded, no IMU motion for 1 s → cut and latch. RESET clears it | ⚠ correct behaviour, but firing because the IMU is dead |
| `MPU6050 init failed ('I2C write failed')` | IMU unreachable over I²C | ⚠ **open item** |
| `Bad HTTP/0.9 request type ('\x16\x03\x01…')` spam | A browser tried **https://** against a plain-HTTP server. `\x16\x03\x01` is a TLS handshake | ✅ harmless, ignore |
| `client disconnected — deadman will zero motors` | Clean disconnect path | ✅ |

**Bottom line:** the repeated stall trips are *correct* given a dead IMU. The safety layer refuses to run motors it can't confirm are moving. That's the design working, not a bug.

---

## 5. To do

1. **Fix the IMU.** If `i2cdetect -y 1` doesn't show `0x68`: check MPU6050 wiring — VCC to 3.3 V rail per design, SDA→GPIO2, SCL→GPIO3, common ground. Stall trips stop once it answers.

2. **Install the systemd unit.** `rccar.service` was written for user `pi`; fix three lines first:
   ```ini
   User=eams-pi
   WorkingDirectory=/home/eams-pi/rccar
   ExecStart=/home/eams-pi/rccar/.venv/bin/python server.py
   ```
   Then:
   ```bash
   sudo cp ~/rccar/rccar.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now rccar
   journalctl -u rccar -f
   ```
   Only after the manual run works.

3. **Hardware first-run checklist**, in order:
   1. Car on blocks, wheels free.
   2. Confirm L298N 5 V jumper is **removed**.
   3. Confirm voltage dividers present on both ECHO pins.
   4. Logic rail only (motor rail disconnected) — verify telemetry streams.
   5. Connect motor rail. Verify deadman: close the browser tab, wheels stop within 300 ms.
   6. Verify direction mapping **before** it ever touches the floor.

4. **Measure real stall current.** Hold one wheel, motor on 12 V, ammeter inline. If per-motor stall > ~1.5 A, lower `DUTY_CAP` in `config.py`. It is deliberately the single most obvious constant in the file.

---

## 6. Glossary

- **BCM numbering** — GPIO pins by Broadcom chip number (GPIO 12), not physical header position.
- **Deadman** — If control packets stop arriving, motors stop automatically.
- **Differential drive / arcade mix** — Steering by running left and right wheels at different speeds; arcade mix converts one joystick (x, y) into left/right values.
- **distutils** — Old Python packaging module, removed in 3.12; cause of the `make install` failure.
- **Duty cycle / duty cap** — Fraction of PWM "on" time (= motor power). Capped at 55 % to protect the L298N.
- **H-bridge (L298N)** — Circuit that runs a motor in both directions. The L298N has two.
- **I²C** — Two-wire bus (SDA/SCL) for the MPU6050.
- **IMU** — Accelerometer + gyroscope (MPU6050).
- **ldconfig** — Refreshes the shared-library cache so new `.so` files are found.
- **pigpio / pigpiod / pigs** — GPIO library, its daemon, its CLI. DMA-timed, so PWM and echo timestamps don't depend on CPU load.
- **PWM** — Rapidly switching a pin on/off to control average power.
- **rsync** — Transfers only differences; pushes code to the Pi over SSH.
- **Slew rate limit** — Ramping duty gradually instead of instantly, to avoid current spikes.
- **systemd unit** — Config telling Linux to run a program at boot and restart it on failure.
- **Telemetry** — Sensor/status stream from Pi to browser.
- **TLS handshake (`\x16\x03\x01`)** — First bytes of an HTTPS connection; shows up as garbage 400s when a browser speaks HTTPS to an HTTP server.
- **venv** — Isolated Python environment.
- **WebSocket** — Persistent two-way connection; carries commands down and telemetry up on one channel.
