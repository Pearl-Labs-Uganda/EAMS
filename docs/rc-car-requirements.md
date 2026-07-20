# RC Car — Build Requirements

**Purpose:** Hand this document to a fresh Claude thread as the complete brief. It should produce a working codebase from this alone.

**Scope:** Manual remote control of a 4-wheel differential-drive car + live sensor telemetry to a browser app served from the Pi. Display only — no autonomy, no mapping, no fusion.

---

## 1. Hardware Inventory (fixed — do not propose changes)

| Item | Qty | Notes |
|---|---|---|
| TT gear motor TGP01D-A130 (12215-48) | 4 | Operating range 3–12 V |
| L298N dual H-bridge (red board, with heatsink) | 1 | **Only one. This is a hard constraint.** |
| US-100 ultrasonic sensor | 2 | Trigger/echo mode |
| IR proximity sensor (LM393-type) | 6 | All six required |
| Raspberry Pi Zero 2 W | 1 | BCM2710A1 |
| MPU6050 IMU | 1 | I²C |
| 18650 cells, 3S3P | 9 | 12.6 V full, ~11.1 V nominal |
| LM2596 buck converter | 1 | 5 V rail for Pi + sensors + L298N logic |

### Drivetrain topology
- **Differential steering.** No servo, no steering linkage.
- Left two motors wired in **parallel** → L298N **Channel A**.
- Right two motors wired in **parallel** → L298N **Channel B**.

### Power topology
- Motor rail: L298N `+12V` taps the 3S pack **directly**.
- Logic rail: LM2596 → 5 V → Pi, all sensors, and L298N `+5V` logic pin.
- **L298N onboard 5 V regulator jumper is REMOVED.** The 78M05 is not used and its output must never touch the Pi.
- **Star ground** at the pack negative terminal. Motor return current must not be daisy-chained through the Pi's ground.
- Pi and L298N share a logic reference via the common 5 V rail ground.

---

## 2. Critical Design Constraint: Thermal Headroom

This is the single most important thing in this document. **Read it before writing any motor code.**

Two TT motors are paralleled on each L298N channel. Estimated stall current is **~2–3 A per channel**. The L298N is rated 2 A/channel under optimistic datasheet conditions, and drops ~2 V across the bridge. At stall this is roughly **5 W dissipated per channel** — beyond what the stock heatsink handles. Thermal shutdown is expected, not hypothetical. Small cars stall constantly: wall contact, carpet, direction reversal.

The build has one L298N by decision. Therefore **the software is the protection**. Every item in §5 (Safety Layer) is a hard requirement, not a nice-to-have. Do not omit or weaken any of them.

---

## 3. GPIO Assignment (BCM numbering)

| Function | GPIO | Direction | Notes |
|---|---|---|---|
| ENA (left channel PWM) | 12 | out | Hardware-PWM capable |
| ENB (right channel PWM) | 13 | out | Hardware-PWM capable |
| IN1 (left) | 5 | out | |
| IN2 (left) | 6 | out | |
| IN3 (right) | 16 | out | |
| IN4 (right) | 26 | out | |
| US-100 #1 TRIG (front) | 23 | out | |
| US-100 #1 ECHO (front) | 24 | in | **Needs 5 V→3.3 V divider** |
| US-100 #2 TRIG (rear) | 27 | out | |
| US-100 #2 ECHO (rear) | 22 | in | **Needs 5 V→3.3 V divider** |
| IR 0 | 4 | in | Divider if module runs at 5 V |
| IR 1 | 17 | in | " |
| IR 2 | 25 | in | " |
| IR 3 | 20 | in | " |
| IR 4 | 21 | in | " |
| IR 5 | 7 | in | " |
| MPU6050 SDA | 2 | I²C | |
| MPU6050 SCL | 3 | I²C | |

18 pins used. GPIO 14/15 (UART) left free. Separate TRIG per ultrasonic is required — they must fire alternately, not together.

All pin numbers live in a single `config.py`. No magic numbers anywhere else.

---

## 4. Software Architecture

**Language:** Python 3 on the Pi. Vanilla HTML/CSS/JS on the client (no build step, no npm, no framework).

**GPIO library:** `pigpio`, via the `pigpiod` daemon.
- Reason: DMA-timed, 5 µs tick, independent of the CPU scheduler. `RPi.GPIO` PWM stutters under load and its echo timing loops are unusably noisy.
- Use **pigpio software PWM** on ENA/ENB. Avoids the `dtparam=audio=off` / `dtoverlay=pwm-2chan` config dance entirely. Pins 12/13 are still assigned in case hardware PWM is wanted later.
- Start `pigpiod` with **default flags**. Do **not** pass `-t 0` — that switches its clock source from PCM to PWM.
- Use `pi.callback()` edge callbacks with pigpio's hardware timestamps for US-100 echo timing. Never `time.time()` polling loops.

**Transport:** a single WebSocket carries commands down and telemetry up. HTTP serves the static page only. No polling. No REST endpoints for control or telemetry.

**Server:** Flask + `flask-sock`. One process, three threads:

1. **Web/socket thread** — serves `static/`, owns the WebSocket.
2. **Sensor thread** — samples all sensors on a fixed tick, writes into a shared latest-value dict under a lock. Must never block the socket thread.
3. **Motor thread** — owns the pigpio handle, runs the safety layer at 50 Hz, is the *only* thing that touches motor pins.

The socket thread never writes GPIO and never reads sensors directly. It reads the shared dict and posts commands to the motor thread.

---

## 5. Safety Layer (mandatory, all of it)

Implemented **inside the motor thread**, server-side. Not in the browser. The client must be incapable of bypassing any of this.

| Guard | Spec |
|---|---|
| **Duty cap** | Hard clamp at **55 %** of full PWM on both channels. Applied last, after all other math. Single named constant in `config.py`. |
| **Slew rate limit** | Ramp actual duty toward commanded duty over **~200 ms**. Never step. Instant reversal on a paralleled channel is the worst-case current spike. |
| **Zero crossing coast** | When a channel's command crosses sign, force **80 ms of coast** (both IN pins low, PWM 0) before applying the new direction. No snapping through zero. |
| **Deadman** | No command packet received for **300 ms** → both channels to zero. Non-negotiable over WiFi. |
| **Stall timeout** | If commanded duty > 0 on either channel and the MPU6050 shows no meaningful motion for **1 s** → cut both channels to zero, latch a `stall` flag, require the client to send an explicit `reset` before accepting new commands. |
| **Startup state** | Motors are zeroed before the socket ever opens. Every exit path (exception, SIGTERM, socket close) zeroes them. Register an atexit/signal handler. |

**PWM frequency:** 1–5 kHz on ENA/ENB. Pick 2 kHz.

---

## 6. Control Protocol

**Client → Pi**, sent at **20 Hz**, JSON over the WebSocket:

```
{ "type": "cmd", "left": <float -1.0..1.0>, "right": <float -1.0..1.0>, "seq": <int> }
{ "type": "reset" }
```

- Joystick → differential mixing is done **in the browser**. The Pi receives per-side values only.
- `left`/`right` are clamped to [-1, 1] on arrival. Reject NaN, non-numeric, missing fields — drop the packet, do not crash.
- Sign convention: positive = forward.
- `seq` monotonic; Pi ignores out-of-order packets.
- The client sends at 20 Hz continuously, including zeros when idle. Silence is how the deadman fires.

---

## 7. Telemetry

**Pi → Client**, **one merged frame at 20 Hz**. Not one message per sensor.

```
{
  "t": <float, monotonic seconds>,
  "imu": { "ax":<int>, "ay":<int>, "az":<int>, "gx":<int>, "gy":<int>, "gz":<int> },
  "us": { "front": <int|null>, "rear": <int|null> },
  "ir": <int bitmask, bits 0..5>,
  "motor": { "left": <float>, "right": <float>, "capped": <bool>, "stall": <bool> },
  "dead": <bool>
}
```

Sampling rates inside the sensor thread:
- **IMU:** 20 Hz. **Raw register counts, unscaled.** No filtering on the Pi. Display-only means the browser does scaling.
- **Ultrasonics:** **strictly alternate** — fire front on even frames, rear on odd. ~10 Hz each. Firing both together causes cross-talk and garbage readings. Report `null` on timeout rather than a stale or bogus number; the client shows "—".
- **IR:** every frame, packed into a single 6-bit integer.

The Pi ships raw. The browser scales, formats, and renders. The Zero 2 W is weak — keep it dumb.

---

## 8. Client Web App

Static, served from `static/`. Single page. No framework, no bundler.

- **Input:** on-screen touch joystick (canvas or pointer events) **and** Gamepad API when a pad is connected. Both feed the same mixer.
- **Mixer:** standard arcade mix — `left = y + x`, `right = y - x`, then normalize so neither exceeds 1.0.
- **Display:**
  - Two distance readouts / bars (front, rear), showing "—" when null.
  - Six IR indicator dots, decoded from the bitmask.
  - IMU: scale raw counts to g and °/s in JS, show as numeric readouts plus a simple tilt indicator.
  - Left/right duty bars showing what the Pi is *actually* applying (from telemetry, not from local command).
  - Visible **STALL** and **DEADMAN** banners driven by those flags, with a reset button that sends `{"type":"reset"}`.
- **Staleness:** if `now - t > 500 ms`, gray out all telemetry and show a connection warning.
- **Reconnect:** auto-reconnect the WebSocket with backoff.
- Layout must work on a phone in landscape. That's the actual use case.

---

## 9. Networking

Two supported modes, selected by a flag in `config.py`:
- **LAN mode** (default for development): Pi joins existing WiFi, app on `http://<pi-ip>:8080`.
- **AP mode**: `hostapd` + `dnsmasq`, Pi serves its own SSID. Use this if the car roams out of range.

Provide the hostapd/dnsmasq config files and setup steps, but leave AP mode **off** by default.

Note: Zero 2 W WiFi gets flaky under sustained load, especially in AP mode. If telemetry stutters, drop telemetry to 10 Hz **before** reducing the 20 Hz control rate.

---

## 10. Deliverables — File Layout

Generate this tree locally first. Do not attempt to SSH or deploy; produce the files, and the human transfers them.

```
rccar/
├── config.py           # ALL pins, constants, tunables, network mode
├── motors.py           # motor thread + safety layer; only file touching motor pins
├── sensors.py          # sensor thread: US-100, IR, MPU6050
├── server.py           # Flask + flask-sock, thread orchestration, entry point
├── requirements.txt
├── rccar.service       # systemd unit
├── static/
│   ├── index.html
│   ├── app.js
│   └── style.css
├── setup/
│   ├── hostapd.conf    # AP mode, unused by default
│   └── dnsmasq.conf
└── README.md           # wiring table, divider values, first-run checklist
```

Additional requirements on the code:
- `config.py` is the only place constants live.
- Every safety constant from §5 is a named, commented constant.
- Log to stdout (systemd captures it). Log deadman trips and stall cuts at WARNING.
- Include a `--dry-run` flag on `server.py` that runs the whole stack with GPIO stubbed, so the web app can be tested on a laptop with no hardware.

---

## 11. Deployment Instructions to Include in README.md

Write these out concretely, with the exact commands.

**On the Pi, once:**
```
sudo apt update
sudo apt install -y pigpio python3-pigpio python3-flask python3-venv
sudo systemctl enable --now pigpiod
sudo raspi-config nonint do_i2c 0      # enable I2C for the MPU6050
```
Verify the IMU is visible: `i2cdetect -y 1` → expect `0x68`.

**Transfer from the dev machine:**
```
rsync -avz --delete ./rccar/ pi@raspberrypi.local:/home/pi/rccar/
```
(Note `raspberrypi.local` requires mDNS; fall back to the IP. Recommend `ssh-copy-id` first so rsync isn't prompting for a password every iteration.)

**Python deps:**
```
ssh pi@raspberrypi.local
cd ~/rccar
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```
`--system-site-packages` matters — it lets the venv see the apt-installed `pigpio`.

**Run manually (do this first, always):**
```
cd ~/rccar && .venv/bin/python server.py
```
Wheels off the ground. Open `http://<pi-ip>:8080`.

**Install as a service (only after manual run works):**
```
sudo cp ~/rccar/rccar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rccar
journalctl -u rccar -f
```

The systemd unit must declare `After=pigpiod.service` and `Requires=pigpiod.service`, run as user `pi`, `Restart=on-failure`, `WorkingDirectory=/home/pi/rccar`.

**First-run checklist for the README:**
1. Car on blocks, wheels free.
2. Confirm L298N 5 V jumper is **removed**.
3. Confirm dividers present on both ECHO pins.
4. Power logic rail only (motor rail disconnected) — verify the web app loads and telemetry streams.
5. Connect motor rail. Verify deadman: close the browser tab, wheels must stop within 300 ms.
6. Verify direction mapping before ever putting it on the floor.

---

## 12. Open Items — Flag These, Don't Guess

The builder should surface these rather than silently assuming:

1. **Stall current is unmeasured.** The 55 % duty cap is an estimate. Real number: hold one wheel, motor on 12 V, ammeter inline. If per-motor stall exceeds ~1.5 A, the cap must come down. **Leave the cap as a single obvious constant and say so in the README.**
2. **L298N current sense (CSA/CSB) exposure is unknown.** Many red-board clones tie them to ground. If the board exposes them, an overcurrent auto-cut is worth adding later — architect the motor thread so a current-sense input could be dropped in.
3. **IR module logic voltage is unconfirmed.** If they run at 5 V, all six outputs need dividers. If 3.3 V, they don't. The README wiring table must state the assumption explicitly. Assume 5 V until told otherwise.
4. **Flyback diodes on the L298N clone** are assumed populated (most red boards are). Worth a visual check.
5. **LM2596 modules are commonly relabeled clones**; the 3 A rating assumes a heatsink they don't have. Pi Zero 2 W peaks ~600 mA plus 8 sensors — likely under 1 A, so probably fine, but measure.
6. **3.3 V → L298N logic inputs** are marginal (L298 V_IH ≈ 2.3 V). Usually works; has been known to fail on clones. Sharing the logic ground reference helps. If direction control behaves erratically, suspect this before suspecting the code.

---

## 13. Explicit Non-Goals

Do not build: autonomy, obstacle avoidance, path planning, sensor fusion, odometry, video streaming, data logging, a database, user accounts, or a JS framework. Display and manual control only.
