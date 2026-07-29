# RC Car Project — Progress Report & Replication Guide

**Date:** 16 July 2026 (updated 20 July 2026)
**Goal:** A 4-wheel differential-drive RC car controlled from a browser, with live sensor telemetry. Raspberry Pi Zero 2 W as the brain, one L298N driving four TT motors (two per channel), controlled over WiFi.

This report describes exactly what we have done so far, in order, so anyone can replicate it on the same setup.

**Status (20 July):** The car has been driven under power with this code. The MPU6050 IMU has **not** been installed — driving is done with the stall guard disabled (`STALL_GUARD_ENABLED = False`), which is now the accepted operating configuration, not a temporary state. See §7. The pin assignments in `config.py` are authoritative and supersede the original requirements table.

> **Update (27 July 2026):** §1–§7 below describe the original **Raspberry Pi Zero 2 W / pigpio** build. The project has since **migrated to the NVIDIA Jetson Orin Nano** and added two simulators and a documentation system — see the new **§8** for current state. Where the Pi-era sections conflict with §8, the runtime code, or `project-brief.md`, the later material wins. `config.py` remains authoritative for pins (now Jetson BOARD numbering).

---

## 1. The Setup

### Hardware
| Part | Role |
|---|---|
| Raspberry Pi Zero 2 W | Runs the server, reads sensors, drives the motor board |
| L298N dual H-bridge (×1) | Drives the motors. Left motor pair on channel A, right pair on channel B |
| 4× TT gear motors | Two wired in parallel per L298N channel |
| 2× US-100 ultrasonic | Front and rear distance |
| 6× IR proximity (LM393) | Near-obstacle dots |
| MPU6050 | IMU (accelerometer + gyroscope) over I²C |
| 3S3P 18650 pack (~11.1 V) | Motor power, direct to L298N +12V |
| LM2596 buck converter | 5 V rail for Pi, sensors, and L298N logic |

### Platforms
- **Pi:** Raspberry Pi OS **Trixie** (Debian 13), kernel `6.18`, Python 3.13, user `eams-pi`, IP `172.20.10.2` (phone hotspot network).
- **Dev machine:** Linux laptop, files in `~/Downloads/iso/pi/rccar/`, IP `172.20.10.3`.

### Key design fact
Two motors are paralleled per L298N channel, so stall current (~2–3 A) exceeds the chip's realistic 2 A rating. **The software is the thermal protection.** All safety limits (duty cap, slew limit, deadman, stall cutoff) live server-side in the motor code.

---

## 2. The Codebase (generated first, on the dev machine)

The full project was generated from a written requirements document. File layout:

```
rccar/
├── config.py        # ALL pins and tunable constants (BCM numbering)
├── motors.py        # motor thread + safety layer (only file touching motor pins)
├── sensors.py       # sensor thread: ultrasonics, IR, MPU6050
├── server.py        # Flask + flask-sock server, entry point, --dry-run flag
├── requirements.txt
├── rccar.service    # systemd unit (installed later)
├── static/          # index.html, app.js, style.css — the browser app
└── setup/           # hostapd.conf, dnsmasq.conf — AP mode, unused for now
```

### How the software works (overview)

**Three threads on the Pi:**
1. **Web/socket thread** — serves the static page over HTTP on port 8080 and owns one WebSocket that carries commands down and telemetry up.
2. **Sensor thread** — samples everything on a 20 Hz tick into a shared dictionary (under a lock). Ultrasonics fire *alternately* (front on even ticks, rear on odd) to avoid cross-talk. IMU is read raw over I²C.
3. **Motor thread** — a 50 Hz loop that is the only code allowed to touch motor pins. Every tick it applies, in order:
   - **Deadman:** no command packet for 300 ms → motors to zero.
   - **Stall cutoff:** duty commanded but no gyro motion for 1 s → cut, latch a `stall` flag, require the client to press RESET.
   - **Slew limit:** duty ramps to target over ~200 ms, never steps.
   - **Zero-cross coast:** on direction reversal, force 80 ms of coast first.
   - **Duty cap:** hard clamp at **55 %** PWM, applied last.

**GPIO library:** `pigpio` via the `pigpiod` daemon — chosen because it uses DMA timing (precise PWM and microsecond echo timestamps, independent of CPU load).

**Browser client:** plain HTML/JS, no framework. Touch joystick + Gamepad API → arcade mix (`left = y + x`, `right = y − x`, normalized) → JSON commands at 20 Hz, *including zeros when idle* (silence is how the deadman fires). It renders distance bars, six IR dots, scaled IMU values, actual applied duty, and STALL/DEADMAN banners.

`server.py --dry-run` stubs all GPIO so the whole stack can be tested on a laptop with no hardware. This was done and passed before deploying.

---

## 3. Problem 1: `pigpio` is gone from apt on Trixie

Planned command:
```bash
sudo apt install -y pigpio python3-pigpio python3-flask python3-venv
```
Result:
```
Error: Package 'pigpio' has no installation candidate
```
**Why:** Raspberry Pi OS Trixie (Debian 13) dropped the `pigpio` package.

**Fix chosen:** build pigpio from source. (Alternative was reflashing Bookworm Legacy.)

### 3.1 Install build tools
```bash
sudo apt install -y python3-venv python3-pip git build-essential
```
*Objective:* compiler (`gcc`, `make`), git to fetch source, venv/pip for Python later. All were already present.

### 3.2 Clone and compile
```bash
cd /tmp
git clone https://github.com/joan2937/pigpio
cd pigpio
make
```
*Objective:* download the pigpio source and compile the C library (`libpigpio.so`), the daemon (`pigpiod`), and the command tool (`pigs`). Compiled clean.

### 3.3 Install
```bash
sudo make install
```
*Result:* installed the libraries to `/usr/local/lib` and binaries (`pigpiod`, `pigs`) to `/usr/local/bin`, **then failed** at the last step:
```
ModuleNotFoundError: No module named 'distutils'
```
**Why:** the Makefile's final step installs the *Python* pigpio module using `distutils`, which was removed in Python 3.12+. **This failure did not matter** — everything the daemon needs was already installed, and we install the Python module from PyPI inside the venv instead (step 5).

### 3.4 Register the libraries
```bash
sudo ldconfig
```
*Objective:* refresh the dynamic-linker cache so the system can find the freshly installed `.so` files in `/usr/local/lib`.

### 3.5 Create a systemd service for the daemon
The apt package normally ships `pigpiod.service`; building from source doesn't, so we wrote one:
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
*Objective:* run `pigpiod` at boot with **default flags** (important — no `-t 0`). `daemon-reload` makes systemd re-read unit files; `enable --now` starts it immediately and at every boot.

### 3.6 Verify
```bash
systemctl status pigpiod   # → active (running)
pigs t                     # → 3214257916 (a tick count = daemon answering)
```
`pigs t` asks the daemon for its current tick; getting a number back proves the daemon is alive and reachable.

---

## 4. Problem 2: transferring the code (two wrong paths)

### Attempt 1 — wrong remote user
```bash
rsync -avz --delete ./rccar/ eams-pi@172.20.10.2:/home/pi/rccar/
# mkdir "/home/pi/rccar" failed: No such file or directory
```
**Why:** the Pi's user is `eams-pi`, not `pi`, so `/home/pi` doesn't exist.

### Attempt 2 — wrong local path
```bash
rsync -avz --delete ./rccar/ eams-pi@172.20.10.2:/home/eams-pi/rccar/
# change_dir ".../rccar" failed — no rccar/ in the current directory
```
**Why:** the shell was not in the folder containing `rccar/`.

### Working command
Run from the directory that actually contains `rccar/`:
```bash
rsync -avz --delete rccar/ eams-pi@172.20.10.2:/home/eams-pi/rccar/
```
All 14 files transferred.
*Flags:* `-a` preserve structure/permissions, `-v` verbose, `-z` compress, `--delete` mirror exactly (remove remote files not present locally). The trailing `/` on `rccar/` means "copy the *contents* of this folder".

**Follow-up needed:** `rccar.service` was written for user `pi`; before installing it, change three lines:
```ini
User=eams-pi
WorkingDirectory=/home/eams-pi/rccar
ExecStart=/home/eams-pi/rccar/.venv/bin/python server.py
```

---

## 5. Python environment on the Pi

```bash
cd ~/rccar
python3 -m venv .venv
.venv/bin/pip install flask flask-sock pigpio
```
*Objective:* create an isolated Python environment and install the three dependencies. Because apt's `python3-pigpio` doesn't exist on Trixie, the pigpio **Python client** comes from PyPI (`pigpio 1.78`) — this replaces the step that failed in 3.3. Note: `--system-site-packages` from the original plan is **not needed** anymore.

One transient network hiccup during download (`Retrying...`) — pip retried and succeeded. Installed: flask 3.1.3, flask-sock 0.7.0, pigpio 1.78 + dependencies.

---

## 6. Enable I²C and first run

```bash
sudo raspi-config nonint do_i2c 0
```
*Objective:* enable the I²C bus for the MPU6050 (non-interactive; `0` means "on").

```bash
i2cdetect -y 1
# -bash: i2cdetect: command not found
```
**Open item:** `i2c-tools` isn't installed. Fix: `sudo apt install -y i2c-tools`, then re-run `i2cdetect -y 1` and expect the IMU at address `0x68`.

### First run (wheels off the ground)
```bash
.venv/bin/python server.py
```
Results, in order of appearance:

| Log line | Meaning | Status |
|---|---|---|
| `MPU6050 init failed ('I2C write failed')` | The IMU is not installed; init fails and the sensor thread ships zeros for the IMU | ℹ expected — IMU deferred, see §7 |
| `Running on http://172.20.10.2:8080` | Server up; browser at that URL loaded the app (200s for `/`, `app.js`, `style.css`) | ✅ |
| `client connected` | WebSocket handshake worked; telemetry streaming | ✅ |
| `DEADMAN tripped: no command for 300 ms` | Fires whenever the client goes quiet (tab in background, disconnect). This is the safety net working | ✅ expected |
| `STALL cut ... awaiting client reset` / `stall latch cleared by client reset` | Seen on the *first* run, when the stall guard was still enabled: with no IMU the code sees "duty commanded but no motion" after 1 s and cuts. RESET button works. The guard was subsequently disabled (`STALL_GUARD_ENABLED = False`) so the car could be driven; these lines no longer appear | ⚠ historical — guard now off, see §7 |
| `Bad HTTP/0.9 request type ('\x16\x03\x01...')` spam | A browser tried **https://** against our plain-HTTP server; `\x16\x03\x01` is a TLS handshake. Harmless — use `http://` explicitly | ✅ ignore |
| `client disconnected — deadman will zero motors` | Clean disconnect path works | ✅ |

**Bottom line:** server, web app, WebSocket, deadman, stall latch, and reset all verified working on real hardware. The stall trips seen here were *correct behavior given an absent IMU* — the safety layer refused to run motors it couldn't confirm were moving. Because the IMU is not installed, the stall guard was then disabled so the car could actually be driven (see §7); with the guard off, the stall path no longer fires and the deadman + duty cap + slew/coast limits are the remaining protections.

---

## 7. Current state & next steps

Done:
- [x] Codebase generated and dry-run tested on laptop
- [x] pigpio built from source, daemon running as a service, verified with `pigs t`
- [x] Code transferred to `/home/eams-pi/rccar`
- [x] venv + dependencies installed
- [x] I²C enabled
- [x] Server runs; web app loads; WebSocket, deadman, stall/reset verified
- [x] First-run hardware checklist completed; motors powered
- [x] **Car driven under power** with this code — direction mapping and control confirmed on the actual vehicle

Deferred by decision:
- **MPU6050 IMU not installed.** The car is driven with `STALL_GUARD_ENABLED = False`, so the stall/thermal cutoff is inactive. This is a deliberate, accepted operating state — **not** a bug to chase. If/when the IMU is fitted (VCC to 3.3 V rail per design, SDA→GPIO2, SCL→GPIO3, common ground; `sudo apt install -y i2c-tools` then `i2cdetect -y 1` expecting `0x68`), set `STALL_GUARD_ENABLED = True` to restore stall protection.

⚠ **Known accepted risk:** the design premise is "the software is the thermal protection" for the paralleled-motor L298N, and the stall cutoff was one of those protections. With the guard off, the remaining protections are the deadman, duty cap (55 %), slew limit, and zero-cross coast. A prolonged stall (wall contact, carpet, jam) is no longer auto-cut — it relies on the duty cap alone to stay within the L298N's thermal envelope. Keep runs short and watch for heatsink temperature until either the IMU is fitted or a non-IMU stall detector is added.

To do:
1. Install the systemd unit. Edit the 3 user paths in `rccar.service` (§4), then:
   ```bash
   sudo cp ~/rccar/rccar.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now rccar
   journalctl -u rccar -f
   ```
2. Measure real motor stall current; adjust `DUTY_CAP` in `config.py` if per-motor stall > ~1.5 A. This matters more now, since the duty cap is the primary thermal guard while the IMU is absent.
3. Optional: add a non-IMU stall/overcurrent detector (e.g. L298N current-sense) so stall protection doesn't depend on fitting the IMU.

---

## 8. Jetson migration, simulators & documentation (27 July 2026)

**Context:** everything above documents the **Raspberry Pi Zero 2 W / pigpio**
build. Since then the project moved to the **NVIDIA Jetson Orin Nano** and we
added two simulators plus a documentation system. This entry catches the logbook
up. From here, `config.py` (Jetson BOARD numbering) is authoritative for pins and
`project-brief.md` is the standing orientation doc.

### 8.1 Platform migration: Pi Zero 2 W → Jetson Orin Nano
- **Why:** the Jetson Orin Nano becomes the primary onboard computer for
  perception/planning/control — the compute the autonomy goal needs, which the
  Pi Zero couldn't provide.
- **GPIO layer:** replaced `pigpio` with **`Jetson.GPIO` (BOARD numbering)** +
  `smbus2`. `hardware.py` adapts the old pigpio-shaped surface (`set_mode`,
  `write`, `read`, PWM, I²C) onto Jetson.GPIO so **`motors.py` / `sensors.py` and
  the whole safety layer were reused unchanged**.
- **Pins changed** from the Pi BCM assignments to Jetson BOARD pins: ENA 32,
  ENB 33, IN1 11, IN2 13, IN3 15, IN4 16, US front 18/22, US rear 24/26, IR
  29/31/36/37/12/38, MPU6050 on **I²C bus 1 @ `0x68`** (physical header pins 3/5).
  **`config.py` is the source of truth**; the old requirements/README pin tables
  (BCM) are superseded.
- **Deps:** `requirements.txt` now pins `Jetson.GPIO` + `smbus2` (was pigpio).
  No `pigpiod` daemon on Jetson, so `rccar.service` no longer declares
  `After/Requires=pigpiod.service`.
- **Docs:** `README.md` and `rc-car-deployment.md` were rewritten for Jetson
  (user `jetson`, `/home/jetson/rccar`, `jetson-io` PWM setup). **Open Jetson
  risk:** PWM on BOARD pins 32/33 must be enabled via pinmux/`jetson-io`; fall
  back to an external PCA9685 if unstable.
- **Carried-over accepted risk:** the MPU6050 is still not installed. On Jetson,
  `config.py` ships `STALL_GUARD_ENABLED = True` (safe default), but with no IMU
  that stall-cuts after 1 s and blocks driving — so wheels-off / driving bring-up
  still needs it set `False`, leaving the **55 % duty cap as the only thermal
  protection**. Keep runs short; a non-IMU stall detector is the mitigation to add.

### 8.2 Simulator #1 — broad digital twin (`eams_simulator/`)
- **Goal:** let the Jetson software stack be developed/validated before sensors
  are wired, using realistic **stand-in data** rather than random values.
- **Design:** one ground-truth `VehicleState` evolved by a **bicycle kinematic
  model**; every sensor observes that shared state (so streams stay mutually
  consistent). Async **master-tick scheduler** (300 Hz base divided to per-sensor
  rates), merged-JSON publisher with stdout / JSONL / CSV / WebSocket sinks,
  seeded RNG for reproducibility, pytest consistency tests (all pass).
- **Sensors:** GPS, IMU, encoder, motor, battery, current, LiDAR, ultrasonic,
  camera perception metadata — an **AV-scale** suite, broader than the current
  car, aligned with the longer-term autonomy vision.

### 8.3 Sensor-mapping analysis (finding)
Checked the simulator's outputs against the **RL policy's 18 inputs**
(`DifferentialCarAgent.CollectObservations`):
- **Match:** local velocity, yaw rate, ultrasonic front/rear.
- **Missing / mismatched:** the broad sim **omitted IR entirely** (6 of 18
  inputs), had **no target/goal** (4 inputs), and emits ultrasonic in **cm** while
  the policy expects **normalised 0..1**. `smoothedLeft/Right` are the policy's own
  rate-limited action memory, not sensor data.
- **Conclusion:** the broad simulator and the trained policy target *slightly
  different robots* (AV-style vs the real minimal ultrasonic+IR car). This drove a
  second, focused simulator.

### 8.4 Simulator #2 — focused rover sim (`eams_rover_sim/`)
- **Scope:** only the real car's suite — **IR ×6, ultrasonic (front/rear), IMU,
  motor RPM/PWM, pose, current-with-stall-latch, camera metadata**.
- **Reconciled the gaps:** added an **IR model** ordered `[FL, FR, RL, RR, L, R]`
  to match the policy's observation order; ultrasonic emitted in **cm and
  normalised 0..1** (policy-compatible).
- **Architecture mirrors the real car:** background **SimThread** (only state
  writer) + **Flask + flask-sock `/ws`** streaming merged frames at **20 Hz**,
  same `take`/`release`/`cmd`/`reset` handshake. A controller's `(left,right)` is
  converted to `(throttle,steer)` — inverse of the browser mixer — so it can drive
  the simulated rover.
- **Stall-detection hook:** current sensing exposes instant/avg/peak and a
  **stall latch** (high current + no motion → latch). A scripted **wedged phase**
  (~t=25–33 s) fires the latch at **t≈26 s** — prototypes the non-IMU stall
  protection the real car still owes.
- **Verified live:** viewer → take control → drive advances pose, motor duty caps
  at 55 %, current spikes under load, and the front ultrasonic + camera pick up the
  same obstacle together (streams consistent).
- **Note:** `left/right_rpm` is *motor-shaft* RPM (gear ratio 48); IMU is SI units,
  not raw MPU6050 counts, so this frame is a **superset** of the real telemetry,
  not byte-identical to what today's `app.js` scales. A counts-mode toggle is a
  small follow-up if drop-in browser compatibility is wanted.

### 8.5 Documentation system
Three-layer scheme so new chats/contributors don't start from scratch:
- **Project instructions** (Claude Project settings) — short, stable, every-chat rules.
- **`project-brief.md`** (new) — standing orientation: vision, phased roadmap,
  hardware, code artifacts, tensions, near-term path.
- **`rc-car-progress-report.md`** (this file) — the **logbook**, authoritative for
  history; every meaningful change gets a dated entry like this one.
- Older docs (`rc-car-requirements.md`, `usb-gadget-setup.md`) were marked
  **historical / Pi-era** with a banner pointing here and to `project-brief.md`.

### 8.6 Open items after this entry
1. Enable/verify PWM on Jetson BOARD pins 32/33 (`jetson-io`), else PCA9685.
2. Add a units **adapter** (normalise ultrasonic, IMU counts) + define a
   **target/goal source** before the trained policy can consume a live feed.
3. Decide whether to fit the MPU6050 or ship the current-sense stall detector
   from `eams_rover_sim` onto the real car.
4. Keep `project-brief.md` in sync as intent (esp. the phase framing) is confirmed.

## 9. Policy inference on-device, dummy sensor + motor modes, Autonomy Lab (27 July 2026)

**Context:** with the electronics team still bringing up the physical sensor
suite, we needed a way to exercise the trained PPO policy against the real
motor safety layer without waiting for real sensors, and to do it safely on the
bench without wheels turning. This entry ships that end-to-end.

### 9.1 Where the policy plugs in
The policy runner is *another controller* on the same interface the browser
uses. Inference in a background thread at `POLICY_HZ = 20` builds the 18-dim
observation vector, runs one ONNX forward pass, mixes `(throttle, steer)` to
`(targetLeft, targetRight)` line-for-line from
`DifferentialCarAgent.OnActionReceived`, and pushes them into
`motor_thread.set_command(...)`. The safety layer (deadman, stall latch, slew,
zero-cross coast, 55 % duty cap) runs downstream unchanged. **`motors.py`,
`hardware.py`, and the safety pipeline were not weakened** — a good sign the
design fits.

Observation source-by-source:

| Slot(s) | Field | Source today |
|---|---|---|
| 0–2 | Target unit vec (Unity local) | Hard-coded local target editable from UI (distance + bearing) |
| 3 | Target distance norm | Same |
| 4–6 | Local linear velocity norm | **Zero** — no odometry; documented gap, first thing to fix |
| 7 | Yaw rate norm | MPU6050 `gz` → dps → rad/s → normalised |
| 8–9 | Ultrasonic F/R (0..1) | Existing cm reading → `min(cm/100, 5) / 5` |
| 10–15 | IR ×6 in `[FL,FR,RL,RR,L,R]` | Existing 6-bit mask; wiring confirmed 27 Jul, `config.IR_LABELS` added |
| 16–17 | smoothedLeft/Right | Controller-side memory maintained in Python via a MoveTowards port |

Inference is CPU (onnxruntime, CPU provider) — enough headroom for a 2×128
MLP at 20 Hz on the Orin Nano; TensorRT is a drop-in later if we ever need
sub-5 ms.

### 9.2 Dummy sensor mode
`SensorThread` is now mode-dispatched, with three modes selectable at runtime:

- `hardware` — the existing GPIO/I²C path, byte-identical to before.
- `dummy_static` — publishes a hand-authored scenario (US cm, IR mask, IMU
  dps/g) editable from the UI. For exercising the observation adapters and
  seeing how the policy responds to fixed inputs.
- `dummy_kinematic` — reads `motor_thread.out_left/right` each tick, advances a
  diff-drive pose with `_KIN_WHEEL_MAX_MPS = 0.6` / `_KIN_WHEELBASE_M = 0.15`,
  and derives US/IR/IMU from that pose against a fixed obstacle map. Closes
  the loop off-hardware: policy commands → predicted motion → next
  observations → next inference.

All three write the **same** shared-dict keys under the same lock, so the
telemetry frame and observation builder don't know which mode is active.
Hardware setup runs at boot regardless of mode; runtime mode swaps do not
touch GPIO.

### 9.3 Dummy motor mode
A new toggle on `MotorThread`: when `_output_enabled` is false, `_drive_channel`
short-circuits — **the full safety pipeline still runs**, applied duty and
`out_left/right` telemetry update normally, but H-bridge pin writes are
suppressed. Coasts immediately on disable so nothing is left commanded. This is
what lets you engage the policy on the real Jetson with real hardware attached
and observe the full loop without wheels turning. Startup default is `real`;
`--motor-output dummy` boots in dummy mode.

### 9.4 Autonomy Lab (UI)
Slide-out panel behind a `LAB` tab in the top-right. Pilot page is visually
unchanged unless the panel is opened. Panel contents:

- **Sensor source** segmented control (hardware / static / kinematic).
- **Motor output** segmented control (real / dummy) with a browser `confirm()`
  when switching to real.
- **Dummy scenario editor** (visible only in static mode): US front/rear cm,
  IR mask with per-bit clickable buttons labelled FL/FR/RL/RR/L/R, IMU gz and
  ax fields. Apply-to-server button.
- **Policy target** editor (distance + bearing) with a compass-arrow preview.
- **Engage/Disengage** with a "wheels off ground" checkbox required to engage
  on real motors. Server enforces the gate — the checkbox is not honor-system.
- **Live observability panel**: throttle/steer as bidirectional bars, target
  vs applied (left, right), US-norm, yaw rate raw and normalised, target
  polar, inference latency (mean and running max), and a rolling ~10 s trace
  plot of throttle (amber), steer (blue), and worst-case ultrasonic (red).

Three small chips added to the pilot flags row so mode/engagement is visible
without opening the lab: `SENS:HW/STA/KIN`, `MTR:REAL/DUMMY`, `POL:ON/OFF`.

### 9.5 Safety interlocks
- Refuse `policy_engage` if a human currently holds control.
- Refuse `policy_engage` if motor output is **real** and the client did not
  set `wheels_off_ground: true` on the request.
- Refuse human `take` while policy is engaged — policy IS the driver.
- Refuse `set_motor_output(false)` while policy is engaged on real motors —
  makes you disengage first, so we don't get half-live states.
- On a WS disconnect the runner's controller state is torn down; the deadman
  fires within 300 ms regardless.
- On an inference exception the runner disengages itself and logs — the
  deadman catches whatever slips through.

### 9.6 Files touched

| File | Change |
|---|---|
| `config.py` | `IR_LABELS`, mirrored obs bounds (`MAX_LINEAR_SPEED = 3.0`, `MAX_ANGULAR_SPEED = 6.0`, `MAX_TARGET_DISTANCE = 20.0`, `ULTRASONIC_RANGE_M = 5.0`), `POLICY_MODEL_PATH`, `POLICY_HZ = 20`, `MOTOR_RESPONSE_RATE = 6.0` |
| `motors.py` | `set_output_enabled(bool)` + `output_enabled` property; `_drive_channel` early-returns when disabled |
| `sensors.py` | `SensorThread` now mode-dispatched (hardware / dummy_static / dummy_kinematic); `set_mode`, `set_scenario`, `reset_kinematic`, `get_kinematic_pose` |
| `policy.py` | **new** — `ObservationBuilder` + `PolicyRunner` (ONNX via onnxruntime, deterministic head, 20 Hz command loop, self-disengage on inference error, snapshot for UI) |
| `server.py` | `--sensor-mode`, `--motor-output`, `--no-policy` CLI flags; new WS messages `set_sensor_mode`, `set_motor_output`, `set_target`, `set_scenario`, `reset_kinematic`, `policy_engage`, `policy_disengage`; extended telemetry with `sensor_mode`, `motor.output_enabled`, `policy`, `scenario`, `kin_pose`, `policy_loaded`; new controller interlocks |
| `static/*` | Autonomy Lab panel (HTML + CSS + JS), pilot-page flag chips |
| `requirements.txt` | `onnxruntime>=1.17`, `numpy>=1.24` |
| `policies/DifferentialCarAgent-obstacles_v3.onnx` | trained model, already checked in |

### 9.7 Dry-run verification
`python3 server.py --dry-run --sensor-mode dummy_kinematic --motor-output dummy`
came up cleanly: motors, sensors, and policy threads all running; ONNX loaded
with `input=obs_0`, `output=deterministic_continuous_actions`. A scripted WS
client connected, set a target `(3.0 m, -20°)`, engaged the policy, and
telemetry frames arrived showing:

- inference latency **0.18 ms / running max 2.36 ms** (well under a 30 ms budget)
- policy commanding `throttle=+1.000, steer=+0.497`; motor applied
  `+0.550 / +0.437` (duty cap correctly biting on the left channel)
- kinematic pose advancing as expected (`x`, `z`, `yaw` all evolving)
- clean disengage; deadman firing 300 ms after the WS closed

Sign conventions all check out in a scenarios sweep (target-left → `steer < 0`
→ turn left through the mixer, obstacle-in-front → throttle drops toward zero,
boxed-in → spin-in-place).

### 9.8 Open items after this entry
1. **Local-velocity observation is zero.** No odometry → the policy never
   "sees" that it's moving. Wheel encoders or a VIO/IMU-integration proxy is
   the first accuracy win; without it the policy can over-command speed. This
   is the biggest sim-to-real gap in the current observation vector.
2. **Kinematic dummy is uncalibrated.** `_KIN_WHEEL_MAX_MPS` and
   `_KIN_WHEELBASE_M` are educated guesses; the mode is legible for demo
   purposes, not quantitatively accurate. Once real motors move a measured
   distance, pin these constants.
3. **Verify `onnxruntime` wheel on the actual Jetson (JetPack 7).** Design
   assumption is that pip installs cleanly; confirm on the device before
   step 5 of the bring-up in `project-brief.md §7`.
4. **PWM on BOARD pins 32/33** still needs `jetson-io`/pinmux verification,
   or a PCA9685 fallback. This is the last blocker for real-motor bring-up
   and is unchanged from §8.6.
5. **MPU6050 still not installed.** With `STALL_GUARD_ENABLED = True` in
   `config.py`, sensors publish zero gyro in hardware mode, which will
   stall-cut after 1 s the moment the policy commands non-zero duty on real
   motors. Either fit the IMU, keep the current-sense stall detector plan
   from §8.6, or explicitly `STALL_GUARD_ENABLED = False` for early policy
   drives (documented risk).
6. **UI defence for scenario input.** The static-scenario input fields are
   `<input type="number">` without min/max clamping in JS — the server
   ignores out-of-range values but the UI could preempt them.

### 9.9 Update `project-brief.md` alongside
- §4 code artifacts: add `policy.py` and note the SensorThread mode-dispatch;
  add "Autonomy Lab" to the client description.
- §6.4: the adapter is now shipped — the target/goal is a UI-editable local
  target, and units conversion is inside `ObservationBuilder`. The remaining
  gap in §6.4 becomes the local-velocity observation (item 1 above).
- §7 near-term path: step 5 now becomes "engage on real motors, wheels off
  ground, dummy sensors, observe via Autonomy Lab" (item 3 above).


---

## Glossary

- **BCM numbering** — Naming GPIO pins by the Broadcom chip's numbers (GPIO 12) instead of physical header positions.
- **Deadman (switch)** — Safety rule: if control packets stop arriving, the motors stop automatically.
- **Differential drive / arcade mix** — Steering by running left and right wheels at different speeds; arcade mix converts one joystick (x, y) into left/right values.
- **distutils** — Old Python packaging module, removed in Python 3.12; caused the `make install` failure.
- **Duty cycle / duty cap** — Fraction of time the PWM signal is "on" (= motor power). The cap limits it to 55 % to protect the L298N.
- **H-bridge (L298N)** — Circuit that lets a motor run in both directions; the L298N has two of them.
- **I²C** — Two-wire bus (SDA data, SCL clock) used to talk to the MPU6050.
- **IMU** — Inertial Measurement Unit; combined accelerometer + gyroscope (MPU6050).
- **ldconfig** — Refreshes Linux's cache of shared libraries so newly installed `.so` files are found.
- **pigpio / pigpiod / pigs** — GPIO library, its background daemon, and its command-line tool. Uses DMA for precise timing.
- **PWM** — Pulse-Width Modulation; rapidly switching a pin on/off to control average power.
- **rsync** — File-copy tool that transfers only differences; used to push code to the Pi over SSH.
- **Slew rate limit** — Changing motor duty gradually instead of instantly, to avoid current spikes.
- **systemd unit / service** — Config file telling Linux to run a program at boot and restart it on failure.
- **Telemetry** — The stream of sensor/status data sent from the Pi to the browser.
- **TLS handshake (`\x16\x03\x01`)** — The first bytes of an HTTPS connection; seen as garbage 400 errors when a browser speaks HTTPS to an HTTP server.
- **venv** — Isolated Python environment so project packages don't touch the system Python.
- **WebSocket** — A persistent two-way connection over HTTP; carries commands down and telemetry up on one channel.