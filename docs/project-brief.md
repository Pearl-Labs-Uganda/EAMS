# EAMS Rover — Project Brief

> **Purpose of this document.** This is the standing orientation for the EAMS
> autonomous rover project. Any new chat should read it first to understand what
> we're building, why, where we are, and how the pieces fit — without
> re-deriving it from the code every time. It describes **stable structure**.
> For *what actually happened and when*, the authoritative record is
> `rc-car-progress-report.md` (the project logbook). If this brief and the
> logbook ever disagree on history, the logbook wins; update this brief to match.

---

## 1. What we're building

A 4-wheel **differential-drive rover** (an RC-car-sized robot) built by a small
team learning robotics as we go. It runs on an **NVIDIA Jetson Orin Nano** as the
onboard computer (migrated from a Raspberry Pi Zero 2 W prototype).

**Ultimate goal:** a *smart car that takes natural-language instructions and
drives itself to target locations*, avoiding obstacles on the way.

We are **not** experts. Documentation exists so that as the hardware and code
change (e.g. Pi → Jetson), we keep a clear trail of what we did and why, and so
new contributors — human or AI — can get up to speed fast.

---

## 2. The roadmap (how the parts relate)

The project looks like it has contradictory sensor suites and goals until you see
it as **phases toward the autonomy goal**. (This phasing is a proposed structure
to organise the work, not a fixed contract — adjust as we learn.)

| Phase | Capability | Status | Main artifacts |
|---|---|---|---|
| **1** | Manual browser control + live telemetry | Built; running, migrating to Jetson | `rccar/` stack |
| **2** | Autonomous point-to-point navigation (drive to a target, avoid obstacles) | Policy trained in sim; sim-to-real transfer pending | Unity ML-Agents agent + configs |
| **3** | Natural-language instruction → pick target / plan → hand off to Phase 2 | Not started (ultimate goal) | TBD |
| **Cross-cutting** | Simulation / digital twin so software can be built before hardware is wired; documentation | Two simulators built | `eams_simulator/`, `eams_rover_sim/`, this brief, the logbook |

Key point: the manual stack (Phase 1) and the RL/autonomy work (Phase 2+) are
**both in scope**. Phase 1 gives us a safe, hand-driveable, well-instrumented
vehicle; Phase 2 replaces the human driver with a policy; Phase 3 replaces the
human *choosing the destination* with language understanding.

---

## 3. Hardware (current: Jetson build)

- **Compute:** NVIDIA Jetson Orin Nano (was Raspberry Pi Zero 2 W).
- **Drivetrain:** 4× TT gear motors, **two paralleled per L298N channel** (left
  pair → channel A, right pair → channel B). **One L298N** by hard constraint.
- **Sensors:** 2× ultrasonic (front + rear), 6× IR proximity (LM393), 1× MPU6050
  IMU (I²C). *The IMU is not physically installed yet — see §6.*
- **Power:** 3S Li-ion pack direct to the L298N motor rail; LM2596 buck →
  5 V logic/sensor rail. L298N onboard 5 V regulator jumper **removed**.
- **Pin map:** `config.py` (Jetson.GPIO **BOARD** numbering) is the **single
  authoritative source of truth**. If any doc's pin table disagrees with
  `config.py`, `config.py` wins.

### The single most important design fact
Two motors share each L298N channel, so stall current (~2–3 A) exceeds the
chip's realistic rating. **The software is the thermal protection.** Every guard
in the motor safety layer is a hard requirement, not a nice-to-have.

---

## 4. The code artifacts

**A. Real control + telemetry stack (`rccar/`) — Phase 1.**
One Python process, three roles:
- Flask + `flask-sock` web/socket thread — serves the browser app, owns one
  WebSocket (commands down, telemetry up), enforces *many viewers / one driver*
  (`take` / `release` / `cmd` / `reset`).
- Sensor thread — samples ultrasonics (alternating front/rear to avoid
  cross-talk), IR (6-bit mask), IMU, at 20 Hz into a shared locked dict.
- Motor thread — the **only** code that touches motor pins; a 50 Hz safety loop
  applying, in order: **deadman → stall latch → slew limit → zero-cross coast →
  duty cap (55 %)**.
`hardware.py` is the Jetson port: it maps the old pigpio surface onto
`Jetson.GPIO` (BOARD mode) + `smbus2`, so the safety architecture was reused
unchanged. Browser client is framework-free (joystick + Gamepad → arcade mixer →
20 Hz JSON, *including zeros* — silence is what fires the deadman).

**B. Unity ML-Agents RL agent (`DifferentialCarAgent.cs` + `car_config_obsatcles.yaml`) — Phase 2.**
A PPO sim-to-real policy for the same car. **18-dim observation:** target
direction (3) + distance (1), local linear velocity (3), yaw rate (1),
ultrasonic front/rear (2), IR ×6 (6), and its own rate-limited motor state (2).
**Action space:** throttle + steer (mixed to wheels), which is
differential-drive-native and sim-to-real safe. Features domain randomization,
sensor-noise injection, a bootstrap-on-timeout path, and a **curriculum** that
ramps `obstacle_count` (0 → full clutter) after warm-starting from an
empty-arena policy.

**C. Broad digital-twin simulator (`eams_simulator/`) — cross-cutting, forward-looking.**
A full AV sensor suite — GPS, IMU, encoder, motor, battery, current, LiDAR,
ultrasonic, camera perception metadata — all derived from one ground-truth
`VehicleState` (bicycle kinematic model), published as merged JSON at per-sensor
rates (async master tick). Broader than the current car; aligns with the
longer-term autonomy vision rather than today's policy.

**D. Focused rover simulator (`eams_rover_sim/`) — cross-cutting, matches the real car.**
Scoped to the *actual* hardware + policy: **IR ×6, ultrasonic (front/rear), IMU,
motor RPM/PWM, pose, current-with-stall-latch, camera metadata**. Streams over a
WebSocket that **mirrors the real `rccar/` architecture** (background SimThread +
`/ws` + `take`/`release`/`cmd`/`reset`), so a controller can actually drive the
simulated rover, and a scripted "wedged" phase demonstrates the current-based
stall latch. IR order is `[FL, FR, RL, RR, L, R]` to match the policy's
observation order; ultrasonic is emitted in cm **and** normalised 0..1.

---

## 5. The one principle behind both simulators

**One ground-truth state, observed by every sensor.** Sensors never invent
independent values — accelerate and the encoder, IMU, RPM, current, GPS speed all
rise together; turn and the wheel speeds, yaw, heading, camera bearing all move
together. That mutual consistency is the entire point of a digital twin: it lets
the Jetson software stack (perception, planning, the RL policy's input pipeline)
be developed and validated **before** the physical sensors are wired in.

---

## 6. Current state & known tensions

These are the things most likely to trip up a new contributor.

1. **Docs straddle Pi and Jetson.** The runtime code (`config.py`, `hardware.py`,
   `requirements.txt`) and `rc-car-deployment.md` are Jetson. The
   *progress report* still narrates the earlier **Pi/pigpio** build (user
   `eams-pi`, BCM numbering, USB-gadget link). The migration happened; not all
   prose caught up. **`config.py` is authoritative for pins.**

2. **IMU not installed → stall guard effectively off.** Running with
   `STALL_GUARD_ENABLED = False` has been the accepted operating state, so the
   **55 % duty cap is currently the only thermal protection**. This is why
   current-sensing / a non-IMU stall detector matters — `eams_rover_sim`
   prototypes exactly that (high current + no motion → latch).

3. **The RL policy and the broad simulator target slightly different robots.**
   The policy assumes the real minimal suite (ultrasonic + IR + a target). The
   broad `eams_simulator` was specced AV-style (LiDAR/GPS/camera) and originally
   **omitted IR**. This isn't a bug so much as Phase-2-vs-future divergence.
   `eams_rover_sim` was built to reconcile toward the real car.

4. **To feed the trained policy from a sim/real feed, an adapter is needed.**
   The policy expects ultrasonic normalised 0..1 (0 = touching, 1 = clear) and
   raw IMU counts. Two things are still missing for a full 18-input feed:
   a **target/goal** (4 obs) and, if using the live telemetry, a units adapter.
   The policy's `smoothedLeft/Right` inputs are controller-side memory, not a
   sensor — the deployment controller maintains them from its own actions.

---

## 7. Near-term path (bring-up order)

1. **Validate software with no hardware** on the Jetson: `rccar/server.py
   --dry-run` (GPIO stubbed) and the simulators. Zero sensors required.
2. **Hardware bring-up**, wheels off the ground. The Jetson-specific risk is
   **hardware PWM on the 40-pin header**: `jetson-io` on our JetPack offers PWM
   on BOARD pins **15, 32 and 33** (confirmed on hardware 30 Jul 2026 — several
   published Orin Nano pinouts list only 15 and 33 and are wrong for us; trust
   the board). We use **15 and 33**, which sit on separate PWM controllers.
   Enable those two via `sudo /opt/nvidia/jetson-io/jetson-io.py` and reboot,
   and **leave PWM off for pin 32** — it carries `US_FRONT_TRIG`, and a pin
   muxed to the PWM controller ignores GPIO writes, so the front ultrasonic
   would silently read nothing. Fall back to an external PCA9685 if 15/33
   prove unstable. Two further header traps: pins **24/26 are SPI chip-selects
   by default** and now carry IN3/IN4, so SPI must be disabled in `jetson-io`
   or the direction writes are silently ignored; and Jetson.GPIO will only
   drive the last PWM channel constructed unless each channel's setup is
   immediately followed by its own PWM object (fixed in `hardware.py`).

   Checklist, in order: L298N 5 V jumper removed → 5 V sensor outputs
   level-shifted (both ultrasonic echo lines need dividers) → `jetson-io` shows
   PWM on 15/33, PWM **off** on 32, and SPI off → **`pwm_bench.py` with the
   motor battery
   disconnected** confirms both PWM chips enabled with nonzero duty in
   `/sys/kernel/debug/pwm` → deadman stops wheels < 300 ms → direction mapping
   verified **before** floor driving.
3. **Service install** (`rccar.service`) once manual run is clean.
4. **Phase 2 onward:** wire the trained policy's input pipeline (with the adapter
   from §6.4), then define the target/goal source; later, Phase 3 language layer.

---

## 8. How our documentation is organised

- **Project instructions** (in the Claude Project settings) — short, stable,
  injected into every chat. Role, scope, hard constraints, doc-usage rules.
- **`project-brief.md`** (this file) — fuller orientation; stable structure.
- **`rc-car-progress-report.md`** — the **logbook**: dated, chronological record
  of what we did and why. *Authoritative for history.* Every meaningful change
  gets an appended, dated entry.
- **`rc-car-requirements.md`, `rc-car-deployment.md`, `usb-gadget-setup.md`, `README.md`** —
  reference docs for the original brief, deployment, and setup.

**Rule of thumb:** decisions and changes get logged in the progress report;
orientation lives here; hard rules live in the project instructions.

**Commit rule:** for every change, include a **git commit message and body**.
Group related files into one logical commit; write a short imperative subject
(matching our `feat:` / `doc:` / `chore:` / `refactor:` convention) and a body
that explains *why*, not just what. When a change is significant enough to log
in the progress report, the commit body and the logbook entry should tell the
same story.

---

## 9. Caveat on intent

Some of this brief — especially the phase framing in §2 and *why* the broad
simulator diverged from the real hardware in §6 — is **inferred from the
artifacts**, not stated anywhere in the original docs. Treat it as our working
model, and correct it in this file as the team's actual intent becomes explicit.