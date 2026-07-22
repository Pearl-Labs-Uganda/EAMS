"""motors.py — motor thread + safety layer.

This is the ONLY file that touches motor pins. All guards from §5 live here,
server-side, inside the 50 Hz loop. The client cannot bypass any of them.

Per-channel pipeline each tick, in order:
  1. deadman        (no packet for DEADMAN_S -> target 0)
  2. stall latch    (latched stall -> target 0 until reset)
  3. slew limit     (ramp applied duty toward target, full scale in ~200 ms)
  4. zero-cross     (sign flip -> ZERO_CROSS_COAST_S of coast first)
  5. duty cap       (hard clamp at DUTY_CAP, applied LAST)
"""

import logging
import threading
import time

import config

log = logging.getLogger("motors")


class MotorThread(threading.Thread):
    """Owns the motor pins. Runs the safety loop at MOTOR_LOOP_HZ."""

    def __init__(self, pi, shared, lock):
        super().__init__(name="motor", daemon=True)
        self.pi = pi
        self.shared = shared          # sensor thread's latest-value dict
        self.lock = lock
        self._stop = threading.Event()

        # Command state (written by socket thread via set_command/reset)
        self._cmd_lock = threading.Lock()
        self._cmd_left = 0.0
        self._cmd_right = 0.0
        self._last_cmd_time = 0.0     # 0 => never; deadman starts tripped
        self._last_seq = -1

        # Applied state (motor thread only)
        self._applied = {"L": 0.0, "R": 0.0}
        self._coast_until = {"L": 0.0, "R": 0.0}
        self._pending_sign = {"L": 0.0, "R": 0.0}

        # Flags exported to telemetry
        self.stall = False
        self.dead = True
        self.capped = False
        self.out_left = 0.0
        self.out_right = 0.0

        self._stall_motion_time = time.monotonic()

        self._setup_pins()
        self._all_stop()              # §5 startup state: zero before socket opens
        
        if not config.STALL_GUARD_ENABLED:
            log.warning("STALL GUARD DISABLED — no stall protection. "
                        "Wheels off the ground only.")

    # ----------------------------------------------------------- external API
    def set_command(self, left, right, seq):
        """Called by the socket thread. Values already validated/clamped."""
        with self._cmd_lock:
            if seq <= self._last_seq:
                return                # ignore out-of-order packets
            self._last_seq = seq
            self._cmd_left = left
            self._cmd_right = right
            self._last_cmd_time = time.monotonic()

    def new_controller(self):
        """Control changed hands (take / release / disconnect).

        Forget the previous controller's seq counter — app.js restarts its
        counter at 0 on every page load, and without this reset every packet
        from the new controller is rejected as out-of-order and the deadman
        latches forever. Also zero the pending command so nothing from the
        old controller carries over; the deadman covers the handover gap.
        """
        with self._cmd_lock:
            self._last_seq = -1
            self._cmd_left = 0.0
            self._cmd_right = 0.0

    def reset(self):
        """Explicit client reset: clears a latched stall."""
        with self._cmd_lock:
            self.stall = False
            self._stall_motion_time = time.monotonic()
        log.info("stall latch cleared by client reset")

    def stop(self):
        self._stop.set()

    def shutdown(self):
        """Every exit path lands here (atexit / signals / exceptions)."""
        self._stop.set()
        self._all_stop()

    # ------------------------------------------------------------------- GPIO
    def _setup_pins(self):
        for pin in (config.ENA, config.ENB, config.IN1, config.IN2,
                    config.IN3, config.IN4):
            self.pi.set_mode(pin, 1)  # OUTPUT
        for pin in (config.ENA, config.ENB):
            self.pi.set_PWM_frequency(pin, config.PWM_FREQ_HZ)
            self.pi.set_PWM_range(pin, 255)

    def _all_stop(self):
        for pin in (config.IN1, config.IN2, config.IN3, config.IN4):
            self.pi.write(pin, 0)
        self.pi.set_PWM_dutycycle(config.ENA, 0)
        self.pi.set_PWM_dutycycle(config.ENB, 0)
        self._applied = {"L": 0.0, "R": 0.0}
        self.out_left = self.out_right = 0.0

    def _drive_channel(self, ch, duty_signed):
        """duty_signed in [-DUTY_CAP, DUTY_CAP] after the cap. Sets IN pins + PWM."""
        if ch == "L":
            in_a, in_b, en = config.IN1, config.IN2, config.ENA
        else:
            in_a, in_b, en = config.IN3, config.IN4, config.ENB

        if duty_signed > 0:
            self.pi.write(in_a, 1); self.pi.write(in_b, 0)
        elif duty_signed < 0:
            self.pi.write(in_a, 0); self.pi.write(in_b, 1)
        else:
            self.pi.write(in_a, 0); self.pi.write(in_b, 0)   # coast

        self.pi.set_PWM_dutycycle(en, int(abs(duty_signed) * 255))

    # ---------------------------------------------------------------- helpers
    def _imu_moving(self):
        with self.lock:
            imu = self.shared.get("imu")
        if not imu:
            return False
        return max(abs(imu["gx"]), abs(imu["gy"]), abs(imu["gz"])) \
            > config.STALL_GYRO_THRESHOLD_COUNTS

    # -------------------------------------------------------------- main loop
    def run(self):
        dt = 1.0 / config.MOTOR_LOOP_HZ
        max_step = dt / config.SLEW_FULL_SCALE_S     # slew: full scale in ~200 ms
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                with self._cmd_lock:
                    tgt = {"L": self._cmd_left, "R": self._cmd_right}
                    last_cmd = self._last_cmd_time

                # 1. deadman
                was_dead = self.dead
                self.dead = (now - last_cmd) > config.DEADMAN_S
                if self.dead:
                    tgt = {"L": 0.0, "R": 0.0}
                    if not was_dead:
                        log.warning("DEADMAN tripped: no command for %.0f ms",
                                    config.DEADMAN_S * 1000)

                # 2. stall detection / latch
                if config.STALL_GUARD_ENABLED:
                    commanded = abs(self._applied["L"]) > 0.02 or \
                                abs(self._applied["R"]) > 0.02
                    if self._imu_moving() or not commanded:
                        self._stall_motion_time = now
                    elif not self.stall and \
                            (now - self._stall_motion_time) > config.STALL_TIMEOUT_S:
                        self.stall = True
                        log.warning("STALL cut: duty commanded, no IMU motion for "
                                    "%.1f s — awaiting client reset",
                                    config.STALL_TIMEOUT_S)
                    if self.stall:
                        tgt = {"L": 0.0, "R": 0.0}

                # 3–5. per channel: slew, zero-cross coast, cap
                any_capped = False
                for ch in ("L", "R"):
                    cur = self._applied[ch]
                    want = tgt[ch]

                    # zero-cross: if sign flips, park at 0 and coast first
                    if cur * want < 0:
                        if cur != 0.0:
                            want = 0.0                      # ramp to zero first
                        if abs(cur) <= max_step:
                            # reaching zero this tick: start the coast window
                            if self._coast_until[ch] < now:
                                self._coast_until[ch] = now + config.ZERO_CROSS_COAST_S
                    if now < self._coast_until[ch]:
                        want = 0.0                          # in coast window

                    # slew limit
                    step = max(-max_step, min(max_step, want - cur))
                    new = cur + step
                    if abs(new) < 1e-4:
                        new = 0.0
                    self._applied[ch] = new

                    # duty cap — applied LAST
                    out = max(-config.DUTY_CAP, min(config.DUTY_CAP, new))
                    if abs(new) > config.DUTY_CAP:
                        any_capped = True
                    self._drive_channel(ch, out)
                    if ch == "L":
                        self.out_left = out
                    else:
                        self.out_right = out

                self.capped = any_capped
                time.sleep(dt)
        finally:
            self._all_stop()          # §5: every exit path zeroes the motors