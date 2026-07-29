"""policy.py — trained-policy inference runner.

Loads the Unity ML-Agents ONNX policy and drives it against the shared sensor
state at POLICY_HZ. Commands reach the motors through the same
motor_thread.set_command(...) interface the browser client uses, so the
existing safety layer (deadman, stall latch, slew, zero-cross, duty cap) runs
downstream unchanged and is not weakened by anything in this file.

The 18-dim observation vector is assembled to match
DifferentialCarAgent.CollectObservations byte-for-byte. If the sim is
retrained with different normalisation bounds, update the mirrored constants
in config.py — the vector must match training or the policy silently
misbehaves.
"""

import logging
import math
import threading
import time

import numpy as np
import onnxruntime as ort

import config

log = logging.getLogger("policy")


# ---------------------------------------------------------------- helpers ---
def _move_toward(cur, target, step):
    """Python port of UnityEngine.Mathf.MoveTowards — used to replicate the
    controller-side action memory (smoothedLeft / smoothedRight) that the
    trained policy feeds back to itself."""
    diff = target - cur
    if abs(diff) <= step:
        return target
    return cur + math.copysign(step, diff)


def _us_cm_to_norm(cm):
    """Ultrasonic reading in cm -> policy-expected 0..1 (0 = touching,
    1 = clear). Missing readings (None) fall back to 1.0 (assume clear) so a
    stale sensor doesn't panic the policy into a permanent stop."""
    if cm is None:
        return 1.0
    m = cm / 100.0
    return max(0.0, min(1.0, m / config.ULTRASONIC_RANGE_M))


# ------------------------------------------------------- observation builder
class ObservationBuilder:
    """Assembles the 18-dim vector the Unity agent trained on.

    Order (must match DifferentialCarAgent.CollectObservations exactly):
        0..2   target direction in car-local frame (unit vector, Unity axes)
        3      target distance normalised (0..1)
        4..6   local linear velocity, normalised — CURRENTLY ZERO, see note
        7      yaw rate normalised (-1..1)
        8      ultrasonic front (0..1)
        9      ultrasonic rear  (0..1)
        10..15 IR ×6 in policy slot order (FL, FR, RL, RR, L, R)
        16     smoothedLeft  — controller-side memory
        17     smoothedRight — controller-side memory
    """

    def __init__(self, shared, lock):
        self.shared = shared
        self.lock = lock

        # Target in car-local polar coords. Default: 2 m ahead, 0° bearing.
        # Bearing convention: 0° straight ahead, positive = left.
        self._target_lock = threading.Lock()
        self._target = (2.0, 0.0)     # (distance_m, bearing_deg)

        # smoothedLeft / smoothedRight — see the Unity agent's
        # OnActionReceived: MoveTowards at motorResponseRate per fixed step.
        self._smoothed_left = 0.0
        self._smoothed_right = 0.0

    # ------------------------------------------------------ target editing
    def set_target(self, distance_m, bearing_deg):
        d = max(0.0, min(config.MAX_TARGET_DISTANCE, float(distance_m)))
        # keep bearing in [-180, 180] for readability in the UI
        b = ((float(bearing_deg) + 180.0) % 360.0) - 180.0
        with self._target_lock:
            self._target = (d, b)

    def get_target(self):
        with self._target_lock:
            return self._target

    # ------------------------------------------ smoothed action bookkeeping
    def update_smoothed(self, target_left, target_right, dt):
        step = config.MOTOR_RESPONSE_RATE * dt
        self._smoothed_left = _move_toward(self._smoothed_left, target_left, step)
        self._smoothed_right = _move_toward(self._smoothed_right, target_right, step)

    def reset_smoothed(self):
        self._smoothed_left = 0.0
        self._smoothed_right = 0.0

    # ------------------------------------------------------ build the vec
    def build(self):
        """Return (obs_np[1,18], debug_dict) — debug is for the UI panel."""
        with self.lock:
            imu = self.shared.get("imu",
                                  {"ax": 0, "ay": 0, "az": 0,
                                   "gx": 0, "gy": 0, "gz": 0})
            us = self.shared.get("us", {"front": None, "rear": None})
            ir_mask = int(self.shared.get("ir", 0))

        # Target -> car-local unit vector, using Unity's axes
        # (x=right, y=up, z=forward). "Bearing left positive" means the
        # target sits at negative x, positive z.
        dist_m, bearing_deg = self.get_target()
        br = math.radians(bearing_deg)
        tx = -dist_m * math.sin(br)
        tz = dist_m * math.cos(br)
        target_len = math.sqrt(tx * tx + tz * tz) or 1e-6
        target_dir = (tx / target_len, 0.0, tz / target_len)
        target_dist_norm = min(1.0, dist_m / config.MAX_TARGET_DISTANCE)

        # Local linear velocity: we don't have odometry yet. Feeding zeros
        # is the honest choice — see project-brief §6.4. This is the largest
        # observation gap and the first thing to fix once wheel encoders or
        # a visual-inertial fallback are on the bench.
        local_vel = (0.0, 0.0, 0.0)

        # Yaw rate: raw MPU6050 gz counts -> dps -> rad/s -> normalised.
        gz_dps = imu.get("gz", 0) / 131.0
        gz_rad = math.radians(gz_dps)
        yaw_norm = max(-1.0, min(1.0, gz_rad / config.MAX_ANGULAR_SPEED))

        # Ultrasonic: cm -> 0..1 (0 = touching, 1 = clear).
        us_front_norm = _us_cm_to_norm(us.get("front"))
        us_rear_norm = _us_cm_to_norm(us.get("rear"))

        # IR ×6 in policy slot order. config.IR_LABELS is already in
        # (FL, FR, RL, RR, L, R) order, and IR_PINS is parallel, so the
        # bit at position i already corresponds to the right slot.
        ir_vec = [float((ir_mask >> b) & 1) for b in range(6)]

        obs = np.array([
            target_dir[0], target_dir[1], target_dir[2],
            target_dist_norm,
            local_vel[0], local_vel[1], local_vel[2],
            yaw_norm,
            us_front_norm, us_rear_norm,
            ir_vec[0], ir_vec[1], ir_vec[2], ir_vec[3], ir_vec[4], ir_vec[5],
            self._smoothed_left, self._smoothed_right,
        ], dtype=np.float32).reshape(1, 18)

        debug = {
            "target": {"distance_m": dist_m, "bearing_deg": bearing_deg,
                       "dist_norm": target_dist_norm,
                       "dir": list(target_dir)},
            "local_vel": list(local_vel),
            "yaw_rate_rad_s": gz_rad,
            "yaw_norm": yaw_norm,
            "us_norm": {"front": us_front_norm, "rear": us_rear_norm},
            "ir": {config.IR_LABELS[i]: ir_vec[i] for i in range(6)},
            "smoothed": {"left": self._smoothed_left,
                         "right": self._smoothed_right},
        }
        return obs, debug


# ---------------------------------------------------------- policy runner
class PolicyRunner(threading.Thread):
    """Runs the ONNX policy at POLICY_HZ and commands the motor thread.

    Not a WebSocket client — commands go directly to motor_thread.set_command,
    which is the same call the socket handler makes for browser cmds. The
    engage/disengage lifecycle is separate from the browser take/release
    lifecycle; server.py enforces "at most one of policy OR human" at the
    socket-handler layer.
    """

    def __init__(self, motor_thread, shared, lock, model_path=None):
        super().__init__(name="policy", daemon=True)
        self.motor_thread = motor_thread
        self.shared = shared
        self.lock = lock

        self._stop_evt = threading.Event()
        self._engage_lock = threading.Lock()
        self._engaged = False
        self._seq = 0

        self.obs_builder = ObservationBuilder(shared, lock)

        model_path = model_path or config.POLICY_MODEL_PATH
        log.info("loading ONNX policy from %s", model_path)
        # CPU provider is plenty for a 2×128 MLP at 20 Hz on the Orin Nano.
        # TensorRT drop-in later if inference ever needs to be sub-5 ms.
        self.session = ort.InferenceSession(
            model_path, providers=["CPUExecutionProvider"])
        self._input_name = "obs_0"
        # Deterministic head: mean of the policy distribution, no exploration
        # noise. `continuous_actions` is the sampled head used in training.
        self._output_name = "deterministic_continuous_actions"

        # Warmup so the first real tick isn't a cold-cache outlier.
        _ = self.session.run(
            [self._output_name],
            {self._input_name: np.zeros((1, 18), dtype=np.float32)})
        log.info("policy loaded; input=%s output=%s",
                 self._input_name, self._output_name)

        # ---- telemetry snapshot for the UI ------------------------------
        self._telem_lock = threading.Lock()
        self._last_debug = {}
        self._last_action = (0.0, 0.0)     # (throttle, steer)
        self._last_wheels = (0.0, 0.0)     # target (left, right) pre-safety
        self._last_latency_ms = 0.0
        self._max_latency_ms = 0.0
        self._last_tick_wall = 0.0

    # ---------------------------------------------------- engage lifecycle
    def engage(self):
        with self._engage_lock:
            if self._engaged:
                return False
            self._engaged = True
            self._seq = 0
        self.obs_builder.reset_smoothed()
        with self._telem_lock:
            self._max_latency_ms = 0.0
        log.warning("POLICY ENGAGED — inference now commanding motors")
        return True

    def disengage(self):
        with self._engage_lock:
            if not self._engaged:
                return False
            self._engaged = False
        # Push one final zero so the motors get a clean stop without waiting
        # for the deadman window to expire.
        self.motor_thread.set_command(0.0, 0.0, self._seq)
        self._seq += 1
        log.warning("POLICY DISENGAGED")
        return True

    @property
    def engaged(self):
        with self._engage_lock:
            return self._engaged

    # ---------------------------------------------------- external hooks
    def set_target(self, distance_m, bearing_deg):
        self.obs_builder.set_target(distance_m, bearing_deg)

    def snapshot(self):
        with self._telem_lock:
            debug = dict(self._last_debug)
            action = self._last_action
            wheels = self._last_wheels
            latency = self._last_latency_ms
            max_latency = self._max_latency_ms
            last_tick = self._last_tick_wall
        dist, bearing = self.obs_builder.get_target()
        return {
            "engaged": self.engaged,
            "throttle": action[0],
            "steer": action[1],
            "target_left": wheels[0],
            "target_right": wheels[1],
            "latency_ms": round(latency, 2),
            "max_latency_ms": round(max_latency, 2),
            "age_ms": round((time.monotonic() - last_tick) * 1000.0, 1)
                       if last_tick else None,
            "target": {"distance_m": dist, "bearing_deg": bearing},
            "obs": debug,
        }

    def stop(self):
        self._stop_evt.set()

    # ---------------------------------------------------- main loop
    def run(self):
        dt = 1.0 / config.POLICY_HZ
        while not self._stop_evt.is_set():
            tick_start = time.monotonic()
            if not self.engaged:
                time.sleep(dt)
                continue

            obs, debug = self.obs_builder.build()
            t0 = time.monotonic()
            try:
                out = self.session.run(
                    [self._output_name], {self._input_name: obs})[0]
            except Exception as e:                                  # noqa: BLE001
                # An inference error must not leave stale commands on the
                # motors. Disengage and let the deadman finish the job.
                log.error("policy inference failed: %s -- disengaging", e)
                self.disengage()
                continue
            latency_ms = (time.monotonic() - t0) * 1000.0

            throttle = float(np.clip(out[0, 0], -1.0, 1.0))
            steer = float(np.clip(out[0, 1], -1.0, 1.0))

            # Mixer — line-for-line from DifferentialCarAgent.OnActionReceived:
            #   targetLeft  = clamp(throttle + steer, -1, 1)
            #   targetRight = clamp(throttle - steer, -1, 1)
            target_left = max(-1.0, min(1.0, throttle + steer))
            target_right = max(-1.0, min(1.0, throttle - steer))

            # Advance the smoothed-action memory the same way Unity does, so
            # observations 16 and 17 on the next tick match training.
            self.obs_builder.update_smoothed(target_left, target_right, dt)

            self.motor_thread.set_command(target_left, target_right, self._seq)
            self._seq += 1

            with self._telem_lock:
                self._last_debug = debug
                self._last_action = (throttle, steer)
                self._last_wheels = (target_left, target_right)
                self._last_latency_ms = latency_ms
                self._max_latency_ms = max(self._max_latency_ms, latency_ms)
                self._last_tick_wall = time.monotonic()

            elapsed = time.monotonic() - tick_start
            time.sleep(max(0.0, dt - elapsed))
