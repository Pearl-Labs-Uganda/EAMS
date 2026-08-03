"""Sensor publisher for the Jetson Orin Nano RC car build.

Samples sensors on a fixed tick and writes the latest values into a shared
dict under a lock. It never touches motor pins and never blocks the socket
thread.

Three runtime modes (switchable via set_mode from server.py):

  hardware         Real GPIO/I²C reads. This is production.
  dummy_static     Static, hand-authored values from an editable scenario.
                   Great for exercising the policy against fixed inputs and
                   confirming the observation adapters are correct.
  dummy_kinematic  Reads the motor thread's applied duty, advances a tiny
                   diff-drive pose, and derives ultrasonic / IR / IMU
                   readings from that pose against a fixed obstacle map.
                   This is what closes the policy loop off-hardware.

The shared-dict publishing contract is identical in all three modes so the
browser telemetry and the observation builder don't need to know which mode
is active. Hardware initialisation runs at boot regardless of mode so a
runtime mode swap does not have to touch the GPIO layer.
"""

import logging
import math
import random
import threading
import time

import config

log = logging.getLogger("sensors")

_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT_H = 0x3B

# MPU6050 scale factors at default full-scale ranges (±2 g, ±250 dps).
# Dummy modes generate raw counts so the browser's existing scaling works
# unchanged.
_ACC_COUNTS_PER_G = 16384
_GYR_COUNTS_PER_DPS = 131


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class _Ultrasonic:
    """One US-100 in trigger/echo mode."""

    def __init__(self, hw, trig, echo):
        self.hw = hw
        self.trig = trig
        self.echo = echo
        hw.set_mode(trig, 1)          # OUTPUT
        hw.set_mode(echo, 0)          # INPUT
        hw.write(trig, 0)

    def measure_cm(self):
        """Blocking single shot; returns int cm or None on timeout/garbage."""
        self.hw.gpio_trigger(self.trig, 10, 1)

        deadline = time.monotonic_ns() + int(config.US_TIMEOUT_S * 1_000_000_000)
        while self.hw.read(self.echo) == 0:
            if time.monotonic_ns() >= deadline:
                return None

        rise = time.monotonic_ns()
        while self.hw.read(self.echo) == 1:
            if time.monotonic_ns() >= deadline:
                return None

        width_us = (time.monotonic_ns() - rise) / 1000.0
        cm = width_us / 58.0
        if cm <= 0 or cm > config.US_MAX_VALID_CM:
            return None
        return int(round(cm))

    def cancel(self):
        pass


# ---------------------------------------------------------------- kinematic ---

# Rough physical constants for the kinematic dummy mode. These are educated
# guesses — good enough for closed-loop policy testing off-hardware. Do not
# treat them as calibrated: the point is that "policy commands forward,
# position advances, front ultrasonic closes, policy backs off" is legible,
# not that the numbers match reality.
_KIN_WHEEL_MAX_MPS = 0.6      # linear speed at 100% commanded duty (~=0.33 m/s at 55% cap)
_KIN_WHEELBASE_M = 0.15       # track width (metres); tuning knob for turn rate
_KIN_US_RANGE_M = config.ULTRASONIC_RANGE_M
_KIN_US_HALF_CONE_DEG = 15.0
_KIN_IR_RANGE_M = 0.30
_KIN_IR_HALF_CONE_DEG = 20.0

# Fake obstacle map used by dummy_kinematic. (x, z) points in the world frame
# where the car starts at origin facing +x. Small enough to keep the geometry
# simple; can be edited via set_dummy_scenario if we want to script scenarios.
_DEFAULT_OBSTACLES = [
    (3.0, 0.5),
    (2.5, -1.2),
    (-1.5, 0.0),
    (1.0, 2.0),
]

# Angles that define each IR sensor's forward direction in the car's local
# frame (0 = straight ahead, +90 = left). Order matches config.IR_LABELS
# (FL, FR, RL, RR, L, R).
_IR_ANGLES_DEG = {
    "FL": 30.0,
    "FR": -30.0,
    "RL": 150.0,
    "RR": -150.0,
    "L": 90.0,
    "R": -90.0,
}


class SensorThread(threading.Thread):
    """Publisher thread. Dispatches to hardware / static / kinematic each tick."""

    MODE_HARDWARE = "hardware"
    MODE_DUMMY_STATIC = "dummy_static"
    MODE_DUMMY_KINEMATIC = "dummy_kinematic"
    VALID_MODES = (MODE_HARDWARE, MODE_DUMMY_STATIC, MODE_DUMMY_KINEMATIC)

    def __init__(self, hw, shared, lock, motor_thread=None):
        super().__init__(name="sensors", daemon=True)
        self.hw = hw
        self.shared = shared
        self.lock = lock
        # motor_thread is optional here so the module keeps working if imported
        # standalone; kinematic mode needs it and will log a warning if absent.
        self.motor_thread = motor_thread
        self._stop_evt = threading.Event()

        self._mode_lock = threading.Lock()
        self._mode = self.MODE_HARDWARE

        # --- Hardware init: always run at boot so runtime mode switches don't
        #     have to touch GPIO. FakeHardware makes this safe on a laptop.
        for pin in config.IR_PINS:
            hw.set_mode(pin, 0)       # INPUT
        self.us_front = _Ultrasonic(hw, config.US_FRONT_TRIG, config.US_FRONT_ECHO)
        self.us_rear = _Ultrasonic(hw, config.US_REAR_TRIG, config.US_REAR_ECHO)

        self._i2c = None
        try:
            self._i2c = hw.i2c_open(config.I2C_BUS, config.MPU6050_ADDR)
            hw.i2c_write_byte_data(self._i2c, _PWR_MGMT_1, 0x00)   # wake
            log.info("MPU6050 online at 0x%02x on i2c bus %d",
                     config.MPU6050_ADDR, config.I2C_BUS)
        except Exception as e:                                      # noqa: BLE001
            # Log the bus: opening the wrong /dev/i2c-N succeeds and only the
            # first write fails, so the bus number is the thing you actually
            # need to see here. On the Orin Nano, header pins 3/5 are bus 7.
            log.error("MPU6050 init FAILED on i2c bus %d addr 0x%02x (%s). "
                      "IMU will report zeros; with STALL_GUARD_ENABLED the "
                      "motors will stall-cut after %.1f s. Check "
                      "`i2cdetect -y %d` and config.I2C_BUS.",
                      config.I2C_BUS, config.MPU6050_ADDR, e,
                      config.STALL_TIMEOUT_S, config.I2C_BUS)

        # --- Dummy state ---------------------------------------------------
        self._scenario_lock = threading.Lock()
        # Static scenario: what the sensors "read" when in dummy_static.
        # Distances in cm to match hardware units; ir is a 6-bit mask in
        # config.IR_PINS order (== IR_LABELS order). imu is raw counts.
        self._scenario = {
            "us_front_cm": 300,
            "us_rear_cm": 300,
            "ir_mask": 0,
            "imu_ax_g": 0.0,
            "imu_ay_g": 0.0,
            "imu_az_g": 1.0,          # 1g down = at rest
            "imu_gx_dps": 0.0,
            "imu_gy_dps": 0.0,
            "imu_gz_dps": 0.0,
        }

        # Kinematic state.
        self._kin_lock = threading.Lock()
        self._kin_last_t = None
        self._kin = {
            "x": 0.0, "z": 0.0, "yaw": 0.0,
            "v": 0.0, "yaw_rate": 0.0,
        }
        self._obstacles = list(_DEFAULT_OBSTACLES)

    # -------------------------------------------------- mode + scenario API
    def set_mode(self, mode):
        if mode not in self.VALID_MODES:
            raise ValueError(f"unknown sensor mode: {mode}")
        with self._mode_lock:
            if mode == self._mode:
                return
            old = self._mode
            self._mode = mode
        # Fresh kinematic run whenever we enter kinematic mode.
        if mode == self.MODE_DUMMY_KINEMATIC:
            self.reset_kinematic()
        log.warning("SENSOR MODE %s -> %s", old, mode)

    def get_mode(self):
        with self._mode_lock:
            return self._mode

    def set_scenario(self, **fields):
        """Merge fields into the static-dummy scenario. Silently ignores keys
        the scenario doesn't know about, so the UI can be sloppy."""
        with self._scenario_lock:
            for k, v in fields.items():
                if k in self._scenario:
                    self._scenario[k] = v

    def get_scenario(self):
        with self._scenario_lock:
            return dict(self._scenario)

    def reset_kinematic(self):
        with self._kin_lock:
            self._kin_last_t = None
            self._kin = {"x": 0.0, "z": 0.0, "yaw": 0.0,
                         "v": 0.0, "yaw_rate": 0.0}

    def get_kinematic_pose(self):
        with self._kin_lock:
            return dict(self._kin)

    # -------------------------------------------------- hardware read path
    def _read_imu_hardware(self):
        if self._i2c is None:
            return {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}
        try:
            n, d = self.hw.i2c_read_i2c_block_data(self._i2c, _ACCEL_XOUT_H, 14)
            if n != 14:
                raise IOError("short read")
            return {
                "ax": _s16(d[0], d[1]),
                "ay": _s16(d[2], d[3]),
                "az": _s16(d[4], d[5]),
                "gx": _s16(d[8], d[9]),
                "gy": _s16(d[10], d[11]),
                "gz": _s16(d[12], d[13]),
            }
        except Exception:                                           # noqa: BLE001
            return {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}

    def _read_ir_hardware(self):
        mask = 0
        for bit, pin in enumerate(config.IR_PINS):
            if self.hw.read(pin):
                mask |= (1 << bit)
        return mask

    def _read_hardware(self, frame):
        imu = self._read_imu_hardware()
        ir_mask = self._read_ir_hardware()
        if frame % 2 == 0:
            us_key, us_cm = "front", self.us_front.measure_cm()
        else:
            us_key, us_cm = "rear", self.us_rear.measure_cm()
        return imu, us_key, us_cm, ir_mask

    # -------------------------------------------------- dummy read paths
    def _read_dummy_static(self, frame):
        with self._scenario_lock:
            s = dict(self._scenario)
        imu = {
            "ax": int(s["imu_ax_g"] * _ACC_COUNTS_PER_G),
            "ay": int(s["imu_ay_g"] * _ACC_COUNTS_PER_G),
            "az": int(s["imu_az_g"] * _ACC_COUNTS_PER_G),
            "gx": int(s["imu_gx_dps"] * _GYR_COUNTS_PER_DPS),
            "gy": int(s["imu_gy_dps"] * _GYR_COUNTS_PER_DPS),
            "gz": int(s["imu_gz_dps"] * _GYR_COUNTS_PER_DPS),
        }
        ir_mask = int(s["ir_mask"]) & 0x3F
        us_cm = int(s["us_front_cm"]) if frame % 2 == 0 else int(s["us_rear_cm"])
        us_key = "front" if frame % 2 == 0 else "rear"
        # None if the caller set the range beyond max — matches hardware behaviour.
        if us_cm <= 0 or us_cm > config.US_MAX_VALID_CM:
            us_cm = None
        return imu, us_key, us_cm, ir_mask

    def _read_dummy_kinematic(self, frame):
        # Advance pose from the motor thread's applied duty. If the motor
        # thread isn't wired, fall back to static so we don't drift silently.
        if self.motor_thread is None:
            return self._read_dummy_static(frame)

        now = time.monotonic()
        with self._kin_lock:
            if self._kin_last_t is None:
                self._kin_last_t = now
                dt = 0.0
            else:
                dt = min(0.2, now - self._kin_last_t)   # clamp big gaps
                self._kin_last_t = now

            left = self.motor_thread.out_left
            right = self.motor_thread.out_right
            v_l = left * _KIN_WHEEL_MAX_MPS
            v_r = right * _KIN_WHEEL_MAX_MPS
            v = (v_l + v_r) / 2.0
            yaw_rate = (v_r - v_l) / _KIN_WHEELBASE_M

            self._kin["x"] += v * math.cos(self._kin["yaw"]) * dt
            self._kin["z"] += v * math.sin(self._kin["yaw"]) * dt
            self._kin["yaw"] += yaw_rate * dt
            self._kin["v"] = v
            self._kin["yaw_rate"] = yaw_rate
            pose = dict(self._kin)

        us_front_cm = self._kin_ultrasonic_cm(pose, forward=True)
        us_rear_cm = self._kin_ultrasonic_cm(pose, forward=False)
        ir_mask = self._kin_ir_mask(pose)

        # IMU: az at rest, gz from yaw rate, ax from linear acceleration
        # (approx: dv/dt would need history — a small sinusoidal wobble reads
        # more honestly than a flat zero and doesn't fool the stall guard).
        gz_dps = math.degrees(pose["yaw_rate"])
        wobble = 0.02 * math.sin(now * 4.0)
        imu = {
            "ax": int(wobble * _ACC_COUNTS_PER_G),
            "ay": 0,
            "az": int(1.0 * _ACC_COUNTS_PER_G),
            "gx": 0,
            "gy": 0,
            "gz": int(gz_dps * _GYR_COUNTS_PER_DPS),
        }
        us_key = "front" if frame % 2 == 0 else "rear"
        us_cm = us_front_cm if frame % 2 == 0 else us_rear_cm
        return imu, us_key, us_cm, ir_mask

    def _kin_ultrasonic_cm(self, pose, forward):
        """Nearest obstacle within a ±15° cone facing forward or backward."""
        base_deg = 0.0 if forward else 180.0
        best_m = _KIN_US_RANGE_M
        for (ox, oz) in self._obstacles:
            dx, dz = ox - pose["x"], oz - pose["z"]
            dist = math.hypot(dx, dz)
            if dist > _KIN_US_RANGE_M:
                continue
            world_bearing = math.degrees(math.atan2(dz, dx))
            local_bearing = _wrap180(world_bearing - math.degrees(pose["yaw"]) - base_deg)
            if abs(local_bearing) <= _KIN_US_HALF_CONE_DEG and dist < best_m:
                best_m = dist
        if best_m >= _KIN_US_RANGE_M:
            return None
        return max(1, int(best_m * 100))

    def _kin_ir_mask(self, pose):
        mask = 0
        for bit, label in enumerate(config.IR_LABELS):
            angle = _IR_ANGLES_DEG.get(label, 0.0)
            if self._kin_ir_tripped(pose, angle):
                mask |= (1 << bit)
        return mask

    def _kin_ir_tripped(self, pose, sensor_angle_deg):
        for (ox, oz) in self._obstacles:
            dx, dz = ox - pose["x"], oz - pose["z"]
            dist = math.hypot(dx, dz)
            if dist > _KIN_IR_RANGE_M:
                continue
            world_bearing = math.degrees(math.atan2(dz, dx))
            local_bearing = _wrap180(world_bearing - math.degrees(pose["yaw"]) - sensor_angle_deg)
            if abs(local_bearing) <= _KIN_IR_HALF_CONE_DEG:
                return True
        return False

    # -------------------------------------------------- main loop
    def run(self):
        dt = 1.0 / config.SENSOR_TICK_HZ
        frame = 0
        try:
            while not self._stop_evt.is_set():
                start = time.monotonic()
                mode = self.get_mode()

                if mode == self.MODE_HARDWARE:
                    imu, us_key, us_cm, ir_mask = self._read_hardware(frame)
                elif mode == self.MODE_DUMMY_STATIC:
                    imu, us_key, us_cm, ir_mask = self._read_dummy_static(frame)
                elif mode == self.MODE_DUMMY_KINEMATIC:
                    imu, us_key, us_cm, ir_mask = self._read_dummy_kinematic(frame)
                else:
                    imu, us_key, us_cm, ir_mask = self._read_hardware(frame)

                with self.lock:
                    self.shared["imu"] = imu
                    self.shared["ir"] = ir_mask
                    us = self.shared.setdefault("us", {"front": None, "rear": None})
                    us[us_key] = us_cm
                    self.shared["t"] = time.monotonic()
                    self.shared["sensor_mode"] = mode

                frame += 1
                time.sleep(max(0.0, dt - (time.monotonic() - start)))
        finally:
            self.us_front.cancel()
            self.us_rear.cancel()
            if self._i2c is not None:
                try:
                    self.hw.i2c_close(self._i2c)
                except Exception:                                   # noqa: BLE001
                    pass

    def stop(self):
        self._stop_evt.set()


def _wrap180(deg):
    """Wrap an angle in degrees into [-180, 180]."""
    d = (deg + 180.0) % 360.0 - 180.0
    return d