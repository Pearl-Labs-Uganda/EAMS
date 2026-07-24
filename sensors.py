"""Sensor thread for the Jetson Orin Nano RC car build.

Samples all sensors on a fixed tick and writes the latest values into a shared
dict under a lock. It never touches motor pins and never blocks the socket
thread.
"""

import logging
import threading
import time

import config

log = logging.getLogger("sensors")

_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT_H = 0x3B


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


class SensorThread(threading.Thread):
    def __init__(self, hw, shared, lock):
        super().__init__(name="sensors", daemon=True)
        self.hw = hw
        self.shared = shared
        self.lock = lock
        self._stop_evt = threading.Event()

        for pin in config.IR_PINS:
            hw.set_mode(pin, 0)       # INPUT

        self.us_front = _Ultrasonic(hw, config.US_FRONT_TRIG, config.US_FRONT_ECHO)
        self.us_rear = _Ultrasonic(hw, config.US_REAR_TRIG, config.US_REAR_ECHO)

        self._i2c = None
        try:
            self._i2c = hw.i2c_open(config.I2C_BUS, config.MPU6050_ADDR)
            hw.i2c_write_byte_data(self._i2c, _PWR_MGMT_1, 0x00)   # wake
            log.info("MPU6050 online at 0x%02x", config.MPU6050_ADDR)
        except Exception as e:                                      # noqa: BLE001
            log.warning("MPU6050 init failed (%s); IMU zeros, stall guard "
                        "will not clear motion", e)

    def _read_imu(self):
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

    def _read_ir(self):
        mask = 0
        for bit, pin in enumerate(config.IR_PINS):
            if self.hw.read(pin):
                mask |= (1 << bit)
        return mask

    def run(self):
        dt = 1.0 / config.SENSOR_TICK_HZ
        frame = 0
        try:
            while not self._stop_evt.is_set():
                start = time.monotonic()

                imu = self._read_imu()
                ir = self._read_ir()

                if frame % 2 == 0:
                    distance = self.us_front.measure_cm()
                    key = "front"
                else:
                    distance = self.us_rear.measure_cm()
                    key = "rear"

                with self.lock:
                    self.shared["imu"] = imu
                    self.shared["ir"] = ir
                    us = self.shared.setdefault("us", {"front": None, "rear": None})
                    us[key] = distance
                    self.shared["t"] = time.monotonic()

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
