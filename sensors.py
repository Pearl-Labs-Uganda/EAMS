"""sensors.py — sensor thread.

Samples everything on a fixed SENSOR_TICK_HZ tick and writes the latest
values into a shared dict under a lock. Never touches motor pins, never
blocks the socket thread.

Ultrasonics fire STRICTLY alternately: front on even ticks, rear on odd
(~10 Hz each). Echo timing uses pi.callback() with pigpio hardware ticks —
never time.time() polling. Timeouts report None (client shows "—").

IMU ships RAW register counts, unscaled, unfiltered. The browser scales.
"""

import logging
import threading
import time

import config

log = logging.getLogger("sensors")

# MPU6050 registers
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT_H = 0x3B


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class _Ultrasonic:
    """One US-100 in trigger/echo mode, timed with pigpio edge callbacks."""

    def __init__(self, pi, trig, echo):
        self.pi = pi
        self.trig = trig
        self.echo = echo
        self._rise = None
        self._width_us = None
        self._done = threading.Event()
        pi.set_mode(trig, 1)          # OUTPUT
        pi.set_mode(echo, 0)          # INPUT
        pi.write(trig, 0)
        self._cb = pi.callback(echo, 2, self._edge)   # 2 = EITHER_EDGE

    def _edge(self, gpio, level, tick):
        if level == 1:
            self._rise = tick
        elif level == 0 and self._rise is not None:
            # pigpio ticks are unsigned 32-bit µs; handle wrap
            self._width_us = (tick - self._rise) & 0xFFFFFFFF
            self._done.set()

    def measure_cm(self):
        """Blocking single shot; returns int cm or None on timeout/garbage."""
        self._rise = None
        self._width_us = None
        self._done.clear()
        self.pi.gpio_trigger(self.trig, 10, 1)        # 10 µs pulse
        if not self._done.wait(config.US_TIMEOUT_S):
            return None
        cm = self._width_us / 58.0
        if cm <= 0 or cm > config.US_MAX_VALID_CM:
            return None
        return int(round(cm))

    def cancel(self):
        self._cb.cancel()


class SensorThread(threading.Thread):
    def __init__(self, pi, shared, lock):
        super().__init__(name="sensors", daemon=True)
        self.pi = pi
        self.shared = shared
        self.lock = lock
        self._stop = threading.Event()

        for pin in config.IR_PINS:
            pi.set_mode(pin, 0)       # INPUT

        self.us_front = _Ultrasonic(pi, config.US_FRONT_TRIG, config.US_FRONT_ECHO)
        self.us_rear = _Ultrasonic(pi, config.US_REAR_TRIG, config.US_REAR_ECHO)

        self._i2c = None
        try:
            self._i2c = pi.i2c_open(config.I2C_BUS, config.MPU6050_ADDR)
            pi.i2c_write_byte_data(self._i2c, _PWR_MGMT_1, 0x00)   # wake
            log.info("MPU6050 online at 0x%02x", config.MPU6050_ADDR)
        except Exception as e:                                      # noqa: BLE001
            log.warning("MPU6050 init failed (%s); IMU zeros, stall guard "
                        "will not clear motion", e)

    def _read_imu(self):
        if self._i2c is None:
            return {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}
        try:
            n, d = self.pi.i2c_read_i2c_block_data(self._i2c, _ACCEL_XOUT_H, 14)
            if n != 14:
                raise IOError("short read")
            return {
                "ax": _s16(d[0], d[1]), "ay": _s16(d[2], d[3]), "az": _s16(d[4], d[5]),
                # d[6:8] is temperature — skipped
                "gx": _s16(d[8], d[9]), "gy": _s16(d[10], d[11]), "gz": _s16(d[12], d[13]),
            }
        except Exception:                                           # noqa: BLE001
            return {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}

    def _read_ir(self):
        mask = 0
        for bit, pin in enumerate(config.IR_PINS):
            if self.pi.read(pin):
                mask |= (1 << bit)
        return mask

    def run(self):
        dt = 1.0 / config.SENSOR_TICK_HZ
        frame = 0
        try:
            while not self._stop.is_set():
                start = time.monotonic()

                imu = self._read_imu()
                ir = self._read_ir()

                # strictly alternate — never fire both (cross-talk)
                if frame % 2 == 0:
                    d = self.us_front.measure_cm()
                    key = "front"
                else:
                    d = self.us_rear.measure_cm()
                    key = "rear"

                with self.lock:
                    self.shared["imu"] = imu
                    self.shared["ir"] = ir
                    us = self.shared.setdefault("us", {"front": None, "rear": None})
                    us[key] = d
                    self.shared["t"] = time.monotonic()

                frame += 1
                time.sleep(max(0.0, dt - (time.monotonic() - start)))
        finally:
            self.us_front.cancel()
            self.us_rear.cancel()
            if self._i2c is not None:
                try:
                    self.pi.i2c_close(self._i2c)
                except Exception:                                   # noqa: BLE001
                    pass

    def stop(self):
        self._stop.set()
