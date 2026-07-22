"""Flask + flask-sock entry point for the Jetson Orin Nano RC car.

Threads:
  1. web/socket thread - serves static/, owns the WebSocket
  2. SensorThread - writes latest values into shared under lock
  3. MotorThread - 50 Hz safety loop; the only code touching motor pins

Any number of clients may watch telemetry, but at most one holds control.
The controller must send {"type":"take"} before driving and may send
{"type":"release"} to give control up. Server-side checks drop drive commands
from non-controllers.
"""

import argparse
import atexit
import json
import logging
import math
import signal
import sys
import threading
import time

from flask import Flask, send_from_directory
from flask_sock import Sock

import config
from hardware import JetsonHardware
from motors import MotorThread
from sensors import SensorThread

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    stream=sys.stdout,
)
log = logging.getLogger("server")


class FakeHardware:
    """Enough of the Jetson hardware surface for --dry-run on a laptop."""

    connected = True

    def set_mode(self, *a): pass
    def write(self, *a): pass
    def read(self, pin): return 0
    def set_PWM_frequency(self, *a): pass
    def set_PWM_range(self, *a): pass
    def set_PWM_dutycycle(self, *a): pass
    def gpio_trigger(self, *a): pass
    def i2c_open(self, *a): return 0
    def i2c_write_byte_data(self, *a): pass

    def i2c_read_i2c_block_data(self, h, reg, n):
        t = time.monotonic()
        g = int(800 * math.sin(t))
        d = [0, 0, 0, 0, 0x40, 0x00, 0, 0,
             (g >> 8) & 0xFF, g & 0xFF, 0, 0, 0, 0]
        return 14, bytearray(d)

    def i2c_close(self, *a): pass
    def stop(self): pass


def make_hardware(dry_run):
    if dry_run:
        log.warning("DRY RUN - GPIO stubbed, no hardware will be touched")
        return FakeHardware()
    try:
        return JetsonHardware()
    except Exception as e:                                      # noqa: BLE001
        log.error("cannot initialize Jetson GPIO/I2C backend: %s", e)
        sys.exit(1)


app = Flask(__name__, static_folder=None)
sock = Sock(app)

shared = {}
lock = threading.Lock()
motor_thread = None

_ctrl_lock = threading.Lock()
_controller = None
_next_conn_id = 0


def _take_control(conn_id):
    """Grant control iff nobody holds it. Returns True on success."""
    global _controller
    with _ctrl_lock:
        if _controller is not None:
            return False
        _controller = conn_id
    motor_thread.new_controller()
    log.info("client %d took control", conn_id)
    return True


def _release_control(conn_id, reason):
    """Release control iff conn_id holds it. Safe to call unconditionally."""
    global _controller
    with _ctrl_lock:
        if _controller != conn_id:
            return
        _controller = None
    motor_thread.new_controller()
    log.info("client %d released control (%s)", conn_id, reason)


def _has_control(conn_id):
    with _ctrl_lock:
        return _controller == conn_id


def _control_state(conn_id):
    with _ctrl_lock:
        held = _controller is not None
        return {"held": held, "mine": _controller == conn_id}


@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/<path:path>")
def static_files(path):
    return send_from_directory("static", path)


def _telemetry_frame(conn_id):
    with lock:
        imu = shared.get("imu", {"ax": 0, "ay": 0, "az": 0,
                                 "gx": 0, "gy": 0, "gz": 0})
        us = dict(shared.get("us", {"front": None, "rear": None}))
        ir = shared.get("ir", 0)
    m = motor_thread
    return json.dumps({
        "t": time.monotonic(),
        "imu": imu,
        "us": us,
        "ir": ir,
        "motor": {"left": round(m.out_left, 3), "right": round(m.out_right, 3),
                  "capped": m.capped, "stall": m.stall},
        "dead": m.dead,
        "ctrl": _control_state(conn_id),
    })


def _telemetry_sender(ws, stop_evt, conn_id):
    dt = 1.0 / config.TELEMETRY_HZ
    try:
        while not stop_evt.is_set():
            ws.send(_telemetry_frame(conn_id))
            time.sleep(dt)
    except Exception:                                   # noqa: BLE001
        pass


def _valid_axis(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


@sock.route("/ws")
def ws_handler(ws):
    global _next_conn_id
    with _ctrl_lock:
        _next_conn_id += 1
        conn_id = _next_conn_id
    log.info("client %d connected (viewer - no control yet)", conn_id)

    stop_evt = threading.Event()
    sender = threading.Thread(target=_telemetry_sender,
                              args=(ws, stop_evt, conn_id), daemon=True)
    sender.start()
    try:
        while True:
            raw = ws.receive()
            if raw is None:
                break
            try:
                msg = json.loads(raw)
            except (ValueError, TypeError):
                continue
            if not isinstance(msg, dict):
                continue
            mtype = msg.get("type")

            if mtype == "cmd":
                if not _has_control(conn_id):
                    continue
                left, right, seq = msg.get("left"), msg.get("right"), msg.get("seq")
                if not (_valid_axis(left) and _valid_axis(right)
                        and isinstance(seq, int)):
                    continue
                left = max(-1.0, min(1.0, float(left)))
                right = max(-1.0, min(1.0, float(right)))
                motor_thread.set_command(left, right, seq)
            elif mtype == "take":
                if not _take_control(conn_id):
                    log.info("client %d asked for control - already held", conn_id)
            elif mtype == "release":
                _release_control(conn_id, "released by client")
            elif mtype == "reset":
                if _has_control(conn_id):
                    motor_thread.reset()
    finally:
        stop_evt.set()
        _release_control(conn_id, "disconnect")
        log.info("client %d disconnected - deadman will zero motors", conn_id)


def main():
    global motor_thread

    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="stub GPIO; run the full stack with no hardware")
    args = ap.parse_args()

    hw = make_hardware(args.dry_run)

    motor_thread = MotorThread(hw, shared, lock)
    sensor_thread = SensorThread(hw, shared, lock)

    did_shutdown = False

    def shutdown(*_):
        nonlocal did_shutdown
        if did_shutdown:
            return
        did_shutdown = True
        log.info("shutting down - zeroing motors")
        motor_thread.shutdown()
        sensor_thread.stop()
        try:
            hw.stop()
        except Exception:                               # noqa: BLE001
            pass

    atexit.register(shutdown)
    signal.signal(signal.SIGTERM, lambda *_: (shutdown(), sys.exit(0)))
    signal.signal(signal.SIGINT, lambda *_: (shutdown(), sys.exit(0)))

    motor_thread.start()
    sensor_thread.start()

    log.info("mode=%s  http://0.0.0.0:%d", config.NETWORK_MODE, config.HTTP_PORT)
    app.run(host=config.HTTP_HOST, port=config.HTTP_PORT, threaded=True)


if __name__ == "__main__":
    main()
