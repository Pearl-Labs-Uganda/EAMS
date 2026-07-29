"""Flask + flask-sock entry point for the Jetson Orin Nano RC car.

Threads:
  1. web/socket thread - serves static/, owns the WebSocket
  2. SensorThread - writes latest values into shared under lock, mode-dispatched
                    (hardware / dummy_static / dummy_kinematic)
  3. MotorThread - 50 Hz safety loop; the only code touching motor pins.
                    Physical output can be toggled off at runtime (dummy motors).
  4. PolicyRunner - 20 Hz ONNX inference. When engaged, commands motors through
                    the same set_command interface the browser uses.

Any number of clients may watch telemetry, but at most one holds control.
While the policy is engaged, human take/release is refused — the policy IS
the driver until it disengages. All safety limits (deadman, stall latch, slew,
zero-cross, duty cap) apply to policy commands identically to browser commands.
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
sensor_thread = None
policy_runner = None            # optional; set only if policy loads cleanly

_ctrl_lock = threading.Lock()
_controller = None
_next_conn_id = 0


# --------------------------------------------------------- control switch
def _take_control(conn_id):
    """Grant control iff nobody holds it AND the policy isn't engaged."""
    global _controller
    if policy_runner is not None and policy_runner.engaged:
        return False
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


# --------------------------------------------------------- policy engage
def _policy_engage_from_client(wheels_off_ground):
    """Client requested policy engagement. Returns (ok, reason)."""
    if policy_runner is None:
        return False, "policy runner not loaded on server"
    with _ctrl_lock:
        if _controller is not None:
            return False, "someone else is driving; release first"
    if motor_thread.output_enabled and not wheels_off_ground:
        return False, "confirm wheels off the ground before engaging on real motors"
    if policy_runner.engage():
        return True, "engaged"
    return False, "already engaged"


def _policy_disengage_from_client():
    if policy_runner is None:
        return False, "policy runner not loaded on server"
    if policy_runner.disengage():
        return True, "disengaged"
    return False, "not engaged"


# --------------------------------------------------------- HTTP routes
@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/<path:path>")
def static_files(path):
    return send_from_directory("static", path)


# --------------------------------------------------------- telemetry
def _telemetry_frame(conn_id):
    with lock:
        imu = shared.get("imu", {"ax": 0, "ay": 0, "az": 0,
                                 "gx": 0, "gy": 0, "gz": 0})
        us = dict(shared.get("us", {"front": None, "rear": None}))
        ir = shared.get("ir", 0)
        sensor_mode = shared.get("sensor_mode", "hardware")
    m = motor_thread
    frame = {
        "t": time.monotonic(),
        "imu": imu,
        "us": us,
        "ir": ir,
        "motor": {"left": round(m.out_left, 3), "right": round(m.out_right, 3),
                  "capped": m.capped, "stall": m.stall,
                  "output_enabled": m.output_enabled},
        "dead": m.dead,
        "ctrl": _control_state(conn_id),
        "sensor_mode": sensor_mode,
    }
    if policy_runner is not None:
        frame["policy"] = policy_runner.snapshot()
        frame["policy_loaded"] = True
    else:
        frame["policy_loaded"] = False
    if sensor_thread is not None:
        frame["scenario"] = sensor_thread.get_scenario()
        if sensor_mode == SensorThread.MODE_DUMMY_KINEMATIC:
            frame["kin_pose"] = sensor_thread.get_kinematic_pose()
    return json.dumps(frame)


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


def _reply(ws, kind, ok, reason=None, extra=None):
    """Send a small ack back for lab-panel actions so the UI can show a
    result without waiting for the next telemetry frame."""
    body = {"type": "ack", "kind": kind, "ok": ok}
    if reason is not None:
        body["reason"] = reason
    if extra:
        body.update(extra)
    try:
        ws.send(json.dumps(body))
    except Exception:                                   # noqa: BLE001
        pass


# --------------------------------------------------------- WebSocket
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

            # ---- pilot commands (unchanged, plus policy interlock) ----
            if mtype == "cmd":
                if policy_runner is not None and policy_runner.engaged:
                    continue                 # policy owns the motors
                if not _has_control(conn_id):
                    continue
                left, right, seq = (msg.get("left"), msg.get("right"),
                                    msg.get("seq"))
                if not (_valid_axis(left) and _valid_axis(right)
                        and isinstance(seq, int)):
                    continue
                left = max(-1.0, min(1.0, float(left)))
                right = max(-1.0, min(1.0, float(right)))
                motor_thread.set_command(left, right, seq)
            elif mtype == "take":
                if not _take_control(conn_id):
                    log.info("client %d asked for control - refused", conn_id)
            elif mtype == "release":
                _release_control(conn_id, "released by client")
            elif mtype == "reset":
                if _has_control(conn_id):
                    motor_thread.reset()

            # ---- Autonomy Lab ------------------------------------------
            elif mtype == "set_sensor_mode":
                mode = msg.get("mode")
                if mode in SensorThread.VALID_MODES:
                    sensor_thread.set_mode(mode)
                    _reply(ws, mtype, True, extra={"mode": mode})
                else:
                    _reply(ws, mtype, False, reason=f"unknown mode {mode!r}")

            elif mtype == "set_motor_output":
                enabled = bool(msg.get("enabled"))
                # Refuse to disable while the policy is actively engaged on
                # real hardware — that would leave the loop half-live and is
                # surprising. Disengage the policy first, then flip motors.
                if (not enabled) and policy_runner is not None \
                        and policy_runner.engaged \
                        and motor_thread.output_enabled:
                    _reply(ws, mtype, False,
                           reason="disengage the policy before disabling motor output")
                else:
                    motor_thread.set_output_enabled(enabled)
                    _reply(ws, mtype, True, extra={"enabled": enabled})

            elif mtype == "set_target":
                if policy_runner is None:
                    _reply(ws, mtype, False, reason="no policy loaded")
                    continue
                d = msg.get("distance_m")
                b = msg.get("bearing_deg")
                if not (_valid_axis(d) and _valid_axis(b)):
                    _reply(ws, mtype, False, reason="bad numeric fields")
                    continue
                policy_runner.set_target(float(d), float(b))
                _reply(ws, mtype, True,
                       extra={"target": {"distance_m": float(d),
                                          "bearing_deg": float(b)}})

            elif mtype == "set_scenario":
                fields = msg.get("fields") or {}
                if not isinstance(fields, dict):
                    _reply(ws, mtype, False, reason="fields must be a dict")
                    continue
                sensor_thread.set_scenario(**fields)
                _reply(ws, mtype, True,
                       extra={"scenario": sensor_thread.get_scenario()})

            elif mtype == "reset_kinematic":
                sensor_thread.reset_kinematic()
                _reply(ws, mtype, True)

            elif mtype == "policy_engage":
                wheels_off = bool(msg.get("wheels_off_ground"))
                ok, reason = _policy_engage_from_client(wheels_off)
                _reply(ws, mtype, ok, reason=reason)

            elif mtype == "policy_disengage":
                ok, reason = _policy_disengage_from_client()
                _reply(ws, mtype, ok, reason=reason)

    finally:
        stop_evt.set()
        _release_control(conn_id, "disconnect")
        log.info("client %d disconnected - deadman will zero motors", conn_id)


# --------------------------------------------------------- main
def main():
    global motor_thread, sensor_thread, policy_runner

    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="stub GPIO; run the full stack with no hardware")
    ap.add_argument("--sensor-mode",
                    choices=SensorThread.VALID_MODES,
                    default=SensorThread.MODE_HARDWARE,
                    help="initial sensor mode (Autonomy Lab can switch at runtime)")
    ap.add_argument("--motor-output", choices=("real", "dummy"), default="real",
                    help="initial motor output mode (Autonomy Lab can switch)")
    ap.add_argument("--no-policy", action="store_true",
                    help="skip loading the ONNX policy even if the file exists")
    args = ap.parse_args()

    hw = make_hardware(args.dry_run)

    motor_thread = MotorThread(hw, shared, lock)
    if args.motor_output == "dummy":
        motor_thread.set_output_enabled(False)

    sensor_thread = SensorThread(hw, shared, lock, motor_thread=motor_thread)
    sensor_thread.set_mode(args.sensor_mode)

    # Policy runner: optional. Missing ONNX or onnxruntime should NOT stop the
    # server booting — the pilot stack must always come up.
    if not args.no_policy:
        try:
            # Local import so onnxruntime is only required if the policy is used.
            from policy import PolicyRunner
            policy_runner = PolicyRunner(motor_thread, shared, lock)
        except Exception as e:                              # noqa: BLE001
            log.error("policy runner not loaded (%s); pilot stack up, "
                      "Autonomy Lab will show as unavailable", e)
            policy_runner = None

    did_shutdown = False

    def shutdown(*_):
        nonlocal did_shutdown
        if did_shutdown:
            return
        did_shutdown = True
        log.info("shutting down - zeroing motors")
        if policy_runner is not None:
            policy_runner.disengage()
            policy_runner.stop()
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
    if policy_runner is not None:
        policy_runner.start()

    log.info("mode=%s  sensor_mode=%s  motor_output=%s  policy=%s  "
             "http://%s:%d",
             config.NETWORK_MODE, args.sensor_mode,
             "real" if motor_thread.output_enabled else "dummy",
             "loaded" if policy_runner is not None else "unavailable",
             config.HTTP_HOST, config.HTTP_PORT)
    app.run(host=config.HTTP_HOST, port=config.HTTP_PORT, threaded=True)


if __name__ == "__main__":
    main()
