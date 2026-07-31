"""USB webcam capture and MJPEG streaming.

DESIGN RULE
-----------
This subsystem is OPTIONAL and must never be able to take down the pilot stack.
A missing camera, a missing OpenCV, an unplugged USB cable, a device that
disappears mid-drive -- all of these log and retry. None of them raise into the
server, and none of them touch the motor or telemetry paths. The car stays
drivable with a dead camera; that is the whole contract.

WHY OPENCV
----------
cv2 decodes the camera's MJPEG to BGR and we re-encode to JPEG. That is
technically wasteful -- a passthrough of the camera's own MJPEG would avoid a
decode/encode round trip -- but at 640x480/15fps the cost on an Orin Nano is
negligible, and it keeps the door open for drawing overlays on the frame later
(detections, target bearing, IR state). If CPU ever becomes the constraint,
this is the place to switch to a v4l2 passthrough.

cv2 is imported lazily inside the thread. On JetPack, OpenCV is installed
system-wide, so a venv built WITHOUT --system-site-packages will not see it.
That is the most likely reason this logs "camera unavailable" on a machine that
definitely has a working webcam.
"""

import logging
import threading
import time

import config

log = logging.getLogger("camera")

BOUNDARY = "frame"


class CameraThread(threading.Thread):
    """Owns the capture device; publishes the latest JPEG to any number of viewers.

    One device, one capture loop, N viewers. Viewers never open the device
    themselves -- they block on a condition variable and are woken when a new
    frame lands, so ten browsers cost the same capture work as one.

    The device is released when nobody is watching. On a battery-powered car,
    holding a USB camera streaming into a void is a waste of both power and
    USB bandwidth.
    """

    daemon = True

    def __init__(self):
        super().__init__(name="camera", daemon=True)
        self._cv = threading.Condition()
        self._frame = None            # latest encoded JPEG bytes
        self._seq = 0                 # increments per frame; viewers track it
        self._viewers = 0
        self._stop = threading.Event()

        # status, read by /camera/status -- deliberately not on the telemetry
        # WebSocket, so camera trouble cannot perturb the control frame
        self._state = "idle"          # idle | opening | streaming | error
        self._error = None
        self._fps = 0.0

    # ------------------------------------------------------------- viewers
    def _viewer_join(self):
        with self._cv:
            self._viewers += 1
            self._cv.notify_all()

    def _viewer_leave(self):
        with self._cv:
            self._viewers = max(0, self._viewers - 1)

    # ---------------------------------------------------------------- run
    def run(self):
        try:
            import cv2
        except ImportError as e:
            self._state = "error"
            self._error = (f"OpenCV not importable ({e}). On JetPack, cv2 is a "
                           f"system package -- rebuild the venv with "
                           f"--system-site-packages, or pip install "
                           f"opencv-python-headless.")
            log.error("camera disabled: %s", self._error)
            return

        backoff = 1.0
        while not self._stop.is_set():
            # Idle until somebody is actually watching.
            with self._cv:
                while self._viewers == 0 and not self._stop.is_set():
                    self._state = "idle"
                    self._cv.wait(timeout=1.0)
            if self._stop.is_set():
                break

            cap = None
            try:
                self._state = "opening"
                cap = cv2.VideoCapture(config.CAMERA_DEVICE, cv2.CAP_V4L2)
                # Ask the camera to deliver MJPEG rather than raw YUV: raw at
                # 640x480 saturates USB 2.0 bandwidth and caps the frame rate.
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
                cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
                # Smallest driver-side buffer we can ask for. A deep buffer
                # trades latency for smoothness, and on a moving vehicle stale
                # frames are worse than dropped ones.
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:                            # noqa: BLE001
                    pass

                if not cap.isOpened():
                    raise RuntimeError(
                        f"could not open camera device {config.CAMERA_DEVICE}")

                log.info("camera open: device=%s %dx%d target %d fps",
                         config.CAMERA_DEVICE, config.CAMERA_WIDTH,
                         config.CAMERA_HEIGHT, config.CAMERA_FPS)
                self._state = "streaming"
                self._error = None
                backoff = 1.0

                enc = [int(cv2.IMWRITE_JPEG_QUALITY), config.CAMERA_JPEG_QUALITY]
                idle_since = None
                fps_t0, fps_n = time.monotonic(), 0

                while not self._stop.is_set():
                    if self._viewers == 0:
                        # Grace period before tearing the device down, so a
                        # page reload does not cause an open/close cycle.
                        idle_since = idle_since or time.monotonic()
                        if time.monotonic() - idle_since > config.CAMERA_IDLE_RELEASE_S:
                            log.info("camera idle, releasing device")
                            break
                        time.sleep(0.1)
                        continue
                    idle_since = None

                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError("frame read failed (device gone?)")

                    ok, buf = cv2.imencode(".jpg", frame, enc)
                    if not ok:
                        continue

                    with self._cv:
                        self._frame = buf.tobytes()
                        self._seq += 1
                        self._cv.notify_all()

                    fps_n += 1
                    dt = time.monotonic() - fps_t0
                    if dt >= 2.0:
                        self._fps = fps_n / dt
                        fps_t0, fps_n = time.monotonic(), 0

            except Exception as e:                            # noqa: BLE001
                self._state = "error"
                self._error = str(e)
                self._fps = 0.0
                log.warning("camera error: %s (retry in %.0fs)", e, backoff)
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 15.0)
            finally:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:                         # noqa: BLE001
                        pass
                with self._cv:
                    self._frame = None
                    self._cv.notify_all()

        self._state = "idle"
        log.info("camera thread stopped")

    # ------------------------------------------------------------- output
    def latest(self, timeout=5.0):
        """Block for the next frame. Returns bytes, or None on timeout."""
        with self._cv:
            start_seq = self._seq
            if not self._cv.wait_for(lambda: self._seq != start_seq
                                     and self._frame is not None,
                                     timeout=timeout):
                return None
            return self._frame

    def snapshot(self, timeout=5.0):
        """One JPEG, for a still grab. Joins as a viewer so the device opens."""
        self._viewer_join()
        try:
            return self.latest(timeout=timeout)
        finally:
            self._viewer_leave()

    def mjpeg(self):
        """multipart/x-mixed-replace generator. One per connected viewer.

        Werkzeug raises into this generator when the browser goes away, so the
        finally clause is what actually decrements the viewer count -- without
        it the device would never idle down.
        """
        self._viewer_join()
        try:
            while not self._stop.is_set():
                frame = self.latest(timeout=5.0)
                if frame is None:
                    continue                 # camera reopening; hold the socket
                yield (b"--" + BOUNDARY.encode() + b"\r\n"
                       b"Content-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                       + frame + b"\r\n")
        finally:
            self._viewer_leave()

    def status(self):
        return {
            "state": self._state,
            "error": self._error,
            "fps": round(self._fps, 1),
            "viewers": self._viewers,
            "width": config.CAMERA_WIDTH,
            "height": config.CAMERA_HEIGHT,
        }

    def stop(self):
        self._stop.set()
        with self._cv:
            self._cv.notify_all()
