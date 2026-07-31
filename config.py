"""All pins, constants, and tunables for the Jetson Orin Nano build.

Pin numbers use Jetson.GPIO BOARD numbering: physical 40-pin header positions,
not Raspberry Pi BCM numbers.
"""

# ---------------------------------------------------------------- network ---
NETWORK_MODE = "lan"
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

# ------------------------------------------------------------- motor pins ---
# ENA/ENB must be PWM-capable Jetson header pins. On this board jetson-io offers
# PWM on BOARD pins 15, 32 AND 33 -- confirmed on hardware 30 Jul 2026. Several
# published Orin Nano pinouts list only 15 and 33; they are wrong for our
# JetPack, so trust the board over the pinout.
#
# We use 15 and 33. They sit on separate PWM controllers (15 ->
# /sys/devices/3280000.pwm ch0, 33 -> /sys/devices/32c0000.pwm ch0) and are the
# pair the bench script proved out. Enable both with:
#     sudo /opt/nvidia/jetson-io/jetson-io.py
# and reboot before expecting PWM at the header.
#
# Leave PWM DISABLED for pin 32. It carries US_FRONT_TRIG below, and a pin muxed
# to the PWM controller ignores GPIO writes.
# Reassigned 31 Jul 2026 to match how the harness is ACTUALLY wired. Observed
# behaviour was a clean 90-degree rotation of the controls (stick left drove
# forward, stick forward pivoted right), which resolves to two independent
# harness faults: the two channels are crossed, and one pair's motor leads are
# reversed. Rather than compensate in the browser mixer -- which would leave the
# ONNX policy driving rotated, since policy.py commands motor_thread directly
# and never touches app.js -- the pin names are corrected here, below every
# consumer.
#
# The pin SET is unchanged; only which name points at which pin.
#   left  channel now uses the pins that physically drive the left pair
#   right channel now uses the pins that physically drive the right pair,
#         with IN3/IN4 deliberately in swapped numeric order to invert that
#         pair's polarity. IN3 = 22 / IN4 = 18 is NOT a typo -- see above.
ENA = 33          # left channel PWM   (PWM-capable; was 15)
ENB = 32          # right channel PWM  (PWM-capable; was 33)
IN1 = 24          # left direction     (was 18)
IN2 = 26          # left direction     (was 22)
IN3 = 22          # right direction    (was 24) -- swapped with IN4 on purpose
IN4 = 18          # right direction    (was 26) -- to reverse this pair

PWM_FREQ_HZ = 2000

# ------------------------------------------------------------ safety layer ---
# Two paralleled TT motors per L298N channel. The software is the thermal
# protection. Do not weaken these without measuring the hardware.
DUTY_CAP = 0.55
SLEW_FULL_SCALE_S = 0.200
ZERO_CROSS_COAST_S = 0.080
DEADMAN_S = 0.300
STALL_TIMEOUT_S = 1.0
STALL_GYRO_THRESHOLD_COUNTS = 400
MOTOR_LOOP_HZ = 50

# Keep enabled for floor driving. If the MPU6050 is not wired yet, set this
# False only for wheels-off-ground bring-up because it removes stall protection.
STALL_GUARD_ENABLED = False

# ------------------------------------------------------------ sensor pins ---
# Moved 29 Jul 2026: the ultrasonics previously sat on 18/22/24/26, which the
# motor direction pins now use. These four pins are the ones the motor move
# freed up, so the two groups simply traded places.
#
# WARNING: pin 32 is PWM-capable on this board and jetson-io can mux it to the
# PWM controller. It MUST be left as plain GPIO or this trigger pulse never
# reaches the header -- and the failure is silent: the sensor reads nothing,
# with no error. If the front ultrasonic goes quiet, check the pin 32 mux
# before suspecting the sensor or the wiring.
US_FRONT_TRIG = 15
US_FRONT_ECHO = 11       # 5 V -> 3.3 V divider REQUIRED
US_REAR_TRIG = 16
US_REAR_ECHO = 13        # 5 V -> 3.3 V divider REQUIRED

IR_PINS = [29, 31, 36, 37, 12, 38]

# IR_PINS[i] is the pin wired to the sensor labelled IR_LABELS[i]. This is
# also the observation order the trained policy expects — matches
# DifferentialCarAgent.CollectObservations (FL, FR, RL, RR, L, R). Physical
# wiring confirmed 27 Jul 2026; if the harness is ever re-crimped, update
# either the pins list or the labels list so they stay parallel.
IR_LABELS = ["FL", "FR", "RL", "RR", "L", "R"]

# Jetson 40-pin header I2C bus. MPU6050 SDA/SCL go to physical pins 3/5.
I2C_BUS = 1
MPU6050_ADDR = 0x68

# --------------------------------------------------------- sensor sampling ---
SENSOR_TICK_HZ = 20
US_TIMEOUT_S = 0.030
US_MAX_VALID_CM = 350

# ------------------------------------------------------------------ camera ---
# USB webcam, mounted forward on the car. Optional: if any of this fails the
# server still boots and the car stays drivable (see camera.py).
#
# The feed is SITUATIONAL AWARENESS, not a driving instrument. MJPEG over WiFi
# lands somewhere around 150-400 ms behind reality, and degrades further as the
# link does. DEADMAN_S is 0.300 -- so on a bad link the video can be a whole
# deadman period stale. Do not drive out of line of sight on camera alone.
CAMERA_ENABLED = True
CAMERA_DEVICE = 0            # /dev/video0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
CAMERA_JPEG_QUALITY = 70     # 60-75 is the sweet spot; 90+ costs bandwidth
                             # that the control path also needs in AP mode
CAMERA_IDLE_RELEASE_S = 5.0  # release the USB device when nobody is watching

# ---------------------------------------------------------------- protocol ---
TELEMETRY_HZ = 20
CMD_RATE_HZ = 20

# ---------------------------------------------------------- policy / autonomy ---
# Path (relative to server.py) of the trained ONNX policy exported from Unity
# ML-Agents. The runner opens this once at boot; changing it requires a restart.
POLICY_MODEL_PATH = "policies/DifferentialCarAgent-obstacles_v3.onnx"

# Inference rate. Match CMD_RATE_HZ — running faster than the browser client
# doesn't help because the motor safety loop rate-limits anyway.
POLICY_HZ = 20

# Observation normalisation constants MIRRORED from the Unity DifferentialCarAgent.
# If the sim is retrained with different bounds, update these together with the
# corresponding fields in DifferentialCarAgent.cs — the observation vector must
# match training or the policy silently misbehaves.
MAX_LINEAR_SPEED = 3.0        # m/s (Unity: maxLinearSpeed)
MAX_ANGULAR_SPEED = 6.0       # rad/s (Unity: maxAngularSpeed)
MAX_TARGET_DISTANCE = 20.0    # m (Unity: maxTargetDistance)
ULTRASONIC_RANGE_M = 5.0      # m (Unity: ultrasonicRange)

# Rate at which the policy runner smoothes its own action memory
# (smoothedLeft / smoothedRight — the "controller-side memory" observations the
# policy feeds back to itself). Same value as motorResponseRate in the ML-Agents
# inspector; applied per policy tick at dt = 1/POLICY_HZ.
MOTOR_RESPONSE_RATE = 6.0