"""All pins, constants, and tunables for the Jetson Orin Nano build.

Pin numbers use Jetson.GPIO BOARD numbering: physical 40-pin header positions,
not Raspberry Pi BCM numbers.
"""

# ---------------------------------------------------------------- network ---
NETWORK_MODE = "lan"
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

# ------------------------------------------------------------- motor pins ---
# ENA/ENB must be PWM-capable Jetson header pins. On the Orin Nano ONLY pins 15
# and 33 are PWM-capable (pin 15 -> /sys/devices/3280000.pwm ch0, pin 33 ->
# /sys/devices/32c0000.pwm ch0). Pin 32 is PWM on the older Nano / Xavier NX but
# NOT on Orin -- ENA lived there until 29 Jul 2026, which meant PWM was assigned
# to a pin that could not produce it. Enable both with:
#     sudo /opt/nvidia/jetson-io/jetson-io.py
# and reboot before expecting PWM at the header.
ENA = 15          # left channel PWM   (PWM-capable)
ENB = 33          # right channel PWM  (PWM-capable)
IN1 = 18          # left direction
IN2 = 22          # left direction
IN3 = 24          # right direction
IN4 = 26          # right direction

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
STALL_GUARD_ENABLED = True

# ------------------------------------------------------------ sensor pins ---
# Moved 29 Jul 2026: the ultrasonics previously sat on 18/22/24/26, which the
# motor direction pins now use. These four pins are the ones the motor move
# freed up, so the two groups simply traded places. TRIG is on pin 32 because
# it is the pin we have least confidence in and a 10 us output pulse is the
# least demanding job on the header; if the front ultrasonic misbehaves, pin 32
# is the first suspect.
US_FRONT_TRIG = 32
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