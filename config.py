"""All pins, constants, and tunables for the Jetson Orin Nano build.

Pin numbers use Jetson.GPIO BOARD numbering: physical 40-pin header positions,
not Raspberry Pi BCM numbers.
"""

# ---------------------------------------------------------------- network ---
NETWORK_MODE = "lan"
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

# ------------------------------------------------------------- motor pins ---
# ENA/ENB must be PWM-capable Jetson header pins. Configure them for PWM with
# Jetson-IO / pinmux tooling before expecting PWM at the header.
ENA = 33          # left channel PWM
ENB = 32          # right channel PWM
IN1 = 22          # left direction
IN2 = 18          # left direction
IN3 = 24          # right direction
IN4 = 13          # right direction

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
US_FRONT_TRIG = 16
US_FRONT_ECHO = 26       # 5 V -> 3.3 V divider REQUIRED
US_REAR_TRIG = 11
US_REAR_ECHO = 15        # 5 V -> 3.3 V divider REQUIRED

IR_PINS = [29, 31, 36, 35, 37, 40]

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
