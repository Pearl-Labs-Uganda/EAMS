"""config.py — ALL pins, constants, and tunables. No magic numbers elsewhere.

BCM numbering throughout.
"""

# ---------------------------------------------------------------- network ---
# "lan"  : Pi joins existing WiFi. App at http://<pi-ip>:8080  (default)
# "ap"   : Pi runs hostapd/dnsmasq (see setup/). Software behaves identically;
#          this flag exists for documentation and future conditional logic.
NETWORK_MODE = "lan"
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

# ------------------------------------------------------------- motor pins ---
ENA = 13          # left channel PWM  (hardware-PWM capable, but we use pigpio soft PWM)
ENB = 12          # right channel PWM
IN1 = 25           # left direction
IN2 = 24           # left direction
IN3 = 8          # right direction
IN4 = 27          # right direction

PWM_FREQ_HZ = 2000        # §5: 1–5 kHz allowed; 2 kHz chosen.

# ------------------------------------------------------------ safety layer ---
# §2/§5: two paralleled TT motors per L298N channel. The software IS the
# thermal protection. Do not weaken any of these.

# Hard clamp on PWM duty, applied LAST after all other math.
# ESTIMATE — stall current is unmeasured (§12.1). If per-motor stall > ~1.5 A,
# lower this. It is deliberately the single most obvious constant in the file.
DUTY_CAP = 0.55

# Ramp actual duty toward commanded duty over ~200 ms. Never step.
SLEW_FULL_SCALE_S = 0.200

# When a channel's command crosses sign: force coast (IN pins low, PWM 0)
# for this long before applying the new direction.
ZERO_CROSS_COAST_S = 0.080

# No command packet for this long -> both channels zeroed.
DEADMAN_S = 0.300

# Commanded duty > 0 but no meaningful IMU motion for this long -> cut both
# channels, latch stall flag, require explicit {"type":"reset"}.
STALL_TIMEOUT_S = 1.0

# "Meaningful motion": any gyro axis above this many raw counts (~ +/-250 dps
# full scale => 131 counts per dps; 400 counts ~ 3 dps).
STALL_GYRO_THRESHOLD_COUNTS = 400

# Motor thread / safety loop rate.
MOTOR_LOOP_HZ = 50

# TEMPORARY — IMU not yet connected (i2cdetect shows no 0x68).
# The stall guard cannot distinguish "IMU absent" from "wheels jammed", so it
# cuts every ~1 s. Setting this False removes the ONLY stall protection the
# L298N has. Wheels off the ground only. Set back to True the moment the
# MPU6050 answers at 0x68.
STALL_GUARD_ENABLED = False

# ------------------------------------------------------------ sensor pins ---
US_FRONT_TRIG = 23
US_FRONT_ECHO = 7        # 5 V -> 3.3 V divider REQUIRED on this pin
US_REAR_TRIG = 17
US_REAR_ECHO = 22         # 5 V -> 3.3 V divider REQUIRED on this pin

IR_PINS = [5, 6, 16, 19, 26, 21]   # bits 0..5 of the telemetry bitmask
                                    # dividers required if modules run at 5 V (§12.3)

I2C_BUS = 1
MPU6050_ADDR = 0x68

# --------------------------------------------------------- sensor sampling ---
SENSOR_TICK_HZ = 20       # one tick per telemetry frame
US_TIMEOUT_S = 0.030      # echo not seen in 30 ms -> report null
US_MAX_VALID_CM = 350

# ---------------------------------------------------------------- protocol ---
TELEMETRY_HZ = 20         # if WiFi stutters, drop THIS to 10 before touching
                          # the 20 Hz control rate (§9)
CMD_RATE_HZ = 20          # client-side send rate (informational; enforced in app.js)

