"""Hardware backend for Jetson Orin Nano.

The rest of the application expects a small GPIO/PWM/I2C surface that looks
enough like the old pigpio handle to keep the safety architecture unchanged.
Jetson.GPIO is used in BOARD numbering mode because the 40-pin header is the
stable thing you wire to.
"""

import time

import config


class JetsonHardware:
    """GPIO, PWM, and I2C adapter for Jetson Orin Nano."""

    connected = True

    def __init__(self):
        import Jetson.GPIO as GPIO
        from smbus2 import SMBus

        self.GPIO = GPIO
        self.SMBus = SMBus
        self._pwm = {}
        self._pwm_freq = {}       # requested freq, may be set before the object
        self._i2c = {}

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

    def set_mode(self, pin, mode):
        if mode == 1:
            self.GPIO.setup(pin, self.GPIO.OUT, initial=self.GPIO.LOW)
        else:
            self.GPIO.setup(pin, self.GPIO.IN)

    def write(self, pin, value):
        self.GPIO.output(pin, self.GPIO.HIGH if value else self.GPIO.LOW)

    def read(self, pin):
        return int(self.GPIO.input(pin))

    def set_PWM_frequency(self, pin, hz):
        # Record it even if the PWM object does not exist yet, so a call made
        # during setup is not silently discarded (it used to be).
        self._pwm_freq[pin] = hz
        pwm = self._pwm.get(pin)
        if pwm is not None:
            pwm.ChangeFrequency(hz)

    def set_PWM_range(self, pin, pwm_range):
        if pwm_range != 255:
            raise ValueError("Jetson backend expects 0..255 PWM duty values")
        if pin not in self._pwm:
            # Jetson.GPIO ordering quirk: a PWM pin's GPIO.setup() must come
            # IMMEDIATELY before its GPIO.PWM() construction. If both enable
            # pins are set up as plain outputs first, only the last PWM object
            # created actually drives its pin -- the other sits at a fixed
            # level. On an L298N enable line that is a channel which is either
            # dead or stuck fully enabled, and "stuck enabled" bypasses
            # DUTY_CAP. The stall guard cannot save us here either: a stuck-
            # enabled pin still spins the wheels, so the IMU sees motion.
            # Doing the setup here keeps the pairing correct for any caller.
            #
            # initial=LOW is deliberate. The published workaround uses HIGH,
            # but LOW means "channel off" on an H-bridge enable pin, which is
            # the safe state to boot into.
            self.GPIO.setup(pin, self.GPIO.OUT, initial=self.GPIO.LOW)
            pwm = self.GPIO.PWM(pin, self._pwm_freq.get(pin, config.PWM_FREQ_HZ))
            pwm.start(0)
            self._pwm[pin] = pwm

    def set_PWM_dutycycle(self, pin, duty):
        pwm = self._pwm.get(pin)
        if pwm is None:
            self.set_PWM_range(pin, 255)
            pwm = self._pwm[pin]
        pwm.ChangeDutyCycle(max(0.0, min(100.0, (float(duty) / 255.0) * 100.0)))

    def gpio_trigger(self, pin, pulse_us, level):
        self.write(pin, level)
        time.sleep(pulse_us / 1_000_000.0)
        self.write(pin, 0 if level else 1)

    def i2c_open(self, bus, addr):
        handle = len(self._i2c) + 1
        self._i2c[handle] = {"bus": self.SMBus(bus), "addr": addr}
        return handle

    def i2c_write_byte_data(self, handle, reg, value):
        item = self._i2c[handle]
        item["bus"].write_byte_data(item["addr"], reg, value)

    def i2c_read_i2c_block_data(self, handle, reg, length):
        item = self._i2c[handle]
        data = item["bus"].read_i2c_block_data(item["addr"], reg, length)
        return len(data), bytearray(data)

    def i2c_close(self, handle):
        item = self._i2c.pop(handle, None)
        if item is not None:
            item["bus"].close()

    def stop(self):
        for pwm in self._pwm.values():
            pwm.stop()
        self._pwm.clear()
        for handle in list(self._i2c):
            self.i2c_close(handle)
        self.GPIO.cleanup()