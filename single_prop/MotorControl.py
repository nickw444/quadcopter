from RPIO import PWM
PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)

class MotorControl(object):
    def __init__(self, pins, pwm_max=2000, pwm_min=1000, debug=False):
        self.pwm_max = pwm_max
        self.pwm_min = pwm_min
        self.pwm_range = pwm_max - pwm_min
        self.pins = pins
        self.is_setup = False

        self.servo = PWM.Servo(pulse_incr_us=1)
        self.debug = debug

    def _print(self, message):
        if self.debug:
            print(message)

    def begin_calibration(self):
        self._print("Starting Calibration")
        for pin in self.pins:
            self.servo.set_servo(pin, self.pwm_max)
        
        self._print("Calibration Needs to continue. Connect motors to power"\
            " and call 'continue_calibration'")

    def continue_calibration(self):
        self._print("Continuing Calibration")

        for pin in self.pins:
            self.servo.set_servo(pin, self.pwm_min)

        self._print("Calibration is now complete.")
        self.is_setup = True
        return True

    def set_motor(self, motor, percentage):
        self._check_setup()
        self._check_valid_pins([motor])

        pwmspeed = (percentage/float(100) * self.pwm_range) + self.pwm_min
        pin = self.pins[motor]
        self._print("Setting motor: {} (pin {}) to {}% (PWM: {})".format(motor, pin, percentage, pwmspeed))
        self.servo.set_servo(pin, pwmspeed)

    def set_motors(self, motors, percentage):
        self._check_setup()
        self._check_valid_pins(motors)

        pwmspeed = (percentage/float(100) * self.pwm_range) + self.pwm_min

        for motor in motors:
            pin = self.pins[motor]
            self.servo.set_servo(pin, pwmspeed)
            self._print("Setting motor: {} (pin {}) to {}% (PWM: {})".format(motor, pin, percentage, pwmspeed))

    def cleanup(self):
        self._print("Cleaning up...")

        for pin in self.pins:
            self.servo.set_servo(pin, self.pwm_min)
            self.servo.stop_servo(pin)

        PWM.cleanup()
        self._print("Cleaned.")


    def _check_setup(self):
        if not self.is_setup:
            raise Exception("Please call 'begin_calibration', connect motors then call"\
                " 'continue_calibration'.")

    def _check_valid_pins(self, motors):
        max_element = len(self.pins)
        for motor in motors:
            if motor >= max_element:
                raise Exception("Attempting to control a motor that is not configured. "\
                    "Asking for pin at index [{}] but we only have {}".format(motor, self.pins))
