#!/usr/bin/env python3

from BasicPID import BasicPID2, VideoPID
from RPIO import PWM
from MiniMu9 import Accelerometer
from MotorControl import MotorControl
import sys
import time


PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)

servo = PWM.Servo(pulse_incr_us=1)

PWM_MAX = 2000
PWM_MIN = 1000
PWM_RANGE = PWM_MAX - PWM_MIN


pid = VideoPID(setpoint=0, kP=2.5, kI=0.2, kD=0.5, zeros=False)
accel = Accelerometer()
controller = MotorControl(pins=[25])


controller.begin_calibration()
print("Press [RETURN] once you have connected power to the motors.")
raw_input()
controller.continue_calibration()
print("Motors Calibrated. Beginning PID Loop")

motor_weight_offset = 27 # Percentage power at which the motor balances.
print("WAIT")
raw_input()

while True:

    # Get Reading
    current = accel.readX()
    # Calculate pid
    output = pid.update(current)
    # Output New Motor value
    scaled_output = motor_weight_offset + output
    scaled_output = round(scaled_output, 0)

    # Put some output caps.
    if scaled_output < 15: scaled_output = 15
    if scaled_output > 50: scaled_output = 50


    controller.set_motor(0, scaled_output)

    # sys.stdout.write('\r')
    # sys.stdout.write("Current Value: {}. Last PID Output: {}. Motor Output: {}".format(round(current, 2), round(output,2), scaled_output))
    # sys.stdout.flush()
    time.sleep(0.05)


print("KEK:")
print("DICKS")
