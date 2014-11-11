#!/usr/bin/env python3

from BasicPID import BasicPID2
from RPIO import PWM

PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)

servo = PWM.Servo(pulse_incr_us=1)

PWM_MAX = 2000
PWM_MIN = 1000
PWM_RANGE = PWM_MAX - PWM_MIN


pid = BasicPID2(setpoint=5, kP=2, kI=0.3, kD=0.5)

while True:
    # Get Error
    current = float(input('Error: '))
    # Calculate pid
    output = pid.update(current)
    # Output New Motor value
    print ("Output: {}".format(output))
