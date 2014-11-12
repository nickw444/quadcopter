"""
Determine the raw speed percentage required to actually
balance a propellor's weight against gravity (helping PID scaling)
"""

from MotorControl import MotorControl

controller = MotorControl(pins=[25], debug=True)
controller.begin_calibration()
raw_input("Press 'Return' once you have connected power to the motors")
controller.continue_calibration()


print ("Ready to calibrate motor weight")
terminated = False
while terminated == False:
    data = raw_input("Enter new speed [0-100%%] or [q] to quit: ")
    if data == 'q':
        terminated = True
        break
    elif int(data) >= 0 and int(data) <= 100:
        print ("Setting speed to {}".format(data))
        speed = int(data)
        controller.set_motor(0, speed)

controller.cleanup()