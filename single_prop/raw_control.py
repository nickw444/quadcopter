from RPIO import PWM

GPIO_PINS = [25]

PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)

servo = PWM.Servo(pulse_incr_us=1)

PWM_MAX = 2000
PWM_MIN = 1000
PWM_RANGE = PWM_MAX - PWM_MIN

# To Initialise we have to set PWM to Max on all PWM Ports.
servo.set_servo(GPIO_PINS[0], PWM_MAX)
raw_input("Press return to begin PWM Calibration. Press Return once the motors are connected to power [RETURN] ")
servo.set_servo(GPIO_PINS[0], PWM_MIN)
print("PWM Calibration Complete. Ready to fly. ")

terminated = False
while True and terminated == False:
	speed = raw_input("Enter New Speed [0-100%] OR [!] to Choose Motors OR [q] to quit: ")
	if speed == "!":
		break
	elif speed == "q":
		terminated = True
		break
	elif int(speed) >= 0 and int(speed) <= 100:
		print("Raw Speed is: " + speed)
		sp = int(speed)
		perc = sp/float(100)
		pwmspeed = (perc * PWM_RANGE) + PWM_MIN
		pin = GPIO_PINS[0]	
		print("Giving Speed " + str(pwmspeed) + " On Pin " + str(pin))
		servo.set_servo(pin, pwmspeed)


servo.set_servo(GPIO_PINS[0], PWM_MIN)
servo.stop_servo(GPIO_PINS[0])

PWM.cleanup()
