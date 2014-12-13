from RPIO import PWM

GPIO_PINS = [25, 24, 8, 7]

PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)

servo = PWM.Servo(pulse_incr_us=1)

PWM_MAX = 2000
PWM_MIN = 1000
PWM_RANGE = PWM_MAX - PWM_MIN

# To Initialise we have to set PWM to Max on all PWM Ports.
servo.set_servo(GPIO_PINS[0], PWM_MAX)
servo.set_servo(GPIO_PINS[1], PWM_MAX)
servo.set_servo(GPIO_PINS[2], PWM_MAX)
servo.set_servo(GPIO_PINS[3], PWM_MAX)

raw_input("Press return to begin PWM Calibration. Press Return once the motors are connected to power [RETURN] ")
servo.set_servo(GPIO_PINS[0], PWM_MIN)
servo.set_servo(GPIO_PINS[1], PWM_MIN)
servo.set_servo(GPIO_PINS[2], PWM_MIN)
servo.set_servo(GPIO_PINS[3], PWM_MIN)
print("PWM Calibration Complete. Ready to fly. ")

terminated = False
while True and terminated == False:
	motors = raw_input("Enter Comma Separated Motor Indexes To Control [0,1,2,3]: ")
	motors = motors.split(",")

	while True:
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
			for motor in motors:
				pin = GPIO_PINS[int(motor)]	
				print("Giving Speed " + str(pwmspeed) + " On Pin " + str(pin))
				servo.set_servo(pin, pwmspeed)


servo.set_servo(GPIO_PINS[0], PWM_MIN)
servo.set_servo(GPIO_PINS[1], PWM_MIN)
servo.set_servo(GPIO_PINS[2], PWM_MIN)
servo.set_servo(GPIO_PINS[3], PWM_MIN)

servo.stop_servo(GPIO_PINS[0])
servo.stop_servo(GPIO_PINS[1])
servo.stop_servo(GPIO_PINS[2])
servo.stop_servo(GPIO_PINS[3])

PWM.cleanup()
