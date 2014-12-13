from RPIO import PWM

GPIO_PINS = [25, 8, 7, 24]
servo = PWM.Servo()


while True:
	speed = raw_input("Gimmie Value For Port 25: ")
	sp = int(speed)
	servo.set_servo(GPIO_PINS[0], sp)


servo.set_servo(GPIO_PINS[0], 1000)
servo.stop_servo(GPIO_PINS[0])

PWM.cleanup()
