from RPIO import PWM
import time

max_PWM = 2000
min_PWM = 700

MOTOR_1_PIN = 25

servo = PWM.Servo()
time.sleep(2)

servo.set_servo(MOTOR_1_PIN, max_PWM)
time.sleep(5)
servo.set_servo(MOTOR_1_PIN, min_PWM)

time.sleep(3)
print("Servo Set Up")

while True:
	try:
		print("Enter New PWM Value:")
		ins = int(raw_input())
		servo.set_servo(MOTOR_1_PIN, ins)

	except KeyboardInterrupt:
		break

servo.stop_servo(MOTOR_1_PIN)
PWM.cleanup()

