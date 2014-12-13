import argparse

from pythonosc import dispatcher
from pythonosc import osc_server
from pythonosc import osc_message_builder
from pythonosc import udp_client

# RPI Stuff
from RPIO import PWM
GPIO_PINS = [25, 24, 8, 7]
PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)
servo = PWM.Servo(pulse_incr_us=1)
PWM_MAX = 2000
PWM_MIN = 1000
PWM_RANGE = PWM_MAX - PWM_MIN


# Client Management
active_client = []

# Running Variables.
armed = False
running = False
manual_control = False
is_waiting = False
last_x = 0.5
last_y = 0.5
last_power = 0

def pwm_max_calibrate():
  servo.set_servo(GPIO_PINS[0], PWM_MAX)
  servo.set_servo(GPIO_PINS[1], PWM_MAX)
  servo.set_servo(GPIO_PINS[2], PWM_MAX)
  servo.set_servo(GPIO_PINS[3], PWM_MAX)
  pass

def pwm_min_calibrate():
  servo.set_servo(GPIO_PINS[0], PWM_MIN)
  servo.set_servo(GPIO_PINS[1], PWM_MIN)
  servo.set_servo(GPIO_PINS[2], PWM_MIN)
  servo.set_servo(GPIO_PINS[3], PWM_MIN)
  pass

def stop_servos():
  servo.set_servo(GPIO_PINS[0], PWM_MIN)
  servo.set_servo(GPIO_PINS[1], PWM_MIN)
  servo.set_servo(GPIO_PINS[2], PWM_MIN)
  servo.set_servo(GPIO_PINS[3], PWM_MIN)
  pass

def update_servo(motor, value):
  pin = GPIO_PINS[motor]
  pwmspeed = (value * PWM_RANGE) + PWM_MIN
  pwmspeed = int(pwmspeed)
  if pwmspeed > PWM_MAX:
    pwmspeed = PWM_MAX
  
  print("Giving Speed " + str(pwmspeed) + " On Pin " + str(pin))
  servo.set_servo(pin, pwmspeed)



def respond_status(client=None, port=9000):
  global running
  global armed
  if client is not None:
    active_client = udp_client.UDPClient(client, port)

  msg = osc_message_builder.OscMessageBuilder(address='/1/armed')
  msg.add_arg(int(armed))
  msg = msg.build()
  active_client.send(msg)
  msg = osc_message_builder.OscMessageBuilder(address='/1/running')
  msg.add_arg(int(running))
  msg = msg.build()
  active_client.send(msg)
  msg = osc_message_builder.OscMessageBuilder(address='/1/arming')
  msg.add_arg(int(armed))
  msg = msg.build()
  active_client.send(msg)

def respond_message(address='/1/status', data=0, client=None, port=9000):
  if client is not None:
    active_client = udp_client.UDPClient(client, port)
  
  msg = osc_message_builder.OscMessageBuilder(address=address)
  msg.add_arg(data)
  msg = msg.build()
  active_client.send(msg)


def action_button1(remote_addr, unused_addr, args, value):
  global is_waiting
  global running

  if not running and not is_waiting:
    respond_message(data='Connect Motors then Continue', client=remote_addr[0])
    respond_message(data='Continue', client=remote_addr[0], address='/1/action_label')
    is_waiting = True
    pwm_max_calibrate()


  elif not running and is_waiting:
    running = True
    is_waiting = False
    respond_message(data='Calibration Complete.', client=remote_addr[0])
    respond_status(client=remote_addr[0])
    pwm_min_calibrate()

  else:
    respond_message(data=0, client=remote_addr[0], address='/1/action1')



def arming_handler(remote_addr, unused_addr, args, value):
  if not running:
    # Setup PWM Servo Stuff
    respond_message(data='Please Press Start Before Arm', client=remote_addr[0])
    respond_message(data=0, client=remote_addr[0], address='/1/arming')
    return

  global armed    
  armed = bool(value)
  if armed:
    respond_message(data='Armed!', client=remote_addr[0])
  else:
    respond_message(data='Disarmed!', client=remote_addr[0])
    stop_servos()

  respond_status(client=remote_addr[0])

def control_slider(remote_addr, slider, args, value):
  if running and armed:
    update_servo(args[0], value)
  else:
    respond_message(data='Need to arm', client=remote_addr[0])
    respond_message(data=0, address=slider, client=remote_addr[0])

  respond_status(client=remote_addr[0])

def update_power(remote_addr, slider, args, power):

  if running and armed:
    global last_power
    last_power = power
    balance_x_y(remote_addr=None, slider=None, args=None, x=last_x, y=last_y)
  else:
    respond_message(data='Need to arm', client=remote_addr[0])
    respond_message(data=0, address=slider, client=remote_addr[0])

  respond_status(client=remote_addr[0])



def balance_x_y(remote_addr, slider, args, x, y):
  if running and armed:
    global last_x
    global last_y

    power = last_power

    last_x = x
    last_y = y

    x = x*2
    y = y*2

    motor_scale = [x, 2-x, y, 2-y]
    new_scale = []
    for motor in motor_scale:
      m = motor
      if motor < 1:
        m = 1

      new_scale.append(m)

    for x, val in enumerate(new_scale):
      update_servo(x, val * power)
  
  else:
    pass



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--ip", default="0.0.0.0", help="The ip to listen on")
  parser.add_argument("--port", type=int, default=6000, help="The port to listen on")
  args = parser.parse_args()

  dispatcher = dispatcher.Dispatcher()
  dispatcher.map("/1/arming", arming_handler, None)
  dispatcher.map("/1/action1", action_button1, None)
  dispatcher.map("/1/motor1", control_slider, 0)
  dispatcher.map("/1/motor2", control_slider, 1)
  dispatcher.map("/1/motor3", control_slider, 2)
  dispatcher.map("/1/motor4", control_slider, 3)
  dispatcher.map("/1/balancexy", balance_x_y, None)
  dispatcher.map("/2/power", update_power, None)
  # dispatcher.map("/logvolume", print_compute_handler, "Log volume", math.log)

  server = osc_server.ThreadingOSCUDPServer(
      (args.ip, args.port), dispatcher)
  print("Serving on {}".format(server.server_address))
  server.serve_forever()