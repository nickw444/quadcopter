#!/usr/bin/python

import time
from Adafruit_I2C import Adafruit_I2C
from RPIO import PWM
from bitstring import BitArray
import sys
import math
import threading


# 0x6b : Gyro
# 0x19 : Accelerometer
# 0x1e : Magetometer.

class LSM303DLH_Compass:
    __LSM303DLH_CRA_REG_M = 0x00
    __LSM303DLH_CRB_REG_M= 0x01
    __LSM303DLH_MR_REG_M = 0x02
    __LSM303DLH_OUT_X_H_M = 0x03
    __LSM303DLH_OUT_X_L_M = 0x04
    __LSM303DLH_OUT_Z_H_M = 0x05
    __LSM303DLH_OUT_Z_L_M = 0x06
    __LSM303DLH_OUT_Y_H_M = 0x07
    __LSM303DLH_OUT_Y_L_M = 0x08

    __LSM303DLH_CRA_REG_M_DEFAULTS = int("00010100",2)
    __LSM303DLH_CRB_REG_M_DEFAULTS = int("01000000",2)

    __LSM303DLH_MR_REG_M_DEFAULTS =  int("00000000",2) # Mode Selection


    i2c = None

        
    def powerOn(self):
        print ("Doing PowerOn")
        self.i2c.write8(self.__LSM303DLH_CRA_REG_M, self.__LSM303DLH_CRA_REG_M_DEFAULTS)
        self.i2c.write8(self.__LSM303DLH_MR_REG_M, self.__LSM303DLH_MR_REG_M_DEFAULTS)
        self.i2c.write8(self.__LSM303DLH_CRB_REG_M, self.__LSM303DLH_CRB_REG_M_DEFAULTS)


    def __init__(self, address=0x1e, debug=False):
        self.i2c = Adafruit_I2C(address=address, debug=debug)
        self.address = address
        self.debug = debug
        self.powerOn()

    def getReading(self):
        data = []
        data.append(self.i2c.readU8(self.__LSM303DLH_OUT_X_H_M))
        data.append(self.i2c.readU8(self.__LSM303DLH_OUT_X_L_M))
        data.append(self.i2c.readU8(self.__LSM303DLH_OUT_Z_H_M))
        data.append(self.i2c.readU8(self.__LSM303DLH_OUT_Z_L_M))
        data.append(self.i2c.readU8(self.__LSM303DLH_OUT_Y_H_M))
        data.append(self.i2c.readU8(self.__LSM303DLH_OUT_Y_L_M))
        return data

    def readX(self):
        data = self.getReading()
        # print data[0] 
        # print data[1]
        # return 0
        return self.u8_to_s16(data[0], data[1])


    def readY(self):
        data = self.getReading()
        return self.u8_to_s16(data[4], data[5])

    def readZ(self):
        data = self.getReading()
        return self.u8_to_s16(data[2], data[3])

    def twos_to_int(self, b1, bits=8):

        if (b1 >= (1 << (bits-1))):
            b1 = -((1 << bits) - b1)

        return b1
        # pass

    def u8_to_s16(self,b1, b2):
        b1 = b1 << 8
        total = b1 + b2
        return self.twos_to_int(total, 16)



class LSM303DLH_Acc: 

    """3-axis accelerometer and 3-axis magnetometer"""

    # Output Registers
    __LSM303DLH_CTRL_REG1        = 0x20  # PowerDown 000 (off default) , 001 (on) | Zen (Z Enable) 1 default | Yen | Xen
    __LSM303DLH_CTRL_REG2        = 0x21  # HPF Mode
    __LSM303DLH_CTRL_REG3        = 0x22
    __LSM303DLH_CTRL_REG4        = 0x23
    __LSM303DLH_CTRL_REG5        = 0x24

    __LSM303DLH_OUT_X_L          = 0x28  # X-axis angular rate data. The value is expressed as twos complement.
    __LSM303DLH_OUT_X_H          = 0x29  
    __LSM303DLH_OUT_Y_L          = 0x2A  # Y-axis angular rate data. The value is expressed as twos complement.
    __LSM303DLH_OUT_Y_H          = 0x2B
    __LSM303DLH_OUT_Z_L          = 0x2C  # Z-axis angular rate data. The value is expressed as twos complement.
    __LSM303DLH_OUT_Z_H          = 0x2D

    POWER_STATE_OFF     = 0
    POWER_STATE_ON      = 1

    # __L3G4200D_CTRL_DEFAULT_POWER_ON  = 0x5F
    __L3G4200D_CTRL_DEFAULT_POWER_ON  = int("01010111",2)

    # __L3G4200D_CTRL_REG4_A_DEFAULT = int("00001000", 2)  # HIGH RES, 2G Scale
    # __L3G4200D_CTRL_REG4_A_DEFAULT = int("00000000", 2)  # LOW RES, 2G Scale
    __L3G4200D_CTRL_REG4_A_DEFAULT = int("00011000", 2)  # HIGH RES, 4G Scale
    # __L3G4200D_CTRL_REG4_A_DEFAULT = int("00101000", 2)  # HIGH RES, 8G Scale
    __L3G4200D_SCALE_SELECTION = 4 # in g forces.



    __L3G4200D_CTRL_DEFAULT_POWER_DOWN  = 0x0F

    i2c = None
    
    def __init__(self, address=0x19, debug=False):
        self.i2c = Adafruit_I2C(address=address, debug=debug)
        self.address = address
        self.debug = debug
        self.powerstate = self.POWER_STATE_OFF

        # Enable HR logging.
        self.i2c.write8(self.__LSM303DLH_CTRL_REG4, self.__L3G4200D_CTRL_REG4_A_DEFAULT)


        self.powerOn()

    def powerDown(self):
        if (self.powerstate != self.POWER_STATE_OFF):
            # Power on the device. 
            self.i2c.write8(self.__LSM303DLH_CTRL_REG1, self.__L3G4200D_CTRL_DEFAULT_POWER_DOWN)
            self.powerstate = self.POWER_STATE_OFF
        
    def powerOn(self):
        if (self.powerstate != self.POWER_STATE_ON):
            # Power on the device. 
            self.i2c.write8(self.__LSM303DLH_CTRL_REG1, self.__L3G4200D_CTRL_DEFAULT_POWER_ON)
            self.powerstate = self.POWER_STATE_ON

    def twos_to_int(self, b1, bits=8):

        if (b1 >= (1 << (bits-1))):
            b1 = -((1 << bits) - b1)

        return b1
        # pass

    def u8_to_s16(self,b1, b2):
        b1 = b1 << 8
        total = b1 + b2
        return self.twos_to_int(total, 16)



    def s16_to_g(self, raw):
        # g = 65

        scale = self.__L3G4200D_SCALE_SELECTION
        bits = 16

        upperLimit = (1<<(bits - 1)) 
        lowerLimit = (1<<(bits-1)) - 1

        if raw < 0:
            gs = raw/float(lowerLimit) * scale

        else:
            gs = raw/float(upperLimit) * scale

        return gs
        # pass


    def readX(self):
        b1 = self.i2c.readU8(self.__LSM303DLH_OUT_X_L)
        b2 = self.i2c.readU8(self.__LSM303DLH_OUT_X_H)

        r = self.u8_to_s16(b2, b1)
        return self.s16_to_g(r)

    def readY(self):
        b1 = self.i2c.readU8(self.__LSM303DLH_OUT_Y_L)
        b2 = self.i2c.readU8(self.__LSM303DLH_OUT_Y_H)

        r = self.u8_to_s16(b2, b1)
        return self.s16_to_g(r)

    def readZ(self):
        b1 = self.i2c.readU8(self.__LSM303DLH_OUT_Z_L)
        b2 = self.i2c.readU8(self.__LSM303DLH_OUT_Z_H)

        r = self.u8_to_s16(b2, b1)
        return self.s16_to_g(r)


class L3G4200D:

    """ultra-stable three-axis digital output gyroscope"""
    
    i2c = None

    # Output Registers
    __L3G4200D_WHO_AM_I         = 0x0F  # Device ID
    __L3G4200D_CTRL_REG1        = 0x20  # Data Rate | Data Rate | Bandwidth | Bandwidth | PowerDown 0 (off default) , 1 (on) | Zen (Z Enable) 1 default | Yen | Xen
    __L3G4200D_CTRL_REG2        = 0x21  # HPF Mode
    __L3G4200D_CTRL_REG3        = 0x22
    __L3G4200D_CTRL_REG4        = 0x23
    __L3G4200D_CTRL_REG5        = 0x24
    __L3G4200D_REFERENCE        = 0x25  # Reference value for Interrupt generation. Default value: 0
    __L3G4200D_OUT_TEMP         = 0x26  # Temperature data.
    __L3G4200D_STATUS_REG       = 0x27  # ZYXOR | ZOR  | YOR | XOR | ZYXDA | ZDA | YDA | XDA - 0 = old data, 1 = new data
    __L3G4200D_OUT_X_L          = 0x28  # X-axis angular rate data. The value is expressed as twos complement.
    __L3G4200D_OUT_X_H          = 0x29  
    __L3G4200D_OUT_Y_L          = 0x2A  # Y-axis angular rate data. The value is expressed as twos complement.
    __L3G4200D_OUT_Y_H          = 0x2B
    __L3G4200D_OUT_Z_L          = 0x2C  # Z-axis angular rate data. The value is expressed as twos complement.
    __L3G4200D_OUT_Z_H          = 0x2D

    __L3G4200D_FIFO_CTRL_REG    = 0x2E
    __L3G4200D_SRC_REG          = 0x2F
    __L3G4200D_INT1_CFG         = 0x30
    __L3G4200D_INT1_SRC         = 0x31
    __L3G4200D_INT1_TSH_XH      = 0x32
    __L3G4200D_INT1_TSH_XL      = 0x33
    __L3G4200D_INT1_TSH_YH      = 0x34
    __L3G4200D_INT1_TSH_YL      = 0x35
    __L3G4200D_INT1_TSH_ZH      = 0x36
    __L3G4200D_INT1_TSH_ZL      = 0x37
    __L3G4200D_INT1_DURATION    = 0x38

    __L3G4200D_CTRL_DEFAULT_POWER_ON  = 0x0F
    __L3G4200D_CTRL_DEFAULT_POWER_DOWN  = 0x07
    __L3G4200D_CTRL_DEFAULT_POWER_STNDBY  = 0x08




    # Constructor
    def __init__(self, address=0x6b, debug=False):
        self.i2c = Adafruit_I2C(address=address, debug=debug)
        self.address = address
        self.debug = debug
        self.i2c.write8(self.__L3G4200D_CTRL_REG1, self.__L3G4200D_CTRL_DEFAULT_POWER_ON)


    def readX(self):
        b1 = self.i2c.readU8(self.__L3G4200D_OUT_X_L)
        b2 = self.i2c.readU8(self.__L3G4200D_OUT_X_H)
        d = self.u8_to_s16(b2, b1)
        return d

    def readY(self):
        b1 = self.i2c.readU8(self.__L3G4200D_OUT_Y_L)
        b2 = self.i2c.readU8(self.__L3G4200D_OUT_Y_H)
        d = self.u8_to_s16(b2, b1)
        return d


    def readZ(self):
        b1 = self.i2c.readU8(self.__L3G4200D_OUT_Z_L)
        b2 = self.i2c.readU8(self.__L3G4200D_OUT_Z_H)
        d = self.u8_to_s16(b2, b1)
        return d
        pass

    def readTemperature(self):
        pass

    def twos_to_int(self, b1, bits=8):

        if (b1 >= (1 << (bits-1))):
            b1 = -((1 << bits) - b1)

        return b1
        # pass

    def u8_to_s16(self,b1, b2):
        b1 = b1 << 8
        total = b1 + b2
        return self.twos_to_int(total, 16)



# d = L3G4200D(0x6b,True)
# e = LSM303DLH_Acc(debug=False)
# d = LSM303DLH_Compass(debug=False)
# g = L3G4200D(debug=False)

# while True:
#     try:
#         x = d.readX()
#         y = d.readY()
#         z = d.readZ()
#         sys.stdout.write('\r')

#         # sys.stdout.write('test')
#         sys.stdout.write("X: {0}, Y: {1}, Z: {2}   ".format(round(x,2),round(y,2), round(z,2)))
#         sys.stdout.flush()
#     except KeyboardInterrupt:
#         break
#     time.sleep(0.05)

def dot_product(vector_a, vector_b):
    if (len(vector_a) is not len(vector_b)):
        raise Exception("Differing vector dimensions")
        return

    dot = 0
    i = 0
    for dimension in vector_a:
        dot += dimension * vector_b[i]
        i+=1

    return dot

def testDot_product():
    a = [0,0,0]
    b = [0,0,0]
    assert dot_product(a,b) == 0

    a = [0,1,2]
    b = [0,2,1]
    assert dot_product(a,b) == 4


def runTests():
    print "Testing Program"
    testDot_product()
    print "Dot Product Tests Passed"
    print "All Tests Passed."


def magnitude(vector):
    total = 0
    for dimension in vector:
        total += (dimension * dimension)

    return math.sqrt(total)

def accPID(sensor,forceVector):

    pass

PWM_MAX = 2000
PWM_MIN = 1000
PWM_RANGE = PWM_MAX - PWM_MIN

class PID(threading.Thread):

    def __init__(self, motors, servo):
        threading.Thread.__init__(self)
        self.isRunning = True
        self.acc = LSM303DLH_Acc(debug=False)
        self.speed = 0
        self.multi = 0.1
        self.motors = motors
        self.servo = servo

    def stop(self):
        self.isRunning = False

    def updateSpeed(self, speed):
        self.speed = speed

    def run(self):
        while self.isRunning:
            x = self.acc.readX() * 100
            y = self.acc.readY() * 100
            z = self.acc.readZ() * 100
            sys.stdout.write('\r')
            vec_a = [x,y,z]
            vec_b = [x,0,z]

            a_dot_b = dot_product(vec_a, vec_b)
            angle = math.acos(a_dot_b/(magnitude(vec_a) * magnitude(vec_b)));
            angle = 90 - (angle * 180/math.pi)

            direction = 0
            r_0 = round(vec_a[0],2)
            r_2 = round(vec_a[2],2)
            if r_2 != 0:
                # Stop division by zero.
                # Determine what quadrant we are in.
                direction = math.atan(r_0/r_2)
                direction = direction * 180/math.pi + 90

                if r_0 < 0 and r_2 > 0:
                    direction += 180
                elif r_0 > 0 and r_2 >= 0:
                    direction += 180 

            sproj = a_dot_b/magnitude(vec_b)


            # Use our maths to now determine where to put the power.
            # Now that we have all the direction calculations, we just need to give the power to the motors.

            motor = [self.speed,self.speed,self.speed,self.speed]

            if direction >= 0 and direction <= 180:
                # Power for motor 0
                directionMultiplier = abs(90 - direction) / float(90)

                motor[0] = self.speed + directionMultiplier * self.multi + sproj


            if direction >= 90 and direction <= 270:
                # Power for motor 3
                d = direction - 90
                directionMultiplier = abs(90 - d)

                motor[3] = self.speed + directionMultiplier * self.multi + sproj
                

            if direction >= 180 and direction <= 360:
                # Power for Motor 1
                d = direction - 180
                directionMultiplier = abs(90 - d)
                
                motor[1] = self.speed + directionMultiplier * self.multi + sproj

            if direction <= 90 or direction >= 270:
                # Power for Motor 2 
                if (direction <= 90):
                    d = direction + 90
                elif (direction >= 270):
                    d = direction - 270

                motor[2] = self.speed + directionMultiplier * self.multi + sproj
                

            # sys.stdout.write("RUNNING PID: X: {0}, Y: {1}, Z: {2} Tilt: {3} deg direction: {4} sproj: {5}".format(round(x,2),round(y,2), round(z,2), round(angle,2), round(direction,2), round(sproj,2)))
            sys.stdout.write("MOTOR: {0} {1} {2} {3}".format(round(motor[0],2),round(motor[1],2), round(motor[2],2), round(motor[3],2)))
            
            sys.stdout.flush()

            servo.set_servo(self.motors[0], round((motor[0]/100)*PWM_RANGE + PWM_MIN))
            servo.set_servo(self.motors[1], round((motor[1]/100)*PWM_RANGE + PWM_MIN))
            servo.set_servo(self.motors[2], round((motor[2]/100)*PWM_RANGE + PWM_MIN))
            servo.set_servo(self.motors[3], round((motor[3]/100)*PWM_RANGE + PWM_MIN))


            # time.sleep(0.2)

runTests()

MOTOR = [25, 24, 8, 7]
PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)
servo = PWM.Servo(pulse_incr_us=1)

# To Initialise we have to set PWM to Max on all PWM Ports.
servo.set_servo(MOTOR[0], PWM_MAX)
servo.set_servo(MOTOR[1], PWM_MAX)
servo.set_servo(MOTOR[2], PWM_MAX)
servo.set_servo(MOTOR[3], PWM_MAX)

raw_input("Press return to begin PWM Calibration. Press Return once the motors are connected to power [RETURN] ")
servo.set_servo(MOTOR[0], PWM_MIN)
servo.set_servo(MOTOR[1], PWM_MIN)
servo.set_servo(MOTOR[2], PWM_MIN)
servo.set_servo(MOTOR[3], PWM_MIN)
print("PWM Calibration Complete. Ready to fly. ")

pid = PID(MOTOR, servo)
speed = input("Enter Starting Speed:")
pid.updateSpeed(speed)
pid.start()

while True:
    try:
        speed = raw_input("Enter New Speed: (or ! to stop)")
        if (speed == "!"):
            break
        pid.updateSpeed(int(speed))
        speeds = pid.getSpeeds()


    except KeyboardInterrupt:
        break

pid.stop()
print "POwering Down"

servo.set_servo(MOTOR[0], PWM_MIN)
servo.set_servo(MOTOR[1], PWM_MIN)
servo.set_servo(MOTOR[2], PWM_MIN)
servo.set_servo(MOTOR[3], PWM_MIN)

servo.stop_servo(MOTOR[0])
servo.stop_servo(MOTOR[1])
servo.stop_servo(MOTOR[2])
servo.stop_servo(MOTOR[3])
PWM.cleanup()
