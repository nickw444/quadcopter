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
d = LSM303DLH_Acc(debug=False)
# d = LSM303DLH_Compass(debug=False)
# g = L3G4200D(debug=False)

while True:
    try:
        x = d.readX()
        y = d.readY()
        z = d.readZ()
        sys.stdout.write('\r')

        # sys.stdout.write('test')
        sys.stdout.write("X: {0}, Y: {1}, Z: {2}   ".format(round(x,2),round(y,2), round(z,2)))
        sys.stdout.flush()
    except KeyboardInterrupt:
        break
    time.sleep(0.05)

