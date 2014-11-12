from Adafruit_I2C import Adafruit_I2C

class Accelerometer: 

    """
    LSM303DLH
    3-axis accelerometer and 3-axis magnetometer
    accelerometer class
    """

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