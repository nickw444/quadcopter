import time

class BasicPID2:
    def __init__(self, setpoint, kP=2.0, kI=0.3, kD=0.0, zeros=True):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.setpoint = setpoint
        self.prevtm = time.time()
        self.prev_err = 0
        self.e_total = 0
        self.zeros = zeros

    def update(self, current):
        # Calculate the error with respect to the setpoint.
        error = self.setpoint - current

        # Update current time
        currtm = time.time()

        # Proportional
        p = error * self.kP

        # Integral 
        # Calculate for this chunk of time since last
        # using the trapezoidal method
        if abs(error) < 0.1 and self.zeros:
            self.e_total = 0
        
        self.e_total += 0.5 * (self.prev_err + error) * (currtm - self.prevtm)
        i = self.e_total * self.kI
        # Possibly take into account zeroing this if we reach the setpoint.

        # Derivative
        chng_y = currtm - self.prevtm
        chng_x = error - self.prev_err
        m = 0
        if chng_x:
            m = chng_y/chng_x
        d = self.kD * m

        self.prev_err = error
        self.prevtm = currtm
        # print ("P: {} I: {} D: {}".format(p, i, d))
        correction = p + i + d
        return correction


class VideoPID(BasicPID2):
    def update(self, current):
        # Calculate the error with respect to the setpoint.
        error = self.setpoint - current

        # Update current time
        currtm = time.time()

        # Proportional
        p = error * self.kP

        # Integral 
        # Calculate for this chunk of time since last
        # using the trapezoidal method
        if abs(error) < 0.1 and self.zeros:
            self.e_total = 0
        
        self.e_total += error
        i = self.e_total * self.kI
        # Possibly take into account zeroing this if we reach the setpoint.

        # Derivative
        d = self.kD * (error - self.prev_err)

        self.prev_err = error
        correction = p + i + d
        return correction


