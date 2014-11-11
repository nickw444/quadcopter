import time

class BasicPID2:
    def __init__(self, setpoint, kP=2.0, kI=0.3, kD=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.setpoint = setpoint
        self.prevtm = time.time()
        self.prev_err = 0
        self.e_total = 0

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
        if abs(error) < 1:
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
        print ("P: {} I: {} D: {}".format(p, i, d))
        correction = p + i + d
        return correction

class BasicPID:
    """ Simple PID control.

        This class implements a simplistic PID control algorithm. When first
        instantiated all the gain variables are set to zero, so calling
        the method GenOut will just return zero.
    """
    def __init__(self):
        # initialze gains
        self.Kp = 0
        self.Kd = 0
        self.Ki = 0

        self.Initialize()

    def SetKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def SetKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def SetKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr

    def Initialize(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0


    def GenOut(self, error):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        de = error - self.prev_err              # get delta error

        self.Cp = self.Kp * error               # proportional term
        self.Ci += error * dt                   # integral term

        self.Cd = 0
        if dt > 0:                              # no div by zero
            self.Cd = de/dt                     # derivative term

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error

        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)