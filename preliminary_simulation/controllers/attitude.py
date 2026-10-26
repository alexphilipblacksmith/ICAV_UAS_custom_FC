from controller import PIDController
import numpy as np
from scipy.spatial.transform import Rotation

class attitudeController:

    # Attitude controller takes and compares quaternions
    
    def __init__(self, Kp, Ki, Kd):

        #Each coefficient is a 3D array, affecting roll, pitch, yaw, in that order
        self.Kp = np.asarray(Kp)
        self.Ki = np.asarray(Ki)
        self.Kd = np.asarray(Kd)

        self.integral = np.zeros(3)
        self.previousError = np.zeros(3)

    def update(self, targetQuat, measuredQuat, dt, integralLimit = None):

        if dt <= 0:
            raise ValueError("dt should be positive!")
        
        #Find rotation error using scipy, then convert rotation object back to quaternion
        rotationCurrent = Rotation.from_quat(np.roll(measuredQuat, -1))
        rotationTarget = Rotation.from_quat(np.roll(targetQuat, -1))

        rotationErr = rotationCurrent.inv() * rotationTarget

        errorVec = rotationErr.as_rotvec()

        self.integral += errorVec *dt
        if integralLimit is not None:
            self.integral = np.clip(self.integral, -integralLimit, integralLimit)

        derivative = (errorVec-self.previousError)/dt

        self.previousError = errorVec.copy()
        
        return self.Kp * errorVec + self.Ki * self.integral + self.Kd * derivative
  

    def reset(self):
        self.integral = np.zeros(3)
        self.previousError = np.zeros(3)
        
