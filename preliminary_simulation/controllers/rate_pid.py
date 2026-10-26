import numpy as np
from controller import PIDController

class RatePIDController:
    """Body angular rate PID. Output is the desired body moments [L, M, N] in N m."""

    # Array convention: [roll, pitch, yaw]

    def __init__(self, kp, ki, kd, integralLimit=(None, None, None), outputLimit=(None, None, None)):
        self.rollRatePID = PIDController(kp[0], ki[0], kd[0], integralLimit[0], outputLimit[0])
        self.pitchRatePID = PIDController(kp[1], ki[1], kd[1], integralLimit[1], outputLimit[1])
        self.yawRatePID = PIDController(kp[2], ki[2], kd[2], integralLimit[2], outputLimit[2])

    def update(self, targetRate, measuredRate, dt):
        rollCommand = self.rollRatePID.update(targetRate[0], measuredRate[0], dt)
        pitchCommand = self.pitchRatePID.update(targetRate[1], measuredRate[1], dt)
        yawCommand = self.yawRatePID.update(targetRate[2], measuredRate[2], dt)

        return np.array([rollCommand, pitchCommand, yawCommand])

    def reset(self):
        self.rollRatePID.reset()
        self.pitchRatePID.reset()
        self.yawRatePID.reset()
