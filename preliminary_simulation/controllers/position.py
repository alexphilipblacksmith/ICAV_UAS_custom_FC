import numpy as np
from controller import PIDController

class PositionController:
    """Horizontal position PID. Output is the desired world-frame acceleration [ax, ay] in m/s^2."""

    # Array convention: [x, y]

    def __init__(self, kp, ki, kd, integralLimit=(None, None), outputLimit=(None, None)):
        self.xPID = PIDController(kp[0], ki[0], kd[0], integralLimit[0], outputLimit[0])
        self.yPID = PIDController(kp[1], ki[1], kd[1], integralLimit[1], outputLimit[1])

    def update(self, targetXY, measuredXY, dt):
        xCommand = self.xPID.update(targetXY[0], measuredXY[0], dt)
        yCommand = self.yPID.update(targetXY[1], measuredXY[1], dt)

        return np.array([xCommand, yCommand])

    def reset(self):
        self.xPID.reset()
        self.yPID.reset()
