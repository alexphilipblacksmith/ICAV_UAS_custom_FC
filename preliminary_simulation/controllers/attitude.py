import numpy as np
from preliminary_simulation.controller import PIDController

class attitudeController:

    #Array Convention: [yaw, pitch, roll]
    
    def __init__(self, Kp, Ki, Kd):

        self.yawPID = PIDController(Kp[0], Ki[0], Kd[0])
        self.pitchPID = PIDController(Kp[1], Ki[1], Kd[1])
        self.rollPID = PIDController(Kp[2], Ki[2], Kd[2])

    def update(self, target, measured, dt):

        yawCommand = self.yawPID.update(target[0], measured[0], dt)
        pitchCommand = self.pitchPID.update(target[1], measured[1], dt)
        rollCommand = self.rollPID.update(target[2], measured[2], dt)
       
        return [yawCommand, pitchCommand, rollCommand] # return output commands for yaw pitch and roll

    def reset(self):
        self.rollPID.reset()
        self.pitchPID.reset()
        self.yawPID.reset()
