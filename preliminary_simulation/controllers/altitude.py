import numpy as np
from preliminary_simulation.controller import PIDController

class altitudeController:
    def __init__(self, Kp, Ki, Kd):
        self.pid = PIDController(Kp, Ki, Kd)

    def update(self, targetAltitude, measuredAltitude, dt):
        return self.pid.update(targetAltitude, measuredAltitude, dt)

    def reset(self):
        self.pid.reset()