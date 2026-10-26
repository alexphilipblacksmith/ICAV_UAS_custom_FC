from controller import PIDController

class AltitudeController:
    """Altitude PID (altitude positive up). Output is a thrust correction [N] on top of hover thrust."""

    def __init__(self, kp, ki, kd, integralLimit=None, outputLimit=None):
        self.pid = PIDController(kp, ki, kd, integralLimit, outputLimit)

    def update(self, targetAltitude, measuredAltitude, dt):
        return self.pid.update(targetAltitude, measuredAltitude, dt)

    def reset(self):
        self.pid.reset()
