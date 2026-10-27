from dataclasses import dataclass, field
import numpy as np

@dataclass
class PIDController:
    """Single-axis PID controller.

    The derivative term acts on the measurement rather than the error, so a jump in the
    target (or the very first call) does not produce a derivative kick.
    """

    # Settings
    kp: float
    ki: float
    kd: float
    integralLimit: float = None
    outputLimit: float = None

    # State
    integral: float = field(default=0.0, init=False)
    previousMeasured: float | None = field(default=None, init=False)

    def update(self, targetVal, measuredVal, dt):

        if dt <= 0:
            raise ValueError("dt should be positive.")

        error = targetVal - measuredVal
        self.integral += error * dt
        if self.integralLimit is not None:
            self.integral = np.clip(self.integral, -self.integralLimit, self.integralLimit)

        # Derivative on measurement: d(error)/dt = -d(measured)/dt when the target is constant
        if self.previousMeasured is None:
            derivative = 0.0
        else:
            derivative = -(measuredVal - self.previousMeasured) / dt
        self.previousMeasured = measuredVal

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.outputLimit is not None:
            output = np.clip(output, -self.outputLimit, self.outputLimit)
        return output

    def reset(self):
        self.integral = 0.0
        self.previousMeasured = None
