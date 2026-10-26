import numpy as np

class PIDController:
    """Single-axis PID controller.

    The derivative term acts on the measurement rather than the error, so a jump in the
    target (or the very first call) does not produce a derivative kick.
    """

    def __init__(self, kp, ki, kd, integralLimit=None, outputLimit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integralLimit = integralLimit  # anti-windup clamp on the integral, None = no limit
        self.outputLimit = outputLimit      # symmetric clamp on the output, None = no limit
        self.integral = 0.0
        self.previousMeasured = None

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
