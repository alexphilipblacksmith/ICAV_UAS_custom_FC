import numpy as np
from scipy.spatial.transform import Rotation

class AttitudeController:
    """Quaternion attitude PID. Output is the desired body angular rates [p, q, r] in rad/s."""

    # Array convention for gains and output: [roll, pitch, yaw] (body x, y, z)

    def __init__(self, kp, ki, kd, integralLimit=None, outputLimit=None):
        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.integralLimit = None if integralLimit is None else np.asarray(integralLimit, dtype=float)
        self.outputLimit = None if outputLimit is None else np.asarray(outputLimit, dtype=float)

        self.integral = np.zeros(3)
        self.previousError = None

    def update(self, targetQuat, measuredQuat, dt):

        if dt <= 0:
            raise ValueError("dt should be positive!")

        # Quaternions are [w, x, y, z] in this project, scipy expects [x, y, z, w]
        rotationCurrent = Rotation.from_quat(np.roll(measuredQuat, -1))
        rotationTarget = Rotation.from_quat(np.roll(targetQuat, -1))

        # Error rotation expressed in the body frame, as a rotation vector (axis * angle)
        rotationErr = rotationCurrent.inv() * rotationTarget
        errorVec = rotationErr.as_rotvec()

        self.integral += errorVec * dt
        if self.integralLimit is not None:
            self.integral = np.clip(self.integral, -self.integralLimit, self.integralLimit)

        # Skip the derivative on the first call to avoid a derivative kick
        if self.previousError is None:
            derivative = np.zeros(3)
        else:
            derivative = (errorVec - self.previousError) / dt
        self.previousError = errorVec.copy()

        output = self.kp * errorVec + self.ki * self.integral + self.kd * derivative
        if self.outputLimit is not None:
            output = np.clip(output, -self.outputLimit, self.outputLimit)
        return output

    def reset(self):
        self.integral = np.zeros(3)
        self.previousError = None
