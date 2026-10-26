import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def update(self, targetVal, measuredVal, dt, integralLimit=None):

        if dt <= 0:
            raise ValueError("dt should be positive.")
        
        error = targetVal - measuredVal
        self.integral += error * dt
        if integralLimit is not None:
            self.integral = np.clip(self.integral, -integralLimit, integralLimit)  # integral limits to be tuned
        
        derivative = (error - self.previous_error) /dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

    def reset(self):
        self.integral = 0
        self.previous_error = 0
    
