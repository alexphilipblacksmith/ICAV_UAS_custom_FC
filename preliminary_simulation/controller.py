import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def update(self, targetVal, measuredVal, dt):
        error = targetVal - measuredVal
        self.integral += error * dt
        self.integral = np.clip(self.integral, -100, 100)  # integral limits to be tuned
        if dt > 0:
            derivative = (error - self.previous_error) /dt
        else:
            derivative = 0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

    def reset(self):
        self.integral = 0
        self.previous_error = 0
    
