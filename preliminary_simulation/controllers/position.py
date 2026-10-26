from controller import PIDController

class positionController:
    def __init__(self, Kp, Ki, Kd):
        self.xPID = PIDController(Kp[0], Ki[0], Kd[0])
        self.yPID = PIDController(Kp[1], Ki[1], Kd[1])

    def update(self, targetXY, measuredXY, dt):
        xCommand = self.xPID.update(targetXY[0], measuredXY[0], dt)
        yCommand = self.yPID.update(targetXY[1], measuredXY[1], dt)

        return([xCommand, yCommand])

    def reset(self):
        self.xPID.reset()
        self.yPID.reset()
        