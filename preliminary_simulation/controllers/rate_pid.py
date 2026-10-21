from preliminary_simulation.controller import PIDController

class ratePIDController:
    def __init__(self, Kp, Ki, Kd):
        self.yawRatePID = PIDController(Kp[0], Ki[0], Kd[0])
        self.pitchRatePID = PIDController(Kp[1], Ki[1], Kd[1])
        self.rollRatePID = PIDController(Kp[2], Ki[2], Kd[2])

    def update(self, targetRate, measuredRate, dt):
        yawCommand = self.yawRatePID.update(targetRate[0], measuredRate[0], dt)
        pitchCommand = self.pitchRatePID.update(targetRate[1], measuredRate[1], dt)
        rollCommand = self.rollRatePID.update(targetRate[2], measuredRate[2], dt)

        return [yawCommand, pitchCommand, rollCommand]

    def reset(self):
        self.yawRatePID.reset()
        self.pitchRatePID.reset()
        self.rollRatePID.reset()
