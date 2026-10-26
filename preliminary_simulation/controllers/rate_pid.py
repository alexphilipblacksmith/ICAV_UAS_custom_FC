from controller import PIDController

class ratePIDController:

    # Array Convention: [roll, pitch, yaw]

    def __init__(self, Kp, Ki, Kd):
        self.rollRatePID = PIDController(Kp[0], Ki[0], Kd[0])
        self.pitchRatePID = PIDController(Kp[1], Ki[1], Kd[1])
        self.yawRatePID = PIDController(Kp[2], Ki[2], Kd[2])

    def update(self, targetRate, measuredRate, dt):
        rollCommand = self.rollRatePID.update(targetRate[0], measuredRate[0], dt)
        pitchCommand = self.pitchRatePID.update(targetRate[1], measuredRate[1], dt)
        yawCommand = self.yawRatePID.update(targetRate[2], measuredRate[2], dt)

        return [rollCommand, pitchCommand, yawCommand]

    def reset(self):
        self.yawRatePID.reset()
        self.pitchRatePID.reset()
        self.rollRatePID.reset()
