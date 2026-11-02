from pathlib import Path

import numpy as np

from quadrotor import Quadrotor
from controllers.altitude import AltitudeController
from controllers.position import PositionController
from controllers.rate_pid import RatePIDController
from controllers.attitude import AttitudeController

# Simulation type (auto/manual)
simType = "manual"

# Create drone object
drone = Quadrotor()

# Physical limits
hoverThrust = drone.mass * drone.g                          # [N]
maxThrust = 4 * drone.kThrust * drone.omegaMax**2           # all motors at omegaMax [N]
thrustMin = 0.1 * maxThrust                                 # keep motors spinning [N]
thrustMax = 0.8 * maxThrust                                 # headroom for attitude control [N]
maxTilt = np.radians(20)                                    # max roll / pitch command [rad]
maxRate = [3.0, 3.0, 2.0]                                   # max body rate command [rad/s]
maxMoment = [1.0, 1.0, 0.5]                                 # max moment command [N m]
maxAccel = drone.g * np.tan(maxTilt)                        # horizontal accel reachable at max tilt [m/s^2]


# Create flight control objects
altCtrl = AltitudeController(kp=4.0, ki=0.5, kd=3.0, integralLimit=2.0)                # output: thrust correction [N]
posCtrl = PositionController(kp=[1.0, 1.0], ki=[0.05, 0.05], kd=[1.6, 1.6],
                              integralLimit=[2.0, 2.0],
                              outputLimit=[maxAccel, maxAccel])                        # Array convention: [x, y]
attCtrl = AttitudeController(kp=[6.0, 6.0, 3.0], ki=[0.0, 0.0, 0.0], kd=[0.0, 0.0, 0.0],
                              outputLimit=maxRate)                                     # Array convention: [roll, pitch, yaw]
rateCtrl = RatePIDController(kp=[0.3, 0.3, 0.2], ki=[0.05, 0.05, 0.05], kd=[0.0, 0.0, 0.0],
                              integralLimit=[0.5, 0.5, 0.5],
                              outputLimit=maxMoment)                                   # Array convention: [roll, pitch, yaw]

# Simulation parameters
dt = 0.1                            # Time step [s]
simDuration = 20                    # Simulation duration [s]
simSteps = int(simDuration / dt)    # Number of simulation steps

target = [-5, 20, -10] # Target position vector [x, y, z] (world NED, z down so -10 is 10 m up)
desiredYaw = 0 # Desired yaw angle [rad]

# Manual (level) mode
manualDt = 0.01                     # Physics time step in manual mode, run in real time [s]
maxLag = 0.25                       # If the simulation falls further behind real time than this, skip ahead [s]

# Radio (Radiomaster Pocket over USB). Run setup/radioCheck.py first to create the calibration file
calibrationFile = Path(__file__).parent / "setup" / "radio_check.csv"
joystickIndex = 0                   # Which joystick to use if several are connected
radioAxes = {"roll": 0, "pitch": 1, "throttle": 2, "yaw": 3}   # Joystick axis number of each stick (AETR order)
radioSigns = {"roll": -1, "pitch": 1, "throttle": 1, "yaw": -1}  # Set to -1 if a stick moves the wrong way:
                                    # roll right, pitch forward, yaw right and throttle up must all give +
stickDeadband = 0.05                # Stick movements smaller than this (around the centre) are ignored
armThrottleLimit = 0.05             # Throttle must be below this before the simulation starts

# Live dashboard
dashboardWindow = 5.0               # Time shown by the scrolling graphs [s]
dashboardFrameRate = 20             # Redraws per second [Hz]

# Arena for manual mode
arenaSize = [10.0, 10.0, 10.0]      # [length x (north), width y (east), height] [m]
