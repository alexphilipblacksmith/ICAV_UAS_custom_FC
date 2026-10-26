from quadrotor import Quadrotor
from controllers.altitude import AltitudeController
from controllers.position import PositionController
from controllers.rate_pid import RatePIDController
from controllers.attitude import AttitudeController
import numpy as np
from scipy.spatial.transform import Rotation
from visualisation import PathPlotter3D, plotMotorSpeeds, plotAttitude

## Create drone object ##
drone = Quadrotor()

## Physical limits ##
hoverThrust = drone.mass * drone.g                          # [N]
maxThrust = 4 * drone.kThrust * drone.omegaMax**2           # all motors at omegaMax [N]
thrustMin = 0.1 * maxThrust                                 # keep motors spinning [N]
thrustMax = 0.8 * maxThrust                                 # headroom for attitude control [N]
maxTilt = np.radians(20)                                    # max roll / pitch command [rad]
maxRate = [3.0, 3.0, 2.0]                                   # max body rate command [rad/s]
maxMoment = [1.0, 1.0, 0.5]                                 # max moment command [N m]
maxAccel = drone.g * np.tan(maxTilt)                        # horizontal accel reachable at max tilt [m/s^2]

## Create flight control objects ##
altCtrl = AltitudeController(kp=4.0, ki=0.5, kd=3.0, integralLimit=2.0)                # output: thrust correction [N]
posCtrl = PositionController(kp=[1.0, 1.0], ki=[0.05, 0.05], kd=[1.6, 1.6],
                              integralLimit=[2.0, 2.0],
                              outputLimit=[maxAccel, maxAccel])                        # Array convention: [x, y]
attCtrl = AttitudeController(kp=[6.0, 6.0, 3.0], ki=[0.0, 0.0, 0.0], kd=[0.0, 0.0, 0.0],
                              outputLimit=maxRate)                                     # Array convention: [roll, pitch, yaw]
rateCtrl = RatePIDController(kp=[0.3, 0.3, 0.2], ki=[0.05, 0.05, 0.05], kd=[0.0, 0.0, 0.0],
                              integralLimit=[0.5, 0.5, 0.5],
                              outputLimit=maxMoment)                                   # Array convention: [roll, pitch, yaw]

## Simulation parameters ##
dt = 0.1                           # Time step [s]
simDuration = 20                    # Simulation duration [s]
simSteps = int(simDuration / dt)    # Number of simulation steps
time = 0                            # Elapsed time of simulation [s]

target = [-5, 20, -10] # Target position vector [x, y, z] (world NED, z down so -10 is 10 m up)

desiredYaw = 0 # Desired yaw angle [rad]

# State history, one entry per step
timeHistory = []              # [s]
positionHistory = []          # world [x, y, z] [m]
velocityHistory = []          # world [vx, vy, vz] [m/s]
motorSpeedHistory = []        # motors 1..4 [rad/s]
attitudeHistory = []          # [roll, pitch, yaw] [rad]
desiredAttitudeHistory = []   # commanded [roll, pitch, yaw] [rad]

# Create graph plotter object
trajectory3D = PathPlotter3D(target, updateFrequency=5, limit=25)

## Simulation Loop ##
for step in range(simSteps):

    # Get state of the drone
    stateVector = drone.getState()
    currentPos = stateVector[0:3] # Current position vector [x, y, z]
    currentVel = stateVector[3:6] # Current velocity vector [vx, vy, vz] (world)
    currentOrientation = stateVector[6:10] # Current orientation quaternion [w, x, y, z]
    currentAngVel = stateVector[10:13] # Current angular velocity vector [p, q, r]

    currentRotation = Rotation.from_quat(np.roll(currentOrientation, -1))
    currentYaw, currentPitch, currentRoll = currentRotation.as_euler('ZYX') # intrinsic yaw-pitch-roll

    # Store state history
    timeHistory.append(time)
    positionHistory.append(currentPos)
    velocityHistory.append(currentVel)
    attitudeHistory.append([currentRoll, currentPitch, currentYaw])

    ## Cascaded PID loops: ##
    # altCtrl gives a thrust correction from altitude error
    # posCtrl gives desired world accelerations from position error -> converted to roll and pitch; yaw is controlled separately
    # attCtrl gives desired angular rates from desired attitude
    # rateCtrl gives desired moments from desired angular rates
    # Motor mixer then produces desired motor speeds from desired thrust and moments

    # Altitude -> thrust. Divide by cos(tilt) so the vertical part of the thrust still balances gravity when tilted
    thrustCorrection = altCtrl.update(targetAltitude=-target[2], measuredAltitude=-currentPos[2], dt=dt)
    cosTilt = max(currentRotation.as_matrix()[2, 2], 0.5) # body z-axis vs world z-axis, clamped to avoid blow-up
    desiredThrust = np.clip((hoverThrust + thrustCorrection) / cosTilt, thrustMin, thrustMax)

    # Position -> world acceleration -> heading-frame acceleration -> roll and pitch
    accelWorld = posCtrl.update(targetXY=target[0:2], measuredXY=currentPos[0:2], dt=dt)
    c, s = np.cos(currentYaw), np.sin(currentYaw)
    accelForward = c * accelWorld[0] + s * accelWorld[1]
    accelRight = -s * accelWorld[0] + c * accelWorld[1]
    desiredPitch = np.clip(-np.arctan(accelForward / drone.g), -maxTilt, maxTilt) # nose down to accelerate forward
    desiredRoll = np.clip(np.arctan(accelRight / drone.g), -maxTilt, maxTilt)     # right side down to accelerate right

    # Convert Euler angles to target quaternion. 'ZYX' (upper case) = intrinsic yaw-pitch-roll, the aerospace convention
    desiredQuat = Rotation.from_euler('ZYX', [desiredYaw, desiredPitch, desiredRoll]).as_quat()
    desiredQuat = np.roll(desiredQuat, 1) # Converts from [x,y,z,w] to [w,x,y,z]

    desiredAngVel = attCtrl.update(targetQuat=desiredQuat, measuredQuat=currentOrientation, dt=dt)
    desiredMoments = rateCtrl.update(targetRate=desiredAngVel, measuredRate=currentAngVel, dt=dt)

    desiredMotorSpeeds = drone.mixer(desiredThrust, desiredMoments)

    drone.step(desiredMotorSpeeds, dt)

    motorSpeedHistory.append(drone.motorSpeeds.copy())
    desiredAttitudeHistory.append([desiredRoll, desiredPitch, desiredYaw])

    trajectory3D.update(positionHistory)

    time += dt


print("Final position [m]:", np.round(drone.position, 2), " target:", target)

# Convert histories to arrays: row = time step
timeHistory = np.array(timeHistory)
positionHistory = np.array(positionHistory)
velocityHistory = np.array(velocityHistory)
motorSpeedHistory = np.array(motorSpeedHistory)
attitudeHistory = np.array(attitudeHistory)
desiredAttitudeHistory = np.array(desiredAttitudeHistory)

plotMotorSpeeds(timeHistory, motorSpeedHistory, omegaMax=drone.omegaMax)
plotAttitude(timeHistory, attitudeHistory, desiredAttitudeHistory)
trajectory3D.finish() # shows all windows and keeps them open
