import numpy as np
from scipy.spatial.transform import Rotation
from visualisation import PathPlotter3D, plotMotorSpeeds, plotAttitude
from config import simType, drone, dt, simSteps, target, attCtrl, rateCtrl
from flightModes import autoFlightMode, manualFlightMode
from setup.radioCheck import chooseJoystick


def main():
    """Run the whole simulation: simulation loop, then the result plots."""

    # State history, one entry per step
    time = 0                      # Elapsed time of simulation [s]
    timeHistory = []              # [s]
    positionHistory = []          # world [x, y, z] [m]
    velocityHistory = []          # world [vx, vy, vz] [m/s]
    motorSpeedHistory = []        # motors 1..4 [rad/s]
    attitudeHistory = []          # [roll, pitch, yaw] [rad]
    desiredAttitudeHistory = []   # commanded [roll, pitch, yaw] [rad]

    # Create graph plotter object
    trajectory3D = PathPlotter3D(target, updateFrequency=5, limit=25)
    if simType =="auto":
        # Simulation loop
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

            # Cascaded PID loops:
            # The flight mode gives the desired attitude and thrust (auto: from the position and altitude controllers)
            # attCtrl gives desired angular rates from desired attitude
            # rateCtrl gives desired moments from desired angular rates
            # Motor mixer then produces desired motor speeds from desired thrust and moments
            setpoint = autoFlightMode(stateVector, dt)

            # Convert Euler angles to target quaternion. 'ZYX' (upper case) = intrinsic yaw-pitch-roll, the aerospace convention
            desiredQuat = Rotation.from_euler('ZYX', [setpoint.yaw, setpoint.pitch, setpoint.roll]).as_quat()
            desiredQuat = np.roll(desiredQuat, 1) # Converts from [x,y,z,w] to [w,x,y,z]

            desiredAngVel = attCtrl.update(targetQuat=desiredQuat, measuredQuat=currentOrientation, dt=dt)
            desiredMoments = rateCtrl.update(targetRate=desiredAngVel, measuredRate=currentAngVel, dt=dt)

            desiredMotorSpeeds = drone.mixer(setpoint.thrust, desiredMoments)

            drone.step(desiredMotorSpeeds, dt)

            motorSpeedHistory.append(drone.motorSpeeds.copy())
            desiredAttitudeHistory.append([setpoint.roll, setpoint.pitch, setpoint.yaw])

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
    elif simType == "manual":
        
        joystick = chooseJoystick()
        stateVector = drone.getState()

        manualFlightMode(stateVector, dt, joystick)

        pass
    else:
        raise Exception("Unknown simulation type selected")

if __name__ == "__main__":
    main()
