import time as clock

import numpy as np
from scipy.spatial.transform import Rotation
from visualisation import PathPlotter3D, LiveDashboard, plotMotorSpeeds, plotAttitude
from config import (simType, drone, dt, simSteps, target, manualDt, maxLag, calibrationFile, joystickIndex,
                    radioAxes, radioSigns, stickDeadband, armThrottleLimit,
                    dashboardWindow, dashboardFrameRate, arenaSize)
from flightModes import autoFlightMode, ManualFlightMode
from flightController import computeMotorSpeeds, resetControllers
from radioInput import RadioInput, RadioDisconnected


def main():
    """Run the whole simulation: simulation loop, then the result plots."""

    if simType == "auto":
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

        # Simulation loop
        for step in range(simSteps):

            # Get state of the drone
            stateVector = drone.getState()
            currentPos = stateVector[0:3] # Current position vector [x, y, z]
            currentVel = stateVector[3:6] # Current velocity vector [vx, vy, vz] (world)
            currentOrientation = stateVector[6:10] # Current orientation quaternion [w, x, y, z]

            currentRotation = Rotation.from_quat(np.roll(currentOrientation, -1))
            currentYaw, currentPitch, currentRoll = currentRotation.as_euler('ZYX') # intrinsic yaw-pitch-roll

            # Store state history
            timeHistory.append(time)
            positionHistory.append(currentPos)
            velocityHistory.append(currentVel)
            attitudeHistory.append([currentRoll, currentPitch, currentYaw])

            setpoint = autoFlightMode(stateVector, dt)
            desiredMotorSpeeds = computeMotorSpeeds(setpoint, stateVector, dt)

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
        # Manual level mode, run in real time with the radio.
        radio = RadioInput(calibrationFile, radioAxes, radioSigns, deadband=stickDeadband, joystickIndex=joystickIndex)
        manualMode = ManualFlightMode()
        drone.arena = arenaSize # confine the drone to the arena (auto mode flies in open space)
        dashboard = LiveDashboard(drone, window=dashboardWindow, dt=manualDt)

        try:
            # Safety check: do not start with the throttle up, the drone would jump off the ground
            print("Lower the throttle to start.")
            while radio.read().throttle > armThrottleLimit:
                if not dashboard.isOpen():
                    return
                dashboard.processEvents()
                clock.sleep(0.05)
            manualMode.holdCurrentHeading(drone.getState())
            print("Armed. Close the window or press Ctrl+C to stop.")

            # Fixed time step physics, paced against the wall clock
            time = 0.0                              # Simulation time [s]
            startTime = clock.perf_counter()        # Wall clock time at simulation time 0 [s]
            frameTime = 1.0 / dashboardFrameRate

            while dashboard.isOpen():
                frameStart = clock.perf_counter()

                # Skip ahead instead of trying to catch up if the computer could not keep up (e.g. window dragged)
                realTime = frameStart - startTime
                if realTime - time > maxLag:
                    startTime += realTime - time - maxLag
                    realTime = frameStart - startTime

                while time < realTime:
                    command = radio.read()
                    stateVector = drone.getState()
                    currentOrientation = stateVector[6:10] # Current orientation quaternion [w, x, y, z]

                    currentRotation = Rotation.from_quat(np.roll(currentOrientation, -1))
                    currentYaw, currentPitch, currentRoll = currentRotation.as_euler('ZYX') # intrinsic yaw-pitch-roll

                    # Sitting on the ground with the throttle down: clear the integrators so they do not wind up
                    onGround = stateVector[2] >= 0.0
                    if onGround and command.throttle < armThrottleLimit:
                        resetControllers()

                    setpoint = manualMode.update(stateVector, manualDt, command)
                    desiredMotorSpeeds = computeMotorSpeeds(setpoint, stateVector, manualDt)

                    drone.step(desiredMotorSpeeds, manualDt)

                    dashboard.record(time, stateVector[0:3], drone.motorSpeeds,
                                     [currentRoll, currentPitch, currentYaw],
                                     [setpoint.roll, setpoint.pitch, setpoint.yaw])
                    time += manualDt

                dashboard.draw(drone.rotationMatrix())

                # Wait for the next frame if there is time left
                clock.sleep(max(0.0, frameTime - (clock.perf_counter() - frameStart)))

        except KeyboardInterrupt:
            pass
        except RadioDisconnected as error:
            print(error)
        finally:
            radio.close()
            print("Final position [m]:", np.round(drone.position, 2))

    else:
        raise ValueError(f"Unknown simulation type '{simType}', use 'auto' or 'manual'")

if __name__ == "__main__":
    main()
