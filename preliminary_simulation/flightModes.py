from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation
import pygame
import os
import csv
from setup.radioCheck import chooseJoystick

from config import drone, target, desiredYaw, hoverThrust, thrustMin, thrustMax, maxTilt, altCtrl, posCtrl, maxRate

#Joystick variable
joystick = None
currentManualYaw = 0.0


@dataclass
class AttitudeSetpoint:
    """Output of every flight mode: the attitude and thrust the attitude and rate loops should achieve."""

    roll: float     # [rad], positive = right side down
    pitch: float    # [rad], positive = nose up
    yaw: float      # [rad], heading relative to world x (north)
    thrust: float   # total thrust magnitude [N]


def autoFlightMode(stateVector, dt):
    """Auto (position) mode: fly to the target position from config and hold it.

    altCtrl gives a thrust correction from altitude error
    posCtrl gives desired world accelerations from position error -> converted to roll and pitch; yaw is controlled separately

    Args:
        stateVector: 13-element drone state [position(3), velocity(3), quaternion(4), angular velocity(3)]
        dt: time step [s]

    Returns:
        AttitudeSetpoint for the attitude and rate loops
    """
    currentPos = stateVector[0:3] # Current position vector [x, y, z]
    currentOrientation = stateVector[6:10] # Current orientation quaternion [w, x, y, z]

    currentRotation = Rotation.from_quat(np.roll(currentOrientation, -1))
    currentYaw = currentRotation.as_euler('ZYX')[0] # intrinsic yaw-pitch-roll

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

    return AttitudeSetpoint(roll=desiredRoll, pitch=desiredPitch, yaw=desiredYaw, thrust=desiredThrust)



def manualFlightMode(stateVector:np.array, dt:float):

    global joystick, currentManualYaw

    def applyDeadband(value, deadband = 0.05):
        '''Apply a deadband, mapping values below 0.05 to 0'''

        if abs(value) < deadband:
            return 0
        return value 

    def getCalibrationVals(file = "setup/radio_check.csv"):
        ''' Read radio_check.csv to get the max and min values necessary to calibrate the joystick inputs'''

        if not os.path.exists(file):
            raise FileNotFoundError("File path not found.")

        minVals = []
        maxVals = []

        with open(file, 'r') as f: 
            reader =csv.DictReader(f)
            for row in reader:
                minVals.append(float(row["Min"]))
                maxVals.append(float(row["Max"]))

        return np.array(minVals), np.array(maxVals)

    def calibrateInputs(input, minVal, maxVal, centreVal = 0):
        '''Use max, min and centre values, to calibrate joystick inputs'''

        if input >= centreVal:
            input = (input - centreVal)/(maxVal- centreVal)
        else:
            input = (input -centreVal)/(centreVal - minVal)        

        return np.clip(input, -1.0, 1.0)

    #Use chooseJoystick function from radioCheck.py to get joystick variable
    if joystick is None:
        joystick = chooseJoystick()

    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEREMOVED:
            raise Exception("Joystick removed")

    #Assuming axes 0, 1, 2, 3 map to  Roll, Pitch, Throttle, Yaw respectively
    #Some axes may be inverted, may need to multiply some inputs by -1
    calMin, calMax = getCalibrationVals()
    inputs = []

    for i in range(4):
        inputs.append(applyDeadband(calibrateInputs(joystick.get_axis(i),calMin[i],calMax[i])))
    rollInput = inputs[0]
    pitchInput = inputs[1]
    throttleInput = np.clip(((inputs[2]+1.0)/2.0),0.0, 1.0)
    yawInput = inputs[3]
   


    #Use joystick inputs to get the desired thrust, roll angle, pitch angle and yaw rate, 
    #mapped between their minimum and maximum values
    desiredThrust = thrustMin + throttleInput *( thrustMax - thrustMin)
    desiredRoll = rollInput * maxTilt
    desiredPitch = pitchInput * maxTilt
    desiredYawRate = yawInput * maxRate[2]

    currentManualYaw += desiredYawRate * dt

    return AttitudeSetpoint(roll = desiredRoll, pitch = desiredPitch, yaw = currentManualYaw, thrust = desiredThrust)
    
        