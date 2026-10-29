from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from config import drone, target, desiredYaw, hoverThrust, thrustMin, thrustMax, maxTilt, altCtrl, posCtrl


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
