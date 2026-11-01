from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from config import drone, target, desiredYaw, hoverThrust, maxThrust, thrustMin, thrustMax, maxTilt, altCtrl, posCtrl, maxRate


@dataclass
class AttitudeSetpoint:
    """Output of every flight mode: the attitude and thrust the attitude and rate loops should achieve."""

    roll: float     # [rad], positive = right side down
    pitch: float    # [rad], positive = nose up
    yaw: float      # [rad], heading relative to world x (north)
    thrust: float   # total thrust magnitude [N]
    yawRate: float = 0.0  # yaw rate feedforward [rad/s], used when the heading target is moving (manual yaw stick)


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


@dataclass
class ManualFlightMode:
    """Manual level (angle) mode: the sticks command tilt angles, the drone levels itself when they are released.

    roll / pitch stick -> roll / pitch angle, up to maxTilt
    yaw stick          -> yaw rate, up to maxRate[2]; the heading is held when the stick is centred
    throttle stick     -> total thrust: bottom = thrustMin, middle = hoverThrust, top = maxThrust (all motors at omegaMax)
    """

    heading: float = 0.0 # heading target [rad], integrated from the yaw stick

    def update(self, stateVector, dt, command):
        """
        Args:
            stateVector: 13-element drone state [position(3), velocity(3), quaternion(4), angular velocity(3)]
            dt: time step [s]
            command: RadioCommand with scaled stick positions

        Returns:
            AttitudeSetpoint for the attitude and rate loops
        """
        # Throttle curve in two straight pieces so that mid-stick hovers, which makes height easier to hold
        if command.throttle < 0.5:
            desiredThrust = thrustMin + 2 * command.throttle * (hoverThrust - thrustMin)
        else:
            # Full stick is the motors' limit, so a stick input at full throttle slows some motors down to give the max thrust, not more
            desiredThrust = hoverThrust + 2 * (command.throttle - 0.5) * (maxThrust - hoverThrust)
        desiredRoll = command.roll * maxTilt
        desiredPitch = -command.pitch * maxTilt
        desiredYawRate = command.yaw * maxRate[2]

        self.heading = (self.heading + desiredYawRate * dt + np.pi) % (2 * np.pi) - np.pi

        return AttitudeSetpoint(roll=desiredRoll, pitch=desiredPitch, yaw=self.heading,
                                thrust=desiredThrust, yawRate=desiredYawRate)

    def holdCurrentHeading(self, stateVector):
        """Set the heading target to the drone's current heading (used before take-off)."""
        currentRotation = Rotation.from_quat(np.roll(stateVector[6:10], -1))
        self.heading = currentRotation.as_euler('ZYX')[0]
