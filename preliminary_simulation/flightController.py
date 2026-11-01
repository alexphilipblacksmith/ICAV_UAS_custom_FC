"""Inner loops shared by every flight mode: attitude -> rate -> mixer with motor saturation handling"""

import numpy as np
from scipy.spatial.transform import Rotation

from config import drone, attCtrl, rateCtrl, maxRate


def computeMotorSpeeds(setpoint, stateVector, dt):
    """Turn a flight mode's AttitudeSetpoint into motor speeds.

    attCtrl gives desired angular rates from the attitude error
    rateCtrl gives desired moments from the rate error
    desaturatedMixer gives motor speeds from desired thrust and moments

    Args:
        setpoint: AttitudeSetpoint from a flight mode
        stateVector: 13-element drone state [position(3), velocity(3), quaternion(4), angular velocity(3)]
        dt: time step [s]

    Returns:
        motor speeds 1..4 [rad/s]
    """
    currentOrientation = stateVector[6:10] # Current orientation quaternion [w, x, y, z]
    currentAngVel = stateVector[10:13] # Current angular velocity vector [p, q, r]

    # Convert Euler angles to target quaternion
    desiredQuat = Rotation.from_euler('ZYX', [setpoint.yaw, setpoint.pitch, setpoint.roll]).as_quat()
    desiredQuat = np.roll(desiredQuat, 1) # Converts from [x,y,z,w] to [w,x,y,z]

    desiredAngVel = attCtrl.update(targetQuat=desiredQuat, measuredQuat=currentOrientation, dt=dt)

    desiredAngVel[2] += setpoint.yawRate
    desiredAngVel = np.clip(desiredAngVel, -np.asarray(maxRate), maxRate)

    desiredMoments = rateCtrl.update(targetRate=desiredAngVel, measuredRate=currentAngVel, dt=dt)

    return desaturatedMixer(setpoint.thrust, desiredMoments)


def desaturatedMixer(thrust, moments):
    """Mixer that keeps attitude control when a motor would go above omegaMax or below zero."""
    speedsSquared = drone.motorSpeedsSquared(thrust, moments)
    maxSquared = drone.omegaMax**2

    spread = speedsSquared.max() - speedsSquared.min()
    if spread > maxSquared:
        mean = speedsSquared.mean()
        speedsSquared = mean + (speedsSquared - mean) * (maxSquared / spread)

    if speedsSquared.max() > maxSquared:
        speedsSquared -= speedsSquared.max() - maxSquared
    if speedsSquared.min() < 0.0:
        speedsSquared -= speedsSquared.min()

    return np.sqrt(np.clip(speedsSquared, 0.0, maxSquared))


def resetControllers():
    """Clear the integrators of the inner loops, e.g. while the drone sits on the ground."""
    attCtrl.reset()
    rateCtrl.reset()
