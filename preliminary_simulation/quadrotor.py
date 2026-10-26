from dataclasses import dataclass, field
import numpy as np
from scipy.spatial.transform import Rotation

@dataclass
class Quadrotor:
    """Plus configuration quadrotor.

    Body frame (FRD): x forward, y right, z down. Origin at the centre of gravity.

        Motor layout (top view):

                    1 (front, +x)
                    |
        4 (left) ---+--- 2 (right, +y)
                    |
                    3 (back)

    Geometry:
        d13 : distance between motors 1 and 3 (front-back pair)  [m]
        d24 : distance between motors 2 and 4 (right-left pair)  [m]
        z13 : vertical position of the 1-3 pair w.r.t. CoG (+ down)  [m]
        z24 : vertical position of the 2-4 pair w.r.t. CoG (+ down)  [m]
        The vertical offset between the pairs is z24 - z13.
    """

    # Geometry of the quadrotor
    d13: float = 0.30
    d24: float = 0.30
    z13: float = 0.0
    z24: float = 0.0

    # Mass properties
    mass: float = 1.5                                   # [kg]
    inertia: np.ndarray = field(                        # [kg m^2], body frame
        default_factory=lambda: np.diag([0.02, 0.02, 0.04]))

    # Propulsion
    kThrust: float = 1.0e-5                            # T = kThrust * w^2  [N s^2]
    kTorque: float = 1.0e-6                            # Q = kTorque * w^2  [N m s^2]
    motorDirs: np.ndarray = field(                     # +1 CW, -1 CCW (from above)
        default_factory=lambda: np.array([1, -1, 1, -1]))
    omegaMax: float = 1000.0                           # [rad/s]

    # Environment
    g: float = 9.81                                     # [m/s^2]

    # State
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))         # world [m]
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))         # world [m/s]
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # [w, x, y, z]
    angularVelocity: np.ndarray = field(default_factory=lambda: np.zeros(3)) # body [rad/s]
    motorSpeeds: np.ndarray = field(default_factory=lambda: np.zeros(4))     # [rad/s]

    def __post_init__(self):
        self.inertia = np.asarray(self.inertia, dtype=float)
        self.motorDirs = np.asarray(self.motorDirs)
        self.position = np.asarray(self.position, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        self.quaternion = np.asarray(self.quaternion, dtype=float)
        self.angularVelocity = np.asarray(self.angularVelocity, dtype=float)
        self.motorSpeeds = np.asarray(self.motorSpeeds, dtype=float)

        if self.mass <= 0:
            raise ValueError("Mass must be positive.")
        if self.inertia.shape != (3, 3):
            raise ValueError("Inertia must be a 3x3 matrix.")
        if self.d13 <= 0 or self.d24 <= 0:
            raise ValueError("Motor distances d13 and d24 must be positive.")

        self.setState(self.getState())               # validates and normalises the quaternion
        self._initialState = self.getState()         # used by reset()

    # Geometry
    @property
    def verticalOffset(self) -> float:
        """Vertical offset between the 2-4 pair and the 1-3 pair [m]."""
        return self.z24 - self.z13

    @property
    def motorPositions(self) -> np.ndarray:
        """Motor positions in the body frame, shape (4, 3), rows = motors 1..4."""
        return np.array([
            [ self.d13 / 2, 0.0,            self.z13],   # 1 front
            [ 0.0,           self.d24 / 2,  self.z24],   # 2 right
            [-self.d13 / 2, 0.0,            self.z13],   # 3 back
            [ 0.0,          -self.d24 / 2,  self.z24],   # 4 left
        ])

    # Propulsion
    def motorThrusts(self, motorSpeeds: np.ndarray) -> np.ndarray:
        """Motor thrust magnitudes, shape (4,), rows = motors 1..4.
        No direction (the thrust acts along body -z). T = kThrust * w^2."""
        return self.kThrust * motorSpeeds**2

    def motorTorques(self, motorSpeeds: np.ndarray) -> np.ndarray:
        """Reaction yaw torques on the body about body z, shape (4,), rows = motors 1..4.
        Signed: opposite to the propeller spin. Q = -dir * kTorque * w^2."""
        return -self.kTorque * motorSpeeds**2 * self.motorDirs

    def forcesAndMoments(self, motorSpeeds: np.ndarray):
        """Compute total forces and moments in the body frame from motor speeds.

        Returns:
            force: total thrust force vector in the body frame (points along -z) [N]
            moments: total moments about body axes [N m]
        """
        motorSpeeds = np.clip(motorSpeeds, 0.0, self.omegaMax)
        thrusts = self.motorThrusts(motorSpeeds)
        torques = self.motorTorques(motorSpeeds)

        # Per-motor thrust vectors, thrust acts along -z in FRD
        zeros = np.zeros(4)
        motorForces = np.column_stack([zeros, zeros, -thrusts])  # shape (4, 3)

        force = motorForces.sum(axis=0)

        # Moments about body axes: r x F for each motor plus reaction torques
        moments = np.cross(self.motorPositions, motorForces).sum(axis=0)
        moments[2] += torques.sum()

        return force, moments

    def mixer(self, thrust: float, moments: np.ndarray) -> np.ndarray:
        """Compute motor speeds from desired thrust and moments.

        Args:
            thrust: desired total thrust magnitude (positive, acts along body -z) [N]
            moments: desired moments about body axes [N m]"""

        # Mixer matrix
        l1 = self.d13 / 2
        l2 = self.d24 / 2
        kf = self.kThrust
        km = self.kTorque

        mixerMatrix = np.array([
            [ kf,     kf,     kf,     kf],      # thrust magnitude
            [ 0,     -l2*kf,  0,      l2*kf],   # roll
            [ l1*kf,  0,     -l1*kf,  0],       # pitch
            -km * self.motorDirs,               # yaw (reaction torques)
        ])

        desired = np.array([thrust, moments[0], moments[1], moments[2]])
        motorSpeedsSquared = np.linalg.solve(mixerMatrix, desired)
        motorSpeedsSquared = np.clip(motorSpeedsSquared, 0.0, None)  # No negative speeds

        return np.clip(np.sqrt(motorSpeedsSquared), 0.0, self.omegaMax)

    # Dynamics
    def rotationMatrix(self, q: np.ndarray = None) -> np.ndarray:
        """Rotation matrix mapping body-frame vectors to world-frame vectors.

        Takes the quaternion as an argument so RK4 can evaluate intermediate states.
        Defaults to the current quaternion.
        """
        if q is None:
            q = self.quaternion

        # Quaternion is in [w, x, y, z] by code convention
        # Scipy takes quaternion in [x, y, z, w] order, so np.roll reorders the quaternion array
        return Rotation.from_quat(np.roll(q, -1)).as_matrix()

    def derivatives(self, state: np.ndarray, motorSpeeds: np.ndarray) -> np.ndarray:
        """State derivative x_dot = f(x, u). Reads only from `state`, never from self.* state fields.

        Returns:
            13-element vector: [velocity(3), acceleration(3), qDot(4), angularAcceleration(3)]
        """
        velocity = state[3:6]
        quaternion = state[6:10]
        angularVelocity = state[10:13]

        forcesBody, momentsBody = self.forcesAndMoments(motorSpeeds)

        # Translational dynamics: rotate body thrust to world frame and add gravity (world z is down)
        forcesWorld = self.rotationMatrix(quaternion) @ forcesBody
        acceleration = forcesWorld / self.mass + np.array([0.0, 0.0, self.g])

        # Rotational dynamics (Euler's equation): omega_dot = I_inv (M - omega x (I omega))
        angularAcceleration = np.linalg.solve(
            self.inertia, momentsBody - np.cross(angularVelocity, self.inertia @ angularVelocity))

        # Quaternion kinematics: qDot = 1/2 * q (x) [0, omega]
        w, x, y, z = quaternion
        p, q, r = angularVelocity
        quaternionMat = np.array([[w, -x, -y, -z],
                                   [x,  w, -z,  y],
                                   [y,  z,  w, -x],
                                   [z, -y,  x,  w]])
        qDot = 0.5 * quaternionMat @ np.array([0.0, p, q, r])

        return np.concatenate([velocity, acceleration, qDot, angularAcceleration])

    def step(self, motorSpeeds: np.ndarray, dt: float):
        """Advance the simulation by dt seconds with RK4, holding the motor speeds constant over the step."""

        if dt <= 0:
            raise ValueError("Time step dt must be positive.")

        self.motorSpeeds = np.clip(np.asarray(motorSpeeds, dtype=float), 0.0, self.omegaMax)

        x = self.getState()
        u = self.motorSpeeds

        k1 = self.derivatives(x, u)
        k2 = self.derivatives(x + 0.5 * dt * k1, u)
        k3 = self.derivatives(x + 0.5 * dt * k2, u)
        k4 = self.derivatives(x + dt * k3, u)

        xNew = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

        # Keep w >= 0 (q and -q are the same orientation)
        if xNew[6] < 0:
            xNew[6:10] *= -1

        self.setState(xNew)  # also normalises the quaternion
        self._groundContact()

    def _groundContact(self):
        """Simple flat ground at z = 0 (world z is down): the drone cannot sink below it."""
        if self.position[2] >= 0.0 and self.velocity[2] > 0.0:
            self.position[2] = 0.0
            self.velocity[2] = 0.0

    # State helpers
    def getState(self) -> np.ndarray:
        """Return the current state as a 13-element vector: [position(3), velocity(3), quaternion(4), angularVelocity(3)]."""

        return np.concatenate([self.position, self.velocity, self.quaternion, self.angularVelocity])

    def setState(self, state: np.ndarray):
        """Set the current state from a 13-element vector: [position(3), velocity(3), quaternion(4), angularVelocity(3)]."""
        state = np.asarray(state, dtype=float)
        if len(state) != 13:
            raise ValueError("State vector must have 13 elements.")
        self.position = state[0:3].copy()
        self.velocity = state[3:6].copy()
        self.quaternion = state[6:10].copy()
        self.angularVelocity = state[10:13].copy()

        # Ensure quaternion can be and is normalised
        qNorm = np.linalg.norm(self.quaternion)
        if qNorm > 0 and np.isfinite(qNorm):
            self.quaternion /= qNorm
        else:
            raise ValueError("Quaternion norm is zero or not finite, cannot normalize.")

    def reset(self):
        """Return to the state the drone was created with."""
        self.setState(self._initialState)
        self.motorSpeeds = np.zeros(4)
