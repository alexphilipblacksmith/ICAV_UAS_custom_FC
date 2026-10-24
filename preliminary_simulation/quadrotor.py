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
        d_13 : distance between motors 1 and 3 (front-back pair)  [m]
        d_24 : distance between motors 2 and 4 (right-left pair)  [m]
        z_13 : vertical position of the 1-3 pair w.r.t. CoG (+ down)  [m]
        z_24 : vertical position of the 2-4 pair w.r.t. CoG (+ down)  [m]
        The vertical offset between the pairs is z_24 - z_13.
    """

    # Geometry of the quadrotor
    d_13: float = 0.30
    d_24: float = 0.30
    z_13: float = 0.0
    z_24: float = 0.0

    # Mass properties
    mass: float = 1.5                                   # [kg]
    inertia: np.ndarray = field(                        # [kg m^2], body frame
        default_factory=lambda: np.diag([0.02, 0.02, 0.04]))

    # Propulsion
    k_thrust: float = 1.0e-5                            # T = k_thrust * w^2  [N s^2]
    k_torque: float = 1.0e-6                            # Q = k_torque * w^2  [N m s^2]
    motor_dirs: np.ndarray = field(                     # +1 CW, -1 CCW (from above)
        default_factory=lambda: np.array([1, -1, 1, -1]))
    omega_max: float = 1000.0                           # [rad/s]

    # Environment
    g: float = 9.81                                     # [m/s^2]

    # State
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))         # world [m]
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))         # world [m/s]
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # [w, x, y, z]
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3)) # body [rad/s]
    motor_speeds: np.ndarray = field(default_factory=lambda: np.zeros(4))     # [rad/s]

    def __post_init__(self):
        self.inertia = np.asarray(self.inertia, dtype=float)
        self.motor_dirs = np.asarray(self.motor_dirs)
        if self.mass <= 0:
            raise ValueError("Mass must be positive.")
        if self.inertia.shape != (3, 3):
            raise ValueError("Inertia must be a 3x3 matrix.")
        if self.d_13 <= 0 or self.d_24 <= 0:
            raise ValueError("Motor distances d_13 and d_24 must be positive.")

    # Geometry
    @property
    def vertical_offset(self) -> float:
        """Vertical offset between the 2-4 pair and the 1-3 pair [m]."""
        return self.z_24 - self.z_13

    @property
    def motor_positions(self) -> np.ndarray:
        """Motor positions in the body frame, shape (4, 3), rows = motors 1..4."""
        return np.array([
            [ self.d_13 / 2, 0.0,            self.z_13],   # 1 front
            [ 0.0,           self.d_24 / 2,  self.z_24],   # 2 right
            [-self.d_13 / 2, 0.0,            self.z_13],   # 3 back
            [ 0.0,          -self.d_24 / 2,  self.z_24],   # 4 left
        ])

    # Propulsion
    def motor_thrusts(self, motor_speeds: np.ndarray) -> np.ndarray:
        """Motor thrusts in the body frame, shape (4,), rows = motors 1..4. 
        Only magnitude, no direction. T = k_thrust * w^2."""
        return self.k_thrust * motor_speeds**2

    def motor_torques(self, motor_speeds: np.ndarray) -> np.ndarray:
        """Reaction yaw torques on the body about body z, shape (4,), rows = motors 1..4.
        Signed: opposite to the propeller spin. Q = -dir * k_torque * w^2."""
        return -self.k_torque * motor_speeds**2 * self.motor_dirs

    def forces_and_moments(self, motor_speeds: np.ndarray):
        """Compute total forces and moments in the body frame from motor speeds.

        Returns:
            force: total thrust force vector in the body frame (points along -z) [N]
            moments: total moments about body axes [N m]
        """
        motor_speeds = np.clip(motor_speeds, 0.0, self.omega_max)
        thrusts = self.motor_thrusts(motor_speeds)
        torques = self.motor_torques(motor_speeds)

        # Per-motor thrust vectors, thrust acts along -z (up) in FRD
        zeros = np.zeros(4)
        motor_forces = np.column_stack([zeros, zeros, -thrusts])  # shape (4, 3)

        force = motor_forces.sum(axis=0)

        # Moments about body axes: r x F for each motor plus reaction torques
        moments = np.cross(self.motor_positions, motor_forces).sum(axis=0)
        moments[2] += torques.sum()

        return force, moments

    def mixer(self, thrust: float, moments: np.ndarray) -> np.ndarray:
        """Compute motor speeds from desired thrust and moments.

        Args:
            thrust: desired total thrust along body z-axis [N]
            moments: desired moments about body axes [N m]"""

        # Mixer matrix
        l1 = self.d_13 / 2
        l2 = self.d_24 / 2
        kf = self.k_thrust
        km = self.k_torque

        mixer_matrix = np.array([
            [ kf,     kf,     kf,     kf],      # thrust magnitude
            [ 0,     -l2*kf,  0,      l2*kf],   # roll
            [ l1*kf,  0,     -l1*kf,  0],       # pitch
            -km * self.motor_dirs,              # yaw (reaction torques)
        ])

        desired = np.array([thrust, moments[0], moments[1], moments[2]])
        motor_speeds_squared = np.linalg.solve(mixer_matrix, desired)
        motor_speeds_squared = np.clip(motor_speeds_squared, 0.0, None)  # No negative speeds

        return np.clip(np.sqrt(motor_speeds_squared), 0.0, self.omega_max)

    # Dynamics
    def rotation_matrix(self) -> np.ndarray:
        """Rotation matrix to map body-frame vectors to world-frame vectors"""


        # Quarternion is in [w, x, y, z] by code convention
        #Scipy takes quaternion in [x, y, z, w] order, so np.roll reorders the quaternion array
        return Rotation.from_quat(np.roll(self.quaternion, -1)).as_matrix() # create a rotation matrix from the quaternion

    

    def derivatives(self, state: np.ndarray, motor_speeds: np.ndarray) -> np.ndarray:
        pass

    def step(self, motor_speeds: np.ndarray, dt: float):
        pass

    # State helpers
    def get_state(self) -> np.ndarray:
        pass

    def set_state(self, state: np.ndarray):
        pass

    def reset(self):
        pass

