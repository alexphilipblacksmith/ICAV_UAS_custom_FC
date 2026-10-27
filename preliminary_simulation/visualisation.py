from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np

radPerSecToRpm = 60 / (2 * np.pi)  # [rad/s] -> [rpm]

motorNames = ["Motor 1 (front, CW)", "Motor 2 (right, CCW)", "Motor 3 (back, CW)", "Motor 4 (left, CCW)"]

@dataclass(eq=False)
class PathPlotter3D:
    """Live 3D plot window of the drone's trajectory.

    Args:
        target: [x, y, z] target location
        updateFrequency: Redraws every N steps to keep simulation running smoothly.
        limit: Sets the lower and upper bounds for X, Y, and Z axes (-limit to limit).
    """

    # Settings (constructor arguments)
    target: np.ndarray
    updateFrequency: int = 5
    limit: float = 20

    # Created in __post_init__ / changed while running (not constructor arguments)
    fig: plt.Figure = field(init=False, repr=False)
    ax: plt.Axes = field(init=False, repr=False)
    line: object = field(init=False, repr=False)
    stepCount: int = field(default=0, init=False)
    startPlotted: bool = field(default=False, init=False)

    def __post_init__(self):
        """Opens the live 3D plot window."""
        self.target = np.asarray(self.target, dtype=float)

        plt.ion() # Turn on interactive mode

        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Set fixed bounds from -limit to limit for all axes
        self.setAxisLimits()

        # Plot static elements (Target)
        self.ax.scatter(
            self.target[0], 
            self.target[1], 
            -self.target[2], 
            color='crimson', 
            s=140, 
            label='Target Position', 
            marker='X'
        )
        
        # Axis labels and formatting
        self.ax.set_xlabel('X Position [m]')
        self.ax.set_ylabel('Y Position [m]')
        self.ax.set_zlabel('Altitude [m]')
        self.ax.set_title('Live Quadrotor 3D Flight Trajectory')
        self.ax.legend(loc='upper left')
        self.ax.grid(True)
        
        # Create an empty line object for the drone's path
        self.line, = self.ax.plot([], [], [], label='Drone Trajectory', color='royalblue', linewidth=2)

    def setAxisLimits(self):
        """Sets explicit fixed limits from -limit to limit for all axes."""
        self.ax.set_xlim(-self.limit, self.limit)
        self.ax.set_ylim(-self.limit, self.limit)
        self.ax.set_zlim(-self.limit, self.limit)

    def update(self, positionHistory):
        """Updates the line data and refreshes the canvas."""
        self.stepCount += 1
        
        # Only update every N steps to prevent rendering lag
        if self.stepCount % self.updateFrequency != 0:
            return
            
        pos = np.array(positionHistory)
        if len(pos) == 0:
            return
            
        # Plot start point once on the first valid step
        if not self.startPlotted:
            self.ax.scatter(
                pos[0, 0], 
                pos[0, 1], 
                -pos[0, 2], 
                color='forestgreen', 
                s=100, 
                label='Start Point', 
                marker='o'
            )
            self.startPlotted = True

        # Update the 3D line coordinates
        self.line.set_data(pos[:, 0], pos[:, 1])
        self.line.set_3d_properties(-pos[:, 2])
        
        # Keep fixed limits
        self.setAxisLimits()
        
        # Redraw canvas and process GUI events
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def finish(self):
        """Keeps the final plot windows open after the simulation ends."""
        plt.ioff()
        plt.show()


def plotMotorSpeeds(timeHistory, motorSpeedHistory, omegaMax=None):
    """Window with 4 graphs: speed of each motor vs time, in rpm.

    Args:
        timeHistory: shape (N,) [s]
        motorSpeedHistory: shape (N, 4), motors 1..4 [rad/s]
        omegaMax: optional motor speed limit [rad/s], drawn as a dashed line
    """
    rpm = np.asarray(motorSpeedHistory) * radPerSecToRpm

    fig, axes = plt.subplots(2, 2, sharex=True, sharey=True, figsize=(12, 8))
    fig.canvas.manager.set_window_title("Motor speeds")

    for i, ax in enumerate(axes.flat):
        ax.plot(timeHistory, rpm[:, i], color="royalblue", linewidth=1.2)
        if omegaMax is not None:
            ax.axhline(omegaMax * radPerSecToRpm, color="grey", linestyle="--", linewidth=1, label="Limit")
            ax.legend(loc="lower right")
        ax.set_title(motorNames[i])
        ax.grid(True)

    for ax in axes[1, :]:
        ax.set_xlabel("Time [s]")
    for ax in axes[:, 0]:
        ax.set_ylabel("Speed [rpm]")

    fig.suptitle("Motor speeds")
    fig.tight_layout()
    return fig


def plotAttitude(timeHistory, attitudeHistory, desiredAttitudeHistory=None):
    """Window with 3 graphs: roll, pitch and yaw vs time, in degrees.

    Roll and pitch are relative to the horizon, yaw is the heading relative to world x (north).

    Args:
        timeHistory: shape (N,) [s]
        attitudeHistory: shape (N, 3), [roll, pitch, yaw] [rad]
        desiredAttitudeHistory: optional shape (N, 3), commanded [roll, pitch, yaw] [rad], drawn dashed
    """
    angles = np.degrees(attitudeHistory)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    fig.canvas.manager.set_window_title("Attitude")

    for i, (ax, name) in enumerate(zip(axes, ["Roll", "Pitch", "Yaw"])):
        ax.plot(timeHistory, angles[:, i], color="royalblue", linewidth=1.2, label="Actual")
        if desiredAttitudeHistory is not None:
            ax.plot(timeHistory, np.degrees(desiredAttitudeHistory)[:, i],
                    color="crimson", linestyle="--", linewidth=1, label="Desired")
            ax.legend(loc="upper right")
        ax.set_ylabel(f"{name} [deg]")
        ax.grid(True)

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Attitude (roll and pitch relative to horizon, yaw relative to north)")
    fig.tight_layout()
    return fig