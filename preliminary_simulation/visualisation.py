from collections import deque
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


@dataclass(eq=False)
class LiveDashboard:
    """Real-time window for manual mode: 3D view of the drone, plus scrolling graphs of the
    motor speeds and the attitude over the last `window` seconds.

    Args:
        drone: Quadrotor, used for the arm geometry of the 3D model, omegaMax and the arena size
        window: time shown by the scrolling graphs [s]
        dt: physics time step [s], sets how many samples the buffers keep
    """

    # Settings (constructor arguments)
    drone: object
    window: float = 5.0
    dt: float = 0.01

    # Created in __post_init__ / changed while running (not constructor arguments)
    fig: plt.Figure = field(init=False, repr=False)
    ax3D: plt.Axes = field(init=False, repr=False)
    buffers: dict = field(init=False, repr=False)
    background: object = field(default=None, init=False, repr=False)
    animatedArtists: list = field(default_factory=list, init=False, repr=False)

    def __post_init__(self):
        """Opens the window and creates every line once; draw() only updates their data."""
        bufferLength = int(round(self.window / self.dt)) + 1
        self.buffers = {name: deque(maxlen=bufferLength)
                        for name in ["time", "position", "motorSpeeds", "attitude", "desiredAttitude"]}

        plt.ion() # Turn on interactive mode
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title("Manual flight")
        grid = self.fig.add_gridspec(1, 3, width_ratios=[1.4, 1, 1])

        # 3D view: trail of the last `window` seconds, the drone's arms and their shadow on the ground
        self.ax3D = self.fig.add_subplot(grid[0], projection="3d")
        self.ax3D.set_xlabel("X (north) [m]")
        self.ax3D.set_ylabel("Y (east) [m]")
        self.ax3D.set_zlabel("Altitude [m]")
        self.ax3D.set_title("Drone")
        self.trail = self.addLine(self.ax3D, color="royalblue", linewidth=1.5, label=f"Last {self.window:g} s")
        self.frontArm = self.addLine(self.ax3D, color="crimson", linewidth=3, label="Front")
        self.otherArms = self.addLine(self.ax3D, color="black", linewidth=3)
        self.shadow = self.addLine(self.ax3D, color="grey", linewidth=2, alpha=0.5)
        self.ax3D.legend(loc="upper left")
        self.setArenaLimits()
        self.infoText = self.fig.text(0.02, 0.02, "", fontsize=11, family="monospace", animated=True)
        self.animatedArtists.append(self.infoText)

        # Motor speeds, one graph per motor
        motorGrid = grid[1].subgridspec(4, 1, hspace=0.5)
        self.motorLines = []
        for i in range(4):
            ax = self.fig.add_subplot(motorGrid[i])
            self.motorLines.append(self.addLine(ax, color="royalblue", linewidth=1.2))
            ax.axhline(self.drone.omegaMax * radPerSecToRpm, color="grey", linestyle="--", linewidth=1)
            ax.set_xlim(-self.window, 0)
            ax.set_ylim(0, self.drone.omegaMax * radPerSecToRpm * 1.05)
            ax.set_title(motorNames[i], fontsize=9)
            ax.set_ylabel("[rpm]")
            ax.grid(True)
        ax.set_xlabel("Time [s] (0 = now)")

        # Attitude: actual and desired roll, pitch, yaw
        attitudeGrid = grid[2].subgridspec(3, 1, hspace=0.4)
        self.attitudeLines, self.desiredLines = [], []
        for i, (name, limit) in enumerate(zip(["Roll", "Pitch", "Yaw"], [45, 45, 180])):
            ax = self.fig.add_subplot(attitudeGrid[i])
            self.attitudeLines.append(self.addLine(ax, color="royalblue", linewidth=1.2, label="Actual"))
            self.desiredLines.append(self.addLine(ax, color="crimson", linestyle="--", linewidth=1, label="Desired"))
            ax.set_xlim(-self.window, 0)
            ax.set_ylim(-limit, limit)
            ax.set_ylabel(f"{name} [deg]")
            ax.grid(True)
            if i == 0:
                ax.legend(loc="upper left", fontsize=8)
        ax.set_xlabel("Time [s] (0 = now)")

        self.fig.subplots_adjust(left=0.03, right=0.98, top=0.95, bottom=0.08, wspace=0.25)

        # Save a new background whenever the whole figure is redrawn (first show, resize, 3D box moved)
        self.fig.canvas.mpl_connect("draw_event", self.saveBackground)
        plt.show(block=False)
        self.fig.canvas.draw()

    def addLine(self, ax, **style):
        """Create an empty animated line (drawn by blitting, not part of the background)."""
        if hasattr(ax, "zaxis"):
            line, = ax.plot([], [], [], animated=True, **style)
        else:
            line, = ax.plot([], [], animated=True, **style)
        self.animatedArtists.append(line)
        return line

    def setArenaLimits(self):
        """Make the 3D box exactly the drone's arena, drawn to scale (1 m is the same length on every axis)."""
        length, width, height = self.drone.arena if self.drone.arena is not None else (20.0, 20.0, 20.0)
        self.ax3D.set_xlim(-length / 2, length / 2)
        self.ax3D.set_ylim(-width / 2, width / 2)
        self.ax3D.set_zlim(0, height)
        self.ax3D.set_box_aspect((length, width, height))

    def saveBackground(self, event=None):
        """Store everything except the animated artists as an image."""
        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)

    def isOpen(self):
        """False once the user has closed the window."""
        return plt.fignum_exists(self.fig.number)

    def record(self, time, position, motorSpeeds, attitude, desiredAttitude):
        """Store one physics step. Old samples drop out of the buffers automatically."""
        self.buffers["time"].append(time)
        self.buffers["position"].append(np.array(position))
        self.buffers["motorSpeeds"].append(np.array(motorSpeeds))
        self.buffers["attitude"].append(np.array(attitude))
        self.buffers["desiredAttitude"].append(np.array(desiredAttitude))

    def draw(self, rotationMatrix):
        """Redraw the window with the buffered data.

        Args:
            rotationMatrix: current body-to-world rotation, used to draw the drone's arms
        """
        if len(self.buffers["time"]) == 0:
            self.processEvents()
            return

        time = np.array(self.buffers["time"])
        timeAgo = time - time[-1] # 0 = now, negative = in the past
        position = np.array(self.buffers["position"])
        rpm = np.array(self.buffers["motorSpeeds"]) * radPerSecToRpm
        attitude = np.degrees(np.array(self.buffers["attitude"]))
        desiredAttitude = np.degrees(np.array(self.buffers["desiredAttitude"]))
        current = position[-1]
        altitude = -current[2]

        # 3D view, altitude = -z. The arms are drawn 4x longer than they are so the attitude is visible
        self.trail.set_data(position[:, 0], position[:, 1])
        self.trail.set_3d_properties(-position[:, 2])

        arms = current + 4 * self.drone.motorPositions @ rotationMatrix.T # rows: motors 1..4 in world frame
        gap = [np.nan] * 3
        self.frontArm.set_data([current[0], arms[0, 0]], [current[1], arms[0, 1]])
        self.frontArm.set_3d_properties([-current[2], -arms[0, 2]])
        others = np.array([current, arms[2], gap, arms[1], arms[3]])     # centre -> 3, gap (nan), 2 -> 4
        self.otherArms.set_data(others[:, 0], others[:, 1])
        self.otherArms.set_3d_properties(-others[:, 2])
        shadow = np.array([arms[0], arms[2], gap, arms[1], arms[3]])      # both arms projected on the ground
        self.shadow.set_data(shadow[:, 0], shadow[:, 1])
        self.shadow.set_3d_properties(np.zeros(len(shadow)))

        self.infoText.set_text(f"t = {time[-1]:6.1f} s   N {current[0]:6.1f} m   E {current[1]:6.1f} m   "
                               f"altitude {altitude:6.1f} m")

        for i, line in enumerate(self.motorLines):
            line.set_data(timeAgo, rpm[:, i])
        for i, (line, desiredLine) in enumerate(zip(self.attitudeLines, self.desiredLines)):
            line.set_data(timeAgo, attitude[:, i])
            desiredLine.set_data(timeAgo, desiredAttitude[:, i])

        # Blit: paste the saved background, draw only the animated artists on top, show the result
        if self.background is None:
            self.fig.canvas.draw()
        self.fig.canvas.restore_region(self.background)
        for artist in self.animatedArtists:
            self.fig.draw_artist(artist)
        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()

    def processEvents(self):
        """Let the window respond (move, resize, close) without redrawing the lines."""
        self.fig.canvas.flush_events()
