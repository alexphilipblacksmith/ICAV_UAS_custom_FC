import matplotlib.pyplot as plt
import numpy as np

class path_plotter3D:
    def __init__(self, target, update_frequency=5, limit=20):
        """
        Initializes the live 3D plot window.
        
        Args:
            target: [x, y, z] target location
            update_frequency: Redraws every N steps to keep simulation running smoothly.
            limit: Sets the lower and upper bounds for X, Y, and Z axes (-limit to limit).
        """
        plt.ion() # Turn on interactive mode
        
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.target = np.array(target)
        self.update_frequency = update_frequency
        self.step_count = 0
        self.limit = limit
        
        # Set fixed bounds from -20 to 20 for all axes
        self.set_axis_limits()

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
        self.ax.set_zlabel('Z Position [m]')
        self.ax.set_title('Live Quadrotor 3D Flight Trajectory')
        self.ax.legend(loc='upper left')
        self.ax.grid(True)
        
        # Create an empty line object for the drone's path
        self.line, = self.ax.plot([], [], [], label='Drone Trajectory', color='royalblue', linewidth=2)
        self.start_plotted = False

    def set_axis_limits(self):
        """Sets explicit fixed limits from -limit to limit for all axes."""
        self.ax.set_xlim(-self.limit, self.limit)
        self.ax.set_ylim(-self.limit, self.limit)
        self.ax.set_zlim(-self.limit, self.limit)

    def update(self, position_history):
        """Updates the line data and refreshes the canvas."""
        self.step_count += 1
        
        # Only update every N steps to prevent rendering lag
        if self.step_count % self.update_frequency != 0:
            return
            
        pos = np.array(position_history)
        if len(pos) == 0:
            return
            
        # Plot start point once on the first valid step
        if not self.start_plotted:
            self.ax.scatter(
                pos[0, 0], 
                pos[0, 1], 
                -pos[0, 2], 
                color='forestgreen', 
                s=100, 
                label='Start Point', 
                marker='o'
            )
            self.start_plotted = True

        # Update the 3D line coordinates
        self.line.set_data(pos[:, 0], pos[:, 1])
        self.line.set_3d_properties(-pos[:, 2])
        
        # Keep fixed limits
        self.set_axis_limits()
        
        # Redraw canvas and process GUI events
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def finish(self):
        """Keeps the final plot window open after the simulation ends."""
        plt.ioff()
        plt.show()