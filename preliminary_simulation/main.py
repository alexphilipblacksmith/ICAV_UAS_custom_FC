from quadrotor import Quadrotor
import numpy as np

# Create drone object
drone = Quadrotor()

#Simulation parameters
dt = 0.01 # Time step [s]
simDuration = 10 # Simulation duration [s]
simSteps = int(simDuration / dt) # Number of simulation steps

target = [0, 0, 0] # Target position vector [x, y, z]

# State history
position_history = []
velocity_history = []
