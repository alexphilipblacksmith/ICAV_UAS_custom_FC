from quadrotor import Quadrotor
from controllers.altitude import altitudeController
from controllers.position import positionController
from controllers.rate_pid import ratePIDController
from controllers.attitude import attitudeController
import numpy as np
from scipy.spatial.transform import Rotation
from visualisation import path_plotter3D

## Create drone object ##
drone = Quadrotor()

## Create flight control objects ##
alt_ctrl = altitudeController(Kp = 5.0, Ki = 0.1, Kd = 0.05)
rate_ctrl = ratePIDController(Kp = [10.0, 10.0, 10.0], Ki = [0.1, 0.1, 0.1], Kd = [0.05, 0.05, 0.05]) # Array convention: [yaw, pitch, roll]
att_ctrl = attitudeController(Kp = [1.0, 1.0, 1.0], Ki = [0.0, 0.0, 0.0], Kd = [0.0, 0.0, 0.0]) # Array convention: [yaw, pitch, roll]
pos_ctrl = positionController(Kp = [1.0, 1.0], Ki = [0.1, 0.1], Kd = [0.05, 0.05]) # Array convention: [x, y, z]

## Simulation parameters ##
dt = 0.01 # Time step [s]
simDuration = 20 # Simulation duration [s]
simSteps = int(simDuration / dt) # Number of simulation steps
step = 0 #Step counter of the simulation
time = 0 #Elapsed time of simulation (s)

target = [-5, 20, -10] # Target position vector [x, y, z]

desired_yaw = 0 # Desired yaw angle [rad]

# State history
position_history = []
velocity_history = []

#Create graph plotter object
trajectory_3D = path_plotter3D(target, update_frequency=5)

## Simulation Loop ##
for step in range (simSteps):
    
    time += dt
    
    #Get state of the drone
    state_vector = drone.get_state()
    current_pos = state_vector[0:3] # Current position vector [x, y, z]
    current_vel = state_vector[3:6] # Current velocity vector [u, v, w]
    current_orientation = state_vector[6:10] # Current orientation quaternion [w, x, y, z]
    current_ang_vel = state_vector[10:13] # Current angular velocity vector [p, q, r]

    # Store position and velocity history
    position_history.append(current_pos)
    velocity_history.append(current_vel)

    ## Cascaded PID loops: ##
    # alt_ctrl gives desired thrust from altitude error
    # pos_ctrl gives desired angles (roll and pitch) from position error - yaw must be controlled separately
    # att_ctrl gives desired angular rates from desired angles
    # rate_ctrl gives desired torques from desired angular rates
    # Motor mixer then produces desired motor speeds from desired thrust and torques
    desired_thrust = alt_ctrl.update(targetAltitude=-target[2], measuredAltitude=-current_pos[2], dt=dt) # Desired thrust from altitude controller
    desired_thrust += (drone.mass * drone.g)
    desired_angles = pos_ctrl.update(targetXY=target[0:2], measuredXY=current_pos[0:2], dt=dt) # Desired angles from position controller
    desired_angles = [(-desired_angles[0]/drone.g), (desired_angles[1]/drone.g)]
    desired_angles = np.insert(desired_angles, 0, desired_yaw) # Add the desired yaw to the start to maintain [yaw, pitch, roll] convention

    #Convert Euler angles to target quaternion
    desired_quat = Rotation.from_euler('zyx', desired_angles, degrees=False).as_quat()
    desired_quat = np.roll(desired_quat, 1) # Converts from [x,y,z,w] to [w,x,y,z]
    desired_quat_norm = np.linalg.norm(desired_quat)

    # Normalise quaternion
    if desired_quat_norm >0 and np.isfinite(desired_quat_norm):
        desired_quat /= desired_quat_norm
    

    desired_ang_vel = att_ctrl.update(targetQuat = desired_quat, measuredQuat = current_orientation, dt=dt)
    desired_torques = rate_ctrl.update(targetRate = desired_ang_vel, measuredRate = current_ang_vel, dt=dt)

    desired_motor_speeds = drone.mixer(desired_thrust, desired_torques)

    drone.step(desired_motor_speeds, dt)

    trajectory_3D.update(position_history)


trajectory_3D.finish()
    
