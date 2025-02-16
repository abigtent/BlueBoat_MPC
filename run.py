import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from nmpc import NMPC  
from animate import animate_trajectory

# Test parameters
target_pos = [10, 10]  # Target position (x, y)
obs = [
    {'x': 4, 'y': 5, 'radius': 1.0},
    {'x': 7, 'y': 3, 'radius': 1.5},
    {'x': 6, 'y': 7, 'radius': 1.2}]         
dyn_obs = []           # Dynamic obstacles (empty for simplicity)
col_buff = 2           # Collision buffer distance
N_steps = 100          # Prediction horizon (number of time steps)
dt = 0.5               # Time step (s)

controller = NMPC(target_pos, obs, dyn_obs, col_buff, N_steps, dt)

try:
    sol = controller.opti.solve()

    # Retrieve the optimized trajectories
    x_opt = sol.value(controller.x)
    y_opt = sol.value(controller.y)
    psi_opt = sol.value(controller.psi)
    u_opt = sol.value(controller.u)
    v_opt = sol.value(controller.v)
    r_opt = sol.value(controller.r)
    left_thruster_opt = sol.value(controller.left_thruster)
    right_thruster_opt = sol.value(controller.right_thruster)

    # Plot the resulting trajectory
    #plt.figure(figsize=(10, 6))
    #plt.plot(x_opt, y_opt, '-o', label='Vessel Trajectory')
    #plt.scatter(target_pos[0], target_pos[1], color='red', label='Target Position')

 
    animate_trajectory(x_opt, y_opt, psi_opt, target_pos, obs, dt, left_thruster_opt, right_thruster_opt, u_opt, v_opt, r_opt)
   
except Exception as e:
    print(f"Solver failed: {e}")