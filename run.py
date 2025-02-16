import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from nmpc import NMPC  # Replace 'your_nmpc_file' with the filename of your NMPC class

# Test parameters
target_pos = [30, 30]  # Target position (x, y)
obs = []               # Static obstacles (empty for simplicity)
dyn_obs = []           # Dynamic obstacles (empty for simplicity)
col_buff = 2           # Collision buffer distance
N_steps = 20           # Prediction horizon (number of time steps)
dt = 0.5               # Time step (s)

# Instantiate the NMPC controller
controller = NMPC(target_pos, obs, dyn_obs, col_buff, N_steps, dt)


#Solve the optimization problem
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
    plt.figure(figsize=(10, 6))
    plt.plot(x_opt, y_opt, '-o', label='Vessel Trajectory')
    plt.scatter(target_pos[0], target_pos[1], color='red', label='Target Position')
    plt.title('NMPC Vessel Trajectory')
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Print the control inputs for each time step
    print("Left Thruster Commands:", left_thruster_opt)
    print("Right Thruster Commands:", right_thruster_opt)

except Exception as e:
    print(f"Solver failed: {e}")