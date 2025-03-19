import time, os
import numpy as np
from acados_settings import *
from plotting import plotFnc as plot
from animate_vessel import animate_vessel
from guidance2 import los_guidance, ssa
from lidar_simulator import LidarSimulator

# Prediction horizon, discretization, simulation duration
Tf = 10.0   # prediction horizon [s]
N = 100     # number of discretization steps
T = 100.0    # maximum simulation time [s]
los_lookahead = 20.0  # Lookahead distance for LOS guidance
thresh_next_wp = 5.0  # Threshold to switch waypoints

# load acados model and solver
constraint, model, acados_solver = acados_settings(Tf, N)

# dimensions from the vessel model: 6 states, 2 controls
nx = model.x.size()[0]   
nu = model.u.size()[0]   
ny = nx + nu             
ny_e = nx                # terminal cost dimension: 6
Nsim = int(T * N / Tf)

# initialize storage for simulation data
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
simError = np.ndarray((Nsim, 3))  # for example: [heading_error, lateral_error, (optional third error)]

tcomp_sum = 0
tcomp_max = 0

# Load waypoints from file
waypoints = np.loadtxt('waypoints.txt') 
current_wp_idx = 0

# Define target for the vessel:
# For example, we want to reach x=10, y=10, with zero heading error and zero velocities.
target_state = np.array([waypoints[current_wp_idx, 0],
                         waypoints[current_wp_idx, 1],
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
target_control = np.array([0.0, 0.0])                        
# Stage cost reference: (state, control) → 8-dimensional
yref = np.concatenate((target_state, target_control))
# Terminal stage reference: only the state matters (6-dimensional)
yref_N = target_state

# Set the initial condition for the solver.
x0 = np.zeros(nx)
# Update the initial state constraint at stage 0 with the full state.
acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)
u_d = 1.5 # desired surge velocity

# Simulation loop
for i in range(Nsim):
    # Get current state from solver to check if the current waypoint is reached
    x0_sol = acados_solver.get(0, "x")
    current_position = x0_sol[:2]  # assume first two entries are x and y
    current_heading = x0_sol[2]

    #distance_to_wp = np.linalg.norm(np.array(current_position) - np.array(waypoints[current_wp_idx]))
    #print(f"Distance to WP {current_wp_idx}: {distance_to_wp}, Threshold: {thresh_next_wp}")
    
    # Compute desired heading using Line of Sight (LOS) guidance
    #chi_d, current_wp_idx, wp_next = los_guidance(current_position[0], current_position[1], waypoints, current_wp_idx, los_lookahead, thresh_next_wp)
    psi_d, current_wp_idx, cross_track_error, wp_next= los_guidance(current_position[0], current_position[1], x0_sol[2], waypoints, current_wp_idx, los_lookahead, thresh_next_wp)
    #alpha_d = pi_p
 
    #chi_d = ssa(chi_d)
    #print("Current WP: ", current_wp_idx)
    #print("Current Position: ", current_position)
    #print("Current Heading: ", current_heading)
    #print("Desired Heading: ", psi_d)
    #print("Cross Track Error: ", cross_track_error)
    #print("Next WP: ", wp_next)
    
    # Update target_state x and y based on the current waypoint.
    target_state[0] = waypoints[current_wp_idx, 0]
    target_state[1] = waypoints[current_wp_idx, 1]
    #target_state[0] = wp_next[0]
    #target_state[1] = wp_next[1]
    target_state[3] = u_d
    target_state[6] = psi_d
    target_state[7] = np.sin(psi_d)
    target_state[8] = np.cos(psi_d)
    target_state[9] = 0.0
    

    yref = np.concatenate((target_state, target_control))
    yref_N = target_state
    # Update the reference at every stage of the horizon
    for j in range(N):
        acados_solver.set(j, "yref", yref)
    acados_solver.set(N, "yref", yref_N)
    
    # Solve the OCP
    t0 = time.time()
    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))
    elapsed = time.time() - t0
    tcomp_sum += elapsed
    tcomp_max = max(tcomp_max, elapsed)

    # Get solution at the current stage
    x0_sol = acados_solver.get(0, "x")
    u0_sol = acados_solver.get(0, "u")
    
    # Save the state and control trajectories
    simX[i, :] = x0_sol.reshape((nx,))
    simU[i, :] = u0_sol.reshape((nu,))

    # Optionally, record some errors.
    # For example, we record the heading error (state index 2) and y-position error.
    psi_error = x0_sol[2]
    ye_error = x0_sol[1] - target_state[1]
    simError[i, 0] = psi_error
    simError[i, 1] = ye_error

    # Update the initial condition for the next OCP solve.
    # Get the state from the next stage (index 1) of the current solution.
    x0_next = acados_solver.get(1, "x")
    # Force the new initial state by updating the state bounds at stage 0.
    acados_solver.set(0, "lbx", x0_next)
    acados_solver.set(0, "ubx", x0_next)

# Define time vector for plotting
t = np.linspace(0.0, Nsim * Tf / N, Nsim)

obstacles = [(5.0, 5.0, 1.0), (8.0, 8.0, 1.0), (12.0, 12.0, 1.0)]

# Animate the vessel trajectory
target_position = [target_state[0], target_state[1]]
lidar_sim = LidarSimulator(obstacles, max_range=20, num_rays=64)
animate_vessel(simX, waypoints, lidar_sim, interval=50)

# Plot the results.
plot(simX, simU, simError, t)

# Print performance statistics
print("Average computation time: {:.4f} s".format(tcomp_sum / Nsim))
print("Maximum computation time: {:.4f} s".format(tcomp_max))
print("Simulation time: {:.4f} s".format(Tf * Nsim / N))

# Avoid plotting when running on Travis
#if os.environ.get("ACADOS_ON_TRAVIS") is None:
 #   import matplotlib.pyplot as plt
  #  plt.show()