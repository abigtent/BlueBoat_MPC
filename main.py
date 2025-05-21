import time
import numpy as np
import casadi as ca
from acados_settings import *
from plotting import plotFnc as plot
from plot_vessel import plot_vessel_trajectory
from animate_vessel import animate_vessel
from guidance3 import los_guidance
from lidar_simulator import LidarSimulator

# Prediction horizon, discretization, simulation duration
Tf = 4.0   # prediction horizon [s]
N = 100     # number of discretization steps
T = 150.0    # maximum simulation time [s]
los_lookahead = 20.0  # Lookahead distance for LOS guidance
thresh_next_wp = 5.0  # Threshold to switch waypoints

# load acados model and solver
constraint, model, acados_solver, parameter_values = acados_settings(Tf, N)


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

# Initialize computation time statistics
tcomp_sum = 0
tcomp_max = 0

# Load waypoints from file
waypoints = np.loadtxt('waypoints.txt') 
current_wp_idx = 0

# Define target for the vessel:
# For example, we want to reach x=10, y=10, with zero heading error and zero velocities.
target_state = np.array([waypoints[current_wp_idx, 0],
                         waypoints[current_wp_idx, 1],
                         np.pi, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
target_control = np.array([0.0, 0.0])                        
# Stage cost reference: (state, control) → 8-dimensional
yref = np.concatenate((target_state, target_control))
# Terminal stage reference: only the state matters (6-dimensional)
yref_N = target_state

# Set the initial condition for the solver.
x0 = np.zeros(nx)
x0[0] = 5.0  
x0[1] = 10.0    
x0[2] = np.deg2rad(-90)# psi (heading)

x0[3] = 0.2      # surge velocity (u)
x0[4] = 0.1      # sway velocity (v)
x0[5] = 0.05      # yaw rate (r)
x0[6] = x0[2]    # chi = psi
x0[7] = np.cos(x0[2])  # cos(chi)
x0[8] = np.sin(x0[2])  # sin(chi)
x0[9] = 10.0    # cross-track error
x0[10] = 0.0     # port thruster
x0[11] = 0.0     # starboard thruster
# Update the initial state constraint at stage 0 with the full state.
acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)
u_d = 1.5 # desired surge velocity


#lidar_sim = LidarSimulator(obstacles, max_range=20, num_rays=64, inflation_radius=1.0)

def rk4_integrate(model, x, u, dt, p):
    """
    Perform RK4 integration for the vessel model with parameter vector p.
    """
    f_expl = model.f_expl_expr
    f_func = ca.Function('f_expl', [model.x, model.u, model.p], [f_expl])

    k1 = np.array(f_func(x, u, p)).squeeze()
    k2 = np.array(f_func(x + 0.5 * dt * k1, u, p)).squeeze()
    k3 = np.array(f_func(x + 0.5 * dt * k2, u, p)).squeeze()
    k4 = np.array(f_func(x + dt * k3, u, p)).squeeze()

    x_next = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return x_next


def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi].
    """
    return np.rad2deg((angle + np.pi) % (2 * np.pi) - np.pi)


for i in range(Nsim):
    # Extract current position and heading from last integration
    current_position = x0[:2]
    current_psi = x0[2]

    # Run LOS guidance using simulated state
    psi_d, cross_track_error, alpha, current_wp_idx, dist_wp, finished = los_guidance(
        current_position[0],
        current_position[1],
        waypoints,
        current_wp_idx,
        los_lookahead,
        thresh_next_wp,
    )
    #heading_error = normalize_angle(chi_d - current_psi)

    # Update reference trajectory
    #target_state[0] = wp_next[0]
    #target_state[1] = wp_next[1]
    target_state[2] = psi_d
    target_state[3] = u_d
    #target_state[6] = alpha
    target_state[7] = np.sin(alpha)
    target_state[8] = np.cos(alpha)
    target_state[9] = 0.0

    yref = np.concatenate((target_state, target_control))


    yref_N = target_state
    parameter_values[0] = alpha
    for j in range(N):
        acados_solver.set(j, "yref", yref)
        acados_solver.set(j, "p", parameter_values)

    acados_solver.set(N, "yref", yref_N)
    acados_solver.set(N, "p", parameter_values)

    # Set initial condition for the solver
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)

    # Solve NMPC
    start = time.perf_counter()
    status = acados_solver.solve()
    elapsed = time.perf_counter() - start

    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

    if status != 0:
        print(f"acados returned status {status} in iteration {i}")
        continue

    # Get solution and apply control
    u0_sol = acados_solver.get(0, "u")

    # Integrate dynamics forward (RK4)
    dt = Tf / N
    x0_next = rk4_integrate(model, x0, u0_sol, dt, p=parameter_values)

    # Log data
    simX[i, :] = x0_next
    simU[i, :] = u0_sol
    simError[i, 0] = cross_track_error
    simError[i, 1] = normalize_angle(alpha - current_psi)
    simError[i, 2] = u_d - x0[3]  
   
    # Update state for next iteration
    x0 = x0_next


# Define time vector for plotting
t = np.linspace(0.0, Nsim * Tf / N, Nsim)



# Animate the vessel trajectory
target_position = [target_state[0], target_state[1]]
#lidar_sim = LidarSimulator(obstacles, max_range=20, num_rays=64)
plot_vessel_trajectory(simX, waypoints, thresh_next_wp)

# Plot the results.
plot(simX, simU, simError, t)

# Print performance statistics
print(f"Average solver time: {tcomp_sum/Nsim:.6f} s")
print(f"Max solver time:     {tcomp_max:.6f} s")
print("Simulation time: {:.4f} s".format(Tf * Nsim / N))



