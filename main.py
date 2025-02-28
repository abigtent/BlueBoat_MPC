import time, os
import numpy as np
from acados_settings import *
from plotting import plotFnc as plot
from animate_vessel import animate_vessel

# Prediction horizon, discretization, simulation duration
Tf = 10.0   # prediction horizon [s]
N = 100     # number of discretization steps
T = 30.0    # maximum simulation time [s]

# load acados model and solver
constraint, model, acados_solver = acados_settings(Tf, N)

# dimensions from the vessel model: 6 states, 2 controls
nx = model.x.size()[0]   # should be 6
nu = model.u.size()[0]   # should be 2
ny = nx + nu             # stage cost dimension: 8
ny_e = nx                # terminal cost dimension: 6
Nsim = int(T * N / Tf)

# initialize storage for simulation data
simX = np.ndarray((Nsim, nx))
simU = np.ndarray((Nsim, nu))
simError = np.ndarray((Nsim, 3))  # for example: [heading_error, lateral_error, (optional third error)]

tcomp_sum = 0
tcomp_max = 0

# Define target for the vessel:
# For example, we want to reach x=10, y=10, with zero heading error and zero velocities.
target_state = np.array([10.0, 5.0, 0.0, 0.0, 0.0, 0.0])    # [x, y, ψ, u, v, r]
target_control = np.array([0.0, 0.0])                           # desired control inputs
# Stage cost reference: (state, control) → 8-dimensional
yref = np.concatenate((target_state, target_control))
# Terminal stage reference: only the state matters (6-dimensional)
yref_N = target_state

# Set the initial condition for the solver.
# Here we choose a starting state different from the target.
x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Update the initial state constraint at stage 0 with the full state.
acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)

# Simulation loop
for i in range(Nsim):
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

# Animate the vessel trajectory
target_position = [target_state[0], target_state[1]]
animate_vessel(simX, target_position, interval=50)

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

