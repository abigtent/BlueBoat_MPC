import casadi as ca
import pickle
import matplotlib.pyplot as plt

# Constants
d = 0.3
m = 20
c_omega = 0.95 # rotational drag coefficient
c_d = 0.95 # longitudinal drag coefficient 
I = 8.5 # moment of intertia


# Define optimization variables
N_steps = 100  # Prediction horizon
dt = 0.5      # Time step
A_max = 0.5       # Max acceleration (m/s²)
V_max = 2.0       # Max velocity (m/s)

opti = ca.Opti()  # Create an optimization problem

# Constraints
#N_min, N_max = 0, 50
#E_min, E_max = 0, 50

n_min, n_max = 0, 50
e_min, e_max = 0, 50
v_min, v_max = 0, 3
omega_min, omega_max = 0, 10

# State variables
#N = opti.variable(N_steps+1)  # North position
#E = opti.variable(N_steps+1)  # East position
#V_N = opti.variable(N_steps+1)  # North velocity
#V_E = opti.variable(N_steps+1)  # East velocity

n = opti.variable(N_steps + 1) # North position
e = opti.variable(N_steps + 1) # East position
v = opti.variable(N_steps + 1) # Linear velocity of the vessels COM
heading_angle = opti.variable(N_steps + 1) # Heading angle of the vessel
omega = opti.variable(N_steps + 1) # Angular velocity of heading angle

# Control inputs (accelerations)
#A_N = opti.variable(N_steps)  # Acceleration in N
#A_E = opti.variable(N_steps)  # Acceleration in E

left_thruster = opti.variable(N_steps)  # Left thruster force
right_thruster = opti.variable(N_steps)  # Right thruster force

# Initial conditions
#N0, E0, V_N0, V_E0 = 0, 0, 0, 0  # Initial state

#opti.subject_to(N[0] == N0)
#opti.subject_to(E[0] == E0)
#opti.subject_to(V_N[0] == V_N0)
#opti.subject_to(V_E[0] == V_E0)

n0, e0, v0, heading_angle0, omega0 = 0, 0, 0, 90, 0  # Initial state

opti.subject_to(n[0] == n0)
opti.subject_to(e[0] == e0)
opti.subject_to(v[0] == v0)
opti.subject_to(heading_angle[0] == heading_angle0)
opti.subject_to(omega[0] == omega0)

# Target position
target_pos = [30, 30]

# Static obstacles
obstacles = [
    [10, 10, 2],
    [25, 25, 2],
    [5, 30, 2]
]

# Define Dynamic Obstacles (ships): [N_init, E_init, V_N, V_E]
dynamic_obstacles = [
    #[10, 15, 1, -0.75],  
    #[15, 25, 0.7, -0.2],
    #[30, 30, -0.2, 0] 
]

collision_buffer = 2.0  # Safe distance margin (meters)

# Store dynamic obstacle trajectories
dynamic_trajs = {i: ([], []) for i in range(len(dynamic_obstacles))}

# Dynamics constraints using Euler discretization
for k in range(N_steps):
    
    #opti.subject_to(N[k] >= N_min)
    #opti.subject_to(N[k] <= N_max)
    #opti.subject_to(E[k] >= E_min)
    #opti.subject_to(E[k] <= E_max)
    
    opti.subject_to(n[k] >= n_min)
    opti.subject_to(n[k] <= n_max)
    opti.subject_to(v[k] >= v_min)
    opti.subject_to(v[k] <= v_max)
    opti.subject_to(omega[k] >= omega_min)
    opti.subject_to(omega[k] <= omega_max)
    
    opti.subject_to(n[k+1] == n[k] + v[k] * ca.cos(heading_angle[k]) * dt)
    opti.subject_to(e[k+1] == e[k] + v[k] * ca.sin(heading_angle[k]) * dt)
    opti.subject_to(heading_angle[k+1] == heading_angle[k] + omega[k] * dt)
    opti.subject_to(omega[k+1] == omega[k] + d/I * (right_thruster[k] - left_thruster[k] - c_omega * omega[k]) * dt)
    opti.subject_to(v[k+1] == v[k] + 1/m * (left_thruster[k] + right_thruster[k] - c_d * v[k] ** 2) * dt)

    for obs in obstacles:
        N_obs, E_obs, r_obs = obs
        opti.subject_to((n[k] - N_obs)**2 + (e[k] - E_obs)**2 >= (r_obs + collision_buffer)**2)

    for i, ship in enumerate(dynamic_obstacles):
        N_ship0, E_ship0, V_N_ship, V_E_ship = ship
        
        N_ship_k = N_ship0 + V_N_ship * (k * dt)
        E_ship_k = E_ship0 + V_E_ship * (k * dt)
        
        # Store trajectory of dynamic obstacles
        dynamic_trajs[i][0].append(N_ship_k)
        dynamic_trajs[i][1].append(E_ship_k)
        
        opti.subject_to((n[k] - N_ship_k)**2 + (e[k] - E_ship_k)**2 >= (collision_buffer*3)**2)


# Objective function (minimize acceleration and tracking error)
#accel_cost = 1 * ca.sumsqr(A_N) + ca.sumsqr(A_E)
goal_cost = 10 * ((n[-1] - target_pos[0])**2 + (e[-1] - target_pos[1])**2)


obstacle_penalty = 0
for obs in obstacles:
    N_obs, E_obs, r_obs = obs
    for k in range(N_steps):
        obstacle_penalty += 10 / ((n[k] - N_obs)**2 + (e[k] - E_obs)**2 + 0.1)

# Penalty for dynamic obstacles (lower weight)
dynamic_obstacle_penalty = 0
for i, ship in enumerate(dynamic_obstacles):
    for k in range(N_steps):
        N_ship_k = N_ship0 + V_N_ship * (k * dt)
        E_ship_k = E_ship0 + V_E_ship * (k * dt)
        dynamic_obstacle_penalty += 10 / ((n[k] - N_ship_k)**2 + (e[k] - E_ship_k)**2 + 0.1)

opti.minimize(goal_cost + obstacle_penalty + dynamic_obstacle_penalty)

# Solver options
opti.solver("ipopt")

# Solve the problem
sol = opti.solve()

# Extract optimal trajectory
N_opt = sol.value(n)
E_opt = sol.value(e)

# Save trajectory data
with open("mpc_results.pkl", "wb") as f:
    pickle.dump({
        "N_opt": N_opt, 
        "E_opt": E_opt, 
        "target_pos": target_pos, 
        "obstacles": obstacles,
        "dynamic_obstacles": dynamic_trajs
        }, f)

print("MPC results saved to 'mpc_results.pkl'. Run 'render_mpc.py' to animate the simulation.")