import casadi as ca
import pickle

# Vessel constants
m = 20.0      # Mass of the boat (kg)
Iz = 8.5     # Moment of inertia (kg.m^2)
X_u_dot = -30  # Added mass in surge
Y_v_dot = -25  # Added mass in sway
N_r_dot = -6  # Added moment of inertia in yaw
Xu = -40     # Linear damping in surge
Yv = -65     # Linear damping in sway
Nr = -50     # Linear damping in yaw
Y_r = -0.15  
N_v = -0.12

# Constants
thruster_d = 0.3
rad_vessel = 1.0
##-----------------##

# Mass matrix
M = ca.DM([
    [m - X_u_dot, 0, 0],
    [0, m - Y_v_dot, 0],
    [0, 0, Iz - N_r_dot]
])

def R_matrix(psi):
    return ca.DM([
        [ca.cos(psi), -ca.sin(psi), 0],
        [ca.sin(psi), ca.cos(psi), 0],
        [0, 0, 1]
    ])

def N(nu):
    u, v, r = nu
    return ca.DM([
        [-Xu, -m*r, Y_v_dot*v],
        [m*r, -Yv, -X_u_dot*u],
        [-Y_v_dot*v, X_u_dot*u, -Nr]
    ])

class NMPC:
    def __init__(self, target_pos, obs, dyn_obs, col_buff, N_steps, dt):
        self.target_pos = target_pos
        self.obs = obs
        self.dyn_obs = dyn_obs
        self.col_buff = col_buff
        self.N_steps = N_steps
        self.dt = dt


        