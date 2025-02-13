import casadi as ca
import json 
import pickle

with open('parameters.json', 'r') as file:
    parameters = json.load(file)['vessel']

m = parameters['m']
Iz = parameters['Iz']
X_u_dot = parameters['X_u_dot']
Y_v_dot = parameters['Y_v_dot']
N_r_dot = parameters['N_r_dot']
Xu = parameters['Xu']
Yv = parameters['Yv']
Nr = parameters['Nr']
Y_r = parameters['Y_r']
N_v = parameters['N_v']
thruster_d = parameters['thruster_d']
rad_vessel = parameters['rad_vessel']


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


        