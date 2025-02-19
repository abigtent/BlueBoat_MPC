import casadi as ca
import json 
import pickle

class NMPC:
    def __init__(self, target_pos, obs, dyn_obs, col_buff, N_steps, dt):
        with open('parameters.json', 'r') as file:
            parameters = json.load(file)['vessel']

        # Imported parameters
        self.m = parameters['m']
        self.Iz = parameters['Iz']
        self.X_u_dot = parameters['X_u_dot']
        self.Y_v_dot = parameters['Y_v_dot']
        self.N_r_dot = parameters['N_r_dot']
        self.Xu = parameters['Xu']
        self.Yv = parameters['Yv']
        self.Nr = parameters['Nr']
        self.Y_r = parameters['Y_r']
        self.N_v = parameters['N_v']
        self.thruster_d = parameters['thruster_d']
        self.rad_vessel = parameters['rad_vessel']
        
        # _________________________
        self.target_pos = target_pos
        self.obs = obs
        self.dyn_obs = dyn_obs
        self.col_buff = col_buff
        self.N_steps = N_steps
        self.dt = dt

        # Constraints
        self.x_min, self.x_max = 0, 50
        self.y_min, self.y_max = 0, 50
        self.u_min, self.u_max = -0.5, 0.5
        self.r_min, self.r_max = -0.5, 0.5

        # Initial states [x, y, psi, u, v, r]
        self.x0, self.y0, self.psi0, self.u0, self.v0, self.r0 = 0, 0, 0, 0, 0, 0

        # Create the optimization problem 
        self.opti = ca.Opti()
        self.variables()
        self.dynamics()
        self.define_objective()
        self.setup_solver()
        
    def variables(self):
        # State variables
        self.x = self.opti.variable(self.N_steps + 1)
        self.y = self.opti.variable(self.N_steps + 1)
        self.psi = self.opti.variable(self.N_steps + 1)
        self.u = self.opti.variable(self.N_steps + 1)
        self.v = self.opti.variable(self.N_steps + 1)
        self.r = self.opti.variable(self.N_steps + 1)

        # Control inputs
        self.left_thruster = self.opti.variable(self.N_steps)
        self.right_thruster = self.opti.variable(self.N_steps)

    def dynamics(self):
        # Initial conditions
        self.opti.subject_to(self.x[0] == self.x0)
        self.opti.subject_to(self.y[0] == self.y0)
        self.opti.subject_to(self.psi[0] == self.psi0)
        self.opti.subject_to(self.u[0] == self.u0)
        self.opti.subject_to(self.v[0] == self.v0)
        self.opti.subject_to(self.r[0] == self.r0)

        for k in range(self.N_steps):
            # State constraints
            self.opti.subject_to(self.x[k] >= self.x_min)
            self.opti.subject_to(self.x[k] <= self.x_max)
            self.opti.subject_to(self.y[k] >= self.y_min)
            self.opti.subject_to(self.y[k] <= self.y_max)
            self.opti.subject_to(self.u[k] >= self.u_min)
            self.opti.subject_to(self.u[k] <= self.u_max)
            self.opti.subject_to(self.r[k] >= self.r_min)
            self.opti.subject_to(self.r[k] <= self.r_max)

            # Euler integration
            nu_k = ca.vertcat(self.u[k], self.v[k], self.r[k])
            tau_k = ca.vertcat(self.left_thruster[k] + self.right_thruster[k], 0, - self.thruster_d * (self.right_thruster[k] - self.left_thruster[k]))
            eta_dot = self.R_matrix(self.psi[k]) @ nu_k
            nu_dot = ca.inv(self.M_matrix()) @ (tau_k - self.N(nu_k) @ nu_k)
            
            # Forward Euler integration
            self.opti.subject_to(self.x[k+1] == self.x[k] + eta_dot[0] * self.dt)
            self.opti.subject_to(self.y[k+1] == self.y[k] + eta_dot[1] * self.dt)
            self.opti.subject_to(self.psi[k+1] == self.psi[k] + eta_dot[2] * self.dt)
            self.opti.subject_to(self.u[k+1] == self.u[k] + nu_dot[0] * self.dt)
            self.opti.subject_to(self.v[k+1] == self.v[k] + nu_dot[1] * self.dt)
            self.opti.subject_to(self.r[k+1] == self.r[k] + nu_dot[2] * self.dt)
    
    def define_objective(self):
        # Define the objective function
        self.min_error = 100 * ca.sumsqr(self.x - self.target_pos[0]) + ca.sumsqr(self.y - self.target_pos[1])

        # Penalizing large changes in yaw
        for k in range(self.N_steps):
            yaw_change = self.psi[k+1] - self.psi[k]
            #self.min_error += 1 * ca.sumsqr(yaw_change)  # Penalty factor for yaw changes

        # Penalizing large changes in thruster inputs
        for k in range(1, self.N_steps):
            self.min_error += 5 * ca.sumsqr(self.left_thruster[k] - self.left_thruster[k-1])
            self.min_error += 5 * ca.sumsqr(self.right_thruster[k] - self.right_thruster[k-1])

        # Add obstacle avoidance penalty using a smooth hinge function
        for obstacle in self.obs:
            obs_x = obstacle['x']
            obs_y = obstacle['y']
            obs_r = obstacle['radius']
    
        for k in range(self.N_steps + 1):
        # Calculate distance to the obstacle
            dist = ca.sqrt((self.x[k] - obs_x)**2 + (self.y[k] - obs_y)**2)
            safe_distance = obs_r + self.col_buff
        
        # Use a smooth hinge: zero penalty when dist >= safe_distance, quadratic when too close.
        penalty = ca.fmax(0, safe_distance - dist)**2 / (safe_distance**2)
        print(safe_distance-dist)
        # Add penalty to the objective with a chosen weight (here, 2)
        self.min_error += 30 * penalty

        self.opti.minimize(self.min_error)

    
    def setup_solver(self):
        self.opti.solver("ipopt")

    def M_matrix(self):
        return ca.vertcat(
            ca.horzcat(self.m - self.X_u_dot, ca.MX(0), ca.MX(0)),
            ca.horzcat(ca.MX(0), self.m - self.Y_v_dot, ca.MX(0)),
            ca.horzcat(ca.MX(0), ca.MX(0), self.Iz - self.N_r_dot)
    )

    def R_matrix(self, psi):
        return ca.vertcat(
            ca.horzcat(ca.cos(psi), -ca.sin(psi), ca.MX(0)),
            ca.horzcat(ca.sin(psi), ca.cos(psi), ca.MX(0)),
            ca.horzcat(ca.MX(0), ca.MX(0), ca.MX(1))
        )

    def N(self, nu):
        u = nu[0]
        v = nu[1]
        r = nu[2]

        return ca.vertcat(
            ca.horzcat(-self.Xu, -self.m * r, self.Y_v_dot * v),
            ca.horzcat(self.m * r, -self.Yv, -self.X_u_dot * u),
            ca.horzcat(-self.Y_v_dot * v, self.X_u_dot * u, -self.Nr)
    )