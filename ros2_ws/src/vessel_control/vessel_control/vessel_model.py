from casadi import *
import numpy as np
import types

def usv_model():
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    
    model_name = 'vessel_model'

    # === Parameters ===
    params = types.SimpleNamespace(
        m = 20.0,   
        Iz = 8.5,
        X_u_dot = -30.0,
        Y_v_dot = -25.0,
        N_r_dot = -6.0,
        Xu = -40.0,
        Yv = -65.0,
        Nr = -50.0,
        Y_r = -0.15,
        N_v = -0.12,
        thruster_d = 0.3
    )
    
    # === State Variables ===
    x   = MX.sym('x')       # x position
    y   = MX.sym('y')       # y position
    psi = MX.sym('psi')     # yaw angle
    u   = MX.sym('u')       # surge velocity
    v   = MX.sym('v')       # sway velocity
    r   = MX.sym('r')       # yaw rate 
    chi = MX.sym('chi')     # course angle
    chi_s = MX.sym('chi_s') # sin(chi)
    chi_c = MX.sym('chi_c') # cos(chi)
    cross_error = MX.sym('cross_error') # cross-track error
    port_thruster = MX.sym('left_thruster') # left thruster
    stbd_thruster = MX.sym('right_thruster') # right thruster
    
    x_states = vertcat(x, y, psi, u, v, r, chi, chi_s, chi_c, cross_error, port_thruster, stbd_thruster)

    # Control inputs
    port_thruster_dot = MX.sym('port_thruster_dot') 
    stbd_thruster_dot = MX.sym('stbd_thruster_dot')
    U = vertcat(port_thruster_dot, stbd_thruster_dot)
    
   # === Parameters ===
    # Desired course angle and obstacle parameters (to be updated via LiDAR)
    alpha = MX.sym('alpha')   # desired course angle
    x_obs = MX.sym('x_obs')   # obstacle x-position
    y_obs = MX.sym('y_obs')   # obstacle y-position
    r_obs = MX.sym('R_obs')   # obstacle radius
    p = vertcat(alpha, x_obs, y_obs, r_obs)

    # ----------------------
    # Model Equations
    # ----------------------

    # Rotation matrix from body-fixed to inertial frame
    R = vertcat(
        horzcat(cos(psi), -sin(psi), MX(0)),
        horzcat(sin(psi),  cos(psi), MX(0)),
        horzcat(MX(0),     MX(0),    MX(1))
    )

    # Kinematics: position and yaw
    eta_dot = mtimes(R, vertcat(u, v, r))

    # Mass Matrix (M)
    M_mat = vertcat(
        horzcat(params.m - params.X_u_dot, MX(0), MX(0)),
        horzcat(MX(0), params.m - params.Y_v_dot, MX(0)),
        horzcat(MX(0), MX(0), params.Iz - params.N_r_dot)
    )

    # Nonlinear damping matrix
    N_mat = vertcat(
        horzcat(-params.Xu, -params.m*r, params.Y_v_dot*v),
        horzcat(params.m*r, -params.Yv, -params.X_u_dot*u),
        horzcat(-params.Y_v_dot*v, params.X_u_dot*u, -params.Nr)
    )

    # Thruster mapping
    tau = vertcat(
        port_thruster + stbd_thruster, 
        0, 
        -params.thruster_d * (stbd_thruster - port_thruster)
    )

    # Dynamics (nu_dot = acceleration)
    nu = vertcat(u, v, r)
    nu_dot = mtimes(inv(M_mat), tau - mtimes(N_mat, nu))

    # Course angle dynamics explicitly
    chi_dot = vertcat(r, r * chi_c, -r * chi_s)

    # Cross-track error dynamics
    cross_error_dot = -(u * np.cos(psi) - v * np.sin(psi)) * np.sin(alpha)  + (u * np.sin(psi) + v * np.cos(psi)) * np.cos(alpha)

    # Thruster dynamics
    thruster_dyn = vertcat(port_thruster_dot, stbd_thruster_dot)

    # Combine all derivatives explicitly
    f_expl = vertcat(
        eta_dot,   # eta_dot: position & heading
        nu_dot,          # velocities acceleration
        chi_dot,         # course angle dynamics
        cross_error_dot, # cross-track error dynamics
        thruster_dyn     # thruster state dynamics
    )

    # ----------------------
    r_v = 3.0
    constraint.expr = vertcat((x - x_obs)**2 + (y - y_obs)**2 - (r_obs + r_v)**2)

    # State derivative placeholders
    xdot = MX.sym('xdot', 12)

    # ----------------------
    # Assign to model structs
    # ----------------------
    
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x_states
    model.xdot = xdot
    model.u = U
    model.z = vertcat([])
    model.p = p
    model.params = params
    model.name = model_name
    
    # ----------------------
    # Model Bounds & Initial Conditions
    # ----------------------
    model.u_min = -1.5
    model.u_max = 1.5
    model.thrust_port_min = -100
    model.thrust_stbd_min = -100
    model.thrust_port_max = 100
    model.thrust_stbd_max = 100

    model.x0 = np.zeros(12)

    return model, constraint