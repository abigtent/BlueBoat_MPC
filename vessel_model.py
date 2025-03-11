from casadi import *

def usv_model():
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()
    
    model_name = 'vessel_model'

    # Parameters
    m         = 20.0   
    Iz        = 8.5    
    X_u_dot   = -30.0
    Y_v_dot   = -25.0
    N_r_dot   = -6.0
    Xu        = -40.0
    Yv        = -65.0
    Nr        = -50.0
    Y_r       = -0.15
    N_v       = -0.12
    thruster_d = 0.3

    # Set up state variables: [x, y, psi, u, v, r]
    x   = MX.sym('x') # x position
    y   = MX.sym('y') # y position
    psi = MX.sym('psi') # yaw angle
    u   = MX.sym('u') # surge velocity
    v   = MX.sym('v') # sway velocity
    r   = MX.sym('r') # yaw rate 
    
    #x_states = vertcat(x, y, psi, u, v, r)
    
    chi = MX.sym('chi') # course angle
    chi_s = MX.sym('chi_s') #
    chi_c = MX.sym('chi_c') #
    cross_error = MX.sym('cross_error')
    x_states = vertcat(x, y, psi, u, v, r, chi, chi_s, chi_c, cross_error)

    # Control inputs
    port_thruster  = MX.sym('left_thruster')
    stbd_thruster = MX.sym('right_thruster')
    U = vertcat(port_thruster, stbd_thruster)

    # xdot
    xdot = MX.sym("xdot")
    ydot = MX.sym("ydot")
    psidot = MX.sym("psidot")
    udot = MX.sym("udot")
    vdot = MX.sym("vdot")
    rdot = MX.sym("rdot")
    
    #xdot = vertcat(xdot, ydot, psidot, udot, vdot, rdot)
    
    chidot = MX.sym("chidot")
    chidot_s = MX.sym("chidot_s")
    chidot_c = MX.sym("chidot_c")
    cross_error_dot = MX.sym("cross_error_dot")
    xdot = vertcat(xdot, ydot, psidot, udot, vdot, rdot, chidot, chidot_s, chidot_c, cross_error_dot)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    # Velocities vector: nu = [u, v, r]
    nu = vertcat(u, v, r)

    # Rotation matrix from body-fixed to inertial frame
    R = vertcat(
            horzcat(cos(psi), -sin(psi), MX(0)),
            horzcat(sin(psi),  cos(psi), MX(0)),
            horzcat(MX(0),    MX(0),   MX(1))
        )
    
    # Body-fixed velocities: eta_dot = R*nu
    eta_dot = mtimes(R, nu)

    # Mass matrix (M) and hydrodynamic derivatives
    M_mat = vertcat(
        horzcat(m - X_u_dot,    MX(0),          MX(0)),
        horzcat(MX(0),          m - Y_v_dot,    MX(0)),
        horzcat(MX(0),          MX(0),          Iz - N_r_dot)
    )

    # Nonlinear damping
    N_mat = vertcat(
        horzcat(-Xu,        -m*r,        Y_v_dot*v),
        horzcat(m*r,         -Yv,         -X_u_dot*u),
        horzcat(-Y_v_dot*v,   X_u_dot*u,   -Nr)
    )

    # Thruster input mapping: total force and moment
    tau = vertcat(port_thruster + stbd_thruster, 0, -thruster_d * (stbd_thruster - port_thruster))

    # Compute the body-fixed acceleration: nu_dot = M^{-1}*(tau - N*nu)
    nu_dot = mtimes(inv(M_mat), (tau - mtimes(N_mat, nu)))
    
    chi_dot = vertcat(r, r * cos(chi), -r * sin(chi))

  
    # Full state derivative
    f_expl = vertcat(eta_dot, nu_dot, chi_dot)
    
    # Model bounds
    model.u_min = 0
    model.u_max = 1.5
    
    # State bounds
    model.thrust_port_min = -50
    model.thrust_stbd_min = -50
    model.thrust_port_max = 50
    model.thrust_stbd_max = 50
    

    # Define initial conditions
    model.x0 = np.array([0, 0, 0, 0, 0, 0])
    
    # Set up the parameters struct
    params = types.SimpleNamespace()
    params.m= m
    params.Iz = Iz
    params.X_u_dot = X_u_dot
    params.Y_v_dot = Y_v_dot
    params.N_r_dot = N_r_dot
    params.Xu = Xu
    params.Yv = Yv
    params.Nr = Nr
    params.Y_r = Y_r
    params.N_v = N_v
    params.thruster_d = thruster_d
    
    # Set up the model struct
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x    = x_states
    model.xdot = xdot
    model.u    = U
    model.z    = z
    model.p    = p
    model.name = model_name
    model.params = params

    return model, constraint
