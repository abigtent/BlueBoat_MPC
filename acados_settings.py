from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from vessel_model import usv_model
import scipy.linalg
import numpy as np
import casadi as ca


def acados_settings(Tf, N):
    # create render arguments
    ocp = AcadosOcp()

    # export model
    model, constraint = usv_model()

    # define acados ODE
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    
    ocp.model = model_ac

    # --- Obstacle Parameters ---
    r_v   = 3.0    # Vessel radius

    x_pos = model_ac.x[0]
    y_pos = model_ac.x[1]
    x_obs = model_ac.p[1]
    y_obs = model_ac.p[2]
    r_obs = model_ac.p[3]

    #model_ac.con_h_expr = (x_pos - x_obs)**2 + (y_pos - y_obs)**2 - (r_obs + r_v)**2 
    #model_ac.con_h_expr = constraint.expr
    ocp.parameter_values = np.array([0.0, 0.0, 0.0, 0.0])
   

    #slack_weight = np.array([50.0])
    #ocp.cost.Zl = slack_weight
    #ocp.cost.Zu = slack_weight
    #ocp.cost.zl = np.array([0.0])
    #ocp.cost.zu = np.array([0.0])
    
    #slack_bounds = np.array([5*np.pi/180])
    #ocp.constraints.lsbx = -slack_bounds # Slack variable lower bounds
    #ocp.constraints.usbx = slack_bounds # Slack variable upper bounds
    #ocp.constraints.idxsbx = np.array([0]) # Index of state variable the soft bounds apply to
    #ocp.constraints.lh = np.array([0.0])
    #ocp.constraints.uh = np.array([1e6])

    # Define slack bounds for the nonlinear constraint (dimension 1)
    #ocp.constraints.lsh = np.array([0.0])   # Lower slack bound (usually 0)
    #ocp.constraints.ush = np.array([0.1])   # Upper slack bound (set to 0 if you want slack to be penalized only, not free)
    #ocp.constraints.idxsh = np.array([0])   # Specify that the slack applies to the first (and only) nonlinear constraint

    # set dimensions
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    ocp.solver_options.N_horizon = N
    ns = 2
    nsh = 2

    Q = np.diag([
    0.0,  # x
    0.0,  # y
    4.0,  # psi
    20.0, # u
    0.0,  # v
    0.1,  # r
    0.0,  # chi (heading alignment)
    2.0,  # chi_s = sin(chi)
    2.0,  # chi_c = cos(chi)
    60.0, # cross-track error
    0.001, 0.001  # thruster effort
    ])

    
    R = np.eye(nu)
    R[0, 0] = 0.01 # Penalize change in port thruster
    R[1, 1] = 0.01 # Penalize change in stbd thruster

    Qe = np.diag([
    0.0,   # x (not directly tracked)
    0.0,   # y (not directly tracked)
    8.0,   # psi (not directly tracked)
    40.0,  # u (surge speed)
    0.0,   # v
    0.2,   # r (yaw rate)
    0.0,   # chi (you can keep this zero if chi_s and chi_c are included)
    4.0,   # chi_s = sin(chi)
    4.0,   # chi_c = cos(chi)
    80.0,   # y_e (cross-track error)
    0.002, # T_port (for smoothness)
    0.002  # T_stbd
    ])



    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    unscale = N / Tf
    
    ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe / unscale

    # map state into y
    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    # map input into the last rows of y
    Vu = np.zeros((ny, nu))
    for i in range(nu):
        Vu[nx + i, i] = 1.0
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    # Weight of the slack variables
    #ocp.cost.zl = 0 * np.ones((ns,)) #previously 100
    #ocp.cost.Zl = 0 * np.ones((ns,))
    #ocp.cost.zu = 0 * np.ones((ns,)) #previously 100
    #ocp.cost.Zu = 0 * np.ones((ns,))

    # set intial references
    ocp.cost.yref = np.zeros((ny))
    ocp.cost.yref_e = np.zeros((nx))

    # Setting constraints and applying them to the corresponding state and input variables
    ocp.constraints.lbx = np.array([model.u_min, model.r_min, model.thrust_port_min, model.thrust_stbd_min])
    ocp.constraints.ubx = np.array([model.u_max, model.r_max, model.thrust_port_max, model.thrust_stbd_max])
    ocp.constraints.idxbx = np.array([3, 5, 10, 11])

    # set intial condition
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    #ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    #ocp.solver_options.nlp_solver_type = "SQP_RTI"
    #ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    #ocp.solver_options.sim_method_num_stages = 4
    #ocp.solver_options.sim_method_num_steps = 3

    #ocp.solver_options.nlp_solver_max_iter = 200
    #ocp.solver_options.tol = 1e-4

    #ocp.solver_options.qp_solver_tol_stat = 1e-2
    #ocp.solver_options.qp_solver_tol_eq = 1e-2
    #ocp.solver_options.qp_solver_tol_ineq = 1e-2
    #ocp.solver_options.qp_solver_tol_comp = 1e-2


    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    parameter_values = ocp.parameter_values


    return constraint, model, acados_solver, parameter_values