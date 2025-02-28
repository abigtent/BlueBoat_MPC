from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from vessel_model import usv_model
import scipy.linalg
import numpy as np


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

    # define constraint
    #model_ac.con_h_expr = constraint.expr

    # set dimensions
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    ocp.solver_options.N_horizon = N
    ns = 2
    nsh = 2

    # set cost
    Q = np.diag([20.0, 20.0, 10.0, 10.0, 10.0, 10.0])
    
    R = np.eye(nu)
    R[0, 0] = 0.2
    R[1, 1] = 0.2

    Qe = np.diag([100.0, 100.0, 10.0, 10.0, 10.0, 10.0])


    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    unscale = N / Tf
    
    ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe / unscale

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx
 
    Vu = np.zeros((ny, nu))
    Vu[6, 0] = 1.0
    Vu[7, 1] = 1.0
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
    ocp.cost.yref = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0, 0])

    # Setting constraints and applying them to the corresponding state and input variables
    ocp.constraints.lbx = np.array([model.u_min])
    ocp.constraints.ubx = np.array([model.u_max])
    ocp.constraints.idxbx = np.array([3])
    ocp.constraints.lbu = np.array([model.thrust_port_min, model.thrust_stbd_min])
    ocp.constraints.ubu = np.array([model.thrust_port_max, model.thrust_stbd_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # ocp.constraints.lsbx=np.zero s([1])
    # ocp.constraints.usbx=np.zeros([1])
    # ocp.constraints.idxsbx=np.array([1])
    '''ocp.constraints.lh = np.array(
        [
            constraint.e_r_min,
        ]
    )
    ocp.constraints.uh = np.array(
        [
            constraint.e_r_max,
        ]
    )'''
    '''ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array([0, 2])'''

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

    return constraint, model, acados_solver