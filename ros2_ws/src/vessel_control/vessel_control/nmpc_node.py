import os
import time
import numpy as np
import rclpy
from rclpy.node import Node

from .acados_settings import acados_settings


class NMPCNode(Node):
    def __init__(self):
        super().__init__('nmpc_node')

        # Parameters
        self.Tf = 4.0
        self.N = 100
        self.T = 10.0
        self.Nsim = int(self.T * self.N / self.Tf)

        # Load waypoints
        pkg_dir = os.path.dirname(os.path.realpath(__file__))
        waypoints_path = os.path.join(pkg_dir, 'waypoints.txt')
        self.waypoints = np.loadtxt(waypoints_path)
        self.current_wp_idx = 0

        # Set up ACADOS solver
        self.constraint, self.model, self.acados_solver = acados_settings(self.Tf, self.N)
        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]

        # Simulation state
        self.x0 = np.zeros(self.nx)
        self.target_state = np.zeros(self.nx)
        self.target_control = np.zeros(self.nu)
        self.acados_solver.set(0, "lbx", self.x0)
        self.acados_solver.set(0, "ubx", self.x0)

        # Timer loop
        self.i = 0
        self.timer = self.create_timer(self.Tf / self.N, self.control_loop)

    def control_loop(self):
        if self.i >= self.Nsim:
            self.get_logger().info("Simulation finished.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        # Set reference to next waypoint (loop through waypoints)
        wp = self.waypoints[self.current_wp_idx]
        self.target_state[0:2] = wp
        yref = np.concatenate((self.target_state, self.target_control))
        yref_N = self.target_state
        p_val = np.zeros(4)  # dummy param if unused

        for j in range(self.N):
            self.acados_solver.set(j, "yref", yref)
            self.acados_solver.set(j, "p", p_val)
        self.acados_solver.set(self.N, "yref", yref_N)

        # Apply initial constraint
        self.acados_solver.set(0, "lbx", self.x0)
        self.acados_solver.set(0, "ubx", self.x0)

        # Solve
        t0 = time.time()
        status = self.acados_solver.solve()
        elapsed = time.time() - t0

        if status != 0:
            self.get_logger().warn(f"ACADOS solver failed at step {self.i} with status {status}")
        else:
            x_sol = self.acados_solver.get(0, "x")
            u_sol = self.acados_solver.get(0, "u")
            self.get_logger().info(f"[{self.i}] x: {x_sol[:2]}, u: {u_sol}, t_comp: {elapsed:.4f}s")

            # Propagate state forward (simple Euler as placeholder)
            self.x0 = self.acados_solver.get(1, "x")

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = NMPCNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
