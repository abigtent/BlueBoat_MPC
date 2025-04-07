import os
import time
import numpy as np
import rclpy
from rclpy.node import Node

from .acados_settings import acados_settings
from .guidance import los_guidance

from blueboat_interfaces.msg import BlueBoatState, BlueBoatActuatorInputs


class NMPCNode(Node):
    def __init__(self):
        super().__init__('nmpc_node')

        self.sub_state = self.create_subscription(
            BlueBoatState, # message type
            'BlueBoatState', # topic name
            self.state_callback,
            10
        )
        self.get_logger().info("BlueBoatState subscriber started.")
   
        self.pub_thruster = self.create_publisher(
            BlueBoatActuatorInputs, # message type
            'BlueBoatThrusterInputs', # topic name
            10
        )
        self.get_logger().info("BlueBoatThrusterInputs publisher started.")

        # NMPC parameters and acados solver initialization
        self.Tf = 4.0        # prediction horizon [s]
        self.N = 100         # number of discretization steps
        self.los_lookahead = 20.0  # Lookahead distance for LOS guidance
        self.thresh_next_wp = 10.0  # Threshold to switch waypoints

        # Load acados model and solver
        self.constraint, self.model, self.acados_solver = acados_settings(self.Tf, self.N)
        self.get_logger().info("Acados model and solver initialized.")

        # Get dimensions from the vessel model
        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]

        # Load waypoints from file 
        pkg_dir = os.path.dirname(os.path.realpath(__file__))
        waypoints_path = os.path.join(pkg_dir, 'waypoints.txt')
        self.waypoints = np.loadtxt(waypoints_path)
        self.current_wp_idx = 0
        self.get_logger().info(f"Waypoints loaded from {waypoints_path}.")

        # Define target state and control
        self.target_state = np.array([
            self.waypoints[self.current_wp_idx, 0],
            self.waypoints[self.current_wp_idx, 1],
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ])
        self.target_control = np.array([0.0, 0.0])
        self.yref = np.concatenate((self.target_state, self.target_control))
        self.yref_N = self.target_state

        # Desired surge velocity
        self.u_d = 1.5

        # Initialize solver with an initial state (zeros)
        self.x0 = np.zeros(self.nx)
        self.acados_solver.set(0, "lbx", self.x0)
        self.acados_solver.set(0, "ubx", self.x0)

        self.get_logger().info(f"NMPC node initialized with desired surge velocity {self.u_d} m/s.")

    def state_callback(self, msg: BlueBoatState):
        
        self.current_state = np.array([msg.x, msg.y, msg.psi, msg.u, msg.v, msg.r])

        # Compute the desired heading using LOS guidance.
        chi_d, alpha, self.current_wp_idx, cross_track_error = los_guidance(
            self.current__state[0],
            self.current_y_pos[1],
            self.waypoints,
            self.current_wp_idx,
            self.los_lookahead,
            self.thresh_next_wp
        )

        # Update target state based on the current waypoint and guidance results.
        self.target_state[0] = self.waypoints[self.current_wp_idx, 0]
        self.target_state[1] = self.waypoints[self.current_wp_idx, 1]
        self.target_state[3] = self.u_d
        self.target_state[6] = chi_d
        self.target_state[7] = np.cos(alpha)
        self.target_state[8] = np.sin(alpha)
        self.target_state[9] = 0.0

        # Update reference vectors for the solver.
        self.yref = np.concatenate((self.target_state, self.target_control))
        self.yref_N = self.target_state

        # Set the reference for every stage of the horizon.
        for j in range(self.N):
            self.acados_solver.set(j, "yref", self.yref)
        self.acados_solver.set(self.N, "yref", self.yref_N)

        # Set the current state as the initial condition.
        self.acados_solver.set(0, "lbx", self.current_state)
        self.acados_solver.set(0, "ubx", self.current_state)

        # Solve the NMPC problem.
        status = self.acados_solver.solve()
        if status != 0:
            self.get_logger().warn("acados solver failed with status {}".format(status))
            return

        # Retrieve the control solution.
        u0_sol = self.acados_solver.get(0, "u")

        # Prepare and publish the thruster command.
        thruster_msg = BlueBoatThrusterInputs()
        # Assuming the thruster message has an 'inputs' field that accepts a list of floats.
        thruster_msg.inputs = u0_sol.flatten().tolist()
        self.pub_thruster.publish(thruster_msg)

        # Optionally update the solver’s initial state for warm starting.
        x0_next = self.acados_solver.get(1, "x")
        self.acados_solver.set(0, "lbx", x0_next)
        self.acados_solver.set(0, "ubx", x0_next)

def main(args=None):
    rclpy.init(args=args)
    nmpc_node = NMPCNode()
    try:
        rclpy.spin(nmpc_node)
    except KeyboardInterrupt:
        pass
    nmpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
