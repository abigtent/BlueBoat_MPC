import matplotlib.pyplot as plt


def plotFnc(simX, simU, simError, t):
    """
    Plot simulation results.
    
    Parameters:
        simX (np.ndarray): Array of states (Nsim x nx).
        simU (np.ndarray): Array of control inputs (Nsim x nu).
        simError (np.ndarray): Array of error signals (Nsim x n_error).
        t (np.ndarray): Time vector.
    """
    # Create a figure with 3 subplots: States, Controls, and Errors.
    plt.figure(figsize=(12, 10))
    
    # --- Plot state trajectories ---
    # 1) Position (x)
    plt.subplot(2, 3, 1)
    plt.plot(t, simX[:, 0], '-b', label='X position [m]')
    plt.title('X position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend()
    plt.grid(True)

    # 2) Position (y)
    plt.subplot(2, 3, 2)
    plt.plot(t, simX[:, 1], '-g', label='Y position [m]')
    plt.title('Y position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.legend()
    plt.grid(True)

    # 2) Heading (ψ)
    plt.subplot(2, 3, 3)
    plt.plot(t, simX[:, 2], '-r', label='Yaw angle [rad]')
    plt.title('Heading')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw ψ [rad]')
    plt.legend()
    plt.grid(True)

    # 3) Velocities (u, v, r)
    plt.subplot(2, 3, 4)
    plt.plot(t, simX[:, 3], '-m', label='Surge velocity [m/s]')
    plt.plot(t, simX[:, 4], '-c', label='Sway velocity [m/s]')
    plt.plot(t, simX[:, 5], '-k', label='Yaw rate [rad/s]')
    plt.title('Vessel velocities')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid(True)
    
    # --- Plot control inputs ---
    plt.subplot(2, 3, 5)
    # Assuming control inputs: [port_thruster, stbd_thruster]
    plt.plot(t, simX[:, 10], label='Port thruster [N]')
    plt.plot(t, simX[:, 11], label='Stbd thruster [N]')
    plt.title('Thruster Inputs')
    plt.xlabel('Time [s]')
    plt.ylabel('Control input')
    plt.legend()
    plt.grid(True)
    
    # --- Plot error signals ---
    plt.subplot(2, 3, 6)
    # Here we assume simError has at least 2 columns: e.g. heading error and lateral error.
    plt.plot(t, simError[:, 0], label='Cross-track error [m]')
    plt.plot(t, simError[:, 1], label='Heading error [degrees]')
    plt.plot(t, simError[:, 2], label='Surge error [m/s]')
    plt.title('Error Signals')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()

    # Save the figure as SVG
    plt.savefig("vessel_simulation.svg",
            format="svg",
            transparent=True,
            bbox_inches="tight")

    plt.show()
