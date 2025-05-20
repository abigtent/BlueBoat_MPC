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
    plt.subplot(3, 1, 1)
    # Assuming the state vector is: [x, y, psi, u, v, r]
    plt.plot(t, simX[:, 0], label='x [m]')
    plt.plot(t, simX[:, 1], label='y [m]')
    plt.plot(t, simX[:, 2], label='psi [rad]')
    plt.plot(t, simX[:, 3], label='u [m/s]')
    plt.plot(t, simX[:, 4], label='v [m/s]')
    plt.plot(t, simX[:, 5], label='r [rad/s]')
    plt.title('State Trajectories')
    plt.xlabel('Time [s]')
    plt.ylabel('States')
    plt.legend()
    plt.grid(True)
    
    # --- Plot control inputs ---
    plt.subplot(3, 1, 2)
    # Assuming control inputs: [port_thruster, stbd_thruster]
    plt.plot(t, simX[:, 10], label='Port thruster [N]')
    plt.plot(t, simX[:, 11], label='Stbd thruster [N]')
    plt.title('Control Inputs')
    plt.xlabel('Time [s]')
    plt.ylabel('Control input')
    plt.legend()
    plt.grid(True)
    
    # --- Plot error signals ---
    plt.subplot(3, 1, 3)
    # Here we assume simError has at least 2 columns: e.g. heading error and lateral error.
    plt.plot(t, simError[:, 0], label='Cross-track error [m]')
    plt.plot(t, simError[:, 1], label='Heading error [degrees]')
    plt.title('Error Signals')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
