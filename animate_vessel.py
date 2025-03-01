import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate_vessel(simX, waypoints=None, interval=100):
    """
    Animate the vessel's position over time and draw waypoints and a dotted connecting line.

    Parameters:
        simX (np.ndarray): Simulation data for states (Nsim x nx). 
                           The first two columns are assumed to be x and y positions.
        waypoints (np.ndarray, optional): Waypoints as an array of shape (n_wp, 2).
        interval (int): Delay between frames in milliseconds.
    """
    waypoints = np.loadtxt('waypoints.txt')

    fig, ax = plt.subplots()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Vessel Position Animation")
    ax.grid(True)
    
    # Set plot limits based on simulation data with some margin.
     # Set plot limits based on the waypoints coordinates plus extra padding.
    padding = 1.0
    wp_x_min, wp_x_max = np.min(waypoints[:, 0]), np.max(waypoints[:, 0])
    wp_y_min, wp_y_max = np.min(waypoints[:, 1]), np.max(waypoints[:, 1])
    margin_x = padding * (wp_x_max - wp_x_min) if wp_x_max > wp_x_min else 1.0
    margin_y = padding * (wp_y_max - wp_y_min) if wp_y_max > wp_y_min else 1.0
    ax.set_xlim(wp_x_min - margin_x, wp_x_max + margin_x)
    ax.set_ylim(wp_y_min - margin_y, wp_y_max + margin_y)
    
    # If waypoints are provided, plot them with markers and a dotted line connecting them.
    if waypoints is not None:
        ax.plot([0, waypoints[0, 0]], [0, waypoints[0, 1]], 'k--', label="Start to 1st WP")
        # Plot dotted line connecting waypoints.
        ax.plot(waypoints[:, 0], waypoints[:, 1], 'k--', label="Path")
        # Plot the waypoints themselves.
        ax.plot(waypoints[:, 0], waypoints[:, 1], 'ko', label="Waypoints")
    
    # Create objects for the vessel marker and its trail.
    trail_line, = ax.plot([], [], 'b-', linewidth=2, label="Trail")
    vessel_marker, = ax.plot([], [], 'bo', markersize=8, label="Vessel")
    
    ax.legend()

    def init():
        """Initialization function for the animation."""
        trail_line.set_data([], [])
        vessel_marker.set_data([], [])
        return trail_line, vessel_marker

    def update(frame):
        """Update function for each frame."""
        # Extract all x and y positions up to the current frame.
        x_data = simX[:frame+1, 0]
        y_data = simX[:frame+1, 1]
        trail_line.set_data(x_data, y_data)
        # Set current vessel position (as a sequence).
        vessel_marker.set_data([simX[frame, 0]], [simX[frame, 1]])
        return trail_line, vessel_marker

    # Create the animation.
    ani = animation.FuncAnimation(fig, update, frames=len(simX),
                                  init_func=init, blit=True, interval=interval)

    plt.show()
    # Optionally, to save the animation to a file, uncomment the line below:
    # ani.save("vessel_animation.mp4", writer="ffmpeg", fps=30)

if __name__ == "__main__":
    # Example usage with dummy simulation data:
    Nsim = 300
    t = np.linspace(0, 30, Nsim)
    # Create a dummy trajectory: vessel moves linearly from (0,0) to (10,10)
    simX = np.zeros((Nsim, 6))
    simX[:, 0] = np.linspace(0, 10, Nsim)  # x position
    simX[:, 1] = np.linspace(0, 10, Nsim)  # y position

    waypoints = np.loadtxt('waypoints.txt')
    animate_vessel(simX, waypoints=waypoints, interval=50)
