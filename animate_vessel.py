import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate_vessel(simX, target_position, interval=100):
    """
    Animate the vessel's position over time.

    Parameters:
        simX (np.ndarray): Simulation data for states (Nsim x nx).
                           The first two columns are assumed to be x and y positions.
        target_position (list or tuple): [x, y] coordinates for the target.
        interval (int): Delay between frames in milliseconds.
    """
    fig, ax = plt.subplots()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Vessel Position Animation")
    ax.grid(True)
    
    # Set plot limits based on simulation data with some margin.
    x_min, x_max = np.min(simX[:, 0]), np.max(simX[:, 0])
    y_min, y_max = np.min(simX[:, 1]), np.max(simX[:, 1])
    margin_x = 0.1 * (x_max - x_min) if x_max > x_min else 1.0
    margin_y = 0.1 * (y_max - y_min) if y_max > y_min else 1.0
    ax.set_xlim(x_min - margin_x, x_max + margin_x)
    ax.set_ylim(y_min - margin_y, y_max + margin_y)
    
    # Plot the target position as a red star.
    target_marker, = ax.plot(target_position[0], target_position[1],
                             'r*', markersize=15, label="Target")
    
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
        # Current vessel position.
        vessel_marker.set_data(simX[frame, 0], simX[frame, 1])
        return trail_line, vessel_marker

    # Create the animation.
    ani = animation.FuncAnimation(fig, update, frames=len(simX),
                                  init_func=init, blit=True, interval=interval)

    plt.show()
    # Optionally, to save the animation to a file, uncomment the line below:
    # ani.save("vessel_animation.mp4", writer="ffmpeg", fps=30)

if __name__ == "__main__":
    # Example usage with dummy simulation data:
    # Assume a simulation over 30 seconds with 300 steps (adjust as needed)
    Nsim = 300
    t = np.linspace(0, 30, Nsim)
    # Create a dummy trajectory: vessel moves linearly from (0,0) to (10,10)
    simX = np.zeros((Nsim, 6))
    simX[:, 0] = np.linspace(0, 10, Nsim)  # x position
    simX[:, 1] = np.linspace(0, 10, Nsim)  # y position
    # For the remaining state dimensions, you could fill with zeros or other dummy data.

    target_position = [10, 10]  # The desired target position.
    animate_vessel(simX, target_position, interval=50)
