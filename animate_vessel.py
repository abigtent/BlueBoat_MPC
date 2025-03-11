import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from lidar_simulator import LidarSimulator
import matplotlib.patches as patches

def animate_vessel(simX, waypoints, lidar_simulator=None, interval=100):
    """
    Animate the vessel's position over time, draw waypoints, render lidar rays,
    and display obstacles.
    
    Parameters:
        simX (np.ndarray): Simulation data for states (Nsim x nx). 
                           The first two columns are x and y positions,
                           and the third column is the heading (psi).
        waypoints (np.ndarray): Waypoints as an array of shape (n_wp, 2).
        lidar_simulator (LidarSimulator, optional): Instance of the LidarSimulator.
        interval (int): Delay between frames in milliseconds.
    """
    
    fig, ax = plt.subplots()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Vessel Position and Lidar Animation")
    ax.grid(True)
    
    # Display obstacles as circles if lidar_simulator is provided.
    if lidar_simulator is not None and hasattr(lidar_simulator, 'obstacles'):
        for obs in lidar_simulator.obstacles:
            # Each obstacle is expected as a tuple (x, y, radius)
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='gray', alpha=0.5, fill=True)
            ax.add_patch(circle)
    
    # Determine plot limits from waypoints (with padding)
    padding = 1.0
    wp_x_min, wp_x_max = np.min(waypoints[:, 0]), np.max(waypoints[:, 0])
    wp_y_min, wp_y_max = np.min(waypoints[:, 1]), np.max(waypoints[:, 1])
    margin_x = padding * (wp_x_max - wp_x_min) if wp_x_max > wp_x_min else 1.0
    margin_y = padding * (wp_y_max - wp_y_min) if wp_y_max > wp_y_min else 1.0
    ax.set_xlim(wp_x_min - margin_x, wp_x_max + margin_x)
    ax.set_ylim(wp_y_min - margin_y, wp_y_max + margin_y)
    
    # Plot the waypoints and connecting path.
    ax.plot([waypoints[0, 0]], [waypoints[0, 1]], 'k--', label="Start")
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'k--', label="Path")
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'ko', label="Waypoints")
    
    # Create objects for the vessel marker and its trail.
    trail_line, = ax.plot([], [], 'b-', linewidth=2, label="Trail")
    vessel_marker, = ax.plot([], [], 'bo', markersize=8, label="Vessel")
    
    # Pre-create line objects for each lidar ray if a simulator is provided.
    lidar_lines = []
    if lidar_simulator is not None:
        for _ in range(lidar_simulator.num_rays):
            line, = ax.plot([], [], 'r-', linewidth=1)
            lidar_lines.append(line)
    
    ax.legend()

    def init():
        """Initialization function for the animation."""
        trail_line.set_data([], [])
        vessel_marker.set_data([], [])
        for line in lidar_lines:
            line.set_data([], [])
        return [trail_line, vessel_marker] + lidar_lines

    def update(frame):
        """Update function for each frame."""
        # Get current vessel position and heading.
        x = simX[frame, 0]
        y = simX[frame, 1]
        psi = simX[frame, 2] if simX.shape[1] > 2 else 0.0
        
        # Update vessel trail and marker.
        x_data = simX[:frame+1, 0]
        y_data = simX[:frame+1, 1]
        trail_line.set_data(x_data, y_data)
        vessel_marker.set_data([x], [y])
        
        # Update lidar rays using the simpler representation.
        if lidar_simulator is not None:
            distances = lidar_simulator.sense_obstacles(x, y, psi)
            for j, (dist, angle_offset) in enumerate(zip(distances, lidar_simulator.angles)):
                angle = psi + angle_offset
                end_x = x + dist * np.cos(angle)
                end_y = y + dist * np.sin(angle)
                lidar_lines[j].set_data([x, end_x], [y, end_y])
                
                # Set ray color based on the measured distance.
                if 15 < dist <= 20:
                    lidar_lines[j].set_color('g')
                elif 10 < dist <= 15:
                    lidar_lines[j].set_color('y')
                elif 5 < dist <= 10:
                    lidar_lines[j].set_color('orange')
                elif 0 <= dist <= 5:
                    lidar_lines[j].set_color('r')
                else:
                    lidar_lines[j].set_color('r')
                    
        return [trail_line, vessel_marker] + lidar_lines

    ani = animation.FuncAnimation(fig, update, frames=len(simX),
                                  init_func=init, blit=True, interval=interval)

    plt.show()





