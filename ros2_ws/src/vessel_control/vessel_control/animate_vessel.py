import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from lidar_simulator import LidarSimulator
import matplotlib.patches as patches

def animate_vessel(simX, waypoints, lidar_simulator=None, interval=100):
    """
    Animate the vessel's position over time, draw waypoints, render lidar rays,
    and display obstacles with color changing upon detection.
    
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

    # Store obstacle patches for dynamic color change
    obstacle_patches = []
    if lidar_simulator is not None and hasattr(lidar_simulator, 'obstacles'):
        for obs in lidar_simulator.obstacles:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='gray', alpha=0.5, fill=True)
            obstacle_patches.append(circle)
            ax.add_patch(circle)

    # Determine plot limits from waypoints (with padding)
    padding = 1.0
    wp_x_min, wp_x_max = np.min(waypoints[:, 0]), np.max(waypoints[:, 0])
    wp_y_min, wp_y_max = np.min(waypoints[:, 1]), np.max(waypoints[:, 1])
    margin_x = padding * (wp_x_max - wp_x_min) if wp_x_max > wp_x_min else 1.0
    margin_y = padding * (wp_y_max - wp_y_min) if wp_y_max > wp_y_min else 1.0
    ax.set_xlim(wp_x_min - margin_x, wp_x_max + margin_x)
    ax.set_ylim(wp_y_min - margin_y, wp_y_max + margin_y)

    # Plot waypoints and connecting path
    ax.plot([waypoints[0, 0]], [waypoints[0, 1]], 'k--', label="Start")
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'k--', label="Path")
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'ko', label="Waypoints")

    # Vessel marker and trail
    trail_line, = ax.plot([], [], 'b-', linewidth=2, label="Trail")
    vessel_marker, = ax.plot([], [], 'bo', markersize=8, label="Vessel")

    # Lidar rays
    lidar_lines = []
    if lidar_simulator is not None:
        for _ in range(lidar_simulator.num_rays):
            line, = ax.plot([], [], 'r-', linewidth=1)
            lidar_lines.append(line)

    ax.legend()

    def init():
        trail_line.set_data([], [])
        vessel_marker.set_data([], [])
        for line in lidar_lines:
            line.set_data([], [])
        return [trail_line, vessel_marker] + lidar_lines + obstacle_patches

    def update(frame):
        x, y = simX[frame, 0], simX[frame, 1]
        psi = simX[frame, 2] if simX.shape[1] > 2 else 0.0

        # Update trail and vessel position
        trail_line.set_data(simX[:frame+1, 0], simX[:frame+1, 1])
        vessel_marker.set_data([x], [y])

        if lidar_simulator is not None:
            distances = lidar_simulator.sense_obstacles(x, y, psi)
            for j, (dist, angle_offset) in enumerate(zip(distances, lidar_simulator.angles)):
                angle = psi + angle_offset
                end_x, end_y = x + dist * np.cos(angle), y + dist * np.sin(angle)
                lidar_lines[j].set_data([x, end_x], [y, end_y])

                # Lidar ray color based on distance
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

            # Obstacle detection (3m detection range)
            detection_range = 3.0
            for obs_patch, obs_info in zip(obstacle_patches, lidar_simulator.obstacles):
                obs_x, obs_y, obs_radius = obs_info
                distance_to_obs_edge = np.hypot(x - obs_x, y - obs_y) - obs_radius
                if distance_to_obs_edge <= detection_range:
                    obs_patch.set_color('red')
                    obs_patch.set_alpha(0.8)
                else:
                    obs_patch.set_color('gray')
                    obs_patch.set_alpha(0.5)

        return [trail_line, vessel_marker] + lidar_lines + obstacle_patches

    ani = animation.FuncAnimation(fig, update, frames=len(simX),
                                  init_func=init, blit=True, interval=interval)
    ax.set_aspect('equal', adjustable='box')

    plt.show()





