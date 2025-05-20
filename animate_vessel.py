import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrow


def animate_vessel(simX, waypoints, interval):
    """
    Animate the vessel's position over time and draw waypoints and trail.
    
    Parameters:
        simX (np.ndarray): Vessel states (Nsim x nx). 
                           Columns 0 and 1 should be x and y positions.
        waypoints (np.ndarray): Array of shape (n_wp, 2) for waypoint coordinates.
        interval (int): Time between frames in ms.
    """
    
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title("Vessel Position Animation")
    ax.grid(True)

    vessel_arrow = [None] 
    # Plot limits
    # Plot limits with 100 m padding
    padding = 100.0  # meters

    east_min, east_max = np.min(waypoints[:, 1]), np.max(waypoints[:, 1])
    north_min, north_max = np.min(waypoints[:, 0]), np.max(waypoints[:, 0])

    ax.set_xlim(east_min - padding, east_max + padding)
    ax.set_ylim(north_min - padding, north_max + padding)


    # Plot waypoints
    ax.plot([waypoints[0, 1]], [waypoints[0, 0]], 'go', label="Start")
    ax.plot(waypoints[:, 1], waypoints[:, 0], 'k--', label="Path")
    ax.plot(waypoints[:, 1], waypoints[:, 0], 'ko', label="Waypoints")

    # Initialize trail and vessel marker
    trail_line, = ax.plot([], [], 'b-', linewidth=2, label="Trail")
    vessel_marker, = ax.plot([], [], 'bo', markersize=8, label="Vessel")

    ax.legend()

    def init():
        trail_line.set_data([], [])
        vessel_marker.set_data([], [])
        return trail_line, vessel_marker

    def update(frame):
        x = simX[frame, 1]  # North
        y = simX[frame, 0]  # East
        psi = simX[frame, 2]  # Heading in radians

        if frame > 0:
            trail_line.set_data(simX[1:frame+1, 1], simX[1:frame+1, 0])
        else:
            trail_line.set_data([], [])

        vessel_marker.set_data([x], [y])

        # Remove old arrow if it exists
        if vessel_arrow[0] is not None:
            vessel_arrow[0].remove()

        # Compute arrow direction in NED (East-North plot)
        arrow_length = 6.0  # length of the arrow in meters
        dx = arrow_length * np.sin(psi)
        dy = arrow_length * np.cos(psi)

        arrow = FancyArrow(
            x, y,
            dx, dy,
            width=1.0,             # thickness of the arrow shaft
            head_width=4.0,        # width of the arrow head
            head_length=4.0,       # length of the arrow head
            color='red',
            length_includes_head=True
        )
        ax.add_patch(arrow)
        vessel_arrow[0] = arrow

        return trail_line, vessel_marker, arrow

    ani = animation.FuncAnimation(
        fig, update, frames=len(simX),
        init_func=init, blit=False, interval=interval
    )

    ax.set_aspect('equal', adjustable='box')
    plt.show()





