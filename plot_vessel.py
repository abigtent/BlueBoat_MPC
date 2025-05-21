import numpy as np
import matplotlib.pyplot as plt

def plot_vessel_trajectory(simX, waypoints, thresh,
                           filename='vessel_trajectory_complete_turning_test.svg'):
    north, east, headings = simX[:,0], simX[:,1], simX[:,2]
    wp_north, wp_east     = waypoints[:,0], waypoints[:,1]

    # Trim trajectory once within thresh of final waypoint
    final_e, final_n = wp_east[-1], wp_north[-1]
    dist = np.hypot(east - final_e, north - final_n)
    if np.any(dist < thresh):
        idx = np.argmax(dist < thresh)
        east, north, headings = east[:idx+1], north[:idx+1], headings[:idx+1]

    fig, ax = plt.subplots(figsize=(10, 8))
    # Vessel path
    ax.plot(east, north, '-', lw=2, color='#bd7ebe',
            label='Vessel trajectory')

    # Path connecting the waypoints
    ax.plot(wp_east, wp_north, '--', lw=1.5, color='k',
            label='Waypoint path')

    # Waypoint markers (black fill)
    ax.scatter(wp_east, wp_north,
               facecolor='k', edgecolor='k',
               s=80, lw=1.5, label='Waypoints')

    # Start and end markers
    ax.scatter(east[0], north[0], c='#2ca02c', s=100, label='Start')
    ax.scatter(east[-1], north[-1], c='#d62728', s=100, label='End')

    # Final‐heading arrow
    dx, dy = np.sin(headings[-1]), np.cos(headings[-1])
    step = np.hypot(east[-1]-east[-2], north[-1]-north[-2])
    arrow_len = min(step, 1.0)
    ax.arrow(east[-1], north[-1],
             dx*arrow_len, dy*arrow_len,
             head_width=arrow_len*0.3,
             head_length=arrow_len*0.3,
             fc='#4e79a7', ec='#4e79a7')

    ax.set_aspect('equal', 'box')
    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.set_title('Vessel Trajectory')
    ax.legend(loc='best')
    ax.grid(True)
    fig.tight_layout()

    plt.savefig(filename, format='svg',
                transparent=True, bbox_inches='tight')
    plt.show()
