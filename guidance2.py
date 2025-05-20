import numpy as np

def normalize_angle(angle):
    """Wrap angle to [-π, π]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

def los_guidance(x, y, psi, waypoints, current_wp_idx, los_lookahead, thresh_next_wp):
    if len(waypoints) < 2 or current_wp_idx >= len(waypoints) - 1:
        return psi, current_wp_idx, 0.0, waypoints[-1]

    wp_curr = waypoints[current_wp_idx]
    wp_next = waypoints[current_wp_idx + 1]

    dx = wp_next[0] - wp_curr[0]
    dy = wp_next[1] - wp_curr[1]

    path_angle = np.arctan2(dy, dx)

    # Cross-track error (in rotated path frame)
    e = (y - wp_curr[1]) * np.cos(path_angle) - (x - wp_curr[0]) * np.sin(path_angle)

    # Line-of-sight guidance law
    psi_d = path_angle - np.arctan(e / los_lookahead)
    psi_d = normalize_angle(psi_d)

    # Check if vessel is close to the **next** waypoint (not just current segment start)
    dist_to_next = np.hypot(x - wp_next[0], y - wp_next[1])
    if dist_to_next < thresh_next_wp and current_wp_idx < len(waypoints) - 2:
        current_wp_idx += 1

    return psi_d, current_wp_idx, e, wp_next