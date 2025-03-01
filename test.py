import numpy as np

import numpy as np

def los_guidance(x_pos, y_pos, psi, waypoints, current_wp_idx, los_lookahead, thresh_next_wp):
    # Special handling: if we are at the last waypoint, drive directly toward it.
    if current_wp_idx >= len(waypoints) - 1:
        wp_final = waypoints[-1]
        psi_d = np.arctan2(wp_final[1] - y_pos, wp_final[0] - x_pos)
        cross_track_error = np.hypot(wp_final[0] - x_pos, wp_final[1] - y_pos)
        return psi_d, current_wp_idx, cross_track_error

    x, y = x_pos, y_pos

    wp_curr = waypoints[current_wp_idx]
    wp_next = waypoints[current_wp_idx + 1]
    
    dx = wp_next[0] - wp_curr[0]
    dy = wp_next[1] - wp_curr[1]
    path_length = np.hypot(dx, dy)

    if path_length < 1e-6:
        # If path length is zero, return the current heading.
        return psi, current_wp_idx, 0.0

    t = ((x - wp_curr[0]) * dx + (y - wp_curr[1]) * dy) / path_length**2
    t = np.clip(t, 0, 1)

    closest_x = wp_curr[0] + t * dx
    closest_y = wp_curr[1] + t * dy

    cross_track_error = np.hypot(x - closest_x, y - closest_y)

    lookahead_x = closest_x + los_lookahead * dx / path_length
    lookahead_y = closest_y + los_lookahead * dy / path_length

    psi_d = np.arctan2(lookahead_y - y, lookahead_x - x)

    distance_to_wp_next = np.hypot(x - wp_next[0], y - wp_next[1])
    print(f"DEBUG: current_wp_idx = {current_wp_idx}, distance_to_wp_next = {distance_to_wp_next:.3f}, t = {t:.3f}, cross_track_error = {cross_track_error:.3f}")

    # Update waypoint index if close enough or almost at the end of the segment.
    if distance_to_wp_next < thresh_next_wp or t > 0.99:
        print("DEBUG: Reached waypoint", current_wp_idx, "-> incrementing")
        current_wp_idx += 1

    return psi_d, current_wp_idx, cross_track_error


def test_los_guidance():
    """
    This function simulates a vessel moving along a series of waypoints and prints
    debugging information at each step to help determine why the algorithm might be
    getting stuck at the second-to-last waypoint.
    """
    # Define a sample set of waypoints.
    waypoints = np.array([
        [0, 0],
        [10, 0],
        [20, 5],
        [30, 10]  # Last waypoint
    ])
    los_lookahead = 5.0
    thresh_next_wp = 2.0

    # Initial settings:
    current_wp_idx = 0
    psi = 0.0  # initial heading
    # Define a series of vessel positions that gradually move along the path.
    positions = [
        (0, 0),
        (5, 0),
        (9, 0),
        (10, 0),   # Near first waypoint's end
        (12, 1),
        (15, 2),
        (18, 4),
        (20, 5),   # At second-to-last waypoint
        (25, 7.5),
        (29, 9.5),
        (30, 10),  # At last waypoint
        (32, 11)   # Beyond last waypoint
    ]
    
    for pos in positions:
        x_pos, y_pos = pos
        psi, current_wp_idx, cte = los_guidance(x_pos, y_pos, psi, waypoints, current_wp_idx, los_lookahead, thresh_next_wp)
        print(f"Position: ({x_pos:.1f}, {y_pos:.1f}) => psi_d: {psi:.3f}, current_wp_idx: {current_wp_idx}, cross_track_error: {cte:.3f}")
        # If we have passed all waypoints, break out.
        if current_wp_idx >= len(waypoints):
            print("DEBUG: Final waypoint reached or exceeded.")
            break

if __name__ == "__main__":
    test_los_guidance()
