import numpy as np

def wp_selector(x_pos, y_pos, waypoints, current_wp_idx, thresh_next_wp):
    num_wp = waypoints.shape[0]

    # prevent index overflow explicitly
    if current_wp_idx >= num_wp - 1:
        current_wp_idx = num_wp - 2

    wp_curr = waypoints[current_wp_idx]
    wp_next = waypoints[current_wp_idx + 1]

    # Corrected: distance from vessel to next waypoint explicitly
    dist_to_next_wp = np.linalg.norm(np.array([x_pos, y_pos]) - wp_next)

    if dist_to_next_wp < thresh_next_wp and current_wp_idx < num_wp - 2:
        current_wp_idx += 1
        wp_curr = waypoints[current_wp_idx]
        wp_next = waypoints[current_wp_idx + 1]
    else:
        wp_curr = waypoints[current_wp_idx]
        wp_next = waypoints[min(current_wp_idx + 1, num_wp - 1)]

    return wp_next, wp_curr, current_wp_idx
        
def cross_track_error(x_pos, y_pos, wp_1, wp_2):
    
    # x_e : along track error
    # y_e : cross track error
    # pi_p : path angle (angle of the path with respect to the North axis)
    
    # Calculate the path angle
    pi_p = np.arctan2(wp_2[1] - wp_1[1], wp_2[0] - wp_1[0])
    
    # Calculate the along and cross track errors
    R_t = np.array([[np.cos(pi_p), np.sin(pi_p)], [-np.sin(pi_p), np.cos(pi_p)]])
    e = R_t @ np.array([x_pos - wp_1[0], y_pos - wp_1[1]])
    e_x = e[0] # along track error
    e_y = e[1] # cross track error

    return e_x, e_y, pi_p

def los_guidance(x_pos, y_pos, waypoints, current_wp_idx, los_lookahead, thresh_next_wp):
    
    wp_next, current_wp, current_wp_idx = wp_selector(x_pos, y_pos, waypoints, current_wp_idx, thresh_next_wp)
    
    _, e_y, pi_p = cross_track_error(x_pos, y_pos, current_wp, wp_next)
    
    Kp = 1/los_lookahead
    
    chi_d = pi_p - np.arctan(Kp * e_y)

    return chi_d, current_wp_idx, cross_track_error, wp_next


def ssa(angle, unit='rad'):
    """
    Smallest-Signed Angle (SSA):
    Maps an angle to the interval [-pi, pi) for radians or [-180, 180) for degrees.
    
    Parameters:
        angle (float): The angle to be wrapped.
        unit (str, optional): The unit of the angle. Use 'rad' for radians (default) or 'deg' for degrees.
    
    Returns:
        float: The wrapped angle.
    
    Examples:
        >>> ssa(4 * math.pi)
        0.0
        >>> ssa(190, 'deg')
        -170.0
    """
    if unit == 'rad':
        return ((angle + np.pi) % (2 * np.pi)) - np.pi
    elif unit == 'deg':
        return ((angle + 180) % 360) - 180
    else:
        raise ValueError(f"Invalid unit argument: '{unit}'. Unit must be either 'deg' or 'rad'.")