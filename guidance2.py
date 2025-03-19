import numpy as np

def los_guidance(x_pos, y_pos, psi, waypoints, current_wp_idx, los_lookahead, thresh_next_wp):
        if current_wp_idx >= len(waypoints) - 1:
            return psi, current_wp_idx, 0.0, waypoints[-1]
        
        x, y = x_pos, y_pos


        wp_curr = waypoints[current_wp_idx]
        if current_wp_idx < len(waypoints) - 1:
            wp_next = waypoints[current_wp_idx + 1]
        else:
            wp_next = wp_curr  # Keep the last waypoint fixed
        
        dx = wp_next[0] - wp_curr[0]
        dy = wp_next[1] - wp_curr[1]
        path_length = np.hypot(dx, dy)

        if path_length < 1e-6:
            #return np.arctan2(dy, dx)  # Avoid division by zero
            return np.arctan2(dy, dx), current_wp_idx, 0.0, wp_next
         
        
        t = ((x - wp_curr[0]) * dx + (y - wp_curr[1]) * dy) / path_length**2
        t = np.clip(t, 0, 1)  # Limit t to [0, 1] for interpolation

        closest_x = wp_curr[0] + t * dx
        closest_y = wp_curr[1] + t * dy

        cross_track_error = np.hypot(x - closest_x, y - closest_y)

        lookahead_x = closest_x + los_lookahead * dx / path_length
        lookahead_y = closest_y + los_lookahead * dy / path_length

        # Compute desired heading
        psi_d = np.arctan2(lookahead_y - y, lookahead_x - x)

        if np.hypot(x - wp_curr[0], y - wp_curr[1]) < thresh_next_wp and current_wp_idx < len(waypoints) - 1:
            current_wp_idx += 1


        return psi_d, current_wp_idx, cross_track_error, wp_next
    
    
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