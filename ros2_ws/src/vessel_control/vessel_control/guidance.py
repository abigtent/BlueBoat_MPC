import numpy as np

def wp_selector(x_pos, y_pos, waypoints, current_wp_idx, thresh_next_wp):

    wp_curr = waypoints[current_wp_idx]
    if current_wp_idx < len(waypoints) - 1:
        wp_next = waypoints[current_wp_idx + 1]
    else:
        wp_next = wp_curr 

    dist_to_next_wp = np.linalg.norm(np.array([x_pos, y_pos]) - wp_curr)

    if dist_to_next_wp < thresh_next_wp and current_wp_idx < len(waypoints) - 1:
        current_wp_idx += 1

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

    return chi_d, pi_p, current_wp_idx, cross_track_error