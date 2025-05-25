import numpy as np

def wp_selector(x_pos, y_pos, waypoints, current_wp_idx, thresh_switch_wp):
    wp1 = waypoints[current_wp_idx]
    wp2 = waypoints[min(current_wp_idx + 1, len(waypoints) - 1)]
    
    # Compute vector along path
    path_vec = wp2 - wp1
    path_unit = path_vec / np.linalg.norm(path_vec)
    vessel_vec = np.array([x_pos, y_pos]) - wp1
    
    # Project vessel position onto path
    proj_length = np.dot(vessel_vec, path_unit)

    if proj_length >= np.linalg.norm(path_vec) and current_wp_idx < len(waypoints) - 1:
        current_wp_idx += 1
        wp1 = waypoints[current_wp_idx]
        wp2 = waypoints[min(current_wp_idx + 1, len(waypoints) - 1)]

    dist_wp2 = np.linalg.norm(np.array([x_pos, y_pos]) - wp2)
    finished = (current_wp_idx == len(waypoints) - 1 and dist_wp2 <= thresh_switch_wp)

    return wp1, wp2, current_wp_idx, dist_wp2, finished



        
def cross_track_error(x_pos, y_pos, waypoint_1, waypoint_2):
    # Calculate the path angle
    pi_p = np.arctan2(waypoint_2[1] - waypoint_1[1], waypoint_2[0] - waypoint_1[0])
    
    # Calculate the along and cross track errors
    R_t = np.array([[np.cos(pi_p), np.sin(pi_p)], [-np.sin(pi_p), np.cos(pi_p)]])
    e = R_t @ np.array([[x_pos - waypoint_1[0]], [y_pos - waypoint_1[1]]])
    e_x = e[0] # along track error
    e_y = e[1] # cross track error

    return e_x, e_y, pi_p

def los_guidance(x_pos, y_pos, waypoints, current_wp_idx, los_lookahead, thresh_switch_wp):
        
    waypoint_1, waypoint_2, current_wp_idx, dist_waypoint_2, finished = wp_selector(x_pos, y_pos, waypoints, current_wp_idx, thresh_switch_wp)
    
    _, e_y, pi_p = cross_track_error(x_pos, y_pos, waypoint_1, waypoint_2)
    
    psi_d = pi_p - np.arctan(e_y / los_lookahead) 

    return psi_d, e_y, pi_p, current_wp_idx, dist_waypoint_2, finished