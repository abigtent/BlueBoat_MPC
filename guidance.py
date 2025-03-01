import numpy as np

def los_guidance(x_pos, y_pos, psi, waypoints, current_wp_idx, los_lookahead, thresh_next_wp):
        if current_wp_idx >= len(waypoints):
            return psi, current_wp_idx, 0.0, waypoints[-1]
        
        x, y = x_pos, y_pos

        wp_curr = waypoints[current_wp_idx]
        wp_next = waypoints[min(current_wp_idx + 1, len(waypoints) - 1)]
        
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

        if np.hypot(x-wp_next[0], y-wp_next[1]) < thresh_next_wp:
            current_wp_idx += 1

        return psi_d, current_wp_idx, cross_track_error, wp_next
