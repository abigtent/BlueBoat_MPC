import numpy as np
from guidance import los_guidance
from guidance import wp_selector

def test_wp_selector():
    # Explicitly define a simple waypoint array:
    waypoints = np.array([[0.0, 0.0],
                          [100.0, 0.0],
                          [100.0, 100.0],
                          [0.0, 100.0]])
    thresh_next_wp = 10.0
    los_lookahead = 5.0

    # Explicitly set a start position near waypoint 0:
    x_pos, y_pos = 0, -5
    current_wp_idx = 0

    print("Starting tests for wp_selector():")

    for step in range(100):
        wp_next, wp_curr, current_wp_idx = wp_selector(x_pos, y_pos, waypoints, current_wp_idx, thresh_next_wp)

        chi_d, _, cross_track_error, _ = los_guidance(x_pos, y_pos, waypoints, current_wp_idx, los_lookahead, thresh_next_wp)

        print(f"Step {step}")
        print(f"Current position: ({x_pos:.2f}, {y_pos:.2f})")
        print(f"Current waypoint idx: {current_wp_idx}")
        print(f"Current waypoint: {wp_curr}, Next waypoint: {wp_next}")
        print(f"Desired course angle: {chi_d:.2f}, Cross-track error: {cross_track_error:.2f}\n")

        # Explicitly move position slightly towards next waypoint for testing
        direction = (wp_next - np.array([x_pos, y_pos]))
        direction /= np.linalg.norm(direction)
        x_pos += direction[0] * 20
        y_pos += direction[1] * 20

    print("Waypoint selector tests completed.")

if __name__ == "__main__":
    test_wp_selector()
