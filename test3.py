import unittest
import numpy as np
from guidance2 import los_guidance  # Adjust the import as needed

class TestWaypointSwitching(unittest.TestCase):

    def setUp(self):
        # Define a simple set of waypoints: a straight line along the x-axis.
        self.waypoints = np.array([[0.0, 0.0],
                                   [10.0, 0.0],
                                   [20.0, 0.0]])
        self.los_lookahead = 5.0
        self.thresh_next_wp = 5.0

    def test_no_switch_when_far(self):
        # Start with the first segment; vessel is far from the second waypoint.
        current_wp_idx = 0
        # Position near the beginning (far from waypoint [10, 0])
        x_pos, y_pos = 1.0, 0.0
        psi_value = 0.0  # current heading
        psi_d, new_wp_idx, cross_track_error, wp_next = los_guidance(
            x_pos, y_pos, psi_value,
            self.waypoints,
            current_wp_idx,
            self.los_lookahead,
            self.thresh_next_wp
        )
        # Expect the waypoint index to remain unchanged.
        self.assertEqual(new_wp_idx, 0, "Waypoint index should not change when far from next waypoint.")

    def test_switch_when_close(self):
        # Start with the first segment; vessel is very close to the second waypoint.
        current_wp_idx = 0
        # Position near waypoint [10, 0]
        x_pos, y_pos = 9.5, 0.0
        psi_value = 0.0  # current heading
        psi_d, new_wp_idx, cross_track_error, wp_next = los_guidance(
            x_pos, y_pos, psi_value,
            self.waypoints,
            current_wp_idx,
            self.los_lookahead,
            self.thresh_next_wp
        )
        # Expect the waypoint index to increment (i.e. switch to the next waypoint)
        self.assertEqual(new_wp_idx, 1, "Waypoint index should increment when close to next waypoint.")

    def test_no_switch_at_last_waypoint(self):
        # If already at the last waypoint, index should remain at the end.
        current_wp_idx = 2  # Last waypoint
        # Position near the last waypoint
        x_pos, y_pos = 20.0, 0.0
        psi_value = 0.0  # current heading
        psi_d, new_wp_idx, cross_track_error, wp_next = los_guidance(
            x_pos, y_pos, psi_value,
            self.waypoints,
            current_wp_idx,
            self.los_lookahead,
            self.thresh_next_wp
        )
        self.assertEqual(new_wp_idx, 2, "At the end, waypoint index should remain unchanged.")

if __name__ == '__main__':
    unittest.main()
