import numpy as np

class LidarSimulator:
    def __init__(self, obstacles, max_range=20, num_rays=64, inflation_radius=1.0):
        self.max_range = max_range  # Max Lidar range (meters)
        self.num_rays = num_rays  # Number of rays (resolution)
        self.angles = np.linspace(-0.5*np.pi, 0.5*np.pi, num_rays)  # 180° scan
        self.obstacles = obstacles
        self.inflation_radius = inflation_radius

    def sense_obstacles(self, boat_x, boat_y, boat_psi):
        """Simulates Lidar scan by checking distance to obstacles"""
        distances = np.full(self.num_rays, float(self.max_range))  # Default: max range

        for i, angle in enumerate(self.angles):
            ray_angle = boat_psi + angle  # Global ray direction

            for obs_x, obs_y, obs_r in self.obstacles:
                dx, dy = obs_x - boat_x, obs_y - boat_y
                distance_to_center = np.hypot(dx, dy)
                angle_to_center = np.arctan2(dy, dx)

                # Compute the signed angular difference
                angle_diff = np.arctan2(np.sin(angle_to_center - ray_angle), 
                                        np.cos(angle_to_center - ray_angle))

                # Ensure the obstacle is in front of the Lidar ray (within ±90 degrees)
                if abs(angle_diff) < np.pi / 2:
                    # Compute shortest distance along the ray
                    perpendicular_distance = abs(distance_to_center * np.sin(angle_diff))

                    if perpendicular_distance < obs_r:
                        # Compute distance to the edge of the obstacle along the ray
                        distance_along_ray = np.sqrt(distance_to_center**2 - perpendicular_distance**2) - obs_r

                        # Store the closest valid obstacle
                        if 0 < distance_along_ray < distances[i]:
                            distances[i] = max(distance_along_ray, 0.1)  # Ensure non-negative distances                 
        return distances

    def get_inflated_obstacles(self, boat_x, boat_y, boat_psi):
        """
        Uses the Lidar scan to create a list of obstacles with inflated radii.
        Each valid detection is converted into an obstacle point with a safety radius.
        """
        distances = self.sense_obstacles(boat_x, boat_y, boat_psi)
        inflated_obstacles = []

        # Loop over each ray and convert the valid measurements to global coordinates.
        for i, d in enumerate(distances):
            # Only consider detections that are less than the maximum range.
            if d < self.max_range:
                # Global angle for the current ray.
                global_angle = boat_psi + self.angles[i]
                x_hit = boat_x + d * np.cos(global_angle)
                y_hit = boat_y + d * np.sin(global_angle)
                # Create an obstacle with the inflated radius.
                inflated_obstacles.append((x_hit, y_hit, self.inflation_radius))
        return inflated_obstacles
    
