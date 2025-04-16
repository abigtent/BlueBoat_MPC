#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import pymap3d as pm
from waypoint_interfaces.srv import SetWaypoints
from waypoint_interfaces.msg import GpsOrigin, Waypoint, WaypointArray

class WaypointServiceNode(Node):
    def __init__(self):
        super().__init__('waypoint_service_node')
        self.srv = self.create_service(SetWaypoints, 'set_waypoints', self.handle_set_waypoints)
        self.get_logger().info("Waypoint Service Node ready.")

    def handle_set_waypoints(self, request, response):
        filepath = request.filepath
        waypoints = []

        # Read the CSV file with robust error checking.
        try:
            with open(filepath, newline='') as csvfile:
                csvreader = csv.reader(csvfile)
                for row in csvreader:
                    if len(row) >= 2:
                        try:
                            lat, lon = map(float, row[:2])
                            waypoints.append((lat, lon))
                            self.get_logger().debug(f"Loaded waypoint: ({lat}, {lon})")
                        except ValueError as ve:
                            self.get_logger().error(f"Error converting row {row} to floats: {ve}")
                    else:
                        self.get_logger().warning(f"Skipping row with insufficient columns: {row}")
        except FileNotFoundError as fnfe:
            self.get_logger().error(f"CSV file not found at {filepath}: {fnfe}")
            return response
        except Exception as e:
            self.get_logger().error(f"Unexpected error reading CSV file: {e}")
            return response

        if not waypoints:
            self.get_logger().error("CSV file is empty or contains no valid waypoints.")
            return response

        # Set the first waypoint as the origin.
        origin_lat, origin_lon = waypoints[0]
        response.origin = GpsOrigin(
            latitude=origin_lat,
            longitude=origin_lon,
            altitude=0.0  # Adjust altitude if needed.
        )
        self.get_logger().info(f"Origin set to: ({origin_lat}, {origin_lon})")

        waypoint_array = WaypointArray()

        # Convert each waypoint from geodetic to NED coordinates.
        for lat, lon in waypoints:
            try:
                north, east, down = pm.geodetic2ned(
                    lat, lon, 0.0,            # waypoint altitude (assumed 0.0)
                    origin_lat, origin_lon, 0.0  # origin altitude (assumed 0.0)
                )
                self.get_logger().debug(f"Converted ({lat}, {lon}) to NED: "
                                          f"North={north}, East={east}, Down={down}")
                waypoint = Waypoint()
                waypoint.x = north
                waypoint.y = east
                waypoint_array.waypoints.append(waypoint)
            except Exception as conv_ex:
                self.get_logger().error(f"Error converting waypoint ({lat}, {lon}) to NED: {conv_ex}")

        if not waypoint_array.waypoints:
            self.get_logger().error("No waypoints were successfully converted to NED.")
            return response

        response.waypoints = waypoint_array
        self.get_logger().info(f"Successfully processed {len(waypoint_array.waypoints)} waypoints.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

