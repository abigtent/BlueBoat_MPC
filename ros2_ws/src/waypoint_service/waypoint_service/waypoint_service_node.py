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

        try:
            with open(filepath, newline='') as csvfile:
                csvreader = csv.reader(csvfile)
                for row in csvreader:
                    if len(row) >= 2:
                        lat, lon = map(float, row[:2])
                        waypoints.append((lat, lon))

            if not waypoints:
                self.get_logger().error('CSV file is empty or invalid.')
                return response

            origin_lat, origin_lon = waypoints[0]

            response.origin = GpsOrigin(
                latitude=origin_lat,
                longitude=origin_lon,
                altitude=0.0  # Define if needed
            )

            waypoint_array = WaypointArray()

            for lat, lon in waypoints:
                # pymap3d.geodetic2ned(lat, lon, alt, lat0, lon0, alt0)
                north, east, down = pm.geodetic2ned(
                    lat, lon, 0.0,  # waypoint altitude (0.0 if unknown)
                    origin_lat, origin_lon, 0.0  # origin altitude (set accordingly)
                )

                waypoint = Waypoint()
                waypoint.x = north
                waypoint.y = east
                waypoint_array.waypoints.append(waypoint)

            response.waypoints = waypoint_array

            self.get_logger().info("Waypoints processed successfully with pymap3d.")

        except Exception as e:
            self.get_logger().error(f'Error processing waypoints: {e}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
