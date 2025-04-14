#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
from pyproj import Transformer
from your_ros2_pkg.srv import SetWaypoints
from your_ros2_pkg.msg import GpsOrigin, Waypoint, WaypointArray

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

            # First waypoint is origin
            origin_lat, origin_lon = waypoints[0]
            response.origin = GpsOrigin(
                latitude=origin_lat,
                longitude=origin_lon,
                altitude=0.0  # default or fixed altitude if needed
            )

            transformer = Transformer.from_crs(
                "epsg:4326",  # WGS84 lat/lon
                f"+proj=tmerc +lat_0={origin_lat} +lon_0={origin_lon} +units=m +datum=WGS84",
                always_xy=True
            )

            waypoint_array = WaypointArray()

            for lat, lon in waypoints:
                east, north = transformer.transform(lon, lat)
                waypoint = Waypoint(x=north, y=east)  # NED: x=north, y=east
                waypoint_array.waypoints.append(waypoint)

            response.waypoints = waypoint_array

            self.get_logger().info("Waypoints processed successfully (XY only).")

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
