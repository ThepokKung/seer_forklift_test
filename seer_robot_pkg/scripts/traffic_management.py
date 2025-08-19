#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os, json
import sys

from seer_robot_pkg.simulation import simulate

class TrafficManagement(Node):
    def __init__(self):
        super().__init__('traffic_management')
        self.get_logger().info('Traffic Management node has been started')

        # Map data
        self.map_file = 'map.json'
        self.map_data = {}

        # Path to map file
        try:
            path_file = os.path.join(
                get_package_share_directory('seer_robot_pkg'),
                'config',
                self.map_file
            )
        except:
            # Fallback to relative path if package not found
            script_dir = os.path.dirname(os.path.abspath(__file__))
            package_dir = os.path.dirname(script_dir)
            path_file = os.path.join(package_dir, 'config', self.map_file)

        # Load map data
        try:
            with open(path_file, 'r') as f:
                self.map_data = json.load(f)
            self.get_logger().info(f"Map data loaded from {self.map_file}")
            # print(f"Map data: {self.map_data}")
        except FileNotFoundError:
            self.get_logger().error(f"Map file {self.map_file} not found at: {path_file}")
            self.get_logger().error("Node error shutdown")
            rclpy.shutdown()
            sys.exit(1)

        self.station_point = self.map_data.get('advancedPointList', [])
        self.station_curve = self.map_data.get('advancedCurveList', [])

        test_result = simulate(
            map_path=path_file,
            stationary_node='Station1',
            route_nodes=['LM44', 'LM55']
        )
        print(f"Simulation result: {test_result}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficManagement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
