#!/usr/bin/env python3
# ROS2 import
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import os
import json

class TrafficManagement(Node):
    def __init__(self):
        super().__init__('traffic_management')
        self.get_logger().info('Traffic Management node has been started')

        # Declare Parameters
        self.declare_parameter('map_file', 'Kraiwich_Map_v.0.3.7.smap')
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value

        # YAML file path
        path_file = os.path.join(
            get_package_share_directory('seer_robot_pkg'),
            'config',
            self.map_file
        )

        # Load map file
        try:
            with open(path_file, 'r') as file:
                self.map_data = json.load(file)
                self.get_logger().info(f"Map data loaded successfully from {self.map_file}")
        except FileNotFoundError:
            self.get_logger().error(f"Map file {self.map_file} not found.")
            self.get_logger().error(f"Failed to load map data. Node Shutdown!")
            rclpy.shutdown()

        print(f"Map data: {self.map_data}")

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