#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class RobotSimPathPlanning(Node):
    def __init__(self):
        super().__init__('robot_sim_path_planning')
        self.get_logger().info('Robot Sim Path Planning node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimPathPlanning()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()