#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RobotManagement(Node):
    def __init__(self):
        super().__init__('robot_management')
        self.get_logger().info('Robot Management node has been started')


def main(args=None):
    rclpy.init(args=args)
    node = RobotManagement()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()