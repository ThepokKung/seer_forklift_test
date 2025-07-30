#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TrafficManagement(Node):
    def __init__(self):
        super().__init__('traffic_management')
        self.get_logger().info('Traffic Management node has been started')

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