#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import fastapi, uvicorn

class FastAPIClient(Node):
    def __init__(self):
        super().__init__('fastAPI_client')
        self.get_logger().info('FastAPI Client node started')

def main(args=None):
    rclpy.init(args=args)
    node = FastAPIClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()