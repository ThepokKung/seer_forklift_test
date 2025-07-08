#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import os

from std_srvs.srv import Trigger

# My backend imports - robust import for ROS2
def import_robot_navigation_api():
    """Import RobotNavigationAPI with fallback for different installation methods"""
    try:
        # Try standard import first
        from backend.robot_navigation_api import RobotNavigationAPI
        return RobotNavigationAPI
    except ModuleNotFoundError:
        try:
            # Try direct import from installed location
            from robot_navigation_api import RobotNavigationAPI
            return RobotNavigationAPI
        except ModuleNotFoundError:
            # Add path and try again
            current_dir = os.path.dirname(os.path.abspath(__file__))
            
            # Try different possible locations
            possible_paths = [
                os.path.join(current_dir, 'backend'),
                os.path.join(current_dir, '..', 'scripts', 'backend'),
                os.path.join(current_dir, '..', '..', 'scripts', 'backend'),
            ]
            
            for path in possible_paths:
                if os.path.exists(os.path.join(path, 'robot_navigation_api.py')):
                    sys.path.insert(0, path)
                    try:
                        from robot_navigation_api import RobotNavigationAPI
                        return RobotNavigationAPI
                    except ImportError:
                        continue
            
            raise ImportError("Could not import RobotNavigationAPI from any location")

# Import RobotNavigationAPI
RobotNavigationAPI = import_robot_navigation_api()

class RobotNavigation(Node):
    def __init__(self):
        super().__init__('robot_navigation')
        self.get_logger().info('Robot Navigation node has been started')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.0.180')
        
        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        
        # Create RobotNavigationAPI instance
        self.robot_navigation_api = RobotNavigationAPI(self.robot_ip)
        
        # Connection status
        self.connection_attempted = False

        # Service call 
        self.robot_test_service = self.create_service(Trigger, 'robot_navigation/test_go', self.test_systemp)
        
        self.get_logger().info(f'Robot Navigation API initialized for {self.robot_ip}')
    
    def ensure_connection(self):
        """Ensure connection to robot navigation API"""
        if not self.connection_attempted:
            self.connection_attempted = True
            if self.robot_navigation_api.connect():
                self.get_logger().info(f"Connected to robot navigation at {self.robot_ip}")
                return True
            else:
                self.get_logger().warn(f"Failed to connect to robot navigation at {self.robot_ip}")
                return False
        return self.robot_navigation_api.connected
    
    def test_systemp(self, request, response):
        self.Test_go()

    def Test_go(self):
        if self.ensure_connection():
            payload = {
                "source_id": "LM2",
                "id": "LM1",
                "task_id": "87654321"
                }
            self.robot_navigation_api.get_navigation_path(id2go=payload)
            self.get_logger().info("Navigation path requested successfully")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()