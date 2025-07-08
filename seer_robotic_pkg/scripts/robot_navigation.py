#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import os

# srv import
from std_srvs.srv import Trigger
from seer_robot_interfaces.srv import CheckRobotNavigationTaskStatus

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
        self.declare_parameter('robot_ip', '192.168.0.181')
        
        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        
        # Create RobotNavigationAPI instance
        self.robot_navigation_api = RobotNavigationAPI(self.robot_ip)
        
        # Connection status
        self.connection_attempted = False

        
        # Service server
        self.robot_test_service = self.create_service(Trigger, 'robot_navigation/test_go', self.test_systemp)

        # Service client
        self.check_robot_navigation_status_client = self.create_client(
            CheckRobotNavigationTaskStatus,
            'check_robot_navigation_status'
        )
        
        # Timer 
        self.timer = self.create_timer(1.0, self.call_check_robot_navigation_status)
        
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
    
    def call_check_robot_navigation_status(self):
        """Call the check_robot_navigation_status service"""
        if not self.check_robot_navigation_status_client.service_is_ready():
            self.get_logger().warn('check_robot_navigation_status service not available')
            return None
            
        request = CheckRobotNavigationTaskStatus.Request()
        future = self.check_robot_navigation_status_client.call_async(request)
        
        # Add callback to handle the response instead of blocking
        future.add_done_callback(self.handle_navigation_status_response)
    
    def handle_navigation_status_response(self, future):
        """Handle the response from the navigation status service"""
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f'Task status: {response.task_status}, Success: {response.success}')
                print(f'Task status: {response.task_status}, Success: {response.success}')
                return response.task_status
            else:
                self.get_logger().error('Service call failed - no response')
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')
        return None

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