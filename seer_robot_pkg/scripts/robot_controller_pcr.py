#!/usr/bin/env python3

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup # Mutiple callback groups for service clients

# srv import
from std_srvs.srv import Trigger

# backend imports
from seer_robot_pkg.robot_navigation_api import RobotNavigationAPI

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('Robot Controller node has been started')

        # Declare parameters
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('robot_name', 'SEER_Robot_01')
        self.declare_parameter('robot_ip', '192.168.0.180')

        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value


        # robot parameter
        self.robot_navigation_status = 0

        # Create RobotNavigationAPI instance
        self.robot_navigation_api = RobotNavigationAPI(self.robot_ip)
        
        # Connection status
        self.connection_attempted = False

        self.create_service(Trigger,'robot_controller/cancel_navigation',self.cancel_navigation_callback)
        self.create_service(Trigger,'robot_controller/pause_navigation',self.pause_navigation_callback)
        self.create_service(Trigger,'robot_controller/resume_navigation',self.resume_navigation_callback)

        # Start log
        self.get_logger().info(f'Robot Navigation API initialized for {self.robot_ip}')

    #####################################################
    ###               Check Connection                ###
    #####################################################
    
    def ensure_connection(self):
        """Ensure connection to robot navigation API"""
        # Check if already connected
        if hasattr(self.robot_navigation_api, 'connected') and self.robot_navigation_api.connected:
            return True
            
        # Try to connect
        try:
            if self.robot_navigation_api.connect():
                self.get_logger().info(f"Connected to robot navigation at {self.robot_ip}")
                self.connection_attempted = True
                return True
            else:
                self.get_logger().warn(f"Failed to connect to robot navigation at {self.robot_ip}")
                return False
        except Exception as e:
            self.get_logger().error(f"Exception during connection: {e}")
            return False

    #####################################################
    ###             Service Callbacks                 ###
    #####################################################

    def cancel_navigation_callback(self, request, response):
        self.get_logger().info('Received request to cancel navigation')
        
        # Ensure connection before making API call
        if not self.ensure_connection():
            self.get_logger().error('Failed to connect to robot navigation API')
            response.success = False
            response.message = 'Failed to connect to robot for canceling navigation'
            return response
        
        temp_response = self.robot_navigation_api.cancel_navigation()
        if temp_response is not None and temp_response.get('success', False):
            response.success = True
            response.message = 'Navigation canceled successfully'
            self.get_logger().info('Navigation canceled successfully')
        else:
            self.get_logger().error('Failed to cancel navigation')
            response.success = False
            response.message = 'Failed to cancel navigation'
        return response

    def pause_navigation_callback(self, request, response):
        self.get_logger().info('Received request to pause navigation')
        
        # Ensure connection before making API call
        if not self.ensure_connection():
            self.get_logger().error('Failed to connect to robot navigation API')
            response.success = False
            response.message = 'Failed to connect to robot for pausing navigation'
            return response
        
        temp_response = self.robot_navigation_api.pause_navigation()
        if temp_response is not None and temp_response.get('success', False):
            response.success = True
            response.message = 'Navigation paused successfully'
            self.get_logger().info('Navigation paused successfully')
        else:
            self.get_logger().error('Failed to pause navigation')
            response.success = False
            response.message = 'Failed to pause navigation'
        return response
    
    def resume_navigation_callback(self, request, response):
        self.get_logger().info('Received request to resume navigation')
        
        # Ensure connection before making API call
        if not self.ensure_connection():
            self.get_logger().error('Failed to connect to robot navigation API')
            response.success = False
            response.message = 'Failed to connect to robot for resuming navigation'
            return response
        
        temp_response = self.robot_navigation_api.resume_navigation()
        if temp_response is not None and temp_response.get('success', False):
            response.success = True
            response.message = 'Navigation resumed successfully'
            self.get_logger().info('Navigation resumed successfully')
        else:
            self.get_logger().error('Failed to resume navigation')
            response.success = False
            response.message = 'Failed to resume navigation'
        return response
    #####################################################
    ###                 Sub callback                  ###
    #####################################################

    def _sub_check_robot_navigation_status_callback(self, msg):
        self.robot_navigation_status = msg.data # Int32

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()