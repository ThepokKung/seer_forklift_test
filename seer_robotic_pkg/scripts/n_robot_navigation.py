#!/usr/bin/env python3

# ROS2 imports
import rclpy
from rclpy.node import Node

# Systemp imports
import sys
import os

# srv import
from std_srvs.srv import Trigger
from seer_robot_interfaces.srv import CheckRobotNavigationTaskStatus, GetNavigationPath, NavigationParameter,PalletID, CheckRobotCurrentLocation

# backend imports
from bn_robot_navigation_api import RobotNavigationAPI
from bn_pallet_loader import PalletLoader
from bn_json_command_builder import JsonCommandBuilder

class RobotNavigation(Node):
    def __init__(self):
        super().__init__('robot_navigation')
        self.get_logger().info('Robot Navigation node has been started')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('robot_name', 'SEER_Robot_01')
        self.declare_parameter('robot_ip', '192.168.0.180')

        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        
        # Create RobotNavigationAPI instance
        self.robot_navigation_api = RobotNavigationAPI(self.robot_ip)
        
        # Connection status
        self.connection_attempted = False

        # Command builder
        self.json_command_builder = JsonCommandBuilder()
        
        # Service server
        # self.create_service(Trigger, 'robot_navigation/test_go', self.test_systemp) # Test only
        # self.create_service(PalletID, 'robot_navigation/test_go', self.test_systemp) # Test only
        self.create_service(GetNavigationPath, 'robot_navigation/get_navigation_path', self.get_navigation_path_callback)
        self.create_service(NavigationParameter, 'robot_navigation/navigation_to_station', self.navigation_to_station_callback)
        self.create_service(PalletID, 'robot_navigation/pickup_pallet_for_init', self.pickup_pallet_for_init_callback)

        # Service client
        self.check_robot_navigation_status_client = self.create_client(CheckRobotNavigationTaskStatus,'check_robot_navigation_status')
        self.check_robot_current_location_client = self.create_client(CheckRobotCurrentLocation, 'check_robot_current_location')
        
        # Timer 
        self.timer = self.create_timer(1.0, self.timer_update_status_callback)

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
    ###                     Timer                     ###
    #####################################################

    def timer_update_status_callback(self):
        self.call_check_robot_navigation_status()

    #####################################################
    ###             Service client call               ###
    #####################################################

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
                # self.get_logger().info(f'Task status: {response.task_status}, Success: {response.success}')
                return response.task_status
            else:
                self.get_logger().error('Service call failed - no response')
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')
        return None
    
    def call_check_robot_current_location(self):
        """Call the check_robot_current_location service"""
        if not self.check_robot_current_location_client.service_is_ready():
            self.get_logger().warn('check_robot_current_location service not available')
            return None

        request = CheckRobotCurrentLocation.Request()
        future = self.check_robot_current_location_client.call_async(request)

        # Add callback to handle the response instead of blocking
        future.add_done_callback(self.handle_current_location_response)

    def handle_current_location_response(self, future):
        """Handle the response from the current location service"""
        try:
            response = future.result()
            if response is not None:
                # self.get_logger().info(f'Current location: {response.current_location}, Success: {response.success}')
                return response.current_location
            else:
                self.get_logger().error('Service call failed - no response')
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')
        return None
    
    #####################################################
    ###             Service Callbacks                 ###
    #####################################################
    
    def get_navigation_path_callback(self, request, response):
        id2go = request.id2go
        self.get_logger().info(f'Received request for navigation path to {id2go}')
        
        # Ensure connection before making API call
        if not self.ensure_connection():
            self.get_logger().error('Failed to connect to robot navigation API')
            response.path = []
            response.success = False
            response.message = f'Failed to connect to robot for navigation path to {id2go}'
            return response
        
        temp_response = self.robot_navigation_api.get_navigation_path(id2go=id2go)
        # print(f"Navigation path response: {temp_response}")
        if temp_response is not None:
            response.path = temp_response.get('path', [])
            response.success = True
            response.message = f'Navigation path to {id2go} retrieved successfully'
            self.get_logger().info(f'Navigation path: {response.path}')
        else:
            self.get_logger().error('Failed to get navigation path')
            response.path = []
            response.success = False
            response.message = f'Failed to get navigation path to {id2go}'
        return response
    
    def navigation_to_station_callback(self, request, response):
        self.get_logger().info(f'Received request to navigate to station {request.id}')
        
        # Ensure connection before making API call
        if not self.ensure_connection():
            self.get_logger().error('Failed to connect to robot navigation API')
            response.success = False
            response.message = 'Failed to connect to robot for navigation'
            return response

        id = request.id
        source_id = request.source_id
        task_id = request.task_id
        operation = request.operation
        end_height = request.end_height
        recognize = request.recognize
        recfile = request.recfile

        # Call the navigation API
        try:
            result = self.robot_navigation_api.navigation_to_goal(id=id, source_id=source_id, task_id=task_id)
            self.get_logger().info(f'Navigation API result: {result}')
            
            # Handle different possible response formats
            if isinstance(result, dict):
                response.success = result.get('success', True)  # Default to True if 'success' key not found
                response.message = result.get('message', f'Navigation to {id} initiated')
            elif result is not None:
                # If result is not a dict but exists, assume success
                response.success = True
                response.message = f'Navigation to {id} initiated successfully'
            else:
                # If result is None, assume failure
                response.success = False
                response.message = f'Navigation to {id} failed - no response from API'
                
            self.get_logger().info(f'Navigation result: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error during navigation: {e}')
            response.success = False
            response.message = f'Error during navigation: {e}'
        
        return response
    
    def pickup_pallet_for_init_callback(self, request, response):
        pass
    
    #####################################################
    ###                    Test                       ###
    #####################################################

    # def test_systemp(self, request, response):
    #     result = self.Test_go()
    #     response.success = result
    #     response.message = "Test navigation path requested"
    #     return response

    # def test_systemp(self, request, response):
    #     id2go = request.pallet_id
    #     result = self.Test_go(current_station, id2go, task_id)
    #     response.success = result
    #     response.message = "Test navigation path requested"
    #     return response

    # def Test_go(self, current_station, id2go, task_id):
    #     if self.ensure_connection():
    #         self.json_command_builder.test_command(current_station,id2go,task_id)
    #     return True

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