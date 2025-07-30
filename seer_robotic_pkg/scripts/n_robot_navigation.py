#!/usr/bin/env python3

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup # Mutiple callback groups for service clients

# System imports
import sys
import os
import json
import time

# srv import
from seer_robot_interfaces.srv import CheckRobotNavigationTaskStatus, GetNavigationPath, PalletID, CheckRobotCurrentLocation

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

        # Pallet loader
        self.pallet_loader = PalletLoader(
            db_host=os.getenv('DB_HOST', 'localhost'),
            db_port=os.getenv('DB_PORT', '5432'),
            db_name=os.getenv('DB_NAME', 'seer_db'),
            db_user=os.getenv('DB_USER', 'seer_user'),
            db_pass=os.getenv('DB_PASS', 'seer_pass')
        )
        
        # Service server
        self.create_service(GetNavigationPath, 'robot_navigation/get_navigation_path', self.get_navigation_path_callback)
        self.create_service(PalletID,'robot_navigation/pallet_pick_init_test', self.pallet_pick_init_callback)
        self.create_service(PalletID,'robot_navigation/pallet_place_init_test', self.pallet_place_init_callback)
        self.create_service(PalletID,'robot_navigation/pallet_pick_to_manipulator_test', self.pallet_pick_to_manipulator_callback)
        self.create_service(PalletID,'robot_navigation/pallet_pick_from_manipulator_test', self.pallet_pick_from_manipulator_callback)

        # Service client
        self.check_robot_current_location_cbg = MutuallyExclusiveCallbackGroup()
        self.check_robot_current_location_client = self.create_client(CheckRobotCurrentLocation, 'check_robot_current_location', callback_group=self.check_robot_current_location_cbg)
        self.check_robot_navigation_state_cbg = MutuallyExclusiveCallbackGroup()
        self.check_robot_navigation_state_client = self.create_client(CheckRobotNavigationTaskStatus, 'check_robot_navigation_status', callback_group=self.check_robot_navigation_state_cbg)

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
    ### Service Call for check robot current location ###
    #####################################################
    
    def call_check_robot_current_location_sync(self):
        """Call the check_robot_current_location service synchronously"""
        if not self.check_robot_current_location_client.service_is_ready():
            self.get_logger().warn('check_robot_current_location service not available')
            return None

        request = CheckRobotCurrentLocation.Request()
        try:
            future = self.check_robot_current_location_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response is not None and response.success:
                return response.robot_current_station
            else:
                self.get_logger().error('Service call failed or returned unsuccessful')
                return None
        except Exception as e:
            self.get_logger().error(f'Service call exception: {e}')
            return None
        
    def call_check_robot_navigation_sync(self):
        """Call the check_robot_navigation service synchronously"""
        if not self.check_robot_navigation_state_client.service_is_ready():
            self.get_logger().warn('check_robot_navigation service not available')
            return None

        request = CheckRobotNavigationTaskStatus.Request()
        try:
            future = self.check_robot_navigation_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response is not None and response.success:
                return response.task_status
            else:
                self.get_logger().error('Service call failed or returned unsuccessful')
                return None
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
    
    def execute_navigation_commands(self, command_list, context_name="Navigation"):
        self.get_logger().info(f"Generated {len(command_list)} {context_name} commands")
        
        # Execute each command and wait for completion
        for step_num, command_dict in enumerate(command_list, 1):
            # Convert command to JSON string
            command_json = json.dumps(command_dict)
            self.get_logger().info(f"Executing {context_name} Step {step_num}: {command_json}")
            
            # Send the navigation command
            response = self.robot_navigation_api.navigation_with_json(command_json)
            self.get_logger().info(f"Response from navigation: {response}")

            # Wait for this step to complete
            # First, wait for status to change from 4 (if it was already 4) to something else (like 2 = in progress)
            robot_navigation_status = self.call_check_robot_navigation_sync()
            self.get_logger().info(f"{context_name} Step {step_num} - Initial status: {robot_navigation_status}")
            
            # If status is already 4, wait for it to change to indicate the new task has started
            if robot_navigation_status == 4:
                self.get_logger().info(f"{context_name} Step {step_num} - Status is 4, waiting for task to start...")
                while robot_navigation_status == 4:
                    time.sleep(0.5)
                    robot_navigation_status = self.call_check_robot_navigation_sync()
                    if robot_navigation_status is not None and robot_navigation_status != 4:
                        self.get_logger().info(f"{context_name} Step {step_num} - Task started, status changed to: {robot_navigation_status}")
                        break
                    elif robot_navigation_status is None:
                        self.get_logger().warn(f"{context_name} Step {step_num} - Failed to get navigation status")
                        return False, f"Failed to get navigation status for {context_name} step {step_num}"
            
            # Now wait for the task to complete (status becomes 4)
            while robot_navigation_status != 4:
                robot_navigation_status = self.call_check_robot_navigation_sync()
                if robot_navigation_status is not None:
                    self.get_logger().info(f"{context_name} Step {step_num} - Robot navigation status: {robot_navigation_status}")
                    if robot_navigation_status == 4:
                        self.get_logger().info(f"{context_name} Step {step_num} completed successfully")
                        break
                    elif robot_navigation_status == 5:
                        self.get_logger().error(f"{context_name} Step {step_num} failed (status: FAILED)")
                        return False, f"{context_name} step {step_num} failed"
                    elif robot_navigation_status == 6:
                        self.get_logger().error(f"{context_name} Step {step_num} canceled (status: CANCELED)")
                        return False, f"{context_name} step {step_num} was canceled"
                else:
                    self.get_logger().warn(f"{context_name} Step {step_num} - Failed to get navigation status")
                    return False, f"Failed to get navigation status for {context_name} step {step_num}"
                
                # Add a small delay to avoid overwhelming the service
                time.sleep(0.5)

        return True, f"All {len(command_list)} {context_name} commands executed successfully."

    #####################################################
    ###              Pick Place Manipulator           ###
    #####################################################

    def pallet_pick_to_manipulator_callback(self, request, response):
        self.get_logger().info(f'Received request to test Pallet Pick to Manipulator: {request.pallet_id}')
        pallet_id_temp = request.pallet_id

        pallet_data = self.pallet_loader.get_pallet_data_id(int(pallet_id_temp)) # type: ignore
        self.get_logger().info(f'Pallet data for ID {pallet_id_temp}: {pallet_data}')

        # Ensure connection to robot navigation API
        if not self.ensure_connection():
            response.success = False
            response.message = "Failed to connect to robot navigation API."
            self.get_logger().error(response.message)
            return response

        try:
            curren_location_temp = self.call_check_robot_current_location_sync()
            self.get_logger().warning(f'Current location: {curren_location_temp}')
            if curren_location_temp is None:
                response.success = False
                response.message = "Failed to get current robot location."
                self.get_logger().error(response.message)
                return response
            
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_pick_to_manipulator_command(curren_location_temp, pallet_data, "task_123")
            
            # Execute commands using helper method
            success, message = self.execute_navigation_commands(command_list, "Pallet Pick to Manipulator")
            
            response.success = success
            response.message = message
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error during pallet pick to manipulator: {e}"
            self.get_logger().error(response.message)
            return response

    def pallet_pick_from_manipulator_callback(self, request, response):
        self.get_logger().info(f'Received request to test Pallet Pick from Manipulator: {request.pallet_id}')
        pallet_id_temp = request.pallet_id

        pallet_data = self.pallet_loader.get_pallet_data_id(int(pallet_id_temp)) # type: ignore
        self.get_logger().info(f'Pallet data for ID {pallet_id_temp}: {pallet_data}')

        # Ensure connection to robot navigation API
        if not self.ensure_connection():
            response.success = False
            response.message = "Failed to connect to robot navigation API."
            self.get_logger().error(response.message)
            return response

        try:
            curren_location_temp = self.call_check_robot_current_location_sync()
            self.get_logger().warning(f'Current location: {curren_location_temp}')
            if curren_location_temp is None:
                response.success = False
                response.message = "Failed to get current robot location."
                self.get_logger().error(response.message)
                return response
            
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_pick_from_manipulator_command(curren_location_temp, pallet_data, "task_123")
            
            # Execute commands using helper method
            success, message = self.execute_navigation_commands(command_list, "Pallet Pick from Manipulator")
            
            response.success = success
            response.message = message
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error during pallet pick from manipulator: {e}"
            self.get_logger().error(response.message)
            return response
        
    #####################################################
    ###           Pick Place Init pallet              ###
    #####################################################

    def pallet_pick_init_callback(self, request, response):
        self.get_logger().info(f'Received request to test Pallet Pick Init: {request.pallet_id}')
        pallet_id_temp = request.pallet_id

        pallet_data = self.pallet_loader.get_pallet_data_id(int(pallet_id_temp)) # type: ignore
        self.get_logger().info(f'Pallet data for ID {pallet_id_temp}: {pallet_data}')

        # Ensure connection to robot navigation API
        if not self.ensure_connection():
            response.success = False
            response.message = "Failed to connect to robot navigation API."
            self.get_logger().error(response.message)
            return response

        try:
            curren_location_temp = self.call_check_robot_current_location_sync()
            self.get_logger().warning(f'Current location: {curren_location_temp}')
            if curren_location_temp is None:
                response.success = False
                response.message = "Failed to get current robot location."
                self.get_logger().error(response.message)
                return response
            
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_pick_init_command(curren_location_temp, pallet_data, "task_123")
            
            # Execute commands using helper method
            success, message = self.execute_navigation_commands(command_list, "Pallet Pick Init")
            
            response.success = success
            response.message = message
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error during pallet pick init: {e}"
            self.get_logger().error(response.message)
            return response
        
    def pallet_place_init_callback(self, request, response):
        self.get_logger().info(f'Received request to test Pallet Place Init: {request.pallet_id}')
        pallet_id_temp = request.pallet_id

        pallet_data = self.pallet_loader.get_pallet_data_id(int(pallet_id_temp)) # type: ignore
        self.get_logger().info(f'Pallet data for ID {pallet_id_temp}: {pallet_data}')

        # Ensure connection to robot navigation API
        if not self.ensure_connection():
            response.success = False
            response.message = "Failed to connect to robot navigation API."
            self.get_logger().error(response.message)
            return response

        try:
            curren_location_temp = self.call_check_robot_current_location_sync()
            self.get_logger().warning(f'Current location: {curren_location_temp}')
            if curren_location_temp is None:
                response.success = False
                response.message = "Failed to get current robot location."
                self.get_logger().error(response.message)
                return response
            
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_place_init_command(curren_location_temp, pallet_data, "task_123")
            
            # Execute commands using helper method
            success, message = self.execute_navigation_commands(command_list, "Pallet Place Init")
            
            response.success = success
            response.message = message
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error during pallet place init: {e}"
            self.get_logger().error(response.message)
            return response

    #####################################################
    ###                    Test                       ###
    #####################################################

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