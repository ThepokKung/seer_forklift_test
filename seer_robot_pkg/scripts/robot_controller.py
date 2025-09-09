#!/usr/bin/env python3

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup # Mutiple callback groups for service clients

# System imports
import os
import json
import time

#msg import
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

# srv import
from seer_robot_interfaces.srv import CheckRobotNavigationTaskStatus, GetNavigationPath, PalletID, AssignTask

# backend imports
from seer_robot_pkg.robot_navigation_api import RobotNavigationAPI
from seer_robot_pkg.pallet_loader import PalletLoader
from seer_robot_pkg.json_command_builder import JsonCommandBuilder

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

        # Subscriptions
        self.create_subscription(Int32, 'robot_status/robot_navigation_status', self._sub_check_robot_navigation_status_callback, 10)

        # Service server
        self.create_service(GetNavigationPath, 'robot_controller/get_navigation_path', self.get_navigation_path_callback)
        self.create_service(AssignTask, 'robot_controller/assign_task', self.assign_task_callback)
        self.create_service(Trigger,'robot_controller/cancel_navigation',self.cancel_navigation_callback)
        self.create_service(Trigger,'robot_controller/pause_navigation',self.pause_navigation_callback)
        self.create_service(Trigger,'robot_controller/resume_navigation',self.resume_navigation_callback)

        # Service client
        self.check_robot_navigation_state_cbg = MutuallyExclusiveCallbackGroup()
        self.check_robot_navigation_state_client = self.create_client(CheckRobotNavigationTaskStatus, 'robot_controller/check_robot_navigation_status', callback_group=self.check_robot_navigation_state_cbg)

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

    def assign_task_callback(self, request, response):
        self.get_logger().info(f'Received request to assign task: {request.task_id}')
        self.get_logger().info(f'Task Type ID: {request.task_type_id}, Pallet ID: {request.pallet_id}')
        
        # Create a PalletID request for the specific task
        pallet_request = PalletID.Request()
        pallet_request.pallet_id = request.pallet_id
        
        # Create a PalletID response
        pallet_response = PalletID.Response()
        
        try:
            # Route to appropriate task based on task_type_id
            if request.task_type_id == 1:  # PickInit
                self.get_logger().info(f'Executing PickInit for pallet {request.pallet_id}')
                pallet_response = self.pallet_pick_init_callback(pallet_request, pallet_response)
                
            elif request.task_type_id == 2:  # PlaceInit
                self.get_logger().info(f'Executing PlaceInit for pallet {request.pallet_id}')
                pallet_response = self.pallet_place_init_callback(pallet_request, pallet_response)
                
            elif request.task_type_id == 3:  # PickToManipulator
                self.get_logger().info(f'Executing PickToManipulator for pallet {request.pallet_id}')
                pallet_response = self.pallet_pick_to_manipulator_callback(pallet_request, pallet_response)
                
            elif request.task_type_id == 4:  # PickFromManipulator
                self.get_logger().info(f'Executing PickFromManipulator for pallet {request.pallet_id}')
                pallet_response = self.pallet_pick_from_manipulator_callback(pallet_request, pallet_response)
                
            else:
                self.get_logger().error(f'Invalid task_type_id: {request.task_type_id}. Valid values are 1-4.')
                response.success = False
                response.message = f'Invalid task_type_id: {request.task_type_id}. Valid values are: 1=PickInit, 2=PlaceInit, 3=PickToManipulator, 4=PickFromManipulator'
                return response
            
            # Set response based on pallet task result
            response.success = pallet_response.success
            if pallet_response.success:
                response.message = f'Task {request.task_id} (Type: {request.task_type_id}, Pallet: {request.pallet_id}) completed successfully. {pallet_response.message}'
            else:
                response.message = f'Task {request.task_id} (Type: {request.task_type_id}, Pallet: {request.pallet_id}) failed. {pallet_response.message}'
                
        except Exception as e:
            self.get_logger().error(f'Exception during task assignment: {e}')
            response.success = False
            response.message = f'Task {request.task_id} failed with exception: {str(e)}'
            
        return response

    # Execute command
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
            # robot_navigation_status = self.call_check_robot_navigation_sync()
            
            self.get_logger().info(f"{context_name} Step {step_num} - Initial status: {self.robot_navigation_status}")
            
            # If status is already 4, wait for it to change to indicate the new task has started
            if self.robot_navigation_status == 4:
                self.get_logger().info(f"{context_name} Step {step_num} - Status is 4, waiting for task to start...")
                while self.robot_navigation_status == 4:
                    time.sleep(0.5)
                    if self.robot_navigation_status is not None and self.robot_navigation_status != 4:
                        self.get_logger().info(f"{context_name} Step {step_num} - Task started, status changed to: {self.robot_navigation_status}")
                        break
                    elif self.robot_navigation_status is None:
                        self.get_logger().warn(f"{context_name} Step {step_num} - Failed to get navigation status")
                        return False, f"Failed to get navigation status for {context_name} step {step_num}"
            
            # Now wait for the task to complete (status becomes 4)
            while self.robot_navigation_status != 4:
                if self.robot_navigation_status is not None:
                    self.get_logger().info(f"{context_name} Step {step_num} - Robot navigation status: {self.robot_navigation_status}")
                    if self.robot_navigation_status == 4:
                        self.get_logger().info(f"{context_name} Step {step_num} completed successfully")
                        break
                    elif self.robot_navigation_status == 5:
                        self.get_logger().error(f"{context_name} Step {step_num} failed (status: FAILED)")
                        return False, f"{context_name} step {step_num} failed"
                    elif self.robot_navigation_status == 6:
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

    # Pick to Manipulator
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
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_pick_to_manipulator_command(pallet_data, "task_123")
            
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

    # Place from manipulator
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
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_pick_from_manipulator_command(pallet_data, "task_123")
            
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

    # Pick Init
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
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_pick_init_command(pallet_data, "task_123")
            
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
        
    # Place Init
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
            # Get the list of navigation commands
            command_list = self.json_command_builder.pallet_place_init_command(pallet_data, "task_123")
            
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
    ###                 Sub callback                  ###
    #####################################################

    def _sub_check_robot_navigation_status_callback(self, msg):
        self.robot_navigation_status = msg.data # Int32

    #####################################################
    ###                    Test                       ###
    #####################################################

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