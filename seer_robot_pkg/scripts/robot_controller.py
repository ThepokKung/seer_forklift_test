#!/usr/bin/env python3

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup  # Multiple callback groups for service clients
from rclpy.executors import MultiThreadedExecutor

# System imports
import os
import json
import time
from dotenv import load_dotenv
load_dotenv()  # Load environment variables from .env file

#msg import
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

# srv import
from seer_robot_interfaces.srv import (
    AssignTask,
    CheckRobotNavigationTaskStatus,
    GetNavigationPath,
)

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
            db_host=os.getenv('DB_HOST'),
            db_port=os.getenv('DB_PORT'),
            db_name=os.getenv('DB_NAME'),
            db_user=os.getenv('DB_USER'),
            db_pass=os.getenv('DB_PASS')
        )

        # Reentrant callback group so subscription updates can run while services execute
        self.reentrant_callback_group = ReentrantCallbackGroup()

        # Subscriptions
        self.create_subscription(
            Int32,
            'robot_status/robot_navigation_status',
            self._sub_check_robot_navigation_status_callback,
            10,
            callback_group=self.reentrant_callback_group,
        )

        # Service servers
        self.create_service(
            GetNavigationPath,
            'robot_controller/get_navigation_path',
            self.get_navigation_path_callback,
            callback_group=self.reentrant_callback_group,
        )
        self.create_service(
            AssignTask,
            'robot_controller/assign_task',
            self.assign_task_callback,
            callback_group=self.reentrant_callback_group,
        )
        self.create_service(
            Trigger,
            'robot_controller/cancel_navigation',
            self.cancel_navigation_callback,
            callback_group=self.reentrant_callback_group,
        )
        self.create_service(
            Trigger,
            'robot_controller/pause_navigation',
            self.pause_navigation_callback,
            callback_group=self.reentrant_callback_group,
        )
        self.create_service(
            Trigger,
            'robot_controller/resume_navigation',
            self.resume_navigation_callback,
            callback_group=self.reentrant_callback_group,
        )

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
                self.get_logger().warning(f"Failed to connect to robot navigation at {self.robot_ip}")
                return False
        except Exception as e:
            self.get_logger().error(f"Exception during connection: {e}")
            return False

    #####################################################
    ###      Navigation Control Service Callbacks     ###
    #####################################################

    def _run_navigation_control(self, action_name, api_call, success_message, failure_message):
        self.get_logger().info(f'Received request to {action_name} navigation')

        if not self.ensure_connection():
            self.get_logger().error('Failed to connect to robot navigation API')
            return False, f'Failed to connect to robot for {action_name} navigation'

        try:
            result = api_call()
        except Exception as error:
            self.get_logger().error(f'Exception during {action_name}: {error}')
            return False, failure_message

        if result is not None and result.get('ret_code') == 0:
            self.get_logger().info(success_message)
            return True, success_message

        self.get_logger().error(failure_message)
        return False, failure_message

    def cancel_navigation_callback(self, request, response):
        success, message = self._run_navigation_control(
            'cancel',
            self.robot_navigation_api.cancel_navigation,
            'Navigation canceled successfully',
            'Failed to cancel navigation',
        )
        response.success = success
        response.message = message
        return response

    def pause_navigation_callback(self, request, response):
        success, message = self._run_navigation_control(
            'pause',
            self.robot_navigation_api.pause_navigation,
            'Navigation paused successfully',
            'Failed to pause navigation',
        )
        response.success = success
        response.message = message
        return response

    def resume_navigation_callback(self, request, response):
        success, message = self._run_navigation_control(
            'resume',
            self.robot_navigation_api.resume_navigation,
            'Navigation resumed successfully',
            'Failed to resume navigation',
        )
        response.success = success
        response.message = message
        return response

    #####################################################
    ###             Service Callbacks                 ###
    #####################################################

    def get_navigation_path_callback(self, request, response):
        id2go = request.id2go
        self.get_logger().info(f'Received request for navigation path to {id2go}')
        
        if not self.ensure_connection():
            self.get_logger().error('Failed to connect to robot navigation API')
            response.path = []
            response.success = False
            response.message = f'Failed to connect to robot for navigation path to {id2go}'
            return response
        
        temp_response = self.robot_navigation_api.get_navigation_path(id2go=id2go)
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
    
    def assign_task_callback(self, request, response):
        self.get_logger().info(f'Received request to assign task: {request.task_id}')
        self.get_logger().info(f'Task Type ID: {request.task_type_id}, Pallet ID: {request.pallet_id}')

        pallet_data = self.pallet_loader.get_pallet_data_id(request.pallet_id)

        self.get_logger().info(f'Pallet data for ID {request.pallet_id}: {pallet_data}')

        if pallet_data is None:
            response.success = False
            response.message = f"Pallet data not found for ID {request.pallet_id}"
            self.get_logger().error(response.message)
            return response

        try:
            # Route to appropriate task based on task_type_id
            if request.task_type_id == 1:  # PickInit
                self.get_logger().info(f"Executing PickInit for pallet {request.pallet_id}")
                success = self.pallet_pick_init_callback(pallet_data, request.task_id)
                response.success = bool(success)
                response.message = "Pallet Pick Init executed successfully" if success else "Pallet Pick Init failed"

            elif request.task_type_id == 2:  # PlaceInit
                self.get_logger().info(f"Executing PlaceInit for pallet {request.pallet_id}")
                success, message = self.pallet_place_init(pallet_data, request.task_id)
                response.success = bool(success)
                response.message = message

            elif request.task_type_id == 3:  # PickToManipulator
                self.get_logger().info(f"Executing PickToManipulator for pallet {request.pallet_id}")
                success, message = self.pallet_pick_to_manipulator(pallet_data, request.task_id)
                response.success = bool(success)
                response.message = message

            elif request.task_type_id == 4:  # PickFromManipulator
                self.get_logger().info(f"Executing PickFromManipulator for pallet {request.pallet_id}")
                success, message = self.pallet_pick_from_manipulator(pallet_data, request.task_id)
                response.success = bool(success)
                response.message = message

            else:
                self.get_logger().error(f"Invalid task_type_id: {request.task_type_id}. Valid values are 1-4.")
                response.success = False
                response.message = (
                    f"Invalid task_type_id: {request.task_type_id}. "
                    "Valid values are: 1=PickInit, 2=PlaceInit, 3=PickToManipulator, 4=PickFromManipulator"
                )
                return response

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
                    self.get_logger().info(f"Now : robot navigation Status {self.robot_navigation_status}")
                    time.sleep(0.5)
                    if self.robot_navigation_status is not None and self.robot_navigation_status != 4:
                        self.get_logger().info(f"{context_name} Step {step_num} - Task started, status changed to: {self.robot_navigation_status}")
                        break
                    elif self.robot_navigation_status is None:
                        self.get_logger().warning(f"{context_name} Step {step_num} - Failed to get navigation status")
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
                    elif self.robot_navigation_status == 0:
                        self.get_logger().error(f"{context_name} Step {step_num} idle (status: IDLE)")
                        return False, f"{context_name} step {step_num} is idle unexpectedly"
                else:
                    self.get_logger().warning(f"{context_name} Step {step_num} - Failed to get navigation status")
                    return False, f"Failed to get navigation status for {context_name} step {step_num}"
                
                # Add a small delay to avoid overwhelming the service
                time.sleep(0.5)

        return True, f"All {len(command_list)} {context_name} commands executed successfully."

    # Internal helper to run a task sequence
    def _run_task_sequence(self, pallet_data, build_commands_fn, context_name, task_id: str = "task_123"):
        if pallet_data is None:
            return False, "Pallet data not provided"

        # Ensure connection to robot navigation API
        if not self.ensure_connection():
            self.get_logger().error("Failed to connect to robot navigation API.")
            return False, "Failed to connect to robot navigation API."

        try:
            command_list = build_commands_fn(pallet_data, task_id)
            success, message = self.execute_navigation_commands(command_list, context_name)
            return success, message
        except Exception as e:
            err = f"Error during {context_name}: {e}"
            self.get_logger().error(err)
            return False, err

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
    def pallet_pick_init_callback(self, pallet_data, task_id: str = "task_123"):
        self.get_logger().info(f"Received request to test Pallet Pick Init: {pallet_data['pallet_id']}")
        # pallet_id_temp = pallet_data.pallet_id

        # pallet_data = self.pallet_loader.get_pallet_data_id(int(pallet_id_temp)) # type: ignore
        # self.get_logger().info(f'Pallet data for ID {pallet_id_temp}: {pallet_data}')

        success, message = self._run_task_sequence(
            pallet_data,
            self.json_command_builder.pallet_pick_init_command,
            "Pallet Pick Init",
            task_id
        )
        self.get_logger().info(message)
        return success
        
    # Place Init
    def pallet_place_init(self, pallet_data, task_id: str = "task_123"):
        self.get_logger().info(f"Received request to test Pallet Place Init: {pallet_data['pallet_id']}")
        return self._run_task_sequence(
            pallet_data,
            self.json_command_builder.pallet_place_init_command,
            "Pallet Place Init",
            task_id
        )

    def pallet_pick_to_manipulator(self, pallet_data, task_id: str = "task_123"):
        self.get_logger().info(f"Received request to test Pallet Pick to Manipulator: {pallet_data['pallet_id']}")
        return self._run_task_sequence(
            pallet_data,
            self.json_command_builder.pallet_pick_to_manipulator_command,
            "Pallet Pick to Manipulator",
            task_id
        )

    def pallet_pick_from_manipulator(self, pallet_data, task_id: str = "task_123"):
        self.get_logger().info(f"Received request to test Pallet Pick from Manipulator: {pallet_data['pallet_id']}")
        return self._run_task_sequence(
            pallet_data,
            self.json_command_builder.pallet_pick_from_manipulator_command,
            "Pallet Pick from Manipulator",
            task_id
        )

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
    executor = MultiThreadedExecutor(num_threads=4)
    node.executor = executor
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
