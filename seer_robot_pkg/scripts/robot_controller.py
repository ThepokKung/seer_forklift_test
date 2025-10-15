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
from seer_robot_interfaces.msg import RobotBatchData

# srv import
from seer_robot_interfaces.srv import (
    AssignTask,
    CheckRobotNavigationTaskStatus,
    GetNavigationPath,
    CheckCollisionNavigationPath
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

        # Extract numeric ID for collision service usage
        self.robot_numeric_id = self._parse_robot_numeric_id(self.robot_id)
        if self.robot_numeric_id is None:
            self.get_logger().warning(f"Unable to determine numeric robot id from '{self.robot_id}', collision checks may fail")

        # robot parameter
        self.robot_navigation_status = 0
        
        # Logging control for navigation status
        self.navigation_status_log_counter = 0
        self.navigation_status_log_interval = 30
        self.last_navigation_status_was_error = False

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

        self.create_subscription(
            RobotBatchData,
            'robot_monitor/robot_batch_data',
            self._sub_batch_data_for_robot_navigation_status_callback,
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
        self.check_collision_navigation_path_cbg = MutuallyExclusiveCallbackGroup()
        self.check_collision_navigation_path_client = self.create_client(
            CheckCollisionNavigationPath,
            '/traffic_management/check_collision_navigation',
            callback_group=self.check_collision_navigation_path_cbg,
        )

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

    def _parse_robot_numeric_id(self, robot_id_value: str):
        """Extract the numeric robot identifier (e.g. robot_01 -> 1)."""
        digits = ''.join(ch for ch in robot_id_value if ch.isdigit())
        if not digits:
            return None

        try:
            stripped = digits.lstrip('0') or '0'
            numeric_id = int(stripped)
        except ValueError:
            return None

        return numeric_id if numeric_id > 0 else None

    def _get_navigation_route(self, target_id: str):
        """Retrieve the navigation path for the given target id."""
        if not target_id:
            return False, [], "Target id not provided for path lookup"

        if not self.ensure_connection():
            message = f"Failed to connect to robot navigation API while fetching path to {target_id}"
            return False, [], message

        try:
            path_response = self.robot_navigation_api.get_navigation_path(id2go=target_id)
        except Exception as exc:
            message = f"Exception raised while retrieving navigation path to {target_id}: {exc}"
            self.get_logger().error(message)
            return False, [], message

        if not path_response:
            message = f"Navigation API returned no data for target {target_id}"
            self.get_logger().error(message)
            return False, [], message

        route = path_response.get('path')
        if route is None:
            route = path_response.get('route')
        if route is None:
            message = f"Navigation API response missing path information for target {target_id}: {path_response}"
            self.get_logger().error(message)
            return False, [], message

        if not isinstance(route, list):
            route = list(route)

        route_as_strings = [str(point) for point in route if point is not None]
        if not route_as_strings:
            message = f"Navigation path to {target_id} is empty"
            self.get_logger().warning(message)
            return False, [], message

        self.get_logger().info(f"Retrieved navigation route to {target_id}: {route_as_strings}")
        return True, route_as_strings, f"Retrieved path with {len(route_as_strings)} nodes for target {target_id}"

    def _check_collision_for_route(self, route):
        """Invoke collision check service for the provided route."""
        if self.robot_numeric_id is None:
            message = "Numeric robot id unavailable; cannot perform collision check"
            self.get_logger().error(message)
            return False, message

        if not route:
            message = "Route is empty; skipping collision verification"
            self.get_logger().info(message)
            return True, message

        if len(route) < 2:
            message = f"Route {route} has fewer than 2 nodes; skipping collision verification"
            self.get_logger().info(message)
            return True, message

        if not self.check_collision_navigation_path_client.wait_for_service(timeout_sec=2.0):
            message = "Collision check service unavailable"
            self.get_logger().error(message)
            return False, message

        request = CheckCollisionNavigationPath.Request()
        request.this_robot_id = int(self.robot_numeric_id)
        request.route = route

        future = self.check_collision_navigation_path_client.call_async(request)

        timeout_sec = 5.0
        poll_interval = 0.05
        waited = 0.0
        while not future.done() and waited < timeout_sec:
            time.sleep(poll_interval)
            waited += poll_interval

        if not future.done():
            message = "Collision check service timed out"
            self.get_logger().error(message)
            return False, message

        try:
            service_response = future.result()
        except Exception as exc:
            message = f"Collision check service failed: {exc}"
            self.get_logger().error(message)
            return False, message

        if service_response is None:
            message = "Collision check service returned no data"
            self.get_logger().error(message)
            return False, message

        if service_response.has_collision:
            collision_message = service_response.message or "Collision detected on route"
            self.get_logger().warning(collision_message)
            return False, collision_message

        success_message = service_response.message or "No collision detected"
        self.get_logger().info(success_message)
        return True, success_message

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
                # response.message = "Pallet Pick Init executed successfully" if success else "Pallet Pick Init failed"
                response.message = f"Pallet Pick Init executed successfully" if success else f"Pallet Pick Init failed for pallet {request.pallet_id}"

            elif request.task_type_id == 2:  # PlaceInit
                self.get_logger().info(f"Executing PlaceInit for pallet {request.pallet_id}")
                success= self.pallet_place_init(pallet_data, request.task_id)
                response.success = bool(success)
                # response.message = message
                response.message = f"Pallet Place Init executed successfully" if success else f"Pallet Place Init failed for pallet {request.pallet_id}"

            elif request.task_type_id == 3:  # PickToManipulator
                self.get_logger().info(f"Executing PickToManipulator for pallet {request.pallet_id}")
                success = self.pallet_pick_to_manipulator(pallet_data, request.task_id)
                response.success = bool(success)
                # response.message = message
                response.message = f"Pallet Pick to Manipulator executed successfully" if success else f"Pallet Pick to Manipulator failed for pallet {request.pallet_id}"

            elif request.task_type_id == 4:  # PickFromManipulator
                self.get_logger().info(f"Executing PickFromManipulator for pallet {request.pallet_id}")
                success = self.pallet_pick_from_manipulator(pallet_data, request.task_id)
                response.success = bool(success)
                # response.message = message
                response.message = f"Pallet Pick from Manipulator executed successfully" if success else f"Pallet Pick from Manipulator failed for pallet {request.pallet_id}"

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

            target_id = command_dict.get("id")
            if target_id and target_id != "SELF_POSITION":
                self.get_logger().info(f"{context_name} Step {step_num} - Fetching navigation route to {target_id}")
                path_ok, route, path_message = self._get_navigation_route(target_id)
                if not path_ok:
                    failure_message = f"{context_name} Step {step_num} - {path_message}"
                    return False, failure_message

                self.get_logger().info(f"{context_name} Step {step_num} - Route lookup result: {path_message}")

                # Check collision detection for the route
                self.get_logger().info(f"{context_name} Step {step_num} - Performing collision check for route")
                collision_ok, collision_message = self._check_collision_for_route(route)
                if not collision_ok:
                    failure_message = f"{context_name} Step {step_num} - {collision_message}"
                    return False, failure_message

                self.get_logger().info(f"{context_name} Step {step_num} - Collision check passed: {collision_message}")
            else:
                self.get_logger().info(f"{context_name} Step {step_num} - No movement target, skipping collision check")

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
                    # Determine if this is an error status
                    is_error_status = self.robot_navigation_status in [5, 6]  # FAILED or CANCELED
                    
                    # Log based on conditions:
                    # 1. Always log errors immediately
                    # 2. Log first time for non-errors
                    # 3. Log every 30th time for non-errors after the first
                    should_log = False
                    
                    if is_error_status:
                        # Always log errors
                        should_log = True
                        # Reset counter when we encounter an error
                        self.navigation_status_log_counter = 0
                        self.last_navigation_status_was_error = True
                    else:
                        # For non-error status
                        if self.last_navigation_status_was_error:
                            # First time after error - log and reset
                            should_log = True
                            self.navigation_status_log_counter = 1
                            self.last_navigation_status_was_error = False
                        elif self.navigation_status_log_counter == 0:
                            # Very first time - log
                            should_log = True
                            self.navigation_status_log_counter = 1
                        elif self.navigation_status_log_counter >= self.navigation_status_log_interval:
                            # Every 30th time - log and reset counter
                            should_log = True
                            self.navigation_status_log_counter = 1
                        else:
                            # Increment counter, don't log
                            self.navigation_status_log_counter += 1
                    
                    if should_log:
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
    ###           Pick Place Init pallet              ###
    #####################################################

    # Pick Init
    def pallet_pick_init_callback(self, pallet_data, task_id: str = "task_123"):
        self.get_logger().info(f"Received request to test Pallet Pick Init: {pallet_data['pallet_id']}")

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

    def _sub_batch_data_for_robot_navigation_status_callback(self, msg):
        self.robot_navigation_status = msg.robot_navigation_status # Int32

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
