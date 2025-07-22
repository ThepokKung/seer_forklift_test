#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

import json
import time

from bn_robot_navigation_api import RobotNavigationAPI
# from bn_robot_status_api import RobotStatusAPI
from bn_pallet_loader import PalletLoader
from bn_json_command_builder import JsonCommandBuilder

from seer_robot_interfaces.srv import PalletID,CheckRobotCurrentLocation,GetNavigationPath,CheckRobotNavigationTaskStatus

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import os
from dotenv import load_dotenv
load_dotenv()  # Load environment variables from .env file

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Test Node started')

        # Declare parameters
        # robot 1
        # self.declare_parameter('robot_id', 'robot_01')
        # self.declare_parameter('robot_name', 'SEER_Robot_01')
        # self.declare_parameter('robot_ip', '192.168.0.180')
        # robot 2
        self.declare_parameter('robot_id', 'robot_02')
        self.declare_parameter('robot_name', 'SEER_Robot_02')
        self.declare_parameter('robot_ip', '192.168.0.181')

        # Parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        # API call
        self.robot_navigation_api = RobotNavigationAPI(self.robot_ip)
        # self.status_api = RobotStatusAPI(self.robot_ip)
        
        self.json_command_builder = JsonCommandBuilder()

        # Database configuration (lazy loading)
        self.db_config = {
            'host': os.getenv("DB_HOST"),
            'port': os.getenv("DB_PORT"),
            'name': os.getenv("DB_NAME"),
            'user': os.getenv("DB_USER"),
            'pass': os.getenv("DB_PASS")
        }
        
        # Initialize PalletLoader as None - will be created when needed
        self.pallet_loader = PalletLoader(
            db_host=self.db_config['host'],
            db_port=self.db_config['port'],
            db_name=self.db_config['name'],
            db_user=self.db_config['user'],
            db_pass=self.db_config['pass']
        ) if all(self.db_config.values()) else None
        
        self.get_logger().info(f'Database config ready: {self.db_config["name"]} at {self.db_config["host"]}:{self.db_config["port"]} as user {self.db_config["user"]}')

        # Service server
        self.create_service(GetNavigationPath,'test_node/id2go', self.test_id2go_callback)

        # Service client
        self.check_robot_current_location_cbg = MutuallyExclusiveCallbackGroup()
        self.check_robot_current_location_client = self.create_client(CheckRobotCurrentLocation, 'check_robot_current_location', callback_group=self.check_robot_current_location_cbg)
        self.check_robot_navigation_cbg = MutuallyExclusiveCallbackGroup()
        self.check_robot_navigation_client = self.create_client(CheckRobotNavigationTaskStatus, 'check_robot_navigation_status', callback_group=self.check_robot_navigation_cbg)

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

    def test_id2go_callback(self, request, response):
        self.get_logger().info(f'Received request to test ID2GO: {request.id2go}')
        id2go_temp = request.id2go

        # Ensure connection to robot navigation API
        if not self.ensure_connection():
            response.success = False
            response.message = "Failed to connect to robot navigation API."
            response.path = [id2go_temp]
            self.get_logger().error(response.message)
            return response

        try:
            curren_location_temp = self.call_check_robot_current_location_sync()
            self.get_logger().warning(f'Current location: {curren_location_temp}')
            if curren_location_temp is None:
                response.success = False
                response.message = "Failed to get current robot location."
                response.path = [id2go_temp]
                self.get_logger().error(response.message)
                return response
            
            # Get the list of navigation commands
            command_list = self.json_command_builder.test_command(curren_location_temp, id2go_temp, "task_123")
            self.get_logger().info(f"Generated {len(command_list)} navigation commands")
            
            # Execute each command and wait for completion
            for step_num, command_dict in enumerate(command_list, 1):
                # Convert command to JSON string
                command_json = json.dumps(command_dict)
                self.get_logger().info(f"Executing Step {step_num}: {command_json}")
                
                # Send the navigation command
                test = self.robot_navigation_api.navigation_with_json(command_json)
                self.get_logger().info(f"response from navigation: {test}")

                # Wait for this step to complete
                # First, wait for status to change from 4 (if it was already 4) to something else (like 2 = in progress)
                robot_navigation_status = self.call_check_robot_navigation_sync()
                self.get_logger().info(f"Step {step_num} - Initial status: {robot_navigation_status}")
                
                # If status is already 4, wait for it to change to indicate the new task has started
                if robot_navigation_status == 4:
                    self.get_logger().info(f"Step {step_num} - Status is 4, waiting for task to start...")
                    while robot_navigation_status == 4:
                        time.sleep(0.5)
                        robot_navigation_status = self.call_check_robot_navigation_sync()
                        if robot_navigation_status is not None and robot_navigation_status != 4:
                            self.get_logger().info(f"Step {step_num} - Task started, status changed to: {robot_navigation_status}")
                            break
                        elif robot_navigation_status is None:
                            self.get_logger().warn(f"Step {step_num} - Failed to get navigation status")
                            break
                
                # Now wait for the task to complete (status becomes 4)
                while robot_navigation_status != 4:
                    robot_navigation_status = self.call_check_robot_navigation_sync()
                    if robot_navigation_status is not None:
                        self.get_logger().info(f"Step {step_num} - Robot navigation status: {robot_navigation_status}")
                        if robot_navigation_status == 4:
                            self.get_logger().info(f"Step {step_num} completed successfully")
                            break
                    else:
                        self.get_logger().warn(f"Step {step_num} - Failed to get navigation status")
                        break
                    
                    # Add a small delay to avoid overwhelming the service
                    time.sleep(0.5)

            response.success = True
            response.message = f"All {len(command_list)} navigation commands executed successfully."
            response.path = [curren_location_temp, id2go_temp]
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error checking robot current location: {e}"
            response.path = [id2go_temp]
            self.get_logger().error(response.message)
            return response
            
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
        if not self.check_robot_navigation_client.service_is_ready():
            self.get_logger().warn('check_robot_navigation service not available')
            return None

        request = CheckRobotNavigationTaskStatus.Request()
        try:
            future = self.check_robot_navigation_client.call_async(request)
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
        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()