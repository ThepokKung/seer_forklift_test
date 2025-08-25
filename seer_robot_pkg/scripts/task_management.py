#!/usr/bin/env python3

# ROS2 Import
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup # Mutiple callback groups for service clients

# Dotenv Import
from dotenv import load_dotenv
import os
load_dotenv()

# srv imports
from std_srvs.srv import Trigger
from seer_robot_interfaces.srv import PalletID, AssignTask, CheckRobotAllForTask, GetNavigationPath
# backend imports
from seer_robot_pkg.pallet_loader import PalletLoader
from seer_robot_pkg.json_command_builder import JsonCommandBuilder

class TaskManagement(Node):
    def __init__(self):
        super().__init__('task_management')
        self.get_logger().info('Task Management node has been started')

        # Initialize PalletLoader with environment variables
        db_host = os.getenv('DB_HOST', 'localhost')
        db_port = os.getenv('DB_PORT', '5432')
        db_name = os.getenv('DB_NAME', 'seer_db')
        db_user = os.getenv('DB_USER', 'seer_user')
        db_pass = os.getenv('DB_PASS', 'seer_pass')

        self.pallet_loader = PalletLoader(
            db_host=db_host,
            db_port=db_port,
            db_name=db_name,
            db_user=db_user,
            db_pass=db_pass
        )

        #Declare parameters
        self.declare_parameter('robot_namespaces', ['robot_01', 'robot_02'])
        
        # Get robot namespaces from parameters
        self.robot_namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        self.get_logger().info(f'Managing robots: {self.robot_namespaces}')
        self.robot_clients = {}

        # Create service clients for each robot
        for robot_ns in self.robot_namespaces:
            # Create client for robot availability check
            availability_callback_group = MutuallyExclusiveCallbackGroup()
            availability_client = self.create_client(Trigger, f'/{robot_ns}/robot_status/check_available', callback_group=availability_callback_group)

            # Create client for robot state check (detailed info)
            state_callback_group = MutuallyExclusiveCallbackGroup()
            state_client = self.create_client(CheckRobotAllForTask,f'/{robot_ns}/robot_status/check_robot_all_for_task',callback_group=state_callback_group)
            
            # Create client for navigation path
            nav_callback_group = MutuallyExclusiveCallbackGroup()
            nav_client = self.create_client(GetNavigationPath,f'/{robot_ns}/robot_controller/get_navigation_path',callback_group=nav_callback_group)

            # Create client for task assignment
            task_callback_group = MutuallyExclusiveCallbackGroup()
            task_client = self.create_client(AssignTask,f'/{robot_ns}/robot_controller/assign_task',callback_group=task_callback_group)

            self.robot_clients[robot_ns] = {
            'availability': availability_client,
            'state': state_client,
            'navigation': nav_client,
            'assign_task': task_client
            }
            self.get_logger().info(f'Created service clients for {robot_ns}')

        # Service server
        self.create_service(AssignTask, 'task_management/assign_task', self.assign_task_callback)
        self.create_service(Trigger, 'task_management/test_robot_availability', self.test_robot_availability_callback)

    #####################################################
    ###             Service Callbacks                 ###
    #####################################################

    
    #####################################################
    ###             Assign Task Callbacks             ###
    #####################################################

    def assign_task_callback(self, request, response):
        # Map task type IDs to readable names
        task_type_names = {
            1: "PickInit",
            2: "PlaceInit", 
            3: "PickToManipulator",
            4: "PickFromManipulator"
        }
        
        task_type_name = task_type_names.get(request.task_type_id, f"Unknown({request.task_type_id})")
        self.get_logger().info(f'Received request to assign task with ID: {request.task_id} and type: {task_type_name} (ID: {request.task_type_id})')

        try:
            # Get pallet data first
            pallet_data = self.pallet_loader.get_pallet_data_id(request.pallet_id)
            if not pallet_data:
                response.success = False
                response.message = f"Pallet with ID {request.pallet_id} not found"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info(f'Pallet data retrieved: {pallet_data}')
            
            # Find an available robot
            available_robot = self.find_available_robot()
            if not available_robot:
                response.success = False
                response.message = "No available robots found to assign the task"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info(f'Found available robot: {available_robot}')

            ### Load path
            
            ### task allocation here

            response.success = True
            response.message = f"Task {request.task_id} ({task_type_name}) for pallet {request.pallet_id} with data: {pallet_data}"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Error assigning task: {response.message}')
        return response
        
    #####################################################
    ###             Check Robot                      ###
    #####################################################

    def find_available_robot(self):
        """Find an available robot from all connected robots"""
        for robot_ns in self.robot_namespaces:
            # First check if robot is available using the simple availability check
            if self.check_robot_availability(robot_ns):
                robot_status = self.check_robot_status(robot_ns)
                if robot_status and robot_status.success:
                    # Check if robot_task_status indicates it's idle
                    if robot_status.robot_task_status == "IDLE" or robot_status.robot_task_status == "READY":
                        self.get_logger().info(f'Robot {robot_ns} is available and ready')
                        return robot_ns
                    else:
                        self.get_logger().info(f'Robot {robot_ns} is connected but busy: {robot_status.robot_task_status}')
                else:
                    # If detailed status unavailable, accept the robot based on availability alone
                    self.get_logger().warning(f'Robot {robot_ns} is available but detailed status unavailable — selecting based on availability')
                    return robot_ns
            else:
                self.get_logger().warning(f'Robot {robot_ns} is not available')
        return None

    def check_robot_availability(self, robot_namespace):
        """Check if a robot is available using the simple availability service"""
        if robot_namespace not in self.robot_clients:
            self.get_logger().error(f'No client found for robot {robot_namespace}')
            return False
            
        client = self.robot_clients[robot_namespace]['availability']
        
        # Wait for service to be ready with a timeout
        if not client.service_is_ready():
            self.get_logger().info(f'Waiting for service {robot_namespace}/robot_status/check_available to become available...')
            ready = client.wait_for_service(timeout_sec=2.0)
            if not ready:
                self.get_logger().warn(f'Service {robot_namespace}/robot_status/check_available not available after waiting')
                return False
        
        service_request = Trigger.Request()
        try:
            self.get_logger().debug(f'Calling robot availability service for {robot_namespace}')
            future = client.call_async(service_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                self.get_logger().warn(f'Availability service call timeout for robot {robot_namespace}')
                return False
                
            service_response = future.result()
            
            if service_response is not None:
                self.get_logger().info(f'Robot {robot_namespace} availability: {service_response.success} - {service_response.message}')
                return service_response.success
            else:
                self.get_logger().error(f'Availability service call failed for robot {robot_namespace}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Exception during availability service call for {robot_namespace}: {str(e)}')
            return False
    
    def check_robot_status(self, robot_namespace):
        """Check status of a specific robot"""
        if robot_namespace not in self.robot_clients:
            self.get_logger().error(f'No client found for robot {robot_namespace}')
            return None
            
        client = self.robot_clients[robot_namespace]['state']
        
        # Wait for service to be ready with a timeout (like availability check)
        if not client.service_is_ready():
            self.get_logger().info(f'Waiting for service {robot_namespace}/robot_status/check_robot_all_for_task to become available...')
            ready = client.wait_for_service(timeout_sec=2.0)
            if not ready:
                self.get_logger().warning(f'Service {robot_namespace}/robot_status/check_robot_all_for_task not available after waiting')
                return None
        
        service_request = CheckRobotAllForTask.Request()
        try:
            self.get_logger().debug(f'Calling robot status service for {robot_namespace}')
            future = client.call_async(service_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if not future.done():
                self.get_logger().warning(f'Service call timeout for robot {robot_namespace}')
                return None
                
            service_response = future.result()
            
            if service_response is not None and service_response.success:
                self.get_logger().info(f'Robot {robot_namespace} status: {service_response.robot_task_status}')
                return service_response
            else:
                self.get_logger().error(f'Service call failed for robot {robot_namespace}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Exception during service call for {robot_namespace}: {str(e)}')
            return None
    
    def check_robot_all(self, request, response):
        """Legacy method - now uses find_available_robot instead"""
        available_robot_ns = self.find_available_robot()
        if available_robot_ns:
            return self.check_robot_status(available_robot_ns)
        else:
            response.success = False
            response.robot_current_station = "Unknown"
            response.robot_task_status = "Unknown"
            response.robot_navigation_status = 0
            return response

    def test_robot_availability_callback(self, request, response):
        """Test service to check robot availability"""
        self.get_logger().info("Testing robot availability...")
        available_robot = self.find_available_robot()
        if available_robot:
            response.success = True
            response.message = f"Found available robot: {available_robot}"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "No available robots found"
            self.get_logger().warn(response.message)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = TaskManagement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()