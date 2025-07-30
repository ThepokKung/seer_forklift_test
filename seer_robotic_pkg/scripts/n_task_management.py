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
from seer_robot_interfaces.srv import PalletID, AssignTask,CheckRobotAllForTask
# backend imports
from bn_pallet_loader import PalletLoader

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
            # Create client for robot state check
            state_callback_group = MutuallyExclusiveCallbackGroup()
            state_client = self.create_client(CheckRobotAllForTask,f'{robot_ns}/robot_state/check_robot_all_for_task',callback_group=state_callback_group)
            
            # Create client for navigation path
            nav_callback_group = MutuallyExclusiveCallbackGroup()
            nav_client = self.create_client(CheckRobotAllForTask,f'{robot_ns}/robot_navigation/get_navigation_path',callback_group=nav_callback_group)
            
            self.robot_clients[robot_ns] = {
                'state': state_client,
                'navigation': nav_client
                }
            self.get_logger().info(f'Created service clients for {robot_ns}')

        # Service server
        self.create_service(AssignTask, 'task_management/assign_task', self.assign_task_callback)


    #####################################################
    ###             Service Callbacks                 ###
    #####################################################

    def pick_pallet_for_init_callback(self, request, response):
        self.get_logger().info(f'Received request to pick pallet with ID: {request.pallet_id}')
        try:
            pallet_data = self.pallet_loader.get_pallet_data_id(request.pallet_id)
            response.success = True
            response.message = f"Pallet with ID {request.pallet_id} has been picked successfully. Details: {pallet_data}"
            self.get_logger().info(f'Pallet picked successfully: {response.message}')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Error picking pallet: {response.message}')
        return response
    
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
            # Check all robots and find available one
            available_robot = self.find_available_robot()

            self.pallet_loader.get_pallet_data_id(request.pallet_id)
            
            if available_robot is None:
                response.success = False
                response.message = "No available robots found for task assignment"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info(f'Selected robot {available_robot} for task assignment')
            
            response.success = True
            response.message = f"Task {request.task_id} of type {request.task_type_id} has been assigned to {available_robot} successfully."
            self.get_logger().info(f'Task assigned successfully: {response.message}')
            
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
            robot_status = self.check_robot_status(robot_ns)
            if robot_status and robot_status.success:
                # Add your logic here to determine if robot is available
                # For example, check if robot_task_status indicates it's idle
                if robot_status.robot_task_status == "IDLE" or robot_status.robot_task_status == "READY":
                    self.get_logger().info(f'Robot {robot_ns} is available')
                    return robot_ns
                else:
                    self.get_logger().info(f'Robot {robot_ns} is busy: {robot_status.robot_task_status}')
            else:
                self.get_logger().warn(f'Could not get status for robot {robot_ns}')
        return None
    
    def check_robot_status(self, robot_namespace):
        """Check status of a specific robot"""
        if robot_namespace not in self.robot_clients:
            self.get_logger().error(f'No client found for robot {robot_namespace}')
            return None
            
        client = self.robot_clients[robot_namespace]
        
        if not client.service_is_ready():
            self.get_logger().warn(f'Service {robot_namespace}/check_robot_all_for_task not available')
            return None
        
        service_request = CheckRobotAllForTask.Request()
        try:
            future = client.call_async(service_request)
            rclpy.spin_until_future_complete(self, future)
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
        available_robot = self.find_available_robot()
        if available_robot:
            return self.check_robot_status(available_robot)
        else:
            response.success = False
            response.robot_current_station = "Unknown"
            response.robot_task_status = "Unknown"
            response.robot_navigation_status = "Unknown"
            return response
    
def main(args=None):
    rclpy.init(args=args)
    node = TaskManagement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()