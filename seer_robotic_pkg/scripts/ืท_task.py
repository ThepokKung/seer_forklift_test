#!/usr/bin/env python3

# ROS2 Import
import rclpy
from rclpy.node import Node

from dotenv import load_dotenv
import os
load_dotenv()

# srv imports
from seer_robot_interfaces.srv import PalletID

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

        # Service server
        self.create_service(PalletID,'task_management/pick_pallet_for_init', self.pick_pallet_for_init_callback)

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

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()