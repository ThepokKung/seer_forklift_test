#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from seer_robot_interfaces.srv import AssignTask

class TestAssignTask(Node):
    def __init__(self):
        super().__init__('test_assign_task')
        self.client = self.create_client(AssignTask, 'robot_controller/assign_task')
        
    def send_request(self, task_type_id, pallet_id, task_id):
        """
        Send an assign task request
        
        Args:
            task_type_id (int): 1=PickInit, 2=PlaceInit, 3=PickToManipulator, 4=PickFromManipulator
            pallet_id (int): The pallet ID to work with
            task_id (str): Unique task identifier
        """
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Create request
        request = AssignTask.Request()
        request.task_type_id = task_type_id
        request.pallet_id = pallet_id
        request.task_id = task_id
        
        # Send request
        self.get_logger().info(f'Sending request: task_type_id={task_type_id}, pallet_id={pallet_id}, task_id={task_id}')
        future = self.client.call_async(request)
        
        return future

def main(args=None):
    rclpy.init(args=args)
    
    node = TestAssignTask()
    
    try:
        # Test different task types
        test_cases = [
            (1, 101, "task_pick_init_001"),      # PickInit
            (2, 102, "task_place_init_002"),     # PlaceInit  
            (3, 103, "task_pick_to_manip_003"),  # PickToManipulator
            (4, 104, "task_pick_from_manip_004") # PickFromManipulator
        ]
        
        for task_type_id, pallet_id, task_id in test_cases:
            future = node.send_request(task_type_id, pallet_id, task_id)
            
            # Wait for response
            rclpy.spin_until_future_complete(node, future)
            
            response = future.result()
            if response:
                if response.success:
                    node.get_logger().info(f'SUCCESS: {response.message}')
                else:
                    node.get_logger().error(f'FAILED: {response.message}')
            else:
                node.get_logger().error('Failed to call service')
            
            # Wait a bit between requests
            import time
            time.sleep(1)
            
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
