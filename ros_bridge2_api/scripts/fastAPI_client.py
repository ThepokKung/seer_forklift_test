#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import fastapi
import uvicorn
import json
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped

class FastAPIClient(Node):
    def __init__(self):
        super().__init__('fastAPI_client')
        self.get_logger().info('FastAPI Client node started')
        
        # Initialize FastAPI app
        self.app = FastAPI(title="ROS2 Service Bridge", version="1.0.0")
        
        # Setup FastAPI routes
        self.setup_routes()
        
        # Start FastAPI server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()
        
    def setup_routes(self):
        # Capture self reference for use in route handlers
        node_instance = self
        
        @self.app.get("/")
        async def root():
            return {"message": "ROS2 FastAPI Bridge is running"}
            
        @self.app.get("/health")
        async def health_check():
            return {"status": "healthy", "node": node_instance.get_name()}
            
        @self.app.post("/assigntask/{task_type_id}/{pallet_id}/{task_id}")
        async def call_assign_task_post(task_type_id: int, pallet_id: int, task_id: str):
            if task_type_id not in [1, 2, 3, 4]:
                return {"error": "task_type_id must be 1, 2, 3, or 4 (PickInit, PlaceInit, PickToManipulator, PickFromManipulator)"}
            
            result = node_instance.call_assign_task_service(task_type_id, pallet_id, task_id)
            if result:
                return {"success": True, "result": result.success, "message": result.message}
            else:
                return {"success": False, "error": "Service call failed"}
                
    
    def start_server(self):
        uvicorn.run(self.app, host="127.0.0.1", port=8000, log_level="info")

    def call_assign_task_service(self, task_type_id: int, pallet_id: int, task_id: str):
        """Call the /assigntask service with the given data"""
        from seer_robot_interfaces.srv import AssignTask
        
        client = self.create_client(AssignTask, '/task_management/assign_task')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /assigntask not available')
            return None
        
        request = AssignTask.Request()
        request.task_type_id = task_type_id
        request.pallet_id = pallet_id
        request.task_id = task_id
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Service call successful: {future.result()}')
            return future.result()
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = FastAPIClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()