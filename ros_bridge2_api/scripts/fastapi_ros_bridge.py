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
from fastapi.middleware.cors import CORSMiddleware
from std_msgs.msg import String, Float32, Bool, Int32
from geometry_msgs.msg import PointStamped

class FastAPIClient(Node):
    def __init__(self):
        super().__init__('fastAPI_client')
        self.get_logger().info('FastAPI Client node started')
        # Service names for each robot
        self.controller_services = {
            'robot_01': {
                'cancel': '/robot_01/robot_controller/cancel_navigation',
                'pause': '/robot_01/robot_controller/pause_navigation',
                'resume': '/robot_01/robot_controller/resume_navigation',
            },
            'robot_02': {
                'cancel': '/robot_02/robot_controller/cancel_navigation',
                'pause': '/robot_02/robot_controller/pause_navigation',
                'resume': '/robot_02/robot_controller/resume_navigation',
            },
        }
        # Initialize FastAPI app
        self.app = FastAPI(title="ROS2 Service Bridge", version="1.0.0")
        # Add CORS middleware to allow requests from any origin
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        # Setup FastAPI routes
        self.setup_routes()
        # Store the event loop for scheduling async tasks from ROS2 callbacks
        self.loop = None
        self.loop_ready = threading.Event()
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()
        # Wait for the event loop to be ready before continuing
        self.loop_ready.wait()
        # --- Webhook/Streaming additions ---
        self.robot_namespaces = ["robot_01", "robot_02"]
        self.status_topics = [
            "robot_battery_percentage",
            "robot_controller_mode_status",
            "robot_current_station",
            "robot_navigation_status"
        ]
        # Map topic to correct ROS2 message type
        self.topic_types = {
            "robot_battery_percentage": Float32,
            "robot_controller_mode_status": Bool,
            "robot_current_station": String,
            "robot_navigation_status": Int32,
        }
        # Store latest messages for each topic per robot
        self.latest_status = {ns: {topic: None for topic in self.status_topics} for ns in self.robot_namespaces}
        # Store websocket connections
        self.active_connections = {ns: set() for ns in self.robot_namespaces}
        # Subscribe to all topics for each robot namespace
        for ns in self.robot_namespaces:
            for topic in self.status_topics:
                full_topic = f"/{ns}/robot_status/{topic}"
                msg_type = self.topic_types[topic]
                self.create_subscription(
                    msg_type,
                    full_topic,
                    self._make_callback(ns, topic),
                    10
                )

    def _make_callback(self, ns, topic):
        def callback(msg):
            # Store latest message as string for UI compatibility
            value = msg.data if hasattr(msg, 'data') else msg
            self.latest_status[ns][topic] = value
            # Always send as string for UI
            try:
                asyncio.run(self._broadcast_status(ns, topic, str(value)))
            except RuntimeError:
                # Already in an event loop, ignore (should not happen in ROS2 callback)
                pass
        return callback

    async def _broadcast_status(self, ns, topic, value):
        # Send update to all connected websocket clients for this robot
        data = {"topic": topic, "value": value}
        to_remove = set()
        for ws in self.active_connections[ns]:
            try:
                await ws.send_text(json.dumps(data))
            except Exception:
                to_remove.add(ws)
        self.active_connections[ns] -= to_remove
        

    def setup_routes(self):
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


        @self.app.post("/robot/{robot_id}/cancel_navigation")
        async def cancel_navigation(robot_id: str):
            return await node_instance._call_controller_service(robot_id, 'cancel')

        @self.app.post("/robot/{robot_id}/pause_navigation")
        async def pause_navigation(robot_id: str):
            return await node_instance._call_controller_service(robot_id, 'pause')

        @self.app.post("/robot/{robot_id}/resume_navigation")
        async def resume_navigation(robot_id: str):
            return await node_instance._call_controller_service(robot_id, 'resume')

        @self.app.websocket("/ws/{robot_id}")
        async def websocket_endpoint(websocket: WebSocket, robot_id: str):
            if robot_id not in node_instance.robot_namespaces:
                await websocket.close(code=1008)
                return
            await websocket.accept()
            node_instance.active_connections[robot_id].add(websocket)
            try:
                # On connect, send the latest status for all topics
                for topic in node_instance.status_topics:
                    value = node_instance.latest_status[robot_id][topic]
                    if value is not None:
                        await websocket.send_text(json.dumps({"topic": topic, "value": value}))
                while True:
                    # Keep connection open, but we don't expect to receive messages
                    await websocket.receive_text()
            except WebSocketDisconnect:
                pass
            finally:
                node_instance.active_connections[robot_id].discard(websocket)


    async def _call_controller_service(self, robot_id, action):
        # Call the appropriate ROS2 service for cancel/pause/resume navigation
        from std_srvs.srv import Trigger
        if robot_id not in self.controller_services:
            return {"success": False, "error": f"Unknown robot_id: {robot_id}"}
        service_name = self.controller_services[robot_id][action]
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            return {"success": False, "error": f"Service {service_name} not available"}
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None and hasattr(result, 'success') and hasattr(result, 'message'):
            return {"success": result.success, "message": result.message}
        else:
            return {"success": False, "error": "Service call failed or returned invalid result"}
                
    
    def start_server(self):
        import asyncio
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop_ready.set()
        uvicorn.run(self.app, host="0.0.0.0", port=8000, log_level="info")

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