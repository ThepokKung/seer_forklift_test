#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import fastapi
import uvicorn
import json
import asyncio
import time
from contextlib import suppress
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
        self.status_last_update = {ns: {topic: None for topic in self.status_topics} for ns in self.robot_namespaces}
        self.status_online = {ns: {topic: False for topic in self.status_topics} for ns in self.robot_namespaces}
        self.offline_timeout = 5.0  # seconds without updates before marking offline
        self.offline_check_interval = 1.0
        # Store websocket connections
        self.active_connections = {ns: set() for ns in self.robot_namespaces}
        self._watchdog_task = None
        # Store the event loop for scheduling async tasks from ROS2 callbacks
        self.loop = None
        self.loop_ready = threading.Event()
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()
        # Wait for the event loop to be ready before continuing
        self.loop_ready.wait()
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
            # Store latest message for UI compatibility
            value = msg.data if hasattr(msg, 'data') else msg
            self.latest_status[ns][topic] = value
            self.status_last_update[ns][topic] = time.time()
            was_offline = not self.status_online[ns][topic]
            self.status_online[ns][topic] = True
            if was_offline:
                self.get_logger().debug(f"{ns}:{topic} stream resumed")
            if self.loop is None or not self.loop.is_running():
                self.get_logger().warn("Event loop not running; skipping websocket broadcast")
                return
            try:
                asyncio.run_coroutine_threadsafe(
                    self._broadcast_status(ns, topic, value, online=True),
                    self.loop
                )
            except Exception as exc:
                self.get_logger().error(f"Failed to schedule websocket broadcast: {exc}")
        return callback

    async def _broadcast_status(self, ns, topic, value, online=True):
        # Send update to all connected websocket clients for this robot
        payload_value = str(value) if value is not None else None
        data = {"topic": topic, "value": payload_value, "online": online}
        to_remove = set()
        for ws in self.active_connections[ns]:
            try:
                await ws.send_text(json.dumps(data))
            except Exception:
                to_remove.add(ws)
        self.active_connections[ns] -= to_remove

    async def _status_watchdog(self):
        while True:
            await asyncio.sleep(self.offline_check_interval)
            now = time.time()
            for ns in self.robot_namespaces:
                for topic in self.status_topics:
                    if not self.status_online[ns][topic]:
                        continue
                    last_update = self.status_last_update[ns][topic]
                    if last_update is None:
                        continue
                    if now - last_update > self.offline_timeout:
                        self.status_online[ns][topic] = False
                        self.latest_status[ns][topic] = None
                        self.get_logger().warn(
                            f"No updates from {ns}:{topic} for {self.offline_timeout} seconds; marking offline"
                        )
                        await self._broadcast_status(ns, topic, None, online=False)
        

    def setup_routes(self):
        node_instance = self

        @self.app.on_event("startup")
        async def on_startup():
            node_instance.loop = asyncio.get_running_loop()
            node_instance.loop_ready.set()
            if node_instance._watchdog_task is None or node_instance._watchdog_task.done():
                node_instance._watchdog_task = asyncio.create_task(node_instance._status_watchdog())

        @self.app.on_event("shutdown")
        async def on_shutdown():
            if node_instance._watchdog_task is not None:
                node_instance._watchdog_task.cancel()
                with suppress(asyncio.CancelledError):
                    await node_instance._watchdog_task
                node_instance._watchdog_task = None

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
                    last_update = node_instance.status_last_update[robot_id][topic]
                    online = node_instance.status_online[robot_id][topic]
                    value = node_instance.latest_status[robot_id][topic]
                    if last_update is None and not online:
                        continue
                    payload = {
                        "topic": topic,
                        "value": str(value) if value is not None else None,
                        "online": online,
                    }
                    await websocket.send_text(json.dumps(payload))
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
        config = uvicorn.Config(self.app, host="0.0.0.0", port=8000, log_level="info", loop="asyncio")
        server = uvicorn.Server(config)
        server.install_signal_handlers = False
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(server.serve())
        finally:
            pending = [task for task in asyncio.all_tasks(self.loop) if not task.done()]
            for task in pending:
                task.cancel()
            if pending:
                self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self.loop.close()

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
