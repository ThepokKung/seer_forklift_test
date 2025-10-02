#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import uvicorn
import json
import time
from typing import Dict, Any, Optional
from contextlib import suppress
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from std_msgs.msg import String, Float32, Bool, Int32
from seer_robot_interfaces.msg import RobotBatchData
import asyncio
import os
from fastapi.responses import HTMLResponse

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
                'assign_task': '/robot_01/robot_controller/assign_task',
            },
            'robot_02': {
                'cancel': '/robot_02/robot_controller/cancel_navigation',
                'pause': '/robot_02/robot_controller/pause_navigation',
                'resume': '/robot_02/robot_controller/resume_navigation',
                'assign_task': '/robot_02/robot_controller/assign_task',
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
        
        # All available topics from RobotBatchData message
        self.available_topics = [
            "robot_battery_percentage",
            "robot_controller_mode_status", 
            "robot_current_station",
            "robot_navigation_status",
            "robot_fork_height",
            "robot_confidence",
            "robot_state"
        ]
        
        # Store latest messages for each topic per robot (for compatibility with existing UI)
        self.latest_status: Dict[str, Dict[str, Any]] = {ns: {topic: None for topic in self.available_topics} for ns in self.robot_namespaces}
        self.status_last_update: Dict[str, Dict[str, Optional[float]]] = {ns: {topic: None for topic in self.available_topics} for ns in self.robot_namespaces}
        self.status_online: Dict[str, Dict[str, bool]] = {ns: {topic: False for topic in self.available_topics} for ns in self.robot_namespaces}
        
        # Store latest batch data
        self.latest_batch_data: Dict[str, Any] = {ns: None for ns in self.robot_namespaces}
        self.batch_last_update: Dict[str, Optional[float]] = {ns: None for ns in self.robot_namespaces}  
        self.batch_online: Dict[str, bool] = {ns: False for ns in self.robot_namespaces}
        self.offline_timeout = 5.0  # seconds without updates before marking offline
        self.offline_check_interval = 1.0
        
        # Health check logging control
        self.health_check_count = 0
        self.health_check_logged = False
        # Store websocket connections
        self.active_connections = {ns: set() for ns in self.robot_namespaces}
        self._watchdog_task = None
        self._assign_task_clients = {}
        # Store the event loop for scheduling async tasks from ROS2 callbacks
        self.loop = None
        self.loop_ready = threading.Event()
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()
        # Wait for the event loop to be ready before continuing
        self.loop_ready.wait()
        # Subscribe to all topics for each robot namespace
        for ns in self.robot_namespaces:
            # Subscribe to the new integrated RobotBatchData message from robot_monitor
            batch_topic = f"/{ns}/robot_monitor/robot_batch_data"
            self.get_logger().info(f"Subscribing to robot batch data topic: {batch_topic}")
            self.create_subscription(
                RobotBatchData,
                batch_topic,
                self._make_batch_callback(ns),
                10
            )

    def _make_batch_callback(self, ns):
        """Create callback for RobotBatchData messages"""
        def callback(msg):
            # Store the batch data and update individual topic values
            self.latest_batch_data[ns] = msg
            self.batch_last_update[ns] = time.time()
            was_offline = not self.batch_online[ns]
            self.batch_online[ns] = True
            
            # Extract individual values from batch data and update status
            batch_values = {
                "robot_battery_percentage": msg.robot_battery_percentage,
                "robot_controller_mode_status": msg.robot_controller_mode_status,
                "robot_current_station": msg.robot_current_station,
                "robot_navigation_status": msg.robot_navigation_status,
                "robot_fork_height": msg.robot_fork_height,
                "robot_confidence": msg.robot_confidence,
                "robot_state": msg.robot_state,
            }
            
            # Update individual topic storage for compatibility
            for topic, value in batch_values.items():
                if topic in self.available_topics:
                    self.latest_status[ns][topic] = value
                    self.status_last_update[ns][topic] = time.time()
                    self.status_online[ns][topic] = True
            
            if was_offline:
                self.get_logger().debug(f"{ns}:batch_data stream resumed")
            
            if self.loop is None or not self.loop.is_running():
                self.get_logger().warning("Event loop not running; skipping websocket broadcast")
                return
            
            try:
                # Broadcast all updated topics
                for topic, value in batch_values.items():
                    if topic in self.available_topics:
                        asyncio.run_coroutine_threadsafe(
                            self._broadcast_status(ns, topic, value, online=True),
                            self.loop
                        )
            except Exception as exc:
                self.get_logger().error(f"Failed to schedule websocket broadcast from batch data: {exc}")
        return callback

    async def _broadcast_status(self, ns, topic, value, online=True):
        # Send update to all connected websocket clients for this robot
        payload_value = str(value) if value is not None else None
        data = {"topic": topic, "value": payload_value, "online": online}
        to_remove = set()
        for ws in list(self.active_connections[ns]):
            try:
                await ws.send_text(json.dumps(data))
            except Exception:
                to_remove.add(ws)
        self.active_connections[ns] -= to_remove

    async def _status_watchdog(self):
        while True:
            await asyncio.sleep(self.offline_check_interval)
            now = time.time()
            
            # Check batch data timeouts
            for ns in self.robot_namespaces:
                if self.batch_online[ns]:
                    last_update = self.batch_last_update[ns]
                    if last_update is not None and now - last_update > self.offline_timeout:
                        self.batch_online[ns] = False
                        self.latest_batch_data[ns] = None
                        self.get_logger().warning(
                            f"No batch updates from {ns} for {self.offline_timeout} seconds; marking offline"
                        )
                        # Mark all individual topics as offline too
                        for topic in self.available_topics:
                            if self.status_online[ns][topic]:
                                self.status_online[ns][topic] = False
                                self.latest_status[ns][topic] = None
                                await self._broadcast_status(ns, topic, None, online=False)
                
                # Check individual topic timeouts (for backward compatibility)
                for topic in self.available_topics:
                    if not self.status_online[ns][topic]:
                        continue
                    last_update = self.status_last_update[ns][topic]
                    if last_update is None:
                        continue
                    if now - last_update > self.offline_timeout:
                        self.status_online[ns][topic] = False
                        self.latest_status[ns][topic] = None
                        self.get_logger().warning(
                            f"No updates from {ns}:{topic} for {self.offline_timeout} seconds; marking offline"
                        )
                        await self._broadcast_status(ns, topic, None, online=False)

    def _robot_connectivity_state(self, robot_id):
        """Return 'online', 'offline', or 'unknown' based on recent status topics."""
        # Check batch data first (preferred)
        if self.batch_online.get(robot_id, False):
            return "online"
        
        # Fall back to individual topics
        topics = self.status_online.get(robot_id)
        if not topics:
            return "unknown"
        if any(topics.values()):
            return "online"
        
        # Check if we have any historical data
        batch_last_update = self.batch_last_update.get(robot_id)
        individual_updates = self.status_last_update.get(robot_id, {})
        
        if batch_last_update is not None or any(ts is not None for ts in individual_updates.values()):
            return "offline"
        return "unknown"

    
    def setup_routes(self):
        node_instance = self
        @self.app.get("/ui", response_class=HTMLResponse)
        async def serve_ui():
            html_path = os.path.join(os.path.dirname(__file__), "ros_bridge_ui.html")
            try:
                with open(html_path, "r", encoding="utf-8") as f:
                    html_content = f.read()
                return HTMLResponse(content=html_content, status_code=200)
            except Exception as e:
                return HTMLResponse(content=f"<h1>UI Not Found</h1><p>{e}</p>", status_code=404)

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
                    await node_instance._watchdog_task # type: ignore
                node_instance._watchdog_task = None

        @self.app.get("/")
        async def root():
            return {"message": "ROS2 FastAPI Bridge is running"}

        @self.app.get("/health")
        async def health_check():
            try:
                # Increment success counter
                node_instance.health_check_count += 1
                
                # Log first success immediately, then every 30 successes
                if node_instance.health_check_count == 1:
                    node_instance.get_logger().info('127.0.0.1 - "GET /health HTTP/1.1" 200 OK')
                elif node_instance.health_check_count >= 30 and not node_instance.health_check_logged:
                    node_instance.get_logger().info('127.0.0.1 - "GET /health HTTP/1.1" 200 OK')
                    node_instance.health_check_logged = True
                
                # Reset counter and flag for next logging cycle after 30 attempts
                if node_instance.health_check_count >= 30:
                    node_instance.health_check_count = 0
                    node_instance.health_check_logged = False
                
                return {"status": "healthy", "node": node_instance.get_name()}
            except Exception as e:
                # Reset counters on error and log immediately
                node_instance.health_check_count = 0
                node_instance.health_check_logged = False
                node_instance.get_logger().error(f'127.0.0.1 - "GET /health HTTP/1.1" 500 ERROR: {str(e)}')
                raise e

        @self.app.post("/assigntask/{task_type_id}/{pallet_id}/{task_id}")
        async def call_assign_task_post(task_type_id: int, pallet_id: int, task_id: str):
            if task_type_id not in [1, 2, 3, 4]:
                return {"error": "task_type_id must be 1, 2, 3, or 4 (PickInit, PlaceInit, PickToManipulator, PickFromManipulator)"}
            result = await node_instance.call_assign_task_service_async(task_type_id, pallet_id, task_id)
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

        @self.app.post("/robot/{robot_id}/assign_task/{task_type_id}/{pallet_id}/{task_id}")
        async def assign_task_robot(robot_id: str, task_type_id: int, pallet_id: int, task_id: str):
            if robot_id not in node_instance.robot_namespaces:
                return {"success": False, "error": f"Unknown robot_id: {robot_id}"}
            if task_type_id not in [1, 2, 3, 4]:
                return {"success": False, "error": "task_type_id must be 1, 2, 3, or 4 (PickInit, PlaceInit, PickToManipulator, PickFromManipulator)"}
            return await node_instance._call_robot_assign_task(robot_id, task_type_id, pallet_id, task_id)

        @self.app.get("/robot/{robot_id}/status")
        async def robot_status(robot_id: str):
            if robot_id not in node_instance.robot_namespaces:
                return {"success": False, "error": f"Unknown robot_id: {robot_id}"}
            state = node_instance._robot_connectivity_state(robot_id)
            
            # Include both batch data and individual topics
            batch_data = node_instance.latest_batch_data.get(robot_id)
            batch_info = None
            if batch_data:
                batch_info = {
                    "robot_battery_percentage": batch_data.robot_battery_percentage,
                    "robot_controller_mode_status": batch_data.robot_controller_mode_status,
                    "robot_current_station": batch_data.robot_current_station,
                    "robot_navigation_status": batch_data.robot_navigation_status,
                    "robot_fork_height": batch_data.robot_fork_height,
                    "robot_confidence": batch_data.robot_confidence,
                    "robot_state": batch_data.robot_state,
                    "last_update": node_instance.batch_last_update.get(robot_id),
                    "online": node_instance.batch_online.get(robot_id, False)
                }
            
            return {
                "success": True,
                "robot_id": robot_id,
                "state": state,
                "batch_data": batch_info,
                "topics": {
                    topic: {
                        "online": node_instance.status_online[robot_id][topic],
                        "last_update": node_instance.status_last_update[robot_id][topic],
                        "value": node_instance.latest_status[robot_id][topic],
                    }
                    for topic in node_instance.available_topics
                },
            }

        @self.app.websocket("/ws/{robot_id}")
        async def websocket_endpoint(websocket: WebSocket, robot_id: str):
            if robot_id not in node_instance.robot_namespaces:
                await websocket.close(code=1008)
                return
            await websocket.accept()
            node_instance.active_connections[robot_id].add(websocket)
            try:
                # On connect, send the latest status for all topics
                for topic in node_instance.available_topics:
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
        if robot_id not in self.controller_services:
            return {"success": False, "error": f"Unknown robot_id: {robot_id}"}
        connectivity_state = self._robot_connectivity_state(robot_id)
        if connectivity_state == "offline":
            return {
                "success": False,
                "error": f"Robot {robot_id} appears offline; no recent status messages"
            }
        service_name = self.controller_services[robot_id][action]
        
        # Run the blocking ROS call in a thread pool
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(None, self._blocking_controller_call, service_name)
        return result
    
    def _blocking_controller_call(self, service_name):
        from std_srvs.srv import Trigger
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
    
    async def _call_robot_assign_task(self, robot_id, task_type_id, pallet_id, task_id):
        if robot_id not in self.controller_services:
            return {"success": False, "error": f"Unknown robot_id: {robot_id}"}
        connectivity_state = self._robot_connectivity_state(robot_id)
        if connectivity_state == "offline":
            return {
                "success": False,
                "error": f"Robot {robot_id} appears offline; no recent status messages"
            }
        service_name = self.controller_services[robot_id].get('assign_task')
        if not service_name:
            return {"success": False, "error": "Assign task service not configured for this robot"}
        # Run the blocking ROS call in a thread pool
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(None, self._blocking_assign_task_call, robot_id, task_type_id, pallet_id, task_id)
        return result
    
    def _blocking_assign_task_call(self, robot_id, task_type_id, pallet_id, task_id):
        from seer_robot_interfaces.srv import AssignTask
        service_name = self.controller_services[robot_id].get('assign_task')
        if not service_name:
            return {"success": False, "error": "Assign task service not configured for this robot"}
        
        client = self._assign_task_clients.get(robot_id)
        if client is None:
            client = self.create_client(AssignTask, service_name)
            self._assign_task_clients[robot_id] = client

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Assign task service {service_name} not available for {robot_id}")
            return {"success": False, "error": f"Service {service_name} not available"}

        try:
            request = AssignTask.Request()
            request.task_type_id = int(task_type_id)
            request.pallet_id = int(pallet_id)
            request.task_id = str(task_id)
        except (TypeError, ValueError) as exc:
            return {"success": False, "error": f"Invalid request parameters: {exc}"}

        self.get_logger().info(
            f"Calling {service_name} with task_type_id={request.task_type_id}, pallet_id={request.pallet_id}, task_id='{request.task_id}'"
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.cancelled():
            self.get_logger().error(f"Assign task call to {service_name} cancelled")
            return {"success": False, "error": "Assign task request cancelled"}

        if future.done():
            exc = future.exception()
            if exc is not None:
                self.get_logger().error(f"Assign task service raised exception: {exc}")
                return {"success": False, "error": f"Service call failed: {exc}"}
            result = future.result()
            if result is not None and hasattr(result, 'success') and hasattr(result, 'message'):
                return {"success": result.success, "message": result.message}

        self.get_logger().error("Assign task service returned no result")
        return {"success": False, "error": "Service call failed or returned invalid result"}
                
    
    def start_server(self):
        # uvicorn.run blocks and manages its own shutdown, but the FastAPI
        # startup hook captures the running loop so ROS callbacks can still
        # schedule websocket broadcasts safely.
        # Disable access_log to prevent automatic logging, we'll handle health endpoint logging manually
        uvicorn.run(self.app, host="0.0.0.0", port=8000, log_level="info", access_log=False)

    async def call_assign_task_service_async(self, task_type_id: int, pallet_id: int, task_id: str):
        """Call the /assigntask service with the given data asynchronously"""
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(None, self.call_assign_task_service, task_type_id, pallet_id, task_id)
        return result

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
