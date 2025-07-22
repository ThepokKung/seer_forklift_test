#!/usr/bin/python3

import sys
import os

from bn_client_api import ClientAPI

class RobotNavigationAPI:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.robot_port = 19206  # Default port for robot navigation API
        self.client = ClientAPI(ip=robot_ip, port=self.robot_port)
        # Don't auto-connect during initialization to avoid blocking
        self.connected = False
    
    def connect(self):
        """Connect to the robot"""
        try:
            self.client.connect()
            self.connected = True
            return True
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the robot"""
        if self.connected:
            self.client.disconnect()
            self.connected = False

    def get_navigation_path(self, id2go):
        """
        Request the navigation path from the robot.
        
        Args:
            id2go: ID of the navigation goal
        
        Returns:
            dict: Navigation path information or None if error.
        """
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None
            
        req_id = 1
        msg_type = 3053
        payload = {"id": id2go}
        self.client.send_request(req_id, msg_type, payload=payload)
        
        temp = self.client.receive_response()
        if temp is None:
            return None
            
        req_id, msg_type, data = temp
        
        # Return the full data if request was successful
        if data.get('ret_code') == 0:
            return data
        return None

    def navigation_to_goal(self, id, source_id, task_id, operation=None, start_height=None, end_height=None, recognize=None, **kwargs):
        """
        Send navigation goal to the robot.
        
        Args:
            id: Goal ID
            source_id: Source ID
            task_id: Task ID
            operation: Optional operation parameter
            start_height: Optional start height parameter
            end_height: Optional end height parameter
            recognize: Optional recognize parameter
            **kwargs: Additional optional parameters
        
        Returns:
            dict: Response from the robot or None if error.
        """
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None
            
        goal = {
            "id": id,
            "source_id": source_id,
            "task_id": task_id
        }
        
        # Add optional parameters if provided
        if operation is not None:
            goal["operation"] = operation
        if start_height is not None:
            goal["start_height"] = start_height
        if end_height is not None:
            goal["end_height"] = end_height
        if recognize is not None:
            goal["recognize"] = recognize
            
        # Add any additional parameters
        goal.update(kwargs)
        
        req_id = 1
        msg_type = 3051
        self.client.send_request(req_id, msg_type, payload=goal)
        temp = self.client.receive_response()
        if temp is None:
            return None

        req_id, msg_type, data = temp

        # Return the full data if request was successful
        if data.get('ret_code') == 0:
            return data

        return None

    def pause_navigation(self):
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None

        req_id = 1
        msg_type = 3001
        self.client.send_request(req_id, msg_type)
        temp = self.client.receive_response()
        if temp is None:
            return None

        req_id, msg_type, data = temp

        # Return the full data if request was successful
        if data.get('ret_code') == 0:
            return data

        return None

    def resume_navigation(self):
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None

        req_id = 1
        msg_type = 3002
        self.client.send_request(req_id, msg_type)
        temp = self.client.receive_response()
        if temp is None:
            return None

        req_id, msg_type, data = temp

        # Return the full data if request was successful
        if data.get('ret_code') == 0:
            return data

        return None
    
    def cancel_navigation(self):
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None

        req_id = 1
        msg_type = 3003
        self.client.send_request(req_id, msg_type)
        temp = self.client.receive_response()
        if temp is None:
            return None

        req_id, msg_type, data = temp

        # Return the full data if request was successful
        if data.get('ret_code') == 0:
            return data

        return None
    
    def designated_navigation(self,json_command):
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None

        req_id = 1
        msg_type = 3066
        self.client.send_request(req_id, msg_type, payload=json_command)
        temp = self.client.receive_response()
        if temp is None:
            return None

        req_id, msg_type, data = temp

        # Return the full data if request was successful
        if data.get('ret_code') == 0:
            return data

        return None