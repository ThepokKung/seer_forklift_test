#!/usr/bin/python3

import sys
import os

from seer_robot_pkg.client_api import ClientAPI

class RobotStatusAPI:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.robot_port = 19204  # Default port for robot status API
        self.client = ClientAPI(ip=robot_ip, port=self.robot_port)
        # Don't auto-connect during initialization to avoid blocking
        self.connected = False
    
    def connect(self):
        """Connect to the robot"""
        try:
            if self.client.connect():
                self.connected = True
                return True
            else:
                self.connected = False
                return False
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the robot"""
        if self.connected:
            self.client.disconnect()
            self.connected = False

    def get_battery_status(self):
        """
        Request the battery status from the robot.
        
        Returns:
            float: Battery level (0.0-1.0) or None if error.
        """
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None
            
        req_id = 1
        msg_type = 1007  # Correct message type for battery data
        self.client.send_request(req_id, msg_type, payload={})
        temp = self.client.receive_response()
        if temp is None:
            return None
        
        req_id, msg_type, data = temp

        # Extract battery data from response
        battery_data = {
            'battery_level': data.get('battery_level'),
            'charging': data.get('charging'),
            'auto_charge': data.get('auto_charge'),
            'manual_charge': data.get('manual_charge'),
            'battery_cycle': data.get('battery_cycle'),
            'battery_temp': data.get('battery_temp'),
            'current': data.get('current'),
            'voltage': data.get('voltage'),
            'max_charge_current': data.get('max_charge_current'),
            'max_charge_voltage': data.get('max_charge_voltage'),
            'battery_user_data': data.get('battery_user_data'),
            'create_on': data.get('create_on')
        }
        
        # Return the battery level if request was successful
        if data.get('ret_code') == 0:
            return battery_data
        else:
            return None

    def get_position_status(self):
        """
        Request the position status from the robot.

        Returns:
            tuple: (x, y, angle, current_position, last_station, confidence) or None if error.
        """
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None
            
        req_id = 1
        msg_type = 1004  # Correct message type for position data
        self.client.send_request(req_id, msg_type, payload={})
        temp = self.client.receive_response()
        if temp is None:
            return None
        
        req_id, msg_type, data = temp
        
        # Return position data if request was successful
        if data.get('ret_code') == 0:
            x = data.get('x')
            y = data.get('y')
            angle = data.get('angle')
            current_position = data.get('current_station')
            last_station = data.get('last_station')
            confidence = data.get('confidence')
            return x, y, angle, current_position, last_station, confidence
        else:
            return None

    def get_navigation_status(self):
        """
        Request the navigation status from the robot.

        Returns:
            dict: Navigation status data or None if error.
        """
        if not self.connected:
            print("Not connected to robot. Call connect() first.")
            return None
            
        req_id = 1
        msg_type = 1020
        self.client.send_request(req_id, msg_type, payload={})
        temp = self.client.receive_response()
        if temp is None:
            return None

        req_id, msg_type, data = temp

        # Return navigation data if request was successful
        if data.get('ret_code') == 0:
            return {
            'task_status': data.get('task_status'),
            'task_type': data.get('task_type'),
            'target_id': data.get('target_id'),
            'finished_path': data.get('finished_path'),
            'unfinished_path': data.get('unfinished_path'),
            'containers': data.get('containers')
            }
        else:
            return None
