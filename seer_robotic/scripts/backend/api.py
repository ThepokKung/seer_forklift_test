#!/usr/bin/env python3

import socket
import json
import time
import struct
from .packker import MessagePacker
from .config_manager import ConfigManager
from .logger import get_logger

class RobotAPI:
    """
    Robot API client for communicating with the robot server.
    """
    
    def __init__(self, robot_key=None, api_name=None, ip=None, port=None, timeout=None):
        """
        Initialize the Robot API client.
        
        Args:
            robot_key (str): Robot key from config (e.g., 'robot_01'). If None, uses default.
            api_name (str): API name from config (e.g., 'robot_status_api'). If None, uses default.
            ip (str): Server IP address. If None, uses config or robot_key.
            port (int): Server port number. If None, uses config or api_name.
            timeout (int): Socket timeout in seconds. If None, uses config.
        """
        # Load configuration
        self.config = ConfigManager()
        
        # Determine IP address
        if ip is not None:
            self.ip = ip
        elif robot_key is not None:
            self.ip = self.config.get_robot_ip(robot_key)
        else:
            self.ip = self.config.get_robot_ip()  # Uses default robot
        
        # Determine port
        if port is not None:
            self.port = port
        elif api_name is not None:
            self.port = self.config.get_api_port(api_name)
        else:
            self.port = self.config.get_api_port()  # Uses default API
        
        # Determine timeout
        if timeout is not None:
            self.timeout = timeout
        else:
            self.timeout = self.config.get_network_timeout()
        
        # Store configuration references
        self.robot_key = robot_key
        self.api_name = api_name
        
        # Initialize connection
        self.socket = None
        self.packer = MessagePacker()
        
        # Initialize logger
        self.logger = get_logger(f"robot_api_{self.ip}_{self.port}")
        
        # Log configuration info
        self.logger.info(f"Robot API initialized - IP: {self.ip}, Port: {self.port}, Timeout: {self.timeout}s")
        if robot_key:
            robot_name = self.config.get_robot_name(robot_key)
            robot_id = self.config.get_robot_id(robot_key)
            self.logger.info(f"Robot: {robot_name} (ID: {robot_id})")
        if api_name:
            api_info = self.config.get_api_info(api_name)
            self.logger.info(f"API: {api_info.get('category', api_name)}")
    
    def connect(self):
        """
        Connect to the robot server.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ip, self.port))
            self.socket.settimeout(self.timeout)
            self.logger.log_connection_info(self.ip, self.port, "established")
            return True
        except Exception as e:
            self.logger.log_error(e, "Connection failed")
            return False
    
    def disconnect(self):
        """
        Disconnect from the robot server.
        """
        if self.socket:
            self.socket.close()
            self.socket = None
            self.logger.log_connection_info(self.ip, self.port, "closed")
    
    def send_api_request(self, api_number, payload={}, req_id=1):
        """
        Send an API request to the robot server.
        
        Args:
            api_number (int): API number/message type
            payload (dict): Request payload
            req_id (int): Request ID
            
        Returns:
            dict: Response data or None if failed
        """
        if not self.socket:
            self.logger.error("Not connected to server")
            return None
        
        try:
            # Pack the message
            packed_msg = self.packer.pack_message(req_id, api_number, payload)
            
            # Log the request
            self.logger.log_api_request(api_number, req_id, payload)
            self.logger.log_hex_data(packed_msg, "Sending request")
            
            # Send the message
            self.socket.send(packed_msg)
            
            # Receive response
            response = self._receive_response()
            return response
            
        except Exception as e:
            self.logger.log_error(e, "API request failed")
            return None
    
    def _receive_response(self):
        """
        Receive and parse response from the server.
        
        Returns:
            dict: Parsed response data
        """
        if not self.socket:
            self.logger.error("Socket not connected")
            return None
            
        try:
            # Receive header (16 bytes)
            header_data = self.socket.recv(16)
            if len(header_data) < 16:
                self.logger.error(f"Header receive error - got {len(header_data)} bytes, expected 16")
                self.logger.log_hex_data(header_data, "Incomplete header data")
                return None
            
            # Unpack header
            header = self.packer.unpack_header(header_data)
            if not header:
                return None
            
            # Log received header
            self.logger.log_hex_data(header_data, "Received header")
            
            # Receive JSON data if present
            json_data = {}
            data = b''
            if header['msg_length'] > 0:
                remaining = header['msg_length']
                read_size = 1024
                
                while remaining > 0:
                    if remaining < read_size:
                        read_size = remaining
                    
                    recv_data = self.socket.recv(read_size)
                    data += recv_data
                    remaining -= len(recv_data)
                
                # Parse JSON
                try:
                    json_data = json.loads(data.decode('ascii'))
                    self.logger.log_json_data(json_data, "Received JSON data")
                except json.JSONDecodeError as e:
                    self.logger.log_error(e, "JSON decode error")
                    return None
            
            # Log full response in hex
            full_response = header_data + data
            self.logger.log_hex_data(full_response, "Full response")
            
            # Log API response
            self.logger.log_api_response(header, json_data)
            
            return {
                'header': header,
                'data': json_data
            }
            
        except socket.timeout:
            self.logger.error('Response timeout')
            return None
        except Exception as e:
            self.logger.log_error(e, "Response receive error")
            return None
    
    def __enter__(self):
        """Context manager entry"""
        if self.connect():
            return self
        else:
            raise ConnectionError("Failed to connect to robot server")
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()

# Example usage
if __name__ == "__main__":
    # Example: Connect and send API request
    from .logger import get_default_logger
    logger = get_default_logger()
    
    api = RobotAPI(ip='192.168.0.180', port=19204)
    
    if api.connect():
        try:
            # Example API call
            response = api.send_api_request(
                api_number=1004,
                payload={},
                req_id=1
            )
            
            if response:
                logger.info("API call successful!")
                logger.debug(f"Response: {response}")
            else:
                logger.warning("API call failed!")
                
        finally:
            api.disconnect()
    else:
        logger.error("Failed to connect to robot server")
