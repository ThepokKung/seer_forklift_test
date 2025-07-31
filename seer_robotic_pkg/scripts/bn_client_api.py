#!/usr/bin/python3

import socket
import json
from bn_msg_packer import MsgPacker

class ClientAPI:
    # def __init__(self, ip=None ):
    # def __init__(self, ip=None, port=None, api_number=None):
    def __init__(self, ip=None, port=None):
        self.sock = None
        self.robot_ip = ip
        self.robot_port = port
        # self.api_number = api_number

        # define the message packer
        self.packer = MsgPacker()

    def connect(self):
        if self.sock is not None:
            self.sock.close()
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)  # Set 5 second timeout
        try:
            self.sock.connect((self.robot_ip, self.robot_port))
            # print(f"Connected to {self.robot_ip}:{self.robot_port}")
            return True
        except (socket.timeout, socket.error) as e:
            print(f"Connection failed: {e}")
            self.sock.close()
            self.sock = None
            return False

    def disconnect(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
            # print("Disconnected")

    def send_request(self, req_id, msg_type, payload=None):
        if self.sock is None:
            print("Not connected")
            return None

        packed_message = self.packer.pack_message(req_id, msg_type, payload)
        # print(f"raw_data = {packed_message}")
        self.sock.sendall(packed_message)

    def receive_response(self):
        """
        Receive and parse response from the server.
        
        Returns:
            tuple: (req_id, msg_type, payload) or None if error
        """
        if not self.sock:
            return None
                
        try:
            # Receive header (16 bytes) first
            header_data = self.sock.recv(16)
            if len(header_data) < 16:
                return None
                
            # Parse header to get message length
            import struct
            try:
                start_byte, version, req_id, msg_len, msg_type, reserved = struct.unpack('!BBHLH6s', header_data)
            except struct.error:
                return None
                
            # Receive remaining data if any
            remaining_data = b''
            if msg_len > 0:
                remaining = msg_len
                while remaining > 0:
                    chunk = self.sock.recv(min(remaining, 1024))
                    if not chunk:
                        break
                    remaining_data += chunk
                    remaining -= len(chunk)
            
            # Combine header and payload for unpacking
            full_message = header_data + remaining_data
            
            try:
                # Use the MsgPacker's unpack_message method
                req_id, msg_type, payload = self.packer.unpack_message(full_message)
                return req_id, msg_type, payload
            except Exception as e:
                print(f"Error unpacking message: {e}")
                return None
        
        except socket.timeout:
            print("Socket timeout occurred")
            return None
        except socket.error as e:
            print(f"Socket error: {e}")
            return None
