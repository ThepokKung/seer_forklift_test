#!/usr/bin/env python3

import struct
import json
from .logger import get_logger

class MessagePacker:
    """
    Message packer for robot communication protocol.
    """
    
    # Protocol constants
    PACK_FMT_STR = '!BBHLH6s'
    START_BYTE = 0x5A
    VERSION = 0x01
    RESERVED = b'\x00\x00\x00\x00\x00\x00'
    
    def __init__(self):
        """Initialize the MessagePacker."""
        self.logger = get_logger("message_packer")
    
    def pack_message(self, req_id, msg_type, msg={}):
        """
        Pack a message according to the robot communication protocol.
        
        Args:
            req_id (int): Request ID
            msg_type (int): Message type/API number
            msg (dict): Message payload
            
        Returns:
            bytes: Packed message ready to send
        """
        # Calculate message length
        msg_len = 0
        json_str = ""
        
        if msg:
            json_str = json.dumps(msg)
            msg_len = len(json_str)
        
        # Pack header
        raw_msg = struct.pack(
            self.PACK_FMT_STR,
            self.START_BYTE,
            self.VERSION,
            req_id,
            msg_len,
            msg_type,
            self.RESERVED
        )
        
        # Debug log header
        self.logger.debug(f"Packing message - Start: {self.START_BYTE:02X}, Version: {self.VERSION:02X}, "
                          f"ReqID: {req_id:04X}, Length: {msg_len:08X}, Type: {msg_type:04X}")
        
        # Add JSON payload if present
        if msg:
            raw_msg += bytearray(json_str, 'ascii')
            self.logger.debug(f"Payload: {msg}")
        
        return raw_msg
    
    def unpack_header(self, header_data):
        """
        Unpack message header.
        
        Args:
            header_data (bytes): 16-byte header data
            
        Returns:
            dict: Unpacked header information or None if invalid
        """
        if len(header_data) != 16:
            self.logger.error(f"Invalid header length: {len(header_data)} (expected 16)")
            return None
        
        try:
            header = struct.unpack(self.PACK_FMT_STR, header_data)
            
            return {
                'start_byte': header[0],
                'version': header[1],
                'req_id': header[2],
                'msg_length': header[3],
                'msg_type': header[4],
                'reserved': header[5]
            }
        except struct.error as e:
            self.logger.log_error(e, "Header unpack error")
            return None
    
    def validate_header(self, header):
        """
        Validate header values.
        
        Args:
            header (dict): Header dictionary
            
        Returns:
            bool: True if valid, False otherwise
        """
        if not header:
            return False
        
        if header['start_byte'] != self.START_BYTE:
            self.logger.error(f"Invalid start byte: {header['start_byte']:02X} (expected {self.START_BYTE:02X})")
            return False
        
        if header['version'] != self.VERSION:
            self.logger.error(f"Invalid version: {header['version']:02X} (expected {self.VERSION:02X})")
            return False
        
        return True