#!/usr/bin/python3

import struct
import json

class MsgPacker:

    PACK_FMT_STR = '!BBHLH6s'
    START_BYTE = 0x5A
    VERSION = 0x01
    RESERVED = b'\x00\x00\x00\x00\x00\x00'

    def __init__(self):
        pass

    def pack_message(self, req_id, msg_type, payload={}):
        # Calculate message length
        msg_len = 0
        json_str = ""
        
        if payload:
            json_str = json.dumps(payload)
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

        if payload:
            raw_msg += bytearray(json_str, 'ascii')

        return raw_msg

    def unpack_message(self, raw_msg):
        if len(raw_msg) < struct.calcsize(self.PACK_FMT_STR):
            raise ValueError("Message too short to unpack")

        # Unpack header
        header = struct.unpack(self.PACK_FMT_STR, raw_msg[:struct.calcsize(self.PACK_FMT_STR)])
        start_byte, version, req_id, msg_len, msg_type, reserved = header

        if start_byte != self.START_BYTE:
            raise ValueError("Invalid start byte")
        
        if version != self.VERSION:
            raise ValueError("Unsupported version")

        # Extract payload
        payload = {}
        if msg_len > 0:
            json_str = raw_msg[struct.calcsize(self.PACK_FMT_STR):].decode('ascii')
            payload = json.loads(json_str)

        return req_id, msg_type, payload
    
    def unpack_header(self, header_data):
        """
        Unpack message header.
        
        Args:
            header_data (bytes): 16-byte header data
            
        Returns:
            dict: Unpacked header information or None if invalid
        """
        if len(header_data) != 16:
            return None
        
        header = struct.unpack(self.PACK_FMT_STR, header_data)
            
        return {
            'start_byte': header[0],
            'version': header[1],
            'req_id': header[2],
            'msg_length': header[3],
            'msg_type': header[4],
            'reserved': header[5]
        }
