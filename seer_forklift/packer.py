import json
import struct
from typing import Optional, Dict
from .exceptions import MessagePackError

PACK_FMT = '!BBHLH6s'

class MessagePacker:
    def __init__(self):
        self.header_fmt = PACK_FMT
        self._header_size = struct.calcsize(self.header_fmt)

    def header_size(self) -> int:
        return self._header_size

    # Packs a message with the given request ID, message type, and optional payload.
    def pack(
        self,
        req_id: int,
        msg_type: int,
        payload: Optional[Dict] = None,
    ) -> bytes:
        # Ensure payload is a dict
        body_dict = payload or {}
        body = json.dumps(body_dict)
        msg_len = len(body)
        try:
            header = struct.pack(
                self.header_fmt,
                0x5A,
                0x01,
                req_id,
                msg_len,
                msg_type,
                b'\x00' * 6,
            )
        except struct.error as e:
            raise MessagePackError(f"Header pack failed: {e}")
        return header + body.encode('ascii')

    # Unpacks the header from the given data.
    def unpack_header(self, data: bytes):
        if len(data) < self._header_size:
            raise MessagePackError("Insufficient data for header")
        return struct.unpack(self.header_fmt, data[:self._header_size])