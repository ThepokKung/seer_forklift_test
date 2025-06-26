import socket, json
import logging
from typing import Optional, Dict
from .packer import MessagePacker
from .exceptions import ConnectionError, TimeoutError
import os

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s %(levelname)s [%(name)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    filename=os.path.join('logs', 'seer_forklift.log'),  # Log to logs folder
    filemode='a'                   # Optional: append mode
)

# Create logs directory if it doesn't exist
os.makedirs('logs', exist_ok=True)

class SeerClient:
    def __init__(self, ip: str, port: int, timeout: float):
        # Initialize the SeerClient with IP, port, and timeout
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.packer = MessagePacker()
        self.logger = logging.getLogger(self.__class__.__name__)

    def connect(self):
        # Create a socket connection to the Seer server
        try:
            self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        except socket.timeout as e:
            raise TimeoutError(e)
        self.logger.debug(f"Connected to {self.ip}:{self.port}") 

    def close(self):
        if self.sock:
            self.sock.close()
            self.logger.debug("Connection closed")

    def send_request(self, req_id: int, msg_type: int, payload: Optional[Dict] = None) -> Dict:
        if self.sock is None:
            raise ConnectionError("Not connected")
        body_dict = payload or {}
        raw = self.packer.pack(req_id, msg_type, body_dict)
        self.sock.sendall(raw)
        hdr_bytes = self.sock.recv(self.packer.header_size())
        _, _, _, body_len, _, _ = self.packer.unpack_header(hdr_bytes)
        data = b''
        while len(data) < body_len:
            chunk = self.sock.recv(body_len - len(data))
            if not chunk:
                raise ConnectionError("Socket closed prematurely")
            data += chunk
        text = data.decode('ascii')
        return json.loads(text) if text else {}

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()