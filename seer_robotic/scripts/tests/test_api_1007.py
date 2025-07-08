#!/usr/bin/env python3

import socket
import json
import struct
import sys
import time

def test_api_1007():
    """Test API 1007 (battery) on port 19204 for robot_02"""
    
    robot_ip = '192.168.0.181'
    api_port = 19204
    
    print(f"Testing API 1007 (battery) on {robot_ip}:{api_port}")
    
    try:
        # Create socket connection
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)  # 10 second timeout
        
        print("Connecting to robot...")
        sock.connect((robot_ip, api_port))
        print("Connection successful!")
        
        # Prepare API 1007 request using the correct protocol format
        # Based on packker.py: '!BBHLH6s'
        # START_BYTE (0x5A), VERSION (0x01), REQ_ID (2 bytes), MSG_LENGTH (4 bytes), MSG_TYPE (2 bytes), RESERVED (6 bytes)
        
        START_BYTE = 0x5A
        VERSION = 0x01
        REQ_ID = 0x0001  # Request ID
        MSG_LENGTH = 0  # No payload
        MSG_TYPE = 1007  # API 1007
        RESERVED = b'\x00\x00\x00\x00\x00\x00'
        
        # Pack the message using the same format as packker.py
        packet = struct.pack(
            '!BBHLH6s',
            START_BYTE,
            VERSION,
            REQ_ID,
            MSG_LENGTH,
            MSG_TYPE,
            RESERVED
        )
        
        print(f"Sending packet: {packet.hex()}")
        print(f"Packet breakdown:")
        print(f"  Start Byte: {START_BYTE:02X}")
        print(f"  Version: {VERSION:02X}")
        print(f"  Request ID: {REQ_ID:04X}")
        print(f"  Message Length: {MSG_LENGTH:08X}")
        print(f"  Message Type (API): {MSG_TYPE:04X}")
        print(f"  Reserved: {RESERVED.hex()}")
        
        # Send the request
        sock.send(packet)
        print("Request sent, waiting for response...")
        
        # Receive the response header first (16 bytes)
        header_response = sock.recv(16)
        print(f"Header response received: {header_response.hex()}")
        print(f"Header length: {len(header_response)} bytes")
        
        if len(header_response) >= 16:
            # Parse response header using the same format
            try:
                header = struct.unpack('!BBHLH6s', header_response)
                start_byte, version, req_id, msg_length, msg_type, reserved = header
                
                print(f"Response header parsed:")
                print(f"  Start Byte: {start_byte:02X}")
                print(f"  Version: {version:02X}")
                print(f"  Request ID: {req_id:04X}")
                print(f"  Message Length: {msg_length:08X}")
                print(f"  Message Type: {msg_type:04X}")
                print(f"  Reserved: {reserved.hex()}")
                
                if start_byte == 0x5A and version == 0x01:
                    print("Valid response header detected")
                    
                    # Receive payload if present
                    if msg_length > 0:
                        print(f"Receiving payload of {msg_length} bytes...")
                        payload_response = sock.recv(msg_length)
                        print(f"Payload received: {payload_response.hex()}")
                        
                        try:
                            # Try to parse as JSON
                            payload_str = payload_response.decode('ascii')
                            payload_data = json.loads(payload_str)
                            print(f"Payload JSON: {payload_data}")
                        except:
                            print("Could not parse payload as JSON")
                            
                            # Try to parse as raw binary data
                            if len(payload_response) >= 4:
                                try:
                                    # Try different interpretations
                                    float_val = struct.unpack('!f', payload_response[:4])[0]
                                    print(f"Payload as float: {float_val}")
                                except:
                                    pass
                                    
                                try:
                                    int_val = struct.unpack('!I', payload_response[:4])[0]
                                    print(f"Payload as int: {int_val}")
                                except:
                                    pass
                    else:
                        print("No payload in response")
                        
                    return True
                else:
                    print(f"Invalid response header: start={start_byte:02X}, version={version:02X}")
            except struct.error as e:
                print(f"Error parsing header: {e}")
        else:
            print("Response header too short")
        
        return False
        
    except socket.timeout:
        print("Connection timed out")
        return False
    except socket.error as e:
        print(f"Socket error: {e}")
        return False
    except Exception as e:
        print(f"Error during API call: {e}")
        return False
    finally:
        try:
            sock.close()
        except:
            pass

if __name__ == "__main__":
    print("Testing API 1007 (battery) on port 19204...")
    print("=" * 50)
    
    success = test_api_1007()
    
    print("=" * 50)
    if success:
        print("✓ API 1007 test completed successfully")
    else:
        print("✗ API 1007 test failed")
    
    sys.exit(0 if success else 1)
