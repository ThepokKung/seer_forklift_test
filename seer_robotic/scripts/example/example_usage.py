#!/usr/bin/env python3

"""
Example script showing how to use the robot API and message packer.
This replicates the functionality of your original code using the modular approach.
"""

import sys
import os

# Add the backend directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

from backend.api import RobotAPI

def main():
    """Main function to demonstrate the robot API usage."""
    
    # Configuration
    IP = '192.168.0.181'
    PORT = 19204
    API_NUMBER = 1004
    PAYLOAD = {}
    REQ_ID = 1
    
    print("Robot API Communication Example")
    print("="*50)
    
    # Create API instance
    api = RobotAPI(ip=IP, port=PORT, timeout=5)
    
    # Connect and send request
    if api.connect():
        try:
            print(f"\nSending API request:")
            print(f"- API Number: {API_NUMBER}")
            print(f"- Request ID: {REQ_ID}")
            print(f"- Payload: {PAYLOAD}")
            
            # Send the API request
            response = api.send_api_request(
                api_number=API_NUMBER,
                payload=PAYLOAD,
                req_id=REQ_ID
            )
            
            if response:
                print(f"\n✓ API call successful!")
                print(f"Header: {response['header']}")
                print(f"Data: {response['data']}")
            else:
                print(f"\n✗ API call failed!")
                
        except KeyboardInterrupt:
            print("\n\nOperation cancelled by user")
        except Exception as e:
            print(f"\n✗ Error occurred: {e}")
        finally:
            api.disconnect()
    else:
        print(f"✗ Failed to connect to {IP}:{PORT}")

def test_with_context_manager():
    """Example using context manager for automatic connection handling."""
    
    print("\n" + "="*50)
    print("Using Context Manager Example")
    print("="*50)
    
    try:
        with RobotAPI(ip='192.168.0.180', port=19204) as api:
            response = api.send_api_request(
                api_number=1004,
                payload={"test": "data"},
                req_id=2
            )
            
            if response:
                print("✓ Context manager API call successful!")
            else:
                print("✗ Context manager API call failed!")
                
    except ConnectionError as e:
        print(f"✗ Connection error: {e}")
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    main()
    # Uncomment the line below to test the context manager approach
    # test_with_context_manager()
