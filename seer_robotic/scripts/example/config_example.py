#!/usr/bin/env python3

"""
Example script showing how to use the robot API with configuration management.
Demonstrates how to use robot and API configurations for flexible robot communication.
"""

import sys
import os

# Add the backend directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

from backend.api import RobotAPI
from backend.config_manager import ConfigManager

def example_basic_usage():
    """Basic usage example using default configuration."""
    print("=" * 60)
    print("Example 1: Basic Usage (Default Configuration)")
    print("=" * 60)
    
    # Create API instance with default settings
    api = RobotAPI()
    
    if api.connect():
        try:
            # Send a status request
            response = api.send_api_request(
                api_number=1004,
                payload={},
                req_id=1
            )
            
            if response:
                print("✓ API call successful!")
                print(f"Response: {response}")
            else:
                print("✗ API call failed!")
                
        finally:
            api.disconnect()
    else:
        print("✗ Connection failed!")

def example_specific_robot():
    """Example using a specific robot configuration."""
    print("\n" + "=" * 60)
    print("Example 2: Specific Robot Configuration")
    print("=" * 60)
    
    # Create API instance for robot_02
    api = RobotAPI(robot_key='robot_02')
    
    if api.connect():
        try:
            # Send a control command
            response = api.send_api_request(
                api_number=1005,
                payload={"command": "get_position"},
                req_id=2
            )
            
            if response:
                print("✓ Robot-specific API call successful!")
                print(f"Response: {response}")
            else:
                print("✗ Robot-specific API call failed!")
                
        finally:
            api.disconnect()
    else:
        print("✗ Connection failed!")

def example_specific_api():
    """Example using a specific API configuration."""
    print("\n" + "=" * 60)
    print("Example 3: Specific API Configuration")
    print("=" * 60)
    
    # Create API instance for control API
    api = RobotAPI(api_name='robot_control_api')
    
    if api.connect():
        try:
            # Send a control command
            response = api.send_api_request(
                api_number=2001,
                payload={"action": "move", "x": 100, "y": 200},
                req_id=3
            )
            
            if response:
                print("✓ Control API call successful!")
                print(f"Response: {response}")
            else:
                print("✗ Control API call failed!")
                
        finally:
            api.disconnect()
    else:
        print("✗ Connection failed!")

def example_robot_and_api():
    """Example using both specific robot and API configurations."""
    print("\n" + "=" * 60)
    print("Example 4: Specific Robot + API Configuration")
    print("=" * 60)
    
    # Create API instance for robot_02 using navigation API
    api = RobotAPI(robot_key='robot_02', api_name='robot_navigation_api')
    
    if api.connect():
        try:
            # Send a navigation command
            response = api.send_api_request(
                api_number=3001,
                payload={"destination": {"x": 500, "y": 300}, "speed": 1.5},
                req_id=4
            )
            
            if response:
                print("✓ Navigation API call successful!")
                print(f"Response: {response}")
            else:
                print("✗ Navigation API call failed!")
                
        finally:
            api.disconnect()
    else:
        print("✗ Connection failed!")

def example_context_manager():
    """Example using context manager with configuration."""
    print("\n" + "=" * 60)
    print("Example 5: Context Manager with Configuration")
    print("=" * 60)
    
    try:
        # Use context manager for automatic connection handling
        with RobotAPI(robot_key='robot_01', api_name='robot_status_api') as api:
            # Send multiple requests
            for i in range(3):
                response = api.send_api_request(
                    api_number=1004,
                    payload={"request_id": i},
                    req_id=i + 10
                )
                
                if response:
                    print(f"✓ Request {i+1} successful!")
                else:
                    print(f"✗ Request {i+1} failed!")
                    
    except ConnectionError as e:
        print(f"✗ Connection error: {e}")
    except Exception as e:
        print(f"✗ Error: {e}")

def example_multiple_robots():
    """Example communicating with multiple robots."""
    print("\n" + "=" * 60)
    print("Example 6: Multiple Robots Communication")
    print("=" * 60)
    
    config = ConfigManager()
    
    for robot_key in config.list_robots():
        print(f"\nCommunicating with {robot_key}:")
        print("-" * 30)
        
        api = RobotAPI(robot_key=robot_key, api_name='robot_status_api')
        
        if api.connect():
            try:
                response = api.send_api_request(
                    api_number=1004,
                    payload={"query": "status"},
                    req_id=100 + config.get_robot_id(robot_key)
                )
                
                if response:
                    print(f"✓ {config.get_robot_name(robot_key)} responded successfully!")
                else:
                    print(f"✗ {config.get_robot_name(robot_key)} failed to respond!")
                    
            finally:
                api.disconnect()
        else:
            print(f"✗ Failed to connect to {config.get_robot_name(robot_key)}")

def show_configuration():
    """Display current configuration."""
    print("\n" + "=" * 60)
    print("Current Configuration")
    print("=" * 60)
    
    config = ConfigManager()
    config.print_config_summary()

def main():
    """Main function to run all examples."""
    print("Robot API Configuration Examples")
    print("=" * 60)
    
    # Show configuration first
    show_configuration()
    
    # Run examples
    try:
        example_basic_usage()
        example_specific_robot()
        example_specific_api()
        example_robot_and_api()
        example_context_manager()
        example_multiple_robots()
        
        print("\n" + "=" * 60)
        print("All examples completed!")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user")
    except Exception as e:
        print(f"\nError occurred: {e}")

if __name__ == "__main__":
    main()
