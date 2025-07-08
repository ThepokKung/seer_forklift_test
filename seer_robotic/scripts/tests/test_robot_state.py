#!/usr/bin/env python3

"""
Test script for robot state node.
This script tests the robot state functionality without requiring actual robot connections.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
import time
import sys

class RobotStateTestNode(Node):
    def __init__(self):
        super().__init__('robot_state_test')
        
        # Test data storage
        self.received_position = None
        self.received_battery = None
        self.received_voltage = None
        self.received_percentage = None
        
        # Subscribe to robot state topics
        self.position_subscription = self.create_subscription(
            PointStamped,
            '/robot_position',
            self.position_callback,
            10
        )
        
        self.battery_subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        self.voltage_subscription = self.create_subscription(
            Float32,
            '/battery_voltage',
            self.voltage_callback,
            10
        )
        
        self.percentage_subscription = self.create_subscription(
            Float32,
            '/battery_percentage',
            self.percentage_callback,
            10
        )
        
        # Test timer
        self.test_timer = self.create_timer(5.0, self.run_test)
        self.test_count = 0
        
        self.get_logger().info('Robot state test node started')
        self.get_logger().info('Waiting for robot state data...')
    
    def position_callback(self, msg):
        """Handle position updates."""
        self.received_position = msg
        self.get_logger().info(
            f'Position received: x={msg.point.x:.2f}, y={msg.point.y:.2f}, z={msg.point.z:.2f}'
        )
    
    def battery_callback(self, msg):
        """Handle battery state updates."""
        self.received_battery = msg
        percentage = msg.percentage * 100
        self.get_logger().info(
            f'Battery received: {msg.voltage:.1f}V, {percentage:.1f}%, {msg.current:.2f}A, {msg.temperature:.1f}°C'
        )
    
    def voltage_callback(self, msg):
        """Handle voltage updates."""
        self.received_voltage = msg
        self.get_logger().debug(f'Voltage: {msg.data:.1f}V')
    
    def percentage_callback(self, msg):
        """Handle percentage updates."""
        self.received_percentage = msg
        self.get_logger().debug(f'Percentage: {msg.data:.1f}%')
    
    def run_test(self):
        """Run periodic test checks."""
        self.test_count += 1
        
        self.get_logger().info(f'=== Test Check #{self.test_count} ===')
        
        # Check position data
        if self.received_position:
            self.get_logger().info('✓ Position data received')
        else:
            self.get_logger().warning('✗ No position data received')
        
        # Check battery data
        if self.received_battery:
            self.get_logger().info('✓ Battery data received')
        else:
            self.get_logger().warning('✗ No battery data received')
        
        # Check individual values
        if self.received_voltage:
            self.get_logger().info('✓ Voltage data received')
        else:
            self.get_logger().warning('✗ No voltage data received')
        
        if self.received_percentage:
            self.get_logger().info('✓ Percentage data received')
        else:
            self.get_logger().warning('✗ No percentage data received')
        
        # Summary
        data_count = sum([
            1 if self.received_position else 0,
            1 if self.received_battery else 0,
            1 if self.received_voltage else 0,
            1 if self.received_percentage else 0
        ])
        
        self.get_logger().info(f'Data streams active: {data_count}/4')
        
        if data_count == 4:
            self.get_logger().info('✓ All robot state data streams working!')
        elif data_count > 0:
            self.get_logger().warning(f'⚠ Partial data received ({data_count}/4 streams)')
        else:
            self.get_logger().error('✗ No robot state data received')
        
        # Stop test after 10 iterations
        if self.test_count >= 10:
            self.get_logger().info('Test completed after 10 checks')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotStateTestNode()
        
        print("Robot State Test Node")
        print("=" * 50)
        print("This node will monitor robot state topics and report status.")
        print("Make sure the robot_state node is running first:")
        print("  ros2 launch seer_robotic robot_state.launch.py")
        print("")
        print("Press Ctrl+C to stop the test")
        print("=" * 50)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
