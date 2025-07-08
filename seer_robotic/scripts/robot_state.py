#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String
import sys
import os
import threading
import time
from typing import Optional, Dict, Any

# Add the backend directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.join(current_dir, 'backend')

# Add paths for different installation scenarios
sys.path.insert(0, current_dir)
if os.path.exists(backend_dir):
    sys.path.insert(0, backend_dir)

from backend.api import RobotAPI
from backend.config_manager import ConfigManager
from backend.logger import get_logger

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state')
        
        # Initialize logger
        self.logger = get_logger(f"robot_state_{self.get_name()}")
        self.logger.info(f'Robot state node {self.get_name()} starting...')
        
        # Get robot configuration from parameters
        self.declare_parameter('robot_key', 'robot_01')
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('robot_namespace', '')
        
        self.robot_key = self.get_parameter('robot_key').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        
        # Initialize configuration
        self.config = ConfigManager()
        
        # Get robot information
        try:
            self.robot_info = self.config.get_robot_info(self.robot_key)
            self.robot_ip = self.robot_info['robot_ip']
            self.robot_name = self.robot_info['robot_name']
            self.robot_id = self.robot_info['robot_id']
            
            self.logger.info(f'Monitoring robot: {self.robot_name} (ID: {self.robot_id}, IP: {self.robot_ip})')
        except Exception as e:
            self.get_logger().error(f'Failed to get robot configuration: {e}')
            self.logger.error(f'Failed to get robot configuration: {e}')
            raise
        
        # API configuration
        self.api_port = self.config.get_api_port('robot_status_api')  # Port 19204
        self.position_api_number = 1004  # Current position API
        self.battery_api_number = 1007   # Battery API
        
        # Initialize publishers with namespace support
        topic_prefix = f'/{self.robot_namespace}' if self.robot_namespace else ''
        
        self.position_publisher = self.create_publisher(
            PointStamped, 
            f'{topic_prefix}/robot_position', 
            10
        )
        
        self.battery_publisher = self.create_publisher(
            BatteryState, 
            f'{topic_prefix}/battery_state', 
            10
        )
        
        # Additional publishers for individual values
        self.battery_voltage_publisher = self.create_publisher(
            Float32, 
            f'{topic_prefix}/battery_voltage', 
            10
        )
        
        self.battery_percentage_publisher = self.create_publisher(
            Float32, 
            f'{topic_prefix}/battery_percentage', 
            10
        )
        
        # Station information publishers
        self.current_station_publisher = self.create_publisher(
            String,
            f'{topic_prefix}/current_station',
            10
        )
        
        self.last_station_publisher = self.create_publisher(
            String,
            f'{topic_prefix}/last_station',
            10
        )
        
        # Position confidence publisher
        self.position_confidence_publisher = self.create_publisher(
            Float32,
            f'{topic_prefix}/position_confidence',
            10
        )
        
        # Robot angle publisher
        self.robot_angle_publisher = self.create_publisher(
            Float32,
            f'{topic_prefix}/robot_angle',
            10
        )
        
        # State variables
        self.last_position = None
        self.last_battery = None
        self.api_connection_active = False
        self.connection_lock = threading.Lock()
        
        # Create timer for periodic updates
        self.timer = self.create_timer(
            1.0 / self.update_rate, 
            self.update_robot_state
        )
        
        # Statistics
        self.position_request_count = 0
        self.battery_request_count = 0
        self.position_error_count = 0
        self.battery_error_count = 0
        
        self.get_logger().info(f'Robot state node initialized for {self.robot_name}')
        self.get_logger().info(f'Publishing to topics with prefix: {topic_prefix}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        
        self.logger.info(f'Robot state node initialization complete')
    
    def create_api_connection(self) -> Optional[RobotAPI]:
        """Create API connection to robot."""
        try:
            api = RobotAPI(
                robot_key=self.robot_key,
                api_name='robot_status_api'
            )
            return api
        except Exception as e:
            self.logger.error(f'Failed to create API connection: {e}')
            return None
    
    def update_robot_state(self):
        """Update robot state by fetching position and battery data."""
        # Run API calls in separate thread to avoid blocking the timer
        threading.Thread(target=self._fetch_robot_data, daemon=True).start()
    
    def _fetch_robot_data(self):
        """Fetch robot data from API (runs in separate thread)."""
        with self.connection_lock:
            # Fetch position
            self._fetch_position()
            
            # Fetch battery
            self._fetch_battery()
    
    def _fetch_position(self):
        """Fetch current position from robot."""
        api = self.create_api_connection()
        if not api:
            return
        
        try:
            if api.connect():
                self.position_request_count += 1
                
                response = api.send_api_request(
                    api_number=self.position_api_number,
                    payload={},  # No payload needed
                    req_id=self.position_request_count
                )
                
                if response and response.get('data'):
                    self._process_position_data(response['data'])
                    self.logger.debug(f'Position data received: {response["data"]}')
                else:
                    self.position_error_count += 1
                    self.logger.warning(f'No position data received (errors: {self.position_error_count})')
                    
            else:
                self.position_error_count += 1
                self.logger.error(f'Failed to connect for position data (errors: {self.position_error_count})')
                
        except Exception as e:
            self.position_error_count += 1
            self.logger.error(f'Position fetch error: {e} (errors: {self.position_error_count})')
        finally:
            if api:
                api.disconnect()
    
    def _fetch_battery(self):
        """Fetch battery state from robot."""
        api = self.create_api_connection()
        if not api:
            return
        
        try:
            if api.connect():
                self.battery_request_count += 1
                
                response = api.send_api_request(
                    api_number=self.battery_api_number,
                    payload={},  # No payload needed
                    req_id=self.battery_request_count
                )
                
                if response and response.get('data'):
                    self._process_battery_data(response['data'])
                    self.logger.debug(f'Battery data received: {response["data"]}')
                else:
                    self.battery_error_count += 1
                    self.logger.warning(f'No battery data received (errors: {self.battery_error_count})')
                    
            else:
                self.battery_error_count += 1
                self.logger.error(f'Failed to connect for battery data (errors: {self.battery_error_count})')
                
        except Exception as e:
            self.battery_error_count += 1
            self.logger.error(f'Battery fetch error: {e} (errors: {self.battery_error_count})')
        finally:
            if api:
                api.disconnect()
    
    def _process_position_data(self, data: Dict[str, Any]):
        """Process position data and publish."""
        try:
            # Extract position data from the actual API response format
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            z = data.get('z', 0.0)  # Most robots don't provide z, so default to 0
            angle = data.get('angle', 0.0)
            confidence = data.get('confidence', 0.0)
            current_station = data.get('current_station', '')
            last_station = data.get('last_station', '')
            
            self.logger.debug(f'Processing position data: x={x}, y={y}, angle={angle}, confidence={confidence}, current_station={current_station}, last_station={last_station}')
            
            # Create and publish PointStamped message
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = f'{self.robot_namespace}/base_link' if self.robot_namespace else 'base_link'
            point_msg.point.x = float(x)
            point_msg.point.y = float(y)
            point_msg.point.z = float(z)
            
            self.position_publisher.publish(point_msg)
            
            # Publish station information
            current_station_msg = String()
            current_station_msg.data = str(current_station)
            self.current_station_publisher.publish(current_station_msg)
            
            last_station_msg = String()
            last_station_msg.data = str(last_station)
            self.last_station_publisher.publish(last_station_msg)
            
            # Publish position confidence
            confidence_msg = Float32()
            confidence_msg.data = float(confidence)
            self.position_confidence_publisher.publish(confidence_msg)
            
            # Publish robot angle
            angle_msg = Float32()
            angle_msg.data = float(angle)
            self.robot_angle_publisher.publish(angle_msg)
            
            # Update last position with all relevant data
            self.last_position = {
                'x': x, 
                'y': y, 
                'z': z,
                'angle': angle,
                'confidence': confidence,
                'current_station': current_station,
                'last_station': last_station
            }
            
            self.logger.debug(f'Published position: x={x}, y={y}, z={z}, angle={angle}, confidence={confidence}')
            self.logger.debug(f'Station info: current={current_station}, last={last_station}')
            
        except Exception as e:
            self.logger.error(f'Error processing position data: {e}')
            self.get_logger().error(f'Error processing position data: {e}')
    
    def _process_battery_data(self, data: Dict[str, Any]):
        """Process battery data and publish."""
        try:
            # Extract battery data from the actual API response format
            voltage = data.get('voltage', 0.0)
            battery_level = data.get('battery_level', 0.0)  # This is typically 0-1 range
            current = data.get('current', 0.0)
            battery_temp = data.get('battery_temp', 0.0)
            charging = data.get('charging', False)
            
            # Convert battery_level to percentage (0-100)
            percentage = battery_level * 100.0 if battery_level <= 1.0 else battery_level
            
            self.logger.debug(f'Processing battery data: voltage={voltage}V, level={battery_level}, percentage={percentage}%, current={current}A, temp={battery_temp}°C, charging={charging}')
            
            # Create and publish BatteryState message
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.header.frame_id = f'{self.robot_namespace}/battery' if self.robot_namespace else 'battery'
            
            battery_msg.voltage = float(voltage)
            battery_msg.percentage = float(battery_level)  # Keep as 0-1 range for ROS standard
            battery_msg.current = float(current)
            battery_msg.temperature = float(battery_temp)
            
            # Set power supply status based on charging state and percentage
            if charging:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif percentage > 95:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            
            # Set health status
            battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            battery_msg.present = True
            
            self.battery_publisher.publish(battery_msg)
            
            # Publish individual values
            voltage_msg = Float32()
            voltage_msg.data = float(voltage)
            self.battery_voltage_publisher.publish(voltage_msg)
            
            percentage_msg = Float32()
            percentage_msg.data = float(percentage)  # As 0-100 for convenience
            self.battery_percentage_publisher.publish(percentage_msg)
            
            # Update last battery state
            self.last_battery = {
                'voltage': voltage,
                'battery_level': battery_level,
                'percentage': percentage,
                'current': current,
                'battery_temp': battery_temp,
                'charging': charging
            }
            
            self.logger.debug(f'Published battery: voltage={voltage}V, percentage={percentage}%, current={current}A, charging={charging}')
            
        except Exception as e:
            self.logger.error(f'Error processing battery data: {e}')
            self.get_logger().error(f'Error processing battery data: {e}')
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get node statistics."""
        return {
            'position_requests': self.position_request_count,
            'battery_requests': self.battery_request_count,
            'position_errors': self.position_error_count,
            'battery_errors': self.battery_error_count,
            'last_position': self.last_position,
            'last_battery': self.last_battery
        }
    
    def destroy_node(self):
        """Clean up resources."""
        self.logger.info('Shutting down robot state node')
        
        # Log final statistics
        stats = self.get_statistics()
        self.logger.info(f'Final statistics: {stats}')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = RobotStateNode()
        
        # Log startup info
        node.get_logger().info('Robot state node started successfully')
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in robot state node: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()