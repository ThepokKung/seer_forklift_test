#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# msg imports
from std_msgs.msg import String
from seer_robot_interfaces.msg import RobotBatchData

# srv imports
from std_srvs.srv import Trigger
from seer_robot_interfaces.srv import CheckRobotStateNow, UpdateRobotStationForCollision

# backend imports
from seer_robot_pkg.robot_status_api import RobotStatusAPI

# Navigation Status Constants
NAV_NONE = 0
NAV_WAITING = 1  # Currently impossible according to robot_state.py
NAV_RUNNING = 2
NAV_SUSPENDED = 3
NAV_COMPLETED = 4
NAV_FAILED = 5
NAV_CANCELED = 6

class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self.get_logger().info('Robot Monitor Node has been started')

        # Declare parameters
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('robot_name', 'SEER_Robot_01')
        self.declare_parameter('robot_ip', '192.168.0.180')

        # Parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        # Initialize API connection
        self.robot_status_api = RobotStatusAPI(self.robot_ip)

        # Robot Status (from robot_status)
        self.robot_battery = None
        self.robot_position = None
        self.robot_current_station = None
        self.robot_task_id = None
        self.robot_have_good = None
        self.robot_confidence = 0.0
        self.robot_navigation_status = 0
        self.robot_charge_status = False
        self.robot_available = False
        self.robot_controller_mode_status = False
        self.robot_fork_height = 0.0
        self.robot_map_name = None

        # Emergency statuses
        self.robot_driver_emergency_status = False
        self.robot_emergency_status = False
        self.robot_solf_emergency_status = False
        self.robot_electric_status = False

        # Previous values for change detection
        self.prev_robot_battery = None
        self.prev_robot_current_station = None
        self.prev_robot_navigation_status = None
        self.prev_robot_controller_mode_status = None

        # Connection status tracking
        self.connection_attempted = False
        self.connection_failure_count = 0
        self.connection_failure_logged = False

        # Robot State (from robot_state) 
        self.robot_state = 'IDLE'

        # Publishers
        self.robot_batch_data_pub = self.create_publisher(RobotBatchData, 'robot_monitor/robot_batch_data', 10)
        # self.robot_state_pub = self.create_publisher(String, 'robot_monitor/robot_state', 10)

        # Service Servers
        self.create_service(Trigger, 'robot_monitor/check_available', self.check_robot_available_callback)
        self.create_service(UpdateRobotStationForCollision, 'robot_monitor/update_robot_station', self.update_robot_station)
        self.create_service(CheckRobotStateNow, 'robot_monitor/check_robot_state_now', self.check_robot_state_now_callback)

        # Timer
        self.timer = self.create_timer(0.5, self.update_loop)

    def update_loop(self):
        """Main update loop - combines both status collection and state calculation"""
        self.update_robot_status()
        self.calculate_robot_state()
        self.publish_data()

    def update_robot_status(self):
        """Update robot status from API (from robot_status logic)"""
        try:
            if self.ensure_connection():
                temp_batch_data_1 = self.robot_status_api.get_batch_data_1()
                if temp_batch_data_1 is not None:
                    # Data from batch only using
                    self.robot_battery = (temp_batch_data_1.get('battery_level', 0) * 100)
                    self.robot_controller_mode_status = temp_batch_data_1.get('fork_auto_flag', False)
                    self.robot_navigation_status = temp_batch_data_1.get('task_status', 0)
                    self.robot_current_station = temp_batch_data_1.get('current_station', None)
                    self.robot_confidence = temp_batch_data_1.get('confidence', 0.0)
                    self.robot_charge_status = temp_batch_data_1.get('charging', False)
                    self.robot_fork_height = temp_batch_data_1.get('fork_height', 0.0)
                    self.robot_map_name = temp_batch_data_1.get('map_name', None)
                    
                    # Emergency Status
                    self.robot_driver_emergency_status = temp_batch_data_1.get('driver_emc', False)
                    self.robot_electric_status = temp_batch_data_1.get('electric', False)
                    self.robot_emergency_status = temp_batch_data_1.get('emergency', False)
                    self.robot_solf_emergency_status = temp_batch_data_1.get('solf_emc', False)
                else:
                    self.get_logger().debug(f"Robot ID: {self.robot_id}, No batch data received")
            else:
                self.get_logger().debug(f"Robot ID: {self.robot_id}, Not connected to robot")
        except Exception as e:
            self.get_logger().error(f"Error updating robot status: {e}")

    def calculate_robot_state(self):
        """Calculate high-level robot state (from robot_state logic)"""
        # Store previous state for change detection
        prev_state = self.robot_state
        
        # State calculation logic based on navigation status and battery
        if not self.robot_controller_mode_status and self.robot_battery is not None:
            self.robot_state = 'EXTERNAL_CONTROL'
        elif self.robot_navigation_status == NAV_RUNNING:
            self.robot_state = 'NAV_MOVING'
        elif self.robot_navigation_status == NAV_SUSPENDED:
            self.robot_state = 'NAV_SUSPENDED'
        elif self.robot_navigation_status == NAV_COMPLETED or (self.robot_navigation_status == NAV_NONE and self.robot_battery is not None):
            self.robot_state = 'READY'
        elif self.robot_navigation_status == NAV_FAILED:
            self.robot_state = 'NAV_FAILED'
        elif self.robot_navigation_status == NAV_CANCELED:
            self.robot_state = 'NAV_CANCELED'
        elif self.robot_navigation_status == NAV_NONE:
            self.robot_state = 'IDLE'
        
        # Log state changes
        if prev_state != self.robot_state:
            self.get_logger().info(f'Robot {self.robot_id} state changed: {prev_state} -> {self.robot_state} (Nav Status: {self.robot_navigation_status}, Battery: {self.robot_battery}%)')

    def publish_data(self):
        """Publish both batch data and state"""
        # Publish batch data message
        batch_msg = RobotBatchData()
        batch_msg.robot_battery_percentage = self.robot_battery if self.robot_battery is not None else 0.0
        batch_msg.robot_current_station = self.robot_current_station if self.robot_current_station is not None else "Unknown"
        batch_msg.robot_navigation_status = self.robot_navigation_status if self.robot_navigation_status is not None else 0
        batch_msg.robot_controller_mode_status = self.robot_controller_mode_status if self.robot_controller_mode_status is not None else False
        batch_msg.robot_fork_height = self.robot_fork_height if self.robot_fork_height is not None else 0.0
        batch_msg.robot_confidence = self.robot_confidence if self.robot_confidence is not None else 0.0
        batch_msg.robot_state = self.robot_state
        self.robot_batch_data_pub.publish(batch_msg)
        
        # Publish robot state
        # self.robot_state_pub.publish(String(data=self.robot_state))

    def ensure_connection(self):
        """Ensure connection to robot"""
        if not self.robot_status_api.connected:
            if self.robot_status_api.connect():
                # Reset failure tracking on successful connection
                if self.connection_failure_count > 0:
                    self.get_logger().info(f"Successfully reconnected to robot at {self.robot_ip} after {self.connection_failure_count} failures")
                else:
                    self.get_logger().info(f"Successfully connected to robot at {self.robot_ip}")
                self.connection_failure_count = 0
                self.connection_failure_logged = False
                self.robot_available = True
                return True
            else:
                # Increment failure counter
                self.connection_failure_count += 1
                
                # Log first failure immediately, then every 30 failures
                if self.connection_failure_count == 1:
                    self.get_logger().warn(f"Failed to connect to robot at {self.robot_ip}")
                elif self.connection_failure_count >= 30 and not self.connection_failure_logged:
                    self.get_logger().warn(f"Failed to connect to robot at {self.robot_ip} (failed {self.connection_failure_count} times)")
                    self.connection_failure_logged = True
                
                # Reset counter and flag for next logging cycle after 30 attempts
                if self.connection_failure_count >= 30:
                    self.connection_failure_count = 0
                    self.connection_failure_logged = False
                
                self.robot_available = False
                return False
        self.robot_available = True
        return True

    # Service callbacks (combine from both files)
    def check_robot_available_callback(self, request, response):
        """Service to check if the robot is available"""
        try:
            self.get_logger().info(f"Service called: checking availability for robot {self.robot_id}")
            self.get_logger().info(f"Robot available status: {self.robot_available}")
            response.success = self.robot_available
            response.message = f"Robot {self.robot_id} is available." if self.robot_available else f"Robot {self.robot_id} is not available."
            self.get_logger().info(f"Returning response: success={response.success}, message='{response.message}'")
            return response
        except Exception as e:
            self.get_logger().error(f"Error in check_robot_available_callback: {e}")
            response.success = False
            response.message = f"Error checking robot {self.robot_id}: {str(e)}"
            return response

    def update_robot_station(self, request, response):
        self.get_logger().info(f"Service called: updating robot station to {request.robot_station} for robot {self.robot_id}")
        self.robot_current_station = request.robot_station
        response.success = True
        response.message = f"Robot {self.robot_id} station updated to {self.robot_current_station}."
        return response

    def check_robot_state_now_callback(self, request, response):
        response.success = True
        response.robot_state = self.robot_state
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()