#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import sys
import os

# msg imports
# from std_msgs.msg import Float32,String
# from geometry_msgs.msg import PointStamped

# srv imports
from std_srvs.srv import Trigger
from seer_robot_interfaces.srv import CheckRobotNavigationTaskStatus,CheckRobotCurrentLocation,CheckRobotAllForTask

# backend imports
from bn_robot_status_api import RobotStatusAPI

class RobotState(Node):
    def __init__(self):
        super().__init__('robot_state_node')
        self.get_logger().info("Robot State Node has been started.")

        # Declare parameters
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('robot_name', 'SEER_Robot_01')
        self.declare_parameter('robot_ip', '192.168.0.180')

        # Parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_battery = None
        self.robot_position = None
        self.robot_current_station = None
        self.robot_last_station = None
        self.robot_task_id = None
        self.robot_state = None
        self.robot_have_good = None
        # self.robot_task_status = 0
        self.robot_navigation_status = 0
        self.robot_charge_status = False
        
        # Create RobotStatusAPI instance but don't auto-connect
        self.robot_status_api = RobotStatusAPI(self.robot_ip)
        
        # Connection status tracking
        self.connection_attempted = False

        # # Publisher topics
        # self.robot_battery_pub = self.create_publisher(Float32, 'robot_battery_percentage', 10)
        # # Robot position publisher
        # self.robot_position_pub = self.create_publisher(PointStamped, 'robot_position', 10)
        # self.robot_current_station_pub = self.create_publisher(String, 'robot_current_station', 10)
        # self.robot_last_station_pub = self.create_publisher(String, 'robot_last_station', 10)

        # Service Server
        self.create_service(CheckRobotNavigationTaskStatus, 'robot_state/check_robot_navigation_status', self.check_robot_navigation_status_callback)
        self.create_service(CheckRobotCurrentLocation, 'robot_state/check_robot_current_location', self.check_robot_current_location_callback)
        self.create_service(CheckRobotAllForTask, 'robot_state/check_robot_all_for_task', self.check_robot_all_for_task_callback)

        # Timer
        self.timer = self.create_timer(1.0, self.update_robot_callbacks) #1.0 seconds interval

    #####################################################
    ###               Check Connection                ###
    #####################################################

    def ensure_connection(self):
        """Ensure connection to robot with retry logic"""
        if not self.robot_status_api.connected:
            if self.robot_status_api.connect():
                self.get_logger().info(f"Successfully connected to robot at {self.robot_ip}")
                return True
            else:
                self.get_logger().warn(f"Failed to connect to robot at {self.robot_ip}")
                return False
        return True

    def force_reconnect(self):
        """Force a reconnection attempt"""
        self.get_logger().info("Forcing reconnection...")
        self.robot_status_api.disconnect()
        self.connection_attempted = False
        return self.ensure_connection()

    #####################################################
    ###                 Update State                  ###
    #####################################################

    def update_robot_callbacks(self):
        """Update robot state by calling battery and position updates."""
        self.update_robot_battery()
        self.update_robot_position()
        self.update_robot_navigation_status()

    def update_robot_battery(self):
        # Ensure we have a connection
        if self.ensure_connection():
            temp = self.robot_status_api.get_battery_status()
            self.robot_battery = (temp.get('battery_level', None) * 100) # type: ignore
            self.robot_charge_status = temp.get('charging', False) # type: ignore
        else:
            self.robot_battery = None

    def update_robot_position(self):
        # Ensure we have a connection
        if self.ensure_connection():
            temp_position = self.robot_status_api.get_position_status()
            # print(f"Robot ID: {self.robot_id}, Position: {temp_position}")

            # Publish the position if we got valid data
            if temp_position is not None:
                # Unpack the tuple returned from position_status()
                x, y, angle, current_position, last_station, confidence = temp_position

                # Store values for internal use
                self.robot_position = {'x': x, 'y': y, 'angle': angle}
                self.robot_current_station = current_position
                self.robot_last_station = last_station
        else:
            self.get_logger().error(f"Robot ID: {self.robot_id}, Not connected to robot")
            self.robot_position = None

    def update_robot_navigation_status(self):
        # Ensure we have a connection
        if self.ensure_connection():
            temp_navigation = self.robot_status_api.get_navigation_status()

            # Publish the navigation status if we got valid data
            if temp_navigation is not None:
                self.robot_navigation_status = temp_navigation.get('task_status', 0)
        else:
            self.get_logger().error(f"Robot ID: {self.robot_id}, Not connected to robot")
            self.robot_navigation_status = None

    #####################################################
    ###             Service Callbacks                 ###
    #####################################################

    def check_robot_navigation_status_callback(self, request, response):
        response.success = True
        response.task_status = self.robot_navigation_status
        return response
    
    def check_robot_current_location_callback(self, request, response):
        if self.robot_current_station is not None:
            response.success = True
            response.robot_current_station = self.robot_current_station
        else:
            response.success = False
            response.robot_current_station = "Unknown"
        return response
    
    def check_robot_all_for_task_callback(self, request, response):
        if self.robot_current_station is not None and self.robot_navigation_status is not None:
            response.success = True
            response.robot_current_station = self.robot_current_station
            response.robot_task_status = self.robot_navigation_status
            response.robot_navigation_status = "Active" if self.robot_navigation_status > 0 else "Idle"
        else:
            response.success = False
            response.robot_current_station = "Unknown"
            response.robot_task_status = "Unknown"
            response.robot_navigation_status = "Unknown"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
