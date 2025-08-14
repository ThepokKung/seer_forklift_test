#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import sys
import os

# msg imports
from std_msgs.msg import String
# from geometry_msgs.msg import PointStamped

# srv imports
from std_srvs.srv import Trigger
from seer_robot_interfaces.srv import CheckRobotNavigationTaskStatus,CheckRobotCurrentLocation,CheckRobotAllForTask

# backend imports
from seer_robot_pkg.seer_robot_pkg.robot_status_api import RobotStatusAPI

class RobotStatus(Node):
    def __init__(self):
        super().__init__('robot_status')
        self.get_logger().info("Robot Status Node has been started.")

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
        self.robot_available = False

        # Create RobotStatusAPI instance but don't auto-connect
        self.robot_status_api = RobotStatusAPI(self.robot_ip)
        
        # Connection status tracking
        self.connection_attempted = False

        # # Publisher topics
        # self.robot_battery_pub = self.create_publisher(Float32, 'robot_battery_percentage', 10)
        # # Robot position publisher
        # self.robot_position_pub = self.create_publisher(PointStamped, 'robot_position', 10)
        self.robot_current_station_pub = self.create_publisher(String, 'robot_state/robot_current_station', 10)
        # self.robot_last_station_pub = self.create_publisher(String, 'robot_last_station', 10)

        # Service Server
        self.create_service(CheckRobotNavigationTaskStatus, 'robot_state/check_robot_navigation_status', self.check_robot_navigation_status_callback)
        self.create_service(CheckRobotCurrentLocation, 'robot_state/check_robot_current_location', self.check_robot_current_location_callback)
        self.create_service(CheckRobotAllForTask, 'robot_state/check_robot_all_for_task', self.check_robot_all_for_task_callback)
        self.create_service(Trigger, 'robot_state/check_available', self.check_robot_available_callback)

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
                self.robot_available = True
                return True
            else:
                self.get_logger().warn(f"Failed to connect to robot at {self.robot_ip}")
                self.robot_available = False
                return False
        self.robot_available = True
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
        try:
            if self.ensure_connection():
                temp = self.robot_status_api.get_battery_status()
                if temp is not None:
                    self.robot_battery = (temp.get('battery_level', None) * 100) # type: ignore
                    self.robot_charge_status = temp.get('charging', False) # type: ignore
                else:
                    self.robot_battery = None
            else:
                self.robot_battery = None
        except Exception as e:
            self.get_logger().error(f"Error updating battery status: {e}")
            self.robot_battery = None

    def update_robot_position(self):
        # Ensure we have a connection
        try:
            if self.ensure_connection():
                temp_position = self.robot_status_api.get_position_status()
                # print(f"Robot ID: {self.robot_id}, Position: {temp_position}")

                # Publish the position if we got valid data
                if temp_position is not None:
                    # Unpack the tuple returned from position_status()
                    x, y, angle, current_position, last_station, confidence = temp_position

                    self.robot_current_station_pub.publish(String(data=current_position))

                    # Store values for internal use
                    self.robot_position = {'x': x, 'y': y, 'angle': angle}
                    self.robot_current_station = current_position
                    self.robot_last_station = last_station
                else:
                    self.robot_position = None
            else:
                self.get_logger().debug(f"Robot ID: {self.robot_id}, Not connected to robot")
                self.robot_position = None
        except Exception as e:
            self.get_logger().error(f"Error updating position: {e}")
            self.robot_position = None

    def update_robot_navigation_status(self):
        # Ensure we have a connection
        try:
            if self.ensure_connection():
                temp_navigation = self.robot_status_api.get_navigation_status()

                # Publish the navigation status if we got valid data
                if temp_navigation is not None:
                    self.robot_navigation_status = temp_navigation.get('task_status', 0)
                else:
                    self.robot_navigation_status = None
            else:
                self.get_logger().debug(f"Robot ID: {self.robot_id}, Not connected to robot")
                self.robot_navigation_status = None
        except Exception as e:
            self.get_logger().error(f"Error updating navigation status: {e}")
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
            response.robot_current_station = str(self.robot_current_station) if self.robot_current_station is not None else "Unknown"
            
            # Convert navigation status integer to meaningful string
            if self.robot_navigation_status == 0:
                response.robot_task_status = "IDLE"
            elif self.robot_navigation_status == 1:
                response.robot_task_status = "READY"
            elif self.robot_navigation_status == 2:
                response.robot_task_status = "BUSY"
            else:
                response.robot_task_status = f"STATUS_{self.robot_navigation_status}"

            response.robot_navigation_status = int(self.robot_navigation_status)
        else:
            response.success = False
            response.robot_current_station = "Unknown"
            response.robot_task_status = "Unknown"
            response.robot_navigation_status = None
        return response
    
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

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
