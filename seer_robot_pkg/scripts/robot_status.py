#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import sys
import os

# msg imports
from std_msgs.msg import String,Float32,Bool,Int32

# srv imports
from std_srvs.srv import Trigger
from seer_robot_interfaces.srv import UpdateRobotStationForCollision

# backend imports
from seer_robot_pkg.robot_status_api import RobotStatusAPI

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

        # Robot Status
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

        # Create RobotStatusAPI instance but don't auto-connect
        self.robot_status_api = RobotStatusAPI(self.robot_ip)
        
        # Connection status tracking
        self.connection_attempted = False

        # # Publisher topics
        self.robot_battery_pub = self.create_publisher(Float32, 'robot_status/robot_battery_percentage', 10)
        self.robot_current_station_pub = self.create_publisher(String, 'robot_status/robot_current_station', 10)
        self.robot_navigation_status_pub = self.create_publisher(Int32, 'robot_status/robot_navigation_status', 10)
        self.robot_controller_mode_status_pub = self.create_publisher(Bool, 'robot_status/robot_controller_mode_status', 10)

        # Service Server
        self.create_service(Trigger, 'robot_status/check_available', self.check_robot_available_callback)
        self.create_service(UpdateRobotStationForCollision, 'robot_status/update_robot_station', self.update_robot_station)

        # Timer
        self.timer = self.create_timer(0.5, self.update_robot_callbacks) #0.5 seconds interval

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
    ###                 Update Status                  ###
    #####################################################

    def update_robot_callbacks(self):
        self.update_robot_batch_data1_status()
        self.pub_data()

    def update_robot_batch_data1_status(self):
        try:
            if self.ensure_connection():
                temp_batch_data_1 = self.robot_status_api.get_batch_data_1()
                if temp_batch_data_1 is not None:
                    # Data on batch only using
                    self.robot_battery = (temp_batch_data_1.get('battery_level', 0) * 100)
                    self.robot_controller_mode_status = temp_batch_data_1.get('fork_auto_flag', False)
                    self.robot_navigation_status = temp_batch_data_1.get('task_status', 0)
                    self.robot_current_station = temp_batch_data_1.get('current_station', None)
                    self.robot_confidence = temp_batch_data_1.get('confidence', 0.0)
                    self.robot_charge_status = temp_batch_data_1.get('charging', False)
                    self.robot_fork_height = temp_batch_data_1.get('fork_height', 0.0)
                    #  Emergency Status
                    self.robot_driver_emergency_status = temp_batch_data_1.get('driver_emc', False)
                    self.robot_electric_status = temp_batch_data_1.get('electric', False)
                    self.robot_emergency_status = temp_batch_data_1.get('emergency', False)
                    self.robot_solf_emergency_status = temp_batch_data_1.get('solf_emc', False)
                    # # Publish relevant data
                    # self.robot_battery_pub.publish(Float32(data=self.robot_battery if self.robot_battery is not None else 0.0))
                    # self.robot_current_station_pub.publish(String(data=self.robot_current_station if self.robot_current_station is not None else "Unknown"))
                    # self.robot_navigation_status_pub.publish(Int32(data=self.robot_navigation_status if self.robot_navigation_status is not None else 0))
                    # self.robot_controller_mode_status_pub.publish(Bool(data=self.robot_controller_mode_status if self.robot_controller_mode_status is not None else False))
                else:
                    self.get_logger().debug(f"Robot ID: {self.robot_id}, No batch data received")
            else:
                self.get_logger().debug(f"Robot ID: {self.robot_id}, Not connected to robot")
        except Exception as e:
            self.get_logger().error(f"Error updating batch data 1 status: {e}")

    def pub_data(self):
        self.robot_battery_pub.publish(Float32(data=self.robot_battery if self.robot_battery is not None else 0.0))
        self.robot_current_station_pub.publish(String(data=self.robot_current_station if self.robot_current_station is not None else "Unknown"))
        self.robot_navigation_status_pub.publish(Int32(data=self.robot_navigation_status if self.robot_navigation_status is not None else 0))
        self.robot_controller_mode_status_pub.publish(Bool(data=self.robot_controller_mode_status if self.robot_controller_mode_status is not None else False))
                    

    #####################################################
    ###             Service Callbacks                 ###
    #####################################################
    
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

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
