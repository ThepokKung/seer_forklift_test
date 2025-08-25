#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


# msg imports
from std_msgs.msg import String,Float32,Int8
from geometry_msgs.msg import PointStamped

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state')
        self.get_logger().info('Robot State Node has been started')

        # Declare parameters
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('robot_name', 'SEER_Robot_01')
        self.declare_parameter('robot_ip', '192.168.0.180')

        # Parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        # Robot Status
        self.robot_state = 'IDLE'
        self.robot_battery = None
        self.robot_available = None
        self.robot_current_station = None
        self.robot_is_charger = False
        self.robot_has_load = False
        self.robot_navigation_status = 0
        self.robot_has_task = False
        self.robot_on_manipulator_station = False

        # Create subscriptions
        self.create_subscription(Int8,'robot_status/robot_navigation_status',self._sub_robot_navigation_status_callback,10)
        self.create_subscription(Float32,'robot_status/robot_battery_percentage',self._sub_robot_battery_callback,10)

        # create service server
        # self.create_service()
        
        # timer
        self.timer = self.create_timer(1.0, self._update_robot_state_callbacks) #1.0 seconds interval

    #####################################################
    ###                 Update State                  ###
    #####################################################

    ##### NAV STATUS #####
    # 0: NONE
    # 1: WAITING(This state is currently impossible)
    # 2: RUNNING
    # 3: SUSPENDED
    # 4: COMPLETED
    # 5: FAILED
    # 6: CANCELED

    def _update_robot_state_callbacks(self):
        if self.robot_navigation_status == 2:
            self.robot_state = 'NAV_MOVING'
        elif self.robot_navigation_status == 3:
            self.robot_state = 'NAV_SUSPENDED'
        elif self.robot_navigation_status == 4:
            self.robot_state = 'READY'
        elif self.robot_navigation_status == 5:
            self.robot_state = 'NAV_FAILED'
        elif self.robot_navigation_status == 6:
            self.robot_state = 'NAV_CANCELED'
        elif self.robot_navigation_status == 0:
            self.robot_state = 'IDLE'

        self.get_logger().info(f'Robot ID : {self.robot_id}, Robot State: {self.robot_state}, Nav Status: {self.robot_navigation_status}, Battery: {self.robot_battery} %')

    #####################################################
    ###              Update robot status              ###
    #####################################################

    def _sub_robot_navigation_status_callback(self,msg):
        self.robot_navigation_status = int(msg.data)

    def _sub_robot_battery_callback(self,msg):
        self.robot_battery = int(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()