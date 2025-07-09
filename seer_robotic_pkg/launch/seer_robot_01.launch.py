from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seer_robotic_pkg',
            executable='robot_state.py',
            name='robot_state_node',
            namespace='robot_01',
            output='screen',
            parameters=[{
                'robot_id': 'robot_01',
                'robot_name': 'SEER_Robot_01',
                'robot_ip': '192.168.0.180'
            }]
        ),
        Node(
            package='seer_robotic_pkg',
            executable='robot_navigation.py',
            name='robot_navigation_node',
            namespace='robot_01',
            output='screen',
            parameters=[{
                'robot_id': 'robot_01',
                'robot_name': 'SEER_Robot_01',
                'robot_ip': '192.168.0.180'
            }]
        )
    ])