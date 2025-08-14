from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seer_robotic_pkg',
            executable='n_robot_state.py',
            name='robot_state_node',
            namespace='robot_02',
            output='screen',
            parameters=[{
                'robot_id': 'robot_02',
                'robot_name': 'SEER_Robot_02',
                'robot_ip': '192.168.0.181'
            }]
        ),
        Node(
            package='seer_robotic_pkg',
            executable='n_robot_navigation.py',
            name='robot_navigation_node',
            namespace='robot_02',
            output='screen',
            parameters=[{
                'robot_id': 'robot_02',
                'robot_name': 'SEER_Robot_02',
                'robot_ip': '192.168.0.181'
            }]
        ),
        Node(
            package='seer_robotic_pkg',
            executable='n_test_only.py',
            name='test_node',
            namespace='robot_02',
            output='screen',
            parameters=[{
                'robot_id': 'robot_02',
                'robot_name': 'SEER_Robot_02',
                'robot_ip': '192.168.0.181'
            }]
        )
    ])