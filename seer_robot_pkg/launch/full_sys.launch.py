from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Robot configuration data
    robots = [
        {
            'robot_id': 'robot_01',
            'robot_name': 'SEER_Robot_01',
            'robot_ip': '192.168.0.180',
            'namespace': 'robot_01'
        },
        {
            'robot_id': 'robot_02',
            'robot_name': 'SEER_Robot_02',
            'robot_ip': '192.168.0.181',
            'namespace': 'robot_02'
        }
    ]
    
    nodes = []
    
    # Create nodes for each robot
    for robot in robots:
        # Robot state node
        nodes.append(Node(
            package='seer_robotic_pkg',
            executable='n_robot_state.py',
            name='robot_state_node',
            namespace=robot['namespace'],
            output='screen',
            parameters=[{
                'robot_id': robot['robot_id'],
                'robot_name': robot['robot_name'],
                'robot_ip': robot['robot_ip']
            }]
        ))
        
        # Robot navigation node
        nodes.append(Node(
            package='seer_robotic_pkg',
            executable='n_robot_navigation.py',
            name='robot_navigation_node',
            namespace=robot['namespace'],
            output='screen',
            parameters=[{
                'robot_id': robot['robot_id'],
                'robot_name': robot['robot_name'],
                'robot_ip': robot['robot_ip']
            }]
        ))
    
    # Add task management node
    nodes.append(Node(
        package='seer_robotic_pkg',
        executable='n_task_management.py',
        name='n_task_management',
        output='screen'
    ))
    
    return LaunchDescription(nodes)