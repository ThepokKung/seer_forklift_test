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
        # Robot status node
        nodes.append(Node(
            package='seer_robot_pkg',
            executable='robot_status.py',
            name='robot_status_node',
            namespace=robot['namespace'],
            output='screen',
            parameters=[{
                'robot_id': robot['robot_id'],
                'robot_name': robot['robot_name'],
                'robot_ip': robot['robot_ip']
            }]
        ))

        # Robot state node
        nodes.append(Node(
            package='seer_robot_pkg',
            executable='robot_state.py',
            name='robot_state_node',
            namespace=robot['namespace'],
            output='screen',
            parameters=[{
                'robot_id': robot['robot_id'],
                'robot_name': robot['robot_name'],
                'robot_ip': robot['robot_ip']
            }]
        ))
        
        # Robot controller node
        nodes.append(Node(
            package='seer_robot_pkg',
            executable='robot_controller.py',
            name='robot_controller_node',
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
        package='seer_robot_pkg',
        executable='task_management.py',
        name='task_management_node',
        output='screen'
    ))

    # fastapi_ros_bridge node
    nodes.append(Node(
        package='ros_bridge2_api',
        executable='fastapi_ros_bridge.py',
        name='fastapi_ros_bridge_node',
        output='screen'
    ))

    # traffic management node
    nodes.append(Node(
        package='seer_robot_pkg',
        executable='traffic_management.py',
        name='traffic_management_node',
        output='screen'
    ))

    return LaunchDescription(nodes)
