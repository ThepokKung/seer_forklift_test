#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'update_rate',
            default_value='1.0',
            description='Update rate in Hz for fetching robot state'
        ),
        
        DeclareLaunchArgument(
            'enable_robot_01',
            default_value='true',
            description='Enable robot_01 state monitoring'
        ),
        
        DeclareLaunchArgument(
            'enable_robot_02',
            default_value='true',
            description='Enable robot_02 state monitoring'
        ),
        
        # Log launch configuration
        LogInfo(
            msg=['Starting multi-robot state monitoring system']
        ),
        LogInfo(
            msg=['  Update rate: ', LaunchConfiguration('update_rate'), ' Hz']
        ),
        LogInfo(
            msg=['  Robot 01 enabled: ', LaunchConfiguration('enable_robot_01')]
        ),
        LogInfo(
            msg=['  Robot 02 enabled: ', LaunchConfiguration('enable_robot_02')]
        ),
        
        # Robot 01 state node (default namespace)
        Node(
            package='seer_robotic',
            executable='robot_state.py',
            name='robot_01_state',
            parameters=[
                {'robot_key': 'robot_01'},
                {'update_rate': LaunchConfiguration('update_rate')},
                {'robot_namespace': 'robot_01'}
            ],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(LaunchConfiguration('enable_robot_01'))
        ),
        
        # Robot 02 state node (with namespace)
        GroupAction([
            PushRosNamespace('robot_02'),
            Node(
                package='seer_robotic',
                executable='robot_state.py',
                name='robot_02_state',
                parameters=[
                    {'robot_key': 'robot_02'},
                    {'update_rate': LaunchConfiguration('update_rate')},
                    {'robot_namespace': 'robot_02'}
                ],
                output='screen',
                emulate_tty=True,
                respawn=True,
                respawn_delay=5.0,
                condition=IfCondition(LaunchConfiguration('enable_robot_02'))
            )
        ])
    ])
