#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'robot_key',
            default_value='robot_01',
            description='Robot key from configuration (robot_01, robot_02, etc.)'
        ),
        
        DeclareLaunchArgument(
            'update_rate',
            default_value='1.0',
            description='Update rate in Hz for fetching robot state'
        ),
        
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Namespace for robot topics (empty for no namespace)'
        ),
        
        DeclareLaunchArgument(
            'node_name',
            default_value='robot_state',
            description='Name of the robot state node'
        ),
        
        # Log launch configuration
        LogInfo(
            msg=['Starting robot state node with:']
        ),
        LogInfo(
            msg=['  Robot key: ', LaunchConfiguration('robot_key')]
        ),
        LogInfo(
            msg=['  Update rate: ', LaunchConfiguration('update_rate'), ' Hz']
        ),
        LogInfo(
            msg=['  Namespace: ', LaunchConfiguration('robot_namespace')]
        ),
        
        # Robot state node
        Node(
            package='seer_robotic',
            executable='robot_state.py',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('robot_namespace'),
            parameters=[
                {'robot_key': LaunchConfiguration('robot_key')},
                {'update_rate': LaunchConfiguration('update_rate')},
                {'robot_namespace': LaunchConfiguration('robot_namespace')}
            ],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=5.0
        )
    ])
