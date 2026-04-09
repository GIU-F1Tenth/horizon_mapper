#!/usr/bin/env python3
"""
Horizon Mapper Launch File
==========================

Launches the horizon_mapper_node with proper parameter configuration.

Author: Mohammed Azab (Mohammed@azab.io)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for horizon_mapper_node"""

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='horizon_mapper.yaml',
        description='Name of the configuration file to use'
    )

    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable detailed logging'
    )

    enable_debugging_arg = DeclareLaunchArgument(
        'enable_debugging',
        default_value='false',
        description='Enable debug output'
    )

    horizon_N_arg = DeclareLaunchArgument(
        'horizon_N',
        default_value='40',
        description='MPC prediction horizon steps'
    )

    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='/path',
        description='Topic to subscribe for trajectory path'
    )

    # Package paths
    pkg_share = FindPackageShare('horizon_mapper')

    # Configuration file path
    config_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        LaunchConfiguration('config_file')
    ])

    def launch_setup(context):
        """Setup function to handle parameter substitution"""

        # Create the node with parameters
        horizon_mapper_node = Node(
            package='horizon_mapper',
            executable='horizon_mapper_node',
            name='horizon_mapper_node',
            parameters=[
                config_file_path,
                {
                    # Override with launch arguments if provided
                    'enable_logging': LaunchConfiguration('enable_logging'),
                    'enable_debugging': LaunchConfiguration('enable_debugging'),
                    'horizon_N': LaunchConfiguration('horizon_N'),
                    'path_topic': LaunchConfiguration('path_topic'),
                }
            ],
            output='screen',
            emulate_tty=True,
            respawn=False,
            respawn_delay=2.0
        )

        return [horizon_mapper_node]

    return LaunchDescription([
        config_file_arg,
        enable_logging_arg,
        enable_debugging_arg,
        horizon_N_arg,
        path_topic_arg,
        OpaqueFunction(function=launch_setup)
    ])
