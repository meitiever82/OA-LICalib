#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get the package directory
    pkg_dir = get_package_share_directory('oa_licalib')
    
    # Get launch configuration
    config_path = LaunchConfiguration('config_path').perform(context)
    
    # If config_path is relative, make it relative to the package share directory
    if not os.path.isabs(config_path):
        config_path = os.path.join(pkg_dir, config_path)
    
    # Main calibration node
    li_calib_node = Node(
        package='oa_licalib',
        executable='li_calib_node',
        name='li_calib_node_batch',
        output='screen',
        parameters=[{
            'config_path': config_path
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return [li_calib_node]


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config_path',
            default_value='config/simu.yaml',
            description='Path to the configuration YAML file'
        ),
        
        # Use OpaqueFunction to allow dynamic path resolution
        OpaqueFunction(function=launch_setup)
    ])