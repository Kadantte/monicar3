#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def get_parameters(context):
    # Get the config choice
    color = LaunchConfiguration('color').perform(context)

    package_share_dir = get_package_share_directory('monicar3_cv')
    # Select the appropriate config file
    if color == 'green':
        config_path = os.path.join(package_share_dir, 'param', 'green.yaml')
    elif color == 'yellow':
        config_path = os.path.join(package_share_dir, 'param', 'yellow.yaml')
    elif color == 'red':
        config_path = os.path.join(package_share_dir, 'param', 'red.yaml')
    else:
        config_path = os.path.join(package_share_dir, 'param', 'blue.yaml')

    return [Node(
        package='monicar3_cv', executable='find_ball', name='blob_detect_node', 
        output='screen',
        parameters=[config_path]
    )]

def generate_launch_description():
    # Include launch file
    launch_file_1 = os.path.join(
        get_package_share_directory('monicar3_cv'), 'launch','usbcam.launch.py'
    )

    include_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_1),
    )

    # Include launch file
    launch_file_2= os.path.join(
        get_package_share_directory('monicar3_control'), 'launch','chase_ball.launch.py'
    )

    include_chaseball = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_2),
    )

    # Include launch file
    launch_file_3= os.path.join(
        get_package_share_directory('monicar3_control'), 'launch','lowlevel.launch.py'
    )

    include_lowlevel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_3),
    )

    color = DeclareLaunchArgument(
    'color',
    default_value='yellow',
    description='Choose green, yellow, red, or blue'
    )

    return LaunchDescription([
        include_cam,
        include_chaseball,
        include_lowlevel,
        color,
        OpaqueFunction(function=get_parameters)
    ])

