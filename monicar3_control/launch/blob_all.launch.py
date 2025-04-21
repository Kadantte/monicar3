#!/usr/bin/env python3
# Author: ChangWhan Le

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    cv_parameter = LaunchConfiguration(
    'cv_parameter',
    default=os.path.join(get_package_share_directory('monicar3_cv'),'param/camera.yaml')
    )

    return LaunchDescription([
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("monicar3_cv"), '/launch', '/csicam.launch.py'])
        ),
 
        DeclareLaunchArgument('cv_parameter', default_value=cv_parameter),
        Node(
        package='monicar3_cv', executable='find_ball', name='blob_detect_node',
        output='screen', emulate_tty=True,
        parameters=[cv_parameter],
        ),
 
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("monicar3_control"), '/launch', '/chase_ball.launch.py'])
        ),
 
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("monicar3_control"), '/launch', '/lowlevel.launch.py'])
        ),
   ])

