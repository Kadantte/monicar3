#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar3_cv"), '/launch', '/csicam.launch.py'])
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar3_yolo"), '/launch', '/yolo.launch.py'])
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar3_control"), '/launch', '/yolo_chase.launch.py'])
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("monicar3_control"), '/launch', '/lowlevel.launch.py'])
    ),
  ])
