#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  cv_parameter = LaunchConfiguration(
    'cv_parameter',
    default=os.path.join(
      get_package_share_directory('monicar3_cv'),
      'param/camera.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('cv_parameter', default_value=cv_parameter),

    Node(
      package='monicar3_cv', executable='usbcam_pub', name='usbcam_node',
      output='screen', emulate_tty=True,
      parameters=[cv_parameter],
    ),
  ])
