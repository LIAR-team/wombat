# Copyright 2024 Soragna Alberto.
# All Rights Reserved.
# Unauthorized copying via any medium is strictly prohibited.
# Proprietary and confidential.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    kennel_share_dir = get_package_share_directory('kennel')

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(kennel_share_dir, 'config', 'kennel.yaml'),
        description='Full path to the ROS 2 parameters file to use for the kennel nodes')

    # Launch kennel
    start_kennel_cmd = Node(
        parameters=[
          params_file
        ],
        package='kennel',
        executable='kennel',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_kennel_cmd)

    return ld
