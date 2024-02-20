# Copyright 2024 Soragna Alberto.
# All Rights Reserved.
# Unauthorized copying via any medium is strictly prohibited.
# Proprietary and confidential.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    kennel_share_dir = get_package_share_directory('kennel')
    wombat_externals_share_dir = get_package_share_directory('wombat_bringup_externals')

    # Define paths
    foxglove_bridge_launch_script = PathJoinSubstitution(
        [wombat_externals_share_dir, 'launch', 'foxglove_bridge_launch.py'])

    # Create the launch configuration variables
    use_foxglove_bridge = LaunchConfiguration('use_foxglove_bridge')
    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_use_foxglove_bridge = DeclareLaunchArgument(
        'use_foxglove_bridge',
        default_value='True',
        description='Whether to start the Foxglove bridge')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(kennel_share_dir, 'config', 'kennel.yaml'),
        description='Full path to the ROS 2 parameters file to use for the kennel nodes')

    # Launch Foxglove Bridge
    foxglove_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(foxglove_bridge_launch_script),
        condition=IfCondition(use_foxglove_bridge))

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

    ld.add_action(declare_use_foxglove_bridge)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(foxglove_bridge_cmd)
    ld.add_action(start_kennel_cmd)

    return ld
