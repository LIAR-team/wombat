# Copyright 2024 Soragna Alberto.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the share directories
    kennel_share_dir = get_package_share_directory('kennel')
    wombat_bringup_share_dir = get_package_share_directory('wombat_bringup')
    wombat_externals_share_dir = get_package_share_directory('wombat_bringup_externals')

    # Define paths
    foxglove_bridge_launch_script = PathJoinSubstitution(
        [wombat_externals_share_dir, 'launch', 'foxglove_bridge_launch.py'])
    nav2_navigation_launch_script = PathJoinSubstitution(
        [wombat_externals_share_dir, 'launch', 'nav2_navigation_launch.py'])
    kennel_launch_script = PathJoinSubstitution(
        [kennel_share_dir, 'launch', 'kennel_launch.py'])

    # Create the launch configuration variables
    kennel_params_file = LaunchConfiguration('kennel_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_foxglove_bridge = LaunchConfiguration('use_foxglove_bridge')

    # Declare the launch arguments
    declare_kennel_params_file_cmd = DeclareLaunchArgument(
        'kennel_params_file',
        default_value=os.path.join(kennel_share_dir, 'config', 'kennel.yaml'),
        description='Full path to the ROS 2 parameters file to use for the kennel nodes')
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(wombat_bringup_share_dir, 'params', 'nav2_kennel_params.yaml'),
        description='Full path to the ROS 2 parameters file to use for the nav2 nodes')
    declare_use_foxglove_bridge = DeclareLaunchArgument(
        'use_foxglove_bridge',
        default_value='True',
        description='Whether to start the Foxglove bridge')

    # Launch Foxglove Bridge
    foxglove_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(foxglove_bridge_launch_script),
        condition=IfCondition(use_foxglove_bridge))

    # Launch kennel
    kennel_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kennel_launch_script),
        launch_arguments={
                          'params_file': kennel_params_file}.items())

    # Launch nav2
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_script),
        launch_arguments={
                          'params_file': nav2_params_file,
                          'use_sim_time': 'True',
                          'autostart': 'True',
                          'use_composition': 'False'}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_kennel_params_file_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_use_foxglove_bridge)

    ld.add_action(foxglove_bridge_cmd)
    ld.add_action(kennel_bringup_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
