# Copyright 2021-2022 Soragna Alberto.
# All Rights Reserved.
# Unauthorized copying via any medium is strictly prohibited.
# Proprietary and confidential.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share_dir = get_package_share_directory('wombat_strategy')
    frontiers_params_file = os.path.join(share_dir, 'config', 'frontier_exploration_params.yaml')

    wombat_externals_dir = get_package_share_directory('wombat_bringup_externals')
    nav2_launch_script = PathJoinSubstitution(
        [wombat_externals_dir, 'launch', 'nav2_navigation_launch.py'])
    nav2_params_file = os.path.join(wombat_externals_dir, 'params', 'nav2_params.yaml')

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_script),
        launch_arguments={
                          'params_file': nav2_params_file,
                          'use_sim_time': 'True',
                          'autostart': 'True',
                          'use_composition': 'False'}.items())

    # Launch exploration server
    start_exploration_server = launch_ros.actions.Node(
        parameters=[
          frontiers_params_file
        ],
        package='wombat_strategy',
        executable='exploration_server',
        name='frontier_exploration',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_exploration_server)
    ld.add_action(nav2_bringup_cmd)

    return ld
