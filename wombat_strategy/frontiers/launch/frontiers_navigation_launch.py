# Copyright 2021-2022 Soragna Alberto.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    share_dir = get_package_share_directory('wombat_strategy')
    frontiers_params_file = os.path.join(share_dir, 'config', 'frontier_exploration_params.yaml')

    # Launch exploration server
    start_exploration_server = launch_ros.actions.Node(
        parameters=[
          frontiers_params_file
        ],
        package='wombat_strategy',
        executable='exploration_server',
        name='frontier_navigation',
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_exploration_server)

    return ld
