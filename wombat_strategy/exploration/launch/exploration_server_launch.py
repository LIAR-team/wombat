# Copyright 2021-2022 Soragna Alberto.
# All Rights Reserved.
# Unauthorized copying via any medium is strictly prohibited.
# Proprietary and confidential.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    share_dir = get_package_share_directory('wombat_strategy')
    params_file = os.path.join(share_dir, 'config', 'frontier_exploration_params.yaml')

    return LaunchDescription([
        launch_ros.actions.Node(
          parameters=[
            params_file
          ],
          package='wombat_strategy',
          executable='exploration_server',
          name='frontier_exploration',
          output='screen'
        )
    ])
