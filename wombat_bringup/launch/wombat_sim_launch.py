import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the launch directory
    wombat_bringup_dir = get_package_share_directory('wombat_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Define paths
    rviz_launch_script = PathJoinSubstitution(
        [wombat_bringup_dir, 'launch', 'rviz_launch.py'])
    unity_launch_script = PathJoinSubstitution(
        [wombat_bringup_dir, 'launch', 'unity_bridge_launch.py'])
    nav2_launch_script = PathJoinSubstitution(
        [nav2_bringup_dir, 'launch', 'bringup_launch.py'])

    # Create the launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(wombat_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Launch ROS TCP endpoint to communicate with Unity simulation
    unity_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(unity_launch_script))

    # Launch RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_script),
        condition=IfCondition(use_rviz))

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_script),
        launch_arguments={'params_file': params_file,
                          'slam': 'True',
                          'map': '',
                          'use_sim_time': 'True',
                          'autostart': 'True',
                          'use_composition': 'False'}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(unity_bridge_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
