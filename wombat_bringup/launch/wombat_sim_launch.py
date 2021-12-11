from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('wombat_bringup')

    # Define paths
    rviz_default_config = PathJoinSubstitution([bringup_dir, 'rviz', 'wombat_default_view.rviz'])
    rviz_launch_script = PathJoinSubstitution([bringup_dir, 'launch', 'rviz_launch.py'])
    unity_launch_script = PathJoinSubstitution([bringup_dir, 'launch', 'unity_bridge_launch.py'])
    wombat_launch_script = PathJoinSubstitution([bringup_dir, 'launch', 'wombat_launch.py'])

    # Create the launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=rviz_default_config,
        description='Full path to the RVIZ config file to use')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Launch ROS TCP endpoint to communicate with Unity simulation
    unity_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(unity_launch_script))

    # Launch Wombat software stack
    wombat_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wombat_launch_script),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'use_sim_time': 'true'}.items())

    # Launch rviz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_script),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': '',
            'rviz_config': rviz_config_file,
            'use_namespace': 'False'}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(unity_bridge_cmd)
    ld.add_action(wombat_cmd)
    ld.add_action(rviz_cmd)

    return ld
