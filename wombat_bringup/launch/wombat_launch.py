from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directories
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    wombat_bringup_dir = get_package_share_directory('wombat_bringup')

    # Define paths
    wombat_default_config = PathJoinSubstitution(
        [wombat_bringup_dir, 'config', 'wombat_params.yaml'])
    slam_launch_script = PathJoinSubstitution(
        [slam_toolbox_dir, 'launch', 'online_async_launch.py'])

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=wombat_default_config,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time},
        convert_types=True)

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
          package='wombat_apps',
          executable='wombat_nav',
          output='screen',
          parameters=[configured_params, {'autostart': True}],
          remappings=remappings),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_script),
            launch_arguments={
                'use_sim_time': use_sim_time}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(bringup_cmd_group)

    return ld
