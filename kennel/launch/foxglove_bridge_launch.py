from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Launch foxglove bridge
    start_foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765,
                     'address': "0.0.0.0",
                     'tls': False,
                     'certfile': "",
                     'keyfile': "",
                     'topic_whitelist': ['.*'],
                     'param_whitelist': ['.*'],
                     'service_whitelist': ['.*'],
                     'client_topic_whitelist': ['.*'],
                     'min_qos_depth': 1,
                     'max_qos_depth': 10,
                     'num_threads': 0,
                     'send_buffer_limit': 10000000,
                     'use_sim_time': False,
                     'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 'services', 'connectionGraph', 'assets'],
                     'asset_uri_allowlist': ['^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$'],
                     'include_hidden': False }],
        arguments=['--ros-args', '--log-level', 'error'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(start_foxglove_bridge)

    return ld
