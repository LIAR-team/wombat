from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # Launch ROS TCP endpoint to communicate with Unity simulation
    ros_tcp_endpoint_cmd = Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            emulate_tty=True,
            parameters=[
                {'ROS_IP': '0.0.0.0'},
                {'ROS_TCP_PORT': 10000}])

    # Launch handler to shutdown everything if the TCP endpoint exits
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros_tcp_endpoint_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='ros tcp endpoint exited'))))

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(ros_tcp_endpoint_cmd)
    ld.add_action(exit_event_handler)

    return ld
