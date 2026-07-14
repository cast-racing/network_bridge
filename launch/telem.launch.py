"""Launch a configured UDP telemetry bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("network_bridge"), "config", LaunchConfiguration("config")]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "name", default_value="network_bridge", description="Node name"
            ),
            DeclareLaunchArgument("config", description="Bridge parameter filename"),
            DeclareLaunchArgument(
                "local_address", description="Local interface IP address"
            ),
            DeclareLaunchArgument(
                "remote_address", description="Remote bridge IP address"
            ),
            DeclareLaunchArgument(
                "receive_port",
                description="Local receive port (the remote bridge's send port)",
            ),
            DeclareLaunchArgument(
                "send_port",
                description="Local send port (the remote bridge's receive port)",
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name=LaunchConfiguration("name"),
                output="screen",
                parameters=[
                    config,
                    {
                        "UdpInterface.local_address": LaunchConfiguration(
                            "local_address"
                        ),
                        "UdpInterface.remote_address": LaunchConfiguration(
                            "remote_address"
                        ),
                        "UdpInterface.receive_port": LaunchConfiguration(
                            "receive_port"
                        ),
                        "UdpInterface.send_port": LaunchConfiguration("send_port"),
                    },
                ],
            ),
        ]
    )
