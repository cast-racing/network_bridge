from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    ld = LaunchDescription()

    # Declare launch arguments
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='network_bridge',
        description='Node name'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='Udp1.yaml',
        description='Network bridge parameters'
    )

    local_address_arg = DeclareLaunchArgument(
        'local_address',
        default_value='127.0.0.1',
        description='This machines local IP (e.g. on LAN, VPN, etc)'
    )

    remote_address_arg = DeclareLaunchArgument(
        'remote_address',
        default_value='127.0.0.1',
        description='Remotes IP address'
    )

    receive_port_arg = DeclareLaunchArgument(
        'receive_port',
        default_value='5000',
        description='Desired receive port (must be send port of remote)'
    )

    send_port_arg = DeclareLaunchArgument(
        'receive_port',
        default_value='5001',
        description='Desired send port (must be receive port of remote)'
    )

    ld.add_action(name_arg)
    ld.add_action(config_arg)
    ld.add_action(local_address_arg)
    ld.add_action(remote_address_arg)
    ld.add_action(receive_port_arg)
    ld.add_action(send_port_arg)

    config = PathJoinSubstitution([FindPackageShare('network_bridge'), 
                                   'config',
                                   LaunchConfiguration('config')])
    
    node = Node(
        package='network_bridge',
        executable='network_bridge',
        name=LaunchConfiguration('name'),
        output='screen',
        parameters=[config,
                    {'UdpInterface.local_address': LaunchConfiguration('local_address')},
                    {'UdpInterface.remote_address': LaunchConfiguration('remote_address')},
                    {'UdpInterface.receive_port': LaunchConfiguration('receive_port')},
                    {'UdpInterface.send_port': LaunchConfiguration('send_port')},
                    ],
        )
    
    ld.add_action(node)

    return ld