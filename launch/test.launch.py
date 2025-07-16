import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    # Network Bridge (Broadcaster)    
    launch_include = TimerAction(
    period=0.0,
    actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('network_bridge'),
                    'launch', 'telem.launch.py'
                )
            ),
            launch_arguments={
                    'name' : 'telem_broadcaster',
                    'config' : 'telem_broadcaster.yaml',
                    'local_address' : '127.0.0.1',
                    'receive_port' : '15991',
                    'remote_address' : '100.95.88.121',
                    'send_port' : '15990',

                }.items()
        )
    ]
    )
    ld.add_action(launch_include)

    # Network Bridge (Listener)
    launch_include = TimerAction(
    period=0.0,
    actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('network_bridge'),
                    'launch', 'telem.launch.py'
                )
            ),
            launch_arguments={
                    'name' : 'telem_listener',
                    'config' : 'telem_listener.yaml',
                    'local_address' : '100.95.88.121',
                    'receive_port' : '15990',
                    'remote_address' : '127.0.0.1',
                    'send_port' : '15991',
                }.items()
        )
    ]
    )
    ld.add_action(launch_include)

    # Foxglove Bridge
    launch_include = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('foxglove_bridge_launch'),
                        'launch', 'foxglove_bridge_launch.xml')
                )
            )
        ]
    )
    ld.add_action(launch_include)

    return ld