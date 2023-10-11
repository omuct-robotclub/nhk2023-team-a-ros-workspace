from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_bridge',
            executable='can_bridge',
            parameters=[
                Path(get_package_share_directory('bringup'))/'config'/'can_bridge.yaml'
            ]
        ),
        Node(
            package='udp_multicast_beacon',
            executable='beacon'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket'
        )
    ])
