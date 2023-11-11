from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    simulation = LaunchConfiguration("simulation")
    enable_wall_tracing = LaunchConfiguration("enable_wall_tracing")

    bringup_dir = Path(get_package_share_directory("bringup"))
    config_dir = bringup_dir / "config"

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation", default_value="False"),
            DeclareLaunchArgument("enable_wall_tracing", default_value="True"),

            # Hardware
            Node(
                package="can_bridge",
                executable="can_bridge",
                parameters=[config_dir / "can_bridge.yaml"],
                condition=IfCondition(PythonExpression(["not ", simulation]))
            ),
            Node(
                package="ldlidar",
                executable="ldlidar",
                name="lidar0_node",
                parameters=[
                    {
                        "serial_port_candidates": [
                            "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2:1.0-port0"
                        ],
                        "lidar_frame": "lidar0_link",
                    }
                ],
                remappings=[("scan", "scan0")],
                condition=IfCondition(PythonExpression(["not ", simulation])),
            ),
            Node(
                package="ldlidar",
                executable="ldlidar",
                name="lidar1_node",
                parameters=[
                    {
                        "serial_port_candidates": [
                            "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0"
                        ],
                        "lidar_frame": "lidar1_link",
                    }
                ],
                remappings=[("scan", "scan1")],
                condition=IfCondition(PythonExpression(["not ", simulation])),
            ),
            # Navigation
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            f"xacro {bringup_dir/'urdf'/'robot.urdf.xacro'}"
                        ),
                        "publish_frequency": 100.0,
                        "ignore_timestamp": True,
                    }
                ],
            ),
            Node(
                package="laserscan_marger",
                executable="laserscan_marger_node",
                parameters=[
                    {
                        "publish_frequency": 10.0,
                        "out_points": 1000,
                        "scan_input_topics": ["scan0", "scan1"],
                        "scan_timeout": 0.5,
                        "out_range_min": 0.5,
                    }
                ],
                remappings=[("scan_out", "scan")]
            ),
            Node(
                package="wall_tracer",
                executable="wall_tracer",
                parameters=[config_dir / "wall_tracer.yaml"],
                condition=IfCondition(enable_wall_tracing)
            ),
            # Connection
            Node(package="udp_multicast_beacon", executable="beacon"),
            Node(package="rosbridge_server", executable="rosbridge_websocket"),
        ]
    )
