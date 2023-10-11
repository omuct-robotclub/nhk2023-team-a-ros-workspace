from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    simulation = LaunchConfiguration("simulation")


    bringup_dir = Path(get_package_share_directory("bringup"))
    config_dir = bringup_dir / "config"
    
    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation", default_value="True"),

            Node(
                package="can_bridge",
                executable="can_bridge",
                parameters=[config_dir / "can_bridge.yaml"],
                condition=IfCondition(PythonExpression(['not ', simulation]))
            ),
            Node(
                package="pfloc",
                executable="pfloc",
            ),
            Node(
                package="emcl2",
                executable="emcl2_node",
                parameters=[config_dir / "emcl2.yaml"],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    {
                        "yaml_filename": str(bringup_dir / "map" / "map.yaml"),
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="nav_map_server",
                parameters=[
                    {
                        "yaml_filename": str(bringup_dir / "map" / "nav_map.yaml"),
                    }
                ],
                remappings=[
                    ("map", "nav_map"),
                    ("map_update", "nav_map_update"),
                ],
            ),
            Node(
                package="nav2_costmap_2d",
                executable="nav2_costmap_2d",
                parameters=[
                    config_dir/"nav2.yaml"
                ]
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                parameters=[{
                    "node_names": ["map_server", "nav_map_server", "costmap/costmap"],
                    "autostart": True,
                }]
            ),
            Node(
                package="laserscan_marger",
                executable="laserscan_marger_node",
                parameters=[{
                    "scan_input_topics": ["scan0", "scan1"],
                    "out_points": 2000,
                    "publish_frequency": 10.0,
                    "out_range_min": 0.5,
                }],
                remappings=[("scan_out", "scan")]
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{
                    "robot_description": Command(f"xacro {bringup_dir/'urdf'/'robot.urdf.xacro'}"),
                    "publish_frequency": 100.0,
                    "ignore_timestamp": True,
                }]
            ),
            Node(package="udp_multicast_beacon", executable="beacon"),
            Node(package="rosbridge_server", executable="rosbridge_websocket"),
        ]
    )
