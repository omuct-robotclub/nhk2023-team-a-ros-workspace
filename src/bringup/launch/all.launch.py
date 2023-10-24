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
            # Hardware
            Node(
                package="can_bridge",
                executable="can_bridge",
                parameters=[config_dir / "can_bridge.yaml"],
                condition=IfCondition(PythonExpression(["not ", simulation])),
            ),
            Node(
                package="ldlidar",
                executable="ldlidar",
                name="lidar0_node",
                parameters=[
                    {
                        "serial_port_candidates": [
                            "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1:1.0-port0"
                        ],
                        "laser_frame_id": "lidar0_link",
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
                            "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1:1.0-port0"
                        ],
                        "laser_frame_id": "lidar0_link",
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
                package="pfloc",
                executable="pfloc",
                parameters=[{"use_odom_pose": True}],
            ),
            Node(
                package="emcl2",
                executable="emcl2_node",
                parameters=[config_dir / "emcl2.yaml"],
                remappings=[("odom", "odom_filtered")],
            ),
            # Navigation
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                parameters=[
                    config_dir / "nav2.yaml",
                    {
                        "default_nav_through_poses_bt_xml": str(
                            bringup_dir / "behavior_tree" / "nav_through_poses.xml"
                        ),
                        "default_nav_to_pose_bt_xml": str(
                            bringup_dir / "behavior_tree" / "nav_to_pose.xml"
                        ),
                    },
                ],
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
                package="nav2_planner",
                executable="planner_server",
                parameters=[config_dir / "nav2.yaml"],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                parameters=[config_dir / "nav2.yaml"],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                parameters=[
                    {
                        "node_names": [
                            "bt_navigator",
                            "map_server",
                            "nav_map_server",
                            "planner_server",
                            "controller_server",
                        ],
                        "autostart": True,
                    }
                ],
            ),
            # Connection
            Node(package="udp_multicast_beacon", executable="beacon"),
            Node(package="rosbridge_server", executable="rosbridge_websocket"),
        ]
    )
