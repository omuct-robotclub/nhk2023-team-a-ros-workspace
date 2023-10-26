from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_system_default
from rclpy.time import Time, Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateThroughPoses, FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, TwistStamped, TransformStamped
from std_msgs.msg import UInt8, String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_py
from math import sin, cos, radians, atan2, hypot, degrees
from typing import cast, Optional, Union
from commander.utils import distance, get_yaw, quaternion_from_euler, new_pose
from commander.waypoints import Waypoint, WaypointNavigator, Side, Goal, Step
import sys
import asyncio


# class Pose2D:
#     def __init__(self, x: float = 0.0, y: float = 0.0, t: Optional[float] = None):
#         self.x = x
#         self.y = y
#         self.t = t

#     def distance(self, msg) -> float:
#         if isinstance(msg, PoseStamped):
#             msg = Pose2D.from_msg(msg)
#         return hypot(msg.x - self.x, msg.y - self.y)

#     def to_msg(self) -> PoseStamped:
#         result = PoseStamped()
#         result.header.frame_id = "map"
#         result.header.stamp.sec = 0
#         result.header.stamp.nanosec = 0
#         result.pose.position.x = self.x
#         result.pose.position.y = self.y
#         result.pose.orientation = quaternion_from_euler(0.0, 0.0, 0.0 if self.t is None else self.t)
#         return result

#     @staticmethod
#     def from_msg(msg: PoseStamped):
#         result = Pose2D()
#         result.x = msg.pose.position.x
#         result.y = msg.pose.position.y
#         result.t = get_yaw(msg.pose.orientation)
#         return result


class Commander(Node):
    def __init__(self):
        super().__init__("commander", parameter_overrides=[])
        self.last_goal = None
        self.wp_nav = WaypointNavigator()
        self.nav = BasicNavigator()
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "nav_goal_pose", self.on_goal_pose, qos_profile_system_default
        )
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self, spin_thread=True)
        self.manual_cmd_vel = self.create_subscription(
            TwistStamped,
            "manual_cmd_vel",
            self.manual_cmd_vel_callback,
            qos_profile_system_default,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", qos_profile_system_default
        )
        self.cmd_vel_stamped_pub = self.create_publisher(
            TwistStamped, "cmd_vel_stamped", qos_profile_system_default
        )
        self.side_sub = self.create_subscription(
            UInt8, "side", self.side_callback, qos_profile_system_default
        )
        self.nav_command_callback_group = ReentrantCallbackGroup()
        self.goal_name_sub = self.create_subscription(
            String,
            "goal_name",
            self.goal_name_callback,
            qos_profile_system_default,
            callback_group=self.nav_command_callback_group,
        )

    def get_robot_pose(self) -> Optional[PoseStamped]:
        try:
            now = self.get_clock().now()
            tf_fut = self.tf.wait_for_transform_async(
                "map", "base_link", now,
            )
            rclpy.spin_until_future_complete(self, tf_fut)
            tf_fut.result()
            tf = self.tf.lookup_transform("map", "base_link", now, timeout=Duration(seconds=1))
            # print(tf_fut)
            # tf =  tf_fut.result()
            # print(tf)
            # assert(isinstance(tf, TransformStamped))
        except Exception:
            return None
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def manual_cmd_vel_callback(self, msg: TwistStamped) -> None:
        self.nav.cancelTask()
        self.cmd_vel_stamped_pub.publish(msg)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        self.process(msg)

    def compute_path(
        self, start: PoseStamped, waypoints: list[Waypoint]
    ) -> Optional[Path]:
        tolerance = 0.01
        poses = [
            wp.pose
            for i, wp in enumerate(waypoints)
            if wp.must_pass or i == len(waypoints) - 1
        ]
        print(start.pose.position)
        path = cast(Path, self.nav.getPathThroughPoses(start, poses, use_start=True))
        if path is None:
            self.get_logger().warn("Failed to compute path")
            return

        yaw = None
        waypoint_index = 0
        for pose in path.poses:
            if (
                waypoint_index < len(poses)
                and distance(poses[waypoint_index], pose) < tolerance
            ):
                yaw = get_yaw(poses[waypoint_index].pose.orientation)
                waypoint_index += 1
            if yaw is not None:
                pose.pose.orientation = new_pose(
                    0.0, 0.0, degrees(yaw)
                ).pose.orientation
            # print(degrees(get_yaw(pose.pose.orientation)))
        return path

    def side_callback(self, msg: UInt8) -> None:
        if msg.data == 0:
            self.wp_nav.side = Side.LEFT
        else:
            self.wp_nav.side = Side.RIGHT
    
    def get_over(self, step_kind: Step) -> None:
        print("get over")
        duration_sec = 2.0 if step_kind == Step.RUN else 1.0
        timeout = self.get_clock().now() + Duration(nanoseconds=int(duration_sec*1e9))
        while True:
            now = self.get_clock().now()
            diff = timeout - self.get_clock().now()
            assert(isinstance(diff, Duration))
            wait_sec = min(0.1, diff.nanoseconds * 1e-9)
            if wait_sec <= 0: break
            twist = Twist()
            twist.linear.x = 1.0
            self.cmd_vel_pub.publish(twist)
            print("pub")
            self.get_clock().sleep_for(Duration(nanoseconds=int(wait_sec * 1e9)))
        self.cmd_vel_pub.publish(Twist())
        print("get over done")

    def goal_name_callback(self, msg: String) -> None:
        print("wut")
        goal = (
            Goal.CENTER if msg.data == "center"
            else Goal.HOME if msg.data == "home"
            else Goal.RUN if msg.data == "run"
            else None
        )
        if goal == None:
            return
        # if not self.nav.isTaskComplete() and self.last_goal == goal:
        #     return
        self.last_goal = goal
        self.nav.cancelTask()
        while True:
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                return
            start_pose, wp_list, next_wp = self.wp_nav.go_from_to(robot_pose, goal)
            path = self.compute_path(start_pose, wp_list)
            if path is None:
                return
            self.nav.followPath(path)
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                if next_wp is None:
                    break
                # TODO
                # print(next_wp.name)
                assert(wp_list[-1].step_kind is not None)
                self.get_over(wp_list[-1].step_kind)
                break
            elif result == TaskResult.FAILED:
                break
            else:
                return
        print("goal name callback done")

    def process(self, msg: PoseStamped) -> None:
        robot_pose = self.get_robot_pose()
        if robot_pose == None:
            return
        path = self.compute_path(robot_pose, [Waypoint("dummy", msg, True)])
        if path is None:
            self.get_logger().warn("Failed to compute path")
            return
        self.nav.followPath(path)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
