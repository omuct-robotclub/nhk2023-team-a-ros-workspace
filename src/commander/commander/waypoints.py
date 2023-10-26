from __future__ import annotations
from geometry_msgs.msg import PoseStamped
from commander.utils import new_pose, get_yaw
from typing import Optional, Tuple
from math import radians, degrees, sin, cos, pi
from enum import Enum, auto


class BBox:
    def __init__(self, x: float, y: float, width: float, height: float):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    
    def contains(self, x: float, y: float) -> bool:
        return self.x <= x <= self.x + self.width and self.y <= y <= self.y + self.height


class Side(Enum):
    RIGHT = auto()
    LEFT = auto()

class Goal(Enum):
    CENTER = auto()
    HOME = auto()
    RUN = auto()

class Step(Enum):
    CENTER = auto()
    RUN = auto()


class Waypoint:
    def __init__(self, name: str, pose: PoseStamped, must_pass: bool = False, bbox: Optional[BBox] = None, use_as_start_pose: bool = False, kind = Goal.RUN, step_kind: Optional[Step] = None):
        assert((not use_as_start_pose) or (use_as_start_pose and bbox is not None))

        self.name = name
        self.pose = pose
        self.bbox = bbox
        self.must_pass = must_pass
        self.use_as_start_pose = use_as_start_pose
        self.kind = kind
        self.step_kind = step_kind

        self.right: Optional[Waypoint] = None
        self.left: Optional[Waypoint] = None
        self.center: Optional[Waypoint] = None
        self.home: Optional[Waypoint] = None
    
    def mirror(self, name: str, cx: float = 4.5, cy: float = 4.5) -> Waypoint:
        return Waypoint(
            name,
            new_pose(2 * cx - self.pose.pose.position.x, 2 * cy - self.pose.pose.position.y, degrees(get_yaw(self.pose.pose.orientation) + pi)),
            self.must_pass,
            None if self.bbox is None else BBox(
                2 * cx - self.bbox.x - self.bbox.width, 2 * cy - self.bbox.y - self.bbox.height, self.bbox.width, self.bbox.height),
            self.use_as_start_pose,
            self.kind,
            self.step_kind
        )

    def next(self, goal: Goal, side: Side) -> Optional[Waypoint]:
        if goal != Goal.RUN and self.kind == goal:
            return None
        elif goal == Goal.CENTER and self.center is not None:
            return self.center
        elif goal == Goal.HOME and self.home is not None:
            return self.home
        elif side == Side.RIGHT and self.right is not None:
            return self.right
        elif side == Side.LEFT and self.left is not None:
            return self.left
        else:
            for wp in [self.right, self.left, self.center, self.home]:
                if wp is not None:
                    return wp
            raise ValueError("no way!")


start_zone = Waypoint("start_zone", new_pose(-0.5, 0.5, 0.0), True, BBox(-1.0, 0.0, 1.0, 1.0), use_as_start_pose=True, kind=Goal.HOME)

first_corner = Waypoint("first_corner", new_pose(1.5, 1.5, 0.0), False, BBox(0.0, 0.0, 3.0, 3.0))

first_step_right = Waypoint("first_step_right", new_pose(4.2, 0.725, 0.0), True, BBox(3.0, 0.0, 2.5, 1.5), step_kind=Step.RUN)
first_step_left = Waypoint("first_step_left", new_pose(4.2, 2.25, 0.0), True, BBox(3.0, 1.5, 2.5, 1.5), step_kind=Step.RUN)

second_corner = Waypoint("second_corner", new_pose(7.5, 1.5, 0.0), False, BBox(5.5, 0.0, 3.5, 3.0))

first_rope_right = Waypoint("first_rope_right", new_pose(8.25, 4.5, 90), True, BBox(7.5, 3.0, 1.5, 3.0))
first_rope_left = Waypoint("first_rope_left", new_pose(6.75, 4.5, 90), True, BBox(6.0, 3.0, 1.5, 3.0))
first_center_entrance = Waypoint("first_center_entrance", new_pose(6.5, 4.5, 180), True, step_kind=Step.CENTER)

third_corner = first_corner.mirror("third_corner")

second_step_right = first_step_right.mirror("second_step_right")
second_step_left = first_step_right.mirror("second_step_left")

forth_corner = second_corner.mirror("forth_corner")

second_rope_right = first_rope_right.mirror("second_rope_right")
second_rope_left = first_rope_left.mirror("second_rope_left")
second_center_entrance = first_center_entrance.mirror("second_center_entrance")

start_zone_entrance = Waypoint("start_zone_entrance", new_pose(1.0, 1.0, 0.0), True)

center_zone = Waypoint("center_zone", new_pose(4.5, 4.5, 0.0), False, BBox(3.0, 3.0, 3.0, 3.0), kind=Goal.CENTER)
near_center_exit = Waypoint("near_center_exit", new_pose(3.6, 4.5, 180), True, step_kind=Step.CENTER)
far_center_exit = near_center_exit.mirror("far_center_exit")

all_waypoints = [
    start_zone,
    first_corner,
    first_step_right,
    first_step_left,
    second_corner,
    first_rope_right,
    first_rope_left,
    first_center_entrance,
    third_corner,
    second_step_right,
    second_step_left,
    forth_corner,
    second_rope_right,
    second_rope_left,
    second_center_entrance,
    start_zone_entrance,
    center_zone,
    near_center_exit,
    far_center_exit,
]


start_zone.right = first_corner

first_corner.right = first_step_right
first_corner.left = first_step_left

first_step_left.right = second_corner
first_step_right.right = second_corner

second_corner.right = first_rope_right
second_corner.left = first_rope_left
second_corner.center = first_rope_left

first_center_entrance.right = center_zone

center_zone.right = near_center_exit
near_center_exit.right = second_rope_left
center_zone.left = far_center_exit
far_center_exit.right = first_rope_left

first_rope_right.right = third_corner
first_rope_left.right = third_corner
first_rope_left.center = first_center_entrance

third_corner.right = second_step_right
third_corner.left = second_step_left

second_step_right.right = forth_corner
second_step_left.right = forth_corner

forth_corner.right = second_rope_right
forth_corner.left = second_rope_left
forth_corner.center = second_rope_left

second_center_entrance.right = center_zone

second_rope_left.right = first_corner
second_rope_left.home = start_zone_entrance
second_rope_left.center = second_center_entrance

second_rope_right.right = first_corner
second_rope_right.home = start_zone_entrance

start_zone_entrance.right = start_zone


class WaypointNavigator:
    def __init__(self):
        self.path = None
        self.side = Side.LEFT
    
    @staticmethod
    def find_waypoint_at(pose: PoseStamped) -> Optional[Waypoint]:
        p = pose.pose.position
        for wp in all_waypoints:
            if wp.bbox is not None:
                print(p.x, p.y, wp.bbox.x, wp.bbox.y, wp.bbox.width, wp.bbox.height)
            if wp.bbox is not None and wp.bbox.contains(p.x, p.y):
                return wp
        return None

    def go_from_to(self, robot_pose: PoseStamped, goal: Goal) -> Tuple[PoseStamped, list[Waypoint], Optional[Waypoint]]:
        start = self.find_waypoint_at(robot_pose)
        if start is None:
            print("start does not found")
            return robot_pose, [], None
        start_wp, path = self.generate_path_to(start, goal, self.side)
        start_pose = start_wp.pose if start_wp is not None else robot_pose
        if start.step_kind is not None:
            return start_pose, [start], path[0] if len(path) > 0 else None
        result = []
        for i, wp in enumerate(path):
            is_last = i == len(path) - 1
            if (not wp.must_pass) and (not is_last):
                continue
            print("name", wp.name)
            print(wp.step_kind)
            result.append(wp)
            if wp.step_kind is not None:
                if is_last:
                    return start_pose, result, None
                else:
                    print("pre get over")
                    return start_pose, result, path[i+1]
        return start_pose, result, None
        # return True

    # def next_path(self) -> list[Waypoint]:
    #     if self.path is None: return []
    #     result = []
    #     step_idx = 0
    #     for i, wp in enumerate(self.path):
    #         result.append(wp)
    #         if wp.is_pre_step:
    #             step_idx = i
    #     self.path = self.path[step_idx:-1]
    #     return result

    def generate_path_to(self, start: Waypoint, goal: Goal, side: Side) -> Tuple[Optional[Waypoint], list[Waypoint]]:
        result = []
        # if start.use_as_start_pose:
        #     result.append(start)
        next = start
        while True:
            print(next.name)
            next = next.next(goal, side)
            if next is None:
                break
            else:
                result.append(next)
        return start if start.use_as_start_pose else None, result
