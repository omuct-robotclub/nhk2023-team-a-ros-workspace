from geometry_msgs.msg import Quaternion, PoseStamped
from math import cos, sin, atan2, hypot, radians
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
#     """
#     Converts euler roll, pitch, yaw to quaternion (w in last place)
#     quat = [x, y, z, w]
#     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#     """
#     cy = cos(yaw * 0.5)
#     sy = sin(yaw * 0.5)
#     cp = cos(pitch * 0.5)
#     sp = sin(pitch * 0.5)
#     cr = cos(roll * 0.5)
#     sr = sin(roll * 0.5)

#     q = Quaternion()
#     q.x = cy * cp * cr + sy * sp * sr
#     q.y = cy * cp * sr - sy * sp * cr
#     q.z = sy * cp * sr + cy * sp * cr
#     q.w = sy * cp * cr - cy * sp * sr

#     return q

def get_yaw(q: Quaternion) -> float:
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def new_pose(x: float, y: float, yaw: float) -> PoseStamped:
    result = PoseStamped()
    result.header.frame_id = "map"
    result.header.stamp.sec = 0
    result.header.stamp.nanosec = 0
    result.pose.position.x = x
    result.pose.position.y = y
    q = quaternion_from_euler(0.0, 0.0, radians(yaw))
    result.pose.orientation.x = q[0]
    result.pose.orientation.y = q[1]
    result.pose.orientation.z = q[2]
    result.pose.orientation.w = q[3]
    return result

def distance(a: PoseStamped, b: PoseStamped) -> float:
    return hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)
