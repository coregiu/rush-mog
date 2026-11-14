"""2D 位姿与四元数工具。

本模块被 edge_server 的 SLAM / 导航 / 目标定位链路共用。纯数学函数不依赖
ROS2，只有在构造 ROS 消息时才延迟导入，方便单元测试。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

TWO_PI = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi)。"""
    return (float(angle) + math.pi) % TWO_PI - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> 平面 yaw（rad）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """平面 yaw -> 四元数 (x, y, z, w)。"""
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass(frozen=True)
class Pose2D:
    """平面位姿。frame_id 只作语义标记，不参与数学运算。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    frame_id: str = "map"

    @property
    def xy(self) -> Tuple[float, float]:
        return float(self.x), float(self.y)

    def as_tuple(self) -> Tuple[float, float, float]:
        return float(self.x), float(self.y), float(self.yaw)

    def is_finite(self) -> bool:
        return all(
            math.isfinite(float(v)) for v in (self.x, self.y, self.yaw)
        )


def pose_from_msg(msg, frame_id: Optional[str] = None) -> Pose2D:
    """geometry_msgs/Pose|PoseStamped|PoseWithCovariance[Stamped]|Odometry -> Pose2D。"""
    frame = frame_id
    if hasattr(msg, "pose") and hasattr(msg, "header"):
        frame = frame or msg.header.frame_id
        msg = msg.pose
    # PoseWithCovariance / Odometry.pose -> Pose
    if hasattr(msg, "pose") and hasattr(msg.pose, "position"):
        msg = msg.pose
    if hasattr(msg, "position") and hasattr(msg, "orientation"):
        q = msg.orientation
        return Pose2D(
            msg.position.x,
            msg.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
            frame or "map",
        )
    raise TypeError(f"unsupported pose message: {type(msg)!r}")


def compose_pose(parent: Pose2D, child: Pose2D) -> Pose2D:
    """parent 在 F 系，child 在 parent 系 -> child 在 F 系。"""
    c, s = math.cos(parent.yaw), math.sin(parent.yaw)
    return Pose2D(
        parent.x + c * child.x - s * child.y,
        parent.y + s * child.x + c * child.y,
        wrap_angle(parent.yaw + child.yaw),
        parent.frame_id,
    )


def inverse_pose(pose: Pose2D) -> Pose2D:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return Pose2D(
        -(c * pose.x + s * pose.y),
        -(-s * pose.x + c * pose.y),
        wrap_angle(-pose.yaw),
        pose.frame_id,
    )


def transform_point(pose: Pose2D, x: float, y: float) -> Tuple[float, float]:
    """把 pose 系下的点变换到 pose.frame_id 系。"""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def to_body_frame(pose: Pose2D, x: float, y: float) -> Tuple[float, float]:
    """把 pose.frame_id 系下的点变换到 pose 系。"""
    dx, dy = x - pose.x, y - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def point_distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


def angle_diff(target: float, current: float) -> float:
    return wrap_angle(target - current)


def make_pose_stamped(frame_id: str, pose: Pose2D, stamp=None):
    """构造 geometry_msgs/PoseStamped；stamp 为空时使用零时间戳。"""
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = frame_id or pose.frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.pose.position.x = float(pose.x)
    msg.pose.position.y = float(pose.y)
    msg.pose.position.z = 0.0
    qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def make_pose_with_covariance(frame_id: str, pose: Pose2D, stamp=None, var_xy=0.25):
    """构造 geometry_msgs/PoseWithCovarianceStamped，用于 /initialpose。"""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id or pose.frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.pose.pose.position.x = float(pose.x)
    msg.pose.pose.position.y = float(pose.y)
    msg.pose.pose.position.z = 0.0
    qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    # 6x6 行优先协方差：x, y, z, roll, pitch, yaw
    cov = msg.pose.covariance
    cov[0] = float(var_xy)
    cov[7] = float(var_xy)
    cov[35] = 0.05
    return msg
