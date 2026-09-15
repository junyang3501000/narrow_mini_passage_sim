"""ROS Odometry 解码和窄道纵向进度管理。

``distance_tracker.py`` 保留纯数学投影；本模块负责 ROS 消息、四元数 yaw、最新样本时间
以及任务开始/结束时的参考位姿管理。ROS 节点只需调用 ingest/reset_task/clear_task。
"""

import math
from dataclasses import dataclass
from typing import Optional

from nav_msgs.msg import Odometry

from .distance_tracker import DistanceTracker, DistanceUpdate


@dataclass(frozen=True)
class OdomPose:
    """从 nav_msgs/Odometry 提取的平面位姿。"""

    x: float
    y: float
    yaw: float


class OdometryProgress:
    """缓存最新 odom，并管理一轮任务的起始航向投影进度。"""

    def __init__(self, max_step: float): # 最大步长
        self._tracker = DistanceTracker(max_step) # 进度跟踪器
        self.latest_pose: Optional[OdomPose] = None # 最新位姿，optional表示可能为空
        self.received_at: Optional[float] = None # 接收时间

    @property
    def started(self) -> bool:
        """当前任务是否已经记录起始位姿。"""

        return self._tracker.started

    @property
    def distance(self) -> float:
        """沿任务起始车头方向的有符号进度。"""

        return self._tracker.distance

    def ingest(self, message: Odometry, received_at: float, tracking: bool) -> Optional[DistanceUpdate]: 
    
        """保存一帧 odom；tracking=True 时同步更新任务进度。"""

        self.received_at = received_at
        pose = self._pose_from_message(message) # 转换为位姿，message是nav_msgs/Odometry类型

        # 检查位姿是否有效
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            # 坏消息不能借用旧位姿却使用新时间戳伪装成“新鲜 odom”。
            self.latest_pose = None
            return (
                DistanceUpdate(False, self.distance, "non_finite_odom") # 拒绝更新，返回距离更新，因为位姿无效
                if tracking
                else None
            )
        self.latest_pose = pose 
        if not tracking: # 如果不跟踪，则返回None
            return None
        return self._tracker.update(pose.x, pose.y, pose.yaw)

    def is_fresh(self, now: float, timeout: float) -> bool:
        """最近 odom 是否存在、不是来自未来且未超过看门狗时间。"""

        return (
            self.received_at is not None
            and self.latest_pose is not None
            and 0.0 <= now - self.received_at <= timeout # 检查是否在时间窗口内，now是当前时间，received_at是接收时间，timeout是超时时间，0.0表示0秒
        )

    def reset_task_if_fresh(self, now: float, timeout: float) -> bool:
        """若缓存位姿仍新鲜，就以它建立本轮起始参考。"""

        self._tracker.clear()
        if not self.is_fresh(now, timeout):
            return False
        self._reset_from_latest()
        return True

    def ensure_started(self) -> bool:
        """控制开始前确保已有起始参考；没有有效位姿时返回 False。"""

        if self.started:
            return True
        if self.latest_pose is None:
            return False
        self._reset_from_latest()
        return True

    def clear_task(self) -> None:
        """清除任务起点和进度，但保留最新 odom 供下一次使能使用。"""

        self._tracker.clear()

    def _reset_from_latest(self) -> None:
        """用最新有效位姿重置纯数学跟踪器。"""

        assert self.latest_pose is not None
        self._tracker.reset(
            self.latest_pose.x,
            self.latest_pose.y,
            self.latest_pose.yaw,
        )

    @staticmethod # 静态方法，不需要实例化就可以调用
    def _pose_from_message(message: Odometry) -> OdomPose:
        """从 ROS 四元数计算平面 yaw；零四元数返回 NaN 供上层拒绝。"""

        # 获取位置和方向
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation # 获取方向
        norm_squared = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if norm_squared <= 1.0e-12: # 如果模长小于1.0e-12，则返回NaN，表示无效
            yaw = float("nan")
        else:
            # 先归一化，避免驱动四元数的微小模长误差污染 yaw。
            inverse_norm = 1.0 / math.sqrt(norm_squared)
            qx = orientation.x * inverse_norm
            qy = orientation.y * inverse_norm
            qz = orientation.z * inverse_norm
            qw = orientation.w * inverse_norm
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
        # 返回位姿，position.x和position.y是位置，yaw是航向
        return OdomPose(float(position.x), float(position.y), yaw)
