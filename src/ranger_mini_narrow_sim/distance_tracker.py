"""不依赖 ROS 的窄道纵向进度计算器。

开始任务时记录 odom 起点和起始航向；后续把“当前位置减起点”的位移投影到该方向。
横移不会虚增通过距离，倒车会减少进度。相邻样本仍做跳变检查，防止定位重置直接成功。
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class DistanceUpdate:
    """每次输入 odom 位置后的结果；拒绝时 distance 保持原值。"""

    accepted: bool
    distance: float
    reason: str = ""


class DistanceTracker:
    """计算相对起始车头方向的纵向进度，并拒绝单帧位置跳变。"""

    def __init__(self, max_step: float):
        # max_step 是“相邻两帧”阈值，不是整次任务距离上限。
        if max_step <= 0.0:
            raise ValueError("max_step must be positive")
        self._max_step = float(max_step)
        self._start: Optional[Tuple[float, float]] = None
        self._forward: Optional[Tuple[float, float]] = None
        self._last: Optional[Tuple[float, float]] = None
        self._distance = 0.0

    @property
    def started(self) -> bool:
        """是否已经记录本轮任务的里程起点。"""

        return self._last is not None

    @property
    def distance(self) -> float:
        """返回沿任务起始航向的有符号纵向进度，单位 m。"""

        return self._distance

    def reset(self, x: float, y: float, yaw: float) -> None:
        """记录有限的 odom 起点和起始航向，并把纵向进度清零。"""

        x_value = float(x)
        y_value = float(y)
        yaw_value = float(yaw)
        if not all(math.isfinite(value) for value in (x_value, y_value, yaw_value)):
            raise ValueError("odometry start pose must be finite")
        self._start = (x_value, y_value)
        # odom 坐标中的起始车头单位向量；整轮任务固定，不随后续转向旋转参考轴。
        self._forward = (math.cos(yaw_value), math.sin(yaw_value))
        self._last = (x_value, y_value)
        self._distance = 0.0

    def clear(self) -> None:
        """彻底清除起始位姿和进度。"""

        self._start = None
        self._forward = None
        self._last = None
        self._distance = 0.0

    def update(
        self, x: float, y: float, yaw: Optional[float] = None
    ) -> DistanceUpdate:
        """加入一个位置样本；未开始时可用 yaw 建立起始位姿。

        保持旧基线很重要：若坏样本之后马上恢复，下一帧仍能相对最后一个可信点计算。
        """

        x_value = float(x)
        y_value = float(y)
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            return DistanceUpdate(False, self._distance, "non_finite_odom")
        if self._last is None:
            if yaw is None or not math.isfinite(float(yaw)):
                return DistanceUpdate(False, self._distance, "missing_start_yaw")
            self.reset(x_value, y_value, float(yaw))
            return DistanceUpdate(True, self._distance, "started")

        # hypot 计算 sqrt(dx²+dy²)，不使用 z，符合地面车辆任务定义。
        step = math.hypot(x_value - self._last[0], y_value - self._last[1])
        if step > self._max_step:
            return DistanceUpdate(False, self._distance, "odom_jump")

        self._last = (x_value, y_value)
        # 不是对每段路程积分，而是始终相对固定起点做一次纵向投影。
        # 因此左右横移、蛇形修正不会让 9 m 任务提前完成，倒车则得到负方向变化。
        assert self._start is not None and self._forward is not None
        displacement_x = x_value - self._start[0]
        displacement_y = y_value - self._start[1]
        self._distance = (
            displacement_x * self._forward[0]
            + displacement_y * self._forward[1]
        )
        return DistanceUpdate(True, self._distance)
