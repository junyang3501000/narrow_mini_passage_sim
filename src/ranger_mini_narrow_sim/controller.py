"""窄道专用 Python 反馈控制器与命令变化率限制器。

该模块不订阅/发布 ROS 话题，只把一帧有效墙体观测映射为 ``vx/vy/wz``。
安全状态机位于 ros_node.py；这里负责连续控制数学，便于纯 Python 单元测试。
"""

import math
from dataclasses import dataclass

from .models import ControlResult, VelocityCommand, WallObservation


def _clamp(value: float, limit: float) -> float:
    """把有符号数限制到 [-limit, +limit]，保留原始修正方向。"""

    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class ControllerConfig:
    """居中/航向控制参数；单位已经统一为 m、s、rad。"""

    forward_speed: float = 0.25       # RUNNING 时固定的前进目标速度。
    allow_lateral_motion: bool = True # false 时强制 vy=0，适配不支持横移的底盘。
    lateral_kp: float = 0.50          # ey -> vy 比例增益。
    heading_kp: float = 1.20          # e_theta -> wz 比例增益。
    center_angular_kp: float = 0.0    # 可选 ey -> wz，用转向逐渐消除位置误差。
    lateral_deadband: float = 0.015   # 横向误差死区，避免厘米级噪声引起抖动。
    heading_deadband: float = math.radians(0.8)  # 航向误差死区，内部使用弧度。
    max_linear_speed: float = 0.30    # |vx| 硬上限。
    max_lateral_speed: float = 0.10   # |vy| 硬上限。
    max_angular_speed: float = 0.30   # |wz| 硬上限。


class NarrowController:
    """把通道中心误差和墙方向误差转换成车体 Twist 候选。"""

    def __init__(self, config: ControllerConfig):
        # 构造时即拒绝危险配置，避免节点已经运行后才发现负速度上限等错误。
        self._config = config
        self._validate_config()

    def _validate_config(self) -> None:
        """校验所有数值有限且非负，并保证目标前进速度不突破硬限制。"""

        cfg = self._config
        values = (
            cfg.forward_speed,
            cfg.lateral_kp,
            cfg.heading_kp,
            cfg.center_angular_kp,
            cfg.lateral_deadband,
            cfg.heading_deadband,
            cfg.max_linear_speed,
            cfg.max_lateral_speed,
            cfg.max_angular_speed,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("controller parameters must be finite and non-negative")
        if cfg.forward_speed > cfg.max_linear_speed:
            raise ValueError("forward_speed exceeds max_linear_speed")

    def compute(self, observation: WallObservation) -> ControlResult:
        """从一帧有效双墙观测计算 vx、vy 和偏航角速度。

        符号约定非常关键：ey>0 表示中心在左侧，所以 vy 应为正；e_theta>0 表示
        通道指向左前方，所以 wz 应为正。无效观测直接返回零命令，绝不复用旧误差。
        """

        if (
            not observation.frame_valid
            or not observation.walls_valid
            or observation.lateral_error is None
            or observation.heading_error is None
        ):
            return ControlResult(VelocityCommand(), True, "invalid_wall_observation")

        ey = observation.lateral_error
        e_theta = observation.heading_error
        if not math.isfinite(ey) or not math.isfinite(e_theta):
            return ControlResult(VelocityCommand(), True, "non_finite_control_error")

        # 死区只抑制很小的稳态误差，不改变死区外的误差幅值。
        lateral_error = 0.0 if abs(ey) <= self._config.lateral_deadband else ey
        heading_error = (
            0.0 if abs(e_theta) <= self._config.heading_deadband else e_theta
        )
        # 位置回路：Ranger Mini 理想平移模式用 vy 直接归中。
        linear_y = self._config.lateral_kp * lateral_error
        if not self._config.allow_lateral_motion:
            linear_y = 0.0
        # 航向回路：默认只使用墙方向；非全向底盘可给 center_angular_kp 一个小正值。
        angular_z = (
            self._config.heading_kp * heading_error
            + self._config.center_angular_kp * lateral_error
        )
        # 比例控制可能在初始偏差大时给出过高速度，发布前逐轴硬限幅。
        command = VelocityCommand(
            linear_x=_clamp(self._config.forward_speed, self._config.max_linear_speed),
            linear_y=_clamp(linear_y, self._config.max_lateral_speed),
            angular_z=_clamp(angular_z, self._config.max_angular_speed),
        )
        # 状态 reason 用于区分正常跟踪和被限幅跟踪，两者都仍可安全运行。
        limited = (
            abs(linear_y) > self._config.max_lateral_speed
            or abs(angular_z) > self._config.max_angular_speed
        )
        return ControlResult(command, limited, "controller_limit" if limited else "tracking")


@dataclass(frozen=True)
class SlewConfig:
    """三轴命令每秒允许的最大变化量。"""

    max_linear_accel: float = 0.30   # vx 斜坡，m/s²。
    max_lateral_accel: float = 0.20  # vy 斜坡，m/s²。
    max_angular_accel: float = 0.50  # wz 斜坡，rad/s²。


class CommandSlewLimiter:
    """限制正常命令突变；安全停车通过 ``stop`` 绕过减速斜坡。"""

    def __init__(self, config: SlewConfig):
        for value in (
            config.max_linear_accel,
            config.max_lateral_accel,
            config.max_angular_accel,
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("slew limits must be finite and positive")
        self._config = config
        self._last = VelocityCommand()

    @property
    def last(self) -> VelocityCommand:
        """返回上一周期已接受的命令，用作下一次斜坡起点。"""

        return self._last

    @staticmethod
    def _approach(current: float, target: float, maximum_delta: float) -> float:
        """单周期最多向目标移动 maximum_delta，正负方向均适用。"""

        delta = target - current
        return current + _clamp(delta, maximum_delta)

    def apply(self, target: VelocityCommand, dt: float) -> VelocityCommand:
        """按实际控制周期 dt 对三轴独立限速，并保存结果。

        dt 非法时保持上一命令，避免时间倒退或零周期产生除零/瞬间跳变。
        """

        if not math.isfinite(dt) or dt <= 0.0:
            return self._last
        cfg = self._config
        self._last = VelocityCommand(
            linear_x=self._approach(
                self._last.linear_x, target.linear_x, cfg.max_linear_accel * dt
            ),
            linear_y=self._approach(
                self._last.linear_y, target.linear_y, cfg.max_lateral_accel * dt
            ),
            angular_z=self._approach(
                self._last.angular_z, target.angular_z, cfg.max_angular_accel * dt
            ),
        )
        return self._last

    def stop(self) -> VelocityCommand:
        """使能关闭、数据超时或障碍事件时立即清零，不经过缓慢减速。"""

        self._last = VelocityCommand()
        return self._last
