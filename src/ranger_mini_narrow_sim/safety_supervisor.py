"""窄道控制器的纯 Python 安全门控与迟滞状态。

本模块不依赖 rospy，也不发布 Twist。它只接收墙体观测和候选速度，维护连续帧计数、
前方障碍锁存、完整 footprint 紧急锁存，并返回可供 ROS 节点执行的安全决定。
"""

from dataclasses import dataclass
from typing import Optional, Tuple

from .models import VelocityCommand, WallObservation


@dataclass(frozen=True)
class SafetyConfig:
    """所有安全阈值；距离单位 m，速度单位 m/s。"""

    wall_confirm_frames: int
    front_stop_distance: float
    front_resume_distance: float
    obstacle_clear_confirm_frames: int
    footprint_emergency_margin: float
    footprint_warning_margin: float
    footprint_warning_forward_speed: float
    footprint_recovery_lateral_speed: float
    footprint_clear_confirm_frames: int
    allow_lateral_motion: bool
    max_linear_speed: float
    max_lateral_speed: float


@dataclass(frozen=True)
class SafetyDecision:
    """安全模块给出的目标速度和原因；stop=True 表示必须绕过减速斜坡立即停车。"""

    command: VelocityCommand
    reason: str
    stop: bool = False


class SafetySupervisor:
    """维护障碍/footprint 迟滞，并限制会继续靠墙的候选命令。"""

    def __init__(self, config: SafetyConfig):
        self._config = config
        self._validate_config()
        self.reset()

    def _validate_config(self) -> None:
        """构造时拒绝相互矛盾的阈值，避免运行中出现无法恢复的状态。"""

        cfg = self._config
        if cfg.wall_confirm_frames <= 0:
            raise ValueError("wall_confirm_frames must be positive")
        if cfg.obstacle_clear_confirm_frames <= 0:
            raise ValueError("obstacle_clear_confirm_frames must be positive")
        if cfg.footprint_clear_confirm_frames <= 0:
            raise ValueError("footprint_clear_confirm_frames must be positive")
        if cfg.front_stop_distance <= 0.0:
            raise ValueError("front_stop_distance must be positive")
        if cfg.front_resume_distance <= cfg.front_stop_distance:
            raise ValueError("front_resume_distance must exceed front_stop_distance")
        if cfg.footprint_emergency_margin <= 0.0:
            raise ValueError("footprint_emergency_margin must be positive")
        if cfg.footprint_warning_margin <= cfg.footprint_emergency_margin:
            raise ValueError(
                "footprint_warning_margin must exceed emergency margin"
            )
        if cfg.footprint_warning_forward_speed <= 0.0:
            raise ValueError("footprint warning speed must be positive")
        if cfg.footprint_recovery_lateral_speed <= 0.0:
            raise ValueError("footprint recovery speed must be positive")

    def reset(self) -> None:
        """开始/结束任务时清除全部连续帧计数和安全锁存。"""

        self.valid_wall_frames = 0
        self.obstacle_latched = False
        self.obstacle_clear_frames = 0
        self.footprint_emergency_latched = False
        self.footprint_emergency_stop_pending = False
        self.footprint_clear_frames = 0

    @property
    def walls_confirmed(self) -> bool:
        """左右墙是否已经连续达到允许控制的帧数。"""

        return self.valid_wall_frames >= self._config.wall_confirm_frames

    def process_observation(self, observation: WallObservation) -> None:
        """每个新雷达帧调用一次，Timer 重复使用同一帧时不得调用。"""

        if observation.frame_valid and observation.walls_valid:
            self.valid_wall_frames += 1
        else:
            self.valid_wall_frames = 0

        self._update_front_obstacle(observation.front_clearance)
        self._update_footprint(observation)

    def _update_front_obstacle(self, clearance: Optional[float]) -> None:
        """近障碍立即锁存；只有越过更远恢复线并连续多帧才清除。"""

        cfg = self._config
        if clearance is not None and clearance <= cfg.front_stop_distance:
            self.obstacle_latched = True
            self.obstacle_clear_frames = 0
        elif self.obstacle_latched:
            if clearance is None or clearance >= cfg.front_resume_distance:
                self.obstacle_clear_frames += 1
                if (
                    self.obstacle_clear_frames
                    >= cfg.obstacle_clear_confirm_frames
                ):
                    self.obstacle_latched = False
                    self.obstacle_clear_frames = 0
            else:
                self.obstacle_clear_frames = 0

    def _update_footprint(self, observation: WallObservation) -> None:
        """根据完整矩形四角最小净空更新紧急锁存和恢复确认。"""

        clearance = observation.minimum_footprint_clearance
        if observation.walls_valid and clearance is not None:
            if clearance <= self._config.footprint_emergency_margin:
                # 首次触发保留一个“必须严格零速”的控制周期，然后才能侧移。
                if not self.footprint_emergency_latched:
                    self.footprint_emergency_stop_pending = True
                self.footprint_emergency_latched = True
                self.footprint_clear_frames = 0
            elif self.footprint_emergency_latched:
                if clearance >= self._config.footprint_warning_margin:
                    self.footprint_clear_frames += 1
                    if (
                        self.footprint_clear_frames
                        >= self._config.footprint_clear_confirm_frames
                    ):
                        self.footprint_emergency_latched = False
                        self.footprint_emergency_stop_pending = False
                        self.footprint_clear_frames = 0
                else:
                    self.footprint_clear_frames = 0
        elif self.footprint_emergency_latched:
            # 墙线失效时不能用未知数据解除一个已经触发的紧急状态。
            self.footprint_clear_frames = 0

    def consume_emergency_stop(self) -> bool:
        """首次紧急触发返回 True 一次，确保当周期严格清零。"""

        if not self.footprint_emergency_stop_pending:
            return False
        self.footprint_emergency_stop_pending = False
        return True

    def warning_side(self, observation: WallObservation) -> Optional[str]:
        """返回进入预警区的一侧：left/right/both；均安全时返回 None。"""

        left = observation.left_footprint_clearance
        right = observation.right_footprint_clearance
        if left is None or right is None:
            return None
        left_warning = left <= self._config.footprint_warning_margin
        right_warning = right <= self._config.footprint_warning_margin
        if left_warning and right_warning:
            return "both"
        if left_warning:
            return "left"
        if right_warning:
            return "right"
        return None

    def limit_warning_command(
        self, command: VelocityCommand, warning_side: str
    ) -> VelocityCommand:
        """预警区内降速，并删除会让危险侧继续靠墙的 vy/wz 分量。"""

        linear_x = min(
            max(0.0, command.linear_x),
            self._config.footprint_warning_forward_speed,
            self._config.max_linear_speed,
        )
        linear_y = command.linear_y
        angular_z = command.angular_z
        # base_link 中 +vy 向左，+wz 使车头逆时针向左摆。
        if warning_side in ("left", "both"):
            linear_y = min(0.0, linear_y)
            angular_z = min(0.0, angular_z)
        if warning_side in ("right", "both"):
            linear_y = max(0.0, linear_y)
            angular_z = max(0.0, angular_z)
        return VelocityCommand(linear_x, linear_y, angular_z)

    def recovery_decision(self, observation: WallObservation) -> SafetyDecision:
        """紧急停车后的低速横移策略；未知或两侧受限时继续严格停车。"""

        left = observation.left_footprint_clearance
        right = observation.right_footprint_clearance
        minimum = observation.minimum_footprint_clearance
        if left is None or right is None or minimum is None:
            return SafetyDecision(
                VelocityCommand(), "footprint_clearance_unknown", True
            )
        if minimum >= self._config.footprint_warning_margin:
            return SafetyDecision(
                VelocityCommand(), "confirming_footprint_clearance", True
            )
        if not self._config.allow_lateral_motion:
            return SafetyDecision(
                VelocityCommand(),
                "footprint_recovery_requires_lateral_motion",
                True,
            )

        speed = min(
            self._config.footprint_recovery_lateral_speed,
            self._config.max_lateral_speed,
        )
        if left < right:
            if right <= self._config.footprint_warning_margin:
                return SafetyDecision(
                    VelocityCommand(), "footprint_both_sides_constrained", True
                )
            return SafetyDecision(
                VelocityCommand(linear_y=-speed), "footprint_recover_right"
            )
        if right < left:
            if left <= self._config.footprint_warning_margin:
                return SafetyDecision(
                    VelocityCommand(), "footprint_both_sides_constrained", True
                )
            return SafetyDecision(
                VelocityCommand(linear_y=speed), "footprint_recover_left"
            )
        return SafetyDecision(
            VelocityCommand(), "footprint_symmetric_constraint", True
        )

    def footprint_zone(self, observation: WallObservation) -> str:
        """把当前净空转换为 clear/warning/emergency/unknown 诊断标签。"""

        clearance = observation.minimum_footprint_clearance
        if clearance is None:
            return "unknown"
        if clearance <= self._config.footprint_emergency_margin:
            return "emergency"
        if clearance <= self._config.footprint_warning_margin:
            return "warning"
        return "clear"

    @property
    def diagnostic_counters(self) -> Tuple[int, int, int]:
        """按“有效墙帧、前障碍恢复帧、footprint恢复帧”返回调试计数。"""

        return (
            self.valid_wall_frames,
            self.obstacle_clear_frames,
            self.footprint_clear_frames,
        )
