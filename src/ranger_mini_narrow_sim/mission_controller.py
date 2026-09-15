"""一轮窄道任务的状态机和最终命令决策。

本模块集中表达安全优先级，但不订阅/发布 ROS：节点只传入当前时间、传感器时间、墙观测
和控制周期。里程、连续控制、安全迟滞仍由各自模块完成。
"""

from typing import Optional

from .controller import CommandSlewLimiter, NarrowController
from .models import PassageState, VelocityCommand, WallObservation
from .node_config import NodeConfig
from .odometry import OdometryProgress
from .safety_supervisor import SafetySupervisor


class MissionController:
    """管理使能/成功/故障状态，并按固定优先级选择最终速度。"""

    # 只有“新鲜点云里墙段自然结束”才允许进入有限出口续行。TF/解码故障不在其中。
    _EXIT_WALL_LOSS_REASONS = frozenset(
        (
            "too_few_points",
            "both_wall_not_found",
            "left_wall_not_found",
            "right_wall_not_found",
        )
    )
    # 初始化,config是节点配置，odometry是里程计进度，
    # controller是控制器，safety是安全监督器，
    # slew_limiter是速度斜坡限制器
    def __init__(
        self,
        config: NodeConfig,
        odometry: OdometryProgress,
        controller: NarrowController,
        safety: SafetySupervisor,
        slew_limiter: CommandSlewLimiter,
    ):
    # 初始化成员变量,用成员变量存储配置、
    #里程计进度、控制器、安全监督器、速度斜坡限制器
        self._config = config
        self._odometry = odometry 
        self._controller = controller
        self._safety = safety
        self._slew_limiter = slew_limiter
        self.enabled = False
        self.state = PassageState.DISABLED
        self.reason = "startup"
        self._enabled_at: Optional[float] = None # 启动时间，optional表示可能为空
        self._has_run = False # 是否已经运行过

    @property # 属性装饰器，表示这是一个属性，而不是方法
    # 返回是否应把新 odom 用于当前任务进度,bool表示布尔值表示真或假
    def tracking_odometry(self) -> bool:
        """是否应把新 odom 用于当前任务进度。"""
        #not in表示不在括号内的状态，则返回True
        return self.enabled and self.state not in (
            PassageState.SUCCESS, # 成功
            PassageState.FAULT, # 故障
        )
    # 返回是否应把新 odom 用于当前任务进度,bool表示布尔值表示真或假

    def start(self, now: float) -> None:
        """开始新任务，并重置安全、速度斜坡和纵向进度参考。"""

        self.enabled = True
        self.state = PassageState.WAITING_SENSORS
        self.reason = "new_run"
        self._enabled_at = now
        self._has_run = False
        self._safety.reset()
        self._slew_limiter.stop()
        self._odometry.reset_task_if_fresh(now, self._config.odom_timeout)

    def disable(self, reason: str) -> VelocityCommand:
        """结束任务、清除子模块状态并返回严格零速度。"""

        self.enabled = False
        self.state = PassageState.DISABLED
        self.reason = reason
        self._enabled_at = None
        self._has_run = False
        self._safety.reset()
        self._odometry.clear_task()
        return self._slew_limiter.stop()

    def fault(self, reason: str) -> VelocityCommand:
        """进入锁定故障并返回严格零速度。"""

        self.state = PassageState.FAULT
        self.reason = reason
        return self._slew_limiter.stop()

    # decide方法执行完整安全优先级，返回本周期唯一可发布的速度命令。
    def decide(
        self,
        now: float,
        observation: WallObservation,
        lidar_received_at: Optional[float],
        dt: float,
      ) -> VelocityCommand: # 返回速度命令
        """执行完整安全优先级，返回本周期唯一可发布的速度命令。"""

        if not self.enabled:
            return self._stop(PassageState.DISABLED, "not_enabled")
        if self.state in (PassageState.SUCCESS, PassageState.FAULT):
            return self._slew_limiter.stop()
        if (
            self._enabled_at is not None
            and now - self._enabled_at > self._config.mission_timeout
        ):
            return self.fault("mission_timeout")
        if self._odometry.distance >= self._config.passage_distance:
            return self._stop(
                PassageState.SUCCESS, "passage_distance_reached"
            )

        # 检查激光雷达接收时间是否有效，lidar_received_at是激光雷达接收时间，now是当前时间，self._config.lidar_timeout是激光雷达超时时间
        if not self._fresh(lidar_received_at, now, self._config.lidar_timeout):
            # 如果激光雷达接收时间无效，则返回等待传感器状态，并返回激光雷达超时原因
            return self._stop(PassageState.WAITING_SENSORS, "lidar_timeout")
        if not self._odometry.is_fresh(now, self._config.odom_timeout):
            return self._stop(PassageState.WAITING_SENSORS, "odom_timeout")
        if not self._odometry.ensure_started():
            return self._stop(PassageState.WAITING_SENSORS, "no_odom_start")
        if not observation.frame_valid:
            if self._can_coast_to_exit(observation):
                return self._coast_to_exit(dt)
            return self._stop(
                PassageState.WAITING_SENSORS,
                observation.reason or "invalid_lidar_frame",
            )
        if self._safety.obstacle_latched:
            return self._stop(PassageState.OBSTACLE_STOP, "front_obstacle")
        if not observation.walls_valid:
            if self._can_coast_to_exit(observation):
                return self._coast_to_exit(dt)
            return self._stop(
                PassageState.WAITING_SENSORS,
                observation.reason or "walls_invalid",
            )
        if not self._safety.walls_confirmed:
            return self._stop(
                PassageState.WAITING_SENSORS, "confirming_walls"
            )

        if self._safety.footprint_emergency_latched:
            self.state = PassageState.SIDE_RECOVERY
            if self._safety.consume_emergency_stop():
                return self._stop(
                    PassageState.SIDE_RECOVERY, "footprint_emergency_stop"
                )
            decision = self._safety.recovery_decision(observation)
            self.reason = decision.reason
            if decision.stop:
                return self._slew_limiter.stop()
            return self._slew_limiter.apply(decision.command, dt)

        result = self._controller.compute(observation)
        if result.reason not in ("tracking", "controller_limit"):
            return self._stop(PassageState.WAITING_SENSORS, result.reason)

        target = result.command
        warning_side = self._safety.warning_side(observation)
        if warning_side is not None:
            target = self._safety.limit_warning_command(target, warning_side)
            self.reason = "footprint_warning_{}".format(warning_side)
        else:
            self.reason = result.reason
        self.state = PassageState.RUNNING
        self._has_run = True
        return self._slew_limiter.apply(target, dt)

    def _can_coast_to_exit(self, observation: WallObservation) -> bool:
        """仅在曾正常跟踪且已非常接近终点时，把自然墙端识别为出口。"""

        remaining = self._config.passage_distance - self._odometry.distance
        return (
            self._has_run
            and self._config.exit_coast_distance > 0.0
            and 0.0 < remaining <= self._config.exit_coast_distance
            and observation.reason in self._EXIT_WALL_LOSS_REASONS
            and not self._safety.obstacle_latched
            and not self._safety.footprint_emergency_latched
        )

    def _coast_to_exit(self, dt: float) -> VelocityCommand:
        """墙端离开前向视场后只保留前进分量，按配置的有限里程驶到出口线。"""

        self.state = PassageState.EXITING
        self.reason = "exit_wall_end_coast"
        target = VelocityCommand(
            linear_x=min(
                self._config.controller.forward_speed,
                self._config.controller.max_linear_speed,
            )
        )
        return self._slew_limiter.apply(target, dt)

    def _stop(self, state: PassageState, reason: str) -> VelocityCommand:
        """统一设置状态并绕过减速斜坡立即清零。"""

        self.state = state
        self.reason = reason
        return self._slew_limiter.stop()

    @staticmethod
    def _fresh(received_at: Optional[float], now: float, timeout: float) -> bool:
        """判断接收时间存在、不是未来值且没有超过看门狗。"""

        return received_at is not None and 0.0 <= now - received_at <= timeout
