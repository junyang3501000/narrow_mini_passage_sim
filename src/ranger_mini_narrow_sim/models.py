"""简化窄道控制器使用的、与 ROS 消息解耦的数据模型。

坐标统一遵循 ``base_link``：x 向前、y 向左、z 向上，角度逆时针为正。
距离单位为米，时间单位为秒，角度单位为弧度。把这些模型与 ROS 分离后，核心算法
不需要 ROS Master 就能进行单元测试，也不会把 ``geometry_msgs`` 传播到算法层。
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class PassageState(Enum):
    """一次窄道任务对外可见的状态。

    SUCCESS 和 FAULT 是锁定状态：控制节点会持续输出零速度，直到再次使能开始新任务。
    OBSTACLE_STOP 不是故障，障碍清除并满足迟滞确认后可以自动回到 RUNNING；
    SIDE_RECOVERY 会在紧急停车后只允许安全方向的低速横移。
    """

    DISABLED = "disabled"                 # 未使能；无论传感器是否正常都输出零速度。
    WAITING_SENSORS = "waiting_sensors"   # 等待新鲜点云、odom、TF 或连续有效双墙。
    RUNNING = "running"                   # 所有安全条件满足，允许发布跟踪命令。
    OBSTACLE_STOP = "obstacle_stop"       # 前方安全带发现近障碍，立即停车但可恢复。
    SIDE_RECOVERY = "side_recovery"       # 完整 footprint 紧急余量触发后，仅低速横移。
    SUCCESS = "success"                   # 纵向投影进度达到目标，锁定停车。
    FAULT = "fault"                       # 超时、odom 跳变等不可自动恢复故障。


@dataclass(frozen=True)
class WallObservation:
    """由单帧点云估计出的双墙几何和前方净空。

    ``frame_valid`` 表示输入/点数层面有效；``walls_valid`` 进一步表示左右墙通过
    宽度、方向和平行性检查。调用者必须同时检查两者，不能沿用上一帧墙参数。
    Optional 字段在对应几何尚未得到时为 None，而不是伪造 0。
    """

    stamp: float                              # 观测生成时刻，用于状态输出和调试。
    frame_valid: bool                         # 点云读取、TF 和总点数是否有效。
    walls_valid: bool                         # 左右墙是否都通过全部几何约束。
    point_count: int                          # 进入估计器的有限前方点数量。
    left_distance: Optional[float] = None      # base_link 原点到左墙的法向距离。
    right_distance: Optional[float] = None     # base_link 原点到右墙的法向距离（正数）。
    width: Optional[float] = None              # left_distance + right_distance。
    lateral_error: Optional[float] = None      # ey=(左距-右距)/2；正值要求向左归中。
    heading_error: Optional[float] = None      # e_theta；正值表示通道指向左前方。
    left_heading: Optional[float] = None       # 左墙相对车头方向角。
    right_heading: Optional[float] = None      # 右墙相对车头方向角。
    left_front_clearance: Optional[float] = None  # 左前 footprint 角点到左墙的法向净空。
    right_front_clearance: Optional[float] = None # 右前 footprint 角点到右墙的法向净空。
    minimum_front_clearance: Optional[float] = None # 两个前角侧向净空的较小值。
    left_rear_clearance: Optional[float] = None   # 左后角到左墙的预测净空（由前方拟合墙外推）。
    right_rear_clearance: Optional[float] = None  # 右后角到右墙的预测净空（由前方拟合墙外推）。
    left_footprint_clearance: Optional[float] = None  # 左侧前/后角净空的较小值。
    right_footprint_clearance: Optional[float] = None # 右侧前/后角净空的较小值。
    minimum_footprint_clearance: Optional[float] = None # 完整矩形四角的最小侧向净空。
    front_clearance: Optional[float] = None    # 车头中央安全带内最近点 x；无点时 None。
    confidence: float = 0.0                    # 由内点支持、残差和平行性组合的 0~1 分数。
    reason: str = ""                          # 稳定机器可读原因，例如 walls_not_parallel。


@dataclass(frozen=True)
class VelocityCommand:
    """平面车体速度，可直接转换成 ``geometry_msgs/Twist``。

    当前仿真把 linear_y 当理想横移；真实 Ranger Mini 必须先确认平行/楔形转向模式
    和轮胎到位反馈，不能仅凭该字段证明真实四轮已经转到安全角度。
    """

    linear_x: float = 0.0   # 前进速度，向前为正，m/s。
    linear_y: float = 0.0   # 横移速度，向左为正，m/s。
    angular_z: float = 0.0  # 偏航角速度，逆时针为正，rad/s。


@dataclass(frozen=True)
class ControlResult:
    """控制器候选命令及其诊断信息。

    ``limited`` 仅说明比例控制结果被速度上限截断，不代表传感器故障；最终是否允许运动
    仍由 ROS 节点的超时、障碍、双墙确认和任务状态安全门控决定。
    """

    command: VelocityCommand
    limited: bool = False
    reason: str = ""
