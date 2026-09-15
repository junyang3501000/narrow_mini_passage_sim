"""把 ROS 参数集中转换成强类型配置。

本模块只依赖一个形如 ``rospy.get_param(name, default)`` 的读取函数，因此可以在没有
ROS Master 的单元测试中传入字典读取器。参数错误会在节点启动阶段直接报出。
"""

import math
from dataclasses import dataclass
from typing import Any, Callable

from .controller import ControllerConfig, SlewConfig
from .safety_supervisor import SafetyConfig
from .wall_estimator import WallEstimatorConfig


ParamGetter = Callable[[str, Any], Any]


@dataclass(frozen=True)
class NodeConfig:
    """ROS接口、调度参数和各功能模块的完整配置。"""

    control_rate: float
    status_rate: float
    lidar_timeout: float
    odom_timeout: float
    mission_timeout: float
    passage_distance: float
    exit_coast_distance: float
    max_odom_step: float
    point_stride: int
    max_points: int
    min_obstacle_z: float
    max_obstacle_z: float
    lidar_topic: str
    odom_topic: str
    cmd_topic: str
    status_topic: str
    enable_service: str
    source_frame: str
    target_frame: str
    tf_timeout: float
    enabled_on_start: bool
    estimator: WallEstimatorConfig
    controller: ControllerConfig
    slew: SlewConfig
    safety: SafetyConfig


def _positive(get_param: ParamGetter, name: str, default: float) -> float:
    """读取有限正浮点数。"""

    value = float(get_param(name, default))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("{} must be finite and positive".format(name))
    return value


def _positive_int(get_param: ParamGetter, name: str, default: int) -> int:
    """读取正整数。"""

    value = int(get_param(name, default))
    if value <= 0:
        raise ValueError("{} must be positive".format(name))
    return value


def _nonnegative(get_param: ParamGetter, name: str, default: float) -> float:
    """读取有限非负浮点数；0 用于显式关闭可选功能。"""

    value = float(get_param(name, default))
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("{} must be finite and non-negative".format(name))
    return value


def load_node_config(get_param: ParamGetter) -> NodeConfig:
    """一次性读取并校验全部参数，避免参数代码散落在 ROS 节点中。"""

    controller = ControllerConfig(
        forward_speed=float(get_param("~forward_speed", 0.25)),
        allow_lateral_motion=bool(get_param("~allow_lateral_motion", True)),
        lateral_kp=float(get_param("~lateral_kp", 0.50)),
        heading_kp=float(get_param("~heading_kp", 1.20)),
        center_angular_kp=float(get_param("~center_angular_kp", 0.0)),
        lateral_deadband=float(get_param("~lateral_deadband", 0.015)),
        heading_deadband=math.radians(
            float(get_param("~heading_deadband_deg", 0.8))
        ),
        max_linear_speed=float(get_param("~max_linear_speed", 0.30)),
        max_lateral_speed=float(get_param("~max_lateral_speed", 0.10)),
        max_angular_speed=float(get_param("~max_angular_speed", 0.30)),
    )

    front_stop_distance = _positive(
        get_param, "~front_stop_distance", 0.65
    )
    front_resume_distance = _positive(
        get_param, "~front_resume_distance", 0.85
    )
    footprint_emergency_margin = _positive(
        get_param, "~footprint_emergency_margin", 0.05
    )
    footprint_warning_margin = _positive(
        get_param, "~footprint_warning_margin", 0.10
    )
    safety = SafetyConfig(
        wall_confirm_frames=_positive_int(
            get_param, "~wall_confirm_frames", 3
        ),
        front_stop_distance=front_stop_distance,
        front_resume_distance=front_resume_distance,
        obstacle_clear_confirm_frames=_positive_int(
            get_param, "~obstacle_clear_confirm_frames", 3
        ),
        footprint_emergency_margin=footprint_emergency_margin,
        footprint_warning_margin=footprint_warning_margin,
        footprint_warning_forward_speed=_positive(
            get_param, "~footprint_warning_forward_speed", 0.12
        ),
        footprint_recovery_lateral_speed=_positive(
            get_param, "~footprint_recovery_lateral_speed", 0.04
        ),
        footprint_clear_confirm_frames=_positive_int(
            get_param, "~footprint_clear_confirm_frames", 3
        ),
        allow_lateral_motion=controller.allow_lateral_motion,
        max_linear_speed=controller.max_linear_speed,
        max_lateral_speed=controller.max_lateral_speed,
    )

    estimator = WallEstimatorConfig(
        min_total_points=int(get_param("~min_total_points", 30)),
        min_side_points=int(get_param("~min_side_points", 12)),
        side_x_min=float(get_param("~side_x_min", 0.20)),
        side_x_max=float(get_param("~side_x_max", 4.00)),
        min_side_distance=float(get_param("~min_side_distance", 0.25)),
        max_side_distance=float(get_param("~max_side_distance", 3.00)),
        min_corridor_width=float(get_param("~min_corridor_width", 0.80)),
        max_corridor_width=float(get_param("~max_corridor_width", 2.00)),
        max_wall_heading=math.radians(
            float(get_param("~max_wall_heading_deg", 25.0))
        ),
        max_parallel_angle=math.radians(
            float(get_param("~max_parallel_angle_deg", 8.0))
        ),
        ransac_iterations=int(get_param("~ransac_iterations", 80)),
        ransac_inlier_distance=float(
            get_param("~ransac_inlier_distance", 0.07)
        ),
        min_wall_span=float(get_param("~min_wall_span", 0.80)),
        fit_sample_limit=int(get_param("~fit_sample_limit", 500)),
        front_x_min=float(get_param("~front_x_min", 0.05)),
        front_x_max=float(get_param("~front_x_max", 3.00)),
        front_half_width=float(get_param("~front_half_width", 0.35)),
        footprint_front_extent=float(
            get_param("~footprint_front_extent", 0.42)
        ),
        footprint_rear_extent=float(
            get_param("~footprint_rear_extent", 0.42)
        ),
        footprint_half_width=float(
            get_param("~footprint_half_width", 0.35)
        ),
    )

    slew = SlewConfig(
        max_linear_accel=float(get_param("~max_linear_accel", 0.30)),
        max_lateral_accel=float(get_param("~max_lateral_accel", 0.20)),
        max_angular_accel=float(get_param("~max_angular_accel", 0.50)),
    )

    min_obstacle_z = float(get_param("~min_obstacle_z", -0.30))
    max_obstacle_z = float(get_param("~max_obstacle_z", 1.50))
    if min_obstacle_z >= max_obstacle_z:
        raise ValueError("~min_obstacle_z must be below ~max_obstacle_z")

    source_frame = str(get_param("~source_frame", "livox_frame")).strip().lstrip("/")
    target_frame = str(get_param("~target_frame", "base_link")).strip().lstrip("/")
    if not source_frame or not target_frame:
        raise ValueError("~source_frame and ~target_frame must be non-empty")

    # 构造算法对象会进一步检查 estimator/controller/slew/safety 内部参数组合。
    return NodeConfig(
        control_rate=_positive(get_param, "~control_rate", 20.0),
        status_rate=_positive(get_param, "~status_rate", 5.0),
        lidar_timeout=_positive(get_param, "~lidar_timeout", 0.30),
        odom_timeout=_positive(get_param, "~odom_timeout", 0.50),
        mission_timeout=_positive(get_param, "~mission_timeout", 60.0),
        passage_distance=_positive(get_param, "~passage_distance", 10.0),
        exit_coast_distance=_nonnegative(
            get_param, "~exit_coast_distance", 0.0
        ),
        max_odom_step=_positive(get_param, "~max_odom_step", 0.75),
        point_stride=_positive_int(get_param, "~point_stride", 5),
        max_points=_positive_int(get_param, "~max_points", 20000),
        min_obstacle_z=min_obstacle_z,
        max_obstacle_z=max_obstacle_z,
        lidar_topic=str(get_param("~lidar_topic", "/livox/lidar")),
        odom_topic=str(get_param("~odom_topic", "/odom")),
        cmd_topic=str(get_param("~cmd_topic", "/cmd_vel")),
        status_topic=str(
            get_param("~status_topic", "/narrow_passage/status")
        ),
        enable_service=str(
            get_param("~enable_service", "/narrow_mode/enable")
        ),
        source_frame=source_frame,
        target_frame=target_frame,
        tf_timeout=_positive(get_param, "~tf_timeout", 0.08),
        enabled_on_start=bool(get_param("~enabled_on_start", False)),
        estimator=estimator,
        controller=controller,
        slew=slew,
        safety=safety,
    )
