"""构造 /narrow_passage/status 使用的 JSON 兼容字典。"""

import math
from typing import Dict

from .models import PassageState, VelocityCommand, WallObservation
from .node_config import NodeConfig
from .safety_supervisor import SafetySupervisor


def build_status(
    now: float,
    enabled: bool,
    state: PassageState,
    reason: str,
    observation: WallObservation,
    transform_valid: bool,
    distance: float,
    command: VelocityCommand,
    config: NodeConfig,
    safety: SafetySupervisor,
) -> Dict[str, object]:
    """汇总接口、感知、安全、任务进度和最终命令，供序列化为 JSON。"""

    status: Dict[str, object] = {
        "stamp": now,
        "enabled": enabled,
        "state": state.value,
        "reason": reason,
        "lidar_topic": config.lidar_topic,
        "odom_topic": config.odom_topic,
        "cmd_topic": config.cmd_topic,
        "source_frame": config.source_frame,
        "target_frame": config.target_frame,
        "transform_valid": transform_valid,
        "frame_valid": observation.frame_valid,
        "walls_valid": observation.walls_valid,
        "wall_reason": observation.reason,
        "point_count": observation.point_count,
        "left_distance": observation.left_distance,
        "right_distance": observation.right_distance,
        "corridor_width": observation.width,
        "left_front_clearance": observation.left_front_clearance,
        "right_front_clearance": observation.right_front_clearance,
        "minimum_front_corner_clearance": observation.minimum_front_clearance,
        "left_rear_clearance": observation.left_rear_clearance,
        "right_rear_clearance": observation.right_rear_clearance,
        "left_footprint_clearance": observation.left_footprint_clearance,
        "right_footprint_clearance": observation.right_footprint_clearance,
        "minimum_footprint_clearance": observation.minimum_footprint_clearance,
        "footprint_emergency_margin": config.safety.footprint_emergency_margin,
        "footprint_warning_margin": config.safety.footprint_warning_margin,
        "footprint_zone": safety.footprint_zone(observation),
        "footprint_emergency_latched": safety.footprint_emergency_latched,
        "footprint_clear_frames": safety.footprint_clear_frames,
        "valid_wall_frames": safety.valid_wall_frames,
        "obstacle_latched": safety.obstacle_latched,
        "obstacle_clear_frames": safety.obstacle_clear_frames,
        "ey": observation.lateral_error,
        "e_theta": observation.heading_error,
        "front_clearance": observation.front_clearance,
        "confidence": observation.confidence,
        "distance": distance,
        "distance_mode": "start_heading_projection",
        "target_distance": config.passage_distance,
        "linear_x": command.linear_x,
        "linear_y": command.linear_y,
        "angular_z": command.angular_z,
    }
    # JSON 标准不接受 NaN/Infinity；无效浮点统一转换为 null。
    for key, value in list(status.items()):
        if isinstance(value, float) and not math.isfinite(value):
            status[key] = None
    return status
