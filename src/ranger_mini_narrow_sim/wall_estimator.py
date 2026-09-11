"""从 ``base_link`` 平面点中估计窄道左右两条近似平行墙。

本简化算法不负责在开放环境中寻找入口，前提是车辆已经位于入口附近并大致朝向通道。
因此可按 y 正负把左右点分开，再分别用 RANSAC 拟合 ``y = a*x + b``。RANSAC 先抵抗
障碍物/噪声离群点，最小二乘再用内点细化直线。输出统一遵循 x 前、y 左、逆时针为正。
"""

import math
import random
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from .models import WallObservation


# 算法层只需要俯视平面坐标；z 高度过滤已在 ros_node.py 中完成。
Point2D = Tuple[float, float]


@dataclass(frozen=True)
class WallEstimatorConfig:
    """墙体搜索、几何验收和前障碍带参数；角度字段均为弧度。"""

    min_total_points: int = 30             # 前方有限点的整帧最低数量。
    min_side_points: int = 12              # 单侧直线最终至少需要的内点数。
    side_x_min: float = 0.20               # 墙体拟合前向起点。
    side_x_max: float = 4.00               # 墙体拟合前向终点。
    min_side_distance: float = 0.25        # 中央排除带半宽。
    max_side_distance: float = 3.00        # 左右搜索范围绝对值上限。
    min_corridor_width: float = 0.80       # 可接受通道净宽下限。
    max_corridor_width: float = 2.00       # 可接受通道净宽上限。
    max_wall_heading: float = math.radians(25.0)   # 单墙相对 x 轴最大偏角。
    max_parallel_angle: float = math.radians(8.0) # 左右墙最大方向差。
    ransac_iterations: int = 80            # 单侧随机模型尝试次数。
    ransac_inlier_distance: float = 0.07   # 点到直线的法向内点阈值。
    min_wall_span: float = 0.80            # 内点沿 x 的最小覆盖长度。
    fit_sample_limit: int = 500            # 单侧 RANSAC 最大输入点数。
    front_x_min: float = 0.05              # 前障碍带起点。
    front_x_max: float = 3.00              # 前障碍带终点。
    front_half_width: float = 0.35         # 前障碍带 y 方向半宽。
    footprint_front_extent: float = 0.42   # base_link 到完整矩形 footprint 最前端。
    footprint_rear_extent: float = 0.42    # base_link 到完整矩形 footprint 最后端的正距离。
    footprint_half_width: float = 0.35     # 真实车体（含轮胎）半宽，不含安全余量。


@dataclass(frozen=True)
class _LineFit:
    """内部直线拟合结果，不直接暴露给 ROS 状态机。"""

    slope: float        # a：y=a*x+b 的斜率。
    intercept: float    # b：直线在 x=0 处的 y 截距。
    angle: float        # atan(a)，墙方向相对车头的有符号角度。
    inlier_count: int   # 最终符合距离阈值的点数。
    sample_count: int   # 有界采样后的总候选点数。
    span: float         # 最终内点 max(x)-min(x)。
    rms_error: float    # 内点到直线法向残差的均方根。


class WallEstimator:
    """寻找一条左墙和一条右墙，并计算 ``ey`` 与 ``e_theta``。"""

    def __init__(self, config: WallEstimatorConfig):
        self._config = config
        self._validate_config()

    def _validate_config(self) -> None:
        """在开始处理点云前检查参数组合，拒绝空区间和不可能阈值。"""

        cfg = self._config
        if cfg.min_total_points <= 0 or cfg.min_side_points < 2:
            raise ValueError("point thresholds must be positive")
        if not 0.0 <= cfg.side_x_min < cfg.side_x_max:
            raise ValueError("side x range is invalid")
        if not 0.0 < cfg.min_side_distance < cfg.max_side_distance:
            raise ValueError("side y range is invalid")
        if not 0.0 < cfg.min_corridor_width < cfg.max_corridor_width:
            raise ValueError("corridor width range is invalid")
        if not 0.0 < cfg.max_wall_heading < math.pi / 2.0:
            raise ValueError("max_wall_heading is invalid")
        if not 0.0 < cfg.max_parallel_angle < math.pi / 2.0:
            raise ValueError("max_parallel_angle is invalid")
        if cfg.ransac_iterations <= 0 or cfg.ransac_inlier_distance <= 0.0:
            raise ValueError("RANSAC parameters are invalid")
        if cfg.min_wall_span <= 0.0 or cfg.fit_sample_limit < cfg.min_side_points:
            raise ValueError("wall span/sample parameters are invalid")
        if not 0.0 <= cfg.front_x_min < cfg.front_x_max:
            raise ValueError("front x range is invalid")
        if cfg.front_half_width <= 0.0:
            raise ValueError("front_half_width must be positive")
        if (
            cfg.footprint_front_extent <= 0.0
            or cfg.footprint_rear_extent <= 0.0
            or cfg.footprint_half_width <= 0.0
        ):
            raise ValueError("footprint dimensions must be positive")

    def estimate(self, points: Iterable[Point2D], stamp: float) -> WallObservation:
        """仅根据当前帧返回几何，不保存或复用历史墙线。

        无效帧仍可携带当前帧独立算出的 ``front_clearance``，使前障碍停车不依赖墙拟合
        成功；但墙无效时上层仍会发布零速度。
        """

        # 第一阶段：数值清洗并只保留车辆前方 x>0 的点；后方射线当前不参与控制。
        finite: List[Point2D] = []
        for raw_x, raw_y in points:
            x = float(raw_x)
            y = float(raw_y)
            if math.isfinite(x) and math.isfinite(y) and x > 0.0:
                finite.append((x, y))

        cfg = self._config
        # 第二阶段：在车头中央窄带内取最小 x，作为保守的前方最近障碍距离。
        front_samples = [
            x
            for x, y in finite
            if cfg.front_x_min <= x <= cfg.front_x_max
            and abs(y) <= cfg.front_half_width
        ]
        front_clearance = min(front_samples) if front_samples else None
        # 总点数不足说明扫描、TF 或环境支撑不足，不能只凭几个点推断通道。
        if len(finite) < cfg.min_total_points:
            return WallObservation(
                stamp=stamp,
                frame_valid=False,
                walls_valid=False,
                point_count=len(finite),
                front_clearance=front_clearance,
                reason="too_few_points",
            )

        # 第三阶段：限制 x 前视区间和 y 搜索范围，再按 y 正/负分成左右候选。
        side_points = [
            point
            for point in finite
            if cfg.side_x_min <= point[0] <= cfg.side_x_max
            and abs(point[1]) <= cfg.max_side_distance
        ]
        left_points = [point for point in side_points if point[1] >= cfg.min_side_distance]
        right_points = [point for point in side_points if point[1] <= -cfg.min_side_distance]
        # 固定但不同的 seed 让测试可复现，同时避免左右侧每次抽到完全相同索引模式。
        left = self._fit_wall(left_points, seed=17)
        right = self._fit_wall(right_points, seed=29)
        if left is None or right is None:
            missing = "both"
            if left is not None:
                missing = "right"
            elif right is not None:
                missing = "left"
            return WallObservation(
                stamp=stamp,
                frame_valid=True,
                walls_valid=False,
                point_count=len(finite),
                front_clearance=front_clearance,
                reason="{}_wall_not_found".format(missing),
            )

        # 两侧都拟合成功仍需检查平行性；斜交直线通常是障碍边缘或错误匹配。
        parallel_error = abs(self._wrap_angle(left.angle - right.angle))
        if parallel_error > cfg.max_parallel_angle:
            return WallObservation(
                stamp=stamp,
                frame_valid=True,
                walls_valid=False,
                point_count=len(finite),
                left_heading=left.angle,
                right_heading=right.angle,
                front_clearance=front_clearance,
                reason="walls_not_parallel",
            )

        # 原点到 ax-y+b=0 的法向距离为 |b|/sqrt(1+a²)。
        # 左墙截距应为正、右墙截距应为负，故右侧显式取负得到正距离。
        left_distance = left.intercept / math.sqrt(1.0 + left.slope * left.slope)
        right_distance = -right.intercept / math.sqrt(1.0 + right.slope * right.slope)
        width = left_distance + right_distance

        # ===== 完整矩形 footprint 的四角侧向净空 =====
        # 雷达虽然只看前方，但连续直墙已由前方回波拟合出来，因此可把墙线外推到 x<0，
        # 预测后角扫墙风险。这个外推不能发现车后独立障碍，状态字段会明确标为 rear。
        # 左墙 y=a*x+b 的车内区域位于直线下方。左前角到墙的有符号法向净空为：
        #   (a*x+b-y)/sqrt(1+a²)
        # 把 a=tan(angle)、d=b/sqrt(1+a²) 代入，可得到下面的数值稳定形式。
        front_x = cfg.footprint_front_extent
        rear_x = cfg.footprint_rear_extent
        half_width = cfg.footprint_half_width
        left_front_clearance = (
            left_distance
            + front_x * math.sin(left.angle)
            - half_width * math.cos(left.angle)
        )
        left_rear_clearance = (
            left_distance
            - rear_x * math.sin(left.angle)
            - half_width * math.cos(left.angle)
        )
        # 右墙的车内区域位于直线上方；右前角坐标为 (front_x, -half_width)。
        right_front_clearance = (
            right_distance
            - front_x * math.sin(right.angle)
            - half_width * math.cos(right.angle)
        )
        right_rear_clearance = (
            right_distance
            + rear_x * math.sin(right.angle)
            - half_width * math.cos(right.angle)
        )
        # 对一条直墙和凸矩形，最近点必定位于该侧的前角或后角，所以无需采样整条边。
        left_footprint_clearance = min(
            left_front_clearance, left_rear_clearance
        )
        right_footprint_clearance = min(
            right_front_clearance, right_rear_clearance
        )
        minimum_front_clearance = min(
            left_front_clearance, right_front_clearance
        )
        minimum_footprint_clearance = min(
            left_footprint_clearance, right_footprint_clearance
        )
        if left_distance <= 0.0 or right_distance <= 0.0:
            reason = "vehicle_not_between_walls"
        elif not cfg.min_corridor_width <= width <= cfg.max_corridor_width:
            reason = "corridor_width_out_of_range"
        else:
            reason = "ok"

        if reason != "ok":
            return WallObservation(
                stamp=stamp,
                frame_valid=True,
                walls_valid=False,
                point_count=len(finite),
                left_distance=left_distance,
                right_distance=right_distance,
                width=width,
                left_heading=left.angle,
                right_heading=right.angle,
                left_front_clearance=left_front_clearance,
                right_front_clearance=right_front_clearance,
                minimum_front_clearance=minimum_front_clearance,
                left_rear_clearance=left_rear_clearance,
                right_rear_clearance=right_rear_clearance,
                left_footprint_clearance=left_footprint_clearance,
                right_footprint_clearance=right_footprint_clearance,
                minimum_footprint_clearance=minimum_footprint_clearance,
                front_clearance=front_clearance,
                reason=reason,
            )

        # 左距更大表示车辆更靠右，因此通道中心在左侧，ey 为正并命令正 vy。
        lateral_error = 0.5 * (left_distance - right_distance)
        # 用单位向量求圆周平均，避免直接平均 +π/-π 附近角度产生错误。
        heading_error = math.atan2(
            math.sin(left.angle) + math.sin(right.angle),
            math.cos(left.angle) + math.cos(right.angle),
        )
        # 置信度仅用于观察：点数越多、残差越小、两墙越平行，分数越高。
        # 当前安全门控不按 confidence 放宽条件，防止低分观测仍推动小车。
        support_score = min(
            1.0,
            min(left.inlier_count, right.inlier_count) / float(cfg.min_side_points * 2),
        )
        residual_score = max(
            0.0,
            1.0
            - max(left.rms_error, right.rms_error) / cfg.ransac_inlier_distance,
        )
        parallel_score = max(0.0, 1.0 - parallel_error / cfg.max_parallel_angle)
        confidence = support_score * residual_score * parallel_score
        return WallObservation(
            stamp=stamp,
            frame_valid=True,
            walls_valid=True,
            point_count=len(finite),
            left_distance=left_distance,
            right_distance=right_distance,
            width=width,
            lateral_error=lateral_error,
            heading_error=heading_error,
            left_heading=left.angle,
            right_heading=right.angle,
            left_front_clearance=left_front_clearance,
            right_front_clearance=right_front_clearance,
            minimum_front_clearance=minimum_front_clearance,
            left_rear_clearance=left_rear_clearance,
            right_rear_clearance=right_rear_clearance,
            left_footprint_clearance=left_footprint_clearance,
            right_footprint_clearance=right_footprint_clearance,
            minimum_footprint_clearance=minimum_footprint_clearance,
            front_clearance=front_clearance,
            confidence=confidence,
            reason="ok",
        )

    def invalid_observation(
        self, stamp: float, reason: str, point_count: int = 0
    ) -> WallObservation:
        """为点云读取/TF 等外部错误构造不含旧几何的统一无效观测。"""

        return WallObservation(
            stamp=stamp,
            frame_valid=False,
            walls_valid=False,
            point_count=point_count,
            reason=reason,
        )

    def _fit_wall(self, points: Sequence[Point2D], seed: int) -> Optional[_LineFit]:
        """对单侧点执行有界、可复现的 RANSAC，并用最小二乘二次细化。"""

        cfg = self._config
        if len(points) < cfg.min_side_points:
            return None
        sample = self._bounded_sample(points, cfg.fit_sample_limit)
        if len(sample) < cfg.min_side_points:
            return None

        # 使用局部随机数生成器，不污染进程全局 random 状态。
        generator = random.Random(seed + len(sample))
        best_inliers: List[Point2D] = []
        best_span = 0.0
        # 每次随机取两点确定候选 y=a*x+b，再统计法向距离内点。
        for _ in range(cfg.ransac_iterations):
            first_index, second_index = generator.sample(range(len(sample)), 2)
            first = sample[first_index]
            second = sample[second_index]
            dx = second[0] - first[0]
            # 两点 x 几乎相同会形成垂直线；该简化模型只接受大致沿 x 的墙。
            if abs(dx) < 1.0e-6:
                continue
            slope = (second[1] - first[1]) / dx
            angle = math.atan(slope)
            if abs(angle) > cfg.max_wall_heading:
                continue
            intercept = first[1] - slope * first[0]
            scale = math.sqrt(1.0 + slope * slope)
            inliers = [
                point
                for point in sample
                if abs(point[1] - slope * point[0] - intercept) / scale
                <= cfg.ransac_inlier_distance
            ]
            if len(inliers) < cfg.min_side_points:
                continue
            span = max(point[0] for point in inliers) - min(point[0] for point in inliers)
            if span < cfg.min_wall_span:
                continue
            # 第一目标最大化内点数；数量相同则优先选择前向覆盖更长的候选。
            if len(inliers) > len(best_inliers) or (
                len(inliers) == len(best_inliers) and span > best_span
            ):
                best_inliers = inliers
                best_span = span

        if len(best_inliers) < cfg.min_side_points:
            return None
        # 用最佳 RANSAC 内点做第一次最小二乘，降低随机两点带来的参数抖动。
        refined = self._least_squares(best_inliers)
        if refined is None:
            return None
        slope, intercept = refined
        angle = math.atan(slope)
        if abs(angle) > cfg.max_wall_heading:
            return None

        scale = math.sqrt(1.0 + slope * slope)
        # 围绕细化直线重新选内点，再拟合一次得到最终 slope/intercept。
        final_inliers = [
            point
            for point in sample
            if abs(point[1] - slope * point[0] - intercept) / scale
            <= cfg.ransac_inlier_distance
        ]
        if len(final_inliers) < cfg.min_side_points:
            return None
        final_refined = self._least_squares(final_inliers)
        if final_refined is None:
            return None
        slope, intercept = final_refined
        angle = math.atan(slope)
        scale = math.sqrt(1.0 + slope * slope)
        span = max(point[0] for point in final_inliers) - min(
            point[0] for point in final_inliers
        )
        if span < cfg.min_wall_span or abs(angle) > cfg.max_wall_heading:
            return None
        residuals = [
            (point[1] - slope * point[0] - intercept) / scale
            for point in final_inliers
        ]
        rms_error = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        return _LineFit(
            slope=slope,
            intercept=intercept,
            angle=angle,
            inlier_count=len(final_inliers),
            sample_count=len(sample),
            span=span,
            rms_error=rms_error,
        )

    @staticmethod
    def _bounded_sample(points: Sequence[Point2D], limit: int) -> List[Point2D]:
        """按 x 排序后均匀抽样，既限制耗时又保留整段墙的空间覆盖。"""

        ordered = sorted(points)
        if len(ordered) <= limit:
            return ordered
        if limit == 1:
            return [ordered[0]]
        return [
            ordered[int(round(index * (len(ordered) - 1) / float(limit - 1)))]
            for index in range(limit)
        ]

    @staticmethod
    def _least_squares(points: Sequence[Point2D]) -> Optional[Tuple[float, float]]:
        """最小化 y 方向残差拟合 ``y=a*x+b``；x 无方差时返回 None。"""

        count = float(len(points))
        mean_x = sum(point[0] for point in points) / count
        mean_y = sum(point[1] for point in points) / count
        denominator = sum((point[0] - mean_x) ** 2 for point in points)
        if denominator <= 1.0e-12:
            return None
        slope = sum(
            (point[0] - mean_x) * (point[1] - mean_y) for point in points
        ) / denominator
        return slope, mean_y - slope * mean_x

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """把任意角度折返到 [-π, π]，用于计算最短方向差。"""

        return math.atan2(math.sin(angle), math.cos(angle))
