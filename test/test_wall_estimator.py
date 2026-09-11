#!/usr/bin/env python3
"""双侧墙面估计器的纯 Python 单元测试。"""

import math
import unittest

from ranger_mini_narrow_sim.wall_estimator import (
    WallEstimator,
    WallEstimatorConfig,
)


def corridor_points(heading=0.0, center=0.0, width=1.4, count=48):
    """生成带轻微确定性噪声的左右墙点，用于复现实验输入。"""

    # 墙线模型为 y = tan(heading) * x + intercept。
    slope = math.tan(heading)
    cosine = math.cos(heading)
    points = []
    for index in range(count):
        x = 0.30 + 0.065 * index
        noise = 0.008 * math.sin(index * 1.3)
        left_y = slope * x + (center + width / 2.0) / cosine + noise
        right_y = slope * x + (center - width / 2.0) / cosine - noise
        points.extend(((x, left_y), (x, right_y)))
    return points


class WallEstimatorTest(unittest.TestCase):
    """分别覆盖正常双墙、前方障碍物和单侧墙缺失场景。"""

    def setUp(self):
        # 每个测试都使用全新的估计器，避免历史滤波状态相互影响。
        self.estimator = WallEstimator(
            WallEstimatorConfig(
                min_total_points=20,
                min_side_points=10,
                ransac_iterations=100,
                min_wall_span=0.7,
            )
        )

    def test_recovers_center_and_heading(self):
        """检查算法能从点云中恢复通道宽度、中心偏移和朝向。"""

        observation = self.estimator.estimate(
            corridor_points(heading=0.10, center=0.16), stamp=1.0
        )
        self.assertTrue(observation.frame_valid)
        self.assertTrue(observation.walls_valid)
        self.assertAlmostEqual(observation.width, 1.4, delta=0.04)
        self.assertAlmostEqual(observation.lateral_error, 0.16, delta=0.03)
        self.assertAlmostEqual(observation.heading_error, 0.10, delta=0.02)

    def test_reports_front_obstacle_independently(self):
        """前方障碍距离应独立于左右墙拟合结果计算。"""

        points = corridor_points()
        points.extend(((0.48, 0.02), (0.52, -0.04), (1.0, 0.0)))
        observation = self.estimator.estimate(points, stamp=2.0)
        self.assertTrue(observation.walls_valid)
        self.assertAlmostEqual(observation.front_clearance, 0.48)

    def test_full_footprint_clearance_uses_vehicle_size_and_heading(self):
        """四角净空应体现车身尺寸，并让前后角共同约束完整矩形。"""

        observation = self.estimator.estimate(
            corridor_points(heading=0.10, center=0.0), stamp=2.5
        )
        self.assertTrue(observation.walls_valid)
        expected_left = 0.7 + 0.42 * math.sin(0.10) - 0.35 * math.cos(0.10)
        expected_right = 0.7 - 0.42 * math.sin(0.10) - 0.35 * math.cos(0.10)
        expected_left_rear = (
            0.7 - 0.42 * math.sin(0.10) - 0.35 * math.cos(0.10)
        )
        expected_right_rear = (
            0.7 + 0.42 * math.sin(0.10) - 0.35 * math.cos(0.10)
        )
        self.assertAlmostEqual(
            observation.left_front_clearance, expected_left, delta=0.03
        )
        self.assertAlmostEqual(
            observation.right_front_clearance, expected_right, delta=0.03
        )
        self.assertAlmostEqual(
            observation.minimum_front_clearance, expected_right, delta=0.03
        )
        self.assertAlmostEqual(
            observation.left_rear_clearance, expected_left_rear, delta=0.03
        )
        self.assertAlmostEqual(
            observation.right_rear_clearance, expected_right_rear, delta=0.03
        )
        self.assertAlmostEqual(
            observation.left_footprint_clearance,
            expected_left_rear,
            delta=0.03,
        )
        self.assertAlmostEqual(
            observation.right_footprint_clearance,
            expected_right,
            delta=0.03,
        )
        self.assertAlmostEqual(
            observation.minimum_footprint_clearance,
            expected_right,
            delta=0.03,
        )

    def test_missing_side_does_not_reuse_geometry(self):
        """缺少右墙时必须判无效，不能继续沿用上一帧的双墙几何。"""

        left_only = [point for point in corridor_points() if point[1] > 0.0]
        observation = self.estimator.estimate(left_only, stamp=3.0)
        self.assertTrue(observation.frame_valid)
        self.assertFalse(observation.walls_valid)
        self.assertEqual(observation.reason, "right_wall_not_found")


if __name__ == "__main__":
    unittest.main()
