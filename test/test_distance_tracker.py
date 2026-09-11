#!/usr/bin/env python3
"""纵向任务进度计算器的纯 Python 测试，不需要启动 ROS。"""

import math
import unittest

from ranger_mini_narrow_sim.distance_tracker import DistanceTracker


class DistanceTrackerTest(unittest.TestCase):
    """验证起始航向投影、横移隔离以及里程计跳变过滤。"""

    def test_projects_displacement_onto_start_heading(self):
        """车头初始朝 +y 时，x 横移不应计入纵向任务距离。"""

        tracker = DistanceTracker(max_step=2.0)
        tracker.reset(0.0, 0.0, math.pi / 2.0)
        tracker.update(0.4, 0.8)
        update = tracker.update(0.7, 1.6)
        self.assertTrue(update.accepted)
        self.assertAlmostEqual(update.distance, 1.6)

    def test_backward_motion_reduces_progress(self):
        """纵向进度是有符号投影，倒车不能帮助达到正向 9 m 目标。"""

        tracker = DistanceTracker(max_step=2.0)
        tracker.reset(1.0, 2.0, 0.0)
        update = tracker.update(0.6, 2.5)
        self.assertAlmostEqual(update.distance, -0.4)

    def test_rejects_jump_without_moving_baseline(self):
        """拒绝异常跳点后仍保留旧基准，使下一帧正常数据可以继续计算。"""

        tracker = DistanceTracker(max_step=0.5)
        tracker.reset(0.0, 0.0, 0.0)
        rejected = tracker.update(2.0, 0.0)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "odom_jump")
        accepted = tracker.update(0.3, 0.0)
        self.assertTrue(accepted.accepted)
        self.assertAlmostEqual(accepted.distance, 0.3)


if __name__ == "__main__":
    unittest.main()
