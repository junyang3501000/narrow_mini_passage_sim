#!/usr/bin/env python3
"""安全迟滞模块的纯 Python 单元测试，不需要 ROS Master。"""

import unittest

from ranger_mini_narrow_sim.models import VelocityCommand, WallObservation
from ranger_mini_narrow_sim.safety_supervisor import SafetyConfig, SafetySupervisor


def valid_observation(
    left=0.25, right=0.25, front=None
) -> WallObservation:
    """构造一帧通过墙体检查的最小观测，便于只测试安全状态。"""

    return WallObservation(
        stamp=1.0,
        frame_valid=True,
        walls_valid=True,
        point_count=100,
        left_footprint_clearance=left,
        right_footprint_clearance=right,
        minimum_footprint_clearance=min(left, right),
        front_clearance=front,
        reason="ok",
    )


class SafetySupervisorTest(unittest.TestCase):
    """覆盖墙线确认、障碍迟滞、预警限速和紧急横移恢复。"""

    def setUp(self):
        self.safety = SafetySupervisor(
            SafetyConfig(
                wall_confirm_frames=3,
                front_stop_distance=0.65,
                front_resume_distance=0.85,
                obstacle_clear_confirm_frames=3,
                footprint_emergency_margin=0.05,
                footprint_warning_margin=0.10,
                footprint_warning_forward_speed=0.12,
                footprint_recovery_lateral_speed=0.04,
                footprint_clear_confirm_frames=3,
                allow_lateral_motion=True,
                max_linear_speed=0.30,
                max_lateral_speed=0.10,
            )
        )

    def test_wall_confirmation_counts_new_observations(self):
        """连续三次显式输入有效观测后，墙线才算确认完成。"""

        for _ in range(2):
            self.safety.process_observation(valid_observation())
            self.assertFalse(self.safety.walls_confirmed)
        self.safety.process_observation(valid_observation())
        self.assertTrue(self.safety.walls_confirmed)

    def test_front_obstacle_requires_three_clear_frames(self):
        """前障碍一帧锁存，越过恢复线后三帧才能解除。"""

        self.safety.process_observation(valid_observation(front=0.50))
        self.assertTrue(self.safety.obstacle_latched)
        for _ in range(2):
            self.safety.process_observation(valid_observation(front=1.0))
            self.assertTrue(self.safety.obstacle_latched)
        self.safety.process_observation(valid_observation(front=1.0))
        self.assertFalse(self.safety.obstacle_latched)

    def test_right_emergency_stops_then_recovers_left(self):
        """右侧进入5 cm区时先停一次，随后只给正 vy 向左脱险。"""

        danger = valid_observation(left=0.30, right=0.03)
        self.safety.process_observation(danger)
        self.assertTrue(self.safety.footprint_emergency_latched)
        self.assertTrue(self.safety.consume_emergency_stop())
        self.assertFalse(self.safety.consume_emergency_stop())
        decision = self.safety.recovery_decision(danger)
        self.assertFalse(decision.stop)
        self.assertEqual(decision.command.linear_x, 0.0)
        self.assertGreater(decision.command.linear_y, 0.0)
        self.assertEqual(decision.command.angular_z, 0.0)

    def test_emergency_releases_after_three_warning_clear_frames(self):
        """必须连续三帧完全退出10 cm预警区，紧急锁存才会解除。"""

        self.safety.process_observation(
            valid_observation(left=0.30, right=0.03)
        )
        for _ in range(2):
            self.safety.process_observation(
                valid_observation(left=0.30, right=0.12)
            )
            self.assertTrue(self.safety.footprint_emergency_latched)
        self.safety.process_observation(
            valid_observation(left=0.30, right=0.12)
        )
        self.assertFalse(self.safety.footprint_emergency_latched)

    def test_left_warning_removes_dangerous_positive_commands(self):
        """左侧预警时，正 vy/正 wz 会继续靠左墙，必须被压成零。"""

        observation = valid_observation(left=0.08, right=0.30)
        side = self.safety.warning_side(observation)
        limited = self.safety.limit_warning_command(
            VelocityCommand(0.25, 0.06, 0.10), side
        )
        self.assertEqual(side, "left")
        self.assertAlmostEqual(limited.linear_x, 0.12)
        self.assertEqual(limited.linear_y, 0.0)
        self.assertEqual(limited.angular_z, 0.0)


if __name__ == "__main__":
    unittest.main()
