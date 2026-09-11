#!/usr/bin/env python3
"""控制器单元测试：验证修正方向、底盘模式和紧急停车行为。"""

import unittest

from ranger_mini_narrow_sim.controller import (
    CommandSlewLimiter,
    ControllerConfig,
    NarrowController,
    SlewConfig,
)
from ranger_mini_narrow_sim.models import VelocityCommand, WallObservation


class ControllerTest(unittest.TestCase):
    """用人工构造的墙面观测验证控制输出，不依赖 ROS 或 Gazebo。"""

    @staticmethod
    def observation(ey=0.0, heading=0.0):
        """生成一帧有效观测；ey 为横向偏差，heading 为朝向偏差。"""

        return WallObservation(
            stamp=1.0,
            frame_valid=True,
            walls_valid=True,
            point_count=100,
            lateral_error=ey,
            heading_error=heading,
        )

    def test_positive_errors_command_left_correction(self):
        """车体位于中心线右侧且向右偏时，应给出向左平移和左转修正。"""

        controller = NarrowController(ControllerConfig())
        result = controller.compute(self.observation(ey=0.12, heading=0.08))
        self.assertGreater(result.command.linear_x, 0.0)
        self.assertGreater(result.command.linear_y, 0.0)
        self.assertGreater(result.command.angular_z, 0.0)

    def test_lateral_command_can_be_disabled_for_non_holonomic_base(self):
        """关闭横移能力后，控制器只能用角速度逐渐修正横向误差。"""

        controller = NarrowController(
            ControllerConfig(allow_lateral_motion=False, center_angular_kp=0.5)
        )
        result = controller.compute(self.observation(ey=-0.2))
        self.assertEqual(result.command.linear_y, 0.0)
        self.assertLess(result.command.angular_z, 0.0)

    def test_slew_limiter_bypasses_ramp_for_stop(self):
        """正常运动受加速度限制，但 stop 必须立即把全部速度分量清零。"""

        limiter = CommandSlewLimiter(
            SlewConfig(
                max_linear_accel=1.0,
                max_lateral_accel=1.0,
                max_angular_accel=1.0,
            )
        )
        moving = limiter.apply(VelocityCommand(1.0, 0.2, 0.3), dt=0.1)
        self.assertGreater(moving.linear_x, 0.0)
        self.assertEqual(limiter.stop(), VelocityCommand())


if __name__ == "__main__":
    unittest.main()
