#!/usr/bin/env python3
"""集中参数加载模块的纯 Python 测试。"""

import unittest

from ranger_mini_narrow_sim.controller import CommandSlewLimiter, NarrowController
from ranger_mini_narrow_sim.node_config import load_node_config
from ranger_mini_narrow_sim.safety_supervisor import SafetySupervisor
from ranger_mini_narrow_sim.wall_estimator import WallEstimator


class NodeConfigTest(unittest.TestCase):
    """确认默认参数能完整构造所有算法模块，并正确清理 frame 前导斜杠。"""

    def test_default_config_builds_all_modules(self):
        """不启动 ROS Master，也能验证全部默认参数组合。"""

        overrides = {"~source_frame": "/livox_frame"}

        def get_param(name, default):
            return overrides.get(name, default)

        config = load_node_config(get_param)
        self.assertEqual(config.source_frame, "livox_frame")
        self.assertEqual(config.passage_distance, 10.0)
        self.assertEqual(config.exit_coast_distance, 0.0)
        self.assertEqual(config.safety.wall_confirm_frames, 3)
        WallEstimator(config.estimator)
        NarrowController(config.controller)
        CommandSlewLimiter(config.slew)
        SafetySupervisor(config.safety)

    def test_exit_coast_distance_cannot_be_negative(self):
        """通用默认可关闭出口续行，但负距离属于无效配置。"""

        def get_param(name, default):
            return -0.1 if name == "~exit_coast_distance" else default

        with self.assertRaises(ValueError):
            load_node_config(get_param)


if __name__ == "__main__":
    unittest.main()
