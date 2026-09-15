#!/usr/bin/env python3
"""任务状态机的出口墙端续行测试。"""

import unittest

from ranger_mini_narrow_sim.controller import CommandSlewLimiter, NarrowController
from ranger_mini_narrow_sim.mission_controller import MissionController
from ranger_mini_narrow_sim.models import PassageState, WallObservation
from ranger_mini_narrow_sim.node_config import load_node_config
from ranger_mini_narrow_sim.safety_supervisor import SafetySupervisor


class _FakeOdometry:
    """只实现状态机需要的里程接口，避免测试依赖 ROS Master。"""

    def __init__(self):
        self.distance = 0.0

    def reset_task_if_fresh(self, _now, _timeout):
        self.distance = 0.0
        return True

    def clear_task(self):
        self.distance = 0.0

    @staticmethod
    def is_fresh(_now, _timeout):
        return True

    @staticmethod
    def ensure_started():
        return True


def _valid_walls(stamp=0.1):
    return WallObservation(
        stamp=stamp,
        frame_valid=True,
        walls_valid=True,
        point_count=100,
        left_distance=0.60,
        right_distance=0.60,
        width=1.20,
        lateral_error=0.0,
        heading_error=0.0,
        left_footprint_clearance=0.25,
        right_footprint_clearance=0.25,
        minimum_footprint_clearance=0.25,
        reason="ok",
    )


def _lost_wall(reason="too_few_points", stamp=0.2):
    return WallObservation(
        stamp=stamp,
        frame_valid=reason != "too_few_points",
        walls_valid=False,
        point_count=0,
        reason=reason,
    )


class MissionExitTest(unittest.TestCase):
    def setUp(self):
        overrides = {
            "~passage_distance": 10.0,
            "~exit_coast_distance": 1.0,
            "~wall_confirm_frames": 1,
        }

        def get_param(name, default):
            return overrides.get(name, default)

        self.config = load_node_config(get_param)
        self.odometry = _FakeOdometry()
        self.safety = SafetySupervisor(self.config.safety)
        self.mission = MissionController(
            self.config,
            self.odometry,
            NarrowController(self.config.controller),
            self.safety,
            CommandSlewLimiter(self.config.slew),
        )
        self.mission.start(0.0)
        observation = _valid_walls()
        self.safety.process_observation(observation)
        command = self.mission.decide(0.1, observation, 0.1, 0.1)
        self.assertEqual(self.mission.state, PassageState.RUNNING)
        self.assertGreater(command.linear_x, 0.0)

    def test_natural_wall_end_coasts_only_near_exit(self):
        self.odometry.distance = 9.4
        observation = _lost_wall()
        self.safety.process_observation(observation)
        command = self.mission.decide(0.2, observation, 0.2, 0.1)

        self.assertEqual(self.mission.state, PassageState.EXITING)
        self.assertEqual(self.mission.reason, "exit_wall_end_coast")
        self.assertGreater(command.linear_x, 0.0)
        self.assertEqual(command.linear_y, 0.0)
        self.assertEqual(command.angular_z, 0.0)

        self.odometry.distance = 10.0
        command = self.mission.decide(0.3, observation, 0.3, 0.1)
        self.assertEqual(self.mission.state, PassageState.SUCCESS)
        self.assertEqual(command.linear_x, 0.0)

    def test_wall_loss_too_early_stops(self):
        self.odometry.distance = 8.5
        observation = _lost_wall()
        self.safety.process_observation(observation)
        command = self.mission.decide(0.2, observation, 0.2, 0.1)

        self.assertEqual(self.mission.state, PassageState.WAITING_SENSORS)
        self.assertEqual(command.linear_x, 0.0)

    def test_tf_failure_near_exit_still_stops(self):
        self.odometry.distance = 9.5
        observation = _lost_wall("pointcloud_or_tf_error")
        self.safety.process_observation(observation)
        command = self.mission.decide(0.2, observation, 0.2, 0.1)

        self.assertEqual(self.mission.state, PassageState.WAITING_SENSORS)
        self.assertEqual(command.linear_x, 0.0)


if __name__ == "__main__":
    unittest.main()
