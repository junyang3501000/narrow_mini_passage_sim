#!/usr/bin/env python3
"""窄道 Gazebo 仿真的端到端自动验收程序。

该脚本不是车辆控制器，而是一个测试监视器：它启动窄道模式、观察里程计和状态话题，
最终检查车辆是否走完目标距离、是否保持居中、朝向误差是否合格，以及停车指令是否归零。
"""

import argparse
import json
import math
import sys
import threading
import time
from typing import Optional, Tuple

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool


class SimulationMonitor:
    """线程安全地汇总控制器状态与里程计，供主测试循环读取。"""

    def __init__(self):
        # rospy 的订阅回调与主循环可能并发访问这些字段，因此统一使用互斥锁保护。
        self._lock = threading.Lock()
        self._state: Optional[str] = None
        self._reason: Optional[str] = None
        self._reported_distance = 0.0
        self._lateral_error: Optional[float] = None
        self._heading_error: Optional[float] = None
        self._command_norm = 0.0
        self._start_position: Optional[Tuple[float, float]] = None
        self._last_position: Optional[Tuple[float, float]] = None
        self._saw_running = False
        self._saw_side_recovery = False

        # 状态话题用于检查控制结果；里程计用于独立验证车辆真实位移。
        rospy.Subscriber("/narrow_passage/status", String, self._status_callback)
        rospy.Subscriber("/odom", Odometry, self._odom_callback)

    def _status_callback(self, message: String) -> None:
        """解析控制器发布的 JSON 状态；畸形消息直接忽略。"""

        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._state = status.get("state")
            self._reason = status.get("reason")
            self._reported_distance = float(status.get("distance", 0.0))
            # 出口续行时前向雷达会自然失去两侧墙，最终状态中的误差为 null；
            # 保留最后一次有效墙观测，用它验收进入出口前已经完成居中和对正。
            lateral_error = status.get("ey")
            heading_error = status.get("e_theta")
            if lateral_error is not None:
                self._lateral_error = lateral_error
            if heading_error is not None:
                self._heading_error = heading_error
            # 三个速度分量中的最大绝对值可判断最终停车指令是否完全归零。
            self._command_norm = max(
                abs(float(status.get("linear_x", 0.0))),
                abs(float(status.get("linear_y", 0.0))),
                abs(float(status.get("angular_z", 0.0))),
            )
            if self._state == "running":
                self._saw_running = True
            if self._state == "side_recovery":
                self._saw_side_recovery = True

    def _odom_callback(self, message: Odometry) -> None:
        """记录测试开始位置和最新位置，用直线位移交叉验证报告距离。"""

        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        with self._lock:
            if self._start_position is None:
                self._start_position = position
            self._last_position = position

    def snapshot(self):
        """原子地复制当前测试状态，避免主循环读到一半更新的数据。"""

        with self._lock:
            displacement = 0.0
            if self._start_position is not None and self._last_position is not None:
                displacement = math.hypot(
                    self._last_position[0] - self._start_position[0],
                    self._last_position[1] - self._start_position[1],
                )
            return {
                "state": self._state,
                "reason": self._reason,
                "reported_distance": self._reported_distance,
                "displacement": displacement,
                "saw_running": self._saw_running,
                "saw_side_recovery": self._saw_side_recovery,
                "odom_seen": self._last_position is not None,
                "lateral_error": self._lateral_error,
                "heading_error": self._heading_error,
                "command_norm": self._command_norm,
            }


def parse_args():
    """读取 roslaunch 传入的测试超时和最小通过距离。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=80.0)
    parser.add_argument("--minimum-distance", type=float, default=9.75)
    parser.add_argument(
        "--require-side-recovery",
        action="store_true",
        help="要求测试期间至少观测到一次 side_recovery 状态",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def fail(message: str, snapshot) -> int:
    """用统一格式输出失败原因和现场快照，并返回非零退出码。"""

    rospy.logerr("SIM_RESULT failed: %s; snapshot=%s", message, json.dumps(snapshot))
    return 1


def main() -> int:
    """启动测试、使能控制器，并等待成功、故障或超时。"""

    rospy.init_node("narrow_passage_smoke_test")
    args = parse_args()
    monitor = SimulationMonitor()
    deadline = time.monotonic() + args.timeout

    # 控制器通过 SetBool 服务启停；先等待服务出现，再开始计时验收。
    try:
        rospy.wait_for_service("/narrow_mode/enable", timeout=20.0)
        enable = rospy.ServiceProxy("/narrow_mode/enable", SetBool)
    except rospy.ROSException:
        return fail("enable service did not appear", monitor.snapshot())

    # 状态和里程计都出现后再使能，保证测试从一个完整、可测量的初始状态开始。
    while time.monotonic() < deadline and not rospy.is_shutdown():
        snapshot = monitor.snapshot()
        if snapshot["state"] is not None and snapshot["odom_seen"]:
            try:
                response = enable(True)
                if not response.success:
                    return fail("controller rejected enable request", snapshot)
                break
            except rospy.ServiceException:
                # 服务可能恰好在重启，短暂失败时继续重试，直至总超时。
                pass
        time.sleep(0.1)
    else:
        return fail("sensor/status startup timed out", monitor.snapshot())

    while time.monotonic() < deadline and not rospy.is_shutdown():
        snapshot = monitor.snapshot()
        # fault 是立即失败条件，不必继续等待总超时。
        if snapshot["state"] == "fault":
            return fail("controller entered fault", snapshot)
        if snapshot["state"] == "success":
            # 以下断言同时覆盖状态机、路程、真实运动、停车、居中和朝向六个方面。
            if not snapshot["saw_running"]:
                return fail("success was reported without a running phase", snapshot)
            if args.require_side_recovery and not snapshot["saw_side_recovery"]:
                return fail("required footprint side recovery was not observed", snapshot)
            if snapshot["reported_distance"] < args.minimum_distance:
                return fail("reported path distance is too short", snapshot)
            if snapshot["displacement"] < args.minimum_distance - 0.25:
                return fail("odometry displacement is too short", snapshot)
            if snapshot["command_norm"] > 1.0e-6:
                return fail("controller did not publish a zero final command", snapshot)
            if snapshot["lateral_error"] is None or abs(snapshot["lateral_error"]) > 0.04:
                return fail("final lateral centring error is too large", snapshot)
            if snapshot["heading_error"] is None or abs(snapshot["heading_error"]) > math.radians(1.5):
                return fail("final heading error is too large", snapshot)
            rospy.loginfo("SIM_RESULT passed: %s", json.dumps(snapshot))
            return 0
        time.sleep(0.1)

    return fail("simulation timed out", monitor.snapshot())


if __name__ == "__main__":
    sys.exit(main())
