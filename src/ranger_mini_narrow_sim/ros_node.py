"""简化窄道 ROS1 节点：只负责通信、任务状态和模块编排。

功能实现已经拆分：参数在 node_config.py，PointCloud2/TF 在 pointcloud_transformer.py，
墙线识别在 wall_estimator.py，里程在 odometry.py，安全迟滞在 safety_supervisor.py，
状态字典在 status_report.py。本文件不再包含这些子功能的具体数学实现。
"""

import json
import threading
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

from .controller import CommandSlewLimiter, NarrowController
from .mission_controller import MissionController
from .models import VelocityCommand, WallObservation
from .node_config import NodeConfig, load_node_config
from .odometry import OdometryProgress
from .pointcloud_transformer import PointCloudTransformer
from .safety_supervisor import SafetySupervisor
from .status_report import build_status
from .wall_estimator import WallEstimator


class RangerNarrowControllerNode:
    """把 ROS 数据送入独立模块，并按安全优先级发布最终 Twist。"""

    def __init__(self):
        # Subscriber 回调与 Timer 可能并发运行，共享快照和任务状态统一加锁。
        self._lock = threading.RLock()
        self._config: NodeConfig = load_node_config(rospy.get_param)

        # ===== 独立功能模块 =====
        self._cloud_transformer = PointCloudTransformer(
            source_frame=self._config.source_frame,
            target_frame=self._config.target_frame,
            tf_timeout=self._config.tf_timeout,
            point_stride=self._config.point_stride,
            max_points=self._config.max_points,
            min_obstacle_z=self._config.min_obstacle_z,
            max_obstacle_z=self._config.max_obstacle_z,
        )
        self._wall_estimator = WallEstimator(self._config.estimator)
        self._odometry = OdometryProgress(self._config.max_odom_step)
        self._controller = NarrowController(self._config.controller)
        self._safety = SafetySupervisor(self._config.safety)
        self._slew_limiter = CommandSlewLimiter(self._config.slew)
        self._mission = MissionController(
            self._config,
            self._odometry,
            self._controller,
            self._safety,
            self._slew_limiter,
        )

        # ===== 最新雷达观测快照 =====
        # sequence 使“连续三帧”等计数只随新雷达帧增加，不随 20 Hz Timer 重复增加。
        self._observation: Optional[WallObservation] = None
        self._observation_sequence = 0
        self._processed_observation_sequence = -1
        self._lidar_received_at: Optional[float] = None
        self._transform_valid = False

        # ===== ROS Timer/状态发布自身的少量运行数据 =====
        self._last_control_at: Optional[float] = None
        self._status_counter = 0

        self._create_ros_interfaces()
        if self._config.enabled_on_start:
            # 安全启动默认值；演示 launch 会显式覆盖为 true。
            self._mission.start(rospy.get_time())

        rospy.logwarn(
            "ranger_mini_narrow_sim publishes directly to %s; ensure it is the only active command publisher",
            self._config.cmd_topic,
        )
        rospy.loginfo(
            "Ranger narrow controller ready: lidar=%s odom=%s frames=%s->%s distance=%.2f m",
            self._config.lidar_topic,
            self._config.odom_topic,
            self._config.source_frame,
            self._config.target_frame,
            self._config.passage_distance,
        )

    def _create_ros_interfaces(self) -> None:
        """集中创建话题、服务、定时器和退出回调。"""

        self._cmd_pub = rospy.Publisher(
            self._config.cmd_topic, Twist, queue_size=1
        )
        self._status_pub = rospy.Publisher(
            self._config.status_topic, String, queue_size=1, latch=True
        )
        self._lidar_sub = rospy.Subscriber(
            self._config.lidar_topic,
            PointCloud2,
            self._lidar_callback,
            queue_size=1,
            # Livox 点云可能很大，提高 TCP 缓冲避免默认缓冲截断或积压。
            buff_size=32 * 1024 * 1024,
        )
        self._odom_sub = rospy.Subscriber(
            self._config.odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=10,
        )
        self._enable_server = rospy.Service(
            self._config.enable_service, SetBool, self._enable_callback
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._config.control_rate),
            self._control_callback,
        )
        rospy.on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------------
    # ROS 输入回调：只做消息交接，不实现点云/里程数学
    # ------------------------------------------------------------------
    def _lidar_callback(self, message: PointCloud2) -> None:
        """PointCloud2 -> TF模块 -> 墙线模块，最后原子替换观测快照。"""

        received_at = rospy.get_time()
        try:
            points = self._cloud_transformer.transform(message)
            observation = self._wall_estimator.estimate(points, received_at)
            transform_valid = True
        except Exception as error:
            observation = self._wall_estimator.invalid_observation(
                received_at, "pointcloud_or_tf_error"
            )
            transform_valid = False
            rospy.logerr_throttle(
                2.0, "PointCloud2/TF processing failed: %s", error
            )

        with self._lock:
            self._observation = observation
            self._lidar_received_at = received_at
            self._transform_valid = transform_valid
            self._observation_sequence += 1

    def _odom_callback(self, message: Odometry) -> None:
        """把 odom 交给里程模块；活动任务中的坏样本会触发锁定故障。"""

        received_at = rospy.get_time()
        with self._lock:
            update = self._odometry.ingest(
                message, received_at, self._mission.tracking_odometry
            )
            if update is not None and not update.accepted:
                self._mission.fault(update.reason)

    # ------------------------------------------------------------------
    # 任务开始、停止和故障
    # ------------------------------------------------------------------
    def _enable_callback(self, request) -> SetBoolResponse:
        """SetBool true 开新任务；false 立即发布零速度并清空任务进度。"""

        now = rospy.get_time()
        with self._lock:
            if request.data:
                self._mission.start(now)
                message = (
                    "new {:.2f} m narrow-passage run armed; waiting for valid sensors"
                ).format(self._config.passage_distance)
            else:
                self._mission.disable("external_disable")
                message = "narrow-passage controller disabled"
        if not request.data:
            # 不等待下一个 Timer，服务返回前先额外发布一次零速度。
            self._publish_twist(VelocityCommand())
        rospy.logwarn("Ranger narrow mode enable=%s: %s", request.data, message)
        return SetBoolResponse(success=True, message=message)

    # ------------------------------------------------------------------
    # 周期编排与安全优先级
    # ------------------------------------------------------------------
    def _control_callback(self, _event) -> None:
        """读取一致快照、调用决策、发布 Twist，并按较低频率发布状态。"""

        now = rospy.get_time()
        with self._lock:
            observation = self._observation
            if observation is None:
                observation = self._wall_estimator.invalid_observation(
                    now, "no_lidar_data"
                )

            if self._observation_sequence != self._processed_observation_sequence:
                self._safety.process_observation(observation)
                self._processed_observation_sequence = self._observation_sequence

            dt = (
                1.0 / self._config.control_rate
                if self._last_control_at is None
                else max(0.0, min(0.5, now - self._last_control_at))
            )
            self._last_control_at = now
            command = self._mission.decide(
                now, observation, self._lidar_received_at, dt
            )

            status = build_status(
                now=now,
                enabled=self._mission.enabled,
                state=self._mission.state,
                reason=self._mission.reason,
                observation=observation,
                transform_valid=self._transform_valid,
                distance=self._odometry.distance,
                command=command,
                config=self._config,
                safety=self._safety,
            )
            self._status_counter += 1
            divisor = max(
                1,
                int(round(self._config.control_rate / self._config.status_rate)),
            )
            publish_status = self._status_counter % divisor == 0

        # 发布放在锁外，避免 ROS 网络阻塞订阅回调。
        self._publish_twist(command)
        if publish_status:
            self._status_pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------
    # ROS 输出与生命周期
    # ------------------------------------------------------------------
    def _publish_twist(self, command: VelocityCommand) -> None:
        """把 ROS 无关速度模型复制到 geometry_msgs/Twist。"""

        message = Twist()
        message.linear.x = command.linear_x
        message.linear.y = command.linear_y
        message.angular.z = command.angular_z
        self._cmd_pub.publish(message)

    def _on_shutdown(self) -> None:
        """节点退出时连续发布三次零速度，提高底盘收到停车命令的概率。"""

        for _ in range(3):
            self._publish_twist(VelocityCommand())
            time.sleep(0.02)
        rospy.loginfo("ranger_mini_narrow_sim stopped with zero velocity")

    def spin(self) -> None:
        """进入 rospy 事件循环。"""

        rospy.spin()


def main() -> None:
    """创建固定节点名并启动适配器。"""

    rospy.init_node("ranger_narrow_controller")
    RangerNarrowControllerNode().spin()
