"""PointCloud2 读取、TF 查询和 base_link 平面点转换。

墙面拟合不应该关心 ROS 消息字段和四元数。本模块把一帧雷达消息转换成过滤后的
``[(x, y), ...]``；字段错误、frame 不一致和 TF 超时均通过异常交给上层安全停车。
"""

import math
from typing import List, Optional, Tuple

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2


Point2D = Tuple[float, float]


class PointCloudTransformer:
    """拥有 TF 缓存，并把 Livox PointCloud2 转成 base_link 二维点。"""

    def __init__(
        self,
        source_frame: str,
        target_frame: str,
        tf_timeout: float,
        point_stride: int,
        max_points: int,
        min_obstacle_z: float,
        max_obstacle_z: float,
    ):
        self.source_frame = source_frame
        self.target_frame = target_frame
        self._tf_timeout = tf_timeout
        self._point_stride = point_stride
        self._max_points = max_points
        self._min_obstacle_z = min_obstacle_z
        self._max_obstacle_z = max_obstacle_z

        # listener 必须保存为成员，否则对象被回收后 TF 订阅也会停止。
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def transform(self, message: PointCloud2) -> List[Point2D]:
        """读取当前帧并返回 base_link 中通过高度过滤的平面点。"""

        frame_id = (message.header.frame_id or "").strip().lstrip("/")
        if frame_id != self.source_frame:
            raise ValueError(
                "unexpected frame '{}' (expected '{}')".format(
                    frame_id, self.source_frame
                )
            )

        transform = self._lookup_transform(message, frame_id)
        points: List[Point2D] = []
        for index, raw_point in enumerate(
            point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
        ):
            # 先按消息中的原始顺序均匀降采样，再做较昂贵的四元数旋转。
            if index % self._point_stride != 0:
                continue
            x, y, z = self._transform_point(raw_point, transform)
            # 必须在 base_link 中过滤高度，因为雷达自身 z=0 不代表车体高度。
            if not self._min_obstacle_z <= z <= self._max_obstacle_z:
                continue
            points.append((x, y))
            if len(points) >= self._max_points:
                break
        return points

    def _lookup_transform(
        self, message: PointCloud2, frame_id: str
    ) -> Optional[TransformStamped]:
        """查询点云时刻的 target<-source 变换；同一坐标系时返回 None。"""

        if frame_id == self.target_frame:
            return None
        return self._tf_buffer.lookup_transform(
            self.target_frame,
            frame_id,
            message.header.stamp,
            rospy.Duration(self._tf_timeout),
        )

    @staticmethod
    def _transform_point(
        point: Tuple[float, float, float],
        transform: Optional[TransformStamped],
    ) -> Tuple[float, float, float]:
        """用归一化四元数把一个点旋转、平移到目标坐标系。"""

        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if transform is None:
            return x, y, z

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1.0e-9:
            raise ValueError("invalid zero-length TF quaternion")
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        rotated_x = (
            (1.0 - 2.0 * (qy * qy + qz * qz)) * x
            + 2.0 * (qx * qy - qz * qw) * y
            + 2.0 * (qx * qz + qy * qw) * z
        )
        rotated_y = (
            2.0 * (qx * qy + qz * qw) * x
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
            + 2.0 * (qy * qz - qx * qw) * z
        )
        rotated_z = (
            2.0 * (qx * qz - qy * qw) * x
            + 2.0 * (qy * qz + qx * qw) * y
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * z
        )
        return (
            rotated_x + translation.x,
            rotated_y + translation.y,
            rotated_z + translation.z,
        )
