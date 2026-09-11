#!/usr/bin/env python3
"""把 Gazebo 的三维扇形点云转换为控制器使用的 PointCloud2 接口。

真实 Ranger Mini 项目通常从 Livox 驱动接收 ``sensor_msgs/PointCloud2``。
当前 Gazebo block-laser 插件输出旧式 ``sensor_msgs/PointCloud``，因此由本节点转换格式，
让上层控制程序无需区分“仿真雷达”和“真实雷达”。
"""

import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud, PointCloud2
from std_msgs.msg import Header


class ScanToLivoxCloud:
    """订阅三维 PointCloud，清理无效点后发布 XYZ32 PointCloud2。"""

    def __init__(self):
        # 话题和输出坐标系都允许在 launch 文件中覆盖，以便以后接入不同模型。
        input_topic = str(rospy.get_param("~input_topic", "/sim_livox_points"))
        cloud_topic = str(rospy.get_param("~cloud_topic", "/livox/lidar"))
        self._output_frame = str(rospy.get_param("~output_frame", "")).lstrip("/")

        # queue_size=1 表示只处理最新一帧，避免控制器使用积压的旧雷达数据。
        self._publisher = rospy.Publisher(cloud_topic, PointCloud2, queue_size=1)
        self._subscriber = rospy.Subscriber(
            input_topic, PointCloud, self._cloud_callback, queue_size=1
        )
        rospy.loginfo(
            "Gazebo fan-cloud bridge ready: %s -> %s", input_topic, cloud_topic
        )

    def _cloud_callback(self, cloud: PointCloud) -> None:
        """保留有限 XYZ 点，并把旧式 PointCloud 封装为 PointCloud2。"""

        # block-laser 已经完成三维射线投影；这里不能再把 z 压成 0。
        points = [
            (point.x, point.y, point.z)
            for point in cloud.points
            if math.isfinite(point.x)
            and math.isfinite(point.y)
            and math.isfinite(point.z)
        ]

        # 保留原始量测时间，确保控制器的数据超时判断仍然有效。
        header = Header()
        header.stamp = cloud.header.stamp
        # output_frame 为空时沿用 Gazebo PointCloud 自带的坐标系。
        header.frame_id = self._output_frame or cloud.header.frame_id.lstrip("/")
        self._publisher.publish(point_cloud2.create_cloud_xyz32(header, points))


def main() -> None:
    """启动格式转换节点并持续处理 ROS 回调。"""

    rospy.init_node("scan_to_livox_cloud")
    ScanToLivoxCloud()
    rospy.spin()


if __name__ == "__main__":
    main()
