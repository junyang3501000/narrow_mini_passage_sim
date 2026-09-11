#!/usr/bin/env python3
"""Ranger Mini 窄道控制器的 ROS1 可执行入口。

rosrun/roslaunch 只负责执行这个薄入口。参数加载、TF、点云处理、墙线拟合、
footprint 安全判断和运动控制均位于本包的 ``src/ranger_mini_narrow_sim`` 中。
"""

from ranger_mini_narrow_sim.ros_node import main


if __name__ == "__main__":
    main()
