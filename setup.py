#!/usr/bin/env python3
"""供 ROS1 catkin 使用的 Python 包安装描述。

``catkin_python_setup()`` 会读取本文件，把本目录中的
``src/ranger_mini_narrow_sim`` 加入工作区 Python 搜索路径。
控制器因此完全属于当前 ROS 包，不需要旁边再放一个算法包。
"""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["ranger_mini_narrow_sim"],
    package_dir={"": "src"},
)

setup(**setup_args)
