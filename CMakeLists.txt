# 这是一个自包含的纯 Python ROS1 包：既包含 Gazebo 仿真，也包含窄道控制算法。
# 没有 C++ 目标，但 src/ranger_mini_narrow_sim 下的 Python 模块仍需由 catkin 安装。
cmake_minimum_required(VERSION 3.0.2)
project(ranger_mini_narrow_sim)

# 这里只声明 ROS/Gazebo 系统组件，不依赖工作区中的其他自制功能包。
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

# 读取本包自己的 setup.py，将 src/ranger_mini_narrow_sim 注册到 Python 路径。
catkin_python_setup()

# 把运行依赖传递给安装后的包；本包没有自定义消息、C++ 库或头文件。
catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    geometry_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
)

# 三个入口分别负责控制、点云桥接和端到端验收；核心逻辑均位于本包 src 目录。
catkin_install_python(PROGRAMS
  scripts/ranger_narrow_controller_node.py
  scripts/scan_to_livox_cloud.py
  scripts/sim_smoke_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装仿真参数、启动图、车辆描述和 Gazebo 世界。
install(DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 单元测试覆盖内嵌控制器；不需要启动 ROS Master 或 Gazebo。
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_controller.py)
  catkin_add_nosetests(test/test_distance_tracker.py)
  catkin_add_nosetests(test/test_node_config.py)
  catkin_add_nosetests(test/test_safety_supervisor.py)
  catkin_add_nosetests(test/test_wall_estimator.py)
endif()
