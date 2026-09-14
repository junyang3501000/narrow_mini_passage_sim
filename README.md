# Ranger Mini 窄道 ROS1 仿真

这是一个可以单独放进 `catkin_ws/src` 的完整 ROS1 Noetic/Gazebo Classic 功能包。
Ranger Mini 代理模型、雷达仿真、窄道控制算法、参数和测试都在
`ranger_mini_narrow_sim` 目录内部，不依赖旁边的自制功能包。完整仿真闭环为：

```text
Gazebo /sim_livox_points (PointCloud) -> /livox/lidar (PointCloud2)
Gazebo planar base -> /odom + TF
ranger_mini_narrow_sim 内置控制器 -> /cmd_vel
```

核心 Python 代码位于 `src/ranger_mini_narrow_sim`。`scripts` 中只放 ROS 可执行入口，
所以源码结构完整，同时仍可使用 `rosrun` 和 `roslaunch`。

代理模型采用公开 Ranger Mini V2 模型的约 0.498 m 轴距、0.583 m 轮距和 0.165 m 轮半径，
但底盘运动是理想平面运动，不模拟四轮转向执行器、轮胎侧偏、CAN 延迟或里程计漂移。因此
当前项目只用于验证算法与 ROS 接口的仿真闭环，不包含真车部署流程。

仿真雷达使用车头前向三维扇形视场：水平 180°（-90°~+90°）；垂直只向上扫描，
范围为 0°~+19.2°，取自经典 Livox Mid-40 38.4°视场的上半部分。扫描采用规则
361×9 射线网格，只近似可见范围，不复刻 Livox 的非重复扫描轨迹、反射强度和逐点时间。

## Git 仓库与目录结构

Git/GitHub 只管理这个 ROS 包本身。仓库根目录对应 WSL 中的：

```text
/home/junyang/ranger_mini_narrow_sim/src/ranger_mini_narrow_sim
```

因此 GitHub 首页会直接显示下面这些包文件：

```text
ranger_mini_narrow_sim/             # Git 仓库根目录，也是 ROS 包根目录
├── src/ranger_mini_narrow_sim/    # 墙线拟合、控制、里程、TF、安全和任务状态机
├── scripts/                        # 控制节点、点云桥接和仿真验收入口
├── config/                         # 完整默认参数与仿真覆盖参数
├── launch/                         # 主仿真和自动演示启动文件
├── urdf/                           # Ranger Mini 代理模型与雷达插件
├── worlds/                         # 窄道 Gazebo 世界
├── test/                           # Python 单元测试
├── setup.py                        # 把 src 下的 Python 模块注册给 catkin
├── CMakeLists.txt
└── package.xml
```

外层 `/home/junyang/ranger_mini_narrow_sim` 是本机 catkin 工作空间，不是 Git
仓库。它包含自动生成的 `build/`、`devel/`、`.catkin_workspace`、
`src/CMakeLists.txt`，以及本机可选的 `wsl/` 辅助脚本；这些内容不上传 GitHub。

完整的本机层级如下：

```text
/home/junyang/ranger_mini_narrow_sim/               # catkin 工作空间
├── build/                                           # catkin_make 自动生成，不进 Git
├── devel/                                           # catkin_make 自动生成，不进 Git
├── wsl/                                             # 本机辅助脚本，不进 Git
└── src/
    ├── CMakeLists.txt                               # catkin 自动生成，不进 Git
    └── ranger_mini_narrow_sim/                      # Git/GitHub 同步范围
```

## WSL 克隆与构建

在新环境中，从 GitHub 克隆时必须把仓库放到 catkin 工作空间的 `src` 下：

```bash
mkdir -p /home/junyang/ranger_mini_narrow_sim/src
git clone https://github.com/junyang3501000/narrow_mini_passage_sim.git \
  /home/junyang/ranger_mini_narrow_sim/src/ranger_mini_narrow_sim
```

本机已有源码时，从 catkin 工作空间根目录构建：

```bash
cd /home/junyang/ranger_mini_narrow_sim
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

该工作空间的 `src` 下只需要这一个自制 ROS 包，不需要
`simple_narrow_passage` 或其他相邻功能包。

提交、拉取和推送代码时，则进入包目录（Git 仓库根目录）：

```bash
cd /home/junyang/ranger_mini_narrow_sim/src/ranger_mini_narrow_sim
git status
git pull
git push
```

## 启动

以下命令均在已经执行 `source devel/setup.bash` 的 WSL 终端中运行。

安全启动（默认不运动）：

```bash
roslaunch ranger_mini_narrow_sim narrow_passage_sim.launch gui:=true auto_enable:=false
```

保持第一个终端运行，再打开第二个已经 source 工作区的 WSL 终端，使能或停止：

```bash
rosservice call /narrow_mode/enable "data: true"   # 开始一次 9 m 任务
rosservice call /narrow_mode/enable "data: false"  # 随时停车并退出窄道模式
```

直接自动演示：

```bash
roslaunch ranger_mini_narrow_sim narrow_passage_demo.launch gui:=true
```

无界面自动运行时使用：

```bash
roslaunch ranger_mini_narrow_sim narrow_passage_demo.launch gui:=false
```

## 端到端自检

```bash
roslaunch ranger_mini_narrow_sim narrow_passage_sim.launch \
  gui:=false auto_enable:=true run_test:=true
```

自检会从带 0.12 m 横向偏移、约 3.4° 航向偏差的初始位姿开始，等待点云和里程计，调用
使能服务，确认经历 `running`，最后在起始航向纵向投影达到 9 m 后进入 `success`。失败时会打印最后状态和
里程快照。

## 观察接口

```bash
rostopic echo /narrow_passage/status
rostopic hz /livox/lidar
rostopic echo /odom
rostopic info /cmd_vel
```

完整默认参数在 `config/controller_defaults.yaml`，仿真差异参数在
`config/simulation.yaml`。默认窄道净宽 1.20 m、速度 0.25 m/s、任务距离 9 m。

## 完整矩形 footprint 两级安全余量验证

仿真用前伸 0.42 m、后伸 0.42 m、半宽 0.35 m 构造闭合矩形，计算四个角到拟合侧墙
的最小净空。后角使用前向回波拟合出的连续墙线外推，并非后向实测。净空 0.10 m 是
预警阈值、0.05 m 是紧急阈值：

- 进入预警罩后仍可低速前进，但控制器不能继续向危险墙侧横移或摆头。
- 进入紧急罩后先立即停车，再只用低速横移远离近墙；不会自动倒车。
- 左右两侧都没有横移净空，或配置为不允许 `linear.y` 时，车辆保持停车等待人工处理。
- 完全退出预警罩并连续收到三帧安全点云后，才恢复正常前进。

可用下面的状态字段观察全过程：

```bash
rostopic echo /narrow_passage/status
# footprint_zone: clear / warning / emergency / unknown
# state: running / side_recovery / obstacle_stop / ...
```

这层逻辑在本包 `src/ranger_mini_narrow_sim` 的 Python 模块内完成，不依赖
move_base、SLAM、二维 costmap 或工作区中的其他自制包，适合当前结构固定、路线单一的
简化窄道实验。

专门测试“初始位置已进入右侧紧急罩，然后停车并向左横移恢复”：

```bash
roslaunch ranger_mini_narrow_sim footprint_recovery_test.launch gui:=false
```

该测试把车辆放在距右墙约 3 cm 的位置，除了最终 9 m 成功条件外，还强制检查运行中确实
出现过 `side_recovery`，避免车辆未触发安全罩却被误判为通过。
