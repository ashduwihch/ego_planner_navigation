# 本项目启动流程（ROS 2 Jazzy）

先启动 PX4 SITL/Gazebo 和 MID-360 仿真，再按设备实际情况启动 FAST-LIO2。以下命令复用 `/home/thunder/GRA_1` 中已安装的传感器/仿真工作空间，但**规划、EGO 和控制器从当前 `ego_planner_navigation` 工作空间运行**。

## 1. PX4 SITL + Gazebo + MID-360

按现有仿真工作空间的说明启动。当前机器原有示例：

```bash
cd ~/PX4-Autopilot/launch
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/mid360_ws/install/setup.bash
cp /home/thunder/GRA_1/mid360_ws/src/mid360_gazebo_harmonic/launch/mid360_posix_sitl.launch.py .
ros2 launch ./mid360_posix_sitl.launch.py
```

只启动一次 `MicroXRCEAgent`。默认由下面本项目的 controller bringup 自动启动；若已由其他流程启动，则传 `start_agent:=false`。

## 2. FAST-LIO2

在另一个终端启动 FAST-LIO2，确保输出至少包括：

- `/fastlio2/lio_odom`
- `/fastlio2/world_cloud`

如需按原工作流运行 PGO，可使用：

```bash
cd ~/GRA_1/fastlio2_pgo_ws
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/fastlio2_pgo_ws/install/setup.bash
ros2 launch pgo pgo_launch.py rviz:=true
```

根据 FAST-LIO/PGO 的 launch 配置，确认传感器驱动和 FAST-LIO 节点已经启动；不要重复启动同一传感器节点。

## 3. 构建并启动本项目的全局路线 + EGO + PX4 控制器

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ego_planner_navigation
colcon build --symlink-install
source install/setup.bash
ros2 launch controller bringup.launch.py start_rviz:=true
```

该 bringup 使用本仓库完整移植的 `px4ctrl_ws2/src`：启动 EGO、FAST-LIO 到 PX4 odometry bridge、EGO PositionCommand 适配器、PD-MRAC 控制器及默认自动起飞程序。它还启动 `global_navigation/world_global_route.py`：RViz `/goal_pose` 会先经过静态地图安全路线规划，再发送给 EGO。**不要再启动旧的 `px4_bridge`（它已不在当前 `src` 中）。**

若 Micro XRCE-DDS Agent 已经运行：

```bash
ros2 launch controller bringup.launch.py start_rviz:=true start_agent:=false
```

## 4. 发送目标

在另一个终端 source 同一套环境后发送目标：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ego_planner_navigation/install/setup.bash
ros2 run controller manual_goal 4.0 -4.0 0.5 --auto-yaw
```

目标单位为米，使用 FAST-LIO `lidar` 局部规划坐标系。也可以在 RViz 使用 3D Goal。静态全局地图位于 `global_navigation/maps/factory_world_static.pcd`。

## 注意

- 本仓库 `src/utils/px4_msgs/FIRMWARE_VERSION` 记录了消息来源 PX4 commit。若使用不同固件，先核对/同步消息定义再构建。
- SITL 测通不代表可直接上真机。真实飞行前需单独校准坐标变换、外部视觉融合、推力参数、Offboard failsafe 和起飞保护。
