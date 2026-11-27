# 无人机全局导航与 EGO 局部规划

本仓库以 `/home/thunder/GRA_1/px4ctrl_ws2/src` 为完整 ROS 2 源码基线，包含 EGO Planner、filtered-trend PD-MRAC PX4 控制器、FAST-LIO2 到 PX4 外部里程计桥，以及对应的 `px4_msgs` / `quadrotor_msgs`。仓库保留 `global_navigation/` 静态地图全局规划模块，并在控制器 bringup 中把 RViz `/goal_pose` 目标先交给全局路线节点，再转发给 EGO 局部规划。

## 运行链路

```text
RViz /goal_pose
  -> global_navigation/world_global_route.py
  -> /inspection/ego_goal + /inspection/ego_cloud
  -> EGO B-spline / PositionCommand
  -> controller/ego_position_command_adapter
  -> controller/msg/DesiredState (PX4 local NED)
  -> filtered-trend PD-MRAC
  -> PX4 attitude/thrust Offboard setpoints
```

## 构建

当前部署使用 ROS 2 Jazzy。完整源码包含来源工作空间中的 `px4_msgs`，可直接在工作空间中构建；构建前仍需核实 `src/utils/px4_msgs/FIRMWARE_VERSION` 与实际 PX4 固件一致：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/thunder/ego_planner_navigation
colcon build --symlink-install
source install/setup.bash
```

可先单独构建控制器核验 C++ 控制节点：

```bash
colcon build --packages-select controller --symlink-install
```

## 启动

先按 `launch.md` 启动 PX4/Gazebo 与 FAST-LIO2，然后在已 source 本工作空间的终端启动规划控制链：

```bash
ros2 launch controller bringup.launch.py start_rviz:=true
```

稳定起飞后，在另一终端发送目标（同样 source Jazzy、匹配 PX4 消息和本工作空间）：

```bash
ros2 run controller manual_goal 4.0 -4.0 0.5 --auto-yaw
```

全局静态地图位于 `global_navigation/maps/factory_world_static.pcd`。真实飞行前必须验证坐标变换、PX4 外部视觉融合、Offboard failsafe 和起飞保护；SITL 启动流程与参数说明见 `launch.md`、`src/controller/README.md` 和 `src/controller/参数修改指南.md`。
