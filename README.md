# 无人机导航与表面巡检集成层

本仓库提供 inspection_ws 使用的导航执行层：完整移植自 `/home/thunder/GRA_1/px4ctrl_ws2/src` 的 EGO Planner、filtered-trend PD-MRAC PX4 控制器、FAST-LIO2 外部视觉里程计桥和与当前 PX4 固件匹配的 ROS 消息；此外保留静态 PCD 全局路线节点。

## inspection_ws 接口链路

```text
inspection_ws 生成的安全视点（Gazebo world）
  -> 根据 /inspection/world_to_lidar 转换至 lidar
  -> /goal_pose
  -> global_navigation/world_global_route.py（静态地图检查/全局绕障）
  -> /inspection/ego_goal + /inspection/ego_cloud
  -> EGO 局部 B-spline / PositionCommand
  -> ego_position_command_adapter（PX4 local NED）
  -> PD-MRAC -> PX4 Offboard 姿态/推力
```

inspection_ws 通过 `EGO_NAVIGATION_WS` 指向本仓库，并复用 `/inspection/world_to_lidar`、`/inspection/global_route_status`、`/controller/reference_adapter_status` 和 `/px4ctrl/*` 等接口。inspection 场景 PCD 与本仓库全局导航 PCD 已核对为同一文件内容。

## 构建与单独启动

部署使用 ROS 2 Jazzy。`src/utils/px4_msgs/FIRMWARE_VERSION` 记录已核对的 PX4 消息来源；更换固件前需重新核验。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/thunder/ego_planner_navigation
colcon build --symlink-install
source install/setup.bash
ros2 launch controller bringup.launch.py start_rviz:=true
```

完整 inspection 工作流从 `/home/thunder/inspection_ws/start_sim.sh` 启动仿真、重定位和导航，再用 `/home/thunder/inspection_ws/run.sh` 生成并执行任务。其说明见 `/home/thunder/inspection_ws/README.md`。单独使用导航时，可用 `controller manual_goal` 发布 `lidar` 坐标目标。

真实飞行前必须验证坐标变换、PX4 外部视觉融合、推力参数、Offboard failsafe 和起飞保护；SITL 验证不等于真机验证。
