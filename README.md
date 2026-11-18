# 无人机全局导航与 EGO 局部规划

## 功能

- 基于静态 PCD 地图规划全局路线，飞行受阻时重新规划。
- 使用 EGO Planner 和实时点云进行局部避障。
- 通过 PX4 Offboard 桥接执行轨迹，支持 RViz 三维选点。

## 使用方法

环境：ROS 2 Humble、PX4 SITL、FAST-LIO2、MID-360、与 PX4 固件匹配的 `px4_msgs`。先启动仿真和传感器，再在本工作空间执行：

```bash
source /opt/ros/humble/setup.bash
source /path/to/px4_msgs_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch px4_bridge bringup.launch.py
```

在 RViz 使用 **3D Goal** 选点，或在另一终端发送目标（坐标单位：米）：

```bash
ros2 run px4_bridge manual_goal X Y Z --auto-yaw
```

全局导航地图位于 `global_navigation/maps/factory_world_static.pcd`；更换场景时需替换地图并调整对应坐标配置。
