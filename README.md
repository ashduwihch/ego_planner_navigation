# EGO 局部规划与全局导航

本工作空间提供 EGO 局部轨迹规划、静态点云全局导航、PX4 Offboard 桥接。全局导航是独立的 Python 脚本，静态碰撞模型为 `global_navigation/maps/factory_world_static.pcd`；EGO 本身通过 ROS 话题接收目标和点云，不直接读取 PCD。

## 三个启动文件的关系

```text
px4_bridge/launch/bringup.launch.py        整套导航的入口
  ├─ ego_planner/launch/single_uav_gazebo.launch.py  本仿真的无人机接线
  │    └─ ego_planner/launch/advanced_param.launch.py EGO 节点及算法参数
  ├─ global_navigation/world_global_route.py         全局路线与静态点云
  ├─ px4_bridge_node、FAST-LIO→PX4 里程计桥
  └─ Micro XRCE-DDS Agent、RViz（可选）
```

- `bringup.launch.py` 负责启动和连接各个进程。它从当前工作空间查找全局导航脚本，因此工作空间可以改名。
- `single_uav_gazebo.launch.py` 指定当前仿真的坐标系、FAST-LIO 里程计、EGO 障碍点云与目标话题，并启动轨迹发送节点。它把参数传给下一层。
- `advanced_param.launch.py` 创建 EGO 规划节点，设置地图分辨率、膨胀半径、速度、加速度、优化安全距离等具体算法参数。修改此处的默认值时，要检查上一层是否覆盖了该值。

当前目标链路为 `/goal_pose` → 全局导航 → `/inspection/ego_goal` → EGO；轨迹链路为 EGO → `/drone_0_planning/pos_cmd` → PX4 桥接。EGO 点云由全局导航汇合静态地图与 FAST-LIO 观测后发布到 `/inspection/ego_cloud`。仿真使用 `/clock`。

## 依赖与启动

编译 EGO 所需的内部软件包在本工作空间：`quadrotor_msgs`、`traj_utils`、`plan_env`、`path_searching`、`bspline_opt`。还需要 ROS 2 Humble、Eigen、PCL、OpenCV 等依赖。

完整仿真另外需要 Gazebo、PX4 SITL、MID-360 仿真、FAST-LIO2、与 PX4 固件匹配的 `px4_msgs`、Micro XRCE-DDS Agent。全局导航脚本使用 Python 的 NumPy、VTK 和 Gazebo Transport 绑定。具体场景的启动顺序由巡检工作空间的 `scripts/start_ego_sitl.sh` 管理。

在相应依赖已启动并已 source 本工作空间 `install/setup.bash` 后，导航入口是：

```bash
ros2 launch px4_bridge bringup.launch.py start_rviz:=true
```

只检查启动参数、不启动仿真：

```bash
ros2 launch px4_bridge bringup.launch.py --show-args
```

## 常改位置

| 内容 | 文件 |
|---|---|
| 本仿真的话题、地图尺寸、速度和避障参数覆盖值 | `src/ego/ego_planner/launch/single_uav_gazebo.launch.py` |
| EGO 节点算法参数与默认值 | `src/ego/ego_planner/launch/advanced_param.launch.py` |
| 全局路线、地图路径及机体安全距离 | `global_navigation/world_global_route.py` |
| PX4 指令话题与坐标变换 | `src/px4_bridge/config/sitl_control.yaml` |
| 全局导航、桥接器和可视化的启动组合 | `src/px4_bridge/launch/bringup.launch.py` |
