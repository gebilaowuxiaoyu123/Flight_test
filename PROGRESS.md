# 项目进度

当前阶段：**P1–P6 已完成并跑通**（光流 + 深度 + Web 遥控器同机联调）
最后更新：2026-10-21 20:30

> 操作手册（从零安装、排错决策树、速查表）：[REBUILD.md](REBUILD.md)

## 断点续做（断线后从这里开始）

1. 打开终端，运行 `./scripts/verify.sh`；`[FAIL]` 必须为 0，`[SKIP]` 表示对应服务未启动。
2. 一键启动：`./scripts/run_demo.sh`（tmux 会话：agent/sitl/bridge/depth/flowlog/ui）。
3. 手动启动完整仿真（4 个进程，建议用 tmux）：

```bash
# 1) PX4 + Gazebo（自定义 x500_flow_depth + flow 世界）
HEADLESS=1 ./scripts/run_sitl_combined.sh

# 2) uXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 3) ROS 2 工作区
source /opt/ros/humble/setup.bash && source install/setup.bash

# 4) 深度桥 + 障碍距离 + Web UI
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
ros2 run flight_perception min_depth --ros-args -p image_topic:=/depth_camera
ros2 run flight_ui ui_server            # http://127.0.0.1:8000/
```

4. 浏览器打开 `http://127.0.0.1:8000/`，按住“按住使能”，油门在最低，点 `TAKEOFF`；`LAND` 降落，`KILL` 紧急停。
5. 若要从头安装，按 [REBUILD.md](REBUILD.md) 第三章，或 [docs/01-environment.md](docs/01-environment.md) → [docs/02-px4-sitl.md](docs/02-px4-sitl.md) 顺序做。

## 里程碑

- [x] **P0** 仓库脚手架、`versions.env`、`verify.sh`、教材、git
- [x] **P1** Gazebo Harmonic 8.15.0 + PX4 v1.16.2，`gz_x500_flow` 光流 quality 255
- [x] **P2** uXRCE-DDS + px4_msgs，`/fmu/out/*` 在线
- [x] **P3** 光流实验：无 GPS 悬停，CSV 记录 flow/quality/距离/EKF 速度
- [x] **P4** 深度相机：`x500_depth` + `ros_gz` + `min_depth` → `/obstacle_distance`
- [x] **P5** Web 遥控器：双虚拟摇杆、deadman、ARM/DISARM/POSCTL/TAKEOFF/LAND/KILL
- [x] **P6** 合并模型 `x500_flow_depth`，光流 + 深度 + 测距同机
- [ ] **P7** 升级演练（Humble→Jazzy / 新 PX4 / Docker）：见 [docs/08-upgrade.md](docs/08-upgrade.md)

## 已验收结果（2026-10-21）

| 项目 | 结果 |
|---|---|
| Gazebo | Harmonic 8.15.0，`gz sim` 正常 |
| PX4 | v1.16.2，SITL 自定义模型 `x500_flow_depth_0` |
| 光流 | `/fmu/out/vehicle_optical_flow` ~50 Hz，quality 最高 255 |
| 测距 | `/fmu/out/distance_sensor` orientation=25，随高度变化 |
| 深度 | `/depth_camera` 640×480 32FC1，约 15 Hz；`/obstacle_distance` 3.86 m（前方 4 m 墙） |
| Web UI | `:8000/health` 正常；ARM、AUTO_TAKEOFF 到 2.5 m、LAND 均验证 |
| 光流 CSV | `/tmp/optical_flow.csv`，含 flow/quality/distance/EKF 速度 |

## 关键启动参数

`scripts/run_sitl_combined.sh` 已经写入以下参数，重启后仍然生效：

```text
PX4_SIM_MODEL=gz_x500_flow_depth
PX4_SYS_AUTOSTART=4021
PX4_GZ_WORLD=flow
PX4_PARAM_NAV_DLL_ACT=0
PX4_PARAM_NAV_RCL_ACT=0
PX4_PARAM_COM_RC_IN_MODE=2
PX4_PARAM_CBRK_SUPPLY_CHK=894281
PX4_PARAM_CBRK_USB_CHK=197848
PX4_PARAM_CBRK_IO_SAFETY=22027
PX4_PARAM_EKF2_HGT_REF=0
PX4_PARAM_EKF2_BARO_CTRL=1
PX4_PARAM_EKF2_RNG_CTRL=1
PX4_PARAM_SIM_BAT_DRAIN=3600
PX4_PARAM_COM_LOW_BAT_ACT=0
PX4_PARAM_BAT_LOW_THR=0.05
PX4_PARAM_COM_DISARM_LAND=0
PX4_PARAM_COM_DISARM_PRFLT=0
```

> `COM_DISARM_LAND=0` 是为了方便演示（降落后不会 2 秒自动上锁）。
> 真实飞行应改回 `2`，并保留手动 DISARM。

## 已知限制 / 下一步

- 4021 没有 GPS/磁罗盘，yaw 会缓慢漂移；适合室内光流实验，不适合长航时。
- 光流水平定位依赖下视激光的 `distance_m`；若升级 PX4 后 `distance_m` 又变成 `NaN`，
  按 [docs/07-integration.md](docs/07-integration.md) 第 4 节检查 gz_bridge 的 orientation 判断。
- `build_ros.sh` 目前仍会构建 `src/` 下所有包，建议改为选择性构建（见脚本注释）。
- 清理：`/dev/shm/.flight_askpass` 是临时 sudo askpass，交付前删除；`/tmp/*.log` 可删。

## C++ 版本（参考实现）

| 包 | 内容 | 状态 |
|---|---|---|
| `src/flight_ui_cpp` | rclcpp + Boost.Beast + jsoncpp 的 Web 遥控器 | 编译通过，`ws_smoke_test.py` 6 PASS |
| `src/flight_perception_cpp` | C++ 版 `min_depth` / `optical_flow_logger` | 编译通过，`/obstacle_distance` 有数据 |

操作步骤与踩坑记录见 [docs/11-cpp-rewrite.md](docs/11-cpp-rewrite.md)。
这两个包**不在** `build_ros.sh` 的默认构建列表里，不影响现有 Python 链路。

```bash
# 单独编译并对比
colcon build --packages-select flight_ui_cpp flight_perception_cpp \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 run flight_ui_cpp rc_server          # 占用 8000，先停掉 python 版 ui
python3 scripts/ws_smoke_test.py          # 同一套黑盒验收
```
