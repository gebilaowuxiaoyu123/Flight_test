# 00 · 总览：我们要做什么

## 学习目标

完成本章后，你应该能回答：

1. 这个项目由哪几层组成，每层负责什么？
2. 遥控指令、遥测、光流、深度图分别走哪条链路？
3. 为什么选 PX4 v1.16.2 + Gazebo Harmonic，而不是 Gazebo Classic？
4. 哪些地方做了“可升级”设计？

## 最终系统

```
┌─────────────────────────────────────────────────────────────┐
│ 浏览器（Web 遥控器）                                          │
│  双虚拟摇杆 / 解锁 / 模式 / Kill / 遥测 HUD                    │
└───────────────┬─────────────────────────────────────────────┘
                │ WebSocket (JSON, 50Hz)
┌───────────────▼─────────────────────────────────────────────┐
│ flight_ui（ROS 2 节点 + FastAPI）                             │
│  - 把摇杆值转成 px4_msgs/ManualControlSetpoint                │
│  - 把按钮转成 px4_msgs/VehicleCommand                         │
│  - 订阅 PX4 遥测并推给浏览器                                   │
└───────────────┬─────────────────────────────────────────────┘
                │ uXRCE-DDS (UDP 8888)
┌───────────────▼─────────────────────────────────────────────┐
│ PX4 SITL v1.16.2                                             │
│  commander / EKF2 / 控制器 / uxrce_dds_client                 │
└───────────────┬─────────────────────────────────────────────┘
                │ gz-transport
┌───────────────▼─────────────────────────────────────────────┐
│ Gazebo Harmonic                                              │
│  x500_flow：下视光流 + LW20 测距（GPS 关闭）                   │
│  x500_depth：OakD-Lite 深度相机                               │
│  x500_flow_depth：两者合并（P6）                               │
└─────────────────────────────────────────────────────────────┘
```

## 四条数据流

### 1. 控制流（人 → 飞机）

```
浏览器摇杆 → WebSocket → flight_ui
  → /fmu/in/manual_control_input  (px4_msgs/ManualControlSetpoint)
  → /fmu/in/vehicle_command       (解锁 / 模式 / Kill)
  → PX4 commander
```

注意：PX4 v1.16 的 `ManualControlSetpoint` 字段是 `roll/pitch/yaw/throttle`，范围 `[-1, 1]`，不是旧版的 `x/y/z/r`。这是升级时最容易踩的坑之一。

### 2. 遥测流（飞机 → 人）

```
PX4 → /fmu/out/vehicle_status_v1
    → /fmu/out/vehicle_local_position
    → /fmu/out/battery_status
    → flight_ui → WebSocket → 浏览器 HUD
```

### 3. 光流链路

```
Gazebo 下视相机 → PX4 gz optical_flow 插件
  → uORB vehicle_optical_flow
  → EKF2（结合 LW20 的 distance_sensor）
  → 无 GPS 的水平速度/位置估计
  → /fmu/out/vehicle_optical_flow（给 ROS 2 观察）
```

### 4. 深度链路

```
Gazebo OakD-Lite 深度相机
  → gz-transport 话题 depth_camera
  → ros_gz_bridge
  → /depth_camera/image、/depth_camera/points
  → RViz2 / depthimage_to_laserscan / 自定义障碍距离节点
```

PX4 默认不消费深度图；它属于“伴飞计算机/避障”侧的数据。

## 为什么是这套版本

| 选择 | 原因 |
|---|---|
| PX4 v1.16.2 | v1.16 起有 `gz_x500_flow` 和 Gazebo 光流插件；v1.15 仍是 EOL 的 Garden，且没有 gz 光流模型 |
| Gazebo Harmonic | LTS，支持到 2029；v1.16 的 `ubuntu.sh` 默认安装它 |
| ROS 2 Humble | 机器上已安装；与 Ubuntu 22.04 匹配 |
| uXRCE-DDS | PX4 官方 ROS 2 接口，稳定、与 PX4 版本解耦 |
| `ros_gz` 源码编译 | Humble 的二进制 `ros_gz` 是 Fortress 版，必须按 `GZ_VERSION=harmonic` 重编 |
| Web 遥控器 | 浏览器/手机都能用，UI 与 ROS 解耦，便于替换 |

## 可升级性设计

1. **版本单点管理**：所有版本号在 `versions.env`，升级只改一处。
2. **PX4 不改源码**：自定义模型/airframe 放 `px4_overlay/`，升级 PX4 后重新叠加。
3. **消息适配层**：只有 `flight_ui/px4_adapter.py` 接触 `px4_msgs`；字段变化只改一处。
4. **标准接口**：PX4 ↔ ROS 2 用 uXRCE-DDS，Gazebo ↔ ROS 2 用 `ros_gz`，不依赖内部实现。
5. **回归测试**：`scripts/verify.sh` 在每次升级后检查关键链路。
6. **兼容 Humble/Jazzy**：代码只用稳定 API；升级到 Jazzy 后 `ros_gz` 可用二进制包。

## 目录说明

| 目录 | 作用 |
|---|---|
| `versions.env` | 版本锁定 |
| `PX4-Autopilot/` | 上游 PX4，脚本克隆，git 忽略 |
| `px4_overlay/` | 自定义模型与 airframe，升级后重新叠加 |
| `src/` | ROS 2 工作区（`px4_msgs`、`ros_gz`、`flight_ui`、`flight_perception`） |
| `web/` | 前端静态资源 |
| `scripts/` | 安装、构建、运行、验证 |
| `docs/` | 本教材 |
| `PROGRESS.md` | 断点续做 |

## 术语表

| 术语 | 含义 |
|---|---|
| SITL | Software In The Loop，飞控软件在电脑上仿真 |
| uXRCE-DDS | PX4 与 ROS 2 之间的 DDS 桥 |
| uORB | PX4 内部发布/订阅消息总线 |
| `px4_msgs` | uORB 消息的 ROS 2 定义 |
| EKF2 | PX4 的状态估计器，融合 IMU/GPS/光流/视觉 |
| 光流 | 图像像素运动，用来估计角速度/水平速度 |
| 深度相机 | 输出每个像素的距离，可转点云 |
| `ros_gz` | Gazebo Transport 与 ROS 2 的桥 |
| POSCTL | 位置控制模式，摇杆控制速度 |

## 本章验收

- [ ] 能画出四条数据流，并说出每条链路的接口
- [ ] 能解释为什么不用 Gazebo Classic
- [ ] 知道升级时应该改哪个文件、跑哪个脚本
