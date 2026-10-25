# 13 · ROS 2 接入说明（汇报用）

> 结论先行：**ROS 2 是整条链路的唯一通道，不是可选项。**
> PX4 不认识 HTTP，浏览器不认识 DDS，两者必须通过 ROS 2 通话。
> 拆掉 ROS 2，网页就控制不了飞机。

---

## 一、链路总览

```
┌──────────────┐  uORB   ┌────────────────────┐  DDS over UDP    ┌────────────────────┐
│  PX4 SITL    │ ──────► │  MicroXRCEAgent    │ ───────────────► │  ROS 2 话题空间     │
│  (飞控固件)   │ ◄────── │  (DDS 代理 :8888)   │ ◄─────────────── │  /fmu/out/*        │
└──────────────┘         └────────────────────┘                  │  /fmu/in/*         │
        ↕                                                         └─────────┬──────────┘
   Gazebo 物理仿真                                                        │
        │                                                                 │
        │ gz-transport                                                    │
        ▼                                                                 │
┌──────────────┐                                                    ┌────▼─────────────┐
│ Gazebo 相机   │ ──► /ros_gz_bridge ──► sensor_msgs/Image ─────────► │  /flight_ui      │
│ 第一视角/俯视  │                                                    │  (自研 C++ 节点)  │
└──────────────┘                                                    └────┬─────────────┘
                                                                         │ HTTP :8000
                                                                         │ WebSocket
                                                                    ┌────▼─────────────┐
                                                                    │ 浏览器 Web 遥控器  │
                                                                    └──────────────────┘
```

**角色分工**

| 组件 | 职责 | 是否 ROS 节点 |
|---|---|---|
| PX4 SITL | 飞控固件，含 EKF2 状态估计 | 否（通过 uXRCE-DDS 桥接） |
| MicroXRCEAgent | uORB ↔ DDS 协议转换 | 否（代理进程） |
| `/flight_ui` | 自研后端：接收摇杆、下发命令、拼装遥测 | **是**（`rclcpp` 节点） |
| `/ros_gz_bridge` | Gazebo 图像 ↔ ROS 2 消息 | **是**（标准 `ros_gz` 节点） |

---

## 二、实测证据（可当场复现）

### 1. 自研节点在 ROS 图里，接口就是 `px4_msgs`

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 node info /flight_ui
```

```
/flight_ui
  Subscribers:
    /fmu/out/vehicle_status_v1        px4_msgs/msg/VehicleStatus
    /fmu/out/vehicle_local_position   px4_msgs/msg/VehicleLocalPosition
    /fmu/out/battery_status           px4_msgs/msg/BatteryStatus
  Publishers:
    /fmu/in/manual_control_input      px4_msgs/msg/ManualControlSetpoint
    /fmu/in/vehicle_command           px4_msgs/msg/VehicleCommand
```

只读节点接口就能确认三件事：
- 消息类型全部是 `px4_msgs`，即 **PX4 源码编译产物**，不是手写的简化结构体
- 方向正确：订阅 `/fmu/out/*`（飞控出向）、发布 `/fmu/in/*`（飞控入向）
- 自动带上 `/parameter_events`、`/rosout` 和 6 个 `rcl_interfaces` 服务
  —— 只有真正 `rclcpp::init` + spin 的节点才会有这些

### 2. 数据真在流（实测频率）

```bash
ros2 topic hz /fmu/in/manual_control_input     # 摇杆下发
ros2 topic hz /fmu/out/vehicle_local_position  # 位置回传
```

| 话题 | 实测频率 | 含义 |
|---|---|---|
| `/fmu/in/manual_control_input` | **50.0 Hz** | 摇杆下发（设计值 50 Hz，20 ms 定时器） |
| `/fmu/out/vehicle_local_position` | **100.1 Hz** | 位置/速度回传，控制环用它做闭环 |
| `/fmu/out/battery_status` | **99.9 Hz** | 电压、剩余电量 |
| `/fmu/out/vehicle_status_v1` | **2.0 Hz** | 解锁状态、`nav_state` |

全图共 **58 个话题**在跑。

### 3. 我们对 PX4 的 DDS 桥做过改动（光流/测距接进 ROS）

`PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` 是 PX4 决定
"哪些 uORB 消息要暴露到 ROS 2" 的配置文件。我们**改过它**：

```diff
@@ publications: @@
+  - topic: /fmu/out/vehicle_optical_flow
+    type: px4_msgs::msg::VehicleOpticalFlow
+
+  - topic: /fmu/out/distance_sensor
+    type: px4_msgs::msg::DistanceSensor
```

不加这两行，光流和激光测距数据**不会出现在 ROS 2 里**（PX4 默认不发布）。
现在实测已可见：

```
/fmu/out/vehicle_optical_flow     ← 光流（EKF2 用来做无 GPS 定位）
/fmu/out/distance_sensor          ← LW20 激光测距（下视高度）
/fmu/in/distance_sensor           ← 也可外部注入
```

> 这一条很能说明接入深度：不是"把 PX4 当黑盒用"，而是改到了固件模块的配置层。

### 4. 消息定义是 PX4 原版

```bash
ros2 interface show px4_msgs/msg/VehicleCommand
```

```
# Vehicle Command uORB message. Used for commanding a mission / action / etc.
uint32 MESSAGE_VERSION = 0
uint64 timestamp
uint16 VEHICLE_CMD_NAV_WAYPOINT = 16
uint16 VEHICLE_CMD_NAV_LOITER_UNLIM = 17
...
```

`param1..4` 是 `float32`、`param5/6` 是 `float64`（装经纬度）、`param7` 是 `float32`
—— 和 PX4 的 `.msg` 定义逐字段一致，**没有任何字段被裁掉或简化**。

---

## 三、代码对应位置

`src/flight_ui_cpp/src/rc_server.cpp`（729 行，单文件后端）

| 行 | 内容 |
|---|---|
| 617 | `rclcpp::init(argc, argv)` —— 成为 ROS 节点 |
| 621 | `rclcpp::QoS(KeepLast(10)).best_effort()` —— **订阅**用 QoS |
| 631/638/644 | 三个 `create_subscription`（里程计、位置、电池） |
| 650 | `rclcpp::QoS(KeepLast(10)).reliable()` —— **发布**用 QoS |
| 651 | `create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input")` |
| 655 | `create_publisher<VehicleCommand>("/fmu/in/vehicle_command")` |
| 669 | `create_wall_timer(20ms, ...)` → **50 Hz 摇杆下发** |
| 719 | `rclcpp::spin(node)` —— 主循环交给 ROS |

### QoS 方向写反的坑（已实测确认）

| 方向 | 正确 QoS | 写反的后果 |
|---|---|---|
| `/fmu/out/*`（订阅） | `best_effort` | **不报错、只是收不到数据** |
| `/fmu/in/*`（发布） | `reliable` | 同上，静默失效 |

这是 ROS 2 ↔ PX4 集成最容易浪费时间的坑：编译通过、启动正常、无任何警告，
只是回调永远不触发。

---

## 四、诚实交代：有两块不在 ROS 里

被问到"是不是全靠 ROS"时，这两点要主动说明，避免被追问时被动。

| 组件 | 用的什么 | 为什么不用 ROS | 是否影响飞行 |
|---|---|---|---|
| `scripts/cam_follow.py`<br>`scripts/pose_writer.py` | **gz-transport**（Gazebo 原生） | gz 的 Python protobuf 绑定与 ROS 2 Humble 的 protobuf **版本冲突**，同进程 import 会崩 | ❌ 不影响，只管拍视频的相机机位 |
| 视频画面合成（PIL） | Python | 纯图像处理，与飞行无关 | ❌ 不影响 |

**控制回路 100% 走 ROS 2。** 上面两块只是"让视频好看"，摘掉照样能飞。

---

## 五、可能被追问的三个问题

**Q1：为什么用 `px4_msgs` 而不用 MAVROS？**

MAVROS 走 MAVLink，中间要经过协议转换，字段可能被裁、时序不可控。
`px4_msgs` 是 PX4 的 uORB 消息 **1:1 映射到 DDS**，`VehicleCommand.param1..7`
一个字段不少，且延迟可控（实测 50 Hz 闭环稳定）。

**Q2：QoS 怎么考虑的？**

见上面第三节。订阅 `best_effort`、发布 `reliable`，与 PX4 的 uXRCE-DDS
默认配置对齐。写反会静默失效，这个坑已经实测确认并写进文档。

**Q3：能不能扩到多机？**

能。现在 `target_system = 1` 写死，多机时按 `ROS_DOMAIN_ID` 或命名空间隔离，
代码里改一个字段；`SharedState` 已按"每个连接一个 client_id"做控制权隔离，
天然支持多客户端。

---

## 六、30 秒自证脚本

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

echo "--- ROS 2 节点 ---";                  ros2 node list
echo "--- 自研节点的 ROS 接口 ---";          ros2 node info /flight_ui
echo "--- 话题总数 ---";                    ros2 topic list | wc -l
echo "--- 50 Hz 摇杆下发 ---";              ros2 topic hz /fmu/in/manual_control_input
echo "--- 100 Hz 位置回传 ---";             ros2 topic hz /fmu/out/vehicle_local_position
echo "--- 光流已接进 ROS ---";              ros2 topic list | grep optical_flow
echo "--- 消息类型是 PX4 原版 ---";          ros2 interface show px4_msgs/msg/VehicleCommand
```
