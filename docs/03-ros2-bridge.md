# 03 · ROS 2 桥：uXRCE-DDS + px4_msgs

## 学习目标

- 理解 PX4 与 ROS 2 之间的数据通路
- 启动 Micro-XRCE-DDS-Agent 和 PX4 客户端
- 用 `ros2 topic` 看到 `/fmu/in/*` 与 `/fmu/out/*`
- 知道版本不匹配时会发生什么

## 架构

```
PX4 SITL
  uxrce_dds_client  ──UDP 8888──►  MicroXRCEAgent  ──DDS──►  ROS 2 节点
       ▲                                                        │
       └──────────────── /fmu/in/*、/fmu/out/* ◄────────────────┘
```

- **Agent** 跑在电脑上，负责 XRCE-DDS ↔ DDS 转换。
- **Client** 是 PX4 里的 `uxrce_dds_client` 模块。
- 话题命名规则：`/fmu/in/<uorb_topic>`（ROS → PX4）和 `/fmu/out/<uorb_topic>`（PX4 → ROS）。

## 前置检查

```bash
./scripts/verify.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix px4_msgs
```

`px4_msgs` 必须是 `release/1.16`，与 PX4 v1.16.2 匹配。

## 1. 启动 Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

保持这个终端不关。

## 2. 启动 PX4 与客户端

另开终端：

```bash
./scripts/run_sitl_flow.sh
```

在 PX4 `pxh>` 里：

```text
uxrce_dds_client start -t udp -p 8888
```

如果提示已经在运行，可以：

```text
uxrce_dds_client status
```

## 3. 看话题

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list | grep fmu
```

常见输出：

```text
/fmu/in/offboard_control_mode
/fmu/in/manual_control_input
/fmu/in/vehicle_command
/fmu/out/battery_status
/fmu/out/distance_sensor
/fmu/out/vehicle_local_position
/fmu/out/vehicle_optical_flow
/fmu/out/vehicle_status
```

### 看消息类型

```bash
ros2 topic info /fmu/out/vehicle_optical_flow
ros2 topic info /fmu/in/manual_control_input
```

期望：

```text
Type: px4_msgs/msg/VehicleOpticalFlow
Type: px4_msgs/msg/ManualControlSetpoint
```

> 注意：v1.16 用的是 `ManualControlSetpoint`（字段 `roll/pitch/yaw/throttle`），不是旧版的 `ManualControl`（`x/y/z/r`）。

### 读一条消息

```bash
ros2 topic echo /fmu/out/vehicle_status --once
ros2 topic echo /fmu/out/vehicle_optical_flow --once
ros2 topic echo /fmu/out/distance_sensor --once
```

如果 `echo` 没数据，试：

```bash
ros2 topic echo /fmu/out/vehicle_status --once --qos-reliability best_effort
```

### 看频率

```bash
ros2 topic hz /fmu/out/vehicle_optical_flow
```

光流正常约 50 Hz；遥测约 2–10 Hz（取决于消息）。

## 4. 发一条命令（可选）

用 `ros2 topic pub` 测试解锁：

```bash
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "
timestamp: 0
command: 400
param1: 1.0
target_system: 1
target_component: 1
source_system: 1
source_component: 1
from_external: true
"
```

> 实际使用建议通过 `flight_ui` 节点发送，便于加时间戳、确认和安全逻辑。

## 常见问题

| 症状 | 原因 | 处理 |
|---|---|---|
| `ros2 topic list` 没有 `/fmu/*` | Agent 未启动 / PX4 客户端未启动 | 检查两个终端；`uxrce_dds_client status` |
| 话题有但没数据 | QoS 不匹配 | 加 `--qos-reliability best_effort` |
| 消息字段对不上 | `px4_msgs` 分支与 PX4 版本不匹配 | 切到 `release/1.16`，重新 `colcon build` |
| Agent 报端口占用 | 8888 被占用 | `ss -lunp | grep 8888`，换端口并同步 PX4 参数 |
| PX4 终端没有 `uxrce_dds_client` | 编译时未包含 | 确认 `make px4_sitl gz_x500_flow` 成功；检查 `build/px4_sitl_default` |

## 本章验收

- [ ] `MicroXRCEAgent udp4 -p 8888` 正常运行
- [ ] `uxrce_dds_client start -t udp -p 8888` 成功
- [ ] `ros2 topic list | grep fmu` 有输出
- [ ] `ros2 topic echo /fmu/out/vehicle_optical_flow --once` 有数据
- [ ] `ros2 topic info /fmu/in/manual_control_input` 显示 `ManualControlSetpoint`
