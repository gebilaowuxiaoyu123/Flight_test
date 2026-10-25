# 13 · ROS 2 接入说明

## 结论

ROS 2 是整条链路的唯一通道。PX4 不懂 HTTP，浏览器不懂 DDS，靠 ROS 2 才通。

```
PX4 SITL ──uORB──> MicroXRCEAgent ──DDS──> ROS 2 ──> /flight_ui ──WS──> 浏览器
```

`/flight_ui` 是自研 C++ 后端，标准 `rclcpp` 节点：

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/fmu/out/vehicle_status_v1` | `VehicleStatus` |
| 订阅 | `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` |
| 订阅 | `/fmu/out/battery_status` | `BatteryStatus` |
| 发布 | `/fmu/in/manual_control_input` | `ManualControlSetpoint` |
| 发布 | `/fmu/in/vehicle_command` | `VehicleCommand` |

消息类型全部是 `px4_msgs`（PX4 源码编译），字段无裁剪。

## 实测数据

| 项目 | 值 |
|---|---|
| 话题总数 | 58 |
| 摇杆下发 | 50.0 Hz |
| 位置回传 | 100.1 Hz |
| 电池 / 状态 | 99.9 / 2.0 Hz |

## 深度接入点

改过 PX4 的 `src/modules/uxrce_dds_client/dds_topics.yaml`，把光流和测距暴露给 ROS 2（默认不发布）：

```diff
+  - topic: /fmu/out/vehicle_optical_flow
+  - topic: /fmu/out/distance_sensor
```

不加这两行，`ros2 topic list` 里看不到 `/fmu/out/vehicle_optical_flow` 和 `/fmu/out/distance_sensor`。

## 自证

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 node info /flight_ui                      # 节点接口
ros2 topic hz /fmu/in/manual_control_input     # 50 Hz
ros2 topic list | wc -l                        # 58
```

## 不在 ROS 里的部分

`scripts/cam_follow.py`、`scripts/pose_writer.py` 用 gz-transport 直接控制 Gazebo 相机，
因为 gz 的 protobuf 与 ROS 2 Humble 冲突。只用于拍视频，**不参与飞行控制**。

## 常见问题

**为什么不用 MAVROS？** MAVLink 要转协议，字段会丢、时序不可控。`px4_msgs` 是 uORB 1:1 映射到 DDS。

**QoS 怎么写？** 订阅 `/fmu/out/*` 用 `best_effort`，发布 `/fmu/in/*` 用 `reliable`。写反不报错、只是收不到数据。

**能多机吗？** 能。现在 `target_system = 1` 写死，多机按 `ROS_DOMAIN_ID` 或命名空间隔离。
