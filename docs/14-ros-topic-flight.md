# 14 · 在终端用 ROS 话题飞飞机

两条路：**单条命令**（用 `ros2 topic pub`）和 **连续杆量**（用脚本循环发 50 Hz）。

## 一、先切模式（单条命令就够）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
"{command: 176, param1: 1.0, param2: 2.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
```

- `176` = `DO_SET_MODE`，`param1 = 1` 是"启用自定义模式"，`param2 = 2` 是主模式号 ALTCTL
- 主模式号：MANUAL=1、**ALTCTL=2**、POSCTL=3、AUTO=4、ACRO=5、OFFBOARD=6、STABILIZED=7

验证：`ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort --once | grep nav_state`

## 二、解锁（必须同时有 50 Hz 摇杆帧）

PX4 要求解锁时油门在最低位，且手动控制要"有效"。开一个终端持续发：

```bash
ros2 topic pub -r 50 /fmu/in/manual_control_input px4_msgs/msg/ManualControlSetpoint \
"{valid: true, data_source: 2, roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: -1.0}"
```

另开终端发解锁：

```bash
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
"{command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
```

`arming_state` 从 1 变 2 就是解锁成功。

## 三、飞（改杆量重新发）

`ros2 topic pub -r 50` 发的是**恒定值**，想让它动就换一条命令重发：

| 动作 | 改哪个字段 |
|---|---|
| 爬升 | `throttle: 0.45` |
| 下降 | `throttle: -0.40` |
| 悬停 | `throttle: 0.0` |
| 前进 | `pitch: 0.30` |
| 后退 | `pitch: -0.30` |
| 左移 / 右移 | `roll: -0.30` / `roll: 0.30` |
| 左转 / 右转 | `yaw: 0.40` / `yaw: -0.40` |

```bash
# 例：爬升
ros2 topic pub -r 50 /fmu/in/manual_control_input px4_msgs/msg/ManualControlSetpoint \
"{valid: true, data_source: 2, roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.45}"
```

> **ALTCTL 语义**：油门杆管的是**爬升率**，不是推力；横滚/俯仰杆是**姿态角**。
> 松杆不会自动回中（`data_source: 2` 走的是 MAVLink 手动通道），飞机会保持当前姿态。

## 四、一条命令飞完整流程（推荐）

手敲 `ros2 topic pub` 只能发恒定值，飞航线需要按时间改杆量，所以写成了脚本：

```bash
python3 scripts/ros_fly_demo.py          # 切模式→解锁→爬升→前进→左移→偏航→降落
python3 scripts/ros_fly_demo.py --land   # 只降落上锁
```

实测输出：

```
1) 切 ALTCTL
  切模式     高度 0.14 m   nav_state=1   DISARMED
2) 解锁
  解锁       高度 0.14 m   nav_state=1   ARMED
3) 爬升到约 1.2 m
  爬升       高度 1.22 m   nav_state=1   ARMED
4) 前进 3 秒
  前进       高度 1.35 m   nav_state=1   ARMED
5) 左移 2 秒
  左移       高度 1.33 m   nav_state=1   ARMED
6) 左偏航 2 秒
  偏航       高度 1.36 m   nav_state=1   ARMED
7) 降落并上锁
  完成       高度 0.25 m   nav_state=1   ARMED
```

## 五、为什么不用 OFFBOARD（自主飞行的"标准答案"）

OFFBOARD + `TrajectorySetpoint` 是 PX4 的官方自主飞行接口，给目标点飞控自己闭环。
**但本机用不了**，实测：

```bash
ros2 topic echo /fmu/out/vehicle_local_position --once | grep xy_valid
# xy_valid: false
```

无 GPS、只有光流时水平位置估计无效，PX4 直接拒绝：

```
WARN [mc_pos_control] invalid setpoints
```

切模式命令发出去了，`nav_state` 仍是 1（没进 14=OFFBOARD）。

**这就是用 ALTCTL 的原因**：它的横滚/俯仰杆是姿态角指令，不依赖位置估计。

如果以后加了 GPS 或视觉里程计，把 `xy_valid` 变成 true，OFFBOARD 就能用：

```bash
# 持续发这两条（频率 >2 Hz），再切 OFFBOARD(176, param2=6)
ros2 topic pub -r 20 /fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
"{position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}"
ros2 topic pub -r 30 /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
"{position: [0.0, 0.0, -1.5], velocity: [0.0, 0.0, 0.0], yaw: 0.0}"
```

## 六、和 Web 界面的关系

两条路**走同一批 ROS 话题**，只是入口不同：

```
ros2 topic pub ─────────────┐
                            ├──> /fmu/in/manual_control_input ──> PX4
浏览器 ──WS──> rc_server ────┘
                            └──> /fmu/in/vehicle_command ──────> PX4
```

所以两边**不要同时飞**：`src/flight_ui_cpp` 会一直以 50 Hz 发杆量（默认 `throttle: -1.0`），
和你的 `ros2 topic pub` 抢同一个话题。用终端飞之前先停掉后端：

```bash
./scripts/run_show.sh --stop     # 或 kill 掉 rc_server
```
