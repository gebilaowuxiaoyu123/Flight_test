# 04 · Web 遥控器：像 RC 发射机一样操作

## 学习目标

- 理解 RC 摇杆到 PX4 手动控制消息的映射
- 理解解锁、模式切换、Kill 的指令格式
- 设计一个安全的 Web 遥控器（双虚拟摇杆 + HUD）
- 知道 P5 需要实现哪些文件、如何验收

## 摇杆布局

采用常见的 Mode 2 布局：

```
左摇杆                     右摇杆
   ↑ 油门(throttle)           ↑ 俯仰(pitch)
   │                         │
←──┼──→ 偏航(yaw)        ←──┼──→ 横滚(roll)
```

| 摇杆 | 轴 | `ManualControlSetpoint` 字段 | 范围 |
|---|---|---|---|
| 左摇杆上下 | 油门 | `throttle` | -1（最低）~ +1（最大） |
| 左摇杆左右 | 偏航 | `yaw` | -1 ~ +1 |
| 右摇杆上下 | 俯仰 | `pitch` | -1 ~ +1（前推为负？见下） |
| 右摇杆左右 | 横滚 | `roll` | -1 ~ +1 |

消息定义（PX4 v1.16）：

```text
uint64 timestamp
uint64 timestamp_sample
bool valid
uint8 data_source
float32 roll
float32 pitch
float32 yaw
float32 throttle
...
uint16 buttons
```

关键点：

- 所有轴范围 `[-1, 1]`。
- `pitch` 的符号约定：正值对应“机头向下/向前”的旋转；UI 里前推摇杆应映射成合适的符号。**第一次试飞前先在地面确认方向**。
- `valid=true` 才表示这帧有效；断线或死区时应设 `valid=false`。
- `data_source` 可设为 `SOURCE_MAVLINK_0`（数值 2），表示来自伴飞/MAVLink 侧。

## PX4 侧参数

```text
param show COM_RC_IN_MODE
```

含义：

| 值 | 行为 |
|---|---|
| 0 | 仅 RC 发射机 |
| 1 | 仅 Joystick |
| 2 | RC 与 Joystick，可回退 |
| 3 | RC 或 Joystick，先到先得（默认） |
| 4 | 禁用手动输入 |

建议显式设置：

```text
param set COM_RC_IN_MODE 1
param save
```

这样 PX4 只接受我们通过 `/fmu/in/manual_control_input` 发来的手动输入，避免和虚拟 RC 混淆。

## 解锁 / 模式 / Kill

使用 `px4_msgs/msg/VehicleCommand`：

### 解锁

```text
command = 400            # VEHICLE_CMD_COMPONENT_ARM_DISARM
param1  = 1.0            # 1=解锁，0=上锁
target_system = 1
target_component = 1
source_system = 1
source_component = 1
from_external = true
confirmation = 0
```

### 切换到 POSCTL

```text
command = 176            # VEHICLE_CMD_DO_SET_MODE
param1  = 1.0            # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
param2  = 3.0            # PX4_CUSTOM_MAIN_MODE_POSCTL
param3  = 0.0
```

常用主模式数值：

| 模式 | 数值 |
|---|---|
| MANUAL | 1 |
| ALTCTL | 2 |
| POSCTL | 3 |
| AUTO | 4 |
| ACRO | 5 |
| OFFBOARD | 6 |
| STABILIZED | 7 |

### Kill

两种方式：

1. 上锁：`ARM_DISARM`，`param1=0`
2. 飞行终止：`VEHICLE_CMD_DO_FLIGHTTERMINATION`（185），`param1=1`

UI 上建议把 Kill 做成“按住 + 二次确认”，避免误触。

## Web 架构

```
web/index.html + app.js
        │ WebSocket /ws
        ▼
flight_ui/
  ├── server.py          # FastAPI + WebSocket
  ├── manual_control.py  # 50Hz 发布 ManualControlSetpoint
  ├── commands.py        # 解锁/模式/Kill
  ├── telemetry.py       # 订阅 /fmu/out/*
  └── px4_adapter.py     # 唯一接触 px4_msgs 的地方
```

### 线程模型

- 主线程：uvicorn 事件循环
- 后台线程：`rclpy` + `MultiThreadedExecutor`
- 50Hz 定时器发布摇杆；WebSocket 每帧只更新“最新摇杆值”
- 遥测节点把 PX4 消息转成 JSON，推给所有浏览器

### WebSocket 协议

客户端 → 服务端：

```json
{"type": "sticks", "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": -0.8, "deadman": true}
{"type": "arm"}
{"type": "disarm"}
{"type": "mode", "mode": "POSCTL"}
{"type": "kill"}
```

服务端 → 客户端：

```json
{"type": "telemetry", "armed": false, "mode": "POSCTL", "alt": 0.0, "battery": 100.0}
{"type": "ack", "action": "arm", "ok": true, "message": "armed"}
```

### 安全逻辑（必须实现）

1. **Deadman**：只有按住“使能”按钮时，摇杆值才有效；松开立即回中并 `valid=false`。
2. **断线保护**：超过 300 ms 没收到 `sticks`，发布零输入且 `valid=false`。
3. **解锁条件**：油门最低 + deadman 按住 + 已切 POSCTL；否则拒绝。
4. **Kill**：随时可用，二次确认。
5. **速率限制**：摇杆变化做一阶低通，避免突跳。
6. **单客户端**：同一时间只允许一个控制端，其他连接只读遥测。

### UI 布局建议

```
┌──────────────────────────────────────────────┐
│  ARMED   POSCTL   高度 2.4m   电量 87%       │
├──────────────────────────────────────────────┤
│                                              │
│   [左摇杆]                    [右摇杆]        │
│                                              │
│   [ARM] [DISARM] [POSCTL] [KILL]             │
│   [使能/Deadman 按住]                         │
└──────────────────────────────────────────────┘
```

## P5 实现清单

- [ ] `src/flight_ui`（ament_python）包
- [ ] `px4_adapter.py`：字段映射与消息构造
- [ ] `manual_control.py`：50Hz 发布、deadman、断线保护
- [ ] `commands.py`：解锁/上锁/模式/Kill
- [ ] `telemetry.py`：状态、位置、电量
- [ ] `server.py`：FastAPI + WebSocket + `/health`
- [ ] `web/index.html`、`web/app.js`、`web/style.css`
- [ ] `launch/ui.launch.py`
- [ ] `scripts/run_ui.sh` 可启动

## 验收

在 SITL 中：

- [ ] 浏览器能连上，HUD 显示 `ARMED/DISARMED`、模式、高度
- [ ] 按住 deadman + 油门最低，点 ARM 能解锁
- [ ] 推油门能起飞，松手/松开 deadman 能停住
- [ ] 右摇杆能控制俯仰/横滚，左摇杆能控制偏航
- [ ] 切 POSCTL 后摇杆控制速度
- [ ] 关闭浏览器或拔网线，300 ms 后飞机不再接受输入
- [ ] KILL 能立即上锁/终止

## 常见问题

| 症状 | 原因 | 处理 |
|---|---|---|
| 解锁被拒 | 油门不在最低 / 没切 POSCTL / arming checks | 看 `commander status` |
| 摇杆没反应 | `COM_RC_IN_MODE` 不对 | 设 1，或确认默认 3 下没有其他输入源 |
| 方向相反 | 轴符号映射错 | 在 `px4_adapter.py` 调整，不要改 PX4 |
| 飞机抖 | 50Hz 发布不稳定 / 摇杆突跳 | 改用定时器、加低通、检查 WebSocket 延迟 |
| 断线后飞机继续飞 | 没实现失效保护 | 必须实现 300ms 超时置零 |
