# 12 · 代码怎么看、怎么跑

这份文档只回答两件事：**这个仓库里的代码按什么顺序读**，以及**每条命令在干什么**。
想了解原理（光流、EKF、PX4 桥）看 `docs/00`–`docs/11`；想从零装环境看 `REBUILD.md`。

---

## 一、代码地图

### 三个后端，同一份前端

前端只有一份（`src/flight_ui/web/`），后面可以接三种后端：

| 后端 | 语言 | 位置 | 用途 |
|---|---|---|---|
| Python 版 | FastAPI + uvicorn | `src/flight_ui/flight_ui/` | 最初跑通的版本，**用来对照读** |
| C++ 版 | rclcpp + Boost.Beast | `src/flight_ui_cpp/` | 当前主力，6 个编译单元 |
| 参考实现 | rclcpp，多文件 | 只在 git 历史 `a18ed7d` 里 | 不想看的话可以无视 |

### 主干文件

```
src/flight_ui/web/            ← 前端（370 行，先读这个）
├── app.js        200 行      双摇杆 / WebSocket / 协议
├── index.html     43 行      骨架
└── style.css     127 行      样式

src/flight_ui_cpp/            ← C++ 后端（一个文件一件事）
├── include/flight_ui_cpp/
│   ├── shared_state.hpp   187 行  跨线程状态 + 两把锁 + 控制权租约
│   ├── mode_tables.hpp        模式名 ↔ 编号（两张表）
│   ├── command_sequence.hpp   解锁/起飞/降落时序
│   ├── http_util.hpp      json/读文件/MIME
│   ├── ws_session.hpp     WebSocket 会话
│   └── http_server.hpp    HTTP 连接分发
└── src/
    ├── main.cpp          159 行  装配 + 订阅 + 定时器 + accept 线程
    ├── mode_tables.cpp    39 行
    ├── command_sequence.cpp 122 行
    ├── http_util.cpp      37 行
    ├── ws_session.cpp    205 行  主循环：读一帧 → 改状态/发命令 → 回 ack
    └── http_server.cpp    98 行

src/flight_perception_cpp/    ← C++ 感知（两个可执行，合计 193 行）
├── src/min_depth.cpp       92 行  32FC1 深度图中央 ROI 最小距离 → /obstacle_distance
└── src/optical_flow_logger.cpp 101 行  光流/测距/EKF 速度写 CSV

scripts/                      ← 运维脚本
├── run_show.sh          一条命令起录制链路（本机实测可用）
├── demo_show.py         飞行演出 + 三视角合成录制
├── cam_follow.py        跟拍/俯视相机（gz set_pose）
├── pose_writer.py       写 Gazebo 真实位姿给相机用
├── rerender_panel.py    用日志重渲染下半屏（不必重飞）
├── run_demo.sh          Python 版链路（含感知节点）
├── wait_ready.sh        SITL 就绪检测（冷启动自愈）
└── ws_smoke_test.py     端到端黑盒验收
```

---

## 二、读代码的顺序（建议严格按这个顺序）

### 第 1 步：先看协议，别看实现

打开 `src/flight_ui/web/app.js`，只看这四处：

| 行 | 看什么 |
|---|---|
| 88–98 | `sendSticks()`：每帧发什么字段 |
| 176 | `setInterval(sendSticks, 20)`：50 Hz 上行 |
| 129–135 | 按钮绑定：每个按钮发什么 `type` |
| 141–150 | `updateTelemetry()`：HUD 只读 `armed / mode / alt / battery` |

**为什么先读这个**：整条链路就是「JSON 进 → ROS 消息出」。把协议搞明白，
后端代码只是"收到这个 JSON 该干什么"的映射表。

上行（浏览器 → 后端）：

```
{"type":"sticks", "roll":.., "pitch":.., "yaw":.., "throttle":.., "deadman":..}
{"type":"arm"} / {"type":"disarm"} / {"type":"mode","mode":"POSCTL"}
{"type":"takeoff","altitude":2.5} / {"type":"land"} / {"type":"kill"}
```

下行（后端 → 浏览器）：

```
{"type":"telemetry", "armed":.., "mode":.., "nav_state":.., "alt":.., "battery":..,
                     "sticks":{...}, "controller":"you|other|none"}
{"type":"ack", "action":.., "ok":.., "message":..}
```

### 第 2 步：C++ 后端的骨架（按文件读，一个文件一件事）

| 文件 | 关键内容 | 一句话 |
|---|---|---|
| `mode_tables.cpp` | `modeNameOf` / `mainModeOf` | `nav_state`(读数) → 模式名；模式名 → **主模式号**。**两张表不是同一套编号** |
| `shared_state.hpp` | `class SharedState` | 全部跨线程数据 + 两把锁 + 控制权租约 + 遥测新鲜度 |
| `command_sequence.cpp` | `sendCommand` / `ensureManualMode` / `armWithRetry` / `takeoffSequence` / `landNow` / `killNow` | PX4 命令时序（解锁重试、起飞、降落兜底） |
| `ws_session.cpp` | `serveWebSocket` | 主循环：读一帧 JSON → 更新状态/发命令 → 回 ack |
| `http_server.cpp` | `handleConnection` | 每连接一线程：WebSocket 升级 or 静态文件 / `/health` |
| `http_util.cpp` | `jsonString` / `readFile` / `contentTypeOf` | 小工具 |
| `main.cpp` | `main` | 装配订阅/发布者/定时器，起 accept 线程 |

### 第 3 步：`SharedState` 是全场唯一要小心的地方

它的存在理由只有一个：**有 4 个线程在同时碰数据**。

| 线程 | 干什么 | 频率 |
|---|---|---|
| ROS 回调线程（`rclcpp::spin`） | 写遥测（`armed`/`nav_state`/`z`/电量） | 2–100 Hz |
| 定时器回调 | 读杆量 → 发 `ManualControlSetpoint` | 50 Hz |
| HTTP accept 线程 | 每连接 `detach` 一个线程 | 事件驱动 |
| 每连接线程 | 读 WS 帧、写 ack；另起一个 10 Hz 遥测推送线程 | 事件驱动 + 10 Hz |

所以类里有两把锁，**按数据分，不按函数分**：

- `mutex_` —— 控制链（杆量、控制权、deadman 时间戳）
- `telemetry_mutex_` —— 遥测链（armed/nav_state/z/电量）

两条链互不阻塞：50 Hz 的定时器不会被 100 Hz 的位置回调卡住。

**读的时候重点看三处**：

1. `setSticks()` 的租约判断 `if (client_ != client && !deadman) return;`
   —— 没按使能的旁观页面不能抢控制权，但 owner 的"松杆"必须生效。
2. `control()` 返回的是**一份快照**（`struct Control`），不是引用 —
   —— 出锁之后再发布，锁只保护拷贝那一瞬间。
3. `getTelemetryFor(client_id)` 里**两把锁分开取，绝不嵌套** —
   —— 只要有一处反序就会死锁。

### 第 4 步：`serveWebSocket` 的两个设计约束

```
1. 同一个 Beast 同步流只允许 1 读 + 1 写并发，禁止 2 个写者
   → 所以 ack 和遥测推送全部走 send_text() 里的 write_mu
2. 解锁要重试 10 秒，绝不能堵在读循环里
   → 所以 armSequence/takeoffSequence 都 detach 到独立线程
   → 且后台线程一个字节都不许往 ws 写（只发 ROS 命令）
```

这两条如果你不遵守，症状分别是：**连接莫名其妙断**、**整页卡死 10 秒**。

---

## 三、怎么跑

### 四终端控制台（推荐）

一个终端一条命令，日志直接打在你眼前，Ctrl-C 只停这一个。

```bash
flight link      # ① 通信连接：MicroXRCEAgent + 节点清单 + 话题频率
flight sim       # ② 仿真：PX4 + Gazebo（带 3D 窗口；HEADLESS=1 则无窗口）
flight remote    # ③ 网页遥控：rc_server 日志 + 自动开浏览器
flight views     # ④ 三视角窗口：第一人称 / 第三人称 / 俯视
```

顺序就是 ①→②→③→④。四条都起来后：

```bash
flight status    # 一行状态
flight takeoff   # 解锁并爬到 1.2 m
flight fwd 3     # 前进 3 秒
flight land      # 降落并上锁
```

首次使用先装短命令（写入 `~/.bashrc`）：

```bash
./scripts/install_aliases.sh   # 装完 unalias flight remote 2>/dev/null; source ~/.bashrc
```

### 一把梭（无人值守）

```bash
./scripts/run_show.sh              # 起全部服务，浏览器访问 http://127.0.0.1:8000/
./scripts/run_show.sh --record     # 起服务并录 180 s 成片
./scripts/run_show.sh --stop       # 停止
```

改过代码之后要先编译（**必须在工作区根目录**）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select flight_ui_cpp
source install/setup.bash
```

> 改了前端（`src/flight_ui/web/`）也要重新 build —— 是实体拷贝到
> `install/.../share/flight_ui_cpp/web/`，不是软链。

### 磁盘

PX4 SITL 会把黑匣子日志写到 `PX4-Autopilot/build/px4_sitl_default/rootfs/log/`，
一直跑就一直涨（实测一整晚 9.6 GB，曾把根分区吃到剩 3.7 G）。

```bash
flight clean           # 删 24 小时前的 .ulg
flight clean --all     # 全删
df -h /               # 看剩余
```

`run_sitl_combined.sh` 现在默认 `PX4_PARAM_SDLOG_MODE=-1`（关闭黑匣子），
要留日志就 `PX4_PARAM_SDLOG_MODE=0 flight sim`。

### 验收

```bash
./scripts/verify.sh              # 13 项环境/链路检查
python3 scripts/ws_smoke_test.py # 端到端：解锁→起飞 2 m→降落→上锁（6 个 PASS）
```

---

## 四、改代码时最容易踩的五个坑

| 坑 | 症状 | 正确做法 |
|---|---|---|
| 话题名少了 `/out/` | 编译通过、启动正常、**回调永不触发、零报错** | `/fmu/out/vehicle_status_v1`、`/fmu/in/manual_control_input` |
| 忘写 `out/` 后没加版本号 | `vehicle_status` 存在但无数据 | `VehicleStatus` 的 `MESSAGE_VERSION = 1`，话题是 `vehicle_status_v1` |
| `create_subscription` 返回值没接住 | 同上，静默失效 | 用变量接住（`auto sub = ...`） |
| QoS 方向写反 | 不报错但收不到 | `/fmu/out/*` 订阅用 `best_effort`；`/fmu/in/*` 发布用 `reliable` |
| 模式号混用两套编号 | 想切 POSCTL 结果进了 ALTCTL | 切模式用**主模式号**（POSCTL=3），HUD 显示用 `nav_state`（POSCTL=2） |

还有一个调试陷阱：**回声式 WebSocket 停发就收不到遥测**。
现在是 10 Hz 独立推送，不再有这个问题；但如果你把推送线程删了，
读到的会是缓存值，"切模式没生效"之类的假故障就会骗到你。
