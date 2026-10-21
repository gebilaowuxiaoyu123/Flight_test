# C++ 重写指南：一步一步把 Python 链路换成 C++

> 这份文档教你自己动手，把现在能用（但用 Python 写的）遥控链路换成 C++。
> 目标不是"抄一份代码"，而是**你能独立判断每一步该写什么、怎么验证、错在哪**。
>
> 参考实现放在 `src/flight_ui_cpp/`（已编译通过、已跑通相同验收），
> 但**强烈建议先自己写**，卡住 20 分钟以上再去看。想彻底不看答案：
>
> ```bash
> mv src/flight_ui_cpp /tmp/answer && colcon build --packages-select flight_ui
> ```

---

## 0. 先看懂这个仓库（read this first）

### 0.1 目录地图

```
flight_project/
├── versions.env        ← 唯一的版本来源（PX4/ROS/Gazebo 版本、并行度）
├── PROGRESS.md         ← 现在做到哪一步、怎么接着做
├── REBUILD.md          ← 从零安装 + 排错手册（操作向）
├── docs/               ← 原理教材 00~10 + 本文档 11（C++ 重写）
│
├── scripts/            ← 所有自动化：安装、编译、启动、验收
│   ├── lib.sh                公共函数（路径、source ROS）
│   ├── install_*.sh          装环境（幂等，可重复跑）
│   ├── build_px4.sh          编 PX4（会自动重放 overlay 补丁）
│   ├── build_ros.sh          编 ROS（只编需要的包）
│   ├── run_sitl_*.sh         起 PX4+Gazebo（flow / depth / combined）
│   ├── run_demo.sh           一键起全链路（含自愈）
│   ├── wait_ready.sh         判断"真起来了"还是"假启动"
│   ├── verify.sh             13 项环境+链路检查
│   └── ws_smoke_test.py      端到端飞行验收（黑盒测试，语言无关）
│
├── src/                ← 我们的代码（其余都是被 .gitignore 排除的上游源码）
│   ├── flight_ui/            【Python】Web 遥控器：FastAPI+WebSocket+rclpy
│   ├── flight_ui_cpp/        【C++ 参考实现】rclcpp+Boost.Beast
│   ├── flight_perception/    【Python】min_depth、optical_flow_logger
│   ├── px4_msgs/             上游（gitignore，脚本克隆）
│   └── ros_gz/               上游（gitignore，脚本克隆）
│
├── px4_overlay/        ← 我们对 PX4 的"叠加层"（升级时可重放）
│   ├── models/x500_flow_depth/model.sdf   光流+激光+深度相机合并模型
│   ├── worlds/flow.sdf                    棋盘格地面 + 4 m 墙
│   └── px4_patches/                       必须重放的 PX4 补丁
│
├── PX4-Autopilot/      ← 上游（gitignore）
├── Micro-XRCE-DDS-Agent/ ← 上游（gitignore）
├── build/ install/ log/  ← colcon 产物（gitignore）
└── web/README.md       ← 早期占位，内容在 src/flight_ui/web/
```

### 0.2 三个入口，从哪个开始看

| 你想干什么 | 从这里进 | 花多久 |
|---|---|---|
| **先把它跑起来** | [REBUILD.md](../REBUILD.md) 第一章 → `./scripts/run_demo.sh` | 5 分钟 |
| **搞懂原理** | [docs/README.md](README.md) → 00 → 01 → … 按章节顺序 | 每章 30~60 分钟 |
| **改代码/换语言** | 本文档 → 先读 `src/flight_ui/`（Python 版当规格书） | 3~6 小时 |

### 0.3 建议的源码阅读顺序（改语言前必读）

1. `src/flight_ui/web/app.js`（178 行）
   —— 前端只做三件事：**每 20 ms 发一次摇杆 JSON**、**收遥测渲染 HUD**、**按钮发命令**。
   先读它，你就知道后端的**协议**长什么样，其它都是实现细节。
2. `src/flight_ui/flight_ui/server.py`（292 行）
   —— WebSocket 服务 + 命令分发。**这是你要重写的核心**。
3. `src/flight_ui/flight_ui/px4_adapter.py`（172 行）
   —— 唯一和 `px4_msgs` 打交道的文件。**换语言时字段名照抄这里**。
4. `src/flight_ui/flight_ui/telemetry.py`（99 行）
   —— 订阅遥测、聚合成一个快照给 HUD。
5. `src/flight_perception/flight_perception/*.py`
   —— 两个小节点（深度图取最小值、光流写 CSV），C++ 化最简单，适合练手。

### 0.4 每个文件"一句话职责"

| 文件 | 职责 | 换语言时的风险 |
|---|---|---|
| `web/app.js` | 协议客户端（不用改） | 无 |
| `server.py` | WS 服务 + 50 Hz 定时发布 + 命令分发 | **线程模型、session 生命周期** |
| `px4_adapter.py` | 消息字段映射 + PX4 时间基准 | **字段名、时间戳基准** |
| `telemetry.py` | 遥测聚合 | QoS、订阅保活 |
| `min_depth.py` | 32FC1 深度图 → 最小距离 | 指针/步长算术 |
| `optical_flow_logger.py` | 光流 → CSV | 无 |

---

## 1. 把 Python 版当成"规格说明书"

重写不是翻译代码，是**复刻行为**。下面这些行为必须逐条对上，否则"能连上但飞不起来"：

| # | 行为 | Python 位置 | 违反了会怎样 |
|---|---|---|---|
| 1 | 摇杆 50 Hz 持续发布 | `server.py::RosWorker._tick` | PX4 300 ms 收不到就判失效，飞机停住 |
| 2 | 300 ms 没收到前端帧 → 发零输入 + `valid=false` | `_tick` 里的 `stale` | 浏览器崩了飞机还在飞 |
| 3 | `valid = valid and deadman` | `px4_adapter.py::publish_manual` | 没按使能也能控制 → 危险 |
| 4 | 时间戳用 **PX4 开机时间**，且**减 100 ms** | `now_us()` | PX4 丢弃输入，摇杆完全没反应 |
| 5 | 发布 QoS = **reliable**，订阅 QoS = **best_effort** | `QoSProfile(...)` | 收不到遥测 / PX4 收不到指令 |
| 6 | 同时订阅 `vehicle_status_v1` 和 `vehicle_status` | `telemetry.py` | 跨 PX4 版本看不到 ARMED |
| 7 | 解锁前先切到非 GPS 模式，并**重试 10 s** | `_ensure_manual_mode` / `_arm_with_retry` | 4021 无 GPS，解锁必被拒 |
| 8 | `NAV_TAKEOFF.param7` 传 **AMSL**（`ref_alt - z + 相对高度`） | `takeoff()` | 冲高或不动 |
| 9 | arm → 等 armed → 切 POSCTL → 延时 → TAKEOFF | `_takeoff_sequence` | PX4 只认第一条命令 |
| 10 | 同一时刻只允许一个控制端（0.6 s 租约） | `set_sticks` 的 owner | 两个标签页互抢摇杆 |
| 11 | `/health` 返回 `{status, ros, telemetry}` | `server.py` | 验收脚本 `verify.sh` 失败 |

### 1.1 协议（前后端约定，必须一字不差）

**前端 → 后端**

```json
{"type":"sticks","roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":-1.0,"deadman":true}
{"type":"arm"}
{"type":"disarm"}
{"type":"mode","mode":"POSCTL"}
{"type":"takeoff","altitude":2.5}
{"type":"land"}
{"type":"kill"}
```

**后端 → 前端**

```json
{"type":"telemetry","armed":false,"mode":"POSCTL","nav_state":2,"alt":0.02,
 "vx":0,"vy":0,"vz":0,"battery":99.9,"voltage":16.2,
 "xy_valid":true,"z_valid":true,"landed":true}
{"type":"ack","action":"takeoff","ok":true,"message":"takeoff to 2.5 m requested"}
```

> 只要协议不变，`web/` 目录一行都不用改。这是重写时最大的优势：
> **前端就是你的验收测试器**。

### 1.2 模块映射

| Python | C++ | 说明 |
|---|---|---|
| `RosWorker`（含 `_tick`、命令方法） | `RcService` | 摇杆状态 + 50 Hz tick + 命令序列 |
| `Px4Adapter` | `Px4Adapter` | 消息构造，字段名只出现在这里 |
| `Telemetry` | `Telemetry` | 订阅 + 加锁快照 |
| FastAPI + uvicorn | `WebServer`（Boost.Beast） | HTTP + WebSocket |
| `rclpy.executors` 线程 | `rclcpp::spin` + `std::thread` | 线程模型要自己设计 |
| `websockets` 库 | Boost.Beast `websocket::stream` | 需要自己处理升级/读/写 |

---

## 2. 技术选型（以及为什么这么选）

| 需求 | 选择 | 为什么 |
|---|---|---|
| ROS 2 客户端 | **rclcpp** | 官方 C++ 客户端，`px4_msgs` 直接可用 |
| HTTP + WebSocket | **Boost.Beast**（header-only） | 系统里已有 Boost 1.74，**零安装**；Beast 同时支持 HTTP 与 WS，且是同步 API，逻辑最直观 |
| JSON | **jsoncpp**（`libjsoncpp-dev`） | 系统已装、有 CMake config；消息很小，够用 |
| 构建 | **ament_cmake** | C++ 包的标准；`ament_python` 只能装 Python 文件 |

不选其它的理由（这些坑我都试过或评估过）：

- **Crow / uWebSockets / cpp-httplib**：要么需要 FetchContent 联网下载，要么没有 WebSocket，
  要么 API 不透明。Beast 虽然啰嗦，但**完全可控、无线程隐藏行为**，适合边写边学。
- **rosbridge_suite**：它自己就是 Python，等于没换语言。
- **rclcpp + 第三方 WS 库（libwebsockets）**：libwebsockets 的回调模型和 rclcpp 的执行器模型
  叠在一起很难 debug，第一次写不建议。

先确认依赖都在（本机已验证，无需 sudo）：

```bash
ls /usr/include/boost/beast/core.hpp          # Beast
ls /usr/include/jsoncpp/json/reader.h         # jsoncpp
ls /opt/ros/humble/include/rclcpp             # rclcpp
ls install/px4_msgs/include/px4_msgs          # px4_msgs（由 ./scripts/build_ros.sh 产生）
```

如果某天真缺了：

```bash
sudo apt install libboost-all-dev libjsoncpp-dev
```

---

## 3. 第 1 步：建包（30 分钟）

### 3.1 建目录

```bash
mkdir -p src/flight_ui_cpp/{include/flight_ui_cpp,src,launch}
```

为什么用 `include/` 分出来：`ament_cmake` 的惯例是"对外暴露的头文件放 include，
只在本包用的放 src"。后面加测试、被别的包依赖时都按这个路径找。

### 3.2 `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>flight_ui_cpp</name>
  <version>0.1.0</version>
  <description>C++ 版 Web 遥控器</description>
  <maintainer email="wiki_wu@example.com">wiki_wu</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>px4_msgs</depend>
  <depend>ament_index_cpp</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

> `<maintainer email>` **必须是合法邮箱**，否则 `colcon build` 直接报错（我踩过）。

### 3.3 `CMakeLists.txt`（能编过的最小版）

```cmake
cmake_minimum_required(VERSION 3.8)
project(flight_ui_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(jsoncpp REQUIRED)

add_executable(rc_server src/rc_server_main.cpp)
ament_target_dependencies(rc_server rclcpp px4_msgs ament_index_cpp)
target_include_directories(rc_server SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(rc_server ${Boost_LIBRARIES} JsonCpp::JsonCpp)

install(TARGETS rc_server DESTINATION lib/${PROJECT_NAME})
ament_package()
```

### 3.4 坑：`target_link_libraries` 签名冲突（我实际撞到的）

第一版我写的是：

```cmake
target_link_libraries(rc_server PRIVATE flight_ui_cpp_core)   # keyword 签名
target_link_libraries(rc_server PRIVATE Boost::system)        # keyword
ament_target_dependencies(rc_server rclcpp px4_msgs)          # 内部用 plain 签名
```

报错：

```
The keyword signature for target_link_libraries has already been used with
the target "rc_server".  All uses of target_link_libraries with a target
must be either all-keyword or all-plain.
```

**原因**：`ament_target_dependencies()` 内部调用的是 **plain** 签名
（`target_link_libraries(tgt ${libs})`），CMake 不允许同一个 target 混用两种签名。

**两条出路**（任选其一，别混）：

```cmake
# A) 全部用 plain（推荐，和 ament 生态一致）
ament_target_dependencies(rc_server rclcpp px4_msgs)
target_link_libraries(rc_server ${Boost_LIBRARIES} JsonCpp::JsonCpp)

# B) 全部用 keyword（要手写所有 ROS 库路径，很啰嗦）
```

### 3.5 验收

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select flight_ui_cpp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 pkg prefix flight_ui_cpp     # 打印 install/flight_ui_cpp
```

> 一定加 `--packages-select`：全量 `colcon build` 会把 `src/ros_gz` 的 demo 包也编一遍。

---

## 4. 第 2 步：最小节点（10 分钟）

先只让它"活着"，别急着写业务。

`src/rc_server_main.cpp`：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("flight_ui");
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1000),
    [&node]() { RCLCPP_INFO(node->get_logger(), "tick"); });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

验收：

```bash
ros2 run flight_ui_cpp rc_server      # 每秒打印一次 tick
ros2 node list                        # 另一个终端：/flight_ui
```

### 4.1 理解 `rclcpp::spin` 的线程模型

这是后面所有线程问题的根源，必须先想清楚：

```
主线程 ── rclcpp::spin(node) ── 阻塞，处理订阅回调 + 定时器回调（单线程执行器）
                                 ↑
Web 线程 ── Beast accept/read ────┘ 直接调用 service_.setSticks() 等（跨线程！）
命令线程 ── std::thread 跑起飞序列 ── 调 adapter_.arm() 等（跨线程！）
```

对照 Python 版：`rclpy` 用 `MultiThreadedExecutor` + 一个 spin 线程，逻辑一样。
C++ 里"跨线程"是**真的跨线程**（Python 有 GIL 兜底），所以：

- 共享状态（摇杆、遥测）**必须加锁**；
- 状态用 `std::atomic` 防重入（`sequence_running_`）；
- rclcpp 的 `publish()` 本身线程安全，可以跨线程调。

---

## 5. 第 3 步：订阅遥测（先把"看得见"打通）

### 5.1 QoS 不能随便写

PX4 的 uXRCE 输出是 **best-effort**，输入要 **reliable**：

```cpp
const auto qos_out = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();  // 订阅
const auto qos_in  = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();     // 发布
```

写反了的现象**很迷惑人**：

| 写错的 QoS | 现象 |
|---|---|
| 订阅写成 reliable，PX4 发 best_effort | 完全不兼容，收不到数据（不会有报错） |
| 发布写成 best_effort，PX4 订阅 reliable | PX4 收不到指令，摇杆没反应 |

用 `ros2 topic info -v` 看两端的 QoS 是否兼容：

```bash
ros2 topic info /fmu/out/vehicle_status_v1 -v | grep -E 'Node name|Reliability'
```

### 5.2 坑：订阅对象必须保活（我实际撞到的第 2 个）

第一版我这样写：

```cpp
node->create_subscription<px4_msgs::msg::VehicleStatus>(
  "/fmu/out/vehicle_status_v1", qos,
  [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) { onStatus(msg); });
```

编译过、能启动、`ros2 topic info` 里甚至**看得见这个订阅者**，但**回调永远不触发**。
`/health` 里 `mode` 一直是 `UNKNOWN`、`nav_state` 一直是 `-1`。

**原因**：`create_subscription` 返回 `SharedPtr`，你没接住 → 语句结束就析构
→ 订阅被撤销。rclcpp **不会**替你保活。

**Python 版没这个坑**：rclpy 的 `Node` 内部持有订阅引用。

**正确写法**（存成成员）：

```cpp
// telemetry.hpp
std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

// telemetry.cpp
subs_.push_back(node->create_subscription<px4_msgs::msg::VehicleStatus>(
  "/fmu/out/vehicle_status_v1", qos,
  [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) { onStatus(msg); }));
```

### 5.3 话题名带版本后缀

`VehicleStatus` 在 PX4 v1.16 的 `MESSAGE_VERSION = 1`，实际话题是
`/fmu/out/vehicle_status_v1`。稳妥做法是**两个都订阅**，跨版本都能跑。

### 5.4 验收

```bash
# 加一行日志最直观
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
  "armed=%d nav_state=%d alt=%.2f", t.armed, t.nav_state, t.alt);
```

```bash
ros2 topic hz /fmu/out/vehicle_status_v1        # 应约 2 Hz
ros2 run flight_ui_cpp rc_server                # 日志里的值会跟着变
```

**这一步必须过了再往下**：遥测看不见就去写控制，等于闭着眼开车。

---

## 6. 第 4 步：发布手动控制（能不能飞起来的关键）

### 6.1 字段表（照抄 `src/px4_msgs/msg/ManualControlSetpoint.msg`）

| 字段 | 取值 | 说明 |
|---|---|---|
| `timestamp` | PX4 开机时间 µs | **不是** ROS 时间 |
| `timestamp_sample` | 同上 | |
| `valid` | `deadman && !stale` | 总开关 |
| `data_source` | `SOURCE_MAVLINK_0` (=2) | 配合 `COM_RC_IN_MODE` |
| `roll/pitch/yaw/throttle` | `[-1,1]` | throttle `-1` = 最低 |
| `sticks_moving` | 任一轴 > 0.01 | PX4 用来判断"人还在操作" |
| `buttons` | 0 | |

### 6.2 时间戳：整个项目最容易卡住的地方

PX4 校验 `manual_control_setpoint` 的时间戳用的是**它自己的开机时间**。
如果你用 ROS 墙钟（约 `1.79e15 µs`），PX4 认为这是"来自未来"的样本，**直接丢弃**。
现象：**摇杆推了完全没反应，但没有任何报错**。

正确做法（`Px4Adapter::nowUs()`）：

```
订阅 /fmu/out/timesync_status  → 拿到 PX4 当前时间 px4_time_us
记下"这一帧是本地哪个 monotonic 时刻收到的" → last_recv_
之后每次需要时间戳：
    px4_time_us + (now_monotonic - last_recv_) - 100ms
```

那个 **-100 ms** 是我调试出来的：PX4 的选择器会拒绝
`timestamp_sample` 比它自己时钟新的样本，留一点余量更稳。

```bash
# 想验证时间戳是否合理：PX4 侧和 ROS 侧对比
ros2 topic echo /fmu/out/timesync_status --once --qos-reliability best_effort
```

### 6.3 50 Hz 定时器 + 失效保护

```cpp
void RcService::tick()        // 20 ms 一次
{
  Sticks sticks;
  bool deadman = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sticks = sticks_;
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_update_).count();
    deadman = sticks_.deadman && age <= 0.3;   // 300 ms 没帧 → 失效
  }
  if (!deadman) {
    adapter_.publishManual(0, 0, 0, -1.0, /*valid=*/false, /*deadman=*/false);
    return;
  }
  adapter_.publishManual(sticks.roll, sticks.pitch, sticks.yaw,
                         sticks.throttle, true, true);
}
```

> 为什么失效时还要**持续发**零输入？因为 PX4 的超时判据是"多久没收到样本"，
> 停止发布 ≠ 主动停止飞机；持续发 `valid=false` 才是明确的"我放手了"。

### 6.4 验收

```bash
# 终端 A：看 PX4 实际收到的值
ros2 topic echo /fmu/out/manual_control_setpoint --qos-reliability best_effort

# 终端 B：跑服务，用前端推摇杆
ros2 run flight_ui_cpp rc_server
```

看到 `valid: true`、`throttle` 跟着变、`data_source: 2` 就对了。
如果 `valid` 一直是 false → 检查 deadman 与 stale 判断。
如果 `throttle` 一直是 `-1.0` → 检查是不是 300 ms 内没收到前端帧（前端每 20 ms 发一次）。

---

## 7. 第 5 步：HTTP 服务（先不用任何框架）

先用**裸 socket + Beast 同步调用**把 HTTP 打通，你才能看清 WebSocket 干了什么。

### 7.1 线程模型（推荐，简单且不阻塞 ROS）

```
run() 线程： accept() 循环
              └─ 每来一个连接 → 起一个 detached 线程处理该连接
                   ├─ http::read → 普通请求：serveHttp() 然后关闭
                   └─ 是升级请求 → 交给 WsSession
```

为什么不用"一个线程 + 异步"?  异步（`async_*` + `io_context`）性能好但**状态机
要自己拆成续体**，第一次写极易写成回调地狱。连接数只有个位数时，**线程-每连接**
是最容易 debug 的选择。

### 7.2 accept 循环

```cpp
listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));  // 否则重启要等端口
// bind + listen ...
while (running_) {
  const int fd = ::accept(listen_fd_, nullptr, nullptr);
  if (fd < 0) { if (!running_) break; continue; }
  std::thread([this, fd]() { handleConnection(fd, web_root_, service_); }).detach();
}
```

`stop()` 里 `::shutdown(listen_fd_, SHUT_RDWR)` 让阻塞的 `accept` 立刻返回 —— 
否则 Ctrl-C 后进程退不干净，"端口已被占用"。

### 7.3 把裸 fd 变成 Beast 的 stream

```cpp
net::io_context ioc(1);
tcp::socket socket(ioc);
socket.assign(tcp::v4(), fd, ec);       // 接管已有 fd
beast::tcp_stream stream(std::move(socket));
```

### 7.4 路由（和 FastAPI 版一一对应）

| 路径 | 行为 |
|---|---|
| `GET /` | 返回 `web/index.html` |
| `GET /static/<f>` | 返回 `web_root/<f>`（含 `..` 直接 400） |
| `GET /health` | `{"status":"ok","ros":true,"telemetry":{...}}` |

```cpp
http::response<http::string_body> res{http::status::ok, req.version()};
res.set(http::field::content_type, contentType(relative));
res.body() = std::move(body);
res.prepare_payload();                  // 自动算 Content-Length
http::write(stream, res, ec);
stream.socket().shutdown(tcp::socket::shutdown_send, ec);
```

**忘了 `prepare_payload()`** 的症状：浏览器/curl 一直等到超时（没有 Content-Length）。

### 7.5 静态资源从哪来

不要复制一份前端。`CMakeLists.txt` 里直接装另一份源码：

```cmake
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../flight_ui/web/
  DESTINATION share/${PROJECT_NAME}/web
  FILES_MATCHING PATTERN "*.html" PATTERN "*.js" PATTERN "*.css")
```

运行时用 ament 索引找：

```cpp
web_root = ament_index_cpp::get_package_share_directory("flight_ui_cpp") + "/web";
```

### 7.6 验收

```bash
curl -s http://127.0.0.1:8000/health
curl -o /dev/null -w '%{http_code} %{size_download}\n' http://127.0.0.1:8000/static/app.js
# 期望：200 5839
```

> 还想验证 404、`..` 越权、keep-alive 行为？用 `curl -i` 和 `curl --path-as-is` 试。

---

## 8. 第 6 步：WebSocket（重写的重点）

### 8.1 升级握手

Beast 里只要一行判断 + 一行接受：

```cpp
if (websocket::is_upgrade(req)) {
  auto session = std::make_shared<WsSession>(std::move(stream), std::move(req), service);
  session->run();
}
// WsSession 构造里：
ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
ws_.accept(req);      // 发 101 Switching Protocols
ws_.text(true);       // 收发文本帧
```

### 8.2 读线程 + 写线程（Beast 的并发约束）

Beast 同步流的规矩：**一个流上可以同时有"1 个读"和"1 个写"，但不能两个读或两个写。**

所以 `WsSession::run()` 的结构是：

```
当前线程   ── for(;;) ws_.read()  → handleMessage()    （读）
新线程     ── while(alive) ws_.write(telemetry)         （写，10 Hz）
退出       ── alive=false; writer.join(); releaseClient()
```

```cpp
void WsSession::run()
{
  std::atomic<bool> alive{true};
  std::thread writer([this, &alive]() { writeLoop(alive); });

  beast::flat_buffer buffer;
  for (;;) {
    beast::error_code ec;
    ws_.read(buffer, ec);
    if (ec) break;                       // 客户端断开是常态
    handleMessage(beast::buffers_to_string(buffer.data()));
    buffer.consume(buffer.size());        // 不清空会让下一帧和上一帧粘在一起
  }
  alive = false;
  writer.join();
  service_.releaseClient(id_);
}
```

三个必踩的坑：

1. **`buffer.consume()` 忘了写**：第一次消息正常，之后每次收到拼接的脏 JSON。
2. **写线程没 join 就析构 session** → 野指针崩溃。所以 `run()` 退出前一定 `join()`。
3. **写失败后读还阻塞**：写失败说明 socket 已坏，读也会立刻报错，所以不会死等；
   但反过来（客户端半关闭）要依赖 timeout 选项，所以 `set_option(timeout::suggested(...))` 不能省。

### 8.3 session 生命周期用 `shared_ptr`

```cpp
class WsSession : public std::enable_shared_from_this<WsSession> { ... };
auto session = std::make_shared<WsSession>(...);   // 由 run() 栈上的 shared_ptr 保活
session->run();
```

`run()` 返回后 `session` 析构，连接关闭。**不要**用裸 `new` 或栈对象：
写线程捕获了 `this`，对象提前析构就是 use-after-free。

### 8.4 JSON 处理（jsoncpp）

```cpp
// 解析
Json::CharReaderBuilder builder;
std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
Json::Value root; std::string errors;
if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors)) return;

// 取值（给默认值，避免字段缺失时抛异常）
sticks.throttle = root.get("throttle", -1.0).asDouble();

// 序列化（indentation 写成空串，省流量）
Json::StreamWriterBuilder wb; wb["indentation"] = "";
const std::string frame = Json::writeString(wb, root);
```

> 坑：`jsoncpp` 的 `CharReader` 是**不完整类型**，必须额外
> `#include <json/reader.h>`（只 include `json/writer.h` 编译不过）。我撞过。

### 8.5 验收

```bash
# 用 Python 当临时客户端（测试工具用 Python 无所谓，被测的是 C++）
python3 - <<'PY'
import asyncio, json, websockets
async def main():
    async with websockets.connect('ws://127.0.0.1:8000/ws') as ws:
        print(json.loads(await ws.recv()))          # 应看到 telemetry
        await ws.send(json.dumps({'type':'sticks','roll':0,'pitch':0,
                                  'yaw':0,'throttle':-1,'deadman':True}))
        print(json.loads(await ws.recv()))
asyncio.run(main())
PY
```

再打开浏览器 `http://127.0.0.1:8000/`：HUD 应显示 ARMED/模式/高度，摇杆能拖动。

---

## 9. 第 7 步：命令状态机（最容易写错的 200 行）

### 9.1 为什么不能"一次发三条命令"

PX4 的 commander 每轮只处理一个模式/命令迁移。你在**同一个周期**里发
`ARM` + `DO_SET_MODE` + `NAV_TAKEOFF`，它只认第一条，后面全丢。
现象：`TAKEOFF` 的 ack 是 OK，但飞机不动。

正确顺序（每步之间要有时间间隔，且要**等状态变化**再继续）：

```
等 deadman + 油门最低
  → 若当前 nav_state 属于 {MANUAL, ALTCTL, POSCTL, ACRO, STABILIZED} 之外，先切 POSCTL（等 0.8 s）
  → arm(true)，最多重试 10 s，每轮等 1 s 看 armed
  → 等 0.3 s
  → setMode(POSCTL)
  → 等 0.8 s
  → NAV_TAKEOFF(param7 = ref_alt - z + 相对高度)
```

### 9.2 为什么要"重试 arm"

冷启动后 EKF 还没收敛、气压计还在预热，PX4 会回 `Arming denied: Resolve system health failures first`。
用户点一次按钮就放弃是不对的——**重试 10 秒**能覆盖绝大多数情况。

### 9.3 为什么 param7 要换算

`NAV_TAKEOFF.param7` 是 **AMSL 绝对高度**，不是离地高度。
4021 机架的高度基准来自气压计，`ref_alt` 可能接近 0、也可能是别的值。

```cpp
double target_alt = altitude_m;
if (reference_alt_valid_) {
  target_alt = reference_alt_ - z_ + altitude_m;   // AMSL
}
```

`ref_alt` 在 4021 上可能是 `NaN`，所以必须先 `std::isfinite()` 判断再存。

### 9.4 线程与竞态

命令序列要跑几秒，**绝不能**在 WebSocket 读线程里 sleep（会把 HUD 卡住）：

```cpp
if (sequence_running_.exchange(true)) {          // 防止连点两次
  return {false, "another command is running"};
}
std::thread([this, altitude_m]() {
  takeoffSequence(altitude_m);
  sequence_running_ = false;
}).detach();
```

`sequence_running_` 用 `std::atomic<bool>`：多个连接线程可能同时点按钮。

### 9.5 单控制器租约（多标签页的坑）

`RosWorker` 的摇杆状态是**全局一份**。两个标签页都开着，谁也不让谁，
摇杆值在两边来回抖 —— 我在测试时就遇到"明明推了油门却没动"。

```cpp
bool RcService::setSticks(const Sticks & sticks, uint64_t client_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  if (owner_ != 0 && owner_ != client_id) {
    const auto age = std::chrono::duration<double>(now - last_update_).count();
    if (age < 0.6) return false;      // 别人 0.6 s 内还在发帧 → 忽略我
  }
  owner_ = client_id;
  sticks_ = sticks;
  last_update_ = now;
  return true;
}
```

`client_id` 用 `session` 的自增编号，断开时 `releaseClient(id)` 归还。

### 9.6 验收（黑盒，别自己点）

```bash
python3 scripts/ws_smoke_test.py --altitude 2.0
```

它做的事：连 WS → 按住 deadman → TAKEOFF → 等飞到 2 m → LAND →
等 `landed:true` → POSCTL → DISARM。6 个 `[PASS]` + `exit=0` 才算过。

> 这个测试脚本**不认识实现语言**，所以它是 Python 版和 C++ 版的共同验收标准。
> 先把 Python 版跑绿，再换成 C++ 跑同一个脚本 —— 这就是"行为等价"的证明。

---

## 10. 第 8 步：把感知节点也换成 C++（练手推荐）

`min_depth` 和 `optical_flow_logger` 很小，适合在写遥控器之前先熟悉 rclcpp。

### 10.1 `min_depth`：32FC1 深度图 → 最小距离

不需要 OpenCV。32FC1 就是 `float32` 数组，直接指针算术最快：

```cpp
if (msg->encoding != "32FC1") return;            // 只处理深度图
const size_t row_len = msg->step / sizeof(float); // step 是字节数！
const float * data = reinterpret_cast<const float *>(msg->data.data());
const size_t w = msg->width, h = msg->height;
if (row_len < w) return;

const size_t rw = w * 0.5, rh = h * 0.6;          // 中央 50%×60% 区域
const size_t x0 = (w - rw) / 2, y0 = (h - rh) / 2;
float best = std::numeric_limits<float>::max();
for (size_t v = y0; v < y0 + rh; ++v) {
  for (size_t u = x0; u < x0 + rw; ++u) {
    const float d = data[v * row_len + u];        // 用 row_len 而不是 w！
    if (std::isfinite(d) && d > 0.01F && d < max_range_) best = std::min(best, d);
  }
}
if (best < std::numeric_limits<float>::max()) { /* publish Float32 */ }
```

三个坑：

1. **`step` 是字节数**，不是像素数。32FC1 要除以 4。
2. **按 `row_len` 索引，不是按 `width`**：行尾可能有 padding。
3. 深度图里有 `0`、`inf`、`nan`，必须过滤，否则最小值永远是 0。

### 10.2 `optical_flow_logger`：CSV

只要注意两件事：

- **同时订阅两个话题**，在回调里把"最近一帧 EKF 速度"和"当前光流"配对；
- 用 `std::ofstream` 打开一次，别每条都开关文件。

```cpp
flow_sub_ = create_subscription<px4_msgs::msg::VehicleOpticalFlow>(
  "/fmu/out/vehicle_optical_flow", qos,
  [this](px4_msgs::msg::VehicleOpticalFlow::SharedPtr msg) { onFlow(msg); });
```

行的格式要和 Python 版一致，否则你之前的实验数据没法对比：

```
t, flow_x, flow_y, dt, distance, quality, ekf_vx, ekf_vy, flow_vx, flow_vy
```

其中 `flow_vx = pixel_flow[0] / dt * distance`（`v ≈ ω·h`）。

### 10.3 验收

```bash
ros2 run flight_perception_cpp min_depth
ros2 topic echo /obstacle_distance --once      # 应看到 3~4 m（前方有堵墙）

ros2 run flight_perception_cpp optical_flow_logger
column -s, -t /tmp/optical_flow_cpp.csv | tail -5
```

---

## 11. C++ 专属调试手册

### 11.1 只编一个包，看完整错误

```bash
colcon build --packages-select flight_ui_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug --event-handlers console_direct+
```

`console_direct+` 让编译输出直接打到终端（不再写进 `log/latest_build`），
这样 `error:` 的上下文（模板实例化链条）才完整。

### 11.2 "回调不触发"三步排查

```bash
# 1) 话题在不在
ros2 topic list | grep vehicle_status

# 2) 有没有数据（注意 QoS）
ros2 topic echo /fmu/out/vehicle_status_v1 --once --qos-reliability best_effort

# 3) 我的订阅者在不在一端？QoS 兼容吗？
ros2 topic info /fmu/out/vehicle_status_v1 -v
```

三步都正常但回调还是不来 → **十有八九是订阅对象没保活**（第 5.2 节）。

### 11.3 段错误 / 崩溃

```bash
# 打开 core dump
ulimit -c unlimited
gdb -ex run -ex bt --args \
  install/flight_ui_cpp/lib/flight_ui_cpp/rc_server
```

写 WebSocket/线程代码时，优先用 **ASan** 找 use-after-free：

```bash
colcon build --packages-select flight_ui_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
               -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
               -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
```

数据竞争用 **TSan**（和 ASan 不能同时开）：

```bash
-DCMAKE_CXX_FLAGS="-fsanitize=thread" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread"
```

> 我在写"两个标签页抢摇杆"那个 bug 时，就是 TSan 先报的 `data race on sticks_`，
> 才想到要加租约的。

### 11.4 日志别刷屏

```cpp
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "alt=%.2f", alt);  // 1 s 一条
```

50 Hz 的回调里直接 `RCLCPP_INFO` 会把终端和磁盘都打死（这也是当初 VS Code OOM 的同类问题）。

---

## 12. 验收清单（逐条打勾才算"打通"）

| # | 检查 | 命令 | 期望 |
|---|---|---|---|
| 1 | 包能编 | `colcon build --packages-select flight_ui_cpp` | `Finished` |
| 2 | 节点能起 | `ros2 run flight_ui_cpp rc_server` | 打印 Web RC 已启动 |
| 3 | HTTP 通 | `curl -s localhost:8000/health` | `{"ros":true,...}` |
| 4 | 遥测活 | 同上，看 `telemetry.mode` | 不是 `UNKNOWN` |
| 5 | 静态资源 | `curl -o /dev/null -w '%{http_code}' localhost:8000/static/app.js` | `200` |
| 6 | WS 通 | 浏览器打开首页 | HUD 显示 ARMED/模式/高度 |
| 7 | 摇杆通 | `ros2 topic echo /fmu/out/manual_control_setpoint --qos-reliability best_effort` | 推油门时 `throttle` 变 |
| 8 | 飞行通 | `python3 scripts/ws_smoke_test.py` | 6 个 PASS，exit 0 |
| 9 | 与 Python 版等价 | 用同一个 `ws_smoke_test.py` 各跑一次 | 都是 exit 0 |

---

## 13. 练习（按难度递增）

1. **加一个 `RTL` 按钮**：前端加按钮 → 后端 `{"type":"mode","mode":"AUTO.RTL"}`。
   提示：`DO_SET_MODE` 的 `param3`（sub mode）要处理。
2. **HUD 显示光流质量**：订阅 `/fmu/out/vehicle_optical_flow`，把 `quality`
   加进 telemetry JSON 和前端。
3. **把 telemetry 从轮询改成推送去重**：值没变就不发，省带宽。注意别让前端收不到心跳。
4. **多机支持**：两个 session 分别控制两架 PX4（`target_system` 1/2）。
   提示：把 `RcService` 变成"每机一个实例"。
5. **把 WebServer 改成异步（`async_accept` + `io_context`）**：
   对比线程-每连接，测 100 个连接时的 CPU 占用。

---

## 附录 A：参考实现说明

| 项 | 值 |
|---|---|
| 位置 | `src/flight_ui_cpp/`（独立 ament_cmake 包，不影响 Python 版） |
| 规模 | 约 1100 行（含头文件），对比 Python 版约 740 行 |
| 依赖 | rclcpp、px4_msgs、ament_index_cpp、Boost.Beast、jsoncpp（**全部系统已装**） |
| 状态 | 已编译通过；`ws_smoke_test.py` 6 个 PASS / exit 0 |
| 与 Python 版差异 | 无协议差异；`build_ros.sh` 的 `PACKAGES` 里**不包含**它，所以默认不会被编 |

想完全自己写、不看答案：

```bash
mv src/flight_ui_cpp /tmp/answer
colcon build --packages-select flight_ui_cpp 2>&1 | head -3   # 应该报 "not found"
# ……自己写……
diff -ru /tmp/answer src/flight_ui_cpp | less                # 最后对照
```

## 附录 B：建议的时间安排

| 时间 | 做什么 | 产出 |
|---|---|---|
| 0.5 h | 读 §0~§1，跑 `./scripts/run_demo.sh` 并操作网页 | 知道协议和模块边界 |
| 0.5 h | §3~§4 建包 + 最小节点 | 能 `ros2 node list` 看到它 |
| 1 h | §5 订阅遥测 | `/health` 里数值会变 |
| 1 h | §6 发布会话（timers/QoS/时间戳） | `manual_control_setpoint` 数值跟着摇杆变 |
| 1 h | §7 HTTP | `curl` 三个路由都对 |
| 2 h | §8 WebSocket | 浏览器 HUD 活了 |
| 2 h | §9 命令状态机 | `ws_smoke_test.py` 绿 |
| 1 h | §10 感知节点 | `/obstacle_distance` 有数 |
| 1 h | §11 调试 + §12 验收 | 全部打勾 |

总工时约 **10 小时**（我第一次写用了两晚，其中一半时间在 §5.2 和 §9.1 这两个坑上）。
