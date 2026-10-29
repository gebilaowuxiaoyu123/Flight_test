# 15 · 怎么读 C++ 源码、怎么调试

这一章解决三件事：

1. **按什么顺序读** —— 从哪下手，避免一上来被 `main.cpp` 劝退
2. **每个函数、每个参数是什么** —— 带参数签名的完整清单
3. **怎么调试** —— gdb 断点、实时日志、话题观察、纯函数单测，四条路都实测过

适用范围：`src/flight_ui_cpp/`（9 个文件 1246 行）和 `src/flight_perception_cpp/`（2 个文件 193 行）。
`src/ros_gz/`、`src/px4_msgs/` 是第三方，不在这章范围内。

---

## 0. 先修一个前置条件：调试符号

没有 `-g` 的话，gdb 能启动但**看不到源码行、看不到局部变量**，等于不可调试。
本章写作时项目正处于这种状态（`readelf` 里 `debug_info` 段数为 0），已修正：

```cmake
# src/flight_ui_cpp/CMakeLists.txt
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -g)
```

验证方式：

```bash
readelf -S build/flight_ui_cpp/rc_server | grep -c debug_info   # 应 ≥ 1
```

> 要压测再显式传 `-DCMAKE_BUILD_TYPE=Release`；`-g` 会一直保留，等同 RelWithDebInfo。
> 本包是控制后端，`-O0` 相对 DDS 的开销可以忽略。

---

## 1. 三遍读法

| 遍 | 目标 | 怎么读 | 耗时 |
|---|---|---|---|
| ① 数据流 | 知道"谁把什么给了谁" | 只跟 4 条线，**忽略所有细节** | ~1 h |
| ② 逐函数 | 每个函数、每个参数都懂 | 对着第 3 节清单，一次一个文件 | ~3 h |
| ③ 动手调试 | 能改、能验证 | gdb 断点 + 实时日志 + 话题观察 | 边读边做 |

### 第①遍：只跟 4 条线

先不要读代码，在纸上画这 4 条：

```
① 摇杆线（连续量，50 Hz）
   浏览器摇杆 → ws_session.cpp 收 JSON → SharedState::setSticks()
              → 50 Hz 定时器取快照 → /fmu/in/manual_control_input

② 命令线（一次性，事件触发）
   浏览器按钮 → ws_session.cpp → command_sequence.cpp（内部 sleep 重试）
              → /fmu/in/vehicle_command （arm / 切模式 / 起飞 / 降落）

③ 遥测线（PX4 → 页面）
   /fmu/out/* → main.cpp 的 4 个订阅回调 → SharedState::setXxx()
              → getTelemetryFor() 打包 JSON → WebSocket 推回浏览器

④ 静态文件线
   浏览器 GET / → http_server.cpp 读 HTTP 头 → http_util.cpp 读文件 + 定 MIME
```

**过关标准**：能不看代码回答"网页上点起飞，消息经过哪几个文件、最后落到哪个话题"。
答案：`ws_session.cpp` → `command_sequence.cpp::takeoffSequence` → `/fmu/in/vehicle_command`。

---

## 2. 文件全景与依赖方向

```
                    浏览器 (JS)
                        │  HTTP / WebSocket
        ┌───────────────┴───────────────┐
        │ http_server.cpp (104)         │  每连接一个线程，先读 HTTP 头
        │   ↓ 升级 WS                    │
        │ ws_session.cpp (212)          │  收 JSON 摇杆/命令，回遥测 + ack
        └───────┬───────────────┬───────┘
                │               │
   command_sequence.cpp (133)   │  mode_tables.cpp (43)
   解锁/模式/起飞/降落时序        │  nav_state ↔ 模式名（两套编号）
                │               │
        ┌───────┴───────────────┴───────┐
        │ shared_state.hpp (218)        │  唯一共享数据 + 两把锁
        └───────────────┬───────────────┘
                        │
     main.cpp (216) ────┴──→ /fmu/in/*   (50 Hz 杆量)
     装配 + 订阅 + 定时器  ←── /fmu/out/*   (遥测回调)

  基础件：config.cpp (37) 读 YAML 全局配置 · control_lease.cpp (92) 控制权租约
         http_util.cpp (41) JSON/读文件/MIME
```

依赖是**单向**的：`main → 各模块 → shared_state`，没有循环。
`http_util` / `mode_tables` / `config` 是叶子节点（无内部依赖），最好改。

---

## 3. 逐函数清单（带参数）

按**依赖顺序**读，从叶子往上，不要从 `main.cpp` 开始。

### 3.1 第一层：纯函数

#### `src/flight_ui_cpp/src/mode_tables.cpp`（43 行）

```cpp
std::string modeNameOf(int nav_state);       // 18 个显式分支 → 名字（只读回显用）
int mainModeOf(const std::string & name);    // 名字 → 主模式号 1~7（发命令用）
```

- `nav_state`：PX4 `vehicle_status.nav_state` 原值，如 `POSCTL=2`、`AUTO_TAKEOFF=17`、`AUTO_LAND=18`
- 未列出的取值落到 `default` → `STATE_<n>`，所以不会返回空串
- ⚠️ **这两个不是反函数**：`modeNameOf` 有 18 个显式分支，`mainModeOf` 只输出主模式号。两套编号不能互套。

#### `src/flight_ui_cpp/src/http_util.cpp`（41 行）

```cpp
std::string jsonString(const Json::Value & value);                 // Json 值 → 字符串
bool readFile(const std::string & path, std::string & content);    // content 是出参
std::string contentTypeOf(const std::string & path);               // 后缀 → MIME
```

- `readFile` 的第二个参数是**引用出参**：返回 `bool` 只表示成败，内容在 `content` 里
- 只用标准库 + jsoncpp，**不碰 ROS**，所以可以直接写小 main 单测（见 4.5）

#### `src/flight_ui_cpp/src/config.cpp`（37 行）

```cpp
void loadConfig(rclcpp::Node & node);       // 从 ROS 参数（config/flight_ui.yaml）读
const FlightUiConfig & config();            // 全局单例，返回 const 引用
```

- `FlightUiConfig` 字段：`port`、`takeoff_altitude_m`、`arm_retry_sec`、`control_rate_hz`、`telemetry_rate_hz`
- 为什么用全局：HTTP 线程、WS 会话、定时器都要读，逐层传参不划算

### 3.2 第二层：状态与契约

#### `src/flight_ui_cpp/include/flight_ui_cpp/shared_state.hpp`（218 行）★ 全项目最重要

```cpp
// ---- 写：WebSocket 线程 ----
void setSticks(uint64_t client, bool deadman,
               double roll, double pitch, double yaw, double throttle);
void attachClient(uint64_t client);         // 网页连上 → any_client 变 true
void releaseClient(uint64_t client);        // 断开 → 归还控制权

// ---- 读：50 Hz 定时器 ----
struct Control {
    bool valid;          // 真按着使能（deadman）且 300 ms 内有更新
    bool any_client;     // 是否还有网页连着
    double roll, pitch, yaw, throttle;
};
Control control() const;                    // 返回一份加锁快照

// ---- 写：PX4 遥测回调线程 ----
void setStatus(bool armed, int nav_state);                  // vehicle_status
void setPosition(bool z_valid, double z, double ref_alt);    // vehicle_local_position
void setObstacle(double meters);                            // /obstacle_distance
void setBattery(float voltage_v, float remaining);           // battery_status

// ---- 派生查询 ----
double takeoffAltitudeAmsl(double relative_m) const;   // 相对高度 → AMSL（加 ref_alt）
double statusAgeSec() const;                           // 遥测断流了多久
Json::Value getTelemetry() const;                      // 打包成 WS 下发格式
Json::Value getTelemetryFor(uint64_t client) const;     // 同上，带连接相关字段
bool armed() const;
int navState() const;
double obstacleDistance() const;
```

**读这个文件只问三个问题**：

1. 哪些字段有锁？`mutex_` 管控制/连接，`telemetry_mutex_` 管遥测，**两把锁不嵌套**（避免死锁）
2. getter 返回的是快照还是引用？**全是值/快照** —— 这是它能安全跨线程的原因
3. 哪些是"派生"的？`valid`、`any_client` 是算出来的，不是存下来的

#### `src/flight_ui_cpp/src/control_lease.cpp`（92 行）

```cpp
long long nowMs();                                   // 匿名 namespace 内部用
bool pidAlive(int pid);
void writeEntry(const std::string & owner, int prio, int pid);
Entry read();
bool acquire(const std::string & me, int prio);      // 抢/续租
void release(const std::string & me);                // 只删自己名下的
```

- 文件：`/tmp/flight_control.lease`，内容 `<owner> <prio> <pid> <unix_ms>`
- 优先级：`kPrioWeb = 10` > `kPrioTerminal = 5`，数值大的赢
- `acquire` 拿得到租约的条件：空 / 过期 / 持有者进程已死 / 自己是持有者（续租）/ 对方 prio 更低
- 写文件用 `tmp + rename` 原子替换，读端不会读到半截

### 3.3 第三层：业务时序

#### `src/flight_ui_cpp/src/command_sequence.cpp`（133 行）

```cpp
void sendCommand(const CommandPublisher & pub,
                 uint64_t timestamp_us,
                 uint16_t command,
                 float p1 = 0.0F, float p2 = 0.0F, float p3 = 0.0F, float p4 = 0.0F,
                 double p5 = 0.0, double p6 = 0.0, float p7 = 0.0F);

uint64_t nowUs(rclcpp::Node * node);                          // 当前时间（微秒）
bool isManualNavState(int nav_state);                         // 判断是否手动模式
void sleepMs(int ms);                                         // 阻塞睡

void ensureManualMode(SharedState & state, rclcpp::Node * node,
                      const CommandPublisher & cmd_pub);
bool armWithRetry(SharedState & state, rclcpp::Node * node,
                  const CommandPublisher & cmd_pub);
void armSequence(SharedState & state, rclcpp::Node * node,
                 const CommandPublisher & cmd_pub);
void takeoffSequence(SharedState & state, rclcpp::Node * node,
                     const CommandPublisher & cmd_pub, double altitude_m);
void landNow(rclcpp::Node * node, const CommandPublisher & cmd_pub);
void killNow(rclcpp::Node * node, const CommandPublisher & cmd_pub);
```

**重点理解两件事**：

1. `p1~p7` 是 PX4 `VehicleCommand` 的 7 个通用参数位，每个命令含义不同。本包实际用到的五个：

   | 命令号 | 名称 | 参数含义 | 本项目调用 |
   |---|---|---|---|
   | `176` | `DO_SET_MODE` | **p1 = 1**（base mode = 自定义模式），**p2 = 主模式号** | `sendCommand(pub, nowUs(node), 176, 1.0F, 3.0F)` → p2=3 = **POSCTL** |
   | `400` | `ARM_DISARM` | p1 = 1 解锁 / 0 上锁 | `400, 1.0F` 解锁；`400, 0.0F` 上锁 |
   | `22` | `NAV_TAKEOFF` | **p7 = 目标高度（AMSL）** | `22, 0,0,0,0, 0,0, takeoffAltitudeAmsl(alt)` |
   | `21` | `NAV_LAND` | 无参数 | `sendCommand(pub, nowUs(node), 21)` |
   | `185` | `FLIGHT_TERMINATION` | p1 = 1 请求终止（kill 用） | `killNow` 里紧接着上锁发一条 |

   ⚠️ **主模式号在 p2 而不是 p1** —— 这个最容易记错。
   主模式号取值来自 `mainModeOf`，与 PX4 的 `PX4_CUSTOM_MAIN_MODE` 枚举一致：
   `MANUAL=1`、`ALTCTL=2`、`POSCTL=3`、`AUTO=4`、`ACRO=5`、`OFFBOARD=6`、`STABILIZED=7`。
   强制上锁（p2 = 21196）本包**没有**用，只在 `scripts/fly.sh` 落地兜底时用到。
2. `sleepMs` 意味着**这些函数是阻塞的**，它们跑在 WebSocket 连接线程里

### 3.4 第四层：IO 外壳

#### `src/flight_ui_cpp/src/http_server.cpp`（104 行）

```cpp
void handleConnection(int fd, SharedState & state, const std::string & web_root,
                      rclcpp::Node * node, const CommandPublisher & cmd_pub);
```

每个 TCP 连接一个 `std::thread`。流程：读 HTTP 头 → 是 `Upgrade: websocket` 就交给 `serveWebSocket`，否则当静态文件回。

#### `src/flight_ui_cpp/src/ws_session.cpp`（212 行，最大，最后读）

```cpp
void serveWebSocket(beast::tcp_stream stream,
                    http::request<http::string_body> req,
                    SharedState & state,
                    rclcpp::Node * node,
                    const CommandPublisher & cmd_pub,
                    uint64_t client_id);
```

**按 JSON 消息类型切块读**，一块一块看：

| `type` 字段 | 行号 | 作用 |
|---|---|---|
| `sticks` | 107 | 更新摇杆，**`deadman` 是这条消息里的一个字段**（不是独立消息） |
| `disarm` | 114 | `sendCommand(..., 400, 0.0F)` 上锁 |
| `mode` | 125 | `mainModeOf(name)` → `DO_SET_MODE`；返回 0 则报"未知模式" |
| `arm` / `takeoff` | 147 | **两个 type 共用一个分支**，靠 `want_takeoff` 区分是否接着起飞 |
| `land` | 183 | `sendCommand(..., 21)` |
| `kill` | 191 | 上锁 + `FLIGHT_TERMINATION` |

回给前端的 `ack` 不是客户端发来的消息，是服务端下发的结果回执（前端靠 `type=="ack"` 在右下角打印）。

### 3.5 第五层：装配

#### `src/flight_ui_cpp/src/main.cpp`（216 行，最后读，当目录用）

```cpp
int main(int argc, char ** argv);
```

只有一个函数，但内部有两个 lambda：

| 位置 | 频率 | 作用 |
|---|---|---|
| `main.cpp:99` | 1 Hz | 打一行遥测日志（先取快照，别在锁里读成员） |
| `main.cpp:116` | **50 Hz** | **全项目的心脏**：租约协商 → 四通道映射 → publish |

50 Hz 定时器分三段，建议按这个顺序读：

1. `main.cpp:121-145` 租约协商（0.5 s 节流、边沿日志、抢不到就 `return`）
2. `main.cpp:147-156` 四通道映射与兜底（`has_pilot` vs `c.valid` 的区别见 5.1）
3. `main.cpp:158-171` 填 PX4 消息字段并 `publish`

### 3.6 感知包

#### `src/flight_perception_cpp/src/min_depth.cpp`（92 行）

```cpp
class MinDepth : public rclcpp::Node;       // 32FC1 深度图 → 中央 ROI 最小距离
int main(int argc, char ** argv);
```

- 订阅 `/depth_camera`（`sensor_msgs/msg/Image`，编码 `32FC1`）
- 发布 `/obstacle_distance`（`std_msgs/msg/Float32`）
- **不用 OpenCV**：`32FC1` 就是 float32 数组，按 `step` 逐行索引最快
- ROI：宽 50%、高 60% 居中；近端盲区 0.2 m 导致恒显示 0.20 m

#### `src/flight_perception_cpp/src/optical_flow_logger.cpp`（101 行）

```cpp
class OpticalFlowLogger : public rclcpp::Node;
int main(int argc, char ** argv);
```

- 订阅 `vehicle_optical_flow` + `vehicle_local_position`
- 写 CSV 到 `/tmp`，用来验证 `v ≈ ω·h`
- 和 `flight_ui_cpp` **零耦合**，靠话题通信，可单独起停

---

## 4. 调试手册（四种手段，都实测过）

### 4.1 gdb 下断点

```bash
cd /home/wiki_wu/flight_project
source /opt/ros/humble/setup.bash && source install/setup.bash
gdb -q --args build/flight_ui_cpp/rc_server

(gdb) break main.cpp:121        # 50 Hz 定时器里租约判断那行
(gdb) run
(gdb) info locals               # 看 c / armed / state / manual_pub
(gdb) bt 3                      # 看是谁调用了这个 lambda
```

真实输出（可对照）：

```
Thread 1 hit Breakpoint 1, operator() (__closure=...) at main.cpp:121
c = {valid = false, any_client = false, roll = 0, pitch = 0, yaw = 0, throttle = -1}
armed = false
state = @0x7fffffffc7a0: {..., client_ = 0, attached_ = std::set with 0 elements,
                           deadman_ = false, armed_ = false, nav_state_ = -1, ...}
manual_pub = std::shared_ptr<...Publisher<ManualControlSetpoint_...>> (use count 1)
#0  operator() at main.cpp:121
#1  rclcpp::GenericTimer<main(...)::<lambda()>, 0>::execute_callback_delegate<>(void) at timer.hpp:244
```

> ⚠️ 断点停在第 121 行时，`msg`、`roll`、`pitch` 显示的是**垃圾值** —— 因为赋值语句还没执行。
> 看到"乱值"先确认断点位置，别急着怀疑代码。

**够用 90% 的命令**：

```
b 文件:行号     break 函数名      next / step      finish
p 变量          info locals       info args        bt / frame N
watch 变量      continue          thread apply all bt
```

### 4.2 attach 会被拦（本机实测）

```
ptrace: 对设备不适当的 ioctl 操作
... check /proc/sys/kernel/yama/ptrace_scope
```

本机 `ptrace_scope = 1`，只能调试**子进程**。两个选择：

```bash
gdb -q --args <程序>                          # ✅ 推荐：gdb 自己启动
sudo sysctl -w kernel.yama.ptrace_scope=0     # 或临时放开（重启失效）
```

### 4.3 看实时日志

`rc_server` 的 stdout 在 `/tmp/rc_full.log`（由 `fly.sh` 的 `setsid ... > /tmp/rc_full.log` 决定）：

```bash
tail -f /tmp/rc_full.log
```

改代码加日志用 ROS 日志宏，不要用 `printf`（没有时间戳和级别，混不进 ROS 日志）：

```cpp
RCLCPP_INFO(node->get_logger(), "[控制权] 网页接管发杆量");
RCLCPP_WARN(node->get_logger(), "杆量无效 age=%.2f", age);
```

### 4.4 从外部观察，不改代码

```bash
ros2 topic echo /fmu/in/manual_control_input --once   # 看自己发的杆量
ros2 topic hz   /fmu/in/manual_control_input          # 应为 50 Hz
ros2 topic echo /fmu/out/vehicle_status_v1 --once     # armed / nav_state
rqt_graph                                             # 看节点连了什么
curl -s localhost:8000/health                         # 后端活着吗
cat /tmp/flight_control.lease                         # 控制权现场
```

⚠️ 两个必须记住的点：

1. **状态话题带 `_v1` 后缀**：`VehicleStatus` 的 `MESSAGE_VERSION=1`，实际话题是
   `/fmu/out/vehicle_status_v1`。写成 `/fmu/out/vehicle_status` 收不到任何数据。
2. **PX4 用的是 best_effort QoS**，`ros2 topic echo` 默认 reliable 可能一条都收不到，
   卡住就加 `--qos-reliability best_effort`。

### 4.5 纯函数直接单测（不用起仿真）

`http_util` / `mode_tables` / `config` 三个没有 ROS 依赖，最快验证方式：

```bash
cat > /tmp/t.cpp <<'EOF'
#include "flight_ui_cpp/mode_tables.hpp"
#include <cstdio>
int main() {
    printf("%s | %d\n", modeNameOf(1).c_str(), mainModeOf("ALTCTL"));
    return 0;
}
EOF

g++ -std=c++17 -I src/flight_ui_cpp/include /tmp/t.cpp \
    src/flight_ui_cpp/src/mode_tables.cpp -o /tmp/t && /tmp/t
```

---

## 5. 参数陷阱与易错点

### 5.1 `msg.valid` 与 `c.valid` 不是一回事

| 变量 | 来源 | 含义 |
|---|---|---|
| `msg.valid` ← `has_pilot` | `c.any_client`（还有网页连着） | **有没有遥控器在**，PX4 用它判断链路丢失 |
| `c.valid` | `deadman_ && age < 0.3` | **手在不在杆上**（真按着使能） |

早期把 `msg.valid` 填成 deadman，用户一松手 PX4 就判"遥控丢失"，触发 failsafe 并拒绝解锁。
油门兜底也要区分：

| 情况 | 油门值 | 原因 |
|---|---|---|
| 按着使能 | 用户给的值 | 正常操作 |
| 没按，**未解锁** | `-1.0` | PX4 解锁硬性要求油门最低位 |
| 没按，**已解锁** | `0.0` | ALTCTL 中位 = 零爬升率 = **悬停**；给 -1 会掉下来 |

### 5.2 单位：微秒 vs 纳秒

PX4 消息一律**微秒**，ROS 时钟给**纳秒**，要 `/1000`：

```cpp
msg.timestamp = static_cast<uint64_t>(node->get_clock()->now().nanoseconds() / 1000);
```

`nowUs()` 就是干这个的。

### 5.3 摇杆是 `[-1, 1]` 归一化

`throttle`：`-1` = 最低、`0` = 中位（ALTCTL 里 = 悬停）、`+1` = 满推。
不是 `0~100`，也不是 `0~1`。

### 5.4 出参 vs 返回值

`readFile(path, content)` 返回 `bool` 只是成败，内容在 `content` 里。
`acquire(me, prio)` 返回 `bool` 表示"我现在有没有控制权"，租约文件是副作用。

### 5.5 阻塞函数会卡住调用线程

`command_sequence.cpp` 里所有 `*Sequence` 内部都会 `sleepMs`（解锁重试最多 8 次 × 1.5 s）。
它们跑在 **WebSocket 连接线程**里，所以那条连接在这十几秒内发不了别的消息。

### 5.6 两套模式编号

`nav_state`（18 种，只读回显）和 `mainModeOf` 返回的主模式号（1~7，发 `DO_SET_MODE`）**不是一套**。
混用会导致"切模式没反应"。

用 4.5 的方法把两张表打出来，一眼就能看出问题（真实输出）：

```
nav_state= 1 -> ALTCTL          ← 读方向
nav_state= 2 -> POSCTL
nav_state=14 -> OFFBOARD
nav_state=17 -> AUTO_TAKEOFF
nav_state=18 -> AUTO_LAND

mainModeOf("MANUAL")     -> 1   ← 写方向
mainModeOf("ALTCTL")     -> 2
mainModeOf("POSCTL")     -> 3
mainModeOf("OFFBOARD")   -> 6
```

同一个「ALTCTL」，读方向是 **1**，写方向是 **2**。
想把读到的状态再写回去，必须先 `modeNameOf` 拿到名字、再 `mainModeOf` 换成主模式号，
**不能直接把 nav_state 塞进 p2**。

---

### 5.7 读代码时绕不开的两个概念

#### 概念一：原子替换（`writeEntry` / `_write_lease` 里的 tmp + rename）

**一句话**：换掉一个文件的过程不可分割 —— 别人要么看到完整的旧文件，要么看到完整的新文件，
**绝看不到"写了一半"的文件**。

**比喻**：你在门口挂白板写"谁在开车"。

- 直接写 = 擦干净白板再一笔一笔写 → 擦完到写完这段时间，路过的人看到**空板**，以为没人负责
- 原子替换 = 在里屋另一块板写好，**咔哒一下把两块板对调** → 外面永远是写着字的板

那个"咔哒一下"就是 `rename()`：它**不抄内容**，只把门牌号指到另一块板上，所以是瞬间的。

```cpp
const std::string tmp = kPath + ".tmp." + std::to_string(getpid());
{
    std::ofstream out(tmp, ...);      // ① 在"里屋"写
    out << owner << ' ' << prio << ...;
}                                     // ② 右花括号：析构 = 关闭文件，内容落盘
std::rename(tmp.c_str(), kPath);      // ③ 咔哒一下
```

**为什么那两个花括号不能省**：`ofstream` 是带缓冲的，内容关闭时才真正写进文件。
不等它关闭就改名，等于"字还没写完就把板子推出去给人看"。

**自己验一下**（10 行就能看出差别）：

```python
import os
bad1 = bad2 = 0
for _ in range(50):
    open('/tmp/f1.txt', 'w').close()          # 直接写：先清空
    if os.path.getsize('/tmp/f1.txt') == 0: bad1 += 1
    open('/tmp/f1.txt', 'w').write('内容')

    with open('/tmp/f2.tmp', 'w') as f: f.write('内容')
    os.replace('/tmp/f2.tmp', '/tmp/f2.txt')  # 原子替换
    if os.path.getsize('/tmp/f2.txt') == 0: bad2 += 1
print('直接写：', bad1, '次读到空文件')
print('原子替换：', bad2, '次读到空文件')
```

实测输出：**直接写 50/50 次读到空文件，原子替换 0/50**。
（`os.replace` 就是 Python 版的 `std::rename`。）

**项目里三处都用这个模式**：`control_lease.cpp`、`stick_keeper.py`、`pose_writer.py`。
`pose_writer.py` 那个是踩过坑才加的 —— 不加的话 `cam_follow.py` 会读到半截 JSON 解析失败，
相机卡在上一帧，表现为"飞到一半飞机不见了"。

**一个必须记住的前提**：`.tmp` 和目标**必须在同一文件系统**（这里同目录，天然满足）。
跨文件系统的 `rename` 会退化成"复制 + 删除"，原子性就没了。

**临时文件名为什么要带 PID**：C++ 和 Python 用的是同一份租约。如果两端共用一个固定的
`.tmp`，同时写就会把内容交织进同一个文件，再被一方 rename 成正式租约。
实测这一改动把并发压测的解析失败从 **35/4000 降到 0/4000**。

#### 概念二：控制权租约（为什么不能两边一起发杆）

`rc_server`（网页后端）和 `stick_keeper.py`（终端摇杆守护）都以 50 Hz 往
`/fmu/in/manual_control_input` 发杆量。两边同时发会互相覆盖（实测曾出现 99.9 Hz、杆量抖动）。

所以做了个**选举**：谁的名字写在 `/tmp/flight_control.lease` 里，谁才有资格发。

```
文件内容：web 10 1234 1793197679795
           │   │   │        └─ 写入时刻（毫秒），超过 2 s 视为过期
           │   │   └─ 进程号，进程死了就可以被抢
           │   └─ 优先级，数值大的赢（web=10 > terminal=5）
           └─ 谁
```

**什么时候能拿到**（`acquire()` 的判断，两端逻辑一致）：

1. 文件不存在
2. 已经过期（超过 2 s 没刷新）
3. 持有者进程已经死了
4. 就是自己（相当于续租）
5. 对方优先级比自己低

所以网页一打开（prio 10）就能夺走终端的（prio 5）；网页关掉后终端又能收回。

**读它要注意**：`read()` 解析失败会返回 `valid = false`，被当成"没有租约"。
这就是为什么必须用原子替换 —— 一次误判就会导致两边同时发杆，正是租约要防的事。

## 6. 建议节奏

```
Day 1 上午：mode_tables + http_util + config（3 个小文件，1 小时够了）
Day 1 下午：shared_state.hpp（慢慢读，画锁关系图）
Day 2 上午：control_lease + command_sequence
Day 2 下午：http_server + ws_session（按 JSON 消息类型切块）
Day 3：main.cpp（当目录读），然后开 gdb 断在 50 Hz 定时器上，
      改一个油门系数、看 topic 变化 —— 走通一次完整"改 → 验"闭环
```

### 练习清单（每项做完都要能看到现象）

1. 用 gdb 断在 `main.cpp:121`，打印 `c`，解释为什么 `any_client = false`
2. 在 `command_sequence.cpp` 的 `takeoffSequence` 里加一行日志，重新 build，网页点起飞，在 `/tmp/rc_full.log` 看到它
3. 把 50 Hz 改成 20 Hz（改 `config/flight_ui.yaml` 的 `control_rate_hz`），用 `ros2 topic hz` 验证
4. 用 4.5 的方法给 `modeNameOf` 写个单测，把 18 个 nav_state 全打出来
5. 把 `takeoffSequence` 里的 `sendCommand(..., 176, 1.0F, 3.0F)` 改成 `1.0F, 1.0F`（p2 变成 MANUAL），
   观察飞机切到了哪个模式（`ros2 topic echo /fmu/out/vehicle_status_v1 --once` 看 `nav_state`）
6. 开两个浏览器标签，观察 `SharedState::attached_` 里有两个元素、`client_` 只有一个

---

## 7. 速查

```bash
# 编译（必须在工作区根目录）
source /opt/ros/humble/setup.bash
colcon build --packages-select flight_ui_cpp
source install/setup.bash

# 起后端
flight remote            # 或 ./scripts/fly.sh remote

# 看日志
tail -f /tmp/rc_full.log

# 调试
gdb -q --args build/flight_ui_cpp/rc_server

# 观察话题
ros2 topic hz   /fmu/in/manual_control_input
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

---

← 上一章：[14-ros-topic-flight.md](14-ros-topic-flight.md) ｜ 返回 [docs/README.md](README.md)
