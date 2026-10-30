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

### 2.1 两个外部工具：Beast 和 tmux

读这个仓库到处会撞到这两个名字，它们是完全不同层次的东西 ——
**Beast 是程序里的库，tmux 是运维工具**。

#### Boost.Beast：HTTP / WebSocket 的协议实现

层次关系：

```
你的代码（http_server.cpp / ws_session.cpp）
     ↓
Boost.Beast        ← 懂 HTTP 报文 和 WebSocket 帧
     ↓
Boost.Asio         ← 懂 TCP socket（收发字节）
     ↓
操作系统
```

**一句话：Asio 只给"能收发字节的管子"，Beast 把字节变成"消息"。**
报文头怎么分、`Content-Length` 怎么数、WS 帧的 mask 怎么算 —— 全替你做了。

项目里的用法（`http_server.cpp` / `ws_session.cpp`）：

```cpp
http::read(stream, buffer, req);      // 读一个完整 HTTP 请求（自己数 Content-Length）
res.prepare_payload();                // 自己算 Content-Length
http::write(stream, res);

websocket::stream<beast::tcp_stream> ws(std::move(stream));
ws.accept(req);                       // 握手：校验 Sec-WebSocket-Key，回 101
ws.read(buffer, ec);                  // 读一个完整 WS 帧
```

**为什么值得用**：Beast 自身是 header-only（不用编一份库出来）、和 Asio 天然一套、
而且 **HTTP 和 WebSocket 用同一个 `tcp_stream`**，所以"同端口先 HTTP 后升级 WS"
写起来很自然（这就是 `websocket::is_upgrade(req)` 那个分支的底气）。

> 注意 `CMakeLists.txt:21` 仍然是 `find_package(Boost REQUIRED COMPONENTS system thread)`
> —— 链接的是 **Asio 依赖的 Boost.System / Boost.Thread**，不是 Beast 本身。

**要记住的特点**：Beast 有同步和异步两套 API。本项目用**同步**（`http::read` 会阻塞），
所以才有"每连接一个线程"的模型（`main.cpp:193`）。异步 API 能一个线程扛几千连接，
但代码复杂得多 —— 局域网点控没必要。

#### tmux：让长跑服务脱离终端存活

**终端复用器**。最关键的一句：**tmux 里的进程不依赖你的终端窗口，
关掉终端、Ctrl-C、SSH 断线，里面的进程照样活着。**

项目里这些服务必须一直跑，所以全进了 tmux：

```bash
tmux new-session -d -s agent "MicroXRCEAgent udp4 -p 8888 > /tmp/flight_agent.log 2>&1"
tmux new-session -d -s sitl  "HEADLESS=1 ./scripts/run_sitl_combined.sh > /tmp/flight_sitl.log 2>&1"
tmux new-session -d -s stick "python3 scripts/stick_keeper.py > /tmp/stick_keeper.log 2>&1"
```

`-d` = detached（后台起，不切过去），`-s` = 会话名。
本项目脚本里实际出现的会话名（`grep` 自 `scripts/*.sh`）：
`agent` `sitl` `web` `stick` `cams` `gzgui` `show` `depthbridge`。

**`tmux ls` 只告诉你"有哪些会话"，看不到里面在干什么。** 想看内容有三条路：

```bash
# ① 连进去（交互式，能敲命令）
tmux attach -t sitl         # 看完按 Ctrl-b 再按 d 脱离 —— 是"脱离"不是"退出"

# ② 不进去，读它屏幕上的内容（脚本里最好用）
tmux capture-pane -pt sitl          # 当前屏
tmux capture-pane -pt sitl -S -200   # 往上翻 200 行历史

# ③ 直接读重定向的日志文件（本项目最常用）
tail -f /tmp/flight_sitl.log
```

> `-p` = 输出到 stdout（不进入），`-t` = 目标会话，`-S -200` = 从历史第 200 行开始。
> 因为 `fly.sh` 起服务时都写了 `> /tmp/xxx.log 2>&1`，所以**多数情况下 `tail -f` 比 attach 方便** ——
> 不占终端、能配合 `grep`、不会因为误按 Ctrl-C 把服务停掉。

`tmux ls` 的实际输出长这样（没有会话时会提示 `no server running`）：

```
sitl: 1 windows (created Thu Oct 29 19:53:13 2026)
web:  1 windows (created Oct 29 20:15:31 2026)
```

其余管理命令：

```bash
tmux kill-session -t web    # 干掉某一个
tmux kill-server            # 全干掉
```

`Ctrl-b` 是前缀键：`Ctrl-b` `d` = 脱离（最常用）、`Ctrl-b` `[` = 滚动翻历史输出。

**为什么不用 `nohup`**：nohup 能活但**看不见输出、没法交互**；
tmux 既能后台存活，又能随时进去看。调试场景下 tmux 更划算。

**排错时最有用的一招**：解锁被拒、仿真起不来时，
`tmux attach -t sitl` 进去看 PX4 的 `Preflight Fail` 原因 ——
`fly.sh` 的提示语指的就是这个。

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

**字段按锁分两拨，这是这个文件的第一层结构**

| 锁 | 保护的字段 | 谁来写 | 谁来读 | 频率 |
|---|---|---|---|---|
| `mutex_`（控制链） | `client_` `attached_` `deadman_` `roll_` `pitch_` `yaw_` `throttle_stick_` `last_update_` | `setSticks` / `attachClient` / `releaseClient`（WS 线程） | `control()`（50 Hz 定时器） | 50 Hz 写 + 50 Hz 读 |
| `telemetry_mutex_`（遥测链） | `armed_` `nav_state_` `last_status_` `z_valid_` `z_` `ref_alt_` `voltage_v_` `battery_pct_` `obstacle_m_` `last_obstacle_` | `setStatus` / `setPosition` / `setBattery` / `setObstacle`（ROS 回调） | `getTelemetryFor()`（10 Hz 推送）、`armed()` 等 | 回调频率 + 10 Hz |

**为什么要拆两把锁**：控制链是 20 ms 一次的高频活，遥测链是 10 Hz。
合成一把锁的话，50 Hz 的 `control()` 会和每次遥测写入互相排队，
在 CPU 吃紧时可能拖慢杆量下发 —— 而杆量下发是安全相关的。

**关键不变量：两把锁从不嵌套。**

看 `getTelemetryFor()` —— 它需要两边数据，但用的是**两个独立作用域**：

```cpp
Json::Value root(Json::objectValue);
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);   // 先遥测
    ...
}                                                          // ← 先释放
{
    std::lock_guard<std::mutex> lock(mutex_);              // 再控制
    ...
}                                                          // ← 再释放
```

不是"持着 A 去抢 B"。代码注释也点明了这个前提：
`// 两把锁分开取，永不嵌套反序（别处没有先 control 后 telemetry 的路径）`。
**只要没有任何路径嵌套，就不可能出现死锁。**

**读这个文件只问四个问题**

1. 这个字段归哪把锁管？（见上面的表）
2. getter 返回的是快照还是引用？**全是值** —— 这是它能安全跨线程的根本原因
3. 哪些是"派生"的？`valid = deadman_ && age < 0.3`、`any_client = !attached_.empty()`
   都是**算出来的**，不是存下来的（存下来就得同步两份，容易忘）
4. 有没有在锁里干活？`getTelemetryFor` 在锁里构造 `Json::Value`，属于轻度活，可以接受；
   但别在锁里发 DDS、别在锁里做文件 IO

**`setSticks` 里那条"旁观者不能抢控制权"的规则**（最容易被忽略的一段）

```cpp
if (client_ != client && !deadman) {
    return;                    // 别的连接发来的、又没按使能 → 直接丢弃
}
client_ = client;              // 按了使能 → 才算接管
```

场景：你开着两个标签页，A 在飞，B 只是"看"。
B 的页面也在每 20 ms 发一帧 `deadman=false` 的摇杆——那是它页面自己在跑。
如果没有上面那个 `if`，B 的帧会把 `client_` 改成 B，HUD 上的"操控者"就来回跳，
A 甚至可能因为 owner 变了而被判失效。

加上这个 `if` 之后：旁观者怎么发都进不来；但只要它**按下使能**，
就会立刻接管（`client_ = client`）—— 这是有意的"抢控制权"通道。

**实测这张规则表**（两标签 A/B 都连着，用 `getTelemetryFor()` 的 `controller` 字段观察 owner）：

| 动作 | `valid` | `any_client` | A 看到的 | B 看到的 |
|---|---|---|---|---|
| 都连上，没人按使能 | 0 | 1 | none | none |
| A 按下使能 | 1 | 1 | **you** | other |
| B 发 `deadman=false` 的帧 | 1 | 1 | **you** | other |
| B 按下使能 | 1 | 1 | other | **you** |
| A 松杆（已不是 owner） | 1 | 1 | other | **you** |
| B 断线 | 0 | 1 | none | none |

两处值得注意：

- 第 3 行证明了旁观者的帧确实进不来；第 4 行证明按下使能就能抢。
- 最后一行：**断线会把控制权清零，而不是转交给另一个还连着的标签**。
  `releaseClient` 里是 `client_ = 0; deadman_ = false;`，所以 A 必须重新按一下使能才能接管。
  这是偏保守的选择 —— 不会让"没在操作的人"突然变成操控者。

**数据口径的三个坑**

| 字段 | 坑 |
|---|---|
| `z_` | NED 坐标，**向下为正**。所以高度是 `-z_`（见 `root["alt"] = z_valid_ ? -z_ : 0.0`） |
| `ref_alt_` | AMSL 基准。起飞要给 `ref_alt_ + 相对高度`，且 NED 下是 `ref_alt_ - z_ + relative_m`；拿不到 `ref_alt_` 时退化成相对高度 |
| `-1` | `obstacle_m_` 和 `battery_pct_` 都用 **-1 表示"未知"**（前端据此显示 `--`）。障碍还带 2 s 过期：没感知在跑时读出来也是 -1 |

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

**逐个函数看**

| 函数 | 行 | 干什么 | 会不会阻塞 |
|---|---|---|---|
| `sendCommand` | 17 | **所有命令的唯一出口**。填 7 个参数位 + 路由字段（`target_system=1`、`from_external=true` 等），然后 `publish` | 否 |
| `nowUs` | 38 | 当前时间（纳秒 / 1000） | 否 |
| `isManualNavState` | 45 | 判断 `nav_state` 是不是手动类：**0(MANUAL) / 1(ALTCTL) / 2(POSCTL) / 10(ACRO) / 15(STABILIZED)** | 否 |
| `sleepMs` | 51 | 阻塞睡 | **是** |
| `ensureManualMode` | 58 | 不是手动模式就先切 POSCTL（`176, 1.0F, 3.0F`），再睡 800 ms 等它生效 | **是（800 ms）** |
| `armWithRetry` | 69 | 解锁重试：每轮发一条 ARM，然后 1 s 内每 100 ms 查一次 `armed`；1 s 没成功就重新 `ensureManualMode` 再来一轮，直到 `arm_retry_sec`（默认 10 s）用尽 | **是（最长 10 s）** |
| `armSequence` | 90 | `ensureManualMode` → `armWithRetry`；**失败就补发一条 `400, 0.0F` 确保留在上锁状态** | **是** |
| `takeoffSequence` | 103 | `ensureManualMode` → 没解锁就先解锁（失败则直接 return，不硬起飞）→ 睡 300 ms → 切 POSCTL → 睡 800 ms → 发 `NAV_TAKEOFF`（p7 = AMSL 高度） | **是（≥1.1 s + 解锁时间）** |
| `landNow` | 120 | 一条 `21` | 否 |
| `killNow` | 127 | 先 `400, 0.0F` 上锁，再 `185, 1.0F` 飞行终止 | 否 |

**`isManualNavState` 里那个注释值得单独记**（第 44 行）：

```cpp
// 注意 5 是 AUTO_RTL，不是 ACRO —— ACRO 是 10。
```

这是 `nav_state` 编号最反直觉的一处。写错的话 `ensureManualMode` 会认为
"AUTO_RTL 也算手动模式"，于是不切模式，解锁就被拒。

**`armWithRetry` 为什么是这个双层循环**

```cpp
while (now < deadline) {                       // 外层：最多 arm_retry_sec 秒（默认 10）
    sendCommand(cmd_pub, nowUs(node), 400, 1.0F);      // 发一条 ARM
    const auto tick = now + 1s;
    while (now < tick) {                       // 内层：1 秒内每 100 ms 看一次 armed
        if (state.armed()) return true;
        sleepMs(100);
    }
    ensureManualMode(state, node, cmd_pub);    // 被拒可能是模式被踢走了，重切一次
}
return false;
```

- **为什么不能发一条就认为成功**：PX4 冷启动要跑 arming checks（EKF 收敛、气压计预热），
  第一次 ARM 十有八九被拒。实测要十几秒才收敛。
- **为什么内层要轮询**：`armed` 是 PX4 遥测回调异步写进来的，只能等，不能"发完就当成了"。
- **为什么每轮都 `ensureManualMode`**：解锁被拒的原因之一是"当前在 AUTO_* 模式"，
  每轮重切一次 POSCTL 把这个因素排除掉。

**`takeoffSequence` 里那两个 sleep 是干嘛的**

```cpp
sleepMs(300);                                  // 等解锁状态稳定
sendCommand(cmd_pub, nowUs(node), 176, 1.0F, 3.0F);   // 切 POSCTL
sleepMs(800);                                  // 等模式切换生效
sendCommand(cmd_pub, nowUs(node), 22, ...);           // 再发 NAV_TAKEOFF
```

PX4 处理模式切换是异步的：发完 `DO_SET_MODE` 立刻发 `NAV_TAKEOFF`，很可能
TAKEOFF 到达时模式还没切过去，被丢弃。所以中间必须留时间。

**一个行为差异要知道**：这里的 `ensureManualMode` / `takeoffSequence` 都会切 **POSCTL**，
而 `scripts/fly.sh` 走的是 **ALTCTL**。原因是无 GPS 时 POSCTL 的横滚/俯仰会被抑制
（`xy_valid` 不成立），只能垂直方向动。所以网页起飞是正常的（起飞只要垂直），
但**起飞后想在 POSCTL 里前后左右飞，需要先点 ALTCTL 按钮**
（这就是 `4023dc3` 加那个按钮的原因）。

### 3.4 第四层：IO 外壳

#### `main.cpp:174-216` 的 socket 引导（读 `http_server.cpp` 的前置）

这段是**裸 POSIX socket**，不经过 Beast。它的作用是"开一个门，等人来敲"：

```cpp
const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);          // ① 造一个 TCP socket
int yes = 1;
::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));  // ② 允许立刻重用端口

sockaddr_in addr{};                        // ③ 全零初始化（必须）
addr.sin_family      = AF_INET;            //    IPv4
addr.sin_port        = htons(port);        //    网络字节序
addr.sin_addr.s_addr = INADDR_ANY;         //    所有网卡都听
if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(logger, "bind(:%d) 失败，端口被调用？", port);   // ④ 端口被占 → 直接退出
    rclcpp::shutdown(); return 1;
}
::listen(listen_fd, 16);                   // ⑤ 进入监听，backlog 16
```

| 步骤 | 要点 |
|---|---|
| ① `::socket(AF_INET, SOCK_STREAM, 0)` | 开头的 `::` 表示调用**全局**的同名函数（不是类成员）。`AF_INET`=IPv4、`SOCK_STREAM`=**TCP**、第三个参数 0 = 用默认协议。返回的 `listen_fd` 是个整数句柄 |
| ② `SO_REUSEADDR` | **很实用**：上次进程退出后，那条连接会在内核里留 `TIME_WAIT` 约 60 秒，没有这个选项重启会报 `Address already in use`。加上之后可以立刻重新 bind |
| ③ `sockaddr_in addr{}` | 那个 `{}` 是**值初始化**，把所有字段清零。不写的话 `sin_zero[8]` 那些填充字节是栈上垃圾值，某些系统会 bind 失败 |
| ③ `htons(port)` | **host to network short**。网络字节序是大端，x86 是小端，所以必须转。忘了写就会连到莫名其妙的端口 |
| ③ `INADDR_ANY` | 监听**所有网卡**，不只是 `127.0.0.1`。好处是同一局域网的手机也能访问；**代价是同一网络的人都能操控这架飞机** |
| ④ bind 失败 | 最常见原因就是端口被占（上次的 rc_server 没退干净）。日志里的"端口被调用？"是笔误，实际意思是"端口已被占用" |
| ⑤ `listen(fd, 16)` | backlog = 16：内核最多帮你在队列里排 16 个"已握手但还没被 accept 的连接"。**`listen` 本身不阻塞**，只是把 socket 标成"接受连接"状态 |

**然后开一个线程循环 `accept`**

```cpp
std::atomic<bool> running{true};
std::thread http_thread([&]() {
    while (running) {
        const int fd = ::accept(listen_fd, nullptr, nullptr);   // ← 阻塞在这里
        if (fd < 0) break;
        std::thread(handleConnection, fd, std::ref(state), web_root,
                    node.get(), cmd_pub).detach();              // 每连接一个线程
    }
});
```

- **`std::atomic<bool>` 不能省**：`running` 在 main 线程写、在 http 线程读，普通 `bool` 是数据竞争（UB）。
- **`accept` 是阻塞调用**：没人连进来时线程就停在这一行。所以必须放在独立线程，不能挡着 `rclcpp::spin`。
- **`std::ref(state)`**：`std::thread` 默认**拷贝**参数，而 `SharedState` 不可拷（含 `std::mutex`）。`std::ref` 传的是引用。
- **`node.get()`**：`node` 是 `shared_ptr`，`.get()` 取出裸指针传给线程——因为节点生命周期比这些线程长（活到 main 结束）。

**关闭时的顺序很讲究（`main.cpp:207-213`）**

```cpp
rclcpp::spin(node);                        // 阻塞直到 Ctrl-C

running = false;                            // ① 只改标志，叫不醒 accept！
::shutdown(listen_fd, SHUT_RDWR);           // ② 这一句才让 accept 立刻返回 -1
::close(listen_fd);
if (http_thread.joinable()) http_thread.join();   // ③ 等线程真正退出
```

**只写 `running = false` 是没用的** —— 因为线程卡在 `accept()` 内部，根本没机会去检查 `running`。
必须用 `::shutdown(listen_fd, SHUT_RDWR)` 让那个阻塞调用立刻返回错误码，`fd < 0` → `break`。

> 这是个经典范式：**"通知标志" + "打断阻塞调用"**，两者缺一不可。
> 实测验证：给进程发 SIGINT 后进程干净退出，8000 端口立刻释放
> （日志里能看到 `signal_handler(SIGINT/SIGTERM)`，说明是 rclcpp 的默认 handler 触发的）。

#### `src/flight_ui_cpp/src/http_server.cpp`（104 行）

```cpp
void handleConnection(int fd, SharedState & state, const std::string & web_root,
                      rclcpp::Node * node, const CommandPublisher & cmd_pub);
```

调用方在 `main.cpp:193-200`：

```cpp
const int fd = ::accept(listen_fd, nullptr, nullptr);   // 主循环收一个连接
std::thread(handleConnection, fd, ...).detach();        // 甩给新线程，主循环继续收
```

**即"每连接一个线程"。** 函数体分 6 块，按顺序读：

| # | 代码 | 在干什么 |
|---|---|---|
| ① | `static std::atomic<uint64_t> next_client{1};`<br>`const uint64_t client_id = next_client++;` | 给连接发号。`static` 让计数器跨调用保留；`atomic` 因为多线程同时执行这两行。这个 id 后面被 `SharedState::attached_` 用来区分是哪个浏览器 |
| ② | `net::io_context ioc(1);`<br>`socket.assign(tcp::v4(), fd, ec);`<br>`if (ec) { ::close(fd); return; }` | 把裸 `fd` 交给 Beast 的 socket 管。**失败分支必须自己 `close(fd)`** —— 那时 socket 还没接管成功，不关就漏 fd。成功后所有权归 socket，不能再手动关 |
| ③ | `beast::tcp_stream stream(std::move(socket));`<br>`http::read(stream, buffer, req);` | **阻塞**读完一个完整 HTTP 请求（读到 `Content-Length` 收齐）。因为每连接一个线程，阻塞只影响自己 |
| ④ | `if (websocket::is_upgrade(req))`<br>`serveWebSocket(std::move(stream), std::move(req), ...); return;` | 判断请求头有没有 `Upgrade: websocket`。`std::move` 是**必须**的：`tcp_stream` 只能移动不能复制。那个 `return` 也**必须**——WS 跑完返回后绝不能再往下写 HTTP 响应，否则协议错乱 |
| ⑤ | `/health` 拼 JSON<br>否则静态文件 | 路径处理见下面的表 |
| ⑥ | `res.prepare_payload(); http::write(stream, res);` | `prepare_payload()` 负责写 `Content-Length`，**必须等 body 最终确定之后调** |

第 ⑤ 块路径处理的三个细节：

| 代码 | 目的 |
|---|---|
| `if (relative.find("..") != npos) → 400` | 挡路径穿越，如 `GET /static/../../etc/passwd` |
| `target == "/"` → `"/index.html"`；`/static/xxx` → `"/xxx"` | 只有这两种映射，其它路径按字面找文件 |
| `keep_alive(false)` | 一次请求一条连接，回完就关，不用管连接复用 |

**整个函数被 `try { } catch (const std::exception &)` 包住，这是必需的**：
线程是 `.detach()` 出去的，异常一旦逃出线程函数就会 `std::terminate` **让整个进程崩掉**。
Beast 的同步 API 在客户端断开时会抛异常，所以这个 catch 是"网页那边出问题不能把飞控后端带走"的最后一道防线。

**背景知识：这里说的「连接」是什么**（读这段代码绕不开）

TCP 连接 = 两台机器之间一条**双向通道**，协议保证**不丢、不乱序、不重复**。
一条连接由 4 个数字唯一确定：`(源IP, 源端口, 目标IP, 目标端口)`。

```bash
ss -tan
# LISTEN      127.0.0.1:8000      0.0.0.0:*      ← 还没人连，只是"守着电话"
# ESTABLISHED 127.0.0.1:8000      127.0.0.1:54321 ← 这条才是连接
```

`::accept()` 返回的 `fd` 就是这条新连接在本进程里的句柄（一个整数，本质是数组下标）。

**最关键的一点：TCP 是「字节流」，不是「消息流」。**
发两次 `"abc"` `"def"`，对方可能一次收到 `"abcdef"`，也可能拆成三段。
**边界是 TCP 不管的，必须靠上层协议自己标记**：

| 协议 | 怎么标记边界 |
|---|---|
| HTTP | `Content-Length: 123`（所以 `http::read` 要读到字节数收齐才返回） |
| WebSocket | 每帧带长度头（所以 `ws.text(true)` 不能省，否则发的是带帧头的二进制，前端 `JSON.parse` 会拿到 Blob） |

**HTTP 和 WebSocket 跑在同一根管子上**：WebSocket 不是新开连接，而是发一个
`Upgrade: websocket` 的 HTTP 请求，服务端回 `101 Switching Protocols`，
之后这根管子就从"一问一答"改成"长期双向"。
这就是上面 `serveWebSocket(...)` 之后**必须 `return`** 的根本原因 ——
管子已经换用法了，再写 HTTP 响应就是往里面塞错格式的数据。

**三个已知取舍**（都能接受，但要知道）：

1. **每连接一线程、无上限**：连接多了线程就多。局域网点控够用，被刷连接会耗尽线程。
2. **`http::read` 没有超时**（实测确认）：客户端连上但一个字节不发，
   等了 12 秒连接仍被占着，那条线程会一直阻塞在这里。
   `ws_session.cpp:34` 那边就设了 `timeout::suggested(...)`（注释写着"客户端拔网线时 read 会永远阻塞，线程就泄漏了"），
   HTTP 这段没加同款保护。要补的话是在 `http::read` 前加：
   ```cpp
   stream.expires_after(std::chrono::seconds(10));
   ```
   超时会抛 `beast::system_error`，被现有的 catch 接住，线程正常退出。
3. **静态文件全量读进内存**（`readFile` → `std::string`）：前端几百 KB 无所谓，
   将来传大文件要改流式。

#### `src/flight_ui_cpp/src/ws_session.cpp`（212 行，最大，最后读）

```cpp
void serveWebSocket(beast::tcp_stream stream,
                    http::request<http::string_body> req,
                    SharedState & state,
                    rclcpp::Node * node,
                    const CommandPublisher & cmd_pub,
                    uint64_t client_id);
```

**一个浏览器标签 = 一条 TCP 连接 = 一个线程 = 这个函数从头跑到尾。** 函数返回，连接就结束了。
整体是「开场五步 → 起一个推送线程 → 死循环读帧分发 → 收尾」。

**开场五步**

| 行 | 代码 | 作用 |
|---|---|---|
| 31 | `websocket::stream<beast::tcp_stream> ws(std::move(stream));` | 把 HTTP 的流包成 WebSocket 流 |
| 34 | `ws.set_option(timeout::suggested(beast::role_type::server));` | **必须设**：拔网线时对端不发 FIN，`read` 会永远阻塞，线程就废了 |
| 37 | `ws.accept(req);` | 握手：校验请求头 + 回 `101 Switching Protocols` |
| 40 | `ws.text(true);` | **必须设**：不设默认发二进制帧，浏览器收到 `Blob`，前端 `JSON.parse` 直接失败 |
| 42 | `state.attachClient(client_id);` | 报到：从此刻起 `any_client = true` → 50 Hz 定时器那边 `msg.valid` 才为 true → PX4 认为有遥控 |

**为什么必须有 `write_mu` 这把锁**（第 49-66 行）

Beast 同步流的规则：同一条连接允许「1 个读 + 1 个写」并发，**绝不能「2 个写」并发**。
而这里有两个写者：

1. 主循环里回 `ack`
2. `pusher` 线程每 100 ms 推一次遥测

两个写者同时往同一个 socket 写，帧会交错，浏览器解析直接崩。
所以**所有写统一走 `send_text` lambda**，由这把锁串行化 —— 这是"唯一出口"模式。

**为什么 `pusher` 要单独开线程**（第 62-68 行）

主循环卡在 `ws.read()` 上等客户端发数据。如果遥测也放在主循环里发，
客户端不打杆时读循环就阻塞着，网页上的高度/模式会全冻住。
所以遥测用独立线程主动推，10 Hz，与客户端是否发帧无关。

**主循环：读一帧 → 解析 → 分发**（第 71-200 行）

```cpp
for (;;) {
    beast::flat_buffer buffer;   // 每轮新建，天然没有上一帧残留
    beast::error_code ec;
    ws.read(buffer, ec);         // 用 error_code 重载：断开时返回错误码而不是抛异常
    if (ec) break;               // 客户端断开/超时，正常收场
    ...解析 JSON（解析失败就跳过，不崩）...
}
```

消息分发（`type` 字段）：

| `type` | 行号 | 作用 |
|---|---|---|
| `sticks` | 107 | 更新摇杆，**`deadman` 是这条消息里的一个字段**（不是独立消息） |
| `disarm` | 114 | `sendCommand(..., 400, 0.0F)` 上锁 |
| `mode` | 125 | `mainModeOf(name)` → `DO_SET_MODE`；返回 0 则回 `ok:false` 报"未知模式" |
| `arm` / `takeoff` | 147 | **两个 type 共用一个分支**，靠 `want_takeoff` 区分是否接着起飞 |
| `land` | 183 | `sendCommand(..., 21)` |
| `kill` | 191 | 上锁 + `FLIGHT_TERMINATION`，同时把杆量置成"松开" |

**`ack` 是什么、怎么用**

`ack` = acknowledgment（**回执**）。它不是前端发来的消息，而是**服务端对每条命令的应答**：
"我收到了、我做了 / 我拒绝了，原因是……"。

三个关键点：

1. **一次命令 → 一条 ack，但不是"成功证明"**
   `ok:true` 只表示**请求被受理**（命令已经发到 `/fmu/in/vehicle_command`），
   **不代表飞机照做了**。真正确认要看遥测里的 `armed` / `mode` 变化。
   所以 `message` 写的是 "arm requested" 而不是 "armed"。

2. **前端只用它打一行日志**（[app.js:197](/home/wiki_flight_project/src/flight_ui/web/app.js:197)）

   ```js
   if (data.type === 'ack')
       log(`${data.action}: ${data.ok ? 'OK' : 'REJECTED'} ${data.message || ''}`);
   ```

   `log()` 是 `logEl.textContent = 时间 + 消息`，所以页面右下角 `#log` 会显示
   类似 `21:52:08  arm: REJECTED 没有遥测：ROS 链路没通` 这样一行。

3. **ack 的四个固定字段**

   | 字段 | 含义 |
   |---|---|
   | `type` | 固定 `"ack"`，前端据此分流（`telemetry` 走另一个分支） |
   | `action` | 回显触发它的命令类型（`arm` / `mode` / `land` / …） |
   | `ok` | `true` = 受理，`false` = 拒绝 |
   | `message` | 给用户看的原因，直接显示在页面上 |

**后端一共有 5 个地方回 ack，`ok:false` 的情形有三种**：

| 位置 | 触发条件 | message |
|---|---|---|
| `ws_session.cpp:96` | 命令类消息但 `statusAgeSec() > 1.5`（ROS 链路没通） | 没有遥测：ROS 链路没通，先跑 flight link |
| `:134` | `mode` 的名字认不出（`mainModeOf` 返回 0） | unknown mode: XXX |
| `:164` | `arm`/`takeoff` 时按着使能且油门 > -0.8 | throttle must be at minimum |

> ⚠️ 注意 `:96` 那条**优先级最高**：没遥测时连 `mode BOGUS` 都会被它挡住，
> 你看到的是"没有遥测"而不是"unknown mode"。

**实测（起 rc_server 但不起仿真，用最小 WS 客户端发命令）**：

```
发 {"type":"arm"}                → {"type":"ack","action":"arm","ok":false,
                                   "message":"没有遥测：ROS 链路没通，先跑 flight link（起 MicroXRCEAgent）"}
发 {"type":"mode","mode":"POSCTL"} → 同上
发 {"type":"mode","mode":"BOGUS"}  → 同上（被链路检查挡住，而不是报 unknown mode）
发 {"type":"land"}                → 同上
```

**调试时怎么用它**：不用开浏览器。WebSocket 里发一条命令，看回来的 `ack.message`
就知道后端是"没收到"、"链路不通"还是"被你自己的油门位置拒了"——
这比盯着网页右下角那一行字高效得多。

**分发里的三个坑**（都是踩出来的）

1. **非 `sticks` 帧绝不能动摇杆**（第 107 行的 `if` 就是干这个的）
   `arm`/`mode`/`takeoff` 这些消息**没有 `deadman` 字段**，若不管类型一律调 `setSticks`，
   `root.get("deadman", false)` 会取到 `false` —— **等于把使能松开了**，解锁必然被拒。

2. **命令类消息先查 ROS 链路通不通**（第 94 行）
   `state.statusAgeSec() > 1.5` 表示 1.5 s 没收到遥测，说明 `/fmu/*` 根本没通
   （典型原因：忘了起 `MicroXRCEAgent`）。这时发命令是石沉大海，
   用户看到的是「点了 ARM 毫无反应」。所以这里主动回 `ok:false` 并说明原因，然后 `continue`。

3. **`arm`/`takeoff` 不因"没按使能"而拒绝，而且必须开线程**（第 147-181 行）
   - 旧逻辑是没按使能就回 `deadman not held`，用户点了起飞毫无反应。
     实际上 PX4 解锁只要求「有有效遥控信号 + 油门在最低」，所以这里干脆
     **替用户把杆量摆成"回中 + 油门最低"**，既满足条件又不会带着偏杆起飞。
   - 反过来，**按着使能却把油门推上去时直接拒**（`throttle must be at minimum`），
     否则一解锁飞机就蹿起来。
   - `takeoffSequence` 内部是**阻塞**的（解锁失败重试 8 次 × 1.5 s ≈ 12 s），
     直接调会堵死读循环，所以 `std::thread(...).detach()` 扔出去。

**收尾**（第 205-210 行）

```cpp
alive = false;                      // 告诉 pusher 别推了
if (pusher.joinable()) pusher.join();   // 等它退出（最多等 100 ms 的 sleep）
state.releaseClient(client_id);     // any_client 变 false → 网页后端主动释放租约 → 终端守护接手
```

**一个值得知道的设计后果**：`armSequence`/`takeoffSequence` 是 `detach()` 出去的，
参数都是 `main` 里的对象，所以**点了起飞马上关网页，飞机照样会起飞**。
作为"命令已下发"是合理的，但从安全角度"操作的人走了命令还在跑"值得留意。

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

### `main.cpp:117` 里的 `c` 是什么（名字容易误会）

```cpp
const auto c = state.control();       // ← c = control，不是 client
const bool armed = state.armed();
```

**`c` 不是"网页"，是"控制状态的一份快照"**。名字来自 `control`（控制），
类型是 `SharedState::Control`，六个字段：

```cpp
struct Control {
    bool valid;                              // 手在不在杆上（deadman + 300ms 内更新）
    bool any_client;                         // 有没有网页连着
    double roll, pitch, yaw, throttle;       // 当前持控制权那个客户端发来的杆量
};
```

**只有 `any_client` 一个字段跟"网页"有关**，其余五个是"操控状态"。
所以 `c` 更准确的名字是 `ctrl` / `snapshot`，而不是 `web`。

**五个场景下 `c` 的实际内容**（用 `SharedState` 直接跑出来的）：

| 场景 | `valid` | `any_client` | roll | pitch | yaw | thr |
|---|---|---|---|---|---|---|
| ① 什么都没连 | 0 | 0 | 0 | 0 | 0 | −1.0 |
| ② 网页 #7 连上，没按使能 | 0 | 1 | 0 | 0 | 0 | −1.0 |
| ③ #7 按着使能打杆 | 1 | 1 | 0.3 | −0.2 | 0.5 | 0.1 |
| ④ 又连了网页 #8（旁观） | 1 | 1 | 0.3 | −0.2 | 0.5 | 0.1 |
| ⑤ #7 断开（#8 还连着） | **0** | 1 | 0.3 | −0.2 | 0.5 | 0.1 |

第 ④ 行说明**旁观者不会覆盖杆量**（`setSticks` 里那条 `if` 挡掉了）。

第 ⑤ 行有个值得注意的细节：**操控者断开后 `valid` 变 0，但四个杆量值还是旧的** ——
因为 `releaseClient` 只清 `client_` 和 `deadman_`，没清杆量。这**不是 bug**：

- 下发路径：`main.cpp:147` 用 `c.valid ? c.roll : 0.0F` 把旧值**屏蔽掉了**
- 遥测路径：`getTelemetryFor` 确实把旧杆量发给网页了，但前端只在
  `controller === 'other'` 时才用它回显摇杆（[app.js:167](/home/wiki_flight_project/src/flight_ui/web/app.js:167)），
  断开后 `controller` 是 `none`，所以也不会显示

即 **"靠 valid 屏蔽"而不是"断开时清零"** —— 少做一次写操作，效果一样。

> 这也是一般做法：不要靠"把状态清干净"来保证安全，
> 而是在**消费端用一个有效性标志把关**。清理总会漏，把关只写一处。

**但 `main()` 里还有一处必须读懂：四个订阅（`main.cpp:59-85`）**

它们只做一件事 —— **把 PX4 的遥测搬进 `SharedState`**，每个回调只有一两行：

| 行 | 话题 | 消息类型 | 搬进哪里 | 关键字段处理 |
|---|---|---|---|---|
| 59 | `/fmu/out/vehicle_status_v1` | `VehicleStatus` | `state.setStatus()` | `arming_state == 2` → armed（`1=DISARMED, 2=ARMED`）；`nav_state` 原样存 |
| 66 | `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | `state.setPosition()` | `z_valid`（是否可信）、`z`（**NED，向下为正**）、`ref_alt`（AMSL 基准） |
| 72 | `/fmu/out/battery_status` | `BatteryStatus` | `state.setBattery()` | `voltage_v`；`remaining` 是 1→0，**-1 表示未知** |
| 80 | `/obstacle_distance` | `std_msgs/Float32` | `state.setObstacle()` | 来自本项目感知包，不是 PX4 |

**为什么订阅用 `best_effort`（`main.cpp:47`）**

```cpp
const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
```

PX4 通过 uXRCE-DDS 发布的话题是 **best_effort**。DDS 的 QoS 兼容规则是
「**可靠的订阅者不能从 best_effort 的发布者收数据**」—— 所以这里写 `reliable()`
的话，四个订阅会**一条都收不到**，而且不会报错，只是静默没数据。

配合 4.4 里那条：`ros2 topic echo` 默认 reliable，也要加
`--qos-reliability best_effort` 才能看到数据。

**实测这条规则**（同一个 best_effort 发布者，两个订阅者各发 194 条）：

```
发布者 QOS = BEST_EFFORT，共发 194 条
  reliable    订阅者收到:   0 条
  best_effort 订阅者收到: 193 条
```

同时 rclpy 会打一条警告，**这就是"订阅了没数据"时该去日志里找的话**：

```
[WARN] [qos_sub]: New publisher discovered on topic '/qos_probe',
offering incompatible QoS. No messages will be received from it.
Last incompatible policy: RELIABILITY
```

看到 `incompatible QoS ... Last incompatible policy: RELIABILITY`，
就说明订阅端写成了 reliable，改成 best_effort 即可。

**唯一一个用 `reliable` 的订阅是 `/obstacle_distance`（`main.cpp:80`）**

```cpp
create_subscription<std_msgs::msg::Float32>(
    "/obstacle_distance", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), ...)
```

因为它的发布者 `min_depth.cpp` 是本项目的节点，用的是默认 QoS（reliable）。
**订阅两端 QoS 必须匹配** —— 这条能收到，而 PX4 那三条不能改成 reliable，
原因就在这里。

**两个容易忽略的细节**

1. **订阅对象必须活到 main 结束**。它们存在局部变量 `status_sub` / `pos_sub` / …
   里，函数返回时析构。若写成临时对象，订阅会被立刻销毁、再也收不到数据 ——
   这是 ROS 2 新手最常踩的坑之一。
2. **`obstacle_m` 默认是 -1**（"没有数据"），所以没跑感知包时 HUD 显示 `--`，
   不是显示 0。这个 `-1` 约定见 5.x 数据口径那节。

### 3.5b 三个小文件（先啃掉，建立信心）

#### `src/flight_ui_cpp/src/mode_tables.cpp`（43 行）

**两张表，方向相反，编号也不是同一套**：

```cpp
std::string modeNameOf(int nav_state);      // 读方向：PX4 状态 → 名字（18 个 case）
int         mainModeOf(const std::string&); // 写方向：名字 → 主模式号（7 个）
```

- `modeNameOf`：18 个显式 `case`（0~6、10、12~15、17~22），其余落到
  `default: "STATE_" + std::to_string(nav_state)` —— 所以**永远不会返回空串**，
  未映射的（7/8/9/11/16）会显示 `STATE_7` 这种。
- `mainModeOf`：只认 7 个主模式名，`MANUAL=1 ALTCTL=2 POSCTL=3 AUTO=4 ACRO=5
  OFFBOARD=6 STABILIZED=7`，**认不出就返回 0**（`ws_session` 据此回 `unknown mode`）。

⚠️ **同一个模式，读和写的数字不一样**（实测）：

| | 读方向 | 写方向 |
|---|---|---|
| ALTCTL | `nav_state = 1` | `mainModeOf → 2` |
| POSCTL | `nav_state = 2` | `mainModeOf → 3` |

所以"把读到的状态再写回去"必须经过 `modeNameOf` → `mainModeOf` 翻译，
**不能把 `nav_state` 直接塞进参数位**。

另外 `mainModeOf` 只切**主模式**，进不了 `AUTO_TAKEOFF`/`AUTO_LAND` 这种子模式 ——
那些是由 `NAV_TAKEOFF`(22) / `NAV_LAND`(21) 命令触发的，不走这条路。

#### `src/flight_ui_cpp/src/http_util.cpp`（41 行）

三个纯函数，不碰 ROS，是最好单测的一个（见 4.5）。

```cpp
std::string jsonString(const Json::Value & v);                    // Json → 紧凑字符串
bool readFile(const std::string & path, std::string & content);   // content 是出参
std::string contentTypeOf(const std::string & path);              // 后缀 → MIME
```

- `jsonString` 里 `builder["indentation"] = ""`：**压缩成一行**。WebSocket 每 100 ms
  推一次遥测，缩进是纯浪费带宽。
- `readFile` 用 `ss << file.rdbuf()` 一把读完，比逐行读快；打不开就返回 false，
  内容留在出参里（不修改）。调用方是 `http_server` 的静态文件分支。
- `contentTypeOf` **手写后缀比较**，注释点明了原因：`std::string::ends_with` 是 C++20，
  本项目 C++17。返回的 MIME 都带 `; charset=utf-8` —— **不加中文会乱码**。
  认不出的后缀兜底 `text/plain`。

#### `src/flight_ui_cpp/src/config.cpp`（37 行）

```cpp
const FlightUiConfig & config();          // 全局单例，返回 const 引用
void loadConfig(rclcpp::Node & node);     // main 里调一次
```

**为什么用全局**（文件头注释就写了）：HTTP 线程、WebSocket 会话、定时器、命令时序
都要读这几个参数。逐层传参会让"6 个文件互相认识"，不如集中放一处。

**`declare_parameter` 的默认值取自结构体**：

```cpp
g_config.port = node.declare_parameter<int>("port", g_config.port);
```

所以优先级是：**YAML 里的值 > `FlightUiConfig` 里的默认值**。

`main.cpp:44` 在创建任何定时器之前先调 `loadConfig(*node)`，
`main.cpp:45` 再 `const auto & cfg = config();` —— 顺序不能反。

**重点是四个 `std::clamp`**：

```cpp
g_config.control_rate_hz   = std::clamp(..., 1.0, 500.0);
g_config.telemetry_rate_hz = std::clamp(..., 0.1, 50.0);
g_config.arm_retry_sec     = std::clamp(..., 1.0, 60.0);
g_config.takeoff_altitude_m= std::clamp(..., 0.3, 20.0);
```

注释解释了最要命的那条：**`control_rate_hz` 写成 0，定时器周期会变成 0（或负）→ 直接崩**。
`std::clamp` 把配置错误挡在启动阶段，而不是让程序在 50 Hz 定时器里除以零。

> `port` 没有 clamp。写成 0 会让系统随机分配端口，写成 80 会被权限拒绝 ——
> 属于"填错了自己看得出来"的类型，可以接受。

### 3.6 感知包（风格完全不同：没有网络、没有服务，只有「订阅 → 算 → 发布」）

#### `src/flight_perception_cpp/src/min_depth.cpp`（92 行）

```cpp
class MinDepth : public rclcpp::Node;       // 32FC1 深度图 → 中央 ROI 最小距离
int main(int argc, char ** argv);
```

**数据流**：`/depth_camera`（`sensor_msgs/msg/Image`，`32FC1`）→ 中央 ROI 取最小 → `/obstacle_distance`（`std_msgs/msg/Float32`）

**四个可调参数**（`declare_parameter`，改的时候不用重新编译）：

```bash
ros2 run flight_perception_cpp min_depth --ros-args \
  -p image_topic:=/depth_camera -p max_range:=20.0 \
  -p roi_width:=0.5 -p roi_height:=0.6
```

**这段代码的全部难点是「怎么正确索引一个 32FC1 图像」**

```cpp
const size_t row_len = msg->step / sizeof(float);   // step 是字节数！
if (row_len < msg->width) return;
if (msg->data.size() < msg->step * msg->height) return;
const float * data = reinterpret_cast<const float *>(msg->data.data());
...
for (size_t v = y0; v < y0 + rh; ++v) {
    const float * row = data + v * row_len;         // 按 row_len 走，不是 width
    for (size_t u = x0; u < x0 + rw; ++u) { const float d = row[u]; ... }
}
```

- `step` = **每行字节数**，可能大于 `width * 4`（行尾有对齐填充，padding）。
  所以必须 `data[v * row_len + u]`；写成 `data[v * width + u]` 在正方形图上凑巧对，
  换成 641 宽的图就会整体错位。
- `reinterpret_cast<const float*>` 安全的前提：`32FC1` 就是连续的 float32 数组。
  这就是"不用 OpenCV"的底气——**不需要解码，`msg->data` 本质上就是 `float[]`**。
- 两处防御：`row_len < width`（step 小得不可能）和 `data.size() < step*height`（消息被截断）都直接 `return`。

**为什么要算这个矩形（ROI = Region Of Interest，感兴趣区域）**

深度相机是**广角**的：`StereoOV7251` 水平 **73.0°**、垂直 **58.1°**。
整张图里除了「正前方」，还有侧墙、地面、甚至机体自己的起落架。

**如果对整图取最小值，拿到的是"视野里最近的任何东西"**，不是"正前方有没有障碍"。
实测对比（合成场景：正前方 3 m 有障碍、左侧墙 1.2 m、底部 4 行是起落架 0.25 m）：

```
整图取最小          → 0.25 m   ← 被起落架/墙拖低，完全没用
默认 ROI 0.5x0.6    → 3.00 m   ← 正确反映了正前方
收窄 ROI 0.3x0.4    → 3.00 m
```

所以 **ROI 的作用是"只看正前方那一块"**，把无关的像素排除掉。

**矩形在物理空间里对应什么**——一个前向视锥：

| 范围 | 像素 | 对应视场 | 2 m 处覆盖宽度 |
|---|---|---|---|
| 全图 | 640×480 | 73.0° × 58.1° | 2.9 m |
| **默认 ROI** | 320×288 | **36.5° × 34.8°** | **1.32 m** |
| 收窄 ROI | 192×192 | 21.9° × 23.2° | 0.79 m |

即「正前方大约 ±18° 的锥体」，差不多是"机身宽度 + 一点余量"。

**为什么用矩形，而不是圆形/椭圆**

图像在内存里是**逐行连续**的。矩形 = "若干连续的行，每行取一段连续区间"，
正好是双层 `for` 循环，**顺序访问、缓存友好**。
圆形就得对每个像素算 `(u-cx)² + (v-cy)² < r²`，多两次乘法和一次比较，
而且访问不连续 —— 在 30 Hz、9 万像素的量级上没必要。

**为什么"算出来"而不是写死 `160 / 96 / 320 / 288`**

1. **分辨率会变**：深度相机 640×480、下视相机 640×360，写死就错
2. **比例可调**：`roi_width` / `roi_height` 是 ROS 参数，改完重启即可（不用重编译）
3. **保证不退化**：`std::max<size_t>(1, ...)` 让极端参数也不产生空框

**ROI 与过滤条件**

```cpp
const size_t rw = std::max<size_t>(1, width * roi_w_);     // 至少 1 像素
const size_t rh = std::max<size_t>(1, height * roi_h_);
const size_t x0 = (msg->width - rw) / 2;                    // 居中
const size_t y0 = (msg->height - rh) / 2;
...
if (std::isfinite(d) && d > 0.01F && d < max_range_) { best = std::min(best, d); }
```

**这段 ROI 计算算出来的具体矩形**（`(W - rw) / 2` 是整数除法，向下取整）：

| 图像 | 比例 | `rw × rh` | x 范围 | y 范围 | 占全图 |
|---|---|---|---|---|---|
| **640×480**（本项目深度相机） | 0.5 / 0.6 | 320 × 288 | `[160, 480)` | `[96, 384)` | 30.0% |
| 640×480，收窄到 0.3 / 0.4 | 0.3 / 0.4 | 192 × 192 | `[224, 416)` | `[144, 336)` | 12.0% |
| 640×360（下视相机） | 0.5 / 0.6 | 320 × 216 | `[160, 480)` | `[72, 288)` | 30.0% |
| 641×481（奇数尺寸） | 0.5 / 0.6 | 320 × 288 | `[160, 480)` | `[96, 384)` | 29.9% |
| 640×480，`roi_width = 0` | 0 / 0 | 1 × 1 | `[319, 320)` | `[239, 240)` | 0.0% |

两点值得注意：

- **`std::max<size_t>(1, ...)` 是防"空 ROI"**：比例传 0 或极小值时会算出 0，
  那样双层循环一次都不执行，永远不发布。这一行保证**至少 1 个像素**。
- **`(W - rw) / 2` 是整数除法**：奇数差值时向下取整，ROI 会偏左/偏上 1 像素。
  影响可以忽略，但知道总比困惑好。

**双层循环：外层跳行、内层扫行**

```cpp
float best = std::numeric_limits<float>::max();      // 以"最大可能值"为初值
for (size_t v = y0; v < y0 + rh; ++v) {
    const float * row = data + v * row_len;          // 先算出这一行的起点
    for (size_t u = x0; u < x0 + rw; ++u) {
        const float d = row[u];                       // 行内偏移
        if (std::isfinite(d) && d > 0.01F && d < max_range_) best = std::min(best, d);
    }
}
```

- **`row` 指针缓存**：把 `data + v * row_len` 提到内层循环外，避免每个像素都做一次乘法。
  这是这种"逐像素扫描"的标准写法（编译器多半也会优化，但写出来更清楚）。
- **初值取 `FLT_MAX` 而不是 0**：要找**最小**值，初值必须是"极大"。用 0 就永远得 0。
- **`std::min(best, d)` 而不是 `if (d < best) best = d`**：等价，前者少一次分支判断。

**为什么必须按 `row_len` 索引（实测演示）**

拿一张 3×2 的图，每行按 16 字节对齐 → `step = 16`、`row_len = 4`（第 4 个 float 是 padding）：

```
step=16 字节, row_len=4 float, width=3
按 row_len 索引（正确）：5 6 7 8 9 10
按 width  索引（错误）：5 6 7 -nan 8 9
```

按 `width` 索引从第二行开始就整体错位，读到了 padding 里的 `0xFF`（被解释成 `-nan`）。
项目里 640×480 的图 `step` 恰好等于 `width*4`（没有 padding），所以这个 bug
"看起来是对的"——**换成任何需要对齐的宽度就会暴露**。

三种值被排除：`NaN`（无数据）、`<= 0.01`（无效）、`>= max_range`（太远不要）。
若整块 ROI 没有一个有效值 → **直接 return，不发布**。前端因此读到的还是上一次的值，
超过 2 s 就变成 `-1`（`shared_state::obstacleDistanceLocked` 里判的）。

**于是有个现象要解释清楚**：立体相机有近端盲区（`StereoOV7251` 是 0.2 m），
飞机停在地面时中央 ROI 里的最近距离恒等于 **0.20 m** —— 那不是 bug，是量程下限。
HUD 和录制视频里"最近 0.20 m"就是这么来的。

#### 怎么被调用（三层：launch → 可执行 → 话题）

```
flight perc
   └─ ros2 launch flight_perception_cpp perception_cpp.launch.py
        └─ Node(executable='min_depth', parameters=[{'image_topic': ...}])
             └─ ros2 run flight_perception_cpp min_depth
                  └─ 可执行文件：install/flight_perception_cpp/lib/flight_perception_cpp/min_depth
```

`fly.sh` 的 `perc` 子命令在调 launch 之前，**先检查 `/depth_camera` 有没有人发**：

```bash
if ! timeout 5 ros2 topic info /depth_camera | grep -qE "Publisher count: [1-9]"; then
    tmux new-session -d -s depthbridge "... parameter_bridge /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image"
    sleep 3
fi
exec ros2 launch flight_perception_cpp perception_cpp.launch.py
```

为什么要这样：`gz` 自己也跑 DDS，`ros2 topic list` 里**能看到** `/depth_camera`，
但可能 `Publisher count: 0`（没人转发）。**判断"有没有数据"必须看 publisher 数，不能只看话题在不在。**

单独跑（不经过 launch）：

```bash
ros2 run flight_perception_cpp min_depth --ros-args \
  -p image_topic:=/depth_camera -p roi_width:=0.3 -p max_range:=5.0
```

> ⚠️ launch 文件只暴露了 `image_topic` 一个参数，`max_range` / `roi_width` /
> `roi_height` 得用 `ros2 run --ros-args -p` 才改得到。走 launch 又想要这些参数，
> 得先给 launch 加 `DeclareLaunchArgument`。

#### 实测：五个场景的输出（真跑出来的）

启动 `min_depth`，用合成的 640×480 `32FC1` 图发布到 `/fake_depth`，
再用可靠订阅者（与 `main.cpp` 同 QoS）读 `/obstacle_distance`：

| 场景 | 期望 | 实收 |
|---|---|---|
| ROI 内 2.5 m / ROI 外 0.8 m / 有 NaN 区 | 2.5 | **2.5** ✓ |
| 近物只在 ROI 外（0.8 m），ROI 内全 15 m | 15.0 | **15.0** ✓（0.8 被忽略） |
| 全图 25 m（> `max_range`=20） | 无输出 | **0 条** ✓ |
| 全图 1.0 m | 1.0 | **1.0** ✓ |
| 全图 0.005 m（< 0.01 下限） | 无输出 | **0 条** ✓ |

**性能**：ROI = 320×288 = 92160 像素，单帧扫描平均 **81.5 µs**（约 12000 帧/秒），
相机 30 Hz 时占用约 0.24% 单核 —— 直接裸循环完全够用，这也是"不用 OpenCV"的另一个理由。

#### 两个实测踩到的坑（不是代码 bug，但会影响你验证）

1. **`min_depth` 的输出日志被 `RCLCPP_INFO_THROTTLE(..., 500, ...)` 限流**
   到每秒 2 行。所以"日志里没看到"不等于"没发布" —— 要确认得订阅 `/obstacle_distance`。
2. **短时间内反复启停发布者时，前面几帧会丢**。
   `best_effort` + `KeepLast(1)` 本来就不保证送达，DDS 发现也要时间（约 1~3 s）。
   用合成数据测试时，发布端要**先起、多等几秒、连续发**，否则会误判成"节点没工作"。
   生产环境里 gz bridge 是 30 Hz 连续发，没有这个问题。

> 补充一个操作经验：把 `min_depth` 用 `./xxx/min_depth > /tmp/x.log 2>&1 &` 的方式
> 从脚本里起，日志是**块缓冲**的，看起来像"没输出"。放进 tmux 直接跑（`fly.sh` 的做法）
> 能立刻看到。
#### `src/flight_perception_cpp/src/optical_flow_logger.cpp`（101 行）

```cpp
class OpticalFlowLogger : public rclcpp::Node;
int main(int argc, char ** argv);
```

**数据流**：`/fmu/out/vehicle_optical_flow` + `/fmu/out/vehicle_local_position` → CSV（默认 `/tmp/optical_flow_cpp.csv`）

CSV 列：`t,flow_x,flow_y,dt,distance,quality,ekf_vx,ekf_vy,flow_vx,flow_vy`

**构造函数里那 6 行在干什么（打开文件 + 写表头）**

```cpp
path_ = declare_parameter<std::string>("csv_path", "/tmp/optical_flow_cpp.csv");
file_.open(path_, std::ios::out | std::ios::trunc);
if (file_.is_open()) {
    file_ << "t,flow_x,flow_y,dt,distance,quality,ekf_vx,ekf_vy,flow_vx,flow_vy\n";
} else {
    RCLCPP_ERROR(get_logger(), "cannot open %s", path_.c_str());
}
```

| 行 | 作用 |
|---|---|
| `declare_parameter` | 输出路径做成 ROS 参数，默认 `/tmp/optical_flow_cpp.csv`，可用 `-p csv_path:=...` 改 |
| `std::ios::out` | 以**写**方式打开（`ofstream` 默认已含，写出来是明确意图） |
| `std::ios::trunc` | 打开时**清空已有内容** —— 每次跑都是新的一份，不会和上次混在一起 |
| `file_ << "t,flow_x,...\n"` | 立刻写表头。文件刚 trunc 完是空的，先写表头能保证"有文件就有表头"，下游读 CSV 不会把第一行数据当表头 |
| `else RCLCPP_ERROR` | 打不开只**报错、不退出**。节点继续跑（订阅照建），只是再也不写数据 |

**实测这两个行为**：

```
# 启动前先塞 22 字节脏数据
启动前: 22 字节
启动后:  0 字节        ← trunc 生效，旧内容清掉了
```

```
# 表头是"延迟落盘"的（ofstream 带缓冲）
运行中:      0 字节     ← 表头还在缓冲区里
SIGINT 退出: 66 字节    ← ~OpticalFlowLogger() 里 file_.flush() 才写出去
           t,flow_x,flow_y,dt,distance,quality,ekf_vx,ekf_vy,flow_vx,flow_vy
```

**两个由此而来的注意事项**：

1. **数据是攒在缓冲区里的**，只有缓冲区满（通常 4 KB）或进程**优雅退出**
   （析构里 `file_.flush()`）才真正落盘。所以 `kill -9` 会**丢掉最后一段数据**；
   想边跑边看，得等缓冲区自然写满。
2. **打不开文件不会让节点退出**（只 `RCLCPP_ERROR`）。所以 CSV 一直是空的，
   先确认路径可写，而不是怀疑话题没数据。

> 另外：`trunc` 意味着**上一次的记录会被覆盖**。要留历史得先备份，
> 或者每次传不同的 `csv_path`。

**核心那三行——它想验证的物理关系 `v ≈ ω · h`**

```cpp
const double dt = msg->integration_timespan_us * 1e-6;
flow_vx = flow_x / dt * distance;      // (rad / s) * m = m/s
```

- `flow_x` 是**积分角度（弧度）**，除以 `dt` 才是角速度（rad/s）
- 再乘距离 `distance` 就得到等效线速度（m/s）
- 这正是光流测速的原理：**角速度 × 高度 = 线速度**

> ⚠️ **本项目里 `flow_vx/flow_vy` 这两列会恒为 0**：PX4 的
> `vehicle_optical_flow.distance_m` 在 SITL 里实测一直是 `NaN`
>（`ros2 topic echo` 直接显示 `distance_m: .nan`），
> 而代码里有 `if (dt > 0.0 && std::isfinite(distance))` 的判断。
> 所以**不是工具坏了**，是缺测距源。要看真实效果得给 PX4 接一个距离传感器。

**为什么两个回调之间要加锁**

`onPosition` 写 `vx_/vy_`，`onFlow` 读它们再写文件。理论上共享了可变状态。

但注意 `main()` 里是 `rclcpp::spin(单节点)`——**默认单线程执行器，回调是串行的**，
所以严格说今天这个锁不是必需的。它属于**防御性写法**：将来若换成
`MultiThreadedExecutor`，或者加了 `callback_group`，这把锁就是必要的。

（对比 `shared_state.hpp` 里的两把锁——那里是真的多线程，锁是刚需。）

**和 `flight_ui_cpp` 的关系：零耦合。** 两个节点各自 `main()`，只通过 ROS 话题通信，
可以单独 `ros2 run` 起停，删掉也不影响后端。

---

### 3.7 录制脚本 `scripts/demo_show.py` 怎么"飞"飞机

这是项目里唯一一处**自动飞**的代码（`fly.sh` 是手动单步命令）。
它不直接连 PX4，而是**冒充浏览器走 WebSocket** —— 这是理解它的第一要点。

#### 调用链：为什么叫"远程"

```
demo_show.py                     （本机 Python，25 Hz 渲染 + 50 Hz 发杆）
   │  WebSocket → 127.0.0.1:8000/ws   ← 和浏览器完全一样的协议
   ▼
rc_server（C++ 后端）             ← 走的就是网页那条路，没有特殊通道
   │  /fmu/in/manual_control_input（50 Hz）
   ▼
PX4 SITL → Gazebo
```

所以"远程操控"在这里的实现方式是：**脚本扮演了一个浏览器的角色**。
好处是它自动继承了网页那套安全机制（deadman 使能、控制权租约、ack 回执）；
代价是它和浏览器**会抢控制权**，所以脚本里专门写了"让贤"逻辑（见 3.7.5）。

#### 3.7.0 文件在哪、怎么调用

**位置**：`scripts/demo_show.py`（997 行，43 KB）

**正常路径**不是直接跑它，而是由 `run_show.sh` 调用（`run_show.sh:163`）：

```bash
flight record                    # = ./scripts/run_show.sh --record
# run_show.sh 先起 agent / PX4 / gz / rc_server / 五路相机桥 / Xephyr 浏览器，
# 最后才：
DISPLAY=:1 python3 -u scripts/demo_show.py \
    --record artifacts/flight_show_final.mp4 --grab ":1.0+0,0" \
    --sensors depth,flow --encoder auto
```

**手动路径**（调参时用）：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 scripts/demo_show.py --dry-run              # 只飞不录
python3 scripts/demo_show.py --record /tmp/t.mp4    # 录一条
python3 scripts/demo_show.py --record /tmp/t.mp4 --frames 20 --sensors none
```

**七个命令行参数**（`demo_show.py:622` 起）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `--record` | 空 | 输出 mp4 路径；不给就只飞不录 |
| `--dry-run` | — | 不录、不渲染，纯调控制 |
| `--grab` | `:1.0+0,0` | 抓哪个 X 屏的网页区域（`:1` = Xephyr 虚拟屏） |
| `--encoder` | `auto` | `auto`/`nvenc`/`x264` |
| `--duration` | 180 | 秒 |
| `--sensors` | `depth,flow` | 画中画开关：`none`/`depth`/`flow`/`depth,flow` |
| `--frames` | 0 | 渲染 N 帧后退出（测试用） |

**配套的四个脚本**（同目录）：

| 文件 | 作用 |
|---|---|
| `run_show.sh` | **调用者**：起全部服务，最后传参给 `demo_show.py` |
| `pose_writer.py` | 写 Gazebo 真值到 `/tmp/cam_target.json`，`demo_show` 读它画航迹 |
| `cam_follow.py` | 每帧搬跟拍/俯视相机的位姿 |
| `rerender_panel.py` | **`from demo_show import FPS, PANEL_H, W, Controller, Overlay`** —— 用日志重渲染下半屏，不必重飞 |

> ⚠️ 因为 `rerender_panel.py` 直接 import 了 `demo_show` 的 `Controller` / `Overlay`，
> **改这两个类的名字或签名会连带影响它**。

**文件内部结构**（997 行）：

| 行范围 | 内容 |
|---|---|
| 1-67 | 文件头说明 + import + 字体/常量 |
| 68-150 | `WSClient` —— 最小 WebSocket 客户端（握手 / 文本帧收发） |
| 151-205 | `FlightState` —— ROS 侧反馈（位置/姿态/光流） |
| 206-216 | `CAM_TOPICS` —— 五路相机的话题名表 |
| 217-233 | `read_true_pose()` —— 读 Gazebo 真值 |
| 234-307 | `CameraFeed` —— 相机订阅 + 深度伪彩色 |
| 308-366 | `Controller` —— 串级控制律 |
| 367-556 | `Overlay` —— 下半屏面板（手柄/姿态仪/航迹/遥测） |
| 557-581 | `build_mission()` + `figure8()` —— 10 个阶段的任务表 |
| 588-591 | `_FramesDone` —— 达到 `--frames` 时跳出整场的异常 |
| 592-619 | `crop_center()` |
| 620-997 | `main()` —— 装配 + 50 Hz 发送线程 + 主循环 + ffmpeg |

#### 3.7.1 两个线程，各管一件事

| 线程 | 频率 | 干什么 |
|---|---|---|
| `sender()`（daemon 线程） | **50 Hz**（`time.sleep(0.02)`） | 把共享字典 `sp` 里的杆量通过 WS 发出去 |
| 主循环 | ~250 Hz（`sleep(0.004)`） | 算目标点 → 调 `Controller` → 算杆量 → 写进 `sp`；每 40 ms 渲染一帧 |

**为什么要拆开**：渲染一帧 1920×1080 要几十毫秒，如果杆量和渲染串在一起发，
PX4 那边就会收到忽快忽慢的杆量（它要求 ≥10 Hz 稳定）。拆开后**发杆永远 50 Hz，不受渲染拖累**。

共享杆量用一个 dict + `threading.Lock`：

```python
sp = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': -1.0}
def apply(roll, pitch, yaw, throttle):
    with sp_lock: sp.update(roll=roll, pitch=pitch, yaw=yaw, throttle=throttle)

# sender 里
with sp_lock: s = dict(sp)      # 加锁只为了取一份快照
ws.sticks(s['roll'], s['pitch'], s['yaw'], s['throttle'])
```

`with sp_lock: s = dict(sp)` —— **加锁只保护"拷贝那一瞬间"**，发 WS 在锁外，
避免网络慢的时候把主循环卡住。和 `shared_state.hpp` 里"锁内取快照、锁外发布"是同一个思路。

#### 3.7.2 控制律：三行看懂

`Controller.sticks()` 是**串级**的，不是简单的"误差乘个系数"：

```
① 位置误差 (tx-x, ty-y)  ×KP_XY → 期望速度 v_d（限幅 VMAX=1.5 m/s）
② 世界系 v_d  →  换算到机体系（用 heading 做旋转）
③ (v_d - v_measured)  →  前馈 FF·v_d + 比例 KV·e + 积分 KI·∫e  →  俯仰/横滚杆
④ 航向误差 → 偏航杆（KYAW）
⑤ 高度误差 → 油门杆（KP_ALT）
```

| 参数 | 值 | 作用 |
|---|---|---|
| `KP_XY` | 0.7 | 位置误差 → 期望速度（1/s） |
| `VMAX` | 1.5 | 期望速度上限（m/s），小场地不需要快 |
| `FF` | 0.16 | **速度前馈**：满杆约 5 m/s，留余量 |
| `KV` | 0.16 | 速度误差 → 杆量 |
| `KI` | 0.045 | 积分，消除稳态误差（积分项限幅 ±3） |
| `KP_ALT` | 0.55 | 高度误差 → 油门杆 |
| `KYAW` | 0.85 | 航向误差 → 偏航杆 |

**为什么要有 `FF` 前馈**（代码注释写了）：只有反馈时，杆量永远"追不上"期望速度，
画面上就会看到"目标点在动、手柄几乎不动"—— 这是上一版最大的观感问题。
加上前馈后，杆量立刻到位，反馈只负责修残差。

**为什么必须先转机体系**：ALTCTL 的横滚/俯仰杆是**机体系**姿态角，
而目标点在世界系。不换算的话，飞机转了个身，"前进"就变成"侧飞"了。

**yaw 在 ALTCTL 里是角速率指令**，不是角度指令 —— 所以 `spin` 阶段直接给常数值：

```python
if kind == 'spin':
    yaw = spin_yaw      # 恒定的偏航角速率 → 机体连续自转
```

#### 3.7.3 任务编排：目标点必须一直动

`build_mission()` 是一张阶段表，10 个阶段共 180 s：

| 阶段 | 时长 | `kind` | 干什么 |
|---|---|---|---|
| 系统就绪 | 8 s | `idle` | 杆量全 0、油门 −1（等解锁条件） |
| 解锁 ARM | 4 s | `arm` | 每 2 s 发一次 `{'type':'arm'}` 直到 armed |
| 起飞 | 14 s | `alt` | 目标高度在 1.0~1.8 之间按正弦起伏，**整整 1.6 个周期** |
| 前进 | 22 s | `fwd` | `tx = base_x + 1.2·sin(...)`，来回 3.4 个周期 |
| 左转 | 20 s | `spin` | 恒定偏航杆 0.30，连续自转约两圈 |
| 俯冲 & 攀升 | 22 s | `alt` | 0.9~2.1 之间起伏，2 个周期 |
| 绕 8 字 | 46 s | `fig8` | Gerono 双纽线：`x=A sin u, y=A sin u cos u`，同时机头跟着速度方向转 |
| 返回原点 | 16 s | `goto` | 螺旋进近：半径 2.6→0 收缩 + 绕一圈 |
| 降落 | 24 s | `descend` | `talt` 每秒降 0.35 m，最后压到 −0.05 |
| 任务完成 | 4 s | `land` | 发 `{'type':'land'}`，杆量清零、油门压到底 |

**核心设计（文件注释点明了）**：

> 每段的**目标都在持续移动**，不是"飞到点就停"。否则控制器很快回中，
> 画面上手柄一动不动 —— 这是上一版最大的问题。

也就是说：**为了让视频"看起来在飞"，目标点被设计成永不静止的**。
这是个"演示优先"的取舍，不是控制理论上的最优轨迹。

#### 3.7.4 四个安全限制

| 限制 | 代码 | 为什么 |
|---|---|---|
| **活动范围** | `ctrl.xlim = (-2.6, 2.4)`、`ylim = (-2.6, 2.6)` | 光流网格只有 ±5.9 m，飞出去就没有光流定位了（实测过一次飞出 9 m 全是白屏） |
| **杆量限幅** | `clamp(..., -0.45, 0.45)`（俯仰/横滚）、`(-0.7, 0.8)`（油门） | 别打满，留修正余量 |
| **姿态异常中止** | `if abs(st.roll) > 1.0 or abs(st.pitch) > 1.0: 杆量归零 + 退出码 3` | 翻转/撞机了就立刻撒手，别继续录 |
| **解锁/降落阶段强制油门最低** | `if kind in ('idle','arm'): thr = -1.0` | PX4 解锁硬性要求油门最低；降落也要压到底才落得下去 |

#### 3.7.5 "让贤"：脚本和真人抢杆怎么办

```python
def sender():
    while running:
        if ws.telemetry.get('controller') == 'other':
            if not yielded['v']:
                yielded['v'] = True
                print('控制权已被网页接管，脚本停止发杆', file=sys.stderr)
            time.sleep(0.05)
            continue
        ...
```

只要遥测里 `controller == 'other'`（说明有别人在操控），脚本**立刻停手**。
这样录制过程中真人打开网页就能随时接管，脚本不会和他抢。

> 这正是 `shared_state.hpp` 里 `controller` 字段的用途之一 —— 它不只是给网页显示
> "本机操控 / 外部操控"，还被自动化脚本当作"该不该让位"的信号。

#### 3.7.6 反馈用什么、不用什么

| 用途 | 数据源 | 为什么 |
|---|---|---|
| **控制反馈**（位置/速度/姿态） | PX4 `vehicle_local_position` + `vehicle_attitude` | 控制器要闭环，必须用飞控自己认得的坐标系 |
| **航迹显示**（画面上的轨迹） | Gazebo 真值 `/tmp/cam_target.json` | PX4 无 GPS 时 XY 估计会缩水到真值的 1/3（实测 2.98 m vs 9.18 m），画出来是假的 |
| **相机跟拍** | 同上真值 | 要让画面里真的看得见飞机 |

代码注释明确写了这个取舍：
`曾试过换 Gazebo 真实位姿做闭环，但 PX4 的 NED 局部系与 Gazebo 世界系在对齐上会漂，混用会让控制器发散（实测飞出 100+ m）`。

**即：控制用 PX4 的估计（内部一致），显示用真值（对外真实）。**

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

**六种组合的完整结果**（按 `main.cpp:147-156` 的公式逐项算出来的）：

假设网页上摇杆停在 `roll=0.3 pitch=0.4 yaw=0.5 throttle=0.6`：

| 场景 | `armed` | `msg.valid` | roll | pitch | yaw | **thr** |
|---|---|---|---|---|---|---|
| 没网页连着 | 0 | **0** | 0 | 0 | 0 | **−1.0** |
| 网页开着 · 没按使能 | 0 | 1 | 0 | 0 | 0 | **−1.0** |
| 网页开着 · 按着使能 | 0 | 1 | 0.3 | 0.4 | 0.5 | 0.6 |
| 没网页连着 | 1 | **0** | 0 | 0 | 0 | **0.0** |
| 网页开着 · 没按使能 | 1 | 1 | 0 | 0 | 0 | **0.0** |
| 网页开着 · 按着使能 | 1 | 1 | 0.3 | 0.4 | 0.5 | 0.6 |

三条规律一眼可见：

1. **只有按着使能，杆量才传下去**；没按就三轴全 0。
2. **油门兜底看 `armed`**：未解锁给 −1（满足解锁条件），已解锁给 0（中位悬停）。
3. **`msg.valid` 只看 `any_client`**，跟按不按使能无关 —— 所以第 2 行和第 5 行
   是 `valid=1` 但杆量全 0：**"有遥控器，但手没在杆上"**。PX4 看到这个组合
   不会触发 failsafe，但也不会让飞机因松手而暴走。

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

### 5.8 `read()` 里三个值得学的写法

读 `control_lease.cpp::read()` 时会遇到几个 C++ 惯用法，认识一次以后到处都能看懂。

#### 写法一：链式提取 `ss >> a >> b >> c >> d`

```cpp
std::istringstream ss(line);
if (!(ss >> e.owner >> e.prio >> e.pid >> e.ts_ms)) {
    return e;                        // 格式不对 = 当成没有
}
```

`>>` 表达式**返回流本身**，而流能隐式转成 `bool`（成功 = true）。所以这是一条
**链式提取**：从左往右逐个拆，**中途任何一个拆不出来，整条链就返回 false**。
拆的规则是"跳过空白，取一个词"，所以它天然按空格分字段。

这比 `sscanf` 安全：类型不对（比如 `prio` 位置是 `abc`）也会失败，不会写出垃圾值。

#### 写法二：默认 `valid = false` —— 先假设失败

```cpp
struct Entry {
    std::string owner;
    int prio = 0;
    int pid = 0;
    long long ts_ms = 0;
    bool valid = false;      // ← 默认就是无效
};
```

函数里有**三个提前 return**（打不开文件、解析失败），它们都直接返回 `e`，
此时 `valid` 仍是 `false`；只有**走到函数最后一行**才 `e.valid = true`。

这种"先假设失败、全部通过才置成功"的写法，比"先置 true、出错再改 false"更难写错 ——
以后新增一个失败分支时，忘记置 false 也不会出 bug。

#### 写法三：只看第一行、多余字段不报错（向前兼容）

`std::getline` 只调了一次，所以**只看第一行**；链式提取拆完 4 个就停，
所以**末尾多出来的字段被忽略**。将来想在文件末尾加新字段（比如心跳计数），
老版本代码不用改也不会崩。

#### 实测：各种坏内容 `read()` 返回什么

| 文件内容 | `valid` | 解析结果 |
|---|---|---|
| `web 10 1234 1793197679795` | ✅ 1 | owner=web prio=10 pid=1234 ts=1793… |
| （空文件） | ❌ 0 | 全空 |
| `web 10 12`（写到一半） | ❌ 0 | owner=web prio=10 pid=12 ts=**0** |
| `web 10` | ❌ 0 | owner=web prio=10 pid=0 ts=0 |
| `web 10 1234 999 额外 东西` | ✅ 1 | 多余字段被忽略 |
| `web 10 1234 999` + 第二行垃圾 | ✅ 1 | 只看第一行 |
| `␣10 1234 999`（owner 位是空格） | ❌ 0 | 拆歪成 owner=`10` prio=1234 pid=999 ts=0 |

注意第三行：**"写到一半"的文件确实会被判成无效** —— 这正是 `writeEntry` 必须用
原子替换的直接原因。

#### `valid = false` 之后会发生什么（为什么要"当成没有"）

```cpp
const Entry e = read();
const bool free_slot =
    !e.valid                                   // ① 读不出来 → 认为"没人占"
    || (nowMs() - e.ts_ms) > kStaleSec * 1000  // ② 超 2 秒没刷新 → 也认为没人占
    || !pidAlive(e.pid);                       // ③ 持有者进程死了 → 也认为没人占
if (free_slot || e.owner == me || e.prio < prio) {
    writeEntry(me, prio, pid);                 // 抢到手
    return true;
}
return false;                                  // 别人占着，我放弃
```

`!e.valid` 的语义是 **"我不知道谁在占 → 那就当没人占 → 我可以试试"**。

这是**乐观策略**，因为两种误判代价不同：

- 误判成"没人占" → 两端都去抢，最后只是重写一次文件，按 prio 分出胜负
- 误判成"有人在占" → 该发杆的人不敢发，飞机直接失控

而且最坏情况还有 2 秒过期兜底：文件彻底坏掉，最多 2 秒也会被重新写。

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
