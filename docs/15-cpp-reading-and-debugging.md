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

回给前端的 `ack` 不是客户端发来的消息，是服务端下发的结果回执（前端靠 `type=="ack"` 在右下角打印）。

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

**ROI 与过滤条件**

```cpp
const size_t rw = std::max<size_t>(1, width * roi_w_);     // 至少 1 像素
const size_t x0 = (msg->width - rw) / 2;                    // 居中
...
if (std::isfinite(d) && d > 0.01F && d < max_range_) { best = std::min(best, d); }
```

三种值被排除：`NaN`（无数据）、`<= 0.01`（无效）、`>= max_range`（太远不要）。
若整块 ROI 没有一个有效值 → **直接 return，不发布**。前端因此读到的还是上一次的值，
超过 2 s 就变成 `-1`（`shared_state::obstacleDistanceLocked` 里判的）。

**于是有个现象要解释清楚**：立体相机有近端盲区（`StereoOV7251` 是 0.2 m），
飞机停在地面时中央 ROI 里的最近距离恒等于 **0.20 m** —— 那不是 bug，是量程下限。
HUD 和录制视频里"最近 0.20 m"就是这么来的。

#### `src/flight_perception_cpp/src/optical_flow_logger.cpp`（101 行）

```cpp
class OpticalFlowLogger : public rclcpp::Node;
int main(int argc, char ** argv);
```

**数据流**：`/fmu/out/vehicle_optical_flow` + `/fmu/out/vehicle_local_position` → CSV（默认 `/tmp/optical_flow_cpp.csv`）

CSV 列：`t,flow_x,flow_y,dt,distance,quality,ekf_vx,ekf_vy,flow_vx,flow_vy`

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
