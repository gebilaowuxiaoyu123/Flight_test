# 14 · 终端飞飞机

把所有 ROS 话题细节都包进了 `scripts/fly.sh`，你只敲动作词。

## 起 / 停

先装一次短命令（写入 `~/.bashrc`）：

```bash
./scripts/install_aliases.sh     # 装完 unalias flight remote 2>/dev/null; source ~/.bashrc
```

### 四个终端（推荐，日志就在眼前）

```bash
flight link      # ① 通信连接：MicroXRCEAgent + 节点清单 + 话题频率
flight sim       # ② 仿真：PX4 + Gazebo 3D 窗口
flight remote    # ③ 网页遥控：rc_server 日志 + 自动开浏览器
flight views     # ④ 三视角：第一人称 / 第三人称 / 俯视
flight perc      # ⑤ 感知：min_depth(/obstacle_distance) + 光流 CSV（会自动桥深度图）
```

每条都是前台进程，Ctrl-C 只停这一个。顺序 ①→②→③→④。

**必须开四个终端吗？** 不一定，两种都行：

| 方式 | 命令 | 日志 |
|---|---|---|
| 四终端（推荐） | 上面四条，一条一个终端 | 每条一个终端，谁出问题一眼看出 |
| 单终端 | `flight start` → `flight gui` → `flight views &` | 全挤在一个终端，`flight logs sitl` 另看 |

**Tab 补全**：装好短命令后直接按 Tab——

```bash
flight <Tab>          # link sim remote views perc clean start stop status watch …
flight <Tab><Tab>     # 顺便把 5 个控制台命令的用途列出来
flight logs <Tab>     # agent sitl stick
flight mode <Tab>     # MANUAL ALTCTL POSCTL AUTO ACRO OFFBOARD STABILIZED
fly <Tab>             # 同上（fly 是 fly.sh 的直连入口）
```

实现在 `scripts/flight_completion.bash`，`flight_env.sh` 自动 source；纯 bash 方案，不依赖 bash-completion 包。

`flight`（不带词）= `start` 的前台版：agent + PX4 + 摇杆守护 + 后端挤在一个终端，
Ctrl-C 一起停；`start` 是同一个东西丢进 tmux，终端立刻还给你。

### 一把梭 / 全部表格

之后：

| 命令 | 等价于 | 作用 |
|---|---|---|
| `flight` | `ros2 launch flight_ui flight.launch.py` | 起整条链路（DDS 桥 + PX4/Gazebo + 摇杆守护 + Web 后端） |
| `flight start` | `./scripts/fly.sh start` | 同上一行，但后台跑（tmux 托管），Ctrl-C 不会全停 |
| `flight <动作>` | `./scripts/fly.sh <动作>` | 21 个动作随便接：`link` `sim` `remote` `views` `status` `takeoff` `fwd 2` … |
| `flight clean` | `./scripts/clean_logs.sh` | 清 PX4 黑匣子日志（掉盘元凶） |
| `remote` | `./scripts/fly.sh remote` | 同 ③，起 Web 后端并开浏览器 |
| `fly <动作>` | `./scripts/fly.sh <动作>` | 终端飞 |
| `tele` | `./scripts/fly.sh watch` | 终端实时看姿态 |
| `flystatus` | `./scripts/fly.sh status` | 一行状态 |

`flight` 不带参数是前台进程，`Ctrl-C` 就全停；带 `<动作>` 时转给 `fly.sh`，后台 tmux 托管。
不想用短命令也可以直接跑 `./scripts/fly.sh start` / `stop`。

定义都在 `scripts/flight_env.sh`，`~/.bashrc` 里只留一行 `source`，改命令不用再动 `.bashrc`。

## 看（终端里实时显示姿态）

```bash
tele            # 实时刷新：模式 / ARMED / 高度 / 横滚 / 俯仰 / 航向 / 速度
flystatus       # 打印一行就退出（约 1 秒，不依赖任何缓存进程）
```

`watch` 的样子（原地刷新，不刷屏）：

```
┌─ 飞行姿态 ─────────────────────────────────────────────┐
│ 模式  ALTCTL        ● ARMED        高度   1.63 m       │
│ 横滚   +2.3°  [----------O----------]                  │
│ 俯仰   -1.8°  [----------O----------]                  │
│ 航向   128°   水平速度 0.42 m/s   垂直 +0.03 m/s       │
└────────────────────────────────────────────────────────┘
```

## 后台在跑什么

`fly.sh start` 会起 5 个 tmux 会话，出问题可以进去看日志：

| 会话 | 内容 |
|---|---|
| `agent` | MicroXRCEAgent（PX4 ↔ ROS 2 的 DDS 桥） |
| `sitl` | PX4 + Gazebo |
| `stick` | 摇杆守护（50 Hz 补发） |

`status` / `watch` 都是自己起一个短命 ROS 节点直接读话题，不依赖常驻缓存，
所以没有"缓存过期"这类问题。

```bash
tmux attach -t tele      # 看遥测守护有没有在收数据；Ctrl-b d 退出
```

## 飞

```bash
fly mode ALTCTL     # 切模式（MANUAL/ALTCTL/POSCTL/AUTO/ACRO/OFFBOARD）
fly takeoff         # 解锁并爬到 1.2 m
fly fwd 3           # 前进 3 秒
fly up 2            # 爬升 2 秒
fly left 2          # 左移 2 秒
fly spin 2          # 左转 2 秒
fly land            # 降落并上锁（用 PX4 的 NAV_LAND，低空 force disarm 兜底）
```

全部子命令：`mode` `arm` `takeoff` `up` `down` `fwd` `back` `left` `right` `spin` `land` `status` `watch` `start` `stop`

不写秒数默认 2 秒。`takeoff` 默认爬到 1.2 m，可以写 `takeoff 2.0`。

## 它内部到底发了什么

`fly.sh` 只是把这两个话题的命令包了一层：

| 动作 | ROS 话题 |
|---|---|
| 切模式 / 解锁 / 上锁 | `/fmu/in/vehicle_command`（`176`=切模式、`400`=解锁上锁） |
| 爬升 / 前进 / 转向 | `/fmu/in/manual_control_input`，50 Hz 连续发 |

想看原始命令：`bash -x ./scripts/fly.sh fwd 1`

## 三个设计上的坑（已经处理，但要知道）

**1. 摇杆必须 50 Hz 连续发**
PX4 超过 0.5 s 收不到手动控制就判失效。所以有个常驻的 `stick_keeper.py`，
`fly.sh` 只改 `/tmp/stick.json` 里那 4 个数。

**2. 杆量文件 2 秒不更新就回中悬停**
否则操控脚本被 Ctrl-C 打断，最后一条杆量会一直发下去 ——
实测油门 0.45 被留在文件里，飞机一口气爬到 17 m。现在过期就回中，最多悬停。

**3. `ros2 topic pub --once` 会丢包**
每次调用都要新建 ROS 节点，DDS 发现没完成时消息直接丢。
所以 `pub_cmd` 改成连发 3 次，解锁/上锁还带重试。

**4. 降落时不能再打杆**
PX4 检测到杆量变动会从 `AUTO_LAND` 踢回手动模式，结果就是降不下来。
所以 `land` 会先把杆量置为 `valid: false`（暂停发杆），让 PX4 自己降。

**5. 无 GPS 时 PX4 的落地检测常常不触发**
高度估计有几十厘米漂移，PX4 以为还在空中，永远不自动上锁。
所以低空（<0.6 m）时用 force disarm（`param2 = 21196`）兜底。

## 为什么是 ALTCTL 不是 OFFBOARD

OFFBOARD + `TrajectorySetpoint` 是 PX4 的自主飞行接口，本机用不了：

```
xy_valid: false                          ← 无 GPS，光流给不出水平位置
WARN [mc_pos_control] invalid setpoints  ← PX4 直接拒
```

ALTCTL 的横滚/俯仰杆是**姿态角指令**，不依赖位置估计，所以能飞。

## 和后端不要同时飞

`rc_server`（网页后端）也会以 50 Hz 往同一个话题发杆量，两边会打架。
用终端飞之前先 `./scripts/fly.sh stop`，或者用 `fly.sh start`（它只起后端不抢杆）。

## 短命令报错怎么修

**`source ~/.bashrc` 报 `未预期的记号 "(" 附近有语法错误`**

老版本把 `flight` 定义成 alias。bash 解析 `flight() {` 时会先把函数名当命令名做 alias 展开，
变成 `ros2 launch ...() {` → 语法错误，而且**后面的定义全不执行**，于是 `flight start` 一直报
`malformed launch argument 'start'`，每次都失败。

修（`flight_env.sh` 开头已经加了 `unalias`，重装即可自愈）：

```bash
unalias flight remote 2>/dev/null; source ~/.bashrc
```

**改完命令为什么不生效**

`.bashrc` 只在开新终端时才读。已经开着的终端要手动 `source ~/.bashrc`。

## 磁盘：为什么会剩 3.9 G

PX4 SITL 每次启动都以 100 Hz 写黑匣子 `.ulg` 到
`PX4-Autopilot/build/px4_sitl_default/rootfs/log/`，只要不关就一直涨。
实测 2026-10-25 一整晚写了 **9.6 GB**，累到 18 GB，根分区只剩 3.7 G。

```bash
flight clean           # 删 24 小时前的
flight clean --all     # 全删
df -h /                # 确认
```

现在 `run_sitl_combined.sh` 默认 `PX4_PARAM_SDLOG_MODE=-1`（关闭黑匣子），
不会再涨。真要留日志：`PX4_PARAM_SDLOG_MODE=0 flight sim`。

## 网页遥控：为什么"点了起飞没反应"

四个原因，都已修好，遇到问题时按这四条自查：

**0. 没起通信链路（最常见）**
`flight sim` 只起 PX4 + Gazebo，`flight remote` 只起后端 —— 它们都不含
`MicroXRCEAgent`（① 通信连接）。没有它 `/fmu/*` 一条话题都没有，
网页上点 ARM/TAKEOFF 就是"点了没反应"。
现在 `flight sim` / `flight remote` 会自动兜底把 agent 拉起来；
后端发现命令到达时遥测已断超过 1.5 s，会直接回
`没有遥测：ROS 链路没通，先跑 flight link`，不再假装成功。


**1. 起飞不用按使能，但打杆必须按住「按住使能」**
- 起飞/解锁：后端会自动把杆量置成"回中 + 油门最低"，直接飞 `NAV_TAKEOFF`
- 打杆：不按「按住使能」时杆量被强制回中（安全设计），飞机会悬停不理会你

**2. 网页和终端摇杆守护同时发杆量（已用租约解决）**
两者都以 50 Hz 往 `/fmu/in/manual_control_input` 发，同时发会互相覆盖，
表现为"网页打杆飞机不动"。

现在改成**控制权租约**（`/tmp/flight_control.lease`）：

| 谁 | 优先级 | 什么时候持有 |
|---|---|---|
| `web`（rc_server） | 10 | 只要有一个网页连着 |
| `terminal`（stick_keeper） | 5 | 没有网页连着时 |

- 网页一连上就抢占，终端守护自动暂停并打印
  `⚠ 控制权被 web 接管，暂停发杆（网页在操控）`
- 网页关掉后守护 0.5 s 内自动恢复
- `flight status` 会多打一行 `控制权 terminal (5 优先，pid 1234)`
- 进程挂了也不算数：租约带 pid 和时间戳（整数毫秒），2 s 没心跳或 pid 不存在就作废

两个进程可以同时开着，不用再手动 kill 谁。

**3. PX4 必须看到"有效遥控信号"才允许解锁**
后端只在**有网页连着**时才发 `valid=true`；没有网页时 PX4 认为遥控丢失，
会拒绝解锁。所以先开网页再点起飞。

排查命令：

```bash
ros2 topic info /fmu/in/manual_control_input -v   # Publisher count 必须是 1
flight status                                     # 模式/解锁/高度
flight logs sitl                                  # PX4 里的 Preflight Fail 原因
```

## 参数在哪改

后端的可调参数都在 `src/flight_ui_cpp/config/flight_ui.yaml`，改完重启 `flight remote` 生效（不用重新编译）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `port` | 8000 | HTTP / WebSocket 端口 |
| `takeoff_altitude_m` | 2.5 | 网页点"起飞"时的默认高度 |
| `arm_retry_sec` | 10.0 | 解锁重试窗口（PX4 冷启动 EKF 收敛要十几秒） |
| `control_rate_hz` | 50.0 | 杆量发布频率，PX4 要求 ≥ 10 Hz |
| `telemetry_rate_hz` | 1.0 | 终端里那行 `armed=/nav_state=/alt=` 的频率 |

临时覆盖（不改文件）：

```bash
ros2 launch flight_ui remote.launch.py port:=8010
# 或者直接跑，参数用 --ros-args
ros2 run flight_ui_cpp rc_server --ros-args -p takeoff_altitude_m:=1.5
```
