# 项目进度

当前阶段：**P1–P6 已完成并跑通**（光流 + 深度 + Web 遥控器同机联调）
最后更新：2026-10-30 22:45

> 操作手册（从零安装、排错决策树、速查表）：[REBUILD.md](REBUILD.md)

## 断点续做（断线后从这里开始）

1. 打开终端，运行 `./scripts/verify.sh`；`[FAIL]` 必须为 0，`[SKIP]` 表示对应服务未启动。
2. 一键启动：`./scripts/run_demo.sh`（tmux 会话：agent/sitl/bridge/depth/flowlog/ui）。
3. 手动启动完整仿真（4 个进程，建议用 tmux）：

```bash
# 1) PX4 + Gazebo（自定义 x500_flow_depth + flow 世界）
HEADLESS=1 ./scripts/run_sitl_combined.sh

# 2) uXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 3) ROS 2 工作区
source /opt/ros/humble/setup.bash && source install/setup.bash

# 4) 深度桥 + 障碍距离 + Web UI
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
ros2 run flight_perception min_depth --ros-args -p image_topic:=/depth_camera
ros2 run flight_ui ui_server            # http://127.0.0.1:8000/
```

4. 浏览器打开 `http://127.0.0.1:8000/`，按住“按住使能”，油门在最低，点 `TAKEOFF`；`LAND` 降落，`KILL` 紧急停。
5. 若要从头安装，按 [REBUILD.md](REBUILD.md) 第三章，或 [docs/01-environment.md](docs/01-environment.md) → [docs/02-px4-sitl.md](docs/02-px4-sitl.md) 顺序做。

## 里程碑

- [x] **P0** 仓库脚手架、`versions.env`、`verify.sh`、教材、git
- [x] **P1** Gazebo Harmonic 8.15.0 + PX4 v1.16.2，`gz_x500_flow` 光流 quality 255
- [x] **P2** uXRCE-DDS + px4_msgs，`/fmu/out/*` 在线
- [x] **P3** 光流实验：无 GPS 悬停，CSV 记录 flow/quality/距离/EKF 速度
- [x] **P4** 深度相机：`x500_depth` + `ros_gz` + `min_depth` → `/obstacle_distance`
- [x] **P5** Web 遥控器：双虚拟摇杆、deadman、ARM/DISARM/POSCTL/TAKEOFF/LAND/KILL
- [x] **P6** 合并模型 `x500_flow_depth`，光流 + 深度 + 测距同机
- [ ] **P7** 升级演练（Humble→Jazzy / 新 PX4 / Docker）：见 [docs/08-upgrade.md](docs/08-upgrade.md)

## 本周进展（2026-10-27 二 ~ 10-30 五）

37 个提交，36 个文件改动，+3879 / −898 行。三条主线：**终端工具化 → 修真实 bug → 把 C++ 讲透**。

### 1. 终端命令体系（10-27）

| 提交 | 内容 |
|---|---|
| `8db1648` | `flight` 从 alias 改成 shell 函数 —— 之前 `flight start` 会被当成 launch 参数 |
| `8fef01c` | 修 `source ~/.bashrc` 语法错：老 alias 会在 `flight() {` 之前展开，吃掉函数名。抽出 `scripts/flight_env.sh`，开头先 `unalias` |
| `379a154` ~ `b4a5cc0` | 四终端控制台 `flight link/sim/remote/views`，`sim` 真开 Gazebo 窗口，`remote` 先起后端再开浏览器 |
| `fb05cf2` | `flight` / `fly` 的 Tab 补全（`scripts/flight_completion.bash`） |

现在共 24 个子命令，按用途分三组：飞行动作 / 状态运维 / 一键服务。

### 2. 修掉的真实 bug

| 提交 | 现象 | 根因 |
|---|---|---|
| `aa5761e` `5ce57c9` | 网页点起飞没反应 | ① 解锁卡在 deadman 上 ② `msg.valid` 填成 deadman → PX4 判遥控丢失 ③ `stick_keeper` 与后端抢同一话题 |
| `b18dfd5` | 只跑 `flight remote` + `flight sim` 也不飞 | 没起 `MicroXRCEAgent`，`/fmu` 零话题。加 `ensure_agent()` 兜底 + 后端"无遥测拒绝命令" |
| `396c583` | `flight stop` 把自己所在的 shell 杀掉 | `kill_by_cmd` 的正则没排除 self / PPID |
| `1f63ed7` | 网页占着控制权时终端打杆"假成功" | 改为直接拒绝并说明原因 |
| `97757f4` | `flight start` 后飞机自己飞走 1 km | `/tmp/stick.json` 里残留上次杆量，摇杆守护一启动就照发。改为启动前先写回中位 |
| `bd00d08` | 两端同时写租约文件会解析失败 | C++ 与 Python 共用同一个 `.tmp` 文件名，内容交织。改成带 PID：并发压测 **4000 次 35 次失败 → 0 次** |

### 3. 架构向 RMCS 看齐（10-27 晚）

| 提交 | 内容 |
|---|---|
| `82ac955` | `rc_server` 从 800 行单文件拆成 **8 个编译单元**（含 `shared_state.hpp` 与 8 个头文件） |
| `3ee0f0e` | **控制权租约** `/tmp/flight_control.lease`，web 优先级 10 > terminal 5，两端逻辑对称 |
| `89b3b2b` | 后端参数外置到 `config/flight_ui.yaml`（5 个参数，改完重启即可） |
| `e6666a1` `66c2072` | `flight perc` 一条命令起感知节点；`/obstacle_distance` 上 HUD |

### 4. 录制出片（10-28）

| 提交 | 内容 |
|---|---|
| `8de9dc4` | **光流 + 深度进录制**：四路画中画（第一视角/俯视/深度/光流），`--sensors depth,flow\|none\|depth\|flow` 终端开关，关掉的那路连相机桥都不建 |
| `97757f4` | **航迹改画 Gazebo 真值** —— 实测 PX4 无 GPS 时 XY 估计只有真值的 1/3（2.98 m vs 9.18 m，方向还偏 9°） |
| `9b5ec40` | **视频编码走 GPU**（`h264_nvenc`）：CPU 时间 32.4 s → **4.4 s**，体积基本持平 |

`flight record [none\|depth\|flow]` 现在可选录哪几路；成片 180 s，四路画中画 + 真值航迹。

### 5. 调试环境 + 代码讲解（10-29 ~ 10-30）

| 提交 | 内容 |
|---|---|
| `3e1dfd9` | **补调试符号**：原来 `debug_info` 段数为 0，gdb 连断点都下不了。改后 `main.cpp:121` 实测能打印 `c` / `state` / `manual_pub` |
| `d632146` | 新增 **[docs/15-cpp-reading-and-debugging.md](docs/15-cpp-reading-and-debugging.md)**：三遍读法 + 逐函数参数清单 + 四种调试手段 |
| `48ae4a3` | 按代码实际状态核对 docs，修掉一批过时事实（6→8 个编译单元、行数全线偏差、`vehicle_status` 少了 `_v1`、`flight` 子命令漏 9 个） |

之后逐个文件讲透（`docs/15` 现在 1920 行）：`control_lease` 的原子替换与租约、
`http_server` 的 TCP/Beast、`ws_session` 全流程、`shared_state` 的两把锁与 Owner 规则、
`command_sequence` 的时序与阻塞点、感知包两个节点、`main.cpp` 的四个订阅与 QoS 匹配、
杆量映射六组合表、`min_depth` 的 ROI 与逐像素扫描、`demo_show.py` 怎么"飞"飞机。

### 本周三个最有价值的发现

| 发现 | 实测数据 |
|---|---|
| PX4 无 GPS 时 XY 估计缩水到真值 1/3 | 2.98 m vs 9.18 m，方向偏 9° → 航迹/跟拍必须用 Gazebo 真值 |
| 视频编码该走 GPU | libx264 32.4 s CPU → h264_nvenc 4.4 s，体积 7.8M vs 8.3M |
| 租约临时文件名撞车会退化成双发杆量 | 并发压测 4000 次 35 次失败 → 改带 PID 后 0 次 |

### 待办

- [ ] **33+ 个提交还没推到 GitHub**（`origin` = `gebilaowuxiaoyu123/Flight_test`）；
  本机无 GitHub 凭据（无 `credential.helper`、`~/.ssh` 只有 `known_hosts`、无 `gh`），
  需在个人终端执行 `git push origin main`（密码填 Personal Access Token）
- [ ] `.git` 已 ~289 MB（含 4 版成片视频）。推送前可考虑迁到 Git LFS 或只保留最终版
- [ ] `docs/13` 里的频率数字是 10-21 的测量快照，若要汇报需重跑一次仿真核对

## 已验收结果（2026-10-21）

| 项目 | 结果 |
|---|---|
| Gazebo | Harmonic 8.15.0，`gz sim` 正常 |
| PX4 | v1.16.2，SITL 自定义模型 `x500_flow_depth_0` |
| 光流 | `/fmu/out/vehicle_optical_flow` ~50 Hz，quality 最高 255 |
| 测距 | `/fmu/out/distance_sensor` orientation=25，随高度变化 |
| 深度 | `/depth_camera` 640×480 32FC1，约 15 Hz；`/obstacle_distance` 3.86 m（前方 4 m 墙） |
| Web UI | `:8000/health` 正常；ARM、AUTO_TAKEOFF 到 2.5 m、LAND 均验证 |
| 光流 CSV | `/tmp/optical_flow.csv`，含 flow/quality/distance/EKF 速度 |

## 关键启动参数

`scripts/run_sitl_combined.sh` 已经写入以下参数，重启后仍然生效：

```text
PX4_SIM_MODEL=gz_x500_flow_depth
PX4_SYS_AUTOSTART=4021
PX4_GZ_WORLD=flow
PX4_PARAM_NAV_DLL_ACT=0
PX4_PARAM_NAV_RCL_ACT=0
PX4_PARAM_COM_RC_IN_MODE=2
PX4_PARAM_CBRK_SUPPLY_CHK=894281
PX4_PARAM_CBRK_USB_CHK=197848
PX4_PARAM_CBRK_IO_SAFETY=22027
PX4_PARAM_EKF2_HGT_REF=0
PX4_PARAM_EKF2_BARO_CTRL=1
PX4_PARAM_EKF2_RNG_CTRL=1
PX4_PARAM_SIM_BAT_DRAIN=3600
PX4_PARAM_COM_LOW_BAT_ACT=0
PX4_PARAM_BAT_LOW_THR=0.05
PX4_PARAM_COM_DISARM_LAND=0
PX4_PARAM_COM_DISARM_PRFLT=0
```

> `COM_DISARM_LAND=0` 是为了方便演示（降落后不会 2 秒自动上锁）。
> 真实飞行应改回 `2`，并保留手动 DISARM。

## 已知限制 / 下一步

- 4021 没有 GPS/磁罗盘，yaw 会缓慢漂移；适合室内光流实验，不适合长航时。
- 光流水平定位依赖下视激光的 `distance_m`；若升级 PX4 后 `distance_m` 又变成 `NaN`，
  按 [docs/07-integration.md](docs/07-integration.md) 第 4 节检查 gz_bridge 的 orientation 判断。
- `build_ros.sh` 目前仍会构建 `src/` 下所有包，建议改为选择性构建（见脚本注释）。
- 清理：`/dev/shm/.flight_askpass` 是临时 sudo askpass，交付前删除；`/tmp/*.log` 可删。

## C++ 版本（参考实现）

| 包 | 内容 | 状态 |
|---|---|---|
| `src/flight_ui_cpp` | rclcpp + Boost.Beast + jsoncpp 的 Web 遥控器 | 编译通过，`ws_smoke_test.py` 6 PASS |
| `src/flight_perception_cpp` | C++ 版 `min_depth` / `optical_flow_logger` | 编译通过，`/obstacle_distance` 有数据 |

操作步骤与踩坑记录见 [docs/11-cpp-rewrite.md](docs/11-cpp-rewrite.md)。
这两个包**不在** `build_ros.sh` 的默认构建列表里，不影响现有 Python 链路。

```bash
# 单独编译并对比
colcon build --packages-select flight_ui_cpp flight_perception_cpp \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 run flight_ui_cpp rc_server          # 占用 8000，先停掉 python 版 ui
python3 scripts/ws_smoke_test.py          # 同一套黑盒验收
```

## 四终端控制台（当前日常用法）

```bash
flight link      # ① 通信连接（MicroXRCEAgent + 节点清单 + 话题频率）
flight sim       # ② 仿真（PX4 + Gazebo 3D 窗口）
flight remote    # ③ 网页遥控（后端日志 + 自动开浏览器）
flight views     # ④ 三视角（第一人称 / 第三人称 / 俯视）
```

`flight stop` 一次收掉全部（含 `flight link` 的 agent、相机桥、三视角窗口）。

**磁盘**：PX4 黑匣子会涨到十几 G 吃满根分区，已默认关闭（`PX4_PARAM_SDLOG_MODE=-1`），
手动清：`flight clean` / `flight clean --all`。
