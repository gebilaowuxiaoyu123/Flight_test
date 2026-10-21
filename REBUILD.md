# 从零复现手册（How I built this）

这份文档是给**没有我在场时的你**准备的：你可以用它

- **A. 把现在这台机器上已经跑通的东西重新启动起来**（5 分钟）→ 看 第一章「先跑起来」
- **B. 在一台干净的新机器上从零装一遍**（1～2 小时，主要是下载）→ 看 第三章「从零安装」
- **C. 出问题时自己定位** → 看 第六章「坏了怎么查」

每一节都写了：**要执行的命令 → 应该看到什么 → 没看到怎么办**。
命令全部可以直接复制粘贴，默认工作目录是 `/home/wiki_wu/flight_project`。

> 项目里还有：`PROGRESS.md`（当前进度与验收结果）、`docs/00`～`docs/10`（原理教材）、
> `scripts/`（全部自动化脚本）。本文档是"操作手册"，教材是"为什么这样做"。

---

## 〇、这套东西到底是什么

一句话：**PX4 飞控固件跑在 Gazebo 里（SITL），ROS 2 通过 uXRCE-DDS 读它的数据、给它发指令，
浏览器里的两个虚拟摇杆通过 WebSocket 控制它；同一架无人机上还装了光流相机、激光测距和深度相机。**

四个进程撑起全部功能：

| 进程 | 作用 | 日志 |
|---|---|---|
| `MicroXRCEAgent udp4 -p 8888` | PX4 ↔ ROS 2 的 DDS 桥 | `/tmp/flight_agent.log` |
| `scripts/run_sitl_combined.sh` | 启动 PX4 + Gazebo + 自定义机型 | `/tmp/flight_sitl.log` |
| `ros_gz_bridge` + `min_depth` + `optical_flow_logger` | 深度图转障碍距离、光流写 CSV | `/tmp/flight_bridge.log` 等 |
| `ros2 run flight_ui ui_server` | Web 遥控器，`http://127.0.0.1:8000/` | `/tmp/flight_ui.log` |

数据流：

```
浏览器摇杆 ──WebSocket──> flight_ui ──/fmu/in/manual_control_input──> PX4
                                 └──/fmu/in/vehicle_command──────> PX4
PX4 ──/fmu/out/*──> ROS 2（遥测、光流、测距）
Gazebo 深度相机 ──/depth_camera──> ros_gz_bridge ──> min_depth ──> /obstacle_distance
```

---

## 一、先跑起来（5 分钟）

### 1.1 一条命令启动

```bash
cd /home/wiki_wu/flight_project
./scripts/run_demo.sh
```

正常输出（实测 29 秒）：

```
[20:15:49] 已启动 tmux 会话 agent
[20:15:49] 已启动 tmux 会话 sitl
[20:15:49] 等待 PX4/Gazebo 启动 ...
[20:16:13] PX4 已就绪（IMU 在流、EKF 高度有效）
[20:16:13] 已启动 tmux 会话 bridge
[20:16:13] 已启动 tmux 会话 depth
[20:16:13] 已启动 tmux 会话 flowlog
[20:16:13] 已启动 tmux 会话 ui
[20:16:18] Web UI: http://127.0.0.1:8000/
```

脚本是**幂等**的：已经存在的 tmux 会话会跳过，所以你随时可以再跑一次"补启"。
如果检测到 PX4 卡在 `ekf2 missing data`（冷启动偶发，见 4.1），它会自动重启一次 SITL。

### 1.2 打开浏览器

<http://127.0.0.1:8000/>

操作顺序（**顺序错了会被拒绝，这是设计**）：

1. **按住"按住使能"**（deadman，不按住一切输入无效）
2. 左边油门保持在**最下面**
3. 点 `TAKEOFF` → 自动解锁并爬到约 2.5 m
4. 想降落点 `LAND`
5. 落地后点 `POSCTL` 再点 `DISARM`（PX4 不允许"还没落地"就上锁）
6. 出问题点 `KILL`

### 1.3 三行验收

```bash
./scripts/verify.sh                    # 期望 PASS=13 FAIL=0 SKIP=0
python3 scripts/ws_smoke_test.py       # 期望 6 个 [PASS]，exit 0
tail -3 /tmp/optical_flow.csv          # 期望有非零 quality
```

`ws_smoke_test.py` 是**无人值守的端到端测试**：它自己连 WebSocket、按住 deadman、解锁、
起飞到 2 m、降落、落地检测、上锁，每一步都打印 `[PASS]/[FAIL]`。
以后改了任何代码，先跑它。

实测通过的输出：

```
[PASS] Web 遥测已就绪
[PASS] 解锁成功
[PASS] 爬升到约 2.0 m
[PASS] 降落到地面附近
[PASS] PX4 检测到已落地
[PASS] 上锁成功
exit=0
```

### 1.4 停止全部

```bash
for s in agent sitl bridge depth flowlog ui; do tmux kill-session -t "$s" 2>/dev/null; done
```

### 1.5 想看 Gazebo 3D 窗口

演示默认 `HEADLESS=1`（省显存、更稳）。要看画面：

```bash
tmux kill-session -t sitl
HEADLESS=0 ./scripts/run_sitl_combined.sh
```

已经在跑无头仿真时，也可以只起一个 GUI 客户端连上去：

```bash
gz sim -g          # 只开界面，连现有仿真（Ctrl-C 关闭界面，不影响仿真）
```

> **关于 `MESA: warning: Driver does not support the 0x7d67 PCI ID`**：
> 本机是 RTX 5070（新卡），Mesa 的 EGL 设备加载器还不认识这个 PCI ID，会打印这条 warning。
> 实测它**不影响仿真**：实时率约 0.8，IMU/光流/深度都正常。
> 想确认到底有没有落到软件渲染，看这里：
>
> ```bash
> glxinfo -B | grep -E 'OpenGL renderer|OpenGL version'
> # 期望：OpenGL renderer string: NVIDIA GeForce RTX 5070 Laptop GPU/PCIe/SSE2
> # 如果显示 llvmpipe / softpipe，才是真的软件渲染
> ```
>
> 只是跑数据/做实验的话，`HEADLESS=1` 完全够用，也最稳。

---

## 二、版本与取舍（为什么是这些版本）

`versions.env` 是**唯一的版本来源**，改版本只改它：

```bash
PX4_TAG=v1.16.2          # PX4 固件
PX4_MSGS_BRANCH=release/1.16
ROS_DISTRO=humble
GZ_VERSION=harmonic
ROS_GZ_BRANCH=humble
JOBS=8                   # 30GB 内存无 swap，限制并行度
```

关键决策及原因（这些是试错后的结论，别再走回头路）：

| 决策 | 原因 |
|---|---|
| PX4 **v1.16.2**，不要 v1.14/1.15 | v1.16 才切到 Gazebo **Harmonic**，并且自带 `gz_x500_flow` 机型 + `optical_flow` gz 插件；v1.15 及以前装的是已 EOL 的 Garden，且没有 gz 光流模型 |
| 不用 Gazebo Classic | 已停止维护，PX4 新版本已移除 |
| `ros_gz` 源码编译（`GZ_VERSION=harmonic`） | Humble 的 apt 包是给 Fortress 编的，和 Harmonic 不兼容 |
| 不用 `ubuntu.sh`，用 `scripts/install_deps.sh` | `ubuntu.sh` 在 `apt-get update` 会因为本机一个无关的 `llvm-apt` 源缺 GPG key 直接退出（见 4.2） |
| PX4 子模块只初始化 9 个 | `--recursive` 会拉 NuttX(1.5GB)+Gazebo Classic+jsbsim+FlightGear，对四旋翼 Gazebo 仿真完全无用，会卡很久 |
| 用环境变量传 PX4 参数 | `pxh>` 里 `param set` 的值重启就丢；`PX4_PARAM_*` 才是可复现的 |

---

## 三、从零安装：完整流程

> 前提：Ubuntu 22.04（jammy），能联网，有 sudo 权限。
> 时间预估：下载 60～90 分钟（主要卡在 GitHub 和 apt），编译 15～30 分钟（8 核）。
> 磁盘：最终约 4.5 GB（PX4 源码 4.0 GB + build 332 MB + install 68 MB）。
> 内存：30 GB 无 swap 的机器实测峰值约 12 GB；扩容后 `versions.env` 里改 `JOBS=auto`。

### 阶段 0：确认系统与 ROS 2

```bash
lsb_release -cs                     # 期望 jammy
source /opt/ros/humble/setup.bash
ros2 --version                      # 期望有输出
nvidia-smi                          # 可选，有独显更快
```

没有 ROS 2 Humble：

```bash
sudo apt install -y ros-humble-desktop
```

### 阶段 1：安装 Gazebo Harmonic + 系统依赖

```bash
cd /home/wiki_wu/flight_project
./scripts/install_gazebo.sh
./scripts/install_deps.sh
```

验收：

```bash
gz sim --version     # 期望：Gazebo Sim, version 8.15.0
```

`install_deps.sh` 对无关 apt 源报错是**容错**的（会 warn 但继续），
最后看到 `完成` 就算成功。它装的是：GStreamer、protobuf、OpenCV 依赖、
`python3-jinja2 / empy / kconfiglib`（PX4 编译需要）、`libeigen3-dev`、`cmake`、`ninja-build` 等。

### 阶段 2：PX4 源码（**注意不要 `--recursive`**）

```bash
./scripts/install_px4.sh
```

它做三件事：

1. `git clone --branch v1.16.2`（不递归）；
2. 只初始化 9 个必需子模块：
   `Tools/simulation/gz`、`mavlink`、`Micro-XRCE-DDS-Client`、`gps/devices`、
   `libtomcrypt`、`libtommath`、`monocypher`、`libevents`、`heatshrink`；
3. 调用 `install_deps.sh`。

验收：

```bash
git -C PX4-Autopilot describe --tags --always                    # 期望 v1.16.2
ls PX4-Autopilot/Tools/simulation/gz/models | grep x500_flow     # 期望有 x500_flow
du -sh PX4-Autopilot                                             # 期望 ~4 GB（递归会 8 GB+）
```

### 阶段 3：编译 PX4 SITL

```bash
./scripts/build_px4.sh gz_x500_flow > /tmp/build_px4.log 2>&1
tail -20 /tmp/build_px4.log
```

它会**自动重放本项目对 PX4 的两个补丁**（见 4.5）：

- `px4_overlay/px4_patches/dds_topics.yaml` → 让 `vehicle_optical_flow`、`distance_sensor` 走 uXRCE；
- `px4_overlay/px4_patches/gz_bridge_downward_lidar.patch` → 修正下视激光的朝向识别。

> **一定要把输出重定向到文件**，否则 VS Code 会因为 1 GB 的终端输出而 OOM 崩掉（见 4.6）。

验收：

```bash
ls -lh PX4-Autopilot/build/px4_sitl_default/bin/px4     # 存在且可执行
```

### 阶段 4：uXRCE-DDS Agent

```bash
./scripts/install_agent.sh
MicroXRCEAgent --help | head -3
```

### 阶段 5：ROS 2 工作区

```bash
./scripts/fetch_deps.sh     # 克隆 px4_msgs(release/1.16) 和 ros_gz(humble) 到 src/
./scripts/build_ros.sh      # 只编 px4_msgs / ros_gz_bridge / flight_ui / flight_perception
```

`build_ros.sh` 里 `PACKAGES=` 控制构建范围。不要全量编 `src/`
（会把 ros_gz 的 demo 包也编一遍，很慢还容易缺依赖）。

额外需要三个消息包（`ros_gz_bridge` 要用）：

```bash
sudo apt install -y ros-humble-actuator-msgs ros-humble-gps-msgs ros-humble-vision-msgs
```

Web 服务还需要新版 websockets（系统的 9.1 和 uvicorn 不兼容）：

```bash
python3 -m pip install --user "websockets>=13" wsproto
```

验收：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 pkg prefix px4_msgs flight_ui flight_perception ros_gz_bridge
```

### 阶段 6：第一次 SITL（先只跑官方光流机型）

```bash
HEADLESS=1 ./scripts/run_sitl_flow.sh > /tmp/sitl.log 2>&1
tail -f /tmp/sitl.log
```

看到 `pxh>` 和 `INFO [commander] Ready for takeoff!` 就成功了。
这一步用的是 PX4 自带的 `gz_x500_flow`（还没用我们的合并模型），用来排除"环境没装好"。

### 阶段 7：光流验证

```bash
./scripts/verify.sh          # 应该有"光流话题在线"

source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic echo /fmu/out/vehicle_optical_flow --once --qos-reliability best_effort
```

期望：`quality` 接近 255、`distance_m` 是有限数（例如 0.11）。

- `quality` 一直是 0 → 地面没有纹理，见 4.3；
- `distance_m` 是 `.nan` → 见 4.5。

### 阶段 8：深度相机验证

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo

# 另开终端
ros2 run flight_perception min_depth --ros-args -p image_topic:=/depth_camera
ros2 topic hz /depth_camera          # 期望 ~10-15 Hz
ros2 topic echo /obstacle_distance --once
```

注意话题名是 `/camera_info`，**不是** `/depth_camera/camera_info`。

### 阶段 9：Web 遥控器

```bash
./scripts/run_ui.sh      # 或 ros2 run flight_ui ui_server
curl -s http://127.0.0.1:8000/health
```

端口是 **8000**（不是 8080）。

### 阶段 10：合并模型（光流 + 深度同机）

```bash
./scripts/run_sitl_combined.sh
```

它做的事（看懂这段就懂整个项目最脆的地方）：

```bash
# 1) PX4 只会在自己的 models/ 目录里找模型，所以把 overlay 模型软链进去
ln -sfn px4_overlay/models/x500_flow_depth \
        PX4-Autopilot/Tools/simulation/gz/models/x500_flow_depth
# 2) 世界文件同理
ln -sfn px4_overlay/worlds/flow.sdf \
        PX4-Autopilot/Tools/simulation/gz/worlds/flow.sdf
# 3) 用 PX4_SIM_MODEL 选模型（PX4_GZ_MODEL 在 v1.16 已废弃）
export PX4_SIM_MODEL=gz_x500_flow_depth
export PX4_SYS_AUTOSTART=4021      # 4021 = 关 GPS + 开光流
export PX4_GZ_WORLD=flow           # 棋盘格地面 + 4 m 处一堵墙
export HEADLESS=1
# 4) 一批 SITL 必需参数（每次重启都生效）
export PX4_PARAM_NAV_DLL_ACT=0 ... # 完整列表见脚本
```

### 阶段 11：总验收

```bash
./scripts/verify.sh                 # PASS=13 FAIL=0 SKIP=0
python3 scripts/ws_smoke_test.py    # 全 PASS
```

---

## 四、我踩过的坑（现象 → 根因 → 修复）

这一节是**整份文档最值钱的部分**，以后遇到同样的报错直接对号入座。
（编号 4.1～4.11，用 `Ctrl-F` 搜报错关键字更快。）

### 4.1 冷启动后 `ekf2 missing data`，IMU 只传了一帧

**现象**：`/tmp/flight_sitl.log` 一直重复
`WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data`，解锁被拒；
PX4 shell 里 `listener sensor_combined` 显示 `timestamp: ... (98.987999 seconds ago)`，
但 Gazebo 侧 `gz topic -f -t .../imu_sensor/imu` 明明有 270 Hz。

**根因**：冷启动竞争。Gazebo 还没把模型传感器发布出来时，`gz_bridge` 就订阅了 IMU，
回调只收到一帧就再没有后续（订阅静默失效）。

**修复**：

- 短期：`tmux kill-session -t sitl` 后重启一次即可（第二次必然正常）；
- 长期：`scripts/run_demo.sh` 会调用 `scripts/wait_ready.sh` 检查
  "IMU 频率 > 50 Hz 且 EKF 高度有效"，不满足就**自动重启一次 SITL**；
- 自检命令：`./scripts/wait_ready.sh 45`。

### 4.2 `ubuntu.sh` 在 `apt-get update` 就退出

**现象**：`PX4-Autopilot/Tools/setup/ubuntu.sh` 第一分钟就失败，报缺 GPG key
`15CF4D18AF4F7421`（`llvm-apt` 源）。

**根因**：系统里有一个和本项目无关的 LLVM APT 源没有导入 key，`apt-get update` 整体返回非零。

**修复**：别用 `ubuntu.sh`，用 `scripts/install_deps.sh`，它把 `apt-get update` 写成容错形式
（失败只 warn）。要彻底修可以自己给那个源导入 key，但对仿真毫无必要。

### 4.3 光流 `quality` 一直为 0

**现象**：`vehicle_optical_flow` 有数据但 `quality: 0`。

**根因**：Gazebo 默认地面是纯灰色，光流算法找不到特征点。

**修复**：世界文件里铺棋盘格。`px4_overlay/worlds/flow.sdf` 的 `flow_grid` 就是棋盘格；
**另外地面碰撞体要足够大**（改成 500×500），否则无人机会从地面边缘掉下去。

### 4.4 解锁被拒：`No connection to the ground control station`

**现象**：`WARN [commander] Arming denied: Resolve system health failures first`，
日志里有 `Preflight Fail: No connection to the ground control station`。

**根因**：`NAV_DLL_ACT` 默认是 2，SITL 没有地面站，于是"必须有数据链路"这项检查失败。

**修复**：`run_sitl_combined.sh` 用环境变量一次性关掉相关检查：

```text
NAV_DLL_ACT=0 NAV_RCL_ACT=0 COM_RC_IN_MODE=2
CBRK_SUPPLY_CHK=894281 CBRK_USB_CHK=197848 CBRK_IO_SAFETY=22027
```

### 4.5 光流 `distance_m` 是 NaN，导致起飞后盲降

**现象**：能解锁，`AUTO_TAKEOFF` 飞 1～2 秒后切到 `DESCEND`，
日志 `WARN [mc_pos_control] Failsafe: blind land`；同时
`ros2 topic echo /fmu/out/distance_sensor` 里 `orientation: 100`。

**根因**：`orientation: 100` 表示 `ROTATION_CUSTOM`，而 PX4 的 `VehicleOpticalFlow`
只接受 `ROTATION_DOWNWARD_FACING(25)` 的测距数据；拿不到 `distance_m` → 光流没有尺度
→ EKF 无法做水平定位 → 盲降。gz_bridge 判断"下视"只认 `(0,1,0,0)` 这个四元数，
而我们的模型是绕 Y 轴 +90° 的 `(w=.707, y=.707)`。

**修复**：`px4_overlay/px4_patches/gz_bridge_downward_lidar.patch` 把后者也算作下视，
`build_px4.sh` 自动应用并重编。验证：

```bash
ros2 topic echo /fmu/out/distance_sensor --once --qos-reliability best_effort | grep orientation
# 必须是 25
```

### 4.6 VS Code 崩溃（V8 OOM）

**现象**：跑 PX4 编译时 VS Code 整个窗口消失，Crashpad 里是 `V8 javascript OOM`。

**根因**：PX4 编译的 stdout 被完整读进一个字符串（1.08 GB），渲染进程直接 OOM。

**修复**（两条都要做）：

1. **所有长命令重定向到文件**，只回读 `tail`：

   ```bash
   ./scripts/build_px4.sh > /tmp/build.log 2>&1; tail -20 /tmp/build.log
   ```

2. 提高 V8 堆上限，`~/.config/Code/User/argv.json`：

   ```json
   { "js-flags": "--max-old-space-size=8192" }
   ```

   改完要**完全退出 VS Code 再打开**才生效。

顺便：长时间跑仿真会被系统休眠打断，我用

```bash
gsettings set org.gnome.desktop.session idle-delay 0
systemd-inhibit --what=idle:sleep:handle-lid-switch --why="sim" sleep infinity
```

### 4.7 深度相机有话题但没有图像

**现象**：`gz topic -l` 里有 `/depth_camera`，但 `ros2 topic hz /depth_camera` 没数据。

**根因**：合并模型里用 `<include merge="true">` 引入 OakD-Lite 时，`camera_link`
有时不会真的合并进最终模型（`gz model -m ... -l | grep camera_link` 查不到）。

**修复**：把 OakD-Lite 的 `link` 和两个 `sensor` **直接内联**写进
`px4_overlay/models/x500_flow_depth/model.sdf`（现在就是这么写的）。

### 4.8 Web 页面能打开但摇杆不动 / 静态资源 404

| 现象 | 根因 | 修复 |
|---|---|---|
| `/static/app.js` 404 | `colcon --symlink-install` 装的是软链，Starlette 默认不跟随 | `StaticFiles(..., follow_symlink=True)` |
| 摇杆推了没反应 | `COM_RC_IN_MODE` 不匹配输入源 | 用 `2`（接受任意源） |
| 摇杆推了没反应（另一种） | 时间戳用 ROS 墙钟，PX4 用开机时间，输入被判为"来自未来" | 跟随 `/fmu/out/timesync_status` 插值，并**故意减 100 ms** |
| 两个页面互抢摇杆 | 服务端只有一份摇杆状态 | 加"单控制器租约"：0.6 s 没发帧才让位 |

### 4.9 `TAKEOFF` 点了没反应 / 报 `deadman not held`

三层原因，现在都修好了：

1. ARM、切模式、NAV_TAKEOFF 三条命令在同一周期发出 → PX4 只接受第一条。
   现在是状态机：`等 deadman → 切 POSCTL → arm（重试 10 s）→ 等 armed → POSCTL → 延时 → NAV_TAKEOFF`；
2. 4021 机架没有 GPS，PX4 在默认 `AUTO_LOITER` 下做位置检查会失败 → 解锁前先切到不依赖 GPS 的模式；
3. `NAV_TAKEOFF.param7` 是 **AMSL 绝对高度**，不是离地高度 → 用 `ref_alt - z + 目标高度` 换算。

### 4.10 有第二个 Gazebo 实例在跑

**现象**：`gz model --list` 里出现莫名其妙的世界/模型，或者 `gz topic -l` 里有重复话题。

**根因**：有人（或上一次调试）在别处又起了一个 `gz sim`。Gazebo 默认共用同一个
transport 分区，第二个实例会和我们的仿真混在一起，`gz_bridge` 可能订阅到错的那个。

**处理**：

```bash
ps -eo pid,cmd | grep 'gz sim' | grep -v grep
# 只应有一条：... --verbose=1 -r -s .../worlds/flow.sdf
# 多余的（尤其 cwd 不在本项目里的）直接 kill 掉
```

**永远不要**在这个项目之外随手 `gz sim` 或 `gz sim -g` 起新仿真；
只想看画面就用 `gz sim -g`（只开界面，连现有仿真，不新建 server）。

### 4.11 其他零碎

| 现象 | 说明 |
|---|---|
| `ros2 topic hz` 报 `unrecognized arguments: --qos-reliability` | `hz` 不支持这个参数，`echo` 才支持 |
| 传感器话题 `echo` 卡住没输出 | 传感器是 **best-effort**，要加 `--qos-reliability best_effort` |
| `/fmu/out/vehicle_status` 不存在 | v1.16 的 `MESSAGE_VERSION=1`，实际叫 `/fmu/out/vehicle_status_v1` |
| 电量几十秒就掉到 0 | `SIM_BAT_DRAIN` 默认 60 秒掉完；已改成 3600 |
| 落地 2 秒后自动上锁 | `COM_DISARM_LAND` 默认 2；演示里设成 0 方便观察，**真机要改回 2** |
| `colcon` 报 email 非法 | `package.xml` 的 `<maintainer email>` 必须是合法邮箱 |
| 缺 `actuator_msgs/gps_msgs/vision_msgs` | `sudo apt install ros-humble-{actuator,gps,vision}-msgs` |
| `MESA: warning: Driver does not support the 0x7d67 PCI ID` | Mesa 不认识这块新卡，属良性 warning；用 `glxinfo -B` 确认渲染器仍是 NVIDIA（见 1.5） |

---

## 五、日常使用

### 5.1 起停

```bash
./scripts/run_demo.sh                 # 启动/补启全部
for s in agent sitl bridge depth flowlog ui; do tmux kill-session -t "$s" 2>/dev/null; done
tmux attach -t sitl                   # 进 PX4 控制台（退出用 Ctrl-b d，不要 Ctrl-c）
```

### 5.2 飞行（三种方式）

1. **Web 界面**：`http://127.0.0.1:8000/`（推荐，有 HUD）
2. **脚本**：`python3 scripts/ws_smoke_test.py --altitude 2.0`（无人值守）
3. **ROS 2**：往 `/fmu/in/vehicle_command` 发 `VehicleCommand`

### 5.3 看数据

```bash
# 光流（含质量与离地距离）
ros2 topic echo /fmu/out/vehicle_optical_flow --once --qos-reliability best_effort
# 测距（orientation 必须是 25）
ros2 topic echo /fmu/out/distance_sensor --once --qos-reliability best_effort
# 深度图频率
ros2 topic hz /depth_camera
# 前方障碍距离
ros2 topic echo /obstacle_distance --once
# 光流 CSV（含 EKF 速度对比，做实验用）
column -s, -t /tmp/optical_flow.csv | tail -20
```

CSV 列：`t, flow_x, flow_y, dt, distance, quality, ekf_vx, ekf_vy, flow_vx, flow_vy`，
其中 `flow_vx = pixel_flow_x / dt * distance`，可直接和 `ekf_vx` 对比验证 `v ≈ ω·h`。

### 5.4 改参数

- **可复现（推荐）**：改 `scripts/run_sitl_combined.sh` 里的 `PX4_PARAM_*`，重启生效；
- **临时**：`pxh>` 里 `param set XXX`，重启就丢；
- **永久**：改完 `param save`，但参数文件在 gitignore 的 build 目录里，换机器就没了。

---

## 六、坏了怎么查（决策树）

```text
起不来 / 页面打不开
├─ curl -s localhost:8000/health 没响应
│   ├─ tmux ls 里没有 ui        → ./scripts/run_demo.sh
│   └─ 有 ui 但日志报错          → tail -50 /tmp/flight_ui.log
├─ 页面能开但"连接断开"
│   └─ 端口不是 8000？          → 看 /tmp/flight_ui.log 里的 "Uvicorn running on"
└─ tmux ls 里没有 sitl
    └─ tail -50 /tmp/flight_sitl.log 找第一条 ERROR

能起来但解锁不了
├─ listener sensor_combined 的 timestamp 是几十秒前 → 冷启动竞争，重启 SITL（4.1）
├─ 日志 Preflight Fail: ekf2 missing data          → 同上，或看 6.1
├─ 日志 Preflight Fail: No connection to GCS       → NAV_DLL_ACT 没生效（4.4）
└─ 日志 Disarming denied: not landed               → 还没落地，别急着 DISARM

能解锁但飞不起来
├─ AUTO_TAKEOFF 几秒后 DESCEND（blind land）→ distance_sensor 的 orientation 是不是 25？（4.5）
├─ TAKEOFF ack OK 但高度不变 → 看 /fmu/out/vehicle_control_mode 的 flag_control_altitude_enabled
└─ 高度一直往上冲 → EKF2_HGT_REF / ref_alt 不对（6.1）

有的话题在、有的不在
├─ /fmu/out/* 缺光流或测距 → dds_topics.yaml 没打补丁（阶段 3）
├─ /depth_camera 在但没数据 → 模型里 camera_link 丢了（4.7）
└─ /obstacle_distance 没有   → min_depth 没起，或 image_topic 参数不对
```

### 6.1 EKF 高度相关

4021 机架**没有 GPS**，所以：

```text
EKF2_HGT_REF=0     # 用气压计做高度参考（=1 是 GPS，会一直等不到）
EKF2_BARO_CTRL=1
EKF2_RNG_CTRL=1    # 保留测距，光流需要它提供尺度
```

`NAV_TAKEOFF.param7` 要传 **AMSL**：`ref_alt - z + 期望离地高度`。
`vehicle_local_position.ref_alt` 在 4021 上可能是 `NaN`，代码里已做兜底。

### 6.2 收集证据的固定套路

```bash
./scripts/verify.sh
tail -60 /tmp/flight_sitl.log
tail -40 /tmp/flight_ui.log
tmux ls
ps -eo pid,cmd | grep -E 'px4_sitl|gz sim|MicroXRCE'
free -h; df -h /
```

把这五条的输出贴给未来的自己，基本就能定位。

---

## 七、如果让我重装一次

按这个顺序，每步用括号里的命令验收，失败就别往下走：

| # | 动作 | 命令 | 期望 |
|---|---|---|---|
| 1 | 装 Gazebo + 依赖 | `./scripts/install_gazebo.sh && ./scripts/install_deps.sh` | `gz sim --version` = 8.x |
| 2 | 拉 PX4（不递归） | `./scripts/install_px4.sh` | `describe` = v1.16.2，约 4 GB |
| 3 | 编 PX4（含补丁） | `./scripts/build_px4.sh gz_x500_flow > /tmp/b.log 2>&1` | `bin/px4` 存在 |
| 4 | 装 Agent | `./scripts/install_agent.sh` | `MicroXRCEAgent --help` |
| 5 | 拉 ROS 依赖并编译 | `./scripts/fetch_deps.sh && ./scripts/build_ros.sh` | `ros2 pkg prefix flight_ui` |
| 6 | 官方光流机型跑通 | `HEADLESS=1 ./scripts/run_sitl_flow.sh` | `Ready for takeoff!` |
| 7 | 起全链路 | `./scripts/run_demo.sh` | 29 秒内出 "PX4 已就绪" |
| 8 | 验收 | `./scripts/verify.sh && python3 scripts/ws_smoke_test.py` | 13 PASS / 全 PASS |

第 3 步最容易失败：先看 `tail -40 /tmp/b.log` 里第一条 `Error`，通常是
`empy/jinja2` 没装、`kconfiglib` 没装、或某个子模块没初始化（重跑 `install_px4.sh`）。

---

## 八、升级与扩容

### 8.1 升级 PX4

```bash
# 1) 只在 versions.env 里改 PX4_TAG 和 PX4_MSGS_BRANCH
# 2) 重放补丁 + 重编（脚本会自动做）
./scripts/install_px4.sh
./scripts/build_px4.sh gz_x500_flow > /tmp/b.log 2>&1
# 3) 回归
./scripts/run_demo.sh && ./scripts/verify.sh && python3 scripts/ws_smoke_test.py
```

升级后**必须重新确认两件事**（我打过补丁的地方，最容易在升级时丢掉）：

1. `/fmu/out/distance_sensor` 的 `orientation` 还是不是 `25`；
2. `dds_topics.yaml` 里还有没有 `vehicle_optical_flow`、`distance_sensor`。

### 8.2 升级 ROS（Humble → Jazzy）

1. 装 `ros-jazzy-desktop`，`versions.env` 的 `ROS_DISTRO` 改 jazzy；
2. `ROS_GZ_BRANCH` 改 jazzy 分支；
3. `rm -rf build install log` 后 `./scripts/build_ros.sh`；
4. `px4_msgs` 也要切对应分支。字段变化只集中在
   `src/flight_ui/flight_ui/px4_adapter.py` 和 `telemetry.py`，改这两处就够。

### 8.3 扩容后

- `versions.env` 里 `JOBS=auto`，编译时间明显下降；
- 想跑多机：`PX4_GZ_MODEL_POSE="0,2"` + `-i 2` 起第二架，但每架要独立的
  `PX4_GZ_MODEL_NAME`，本项目没做多机适配；
- 大盘可以用 `WORKSPACE_ROOT` 把 build/install 迁出去（脚本已支持）。

---

## 九、目录说明与速查表

```
flight_project/
├── versions.env              # 唯一版本来源
├── PROGRESS.md               # 当前进度 / 验收结果 / 断点续做
├── REBUILD.md                # 本文档
├── docs/                     # 原理教材 00-10
├── scripts/
│   ├── lib.sh                # 公共函数（路径、source ROS）
│   ├── install_*.sh          # 装环境
│   ├── build_px4.sh          # 编 PX4（自动重放补丁）
│   ├── build_ros.sh          # 编 ROS（只编需要的包）
│   ├── run_sitl_*.sh         # 起 SITL（flow / depth / combined）
│   ├── run_demo.sh           # 一键起全链路（含自愈）
│   ├── wait_ready.sh         # 判断"真起来了"还是"假启动"
│   ├── verify.sh             # 13 项环境+链路检查
│   └── ws_smoke_test.py      # 端到端飞行验收
├── src/
│   ├── flight_ui/            # Web 遥控器（FastAPI + WebSocket + rclpy）
│   ├── flight_perception/    # min_depth、optical_flow_logger
│   └── px4_msgs/  ros_gz/    # 上游源码（gitignore）
├── px4_overlay/
│   ├── models/x500_flow_depth/model.sdf   # 光流+LW20+OakD-Lite 合并模型
│   ├── worlds/flow.sdf                    # 棋盘格 + 4 m 墙
│   └── px4_patches/                       # 必须重放的 PX4 补丁
└── PX4-Autopilot/            # 上游源码（gitignore）
```

### 一页速查

| 我要… | 命令 |
|---|---|
| 全部启动 | `./scripts/run_demo.sh` |
| 全部停止 | `for s in agent sitl bridge depth flowlog ui; do tmux kill-session -t "$s"; done` |
| 看是否正常 | `./scripts/verify.sh` |
| 自动飞一次 | `python3 scripts/ws_smoke_test.py` |
| 看 PX4 控制台 | `tmux attach -t sitl`（`Ctrl-b d` 退出） |
| 看深度 | `ros2 topic hz /depth_camera` |
| 看障碍距离 | `ros2 topic echo /obstacle_distance --once` |
| 看光流 | `ros2 topic echo /fmu/out/vehicle_optical_flow --once --qos-reliability best_effort` |
| 起不来 | `tail -60 /tmp/flight_sitl.log` |
| 重新编译 | `./scripts/build_px4.sh gz_x500_flow > /tmp/b.log 2>&1` |
