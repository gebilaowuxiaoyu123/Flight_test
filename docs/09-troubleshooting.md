# 09 · 常见问题排查

按症状查，不要从头重装。每条都给出“先看什么、再怎么改”。

## 环境 / 安装

### `gz: command not found`

```bash
which gz
ls /usr/bin/gz*
```

- 没装：`./scripts/install_gazebo.sh`
- 装了但 PATH 没有：重新登录，或检查 `/usr/share/gazebo/setup.sh`

### `gz sim --version` 不是 8.x

可能装了 Garden/Fortress/Classic。检查：

```bash
dpkg -l | grep -E 'gz-|ignition-|gazebo'
```

只保留 `gz-harmonic` 相关包；移除 `gz-garden`、`gz-fortress`、`gazebo11`（如果不用 Classic）。

### PX4 编译 OOM

```bash
free -h
dmesg | tail
```

处理：

```bash
JOBS=4 ./scripts/build_px4.sh gz_x500_flow
```

或临时加 swap：

```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### `apt-get update` 因 `llvm-apt` 缺 key 失败

症状：

```text
W: GPG 错误：https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/jammy ...
   NO_PUBKEY 15CF4D18AF4F7421
E: 仓库 ... 没有数字签名。
```

`apt-get update` 返回 100，PX4 的 `ubuntu.sh`（`set -e`）会直接退出，后面的 Gazebo/PX4 依赖一个都不会装。

**快速绕过（临时禁用这个无关源）**：

```bash
sudo mv /etc/apt/sources.list.d/llvm-apt.list{,.disabled}
# 跑安装
sudo mv /etc/apt/sources.list.d/llvm-apt.list{.disabled,}
```

**彻底修复（补上 LLVM 的 key）**：

```bash
curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/llvm-snapshot.gpg
sudo sed -i '1s#^deb #deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] #' \
  /etc/apt/sources.list.d/llvm-apt.list
sudo apt-get update
```

key 指纹应为 `6084F3CF814B57C1CF12EFD515CF4D18AF4F7421`（短 ID `15CF4D18AF4F7421`）。

> 这是系统里原本就存在的问题，不是本项目引入的。项目脚本 `install_deps.sh` 会容忍它继续安装，但建议你有空时彻底修掉。

### `ubuntu.sh` 又装了一遍 Gazebo

正常。它按 Ubuntu 版本安装 `gz-harmonic`，不会移除现有的 Harmonic 数据。

### git/apt 走代理失败

用户 shell 里有 `http_proxy`，但 `sudo` 默认不继承。用：

```bash
sudo -E apt-get update
```

或给 apt 单独配置：

```bash
sudo tee /etc/apt/apt.conf.d/95proxy <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
EOF
```

## Gazebo / GPU

### GUI 黑屏、闪退、软件渲染

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim
```

或先无头验证：

```bash
HEADLESS=1 ./scripts/run_sitl_flow.sh
```

### `gz sim` 报 Ogre/OpenGL 错误

```bash
glxinfo -B | grep -E 'OpenGL renderer|OpenGL version'
```

确保走的是 NVIDIA 而不是 llvmpipe。必要时：

```bash
export LIBGL_ALWAYS_SOFTWARE=1   # 仅用于确认是不是 GPU 问题
```

## PX4 / SITL

### `ninja: error: unknown target 'gz_x500_flow'`

- PX4 版本低于 v1.16；
- 或 build 目录是旧配置。

```bash
cd PX4-Autopilot
make distclean
make px4_sitl gz_x500_flow
```

### 飞机不动 / 无法解锁

```text
commander status
```

看 arming checks。SITL 常见原因：

- 没切 POSCTL；
- 油门不在最低；
- EKF2 没准备好（等几秒）；
- `COM_RC_IN_MODE` 与输入源不匹配。

### `listener vehicle_optical_flow` 没数据

1. 确认机型是 `gz_x500_flow`；
2. 看 Gazebo 里光流传感器是否加载；
3. 看 `quality` 是否长期为 0；
4. `param show EKF2_OF_CTRL` 是否为 1。

### 光流有数据但飞机漂移

- 光流是速度观测，位置本来就会漂；
- 检查 `distance_sensor` 是否有效；
- 检查 `EKF2_OF_POS_*` 安装偏移；
- 地面纹理太弱会导致 `quality` 低。

## uXRCE-DDS / ROS 2

### `ros2 topic list` 没有 `/fmu/*`

```bash
ps aux | grep -E 'MicroXRCEAgent|px4'
ss -lunp | grep 8888
```

在 PX4 shell：

```text
uxrce_dds_client status
uxrce_dds_client start -t udp -p 8888
```

### 话题有但 `echo` 没数据

QoS 不匹配：

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 --once --qos-reliability best_effort
```

### 消息类型/字段对不上

`px4_msgs` 分支与 PX4 不匹配：

```bash
git -C src/px4_msgs branch --show-current
# 应为 release/1.16
```

改 `versions.env` 后重新 `fetch_deps.sh` + `build_ros.sh`。

### `ros_gz_bridge` 启动报 ABI/符号错误

你装的是 Fortress 版二进制，或和 Harmonic 混用：

```bash
dpkg -l | grep ros-humble-ros-gz
sudo apt remove 'ros-humble-ros-gz*'
```

只用源码编译、且 `GZ_VERSION=harmonic` 的版本。

## 深度相机

### `gz topic -l` 没有 depth 话题

- 机型不是 `gz_x500_depth`；
- 模型没加载 OakD-Lite；
- 看 Gazebo 终端日志。

### 桥接后 ROS 没图像

- 话题名写错：以 `gz topic -l` 为准；
- QoS：RViz2 用 Best Effort；
- 深度相机在远处/无纹理时可能全是 NaN。

### 点云为空

- 障碍物超出 `max_range`；
- 深度图编码不是 `32FC1`；
- RViz2 的 Fixed Frame 不对。

## Web 遥控器

### 解锁被拒

- 油门不在最低；
- 没切 POSCTL；
- `COM_RC_IN_MODE` 不允许 Joystick；
- 看 `commander status` 的 arming checks。

### 摇杆方向相反

只改 `flight_ui/px4_adapter.py` 的符号，不要改 PX4。

### 断线后飞机还在动

没有实现失效保护。必须：

- 超过 300 ms 没收到摇杆 → 发布零输入 + `valid=false`；
- 或者直接上锁。

## 磁盘 / 性能

### 磁盘满

```bash
du -sh PX4-Autopilot/build build install ~/.gz 2>/dev/null
```

清理：

```bash
cd PX4-Autopilot && make distclean
rm -rf build install log
```

### 仿真很卡

- 用 `HEADLESS=1` 排除 GUI；
- 降低 Gazebo 分辨率/关闭阴影；
- 确认 `glxinfo -B` 使用 NVIDIA；
- 关闭其他占显存的程序。

## 还是解决不了

按这个顺序收集信息：

```bash
./scripts/verify.sh
git -C PX4-Autopilot describe --tags --always
gz sim --version
ros2 topic list | grep fmu
free -h
df -h
```

把输出和 `PROGRESS.md` 的“当前卡点”一起记录，下一次就能从断点继续。

## 本项目实测坑（2026-10-21）

### 1. 解锁被拒：No connection to the ground control station

PX4 的 `NAV_DLL_ACT` 默认是 2，SITL 没有 GCS 时会拒绝解锁。
`scripts/run_sitl_combined.sh` 已通过 `PX4_PARAM_NAV_DLL_ACT=0` 等环境变量在启动时设置。
也可以在 `pxh>` 里临时执行：

```text
param set NAV_DLL_ACT 0
param set CBRK_SUPPLY_CHK 894281
param set CBRK_USB_CHK 197848
param set CBRK_IO_SAFETY 22027
```

注意：用 MAVLink shell 改的参数带 `*`，重启后会丢；脚本里的 `PX4_PARAM_*` 才是可复现的。

### 2. `AUTO_TAKEOFF` 飞得很高 / 进入 blind land

两个独立原因：

1. `EKF2_HGT_REF=1`（GPS）但 4021 关闭了 GPS，EKF 没有高度参考 → 切换为
   `EKF2_HGT_REF=0`（baro），并保留 `EKF2_BARO_CTRL=1`；
2. 下视激光被 gz_bridge 标成 `ROTATION_CUSTOM(100)`，光流模块只接受
   `ROTATION_DOWNWARD_FACING(25)`，导致 `vehicle_optical_flow.distance_m=NaN`，
   EKF 无法用光流做水平定位 → 修正 GZBridge 的下视判断并重编 PX4。

验收：

```bash
ros2 topic echo /fmu/out/distance_sensor --once --qos-reliability best_effort | grep orientation
# 必须是 25
ros2 topic echo /fmu/out/vehicle_optical_flow --once --qos-reliability best_effort | grep distance_m
# 不能是 .nan
```

### 3. 深度相机有话题但没有图像

`<include merge="true">` 合并 OakD-Lite 时，`camera_link` 可能没有进入最终模型。
用 `gz model -m x500_flow_depth_0 -l | grep camera_link` 确认；
本项目改为把 OakD-Lite 的 link 和两个 sensor **直接内联**到合并模型。
另外 `/camera_info` 不是 `/depth_camera/camera_info`。

### 4. Web UI 端口与 QoS

- `ui_server` 默认监听 **8000**，不是 8080；
- 传感器话题是 best-effort，`ros2 topic echo --once` 要加
  `--qos-reliability best_effort`，`ros2 topic hz` 不支持这个参数。

### 5. 多个浏览器/客户端抢摇杆

`RosWorker` 的摇杆状态是全局的；同时开两个页面时，后到的消息会覆盖先到的。
调试时先关掉多余页面，或只保留一个控制器。

### 6. VS Code 崩溃（V8 OOM）

PX4 编译输出被完整读成 1 GB 字符串会导致渲染进程 OOM。
所有长命令都重定向到 `/tmp/*.log`，只回读 tail；`argv.json` 里可加：

```json
"js-flags": "--max-old-space-size=8192"
```
