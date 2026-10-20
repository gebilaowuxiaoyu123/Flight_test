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
ros2 topic echo /fmu/out/vehicle_status --once --qos-reliability best_effort
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
