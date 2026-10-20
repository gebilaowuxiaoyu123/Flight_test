# 01 · 环境安装

## 学习目标

- 安装 Gazebo Harmonic、PX4 v1.16.2、ROS 2 工作区、Micro-XRCE-DDS-Agent
- 理解每个组件为什么需要、装在哪里
- 用 `scripts/verify.sh` 做客观验收

## 前置条件

| 项 | 要求 |
|---|---|
| Ubuntu | 22.04（24.04 也可，脚本按当前系统源安装） |
| ROS 2 | Humble 已安装（`/opt/ros/humble/setup.bash` 存在） |
| 内存 | 建议 ≥16 GB；当前 30 GB，无 swap，编译并行度 8 |
| 磁盘 | 整套约 8–11 GB；当前约 29 GB 可用 |
| 网络 | 能访问 GitHub、`packages.osrfoundation.org`；如有代理，确保 `sudo` 环境也能用 |
| sudo | 安装需要密码 |

先跑一次：

```bash
cd ~/flight_project
./scripts/verify.sh
```

此时大部分项应该是 `[SKIP]`，这是正常的。

## 1. Gazebo Harmonic

### 为什么是 Harmonic

PX4 v1.16 的 `Tools/setup/ubuntu.sh` 在 Ubuntu 22.04 上安装的是 `gz-harmonic`（`gz-sim8`）。Harmonic 是 LTS，支持到 2029；Garden 已 EOL，Classic 更早 EOL。

### 安装

```bash
./scripts/install_gazebo.sh
```

脚本做三件事：

1. 添加 OSRF Gazebo 源（带 keyring）
2. `apt install gz-harmonic libunwind-dev`
3. 校验 `gz sim --version` 是 8.x

手动命令等价于：

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo apt install -y gz-harmonic libunwind-dev
```

预期：

```text
$ gz sim --version
Gazebo Sim, version 8.x.x
```

> 如果 GUI 黑屏或明显走核显：
> ```bash
> __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim
> ```

## 2. PX4 v1.16.2

### 克隆

```bash
./scripts/install_px4.sh
```

脚本会：

1. `git clone --recursive -b v1.16.2`
2. 执行 `Tools/setup/ubuntu.sh --no-nuttx`
3. 不编译（编译在下一章）

`--no-nuttx` 跳过 ARM 工具链；我们只跑 SITL，不需要烧录飞控。

> `ubuntu.sh` 会再次安装 `gz-harmonic`，这是正常的；它不会移除 Harmonic。

### 检查版本

```bash
git -C PX4-Autopilot describe --tags --always
# 期望：v1.16.2
```

### 编译

```bash
./scripts/build_px4.sh gz_x500_flow
```

- 首次约 20–40 分钟。
- 并行度由 `JOBS` 控制，默认 8；当前无 swap，不要用 24。
- 产物在 `PX4-Autopilot/build/px4_sitl_default/`。

> 如果中途 OOM，关闭浏览器/降低 `JOBS`，或临时加 swap：
> ```bash
> sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile
> sudo mkswap /swapfile && sudo swapon /swapfile
> ```

## 3. ROS 2 工作区

### 拉取依赖

```bash
./scripts/fetch_deps.sh
```

会克隆：

- `src/px4_msgs`：分支 `release/1.16`，必须与 PX4 v1.16.2 匹配
- `src/ros_gz`：分支 `humble`，编译时用 `GZ_VERSION=harmonic`

### 编译

```bash
./scripts/build_ros.sh
```

脚本会：

1. `source /opt/ros/humble/setup.bash`
2. 安装 rosdep 依赖
3. `export GZ_VERSION=harmonic`
4. `colcon build --symlink-install`

> 关键：不要安装 `ros-humble-ros-gz*` 的二进制包。它链接的是 Fortress 的 `ignition-transport11`，和 Harmonic 的 `gz-transport13` 不是一套 ABI。我们只用源码编译的 `ros_gz`。

## 4. Micro-XRCE-DDS-Agent

```bash
./scripts/install_agent.sh
```

脚本会克隆、编译并安装到 `/usr/local`：

```bash
MicroXRCEAgent udp4 -p 8888
```

这是 PX4 与 ROS 2 之间的桥。PX4 侧的客户端在 SITL 里启动：

```text
uxrce_dds_client start -t udp -p 8888
```

## 5. 验收

```bash
./scripts/verify.sh
```

期望（还没启动 SITL 时）：

```text
[PASS] versions.env ...
[PASS] Gazebo Harmonic ...
[PASS] PX4 v1.16.2
[PASS] ROS 2 humble 已安装
[PASS] MicroXRCEAgent ...
[PASS] ROS 工作区可用（px4_msgs）
[SKIP] 未检测到 PX4 遥测话题 ...
[SKIP] 未检测到光流话题 ...
[SKIP] 未检测到 Gazebo 深度相机话题 ...
[SKIP] Web 遥控器未运行 ...
PASS=6  FAIL=0  SKIP=4
```

`[FAIL]` 必须为 0；`[SKIP]` 是“还没到那一步”。

## 常见问题

| 症状 | 原因 | 处理 |
|---|---|---|
| `gz: command not found` | 没装 Harmonic 或 PATH 未更新 | 重新登录，或 `source /etc/profile.d/gazebo.sh`（如存在） |
| `gz sim --version` 不是 8.x | 装了 Garden/Fortress | `sudo apt remove gz-garden`，再跑安装脚本 |
| PX4 编译 OOM | 24 线程 + 无 swap | `JOBS=8 ./scripts/build_px4.sh ...` |
| rosdep 找不到 `gz-msgs10` | 缺 Harmonic 开发包 | `sudo apt install libgz-msgs10-dev libgz-transport13-dev libgz-sim8-dev` |
| `ubuntu.sh` 重装 Gazebo | 脚本按 22.04 默认装 Harmonic | 正常；不要手动装 Classic |
| 代理导致 git/apt 失败 | `sudo` 不继承用户代理 | 用 `sudo -E`，或在 apt 里单独配置代理 |

更多见 [09-troubleshooting.md](09-troubleshooting.md)。

## 本章验收

- [ ] `gz sim --version` 显示 8.x
- [ ] `PX4-Autopilot` 在 `v1.16.2`
- [ ] `./scripts/verify.sh` 中 `[FAIL]` 为 0
- [ ] `source install/setup.bash` 后 `ros2 pkg prefix px4_msgs` 有输出
