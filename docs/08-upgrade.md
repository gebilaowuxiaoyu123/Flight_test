# 08 · 升级与扩容：跟得上新技术

## 学习目标

- 知道每个组件升级时改哪里、验什么
- 能在 Humble → Jazzy、PX4 小版本、Gazebo LTS 之间平滑迁移
- 扩容磁盘后把大型构建目录迁走，不破坏现有流程

## 原则

1. **一次只升一层**：不要同时换 PX4、ROS、Gazebo。
2. **先改 `versions.env`**，再跑脚本，最后跑 `verify.sh`。
3. **保留可回滚的旧版本**：至少保留上一个 PX4 tag 和一份 `versions.env`。
4. **升级后必须跑冒烟测试**：话题、光流、深度、UI 控制四项。
5. **不追最新**：优先 LTS；新版本至少等 PX4/ros_gz 官方支持后再上。

## 版本矩阵

| 阶段 | Ubuntu | ROS 2 | PX4 | Gazebo | ros_gz |
|---|---|---|---|---|---|
| 当前 | 22.04 | Humble | v1.16.2 | Harmonic | 源码（GZ_VERSION=harmonic） |
| 小升级 | 22.04 | Humble | v1.17.x | Harmonic | 源码 |
| 扩容后 | 24.04 | Jazzy | v1.17/1.18 | Harmonic | 二进制 `ros-jazzy-ros-gz` |
| 更远 | 24.04+ | Kilted/Rolling | 最新 | Jetty | 对应二进制/源码 |

> 关键差异：Jazzy 的 `ros_gz` 是按 Harmonic 编译的，升级后可以省掉 `GZ_VERSION` 源码编译这一环。

## 1. 升级 PX4 小版本（例如 v1.16.2 → v1.17.0）

```bash
# 1) 改版本
$EDITOR versions.env
# PX4_TAG=v1.17.0
# PX4_MSGS_BRANCH=release/1.17

# 2) 切换 PX4
./scripts/install_px4.sh

# 3) 重新编译
./scripts/build_px4.sh gz_x500_flow

# 4) 切换 px4_msgs 并重编 ROS
./scripts/fetch_deps.sh
./scripts/build_ros.sh

# 5) 回归
./scripts/verify.sh
```

重点检查：

- `ROMFS/.../airframes/4021_gz_x500_flow` 是否仍存在；
- 机型名是否仍是 `x500_flow` / `x500_depth`；
- `ManualControlSetpoint` 字段是否变化；
- `vehicle_optical_flow` 字段是否变化；
- EKF2 参数名是否变化（`param show EKF2_OF*`）。

如果 airframe 编号/名字变了，更新 `scripts/run_sitl_*.sh` 和文档。

## 2. 升级到 Ubuntu 24.04 + ROS 2 Jazzy

这是“扩容后”的主线。步骤：

1. 备份 `versions.env`、`px4_overlay/`、`src/` 里的自研包、`web/`。
2. 安装 Ubuntu 24.04 + ROS 2 Jazzy。
3. 安装 Gazebo Harmonic（Jazzy 配套）。
4. 安装 `ros-jazzy-ros-gz` 二进制包（不再源码编译）。
5. 修改 `versions.env`：

```bash
UBUNTU_CODENAME=noble
ROS_DISTRO=jazzy
ROS_GZ_BRANCH=jazzy
```

6. 重新 `fetch_deps.sh`、`build_ros.sh`。
7. 运行 `verify.sh`，重点看：
   - `px4_msgs` 是否能编译；
   - `flight_ui` 的 `rclpy` API 是否兼容；
   - `ros_gz_bridge` 是否能桥接深度话题。

代码层面要避免：

- Humble 特有的 launch/参数 API；
- 直接依赖 `ignition-*` 旧包名；
- 在业务代码里硬编码消息字段。

## 3. 升级 Gazebo（Harmonic → Jetty）

不要主动追。满足以下条件再考虑：

- PX4 官方支持 Jetty；
- `ros_gz` 有对应分支/二进制；
- 光流插件、深度相机模型在 Jetty 下有对应实现。

升级时只改：

```bash
GZ_VERSION=jetty
GZ_PKG=gz-jetty
```

然后重装 Gazebo、重编 `ros_gz`、跑 `verify.sh`。

## 4. 扩容磁盘后迁移

### 方案 A：整个项目搬到大盘（推荐）

```bash
mv ~/flight_project /data/flight_project
cd /data/flight_project
./scripts/verify.sh
```

脚本使用相对路径，不需要改配置。

### 方案 B：只把大型目录搬走

```bash
mkdir -p /data/flight_build
mv PX4-Autopilot Micro-XRCE-DDS-Agent /data/flight_build/
mv build install /data/flight_build/ 2>/dev/null || true

export WORKSPACE_ROOT=/data/flight_build
./scripts/verify.sh
```

`WORKSPACE_ROOT` 会用于：

- `PX4-Autopilot/`
- `Micro-XRCE-DDS-Agent/`
- ROS 的 `build/`、`install/`

`build_ros.sh` 会自动把 colcon 的 build/install 指到 `WORKSPACE_ROOT`。

### 方案 C：符号链接

```bash
mv PX4-Autopilot /data/PX4-Autopilot
ln -s /data/PX4-Autopilot PX4-Autopilot
```

简单，但跨机器/备份时容易断链；优先用 A 或 B。

## 5. 扩容内存后

修改 `versions.env`：

```bash
JOBS=auto
```

或：

```bash
JOBS=16
```

然后重新编译。`build_px4.sh` 会通过 `CMAKE_BUILD_PARALLEL_LEVEL` 限制并行度。

## 6. 可选：Docker 并行验证

如果以后想同时保留 Humble 和 Jazzy：

```
docker/
├── humble/Dockerfile
├── jazzy/Dockerfile
└── run.sh
```

要点：

- `--network host`（uXRCE、Gazebo transport）
- `--gpus all` + X11 socket（Gazebo GUI）
- 挂载 `flight_project/` 和工作区
- 每个镜像固定 `versions.env` 的快照

这样新版本可以先在容器里跑 `verify.sh`，通过后再切主机环境。

## 7. 回滚

```bash
git checkout -- versions.env
./scripts/install_px4.sh
./scripts/build_px4.sh gz_x500_flow
./scripts/fetch_deps.sh
./scripts/build_ros.sh
./scripts/verify.sh
```

如果 PX4 目录已被切到新 tag，`install_px4.sh` 会 checkout 回旧 tag 并更新 submodule。

## 升级检查清单

- [ ] `versions.env` 已更新
- [ ] `PX4-Autopilot` tag 正确
- [ ] `px4_msgs` 分支与 PX4 匹配
- [ ] `GZ_VERSION` 与已安装 Gazebo 匹配
- [ ] `ros_gz` 编译时使用了正确的 `GZ_VERSION`
- [ ] `verify.sh` 无 `[FAIL]`
- [ ] SITL 能起飞
- [ ] 光流有数据
- [ ] 深度图有点云
- [ ] Web 遥控器能控制
- [ ] 失败时有旧版本可回滚
