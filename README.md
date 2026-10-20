# flight_project

ROS 2 + PX4（四旋翼）+ Gazebo Harmonic 的仿真、遥控器界面与感知实验项目。

## 最终目标

- 在 Gazebo 中运行 PX4 四旋翼 SITL
- 用浏览器里的“遥控器界面”手动飞行（双虚拟摇杆、解锁、模式、Kill）
- 理解并验证光流（Optical Flow）辅助定位
- 理解并验证深度相机（Depth Camera）与障碍距离
- 保持可升级：PX4 / ROS 2 / Gazebo 都能平滑迁移到新版本

## 当前状态

**P0：仓库脚手架、版本锁定、验证脚本、教材骨架** ✅

进度与断点续做见 [PROGRESS.md](PROGRESS.md)。

## 快速开始

```bash
# 1. 查看当前环境与缺失项
./scripts/verify.sh

# 2. 安装 Gazebo Harmonic（需要 sudo）
./scripts/install_gazebo.sh

# 3. 克隆并安装 PX4 v1.16.2（需要 sudo）
./scripts/install_px4.sh

# 4. 首次编译并启动光流机型
./scripts/build_px4.sh gz_x500_flow
./scripts/run_sitl_flow.sh
```

完整步骤从 [docs/README.md](docs/README.md) 开始。

## 目录结构

```
flight_project/
├── versions.env          # 所有版本号的唯一来源
├── PX4-Autopilot/        # 上游 PX4（脚本克隆，git 忽略）
├── px4_overlay/          # 自定义模型 / airframe，升级 PX4 后重新叠加
├── src/                  # ROS 2 工作区
├── web/                  # Web 遥控器前端
├── scripts/              # 安装 / 构建 / 运行 / 验证
├── docs/                 # 配套教材
└── PROGRESS.md           # 断点续做
```

## 环境基线

| 项 | 当前 |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4 | v1.16.2 |
| Gazebo | Harmonic (`gz-sim8`) |
| 内存 | 30 GB（无 swap，编译默认 `-j8`） |
| 磁盘 | 约 29 GB 可用，整套预计 8–11 GB |

## 升级路线

见 [docs/08-upgrade.md](docs/08-upgrade.md)。简表：

| 阶段 | 推荐组合 |
|---|---|
| 现在 | 22.04 + Humble + PX4 v1.16.2 + Harmonic |
| 扩容后 | 24.04 + Jazzy + PX4 v1.17/1.18 + Harmonic（`ros_gz` 可用二进制） |
| 更远 | Gazebo Jetty + ROS 2 Kilted/Rolling |
