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
**P1：PX4 v1.16.2 已克隆，等待 sudo 安装 Gazebo 与系统依赖** 🟡

进度与断点续做见 [PROGRESS.md](PROGRESS.md)。

## 快速开始

```bash
# 1. 查看当前环境与缺失项
./scripts/verify.sh

# 2. 安装 Gazebo Harmonic（需要 sudo）
./scripts/install_gazebo.sh

# 3. 克隆并安装 PX4 v1.16.2 + 依赖（容错版，需要 sudo）
./scripts/install_px4.sh

# 4. 首次编译并启动光流机型
./scripts/build_px4.sh gz_x500_flow
./scripts/run_sitl_flow.sh
```

**只想跑起来 / 飞一飞（先装一次短命令）：**

```bash
./scripts/install_aliases.sh     # 之后重开终端
flight                           # 起整条链路
tele                             # 终端实时看姿态
fly takeoff                      # 解锁并爬到 1.2 m
fly land                         # 降落上锁
```

**想录演示视频：**

```bash
./scripts/run_show.sh --record   # 起全部服务并录 180 s 三视角成片
```

代码按什么顺序读、每条命令在干什么，见 [docs/12-how-to-read-and-run.md](docs/12-how-to-read-and-run.md)。

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

## 从零复现手册

**如果你要自己从零重装、或者我不在场时需要排查问题，先看 [REBUILD.md](REBUILD.md)。**
里面有：一条命令启动、每个阶段的命令与验收、我踩过的 10 个坑、故障决策树、速查表。

## 想换成 C++？

看 [docs/11-cpp-rewrite.md](docs/11-cpp-rewrite.md)：从"怎么读这个仓库"讲到
每一步写什么代码、怎么验证、我踩过哪些坑。`src/flight_ui_cpp/` 与
`src/flight_perception_cpp/` 是配套的参考实现（已编译、已通过同一套端到端验收）。

## 一键启动完整演示

```bash
./scripts/run_demo.sh
# 浏览器打开 http://127.0.0.1:8000/
```

脚本用 tmux 启动 Agent、PX4/Gazebo、深度桥、障碍距离、光流日志和 Web 遥控器；
重复运行会跳过已存在的会话。停止：

```bash
for s in agent sitl bridge depth flowlog ui; do tmux kill-session -t "$s" 2>/dev/null || true; done
```

操作顺序：按住“按住使能” → 油门在最低 → `TAKEOFF`；`LAND` 降落；`KILL` 紧急停。
