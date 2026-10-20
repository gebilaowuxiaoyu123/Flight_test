# 配套教材：ROS 2 + PX4 + Gazebo 四旋翼仿真

这是一套可以“断点续做”的教材：每一章都有明确的前置条件、操作步骤、预期输出和验收标准。即使中途换人、换机器或隔几周再回来，也能从 [PROGRESS.md](../PROGRESS.md) 和 `./scripts/verify.sh` 找到当前位置。

## 怎么用这套教材

1. **按章节顺序做**：后一章默认依赖前一章的验收项，不要跳章。
2. **每章结束先验收**：运行 `./scripts/verify.sh`，确认 `[FAIL]` 为 0；`[SKIP]` 表示还没到那一步，不是错误。
3. **卡住先查第九章**：[09-troubleshooting.md](09-troubleshooting.md) 按症状组织，覆盖了大部分常见问题。
4. **升级前先读第八章**：[08-upgrade.md](08-upgrade.md) 有版本矩阵和回滚步骤。

## 章节状态

| 章节 | 内容 | 状态 |
|---|---|---|
| [00-overview.md](00-overview.md) | 目标、架构、数据流、术语 | ✅ 已完成 |
| [01-environment.md](01-environment.md) | Gazebo / PX4 / ROS 2 / Agent 安装 | ✅ 已完成 |
| [02-px4-sitl.md](02-px4-sitl.md) | 首次 SITL、PX4 shell、光流机型 | ✅ 已完成 |
| [03-ros2-bridge.md](03-ros2-bridge.md) | uXRCE-DDS、px4_msgs、话题 | ✅ 已完成 |
| [04-rc-ui.md](04-rc-ui.md) | 遥控器原理与 Web UI 设计 | 🟡 设计与接口已完成，实现随 P5 补充 |
| [05-optical-flow.md](05-optical-flow.md) | 光流原理、EKF2、实验 | ✅ 原理与步骤已完成 |
| [06-depth-camera.md](06-depth-camera.md) | 深度相机、ros_gz、点云/避障 | ✅ 原理与步骤已完成 |
| [07-integration.md](07-integration.md) | 光流 + 深度合并模型 | ✅ 步骤已完成 |
| [08-upgrade.md](08-upgrade.md) | PX4 / ROS / Gazebo 升级与扩容 | ✅ 已完成 |
| [09-troubleshooting.md](09-troubleshooting.md) | 常见错误排查 | ✅ 已完成 |
| [10-exercises.md](10-exercises.md) | 练习与总验收 | ✅ 已完成 |

> 状态说明：✅ 表示命令和验收步骤可直接照做；🟡 表示设计已定，代码实现会在对应里程碑补齐。

## 约定

- 所有命令默认在项目根目录 `flight_project/` 下执行。
- `$PROJECT_ROOT` 指项目根目录；脚本内部会自动计算，不依赖绝对路径。
- 需要 `sudo` 的命令会明确标出；脚本会提示输入密码。
- 版本号只在 [versions.env](../versions.env) 里改，不要散落在脚本里。
- 涉及“预期输出”的地方，输出可能因版本略有差异，重点是看关键字段是否存在。

## 断点续做流程

```bash
cd ~/flight_project
./scripts/verify.sh          # 看当前哪些项 PASS / FAIL / SKIP
cat PROGRESS.md              # 看下一步
```

如果 `verify.sh` 显示某个组件缺失，按提示运行对应脚本即可；不需要从头重装。

## 最终形态

完成后你会得到：

- 一条命令启动 PX4 + Gazebo 四旋翼 SITL；
- 浏览器里的双摇杆遥控器，能解锁、切换 POSCTL、Kill；
- 无 GPS 下基于光流 + 测距的稳定悬停；
- 深度相机图像、点云和“前方障碍距离”；
- 一套能在 PX4 / ROS 2 / Gazebo 升级后复用的代码和验证脚本。
