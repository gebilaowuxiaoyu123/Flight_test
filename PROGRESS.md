# 项目进度

当前阶段：**P1 进行中**（PX4 已克隆，等待 sudo 安装 Gazebo 与系统依赖）
最后更新：2026-10-21 02:15

## 断点续做（断线后从这里开始）

1. 运行 `./scripts/verify.sh`，确认哪些组件已就绪。
2. 打开 [docs/README.md](docs/README.md)，查看各章状态。
3. 从下面的“下一步”继续；不要跳章，后一章默认依赖前一章的验收项。
4. 每完成一步，更新本文件的勾选与“下一步”。

## 里程碑

- [x] **P0** 仓库脚手架、`versions.env`、`verify.sh`、教材骨架、git 初始化
- [ ] **P1** 安装 Gazebo Harmonic + PX4 v1.16.2，跑通 `gz_x500_flow`（进行中）
- [ ] **P2** ROS 2 桥：uXRCE-DDS + `px4_msgs`，看到 `/fmu/out/*`
- [ ] **P3** 光流实验：无 GPS 悬停，记录/对比光流数据
- [ ] **P4** 深度相机：`x500_depth` + `ros_gz` + RViz2 / 障碍距离
- [ ] **P5** Web 遥控器：双摇杆 + 解锁/模式/Kill + 遥测
- [ ] **P6** 合并模型 `x500_flow_depth`，光流与深度同机联调
- [ ] **P7** 升级演练：Humble→Jazzy / PX4 新版本 / 可选 Docker

## P1 详细状态

- [x] `PX4-Autopilot` 克隆到 `v1.16.2`（含 SITL 所需子模块）
- [x] `Tools/simulation/gz/models/x500_flow` 存在
- [x] `src/modules/mavlink/mavlink`、`uxrce_dds_client`、`gps/devices`、`libevents`、`crypto` 已初始化
- [x] `src/px4_msgs`（`release/1.16`）
- [x] `src/ros_gz`（`humble`）
- [ ] Gazebo Harmonic（`gz-harmonic`）
- [ ] GStreamer / protobuf / Java 等系统依赖
- [ ] PX4 Python 依赖（`jinja2`、`empy`、`kconfiglib`）
- [ ] `gz_x500_flow` 编译
- [ ] SITL 首次启动与光流数据

## 下一步（需要你执行的 sudo 步骤）

```bash
cd ~/flight_project
bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```

完成后：

```bash
./scripts/verify.sh
./scripts/build_px4.sh gz_x500_flow
HEADLESS=1 ./scripts/run_sitl_flow.sh
```

验收：Gazebo 出现 x500_flow，PX4 终端 `listener vehicle_optical_flow` 有数据。

## 已知约束

- `sudo` 需要密码，安装脚本会提示输入。
- 当前没有 swap；编译并行度默认 8，避免 OOM。
- 根分区约 29 GB 可用；不要把数据集/录包放进项目。
- 后续扩容后，可通过 `WORKSPACE_ROOT` 把 `PX4-Autopilot/`、`build/`、`install/` 迁到大盘。

## 决策记录

| 日期 | 决策 | 原因 |
|---|---|---|
| 2026-10-21 | 选 PX4 v1.16.2 + Gazebo Harmonic | v1.16 起有 `gz_x500_flow`，Harmonic 是 LTS；v1.15 仍是 EOL 的 Garden |
| 2026-10-21 | 不直接改 PX4 源码，用 `px4_overlay/` | 便于升级 PX4 |
| 2026-10-21 | 业务代码与 `px4_msgs` 解耦到 adapter | 消息字段变化时只改一处 |
| 2026-10-21 | Web 遥控器用 WebSocket + FastAPI | 浏览器/手机可用，控制链路可控 |
