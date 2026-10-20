# ROS 2 工作区

这里放 ROS 2 包。上游依赖由脚本克隆，git 忽略：

| 包 | 来源 | 作用 |
|---|---|---|
| `px4_msgs` | `PX4/px4_msgs` `release/1.16` | PX4 消息定义 |
| `ros_gz` | `gazebosim/ros_gz` `humble` | Gazebo ↔ ROS 2 桥（`GZ_VERSION=harmonic`） |

自研包（按里程碑创建）：

| 包 | 里程碑 | 作用 |
|---|---|---|
| `flight_bringup` | P5 | launch 文件、参数 |
| `flight_ui` | P5 | Web 遥控器后端 + 前端资源 |
| `flight_perception` | P3/P4 | 光流记录、深度转障碍距离 |

## 构建

```bash
cd ~/flight_project
./scripts/fetch_deps.sh
./scripts/build_ros.sh
source install/setup.bash
```

`build_ros.sh` 会设置 `GZ_VERSION=harmonic`，并把 `build/install` 放到 `WORKSPACE_ROOT`（默认项目根目录）。

## 约定

- 业务代码不要直接到处 import `px4_msgs`；统一走 `flight_ui/px4_adapter.py`。
- 优先使用 `rclpy` 的稳定 API，兼容 Humble/Jazzy。
- 新增包后，在 `docs/` 对应章节补上启动/验收命令。
