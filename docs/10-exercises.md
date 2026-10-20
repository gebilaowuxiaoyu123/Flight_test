# 10 · 练习与总验收

## 使用方式

每完成一章，做对应练习；不要求全部做完，但至少完成“必做”。总验收用于确认整个系统可交付。

## 第 1 章：环境

**必做**

- [ ] 画出组件依赖图：Gazebo / PX4 / ROS 2 / Agent / 浏览器
- [ ] 解释为什么 Humble 的 `ros_gz` 不能直接用 Harmonic
- [ ] 用 `verify.sh` 记录一次 `PASS/FAIL/SKIP` 快照

**选做**

- [ ] 手动执行一遍 `install_gazebo.sh` 里的 apt 命令，理解每一步
- [ ] 给 PX4 编译加 8G swap，比较编译时间

## 第 2 章：SITL

**必做**

- [ ] `gz_x500_flow` 起飞、悬停、降落
- [ ] 用 `listener` 读出 `vehicle_optical_flow` 和 `distance_sensor`
- [ ] 解释 `SYS_HAS_GPS=0`、`SIM_GPS_USED=0`、`EKF2_GPS_CTRL=0` 的作用

**选做**

- [ ] 用 QGC 连接 SITL
- [ ] `HEADLESS=1` 跑一次，比较内存占用

## 第 3 章：ROS 2 桥

**必做**

- [ ] 启动 Agent + PX4 客户端，列出 `/fmu/*`
- [ ] 用 `ros2 topic hz` 测光流频率
- [ ] 用 `ros2 topic pub` 发一次解锁命令

**选做**

- [ ] 写一个节点，订阅 `vehicle_status` 并打印 armed 状态
- [ ] 故意用错 `px4_msgs` 分支，观察编译/运行错误

## 第 4 章：Web 遥控器

**必做**

- [ ] 实现 50Hz 摇杆发布
- [ ] 实现 deadman + 断线置零
- [ ] 实现解锁/上锁/POSCTL/Kill
- [ ] 在 SITL 中完成一次手动起飞、平移、降落

**选做**

- [ ] 加实体手柄（Gamepad API）
- [ ] 加摇杆低通滤波和死区
- [ ] 手机浏览器访问

## 第 5 章：光流

**必做**

- [ ] 记录 1 m 和 3 m 高度、相同速度下的 `pixel_flow`
- [ ] 计算 `v ≈ ωh`，与 `vehicle_local_position.vx/vy` 对比
- [ ] 关闭 `EKF2_OF_CTRL` 做一次对照

**选做**

- [ ] 换纯色地面，观察 `quality`
- [ ] 故意设错 `EKF2_OF_POS_X`，观察耦合
- [ ] 写 CSV 记录 + 画图脚本

## 第 6 章：深度相机

**必做**

- [ ] `gz_x500_depth` + `ros_gz_bridge` 出深度图
- [ ] RViz2 看到点云
- [ ] 计算前方最小深度并发布 `/obstacle_distance`

**选做**

- [ ] `depthimage_to_laserscan` 输出 `/scan`
- [ ] 对比中心 ROI 与全图最小值的差异
- [ ] 把深度图转成局部代价地图

## 第 7 章：合并模型

**必做**

- [ ] `x500_flow_depth` 同时输出光流、测距、深度
- [ ] 一次起飞同时观察三种数据

**选做**

- [ ] 自定义 airframe，不依赖 `PX4_GZ_MODEL`
- [ ] 给深度相机加倾斜角

## 第 8 章：升级

**必做**

- [ ] 在 `versions.env` 里模拟一次 PX4 小版本升级，跑通 `verify.sh`
- [ ] 写一页“升级检查清单”贴在项目里

**选做**

- [ ] 用 Docker 跑 Jazzy + Harmonic
- [ ] 把构建目录迁到大盘，验证 `WORKSPACE_ROOT`

## 总验收清单

### 功能

- [ ] `./scripts/run_sitl_flow.sh` 能启动并起飞
- [ ] 浏览器遥控器能控制
- [ ] 光流数据有效，无 GPS 能短时悬停
- [ ] 深度图、点云、障碍距离可用
- [ ] 合并模型三者同时可用

### 工程

- [ ] `versions.env` 是唯一版本来源
- [ ] `PX4-Autopilot/` 没有手改（改动都在 `px4_overlay/`）
- [ ] `px4_adapter.py` 是唯一接触 `px4_msgs` 的业务代码
- [ ] `verify.sh` 无 `[FAIL]`
- [ ] `PROGRESS.md` 与实际进度一致
- [ ] 所有脚本使用相对路径

### 安全

- [ ] Deadman 失效时摇杆归零
- [ ] 断线 300ms 内停止输入
- [ ] Kill 随时可用
- [ ] 解锁需要油门最低 + 模式正确

## 结课项目（可选）

实现一个“自动起飞 → 定高悬停 → 前飞 3 m → 测障碍距离 → 返航降落”的脚本，要求：

- 用 `/fmu/in/vehicle_command` 和 `ManualControlSetpoint` 或 Offboard 模式；
- 全程记录光流与深度数据；
- 异常时自动上锁；
- 用 `verify.sh` 增加一项端到端检查。
