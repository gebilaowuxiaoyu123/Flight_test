# 02 · 首次 SITL：让四旋翼飞起来

## 学习目标

- 启动 PX4 SITL + Gazebo Harmonic
- 认识 PX4 shell 的常用命令
- 理解 `gz_x500_flow` 这个机型为什么适合做光流实验
- 用 `commander` 完成起飞/降落

## 前置检查

```bash
./scripts/verify.sh
gz sim --version
git -C PX4-Autopilot describe --tags --always
```

要求：Gazebo 8.x、PX4 v1.16.2、`[FAIL]` 为 0。

## 1. 启动光流机型

```bash
./scripts/run_sitl_flow.sh
```

等价于：

```bash
cd PX4-Autopilot
make px4_sitl gz_x500_flow
```

第一次会先编译，然后：

- Gazebo 打开一个 x500 四旋翼；
- PX4 在终端进入 `pxh>` shell；
- 默认机型是 `4021_gz_x500_flow`。

无 GUI（服务器/远程）：

```bash
HEADLESS=1 ./scripts/run_sitl_flow.sh
```

## 2. 这个机型做了什么

`ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_flow`：

```sh
PX4_SIM_MODEL=x500_flow
. .../4001_gz_x500

param set-default SYS_HAS_GPS 0
param set-default SIM_GPS_USED 0
param set-default EKF2_GPS_CTRL 0
```

模型 `x500_flow` 由 `PX4-gazebo-models` 提供：

- `optical_flow`：下视光流相机 + `gz:type="optical_flow"` 自定义传感器
- `LW20`：下视激光测距
- 光流与测距的组合，正好满足 EKF2 在无 GPS 下估计水平速度的条件

## 3. PX4 shell 常用命令

在 `pxh>` 提示符下：

```text
commander status          # 解锁状态、模式
commander takeoff         # 自动起飞到默认高度
commander land            # 降落
commander arm             # 解锁
commander disarm          # 上锁
listener vehicle_optical_flow   # 看光流数据
listener distance_sensor        # 看测距数据
listener vehicle_local_position # 看本地位置/速度
param show EKF2_OF*       # 光流相关参数
param show COM_RC_IN_MODE # 手动控制输入模式
```

### 看光流是否真的在出数据

```text
listener vehicle_optical_flow
```

关键字段：

| 字段 | 含义 |
|---|---|
| `pixel_flow[0/1]` | 累积光流（弧度），正值为绕机体轴右手旋转 |
| `delta_angle[0/1]` | 陀螺积分角度（弧度），用于补偿旋转 |
| `distance_m` | 光流场中心到地面的距离 |
| `quality` | 0（差）~255（好） |
| `integration_timespan_us` | 累积时间（微秒） |

如果一直为 0 或 `quality=0`，见第九章。

## 4. 起飞与降落

最简单的方式是在 PX4 shell：

```text
commander takeoff
```

观察 Gazebo 里的飞机升到约 2.5 m。然后：

```text
commander land
```

如果想用 QGroundControl：

- QGC 默认监听 UDP 14550；
- PX4 SITL 会自动把 MAVLink 发到本机 14550；
- 打开 QGC 应能自动连接。

## 5. 无 GPS 悬停验证

由于 airframe 已经关闭 GPS：

```text
listener vehicle_local_position
```

看 `xy_valid`、`vx`、`vy`。在光流 + 测距正常时，EKF2 应该能在无 GPS 下给出水平速度估计。

> 注意：光流定位会有漂移，这是正常的；它估计的是速度，不是绝对位置。

## 本章验收

- [ ] `./scripts/run_sitl_flow.sh` 能打开 Gazebo 并进入 `pxh>`
- [ ] `listener vehicle_optical_flow` 有非零 `pixel_flow` 和 `quality`
- [ ] `listener distance_sensor` 有 `current_distance`
- [ ] `commander takeoff` 能起飞，`commander land` 能降落
- [ ] `param show EKF2_GPS_CTRL` 为 0（GPS 关闭）

## 常见问题

| 症状 | 处理 |
|---|---|
| `ninja: error: unknown target 'gz_x500_flow'` | `make distclean` 后重试；确认 PX4 是 v1.16+ |
| Gazebo 黑屏/崩溃 | 用 `HEADLESS=1` 先验证逻辑；GUI 问题见第九章 |
| 飞机不动/无法解锁 | 看 `commander status` 的 arming checks；SITL 一般可直接解锁 |
| 光流一直为 0 | 确认机型是 `gz_x500_flow`，不是 `gz_x500` |
