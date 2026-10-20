# 05 · 光流：无 GPS 下的速度估计

## 学习目标

- 理解光流为什么只能测角速度，必须配合测距才能得到线速度
- 知道 PX4 的 `vehicle_optical_flow` 消息里每个字段的含义
- 会配置/检查 EKF2 光流参数
- 会做“悬停 + 平移 + 对比速度”的实验

## 原理

### 1. 光流测的是什么

下视相机连续两帧图像中，地面纹理的像素位移：

```
像素位移 / 时间  →  角速度 ω（rad/s）
```

PX4 的光流消息给出的是**累积角度**（弧度）：

```text
pixel_flow[0]  # x 方向累积光流
pixel_flow[1]  # y 方向累积光流
integration_timespan_us
```

角速度近似：

```text
ω_x ≈ pixel_flow[0] / (integration_timespan_us * 1e-6)
ω_y ≈ pixel_flow[1] / (integration_timespan_us * 1e-6)
```

### 2. 为什么需要测距

光流只知道“转得多快”，不知道“离地多高”。同样的像素位移：

- 离地 1 m：对应较小的线速度
- 离地 5 m：对应较大的线速度

小角度近似：

```text
v ≈ ω × h
```

`h` 来自下视测距（`distance_sensor`）。`x500_flow` 用 LW20 激光测距提供这个高度。

### 3. 为什么要补偿机体旋转

相机随飞机一起旋转，图像运动里混入了飞机自身的滚转/俯仰/偏航。PX4 用陀螺数据（`delta_angle`）做补偿，只留下“平移引起的光流”。

### 4. EKF2 怎么用

EKF2 把光流速度作为水平速度观测，与 IMU、气压计融合：

- 无 GPS 时，光流提供水平速度；
- 高度来自气压计/测距；
- 位置会漂移（光流是速度观测，不是绝对位置）。

## 仿真模型

`x500_flow` 由三部分组成：

| 部分 | 作用 |
|---|---|
| `x500` | 四旋翼机体 |
| `optical_flow` | 下视相机 + `gz:type="optical_flow"` 传感器 |
| `LW20` | 下视激光测距 |

airframe `4021_gz_x500_flow` 默认关闭 GPS：

```text
SYS_HAS_GPS=0
SIM_GPS_USED=0
EKF2_GPS_CTRL=0
```

## 关键参数

```text
param show EKF2_OF*
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `EKF2_OF_CTRL` | 1 | 启用光流融合 |
| `EKF2_OF_QMIN` | 1 | 空中使用光流的最低质量 |
| `EKF2_OF_QMIN_GND` | 0 | 地面使用光流的最低质量 |
| `EKF2_OF_N_MIN` | 0.15 | 最高质量时的噪声（rad/s） |
| `EKF2_OF_N_MAX` | 0.5 | 最低质量时的噪声（rad/s） |
| `EKF2_OF_GATE` | 3.0 | 新息一致性门限（SD） |
| `EKF2_OF_DELAY` | 20 | 光流相对 IMU 的延迟（ms） |
| `EKF2_OF_POS_X/Y/Z` | 0 | 光流传感器相对机体的安装位置 |
| `EKF2_OF_GYR_SRC` | 0 | 0=自动，1=内部陀螺 |

`x500_flow` 的安装位置大约在 `(0.03, 0, -0.1)`；如果你改了模型，记得同步 `EKF2_OF_POS_*`。

## 实验步骤

### 1. 启动

```bash
./scripts/run_sitl_flow.sh
MicroXRCEAgent udp4 -p 8888     # 另一个终端
# PX4 shell:
uxrce_dds_client start -t udp -p 8888
```

### 2. 确认数据

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /fmu/out/vehicle_optical_flow
ros2 topic echo /fmu/out/vehicle_optical_flow --once
ros2 topic echo /fmu/out/distance_sensor --once
```

期望：

- 光流约 50 Hz；
- `quality > 0`；
- `distance_m` 在 0.1–100 m 之间；
- `pixel_flow` 在飞机静止时接近 0，移动时非零。

### 3. 起飞

```text
commander takeoff
```

### 4. 平移并观察

用 Web 遥控器或 `commander` 让飞机前后/左右移动，记录：

- `vehicle_optical_flow.pixel_flow`
- `vehicle_optical_flow.distance_m`
- `vehicle_local_position.vx/vy`

### 5. 对比

写一个简单节点（P3 实现）把以下数据记成 CSV：

```text
t, flow_x, flow_y, distance_m, quality, vx, vy
```

计算：

```text
v_flow_x ≈ (pixel_flow[0] / dt) * distance_m
v_flow_y ≈ (pixel_flow[1] / dt) * distance_m
```

与 `vehicle_local_position.vx/vy` 对比。注意符号约定，可能需要取负。

### 6. 关掉光流做对照

```text
param set EKF2_OF_CTRL 0
```

再飞一次，观察无 GPS + 无光流时水平速度是否快速漂移。实验后改回：

```text
param set EKF2_OF_CTRL 1
param save
```

## 误差来源

| 来源 | 表现 | 改善 |
|---|---|---|
| 地面纹理弱/重复 | `quality` 低，漂移大 | 换有纹理的地面/增加光照 |
| 运动模糊 | 快速移动时 quality 下降 | 降低速度/提高快门 |
| 测距噪声 | 速度尺度抖动 | 检查 `distance_sensor`，加滤波 |
| 安装偏移/旋转 | 平移时出现额外耦合 | 设置 `EKF2_OF_POS_*`、`SENS_FLOW_ROT` |
| 延迟 | 高速时相位滞后 | 调 `EKF2_OF_DELAY` |
| 光照变化 | 光流突变 | 避免强反光/阴影 |

## 练习

1. 在 1 m 和 3 m 高度以相同速度平移，比较 `pixel_flow` 大小，验证 `v ≈ ωh`。
2. 把地面换成纯色（修改 world），观察 `quality` 和漂移。
3. 故意把 `EKF2_OF_POS_X` 设错，观察平移时是否出现额外速度。
4. 记录一次“光流关闭 vs 开启”的悬停漂移对比。

## 本章验收

- [ ] `gz_x500_flow` 能启动，`vehicle_optical_flow` 有数据
- [ ] `distance_sensor` 有有效高度
- [ ] 无 GPS 下能起飞并短时悬停
- [ ] 能画出一张光流推算速度 vs EKF2 速度的对比图
- [ ] 能解释 `quality`、`distance_m`、`integration_timespan_us` 的作用
