# 06 · 深度相机：从深度图到障碍距离

## 学习目标

- 理解深度相机输出什么、和普通 RGB 相机的区别
- 用 `gz_x500_depth` 启动 OakD-Lite 深度相机
- 用 `ros_gz_bridge` 把 Gazebo 深度话题桥到 ROS 2
- 在 RViz2 看深度图和点云
- 把深度图变成“前方障碍距离”

## 原理

### 1. 深度相机怎么得到距离

OakD-Lite 是双目立体相机：

```text
视差 d = 左图与右图的像素差
深度 Z = (焦距 f × 基线 b) / d
```

Gazebo 的 `depth_camera` 传感器直接输出：

- 深度图（`R_FLOAT32`，单位米）
- 点云（`PointCloudPacked`）
- `CameraInfo`（内参）

### 2. 和 RGB 的区别

| 数据 | 内容 | 用途 |
|---|---|---|
| RGB 图像 | 颜色 | 识别、视觉里程计 |
| 深度图 | 每个像素的距离 | 避障、三维重建 |
| 点云 | 三维坐标 | 建图、障碍物检测 |

PX4 默认不消费深度图；它需要伴飞计算机/ROS 2 节点来处理。

## 模型

`x500_depth` 在 x500 上加了 `OakD-Lite`：

- RGB 相机：`IMX214`，1920×1080，30 Hz
- 深度相机：`StereoOV7251`，640×480，30 Hz，`R_FLOAT32`
- 深度话题：`depth_camera`

## 1. 启动

```bash
./scripts/run_sitl_depth.sh
```

等价于：

```bash
cd PX4-Autopilot
make px4_sitl gz_x500_depth
```

## 2. 看 Gazebo 话题

```bash
gz topic -l | grep -i depth
```

可能看到：

```text
/depth_camera
/depth_camera/points
/depth_camera/camera_info
```

名字以实际输出为准；下面的桥接命令要按实际名字改。

也可以直接用 Gazebo 看：

```bash
gz topic -e -t /depth_camera
```

## 3. 编译 ros_gz（Humble + Harmonic）

Humble 的 `ros-humble-ros-gz` 二进制是 Fortress 版，不能用于 Harmonic。必须源码编译：

```bash
./scripts/fetch_deps.sh
export GZ_VERSION=harmonic
./scripts/build_ros.sh
```

如果 rosdep 找不到 Harmonic 的键，手动补：

```bash
sudo apt install -y libgz-msgs10-dev libgz-transport13-dev libgz-sim8-dev
```

## 4. 桥接

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

检查：

```bash
ros2 topic list | grep depth
ros2 topic hz /depth_camera
ros2 topic echo /depth_camera --once | head
```

### 关于 QoS

Gazebo 传感器通常是 best-effort。RViz2 订阅时如果没图像，把 Reliability 设为 `Best Effort`。

## 5. RViz2 可视化

```bash
rviz2
```

添加：

1. **Image**：Topic 选 `/depth_camera`，Color Scheme 选 `mode: depth`（或 `rgb8`）
2. **PointCloud2**：Topic 选 `/depth_camera/points`
3. Fixed Frame 设为 `odom` 或 `base_link`（按 TF 实际情况）

如果点云颜色不对，设置 `Color Transformer: AxisColor` 或 `Intensity`。

## 6. 深度图 → 障碍距离

### 方案 A：depthimage_to_laserscan

```bash
sudo apt install -y ros-humble-depthimage-to-laserscan
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
  --ros-args \
  -r image:=/depth_camera \
  -r camera_info:=/depth_camera/camera_info \
  -r scan:=/scan
```

然后在 RViz2 里看 `/scan`。

### 方案 B：自己写最小深度节点

伪代码：

```python
# 订阅 /depth_camera (sensor_msgs/Image, encoding=32FC1)
# 取图像中央 ROI（如 40% 宽、60% 高）
# 过滤 NaN/0/超过 max_range 的值
# 取最小值 → 发布 std_msgs/Float32 到 /obstacle_distance
```

这个节点会放在 `src/flight_perception`，P4 实现。

## 7. 实验

1. 让飞机悬停，在正前方放一个障碍物，观察 `/obstacle_distance` 变化。
2. 缓慢前飞，记录距离从 5 m 到 1 m 的变化。
3. 对比深度图中心和边缘的精度。
4. 把 `max_range` 设为 5 m，观察远处噪声是否被过滤。

## 常见问题

| 症状 | 原因 | 处理 |
|---|---|---|
| `gz topic -l` 没有 depth 话题 | 机型不对/相机未加载 | 确认 `gz_x500_depth`；看 Gazebo 日志 |
| 桥接后 ROS 没话题 | 话题名写错 | 用 `gz topic -l` 的实际名字 |
| 有话题没数据 | QoS 不匹配 | RViz/echo 用 Best Effort |
| `ros_gz_bridge` 启动报 ABI 错误 | 用了 Fortress 二进制 | 卸载 `ros-humble-ros-gz*`，源码编译 Harmonic 版 |
| 点云为空 | 深度图全是 NaN/超出范围 | 靠近障碍物；检查 `max_range` |
| RViz2 黑屏 | 渲染/GPU | 用 `LIBGL_ALWAYS_SOFTWARE=1` 试一次 |

## 本章验收

- [ ] `gz_x500_depth` 能启动
- [ ] `gz topic -l` 能看到深度话题
- [ ] `ros_gz_bridge` 桥接后 `/depth_camera` 有图像
- [ ] RViz2 能看到深度图和点云
- [ ] `depthimage_to_laserscan` 或自定义节点能输出障碍距离
- [ ] 能解释视差、基线、深度三者的关系
