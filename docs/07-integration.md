# 07 · 合并模型：光流 + 深度相机同机

## 学习目标

- 理解 PX4 的 Gazebo 模型如何组合
- 用 `px4_overlay/` 维护自定义模型，不污染上游 PX4
- 让一架飞机同时提供光流、测距和深度相机

## 为什么要合并

`x500_flow` 和 `x500_depth` 是两个独立机型：

- `x500_flow`：光流 + LW20，适合做无 GPS 定位
- `x500_depth`：OakD-Lite 深度相机，适合做避障

合并后，一次仿真可以同时观察：

- `/fmu/out/vehicle_optical_flow`
- `/fmu/out/distance_sensor`
- `/depth_camera`（经 `ros_gz_bridge`）

## 目录

```
px4_overlay/
├── models/
│   └── x500_flow_depth/
│       └── model.sdf
└── airframes/              # 可选：自定义 airframe
```

## 1. 创建模型

以 `x500_flow` 为基准，加入 `OakD-Lite`：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="x500_flow_depth">
    <self_collide>false</self_collide>

    <include merge="true">
      <uri>x500</uri>
    </include>

    <!-- 下视光流 -->
    <include merge="true">
      <uri>model://optical_flow</uri>
      <pose relative_to="base_link">0.03 0 -0.1 0 0 0</pose>
    </include>

    <!-- 下视激光测距 -->
    <include merge="true">
      <uri>model://LW20</uri>
      <pose relative_to="base_link">0 0 -0.079 0 1.57 0</pose>
    </include>

    <!-- 前向深度相机 -->
    <include merge="true">
      <uri>model://OakD-Lite</uri>
      <pose relative_to="base_link">0.12 0.03 0.242 0 0 0</pose>
    </include>

    <!-- 下视测距传感器本体：从 x500_flow 复制，不能省 -->
    <link name="lidar_sensor_link">
      <pose relative_to="base_link">0 0 -0.05 0 1.57 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <iyy>0.00001</iyy>
          <izz>0.00001</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <sensor name="lidar" type="gpu_lidar">
        <gz_frame_id>lidar_sensor_link</gz_frame_id>
        <pose>0 0 0 3.14 0 0</pose>
        <update_rate>50</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>100.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <always_on>1</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>

    <!-- 关节：直接复用上游模型里的定义 -->
    <joint name="optical_flow_joint" type="fixed">
      <parent>base_link</parent>
      <child>flow_link</child>
      <pose relative_to="base_link">0 0 -0.1 0 1.5707 0</pose>
    </joint>

    <joint name="lidar_model_joint" type="fixed">
      <parent>base_link</parent>
      <child>lw20_link</child>
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
    </joint>

    <joint name="lidar_sensor_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_sensor_link</child>
    </joint>

    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">0.12 0.03 0.242 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

> 最稳妥的做法是直接从 `PX4-Autopilot/Tools/simulation/gz/models/x500_flow/model.sdf` 复制，再把 `x500_depth` 里的 OakD-Lite include 和 CameraJoint 合并进去。

## 2. 运行

```bash
./scripts/run_sitl_combined.sh
```

脚本等价于：

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/px4_overlay/models:$GZ_SIM_RESOURCE_PATH
export PX4_GZ_MODEL=x500_flow_depth
cd PX4-Autopilot
make px4_sitl gz_x500_flow
```

为什么 airframe 还用 `gz_x500_flow`：

- 4021 的 airframe 已经配置好“关闭 GPS、启用光流”的参数；
- `PX4_GZ_MODEL` 只替换 Gazebo 模型，不改变 PX4 参数。

## 3. 验收

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic list | grep -E 'vehicle_optical_flow|distance_sensor'
gz topic -l | grep -i depth
```

再桥接深度：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

- [ ] 光流话题有数据
- [ ] 测距话题有数据
- [ ] 深度图有点云
- [ ] 起飞后光流和深度同时可用

## 4. 如果 `PX4_GZ_MODEL` 不生效

某些版本/启动脚本会优先使用 airframe 里的 `PX4_SIM_MODEL`。这时用自定义 airframe：

1. 复制 `4021_gz_x500_flow` 为 `4022_gz_x500_flow_depth`；
2. 把 `PX4_SIM_MODEL=x500_flow` 改成 `x500_flow_depth`；
3. 在 `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` 里注册；
4. 重新 `make px4_sitl gz_x500_flow_depth`。

自定义 airframe 也放 `px4_overlay/airframes/`，并写一个叠加脚本。这样升级 PX4 后可以重放。

## 5. 进阶

- 给深度相机加一个前向倾斜角，模拟避障视角；
- 给光流相机加 `SENS_FLOW_ROT` 对应的安装旋转；
- 用 `ros_gz_bridge` 同时桥 RGB 和深度，做视觉里程计对比；
- 在 `flight_perception` 里把深度图转成局部代价地图。

## 本章验收

- [ ] `x500_flow_depth` 能被 Gazebo 加载
- [ ] 光流、测距、深度三种数据同时在线
- [ ] 能在 RViz2 看到深度点云
- [ ] 能解释 `PX4_GZ_MODEL` 与 airframe 的分工
