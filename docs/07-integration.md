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
# PX4 v1.16 的 PX4_GZ_MODEL 已废弃，模型由 PX4_SIM_MODEL 选择
ln -sfn "$PWD/px4_overlay/models/x500_flow_depth" \
  PX4-Autopilot/Tools/simulation/gz/models/x500_flow_depth

export PX4_SIM_MODEL=gz_x500_flow_depth
export PX4_SYS_AUTOSTART=4021
export PX4_GZ_WORLD=flow        # px4_overlay/worlds/flow.sdf
export HEADLESS=1
cd PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
exec "$PWD/../../../bin/px4"
```

脚本 `scripts/run_sitl_combined.sh` 已经封装了这些步骤。关键点：

- PX4 的 `px4-rc.gzsim` 用 `${PX4_GZ_MODELS}/<model>/model.sdf` 发 spawn 请求，
  所以 overlay 模型必须软链进 PX4 自己的 `models/` 目录；
- airframe 仍用 4021（关闭 GPS、启用光流），只替换 Gazebo 模型；
- 本项目的 OakD-Lite 传感器直接内联在合并模型里。SDFormat 的
  `merge="true"` 在某些情况下不会保留深度相机，导致只有 `/depth_camera`
  的空话题但没有图像；内联后 `camera_link`/`StereoOV7251` 才会出现。

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
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

- [ ] 光流话题有数据
- [ ] 测距话题有数据
- [ ] 深度图有点云
- [ ] 起飞后光流和深度同时可用

## 4. 调试清单：模型加载了但没有传感器数据

按顺序检查：

1. `gz model -m x500_flow_depth_0 -l | grep camera_link` 是否出现；
2. `gz topic -l | grep depth_camera` 是否有 `/depth_camera`；
3. `gz topic -f -t /depth_camera -d 3` 是否约 30 Hz；
4. `ros2 topic echo /fmu/out/distance_sensor --once --qos-reliability best_effort`
   的 `orientation` 必须是 `25`（ROTATION_DOWNWARD_FACING），不能是 `100`；
5. `ros2 topic echo /fmu/out/vehicle_optical_flow --once ...` 的
   `distance_m` 不能是 `NaN`，否则 EKF 无法用光流做水平定位。

本项目遇到的两个坑：

- 深度相机 include 被合并后丢失 → 直接内联 OakD-Lite 的 link/sensor；
- 下视激光的安装旋转让 gz_bridge 把它标成 `ROTATION_CUSTOM(100)`，
  光流模块只接受 `ROTATION_DOWNWARD_FACING(25)`，因此 `distance_m=NaN`。
  已在上游 `GZBridge.cpp` 增加“+90° pitch 也算下视”的判断；
  该补丁记录在 `px4_overlay/px4_patches/`，升级 PX4 后需要重放。

## 5. 进阶

- 给深度相机加一个前向倾斜角，模拟避障视角；
- 给光流相机加 `SENS_FLOW_ROT` 对应的安装旋转；
- 用 `ros_gz_bridge` 同时桥 RGB 和深度，做视觉里程计对比；
- 在 `flight_perception` 里把深度图转成局部代价地图。

## 本章验收

- [ ] `x500_flow_depth` 能被 Gazebo 加载
- [ ] 光流、测距、深度三种数据同时在线
- [ ] 能在 RViz2 看到深度点云
- [ ] 能解释 `PX4_SIM_MODEL`、softlink 与 airframe 的分工
