# px4_overlay

存放**不修改上游 PX4 源码**的定制内容：

```
px4_overlay/
├── models/          # 自定义 Gazebo 模型（如 x500_flow_depth）
└── airframes/       # 自定义 PX4 airframe 启动脚本（可选）
```

## 为什么用 overlay

PX4 通过 git submodule 管理 Gazebo 模型（`Tools/simulation/gz` → `PX4-gazebo-models`）。直接改子模块会导致：

- 升级 PX4 时冲突；
- 换机器后改动丢失；
- 无法回滚。

overlay 的原则：

1. 上游代码只读；
2. 我们的改动只放在这里；
3. 通过 `GZ_SIM_RESOURCE_PATH` / airframe 注册让 PX4 看到它们；
4. 升级 PX4 后重新叠加即可。

## 当前内容

| 路径 | 状态 |
|---|---|
| `models/x500_flow_depth/` | P6 实现（光流 + LW20 + OakD-Lite） |
| `airframes/` | 可选，P6 视 `PX4_GZ_MODEL` 是否生效决定 |

## 使用方式

### 只替换 Gazebo 模型

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/px4_overlay/models:$GZ_SIM_RESOURCE_PATH
export PX4_GZ_MODEL=x500_flow_depth
cd PX4-Autopilot
make px4_sitl gz_x500_flow
```

`scripts/run_sitl_combined.sh` 已封装这一步。

### 需要自定义 airframe

1. 复制 `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_flow`；
2. 改 `PX4_SIM_MODEL=x500_flow_depth`；
3. 放到 `px4_overlay/airframes/`，并记录它需要复制到 PX4 的目标路径；
4. 升级 PX4 后重新复制/注册。

> 后续会加 `scripts/apply_overlay.sh`，把 overlay 自动同步到 PX4 目录，并在 `verify.sh` 里检查。

## 规则

- 不要在 `PX4-Autopilot/` 里留下未记录的修改；
- 每个自定义文件都要能从 overlay 重建；
- 模型里的传感器安装位置、`EKF2_OF_POS_*`、`SENS_FLOW_ROT` 要一致；
- 改动后跑 `./scripts/verify.sh`。

## PX4 补丁（升级后需要重放）

| 文件 | 作用 |
|---|---|
| `px4_patches/dds_topics.yaml` | 把 `vehicle_optical_flow`、`distance_sensor` 加入 uXRCE 转发列表 |
| `px4_patches/gz_bridge_downward_lidar.patch` | 让 +90° pitch 的下视激光被标记为 `ROTATION_DOWNWARD_FACING`，否则光流 `distance_m` 为 NaN |

重放 dds_topics.yaml：

```bash
cp px4_overlay/px4_patches/dds_topics.yaml \
   PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

重放补丁并重编：

```bash
git -C PX4-Autopilot apply px4_overlay/px4_patches/gz_bridge_downward_lidar.patch
cmake --build PX4-Autopilot/build/px4_sitl_default --target px4 -- -j8
```
