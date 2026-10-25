# 14 · 终端飞飞机

把所有 ROS 话题细节都包进了 `scripts/fly.sh`，你只敲动作词。

## 起 / 停

```bash
./scripts/fly.sh start      # 起 PX4+Gazebo、摇杆守护、遥测缓存、网页后端
./scripts/fly.sh stop       # 全停
```

## 看（终端里实时显示姿态）

```bash
./scripts/fly.sh watch      # 实时刷新：模式 / ARMED / 高度 / 横滚 / 俯仰 / 航向 / 速度
./scripts/fly.sh status     # 打印一行就退出
```

`watch` 的样子（原地刷新，不刷屏）：

```
┌─ 飞行姿态 ─────────────────────────────────────────────┐
│ 模式  ALTCTL        ● ARMED        高度   1.63 m       │
│ 横滚   +2.3°  [----------O----------]                  │
│ 俯仰   -1.8°  [----------O----------]                  │
│ 航向   128°   水平速度 0.42 m/s   垂直 +0.03 m/s       │
└────────────────────────────────────────────────────────┘
```

## 飞

```bash
./scripts/fly.sh mode ALTCTL     # 切模式（MANUAL/ALTCTL/POSCTL/AUTO/ACRO/OFFBOARD）
./scripts/fly.sh takeoff         # 解锁并爬到 1.2 m
./scripts/fly.sh fwd 3           # 前进 3 秒
./scripts/fly.sh up 2            # 爬升 2 秒
./scripts/fly.sh left 2          # 左移 2 秒
./scripts/fly.sh spin 2          # 左转 2 秒
./scripts/fly.sh land            # 降落并上锁
```

全部子命令：`mode` `arm` `takeoff` `up` `down` `fwd` `back` `left` `right` `spin` `land` `status` `watch` `start` `stop`

不写秒数默认 2 秒。`takeoff` 默认爬到 1.2 m，可以写 `takeoff 2.0`。

## 它内部到底发了什么

`fly.sh` 只是把这两个话题的命令包了一层：

| 动作 | ROS 话题 |
|---|---|
| 切模式 / 解锁 / 上锁 | `/fmu/in/vehicle_command`（`176`=切模式、`400`=解锁上锁） |
| 爬升 / 前进 / 转向 | `/fmu/in/manual_control_input`，50 Hz 连续发 |

想看原始命令：`bash -x ./scripts/fly.sh fwd 1`

## 三个设计上的坑（已经处理，但要知道）

**1. 摇杆必须 50 Hz 连续发**
PX4 超过 0.5 s 收不到手动控制就判失效。所以有个常驻的 `stick_keeper.py`，
`fly.sh` 只改 `/tmp/stick.json` 里那 4 个数。

**2. 杆量文件 2 秒不更新就回中悬停**
否则操控脚本被 Ctrl-C 打断，最后一条杆量会一直发下去 ——
实测油门 0.45 被留在文件里，飞机一口气爬到 17 m。现在过期就回中，最多悬停。

**3. `ros2 topic pub --once` 会丢包**
每次调用都要新建 ROS 节点，DDS 发现没完成时消息直接丢。
所以 `pub_cmd` 改成连发 3 次，解锁/上锁还带重试。

## 为什么是 ALTCTL 不是 OFFBOARD

OFFBOARD + `TrajectorySetpoint` 是 PX4 的自主飞行接口，本机用不了：

```
xy_valid: false                          ← 无 GPS，光流给不出水平位置
WARN [mc_pos_control] invalid setpoints  ← PX4 直接拒
```

ALTCTL 的横滚/俯仰杆是**姿态角指令**，不依赖位置估计，所以能飞。

## 和后端不要同时飞

`rc_server`（网页后端）也会以 50 Hz 往同一个话题发杆量，两边会打架。
用终端飞之前先 `./scripts/fly.sh stop`，或者用 `fly.sh start`（它只起后端不抢杆）。
