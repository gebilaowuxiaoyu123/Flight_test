#include "flight_ui_cpp/command_sequence.hpp"

#include <chrono>
#include <thread>

#include <rclcpp/logging.hpp>

// 发一条 VehicleCommand。所有命令都走这里，字段填法只写一次。
// 命令号：400=解锁/上锁，176=切模式，22=起飞，21=降落，185=飞行终止
void sendCommand(const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & pub,
                 uint64_t timestamp_us,
                 uint16_t command,
                 float p1, float p2, float p3, float p4,
                 double p5, double p6, float p7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = timestamp_us;
    msg.command = command;
    msg.param1 = p1;    msg.param2 = p2;    msg.param3 = p3;    msg.param4 = p4;
    msg.param5 = p5;    msg.param6 = p6;    msg.param7 = p7;
    msg.target_system = 1;        // 1 = 本机（多机时是 2/3…）
    msg.target_component = 1;     // 1 = 自驾仓
    msg.source_system = 1;
    msg.source_component = 1;
    msg.confirmation = 0;         // 0 = 首次发送
    msg.from_external = true;     // 声明是外部程序发的，不是飞控内部逻辑
    pub->publish(msg);
}

uint64_t nowUs(rclcpp::Node * node)   // 当前时间（微秒），PX4 消息统一用微秒
{
    return static_cast<uint64_t>(node->get_clock()->now().nanoseconds() / 1000);
}

// 手动类模式：MANUAL/ALTCTL/POSCTL/ACRO/STABILIZED。
// 注意 5 是 AUTO_RTL，不是 ACRO —— ACRO 是 10。
bool isManualNavState(int nav_state)
{
    return nav_state == 0 || nav_state == 1 || nav_state == 2
        || nav_state == 10 || nav_state == 15;
}

void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// 当前不是手动模式就先切 POSCTL，并等它生效
void ensureManualMode(SharedState & state, rclcpp::Node * node,
                      const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub)
{
    if (!isManualNavState(state.navState())) {
        sendCommand(cmd_pub, nowUs(node), 176, 1.0F, 3.0F);   // 176 = DO_SET_MODE, param2=3 = POSCTL
        sleepMs(800);
    }
}

// 10 秒内反复请求解锁，每次等 1 秒看 armed 有没有变 true。
// PX4 冷启动要跑 arming checks（EKF 收敛、气压计预热），只能重试。
bool armWithRetry(SharedState & state, rclcpp::Node * node,
                  const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        sendCommand(cmd_pub, nowUs(node), 400, 1.0F);          // 400 = ARM_DISARM, param1=1 解锁

        const auto tick = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (std::chrono::steady_clock::now() < tick) {
            if (state.armed()) {
                return true;
            }
            sleepMs(100);
        }
        ensureManualMode(state, node, cmd_pub);                // 被拒可能是模式被踢走了
    }
    return false;
}

// 跑在独立线程里的完整解锁流程（绝不能写 ws！）
void armSequence(SharedState & state, rclcpp::Node * node,
                 const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub)
{
    ensureManualMode(state, node, cmd_pub);
    if (!armWithRetry(state, node, cmd_pub)) {
        RCLCPP_WARN(rclcpp::get_logger("flight_ui"),
                    "[arm] 解锁被拒：检查 PX4 终端里的 Preflight Fail 原因"
                    "（EKF 未收敛 / 机体倾斜 / 电池）");
        sendCommand(cmd_pub, nowUs(node), 400, 0.0F);          // 失败就确保留在上锁状态
    }
}

// 起飞：解锁 → 切 POSCTL → 发 NAV_TAKEOFF。参数分散在多条命令里，必须分步。
void takeoffSequence(SharedState & state, rclcpp::Node * node,
                     const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub,
                     double altitude_m)
{
    ensureManualMode(state, node, cmd_pub);
    if (!state.armed() && !armWithRetry(state, node, cmd_pub)) {
        RCLCPP_WARN(rclcpp::get_logger("flight_ui"),
                    "[takeoff] 解锁被拒，不起飞；看 PX4 终端里的 Preflight Fail 原因");
        return;                                                // 解锁失败就别硬起飞
    }
    sleepMs(300);
    sendCommand(cmd_pub, nowUs(node), 176, 1.0F, 3.0F);        // 切 POSCTL
    sleepMs(800);
    sendCommand(cmd_pub, nowUs(node), 22, 0.0F, 0.0F, 0.0F, 0.0F,
                0.0, 0.0, static_cast<float>(state.takeoffAltitudeAmsl(altitude_m)));
}

void landNow(rclcpp::Node * node,
             const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub)
{
    sendCommand(cmd_pub, nowUs(node), 21);                     // 21 = NAV_LAND
}

// 紧急停机：先上锁，再请求飞行终止（任一条件成立电机都会停）
void killNow(rclcpp::Node * node,
             const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub)
{
    sendCommand(cmd_pub, nowUs(node), 400, 0.0F);
    sendCommand(cmd_pub, nowUs(node), 185, 1.0F);
}
