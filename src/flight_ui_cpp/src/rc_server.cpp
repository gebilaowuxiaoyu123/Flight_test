#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("flight_ui");
    auto logger = rclcpp::get_logger("flight_ui");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    bool armed = false;
    int nav_state = -1;
    bool z_valid = false;
    float z=0.0F;
    float voltage_v =0.0F;

    auto status_sub = node->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", qos,
        [&](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            armed = msg->arming_state == 2; // PX4 arming state: 2 means armed
            nav_state = msg->nav_state;
        });

    auto pos_sub = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        [&](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            z_valid = msg->z_valid;
            z = msg->z;
        });

    auto battery_sub = node->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status", qos,
        [&](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            voltage_v = msg->voltage_v;
        });
    
    auto timer = node->create_wall_timer(std::chrono::milliseconds(1000), [&, logger]() {
        RCLCPP_INFO(logger, "Armed: %s, Nav State: %d, Z Valid: %s, Z: %.2f, Voltage: %.2f V",
                    armed ? "Yes" : "No", nav_state,
                    z_valid ? "Yes" : "No", z, voltage_v);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}