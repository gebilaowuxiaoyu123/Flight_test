#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "flight_ui_cpp/shared_state.hpp"

using CommandPublisher = rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr;

uint64_t nowUs(rclcpp::Node * node);

void sendCommand(const CommandPublisher & pub,
                 uint64_t timestamp_us,
                 uint16_t command,
                 float p1 = 0.0F, float p2 = 0.0F, float p3 = 0.0F, float p4 = 0.0F,
                 double p5 = 0.0, double p6 = 0.0, float p7 = 0.0F);

bool isManualNavState(int nav_state);
void sleepMs(int ms);

void ensureManualMode(SharedState & state, rclcpp::Node * node, const CommandPublisher & cmd_pub);
bool armWithRetry(SharedState & state, rclcpp::Node * node, const CommandPublisher & cmd_pub);
void armSequence(SharedState & state, rclcpp::Node * node, const CommandPublisher & cmd_pub);
void takeoffSequence(SharedState & state, rclcpp::Node * node,
                     const CommandPublisher & cmd_pub, double altitude_m);
void landNow(rclcpp::Node * node, const CommandPublisher & cmd_pub);
void killNow(rclcpp::Node * node, const CommandPublisher & cmd_pub);
