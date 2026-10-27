#pragma once

#include <string>

// 控制权租约：多个"想发杆量"的进程（网页后端 web、终端摇杆守护 terminal）
// 通过 /tmp/flight_control.lease 协商，同一时刻只让一个真正发布。
//
// 文件格式（一行，空格分隔，写的时候先写临时文件再 rename，读端不会读到半个）：
//     <owner> <prio> <pid> <unix_ms>     ← 整数毫秒，避免浮点精度坑
//
// 规则：
//   - 持有者每 0.5 s 刷新一次时间戳（心跳）
//   - 时间戳超过 2 s 或 pid 已死 → 视为无主，谁先到谁拿
//   - 优先级高的可以抢占低的（web 10 > terminal 5）
//   - 同优先级不抢占（先来先得）
namespace control_lease {

constexpr const char * kPath = "/tmp/flight_control.lease";
constexpr double kStaleSec = 2.0;
constexpr int kPrioWeb = 10;
constexpr int kPrioTerminal = 5;

struct Entry {
    std::string owner;
    int prio = 0;
    int pid = 0;
    long long ts_ms = 0;
    bool valid = false;
};

Entry read();

// 我是否能发杆量。能就顺手刷新心跳（或抢占），不能就什么都不做。
bool acquire(const std::string & me, int prio);

// 我不再需要控制权时调用（比如网页一个都没连）：只有文件是我写的才会删。
void release(const std::string & me);

}  // namespace control_lease
