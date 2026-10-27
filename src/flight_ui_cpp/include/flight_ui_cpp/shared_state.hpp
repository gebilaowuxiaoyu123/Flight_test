#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <utility>

#include <json/json.h>

#include "flight_ui_cpp/mode_tables.hpp"

// 跨线程共享状态：控制链（50 Hz 发杆）和遥测链（10 Hz 上报）各用一把锁，互不阻塞。
class SharedState
{
public:
    // 写：网页/脚本的摇杆帧进来。
    // 控制权租约：谁先按下 deadman 谁拿控制权；旁观页面（每 20ms 发一帧
    // deadman=false）不会把正在飞的控制端冲掉。
    void setSticks(uint64_t client, bool deadman, double roll, double pitch,
                   double yaw, double throttle)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 没按使能的帧不能"占住"控制权（否则旁观页面会把 owner 抢走，HUD 显示错），
        // 但当前 owner 发来的 deadman=false 必须生效 —— 那是"松杆"动作。
        if (client_ != client && !deadman) {
            return;
        }
        client_ = client;
        deadman_ = deadman;
        roll_ = roll;
        pitch_ = pitch;
        yaw_ = yaw;
        throttle_stick_ = throttle;
        last_update_ = std::chrono::steady_clock::now();
    }

    void attachClient(uint64_t client)              // 网页连上：从此"有人在控"
    {
        std::lock_guard<std::mutex> lock(mutex_);
        attached_.insert(client);
    }

    void releaseClient(uint64_t client)             // 连接断开时交还控制权
    {
        std::lock_guard<std::mutex> lock(mutex_);
        attached_.erase(client);
        if (client_ == client) {
            client_ = 0;
            deadman_ = false;
        }
    }

    struct Control {                                // 定时器要用的一份完整快照
        bool valid;                                 // 真按着使能（杆量可用）
        bool any_client;                            // 是否还有网页连着
        double roll, pitch, yaw, throttle;
    };

    //300ms没有更新判定失效
    Control control() const                         // 读：定时器取"有效?+四通道"
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_update_).count();
        return {deadman_ && age < 0.3, !attached_.empty(),
                roll_, pitch_, yaw_, throttle_stick_};
    }

    // 写：PX4 遥测进来。拆成 3 个 setter，因为三个回调各只知道一部分字段
    void setStatus(bool armed, int nav_state)              // 来自 vehicle_status
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        armed_ = armed;
        nav_state_ = nav_state;
        last_status_ = std::chrono::steady_clock::now();
    }

    // 遥测多久没来了（秒）。用来识别"ROS 链路根本没通"：
    // 没起 MicroXRCEAgent 时 /fmu/* 话题一条都没有，命令发出去也没人收。
    double statusAgeSec() const
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        if (last_status_.time_since_epoch().count() == 0) {
            return 1e9;
        }
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_status_).count();
    }

    void setPosition(bool z_valid, double z, double ref_alt)   // 来自 vehicle_local_position
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        z_valid_ = z_valid;
        z_ = z;
        ref_alt_ = ref_alt;
    }

    // 起飞要用 AMSL 绝对高度：当前海压高 + 相对高度
    double takeoffAltitudeAmsl(double relative_m) const
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        if (!std::isfinite(ref_alt_)) {
            return relative_m;                      // 拿不到基准就退化成相对高度
        }
        return ref_alt_ - z_ + relative_m;
    }

    // 来自 /obstacle_distance（感知包 flight_perception_cpp 的输出）
    void setObstacle(double meters)
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        obstacle_m_ = meters;
        last_obstacle_ = std::chrono::steady_clock::now();
    }

    // 没有感知在跑时返回 -1（前端据此显示 "--"）；超过 2 s 没更新也算失效
    double obstacleDistance() const
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        return obstacleDistanceLocked();
    }

    // 来自 battery_status：voltage_v 是电压，remaining 是剩余比例(0~1，-1 表示未知)
    void setBattery(float voltage_v, float remaining)
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        voltage_v_ = voltage_v;
        battery_pct_ = (remaining >= 0.0F) ? static_cast<double>(remaining * 100.0F) : -1.0;
    }

    Json::Value getTelemetry() const { return getTelemetryFor(0); }

    // 读：拼成 JSON 发给网页。带上当前真正下发的杆量和控制权归属，
    // 页面据此把摇杆"回放"到真实位置（自己操控时不回放，避免和手指打架）。
    Json::Value getTelemetryFor(uint64_t client) const
    {
        Json::Value root(Json::objectValue);
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            root["type"] = "telemetry";          // 前端靠它分流
            root["armed"] = armed_;
            root["nav_state"] = nav_state_;
            root["mode"] = modeNameOf(nav_state_);   // 前端 HUD 显示用
            root["z_valid"] = z_valid_;
            root["z"] = z_;                          // 原始 NED，调试用
            root["alt"] = z_valid_ ? -z_ : 0.0;      // NED 向下为正，取负才是高度
            root["obstacle_m"] = obstacleDistanceLocked();
            root["voltage_v"] = voltage_v_;
            root["battery"] = battery_pct_;          // 百分数，-1 表示未知
        }
        // 两把锁分开取，永不嵌套反序（别处没有先 control 后 telemetry 的路径）
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Json::Value sticks(Json::objectValue);
            sticks["roll"] = roll_;
            sticks["pitch"] = pitch_;
            sticks["yaw"] = yaw_;
            sticks["throttle"] = throttle_stick_;
            sticks["deadman"] = deadman_;
            root["sticks"] = sticks;
            root["controller"] = (client_ == 0) ? "none"
                                : (client_ == client ? "you" : "other");
        }
        return root;
    }

    bool armed() const
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        return armed_;
    }

    int navState() const
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        return nav_state_;
    }

private:
    // 调用方必须已经持有 telemetry_mutex_
    double obstacleDistanceLocked() const
    {
        if (last_obstacle_.time_since_epoch().count() == 0) {
            return -1.0;
        }
        if (std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_obstacle_).count() > 2.0) {
            return -1.0;
        }
        return obstacle_m_;
    }

    // 两把锁：控制链（20ms 高频）和遥测链（10Hz）互不阻塞
    mutable std::mutex mutex_;
    mutable std::mutex telemetry_mutex_;
    uint64_t client_ = 0;            // 当前持有控制权的连接 id（0 = 没人）
    std::set<uint64_t> attached_;    // 还连着的网页（含只旁观的）
    bool deadman_ = false;
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    double throttle_stick_ = -1.0;
    std::chrono::steady_clock::time_point last_update_;

    bool armed_ = false;
    int nav_state_ = -1;
    std::chrono::steady_clock::time_point last_status_{};
    bool z_valid_ = false;
    double z_ = 0.0;
    double ref_alt_ = std::nan("");
    float voltage_v_ = 0.0F;
    double battery_pct_ = -1.0;      // -1 表示未知
    double obstacle_m_ = -1.0;       // 正前方最近障碍（m），-1 表示没有感知数据
    std::chrono::steady_clock::time_point last_obstacle_{};
};
