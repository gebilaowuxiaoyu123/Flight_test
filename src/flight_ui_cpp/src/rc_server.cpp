#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

#include <chrono>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <json/json.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>

#include <mutex>
#include <utility>
#include <memory>

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = net::ip::tcp;               // from <boost/asio/ip/tcp

class SharedState
{
public:
    void setSticks(bool deadman,double throttle)   // 写：网页摇杆帧进来
    {
        std::lock_guard<std::mutex> lock(mutex_);
        deadman_ = deadman;
        throttle_stick_ = throttle;
        last_update_ = std::chrono::steady_clock::now();
    }

    //300ms没有更新判定失效
    std::pair<bool, double> control() const        // 读：定时器取"有效?+油门"
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_update_).count(); 
        return {deadman_ && age < 0.3, throttle_stick_};  
    }

    // 写：PX4 遥测进来。拆成 3 个 setter，因为三个回调各只知道一部分字段
    void setStatus(bool armed, int nav_state)              // 来自 vehicle_status
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        armed_ = armed;
        nav_state_ = nav_state;
    }

    void setPosition(bool z_valid, double z)               // 来自 vehicle_local_position
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        z_valid_ = z_valid;
        z_ = z;
    }

    void setBattery(float voltage_v)                       // 来自 battery_status
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        voltage_v_ = voltage_v;
    }

    Json::Value getTelemetry() const               // 读：拼成 JSON 发给网页
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        Json::Value root(Json::objectValue);
        root["armed"] = armed_;
        root["nav_state"] = nav_state_;
        root["z_valid"] = z_valid_;
        root["z"] = z_;
        root["alt"] = z_valid_ ? -z_ : 0.0;   // NED 向下为正，取负才是高度
        root["voltage_v"] = voltage_v_;
        return root;
    }

private:
    // 两把锁：控制链（20ms 高频）和遥测链（10Hz）互不阻塞
    mutable std::mutex mutex_;
    mutable std::mutex telemetry_mutex_;
    bool deadman_ = false;
    double throttle_stick_ = -1.0;
    std::chrono::steady_clock::time_point last_update_;

    bool armed_ = false;
    int nav_state_ = -1;
    bool z_valid_ = false;
    double z_ = 0.0;
    float voltage_v_ = 0.0F;
};  



std::string jsonString(const Json::Value & value)   // Json 值转字符串
{
    Json::StreamWriterBuilder builder;
    builder["indentation"] = ""; // No indentation
    return Json::writeString(builder, value);
}

bool readFile(const std::string & path, std::string & content)   // 读文件到字符串
{
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    content = ss.str();
    return true;
}

// 根据后缀返回 MIME 类型。
// 注意：std::string::ends_with 是 C++20，本项目是 C++17，所以自己取后缀比较。
std::string contentTypeOf(const std::string & path)
{
    const auto pos = path.find_last_of('.');
    const std::string ext = (pos != std::string::npos) ? path.substr(pos) : "";
    if (ext == ".html") return "text/html; charset=utf-8";   // charset 保证中文不乱码
    if (ext == ".js")   return "application/javascript; charset=utf-8";
    if (ext == ".css")  return "text/css; charset=utf-8";
    if (ext == ".png")  return "image/png";
    if (ext == ".ico")  return "image/x-icon";
    return "text/plain; charset=utf-8";                      // 兜底
}

void handleConnection(int fd, SharedState & state, const std::string & web_root)   // 每连接一个线程
{
    try
    {
        // tcp_stream 不能直接从 int fd 构造：需要 io_context + socket.assign 接管 fd
        net::io_context ioc(1);
        tcp::socket socket(ioc);
        beast::error_code ec;
        socket.assign(tcp::v4(), fd, ec);
        if (ec) {
            ::close(fd);
            return;
        }
        beast::tcp_stream stream(std::move(socket));
        beast::flat_buffer buffer;

        http::request<http::string_body> req;
        http::read(stream, buffer, req);

        // 从共享状态里取一份加锁快照，再拼成 /health 的响应
        http::response<http::string_body> res{http::status::ok, req.version()};
        res.set(http::field::server, "FlightUI");
        res.keep_alive(false);

        const std::string target(req.target());
        if (target == "/health") {
            Json::Value root(Json::objectValue);
            root["status"] = "ok";
            root["ros"] = true;
            root["telemetry"] = state.getTelemetry();
            res.set(http::field::content_type, "application/json");
            res.body() = jsonString(root);
        } else {
            // 只有 "/" 映射到首页，其它路径按字面找文件；找不到就 404
            std::string relative = target;
            if (target == "/") {
                relative = "/index.html";
            } else if (target.rfind("/static/", 0) == 0) {
                relative = target.substr(7);          // "/static/app.js" → "/app.js"
            }

            std::string body;
            if (relative.find("..") != std::string::npos) {
                // 挡路径穿越：GET /static/../../etc/passwd
                res.result(http::status::bad_request);
                res.set(http::field::content_type, "text/plain; charset=utf-8");
                res.body() = "bad path";
            } else if (readFile(web_root + relative, body)) {
                res.set(http::field::content_type, contentTypeOf(relative));
                res.body() = std::move(body);
            } else {
                res.result(http::status::not_found);
                res.set(http::field::content_type, "application/json");
                res.body() = R"({"detail":"Not Found"})";
            }
        }
        // 所有分支之后再算 Content-Length：此时 body 才最终确定
        res.prepare_payload();

        http::write(stream, res);
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error handling connection: " << e.what() << std::endl;
    }
}

int main(int argc, char ** argv)   // 装配 + spin
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("flight_ui");
    auto logger = rclcpp::get_logger("flight_ui");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 跨线程共享的数据全放这里，不再有裸变量
    SharedState state;

    // 前端文件装在哪：查 ament 索引，换机器/换目录都不用改代码
    const std::string web_root =
        ament_index_cpp::get_package_share_directory("flight_ui_cpp") + "/web";
    RCLCPP_INFO(logger, "静态资源目录：%s", web_root.c_str());

    auto status_sub = node->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", qos,
        [&](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            state.setStatus(msg->arming_state == 2, // 2 = ARMED
                            msg->nav_state);
        });

    auto pos_sub = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        [&](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            state.setPosition(msg->z_valid, msg->z);
        });

    auto battery_sub = node->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status", qos,
        [&](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            state.setBattery(msg->voltage_v);
        });

    const auto qos_in = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto manual_pub = node->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
        "/fmu/in/manual_control_input", qos_in);

    // 1 Hz 日志：读一份快照打印（别在这里直接读成员变量）
    auto timer = node->create_wall_timer(std::chrono::milliseconds(1000), [&, logger]() {
        const Json::Value t = state.getTelemetry();
        RCLCPP_INFO(logger, "armed=%s nav_state=%d alt=%.2f voltage=%.2f",
                    t["armed"].asBool() ? "Yes" : "No",
                    t["nav_state"].asInt(),
                    t["alt"].asDouble(),
                    t["voltage_v"].asDouble());
    });

    // 50 Hz：先从 state 取一份加锁快照，再在锁外发布
    auto control_timer = node->create_wall_timer(std::chrono::milliseconds(20), [&]() {
        const auto [valid, throttle] = state.control();

        px4_msgs::msg::ManualControlSetpoint msg;
        msg.timestamp = static_cast<uint64_t>(node->get_clock()->now().nanoseconds() / 1000);
        msg.timestamp_sample = msg.timestamp;
        msg.valid = valid;
        msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0; // 2
        msg.roll = 0.0F;
        msg.pitch = 0.0F;
        msg.yaw = 0.0F;
        msg.throttle = static_cast<float>(throttle);
        msg.flaps = 0.0F;
        msg.sticks_moving = false;
        msg.buttons = 0;

        manual_pub->publish(msg);
    });

    // -- HTTP 服务 --
    const int port = 8000;   // 和 Python 版、verify.sh、ws_smoke_test.py 保持一致

    const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_FATAL(logger, "bind(:%d) 失败，端口被调用？", port);
        rclcpp::shutdown();
        return 1;
    }
    ::listen(listen_fd, 16);

    std::atomic<bool> running{true};
    std::thread http_thread([&]() {
        while (running) {
            const int fd = ::accept(listen_fd, nullptr, nullptr);
            if (fd < 0) {
                break;
            }
            std::thread(handleConnection, fd, std::ref(state), web_root).detach();   // 每连接一个线程
        }
    });
    RCLCPP_INFO(logger, "HTTP 服务已启动，监听端口 %d", port);

    rclcpp::spin(node);

    running = false;
    ::shutdown(listen_fd, SHUT_RDWR);   // 让阻塞在 accept 的线程立刻返回
    ::close(listen_fd);
    if (http_thread.joinable()) {
        http_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}