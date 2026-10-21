// C++ 版 Web 遥控器入口：一个 rclcpp 节点 + 一个 Beast WebSocket 服务。
//
// 与 Python 版 flight_ui 的协议完全一致，所以前端 web/ 目录不用改。

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "flight_ui_cpp/px4_adapter.hpp"
#include "flight_ui_cpp/rc_service.hpp"
#include "flight_ui_cpp/telemetry.hpp"
#include "flight_ui_cpp/web_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string host = "0.0.0.0";
  int port = 8000;
  std::string web_root;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--host" && i + 1 < argc) {
      host = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    } else if (arg == "--web-root" && i + 1 < argc) {
      web_root = argv[++i];
    }
  }
  if (web_root.empty()) {
    web_root = ament_index_cpp::get_package_share_directory("flight_ui_cpp") + "/web";
  }

  auto node = std::make_shared<rclcpp::Node>("flight_ui");
  flight_ui_cpp::Px4Adapter adapter(node.get());
  flight_ui_cpp::Telemetry telemetry(node.get());
  flight_ui_cpp::RcService service(adapter, telemetry);

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(20), [&service]() {service.tick();});

  flight_ui_cpp::WebServer server(service, web_root, host, static_cast<uint16_t>(port));
  std::thread web_thread;
  try {
    web_thread = std::thread([&server]() {server.run();});
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(node->get_logger(), "Web 服务启动失败：%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(), "Web RC 已启动：http://%s:%d/  （静态资源 %s）",
    host.c_str(), port, web_root.c_str());

  rclcpp::spin(node);
  server.stop();
  if (web_thread.joinable()) {
    web_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
