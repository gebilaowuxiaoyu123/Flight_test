#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "flight_ui_cpp/rc_service.hpp"

namespace flight_ui_cpp
{

// Boost.Beast 实现的 HTTP + WebSocket 服务：
//   GET /             -> index.html
//   GET /static/<f>   -> 静态资源
//   GET /health       -> JSON 健康检查
//   WS  /ws           -> 与前端交换 JSON（协议和 Python 版完全一致）
class WebServer
{
public:
  WebServer(RcService & service, std::string web_root, std::string host, uint16_t port);
  ~WebServer();

  // 阻塞运行 accept 循环；放在单独线程里调用
  void run();
  void stop();

private:
  RcService & service_;
  std::string web_root_;
  std::string host_;
  uint16_t port_;
  std::atomic<bool> running_{false};
  int listen_fd_{-1};
};

}  // namespace flight_ui_cpp
