#include "flight_ui_cpp/web_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <json/reader.h>
#include <json/writer.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

#include "flight_ui_cpp/types.hpp"

namespace flight_ui_cpp
{
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace
{

std::atomic<uint64_t> g_session_counter{1};

std::string jsonString(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

std::string contentType(const std::string & path)
{
  const auto pos = path.find_last_of('.');
  const std::string ext = (pos == std::string::npos) ? "" : path.substr(pos);
  if (ext == ".html") {return "text/html; charset=utf-8";}
  if (ext == ".js") {return "application/javascript; charset=utf-8";}
  if (ext == ".css") {return "text/css; charset=utf-8";}
  if (ext == ".json") {return "application/json";}
  if (ext == ".png") {return "image/png";}
  return "text/plain; charset=utf-8";
}

bool readFile(const std::string & path, std::string & out)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  out = ss.str();
  return true;
}

// ---------------------------------------------------------------- WebSocket
class WsSession : public std::enable_shared_from_this<WsSession>
{
public:
  WsSession(beast::tcp_stream stream, http::request<http::string_body> req, RcService & service)
  : ws_(std::move(stream)), service_(service), id_(g_session_counter++)
  {
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator([](websocket::response_type & res) {
      res.set(http::field::server, "flight_ui_cpp");
    }));
    ws_.accept(req);
    ws_.text(true);
  }

  // 读循环在自己线程，写循环（推送遥测）在另一个线程：
  // Beast 允许一个流上同时只有一个读和一个写。
  void run()
  {
    std::atomic<bool> alive{true};
    std::thread writer([this, &alive]() {writeLoop(alive);});

    beast::flat_buffer buffer;
    for (;;) {
      beast::error_code ec;
      ws_.read(buffer, ec);
      if (ec) {
        break;
      }
      handleMessage(beast::buffers_to_string(buffer.data()));
      buffer.consume(buffer.size());
    }
    alive = false;
    writer.join();
    service_.releaseClient(id_);
  }

private:
  void writeLoop(std::atomic<bool> & alive)
  {
    while (alive) {
      Json::Value root = service_.telemetryJson();
      root["type"] = "telemetry";
      const std::string frame = jsonString(root);
      beast::error_code ec;
      ws_.write(net::buffer(frame), ec);
      if (ec) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    alive = false;
  }

  void sendAck(const std::string & action, const std::pair<bool, std::string> & result)
  {
    Json::Value ack(Json::objectValue);
    ack["type"] = "ack";
    ack["action"] = action;
    ack["ok"] = result.first;
    ack["message"] = result.second;
    beast::error_code ec;
    ws_.write(net::buffer(jsonString(ack)), ec);
  }

  void handleMessage(const std::string & text)
  {
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    std::string errors;
    if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors)) {
      return;
    }
    const std::string kind = root.get("type", "").asString();

    if (kind == "sticks") {
      Sticks sticks;
      sticks.roll = root.get("roll", 0.0).asDouble();
      sticks.pitch = root.get("pitch", 0.0).asDouble();
      sticks.yaw = root.get("yaw", 0.0).asDouble();
      sticks.throttle = root.get("throttle", -1.0).asDouble();
      sticks.deadman = root.get("deadman", false).asBool();
      service_.setSticks(sticks, id_);
    } else if (kind == "arm") {
      sendAck("arm", service_.arm(id_));
    } else if (kind == "disarm") {
      sendAck("disarm", service_.disarm(id_));
    } else if (kind == "mode") {
      sendAck("mode", service_.setMode(root.get("mode", "POSCTL").asString(), id_));
    } else if (kind == "takeoff") {
      sendAck("takeoff", service_.takeoff(root.get("altitude", 2.5).asDouble(), id_));
    } else if (kind == "land") {
      sendAck("land", service_.land(id_));
    } else if (kind == "kill") {
      sendAck("kill", service_.kill(id_));
    }
  }

  websocket::stream<beast::tcp_stream> ws_;
  RcService & service_;
  const uint64_t id_;
};

// ---------------------------------------------------------------- HTTP
void serveHttp(
  beast::tcp_stream & stream, const http::request<http::string_body> & req,
  const std::string & web_root, RcService & service)
{
  http::response<http::string_body> res{http::status::ok, req.version()};
  res.set(http::field::server, "flight_ui_cpp");
  res.keep_alive(false);

  std::string target(req.target());
  if (target == "/health") {
    Json::Value root(Json::objectValue);
    root["status"] = "ok";
    root["ros"] = true;
    root["telemetry"] = service.telemetryJson();
    res.set(http::field::content_type, "application/json");
    res.body() = jsonString(root);
  } else {
    std::string relative = "/index.html";
    if (target.rfind("/static/", 0) == 0) {
      relative = target.substr(7);
    }
    if (relative.find("..") != std::string::npos) {
      res.result(http::status::bad_request);
      res.body() = "bad path";
    } else {
      std::string body;
      if (readFile(web_root + relative, body)) {
        res.set(http::field::content_type, contentType(relative));
        res.body() = std::move(body);
      } else {
        res.result(http::status::not_found);
        res.set(http::field::content_type, "application/json");
        res.body() = R"({"detail":"Not Found"})";
      }
    }
  }
  res.prepare_payload();

  beast::error_code ec;
  http::write(stream, res, ec);
  stream.socket().shutdown(tcp::socket::shutdown_send, ec);
}

void handleConnection(int fd, const std::string & web_root, RcService & service)
{
  net::io_context ioc(1);
  tcp::socket socket(ioc);
  beast::error_code ec;
  socket.assign(tcp::v4(), fd, ec);
  if (ec) {
    ::close(fd);
    return;
  }
  beast::tcp_stream stream(std::move(socket));

  try {
    beast::flat_buffer buffer;
    http::request<http::string_body> req;
    http::read(stream, buffer, req, ec);
    if (ec) {
      return;
    }
    if (websocket::is_upgrade(req)) {
      auto session = std::make_shared<WsSession>(std::move(stream), std::move(req), service);
      session->run();
    } else {
      serveHttp(stream, req, web_root, service);
    }
  } catch (const std::exception &) {
    // 客户端断开是常态，不打扰日志
  }
}

}  // namespace

WebServer::WebServer(RcService & service, std::string web_root, std::string host, uint16_t port)
: service_(service), web_root_(std::move(web_root)), host_(std::move(host)), port_(port)
{
}

WebServer::~WebServer()
{
  stop();
}

void WebServer::run()
{
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    throw std::runtime_error("socket() failed");
  }
  int yes = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
    addr.sin_addr.s_addr = INADDR_ANY;
  }
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    throw std::runtime_error("bind(" + host_ + ":" + std::to_string(port_) + ") failed");
  }
  if (::listen(listen_fd_, 16) < 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    throw std::runtime_error("listen() failed");
  }

  running_ = true;
  while (running_) {
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) {
      if (!running_) {
        break;
      }
      continue;
    }
    std::thread([this, fd]() {handleConnection(fd, web_root_, service_);}).detach();
  }
}

void WebServer::stop()
{
  running_ = false;
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
}

}  // namespace flight_ui_cpp
