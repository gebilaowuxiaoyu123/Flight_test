// http_server.cpp —— 每个 TCP 连接一个线程：先读 HTTP 头，再决定升级 WebSocket 还是回文件
// 对外：handleConnection()
// 依赖：http_util（json / 读文件 / MIME）、ws_session（升级后整条连接交给它）

#include "flight_ui_cpp/http_server.hpp"

#include <atomic>
#include <string>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <json/json.h>
#include <rclcpp/logging.hpp>

#include "flight_ui_cpp/http_util.hpp"
#include "flight_ui_cpp/ws_session.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
namespace websocket = beast::websocket;
using tcp = net::ip::tcp;

void handleConnection(int fd, SharedState & state, const std::string & web_root,
                      rclcpp::Node * node,
                      const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub)   // 每连接一个线程
{
    static std::atomic<uint64_t> next_client{1};
    const uint64_t client_id = next_client++;
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

        // 读到了一个完整的 HTTP 请求头。判断它是不是想升级成 WebSocket
        // ---- WebSocket 升级 ----
        if (websocket::is_upgrade(req)) {
            serveWebSocket(std::move(stream), std::move(req), state, node, cmd_pub, client_id);
            return;                        // 连接已被 WS 接管，HTTP 分支不再执行
        }

        // 从共享状态里取一份加锁快照，再拼成 /health 的响应
        http::response<http::string_body> res{http::status::ok, req.version()};
        res.set(http::field::server, "FlightUI");
        res.keep_alive(false);

        // ---- 普通 HTTP：/health + 静态文件 ----
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
