// http_util.cpp —— 三个小工具：JSON 序列化、整文件读入、按后缀给 MIME
// 对外：jsonString / readFile / contentTypeOf
// 依赖：只用标准库 + jsoncpp，不碰 ROS

#include "flight_ui_cpp/http_util.hpp"

#include <fstream>
#include <sstream>

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
