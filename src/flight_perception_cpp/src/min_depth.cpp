// 32FC1 深度图 → 中央 ROI 的最小距离（/obstacle_distance）。
// 不依赖 OpenCV：32FC1 就是 float32 数组，直接按行步长索引最快。

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

namespace
{
class MinDepth : public rclcpp::Node
{
public:
  MinDepth()
  : rclcpp::Node("min_depth")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/depth_camera");
    max_range_ = declare_parameter<double>("max_range", 20.0);
    roi_w_ = declare_parameter<double>("roi_width", 0.5);
    roi_h_ = declare_parameter<double>("roi_height", 0.6);

    // 传感器类话题一律 best_effort
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {onImage(msg);});
    pub_ = create_publisher<std_msgs::msg::Float32>("obstacle_distance", 10);
    RCLCPP_INFO(get_logger(), "listening %s", image_topic_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->encoding != "32FC1" && msg->encoding != "32FC1;") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    // step 是字节数；32FC1 每像素 4 字节。行尾可能有 padding，所以按 row_len 索引。
    const size_t row_len = msg->step / sizeof(float);
    if (row_len < msg->width) {
      return;
    }
    if (msg->data.size() < msg->step * msg->height) {
      return;
    }
    const float * data = reinterpret_cast<const float *>(msg->data.data());

    const size_t rw = std::max<size_t>(1, static_cast<size_t>(msg->width * roi_w_));
    const size_t rh = std::max<size_t>(1, static_cast<size_t>(msg->height * roi_h_));
    const size_t x0 = (msg->width - rw) / 2;
    const size_t y0 = (msg->height - rh) / 2;

    float best = std::numeric_limits<float>::max();
    for (size_t v = y0; v < y0 + rh; ++v) {
      const float * row = data + v * row_len;
      for (size_t u = x0; u < x0 + rw; ++u) {
        const float d = row[u];
        if (std::isfinite(d) && d > 0.01F && d < static_cast<float>(max_range_)) {
          best = std::min(best, d);
        }
      }
    }
    if (best == std::numeric_limits<float>::max()) {
      return;
    }

    std_msgs::msg::Float32 out;
    out.data = best;
    pub_->publish(out);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "obstacle distance: %.2f m", best);
  }

  std::string image_topic_;
  double max_range_{20.0};
  double roi_w_{0.5};
  double roi_h_{0.6};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinDepth>());
  rclcpp::shutdown();
  return 0;
}
