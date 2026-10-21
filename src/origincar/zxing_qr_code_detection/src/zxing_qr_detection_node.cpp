#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/imgproc.hpp>

#include <ZXing/BarcodeFormat.h>
#include <ZXing/DecodeHints.h>
#include <ZXing/ReadBarcode.h>

#include "cv_bridge/cv_bridge.h"
#include "origincar_msg/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

std::string trim_ascii_whitespace(std::string value)
{
  const auto is_space = [](unsigned char character) {return std::isspace(character) != 0;};
  const auto first = std::find_if_not(value.begin(), value.end(), is_space);
  const auto last = std::find_if_not(value.rbegin(), value.rend(), is_space).base();
  return first < last ? std::string(first, last) : std::string{};
}

void append_utf8(std::string & output, uint32_t code_point)
{
  if (code_point <= 0x7fU) {
    output.push_back(static_cast<char>(code_point));
  } else if (code_point <= 0x7ffU) {
    output.push_back(static_cast<char>(0xc0U | (code_point >> 6U)));
    output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
  } else if (code_point <= 0xffffU) {
    output.push_back(static_cast<char>(0xe0U | (code_point >> 12U)));
    output.push_back(static_cast<char>(0x80U | ((code_point >> 6U) & 0x3fU)));
    output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
  } else if (code_point <= 0x10ffffU) {
    output.push_back(static_cast<char>(0xf0U | (code_point >> 18U)));
    output.push_back(static_cast<char>(0x80U | ((code_point >> 12U) & 0x3fU)));
    output.push_back(static_cast<char>(0x80U | ((code_point >> 6U) & 0x3fU)));
    output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
  }
}

// zxing-cpp 1.x (Ubuntu 22.04) returns std::wstring, while 2.x returns UTF-8.
std::string to_utf8(const std::string & value)
{
  return value;
}

std::string to_utf8(const std::wstring & value)
{
  std::string output;
  output.reserve(value.size());

  for (std::size_t index = 0; index < value.size(); ++index) {
    uint32_t code_point = static_cast<uint32_t>(value[index]);
    if constexpr (sizeof(wchar_t) == 2) {
      if (code_point >= 0xd800U && code_point <= 0xdbffU && index + 1 < value.size()) {
        const uint32_t low = static_cast<uint32_t>(value[index + 1]);
        if (low >= 0xdc00U && low <= 0xdfffU) {
          code_point = 0x10000U + ((code_point - 0xd800U) << 10U) + (low - 0xdc00U);
          ++index;
        }
      }
    }

    if (code_point >= 0xd800U && code_point <= 0xdfffU) {
      code_point = 0xfffdU;
    }
    append_utf8(output, code_point);
  }
  return output;
}

class AtomicFlagGuard
{
public:
  explicit AtomicFlagGuard(std::atomic_flag & flag) : flag_(flag) {}
  ~AtomicFlagGuard() {flag_.clear();}

  AtomicFlagGuard(const AtomicFlagGuard &) = delete;
  AtomicFlagGuard & operator=(const AtomicFlagGuard &) = delete;

private:
  std::atomic_flag & flag_;
};

}  // namespace

class ZxingQrDetectionNode : public rclcpp::Node
{
public:
  ZxingQrDetectionNode()
  : Node("zxing_qr_detection"),
    last_raw_scan_(std::chrono::steady_clock::time_point::min()),
    last_compressed_scan_(std::chrono::steady_clock::time_point::min())
  {
    const auto raw_image_topic =
      declare_parameter<std::string>("raw_image_topic", "/aurora/rgb/image_raw");
    const auto compressed_image_topic =
      declare_parameter<std::string>("compressed_image_topic", "/image");
    scan_interval_ = std::chrono::milliseconds(
      std::max<int64_t>(0, declare_parameter<int64_t>("scan_interval_ms", 150)));
    stop_after_detection_ = declare_parameter<bool>("stop_after_detection", true);
    try_harder_ = declare_parameter<bool>("try_harder", true);
    try_rotate_ = declare_parameter<bool>("try_rotate", true);

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    qr_result_publisher_ = create_publisher<std_msgs::msg::String>(
      "/qr_code_result", output_qos);
    sign_publisher_ = create_publisher<origincar_msg::msg::Sign>(
      "/sign_switch", output_qos);

    if (!raw_image_topic.empty()) {
      raw_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        raw_image_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ZxingQrDetectionNode::raw_image_callback, this, std::placeholders::_1));
    }

    if (!compressed_image_topic.empty()) {
      compressed_image_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        compressed_image_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ZxingQrDetectionNode::compressed_image_callback, this, std::placeholders::_1));
    }

    if (!raw_image_subscription_ && !compressed_image_subscription_) {
      throw std::invalid_argument(
              "raw_image_topic and compressed_image_topic cannot both be empty");
    }

    RCLCPP_INFO(
      get_logger(),
      "zxing-cpp QR detector started (raw='%s', compressed='%s')",
      raw_image_topic.c_str(), compressed_image_topic.c_str());
  }

private:
  bool scan_is_due(std::chrono::steady_clock::time_point & last_scan)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_scan != std::chrono::steady_clock::time_point::min() &&
      now - last_scan < scan_interval_)
    {
      return false;
    }
    last_scan = now;
    return true;
  }

  void raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (scan_finished_.load() || !scan_is_due(last_raw_scan_)) {
      return;
    }

    try {
      const auto image = cv_bridge::toCvShare(msg, "bgr8")->image;
      detect(image, "raw image");
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to convert raw image: %s", exception.what());
    }
  }

  void compressed_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    if (scan_finished_.load() || !scan_is_due(last_compressed_scan_)) {
      return;
    }

    try {
      const auto image = cv_bridge::toCvCopy(msg, "bgr8")->image;
      detect(image, "compressed image");
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to convert compressed image: %s", exception.what());
    }
  }

  void detect(const cv::Mat & bgr_image, const char * source)
  {
    if (scan_finished_.load() || scan_in_progress_.test_and_set()) {
      return;
    }
    AtomicFlagGuard scan_guard(scan_in_progress_);

    try {
      cv::Mat grayscale;
      cv::cvtColor(bgr_image, grayscale, cv::COLOR_BGR2GRAY);

      ZXing::DecodeHints hints;
      hints.setFormats(ZXing::BarcodeFormat::QRCode);
      hints.setTryHarder(try_harder_);
      hints.setTryRotate(try_rotate_);

      const ZXing::ImageView image_view(
        grayscale.data,
        grayscale.cols,
        grayscale.rows,
        ZXing::ImageFormat::Lum,
        static_cast<int>(grayscale.step));
      const auto result = ZXing::ReadBarcode(image_view, hints);
      if (!result.isValid()) {
        return;
      }

      const auto qr_text = trim_ascii_whitespace(to_utf8(result.text()));
      if (!qr_text.empty()) {
        publish_result(qr_text, source);
      }
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "OpenCV processing failed: %s", exception.what());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "zxing-cpp processing failed: %s", exception.what());
    }
  }

  void publish_result(const std::string & qr_text, const char * source)
  {
    if (stop_after_detection_ && scan_finished_.exchange(true)) {
      return;
    }

    std_msgs::msg::String result_message;
    result_message.data = qr_text;
    qr_result_publisher_->publish(result_message);
    RCLCPP_INFO(get_logger(), "QR code detected from %s: %s", source, qr_text.c_str());

    int32_t sign_data = 0;
    const unsigned char last_character = static_cast<unsigned char>(qr_text.back());
    if (std::isdigit(last_character) != 0) {
      sign_data = ((last_character - static_cast<unsigned char>('0')) % 2U == 1U) ? 3 : 4;
    } else if (qr_text == "ClockWise") {
      sign_data = 3;
    } else if (qr_text == "AntiClockWise") {
      sign_data = 4;
    }

    if (sign_data != 0) {
      origincar_msg::msg::Sign sign_message;
      sign_message.sign_data = sign_data;
      sign_publisher_->publish(sign_message);
      RCLCPP_INFO(get_logger(), "Published /sign_switch sign_data=%d", sign_data);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "QR content has no direction mapping; only /qr_code_result was published");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_result_publisher_;
  rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
    compressed_image_subscription_;

  std::chrono::milliseconds scan_interval_{150};
  std::chrono::steady_clock::time_point last_raw_scan_;
  std::chrono::steady_clock::time_point last_compressed_scan_;
  std::atomic<bool> scan_finished_{false};
  std::atomic_flag scan_in_progress_ = ATOMIC_FLAG_INIT;
  bool stop_after_detection_{true};
  bool try_harder_{true};
  bool try_rotate_{true};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZxingQrDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
