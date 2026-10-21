#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

class TakePhotoNode : public rclcpp::Node
{
public:
  TakePhotoNode()
  : Node("take_photo_node")
  {
    declare_parameter<std::string>("image_topic", "/image_raw");

    // 触发拍照的话题 (std_msgs::msg::Bool, data=true 时触发)
    declare_parameter<std::string>("trigger_topic", "/take_photo/trigger");

    // 暂停 cure_line 多久后恢复
    declare_parameter<double>("pause_duration_sec", 3.0);

    // 保存图片路径
    declare_parameter<std::string>(
      "photo_save_path",
      "/userdata/dev_ws/src/photo/my_photo.png");

    // 你之前写好的 Python 通信脚本路径
    declare_parameter<std::string>(
      "comm_script_path",
      "/userdata/dev_ws/src/photo/rdk_client_test_0.4.py");

    // 主机 PC 的 IP
    declare_parameter<std::string>(
      "comm_server_ip",
      "172.20.10.2");

    // 用于控制 cure_line 停车/恢复的 ROS2 话题
    // true  = 停车
    // false = 恢复巡线
    declare_parameter<std::string>(
      "stop_command_topic",
      "/take_photo/stop_cure_line");

    // 是否只执行一次（默认 false，支持多次触发）
    declare_parameter<bool>("run_once", false);

    // 是否等待 Python 通信脚本执行结束
    // false：调用脚本后，不等它结束，3 秒后恢复 cure_line
    // true ：等脚本执行结束后，再恢复 cure_line
    declare_parameter<bool>("wait_script_finish", false);

    image_topic_ = get_parameter("image_topic").as_string();
    trigger_topic_ = get_parameter("trigger_topic").as_string();
    pause_duration_sec_ = get_parameter("pause_duration_sec").as_double();
    photo_save_path_ = get_parameter("photo_save_path").as_string();
    comm_script_path_ = get_parameter("comm_script_path").as_string();
    comm_server_ip_ = get_parameter("comm_server_ip").as_string();
    stop_command_topic_ = get_parameter("stop_command_topic").as_string();
    run_once_ = get_parameter("run_once").as_bool();
    wait_script_finish_ = get_parameter("wait_script_finish").as_bool();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TakePhotoNode::on_image, this, std::placeholders::_1));

    auto stop_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_command_topic_, stop_qos);

    auto trigger_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
      trigger_topic_,
      trigger_qos,
      std::bind(&TakePhotoNode::on_trigger, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "【take_photo】节点已启动 | 图像话题=%s | 触发话题=%s | 停车 %.1f 秒 | 保存路径=%s | 通信脚本=%s | 主机IP=%s | 停车控制话题=%s",
      image_topic_.c_str(),
      trigger_topic_.c_str(),
      pause_duration_sec_,
      photo_save_path_.c_str(),
      comm_script_path_.c_str(),
      comm_server_ip_.c_str(),
      stop_command_topic_.c_str());
  }

private:
  static std::string shell_quote(const std::string& s)
  {
    std::string out = "'";

    for (char c : s) {
      if (c == '\'') {
        out += "'\\''";
      } else {
        out += c;
      }
    }

    out += "'";
    return out;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat bgr;

  try {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "【take_photo】收到图像 | encoding=%s | w=%u | h=%u | step=%u | data_size=%zu",
      msg->encoding.c_str(),
      msg->width,
      msg->height,
      msg->step,
      msg->data.size());

    if (msg->encoding == "bgr8") {
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    else if (msg->encoding == "rgb8") {
      cv::Mat rgb = cv_bridge::toCvCopy(msg, "rgb8")->image;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    }
    else if (msg->encoding == "nv12" || msg->encoding == "NV12") {
      const int width = static_cast<int>(msg->width);
      const int height = static_cast<int>(msg->height);

      const size_t need_size =
        static_cast<size_t>(width) * static_cast<size_t>(height) * 3 / 2;

      if (msg->data.size() < need_size) {
        RCLCPP_ERROR(
          get_logger(),
          "【take_photo】NV12 图像数据长度不足，当前=%zu，需要=%zu",
          msg->data.size(),
          need_size);
        return;
      }

      cv::Mat nv12(
        height * 3 / 2,
        width,
        CV_8UC1,
        const_cast<unsigned char*>(msg->data.data()));

      cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    }
    else {
      RCLCPP_ERROR(
        get_logger(),
        "【take_photo】暂不支持的图像编码：%s",
        msg->encoding.c_str());
      return;
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "【take_photo】图像转换失败：%s", e.what());
    return;
  }

  if (bgr.empty()) {
    RCLCPP_ERROR(get_logger(), "【take_photo】转换后的图像为空");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_frame_ = bgr.clone();
    has_frame_ = true;
  }
}

  void on_trigger(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }

    if (running_) {
      RCLCPP_WARN(get_logger(), "【take_photo】正在执行拍照流程，忽略本次触发");
      return;
    }

    if (run_once_ && already_triggered_) {
      RCLCPP_WARN(get_logger(), "【take_photo】已执行过一次（run_once=true），忽略本次触发");
      return;
    }

    already_triggered_ = true;
    running_ = true;

    RCLCPP_INFO(get_logger(), "【take_photo】收到触发信号，开始拍照流程");
    std::thread(&TakePhotoNode::do_photo_and_comm, this).detach();
  }

  void do_photo_and_comm()
  {
    RCLCPP_INFO(get_logger(), "【take_photo】触发拍照通信流程");

    // 计时到达后，先通过 ROS2 话题通知 cure_line 停车。
    publish_cure_line_stop(true);

    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);

      if (!has_frame_ || latest_frame_.empty()) {
        RCLCPP_ERROR(get_logger(), "【take_photo】当前还没有收到有效摄像头图像，无法拍照");
        sleep_then_resume();
        running_ = false;
        return;
      }

      frame = latest_frame_.clone();
    }

    if (!cv::imwrite(photo_save_path_, frame)) {
      RCLCPP_ERROR(
        get_logger(),
        "【take_photo】图片保存失败：%s",
        photo_save_path_.c_str());

      sleep_then_resume();
      running_ = false;
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "【take_photo】已保存当前帧：%s",
      photo_save_path_.c_str());

    if (wait_script_finish_) {
      call_comm_script_sync();
      sleep_then_resume();
    } else {
      call_comm_script_async();
      sleep_then_resume();
    }

    running_ = false;
  }

  void publish_cure_line_stop(bool stop)
  {
    std_msgs::msg::Bool msg;
    msg.data = stop;

    stop_pub_->publish(msg);

    RCLCPP_WARN(
      get_logger(),
      "【take_photo】发布 cure_line 控制命令：%s | 话题=%s",
      stop ? "停车 true" : "恢复 false",
      stop_command_topic_.c_str());
  }

  void sleep_then_resume()
  {
    RCLCPP_INFO(
      get_logger(),
      "【take_photo】保持停车 %.1f 秒",
      pause_duration_sec_);

    std::this_thread::sleep_for(
      std::chrono::duration<double>(pause_duration_sec_));

    publish_cure_line_stop(false);
  }

  void call_comm_script_sync()
  {
    std::ostringstream cmd;
    cmd << " python3 " << shell_quote(comm_script_path_);

    RCLCPP_INFO(
      get_logger(),
      "【take_photo】同步调用通信脚本：%s",
      cmd.str().c_str());

    const int ret = std::system(cmd.str().c_str());

    RCLCPP_INFO(
      get_logger(),
      "【take_photo】通信脚本执行结束，返回值=%d",
      ret);
  }

  void call_comm_script_async()
  {
    std::ostringstream cmd;
    cmd << "SERVER_IP=" << shell_quote(comm_server_ip_)
        << " IMAGE_PATH=" << shell_quote(photo_save_path_)
        << " python3 " << shell_quote(comm_script_path_);

    const std::string cmd_str = cmd.str();

    RCLCPP_INFO(
      get_logger(),
      "【take_photo】异步调用通信脚本：%s",
      cmd_str.c_str());

    std::thread([this, cmd_str]() {
      const int ret = std::system(cmd_str.c_str());

      RCLCPP_INFO(
        get_logger(),
        "【take_photo】通信脚本执行结束，返回值=%d",
        ret);
    }).detach();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;

  std::string image_topic_;
  std::string trigger_topic_;
  std::string photo_save_path_;
  std::string comm_script_path_;
  std::string comm_server_ip_;
  std::string stop_command_topic_;

  double pause_duration_sec_{3.0};

  bool run_once_{false};
  bool wait_script_finish_{false};

  std::mutex image_mutex_;
  cv::Mat latest_frame_;
  bool has_frame_{false};

  std::atomic<bool> already_triggered_{false};
  std::atomic<bool> running_{false};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakePhotoNode>());
  rclcpp::shutdown();
  return 0;
}