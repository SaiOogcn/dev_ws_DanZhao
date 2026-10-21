#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>
#include <cstring>
#include <algorithm>

class ImageBridgeHSVNode : public rclcpp::Node
{
public:
    ImageBridgeHSVNode() : Node("image_bridge_hsv")
    {
        // ========= 参数声明 =========
        this->declare_parameter<std::string>("input_image_topic", "/image_new");
        this->declare_parameter<std::string>("output_hsv_topic", "/image_new_hsv");

        // 你的摄像头消息 encoding 写的是 rgb8，
        // 但实际 data 排列是 B-G-R，所以这里默认按 BGR 解析
        this->declare_parameter<bool>("input_is_bgr", true);

        // 是否周期性打印第一帧/部分像素调试信息
        this->declare_parameter<bool>("show_debug_log", true);
        this->declare_parameter<bool>("start_enabled", true);
        this->declare_parameter<std::string>("enable_topic", "");

        // ========= 读取参数 =========
        input_image_topic_ = this->get_parameter("input_image_topic").as_string();
        output_hsv_topic_  = this->get_parameter("output_hsv_topic").as_string();

        input_is_bgr_      = this->get_parameter("input_is_bgr").as_bool();
        show_debug_log_    = this->get_parameter("show_debug_log").as_bool();
        processing_enabled_ = this->get_parameter("start_enabled").as_bool();
        enable_topic_ = this->get_parameter("enable_topic").as_string();

// ========= QoS =========

// 订阅摄像头使用 SensorDataQoS
auto sub_qos = rclcpp::SensorDataQoS();

// 输出 HSV 使用 reliable
auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
pub_qos.reliable();
pub_qos.durability_volatile();

// ========= 发布与订阅 =========
hsv_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_hsv_topic_,
    pub_qos
);

image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_image_topic_,
    sub_qos,
    std::bind(
        &ImageBridgeHSVNode::imageCallback,
        this,
        std::placeholders::_1
    )
);

if (!enable_topic_.empty())
{
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        enable_topic_,
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            if (msg->data && !processing_enabled_)
            {
                processing_enabled_ = true;
                RCLCPP_INFO(this->get_logger(), "HSV processing enabled by %s", enable_topic_.c_str());
            }
        }
    );
}

        debug_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ImageBridgeHSVNode::timerDebugCallback, this)
        );

        callback_count_ = 0;
        publish_count_ = 0;

        RCLCPP_INFO(this->get_logger(), "image_bridge_hsv started.");
        RCLCPP_INFO(this->get_logger(), "Subscribe image topic: %s", input_image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish HSV topic: %s", output_hsv_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "input_is_bgr: %s", input_is_bgr_ ? "true" : "false");
        RCLCPP_INFO(
            this->get_logger(), "processing: %s, enable_topic: %s",
            processing_enabled_ ? "enabled" : "waiting",
            enable_topic_.empty() ? "disabled" : enable_topic_.c_str());
    }

private:
    void timerDebugCallback()
    {
        if (!show_debug_log_)
        {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[TIMER] callback_count=%d publish_count=%d",
            callback_count_,
            publish_count_
        );
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!processing_enabled_)
        {
            return;
        }
        callback_count_++;

        const int height = static_cast<int>(msg->height);
        const int width  = static_cast<int>(msg->width);
        const int step   = static_cast<int>(msg->step);

        if (height <= 0 || width <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid image size: width=%d height=%d", width, height);
            return;
        }

        if (step < width * 3)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid image step: step=%d, width*3=%d",
                step,
                width * 3
            );
            return;
        }

        const size_t expected_size = static_cast<size_t>(height) * static_cast<size_t>(step);

        if (msg->data.size() < expected_size)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image data too small: data.size=%zu, expected=%zu",
                msg->data.size(),
                expected_size
            );
            return;
        }

        if (callback_count_ % 30 == 1)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[IMAGE] width=%d height=%d encoding=%s step=%d data_size=%zu",
                width,
                height,
                msg->encoding.c_str(),
                step,
                msg->data.size()
            );
        }

        // ========= 1. 用 Mat 包装原始 data =========
        //
        // 你的摄像头：
        //   height = 1080
        //   width  = 1920
        //   step   = 5760
        //
        // 每个像素 3 字节。
        // 虽然 msg->encoding 是 rgb8，但你已经确认实际排列是 B-G-R。
        //
        // 注意：这里没有直接 memcpy，cv::Mat 只是引用 msg->data 内存。
        cv::Mat raw_3ch(
            height,
            width,
            CV_8UC3,
            const_cast<unsigned char *>(msg->data.data()),
            static_cast<size_t>(step)
        );

        // ========= 3. 转 HSV =========
        cv::Mat hsv_img;

        if (input_is_bgr_)
        {
            // 实际 data 顺序是 B-G-R
            cv::cvtColor(raw_3ch, hsv_img, cv::COLOR_BGR2HSV);
        }
        else
        {
            // 如果后续确认 data 真的是 R-G-B，可以把 input_is_bgr 设置为 false
            cv::cvtColor(raw_3ch, hsv_img, cv::COLOR_RGB2HSV);
        }

        if (hsv_img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "HSV image is empty.");
            return;
        }

        // ========= 4. 发布 HSV 图像 =========
        //
        // ROS Image 没有一个通用标准叫 hsv8 的 encoding。
        // 所以这里用 8UC3，表示每个像素 3 个 uint8 通道。
        //
        // /image_new_hsv 的 data 顺序为：
        //   H, S, V, H, S, V, ...
        //
        // OpenCV HSV 范围：
        //   H: 0 ~ 179
        //   S: 0 ~ 255
        //   V: 0 ~ 255
        sensor_msgs::msg::Image hsv_msg;
        hsv_msg.header = msg->header;
        hsv_msg.height = static_cast<uint32_t>(hsv_img.rows);
        hsv_msg.width  = static_cast<uint32_t>(hsv_img.cols);
        hsv_msg.encoding = "8UC3";
        hsv_msg.is_bigendian = msg->is_bigendian;
        hsv_msg.step = static_cast<uint32_t>(hsv_img.step);

        const size_t hsv_data_size =
            static_cast<size_t>(hsv_msg.step) * static_cast<size_t>(hsv_msg.height);

        hsv_msg.data.resize(hsv_data_size);

        if (hsv_img.isContinuous())
        {
            std::memcpy(hsv_msg.data.data(), hsv_img.data, hsv_data_size);
        }
        else
        {
            for (int y = 0; y < hsv_img.rows; ++y)
            {
                const unsigned char *src_row = hsv_img.ptr<unsigned char>(y);
                unsigned char *dst_row = hsv_msg.data.data() + static_cast<size_t>(y) * hsv_msg.step;
                std::memcpy(dst_row, src_row, hsv_msg.step);
            }
        }

        hsv_pub_->publish(hsv_msg);
        publish_count_++;

        if (show_debug_log_ && callback_count_ % 30 == 1)
        {
            const cv::Vec3b hsv00 = hsv_img.at<cv::Vec3b>(0, 0);

            RCLCPP_INFO(
                this->get_logger(),
                "[PUBLISH] /image_new_hsv width=%u height=%u step=%u encoding=%s  HSV(0,0)=[%u,%u,%u]",
                hsv_msg.width,
                hsv_msg.height,
                hsv_msg.step,
                hsv_msg.encoding.c_str(),
                static_cast<unsigned int>(hsv00[0]),
                static_cast<unsigned int>(hsv00[1]),
                static_cast<unsigned int>(hsv00[2])
            );
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hsv_pub_;
    rclcpp::TimerBase::SharedPtr debug_timer_;

    std::string input_image_topic_;
    std::string output_hsv_topic_;
    std::string enable_topic_;

    bool input_is_bgr_;
    bool show_debug_log_;
    bool processing_enabled_;

    int callback_count_;
    int publish_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageBridgeHSVNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
