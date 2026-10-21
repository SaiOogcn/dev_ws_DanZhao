#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CompressedToRawNode : public rclcpp::Node
{
public:
    CompressedToRawNode() : Node("compressed_to_raw_node")
    {
        this->declare_parameter<std::string>("input_topic", "/image");
        this->declare_parameter<std::string>("output_topic", "/image_raw");

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        auto qos = rclcpp::SensorDataQoS();

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, qos);

        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic_,
            qos,
            std::bind(&CompressedToRawNode::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(),
                    "compressed_to_raw_node started. input=%s output=%s",
                    input_topic_.c_str(), output_topic_.c_str());
    }

private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try {
            // 把压缩数据拷到 OpenCV buffer
            cv::Mat compressed_data(1, static_cast<int>(msg->data.size()), CV_8UC1,
                                    const_cast<unsigned char*>(msg->data.data()));

            // 解码成 BGR 图像
            cv::Mat bgr = cv::imdecode(compressed_data, cv::IMREAD_COLOR);

            if (bgr.empty()) {
                RCLCPP_WARN(this->get_logger(), "Failed to decode compressed image.");
                return;
            }

            // 转成 ROS Image 消息
            std_msgs::msg::Header header = msg->header;
            auto image_msg = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();

            pub_->publish(*image_msg);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in decode callback: %s", e.what());
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::string input_topic_;
    std::string output_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompressedToRawNode>());
    rclcpp::shutdown();
    return 0;
}