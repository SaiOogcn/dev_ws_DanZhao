#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher()
    : Node("cmd_vel_publisher_node"),
      stage_(0)  // 初始阶段为 0（前进）
    {
        // 声明参数（可通过launch或CLI修改）
        this->declare_parameter<double>("forward_duration", 2.0);   // 前进时长（秒）
        this->declare_parameter<double>("backward_duration", 2.0);  // 后退时长（秒）

        // 获取参数
        forward_duration_ = this->get_parameter("forward_duration").as_double();
        backward_duration_ = this->get_parameter("backward_duration").as_double();

        // 初始化发布器（发布到 /cmd_vel 话题）
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 设置定时器，每100ms触发一次回调
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CmdVelPublisher::timer_callback, this)
        );

        // 记录开始时间
        start_time_ = now();
        RCLCPP_INFO(this->get_logger(), "节点启动，开始前进...");
    }

private:
    void timer_callback()
    {
        auto current_time = now();
        auto elapsed = (current_time - start_time_).seconds();

        geometry_msgs::msg::Twist msg;

        if (stage_ == 0) {
            // 第0阶段：前进
            msg.linear.x = 0.9;
            publisher_->publish(msg);

            if (elapsed >= forward_duration_) {
                stage_ = 1;
                start_time_ = current_time;
                RCLCPP_INFO(this->get_logger(), "前进完成，开始后退...");
            }
        }
        else if (stage_ == 1) {
            // 第1阶段：后退
            msg.linear.x = -0.9;
            publisher_->publish(msg);

            if (elapsed >= backward_duration_) {
                stage_ = 2;
                RCLCPP_INFO(this->get_logger(), "后退完成，节点退出...");
                rclcpp::shutdown();  // 结束节点运行
            }
        }
        else if (stage_ == 2){
              // 第2阶段：停止
              msg.linear.x = 0;
              publisher_->publish(msg);
        }
    }

    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    int stage_;  // 0: 前进，1: 后退

    double forward_duration_;
    double backward_duration_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
