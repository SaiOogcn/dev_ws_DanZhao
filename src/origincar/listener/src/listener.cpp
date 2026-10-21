#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cstdlib>      // system()
#include <chrono>       // std::this_thread::sleep_for
#include <thread>       // std::this_thread

using std::placeholders::_1;

class ShutdownListener : public rclcpp::Node
{
public:
    ShutdownListener() : Node("shutdown_listener")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sign4return", 10, std::bind(&ShutdownListener::callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "ShutdownListener 节点启动，等待 /sign4return 消息...");
    }

private:
    void callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (msg->data == 5)
        {
            RCLCPP_INFO(this->get_logger(), "收到值为 5，准备杀死进程...");

            // 执行 pkill 命令
            int ret1 = std::system("pkill -f qr_code/qr_code");
            int ret2 = std::system("pkill -f qr_code_detection/qr_detection_node");

            if (ret1 == 0 && ret2 == 0)
                RCLCPP_INFO(this->get_logger(), "成功执行 pkill。");
            else
                RCLCPP_WARN(this->get_logger(), "至少一个 pkill 命令未成功执行（可能进程不存在）。");

            // 等待 2 秒
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // 关闭节点
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};
  
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShutdownListener>());
    rclcpp::shutdown();
    return 0;
}
