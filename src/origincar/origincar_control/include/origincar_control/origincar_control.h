#include "ai_msgs/msg/perception_targets.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "origincar_msg/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"


//存储yolo的检测结果
struct YoloV8Result {
    // 目标类别ID
    int id;
    // 目标检测框
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    // 检测结果的置信度
    float score;
    // 目标类别
    std::string class_name;
  
    YoloV8Result(int id_,
                 float xmin_,
                 float ymin_,
                 float xmax_,
                 float ymax_,
                 float score_,
                 std::string class_name_)
        : id(id_),
          xmin(xmin_),
          ymin(ymin_),
          xmax(xmax_),
          ymax(ymax_),
          score(score_),
          class_name(class_name_) {}
  
    friend bool operator>(const YoloV8Result &lhs, const YoloV8Result &rhs) {
      return (lhs.score > rhs.score);
    }
  };



class OrigincarControl : public rclcpp::Node
{
public:
  //声明构造函数
  OrigincarControl(const std::string& node_name = "origincar_control",
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // yolo检测消息订阅者
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_subscription_;  

  // cmd_vel速度消息发布者
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // 第一赛段已由人工按键启动
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_started_publisher_;

  // 扫码后通知 LIO：视觉控制器已停止发布运动指令
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_released_publisher_;

  // sign4return的订阅者(来自上位机)
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_return_; 

  // sign_switch的订阅者(来自二维码节点)
  rclcpp::Subscription<origincar_msg::msg::Sign>::SharedPtr subscriber_switch_;

  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string qr_result_topic_{"/sign_switch"};
  std::string control_started_topic_{"/origincar_control/started"};
  std::string control_released_topic_{"/origincar_control/control_released"};
  bool wait_for_key_{true};
  bool started_{false};
  bool control_released_{false};
  
  // 回调函数，处理接收到的ai消息
  int yolo_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  // sign4return的回调函数（收到上位机的消息）
  void subscription_callback_return(const std_msgs::msg::Int32::SharedPtr msg);   

  // 关闭三个订阅节点的函数
  void stop_subscription(int choice);

  // sign_switch的回调函数(收到二维码节点的消息)
  void subscription_callback_switch(const origincar_msg::msg::Sign::SharedPtr msg);




};//这个分号必须有
