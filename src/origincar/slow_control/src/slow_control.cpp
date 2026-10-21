#include <iostream>
#include "slow_control/slow_control.h"
// #include <chrono> //获取时间
// #include <thread> //睡眠进程
/* 一些固定的参数 */
/* 固定的参数常量 */
const float REAL_MIDX = 920.0; // 将905视为中点，因为摄像头像素点偏差
const float ACC_LINEAR_X = 0.30;//码周围很大一片都没有障碍物时加速冲码
const float STD_LINEAR_X = 0.70;
const float SLOW_LINEAR_X = 0.50;
const double IMAGE_WIDTH = 1920.0;
const double IMAGE_HEIGHT = 1080.0;
const double IMAGE_AREA = 1920.0 * 1080.0;
/*PID*/
class PIDController
{
public:
  PIDController(double Kp, double Ki, double Kd)
      : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0)
  {
    // 初始化上一次更新时间为构造函数调用时的时间
    last_time_ = std::chrono::high_resolution_clock::now();
  }

  // update() 接收当前误差，不需要外部传入 dt，内部会计算时间差
  double update(double error)
  {
    // 获取当前时间
    auto now = std::chrono::high_resolution_clock::now();
    // 计算 dt（时间差，以秒为单位），采用 double 类型
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time_).count();
    if (dt > 0.3)
      dt = 0.1;
    // 如果 dt 太小，赋予一个最小值（防止 dt 为0造成除零错误）
    if (dt < 1e-6)
    {
      dt = 1e-6;
    }

    // 积分累加
    integral_ += error * dt;
    // 求微分（变化率）
    double derivative = (error - prev_error_) / dt;
    // 计算 PID 输出
    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    RCLCPP_WARN(rclcpp::get_logger("PID"), "error：%f,integral：%f，derivative：%f", error, integral_, derivative);
    // 保存当前误差和时间，为下一次调用做准备
    prev_error_ = error;
    last_time_ = now;

    return output;
  }
  int continue_cnt()
  { // 判断是否需要当看不到障碍物了，以上一次的误差在转一下
    RCLCPP_WARN(rclcpp::get_logger("PID"), "pre_error：%f", prev_error_);
    if (fabs(prev_error_) > straight_min)
    {
      double interval = fabs(prev_error_) > 300 ? straight_interval2 : straight_interval1;
      if (prev_error_ > 0)
        prev_error_ -= interval;
      else
        prev_error_ += interval;
      return int(fabs(prev_error_)) / interval;
    }
    return 0;
  }
  double continue_v()
  { // 当看不到障碍物了，以上一次的误差在转一下
    if (fabs(prev_error_) > 300)
      return Kp_ * prev_error_;
    else
      return 0.0;
  }
  int reset()
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
    return 0;
  }

private:
  double Kp_, Ki_, Kd_;
  const int straight_min = 50, straight_interval1 = 35, straight_interval2 = 100;
  double prev_error_, integral_;
  // int cnt;
  std::chrono::high_resolution_clock::time_point last_time_;
};
PIDController to_code_pid(0.002, 0.00001, 0.00001); // 需调参
PIDController bizhang_pid(0.002, 0.0001, 0.0001);
/* 状态控制参数 */
bool is_need_infer = true;  // 是否需要策略推理
bool is_exe_obs = false;    // 是否有需要避的障
float bias_bizhang = 160.0; // 判断往左往右转时让其更偏向于向左转
int is_left = 0;            // 障碍物是否在左侧;1:在左侧；-1：在右侧，0：无需避障
bool is_over_task = false;  // 行走任务是否结束
bool is_find_stop = false;
bool is_get_code = false;   // 已经扫到码
bool is_straight = false;   // 是否需要避完障直走阶段
bool is_acc_tocode = false; //是否加速冲码
/* 避障相关参数 */
float _1l_lose_anguz = 0.90; // 丢线的角速度往左转
// //float  _1l_end_anguz = 1.5;
float _1r_lose_anguz = 0.50; // 往右转
// //float  _1r_end_anguz = 3.0;
const float OBS_JUDGE = 0.34;  // 避障判断阈值,普通状态,以y占画面高的比值
const float OBS_JUDGE2 = 0.19; // 避障判断阈值,未回线状态
double y_ratio = 0.0;          // 障碍物所占图片的比率
// 避障2，朝障碍物旁边冲
double bizhang_x = REAL_MIDX;
const double bizhang_ratio = 1.6;
const int BIZHANG_STRAIGHT = 130; // 避障完后直走阶段的时间，单位ms//试着改小一点
float target_x = REAL_MIDX;       // 障碍物中点
float last_target_x =-1;
double max_rect_x = 0.0;
/* 巡线相关参数 */
float line_x = REAL_MIDX; // 当前线的x坐标
float line_y = 0;         // 当前线的y坐标

float closest_line_x = 0; // 距离障碍物最近的line的横坐标
/* 二维码相关参数 */
float qrcode_score_threshold_ = 0.70; // 二维码置信度阈值
double code_y = 0.0;                  // 二维码y长度
double code_x = 0;                    // 二维码x坐标
double code_min_x = 0;
double code_max_x = IMAGE_WIDTH;
double stop_code_y=150;
// const double CODE_AREA_RATIO=0.01;

// 速度控制的消息变量
auto cmd_vel_message = geometry_msgs::msg::Twist(); // 原来在这。。。。。。。。。。。。。。。。。。。。。。。

void OrigincarControl::stop_subscription(int choice)
{
  if (choice == 0)
  {
    ai_subscription_.reset();
    RCLCPP_INFO(rclcpp::get_logger("origincar_control"), "Subscription topic win_detect stopped");
  }
  else if (choice == 1)
  {
    subscriber_switch_.reset();
    RCLCPP_INFO(rclcpp::get_logger("origincar_control"), "Subscription topic sign_switch stopped");
  }
  else if (choice == 2)
  {
    subscriber_return_.reset();
    RCLCPP_INFO(rclcpp::get_logger("origincar_control"), "Subscription topic sign4return stopped");
  }
}

void OrigincarControl::subscription_callback_switch(const origincar_msg::msg::Sign::SharedPtr msg)
{
  // 如果扫描到二维码，会被调用，关闭hbmem和sign_switch的订阅
  if (msg->sign_data == 3 || msg->sign_data == 4)
  {
    // stop_subscription(0); // 关闭订阅者hbmem_img,这里先不关，防止惯性撞障碍物
    stop_subscription(1); // 关闭订阅者sign_switch
    // 速度置为0停车
    // cmd_vel_message.linear.x = 0.0, cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    // cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;//cmd_vel_message.angular.z = 0.0;
    // cmd_vel_publisher_->publish(cmd_vel_message);
    is_over_task = true;
    is_get_code = true;
    RCLCPP_WARN(rclcpp::get_logger("origincar_control"), "扫到二维码，任务1结束%f", cmd_vel_message.angular.z);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("origincar_control"), "Topic sign_switch did not receive msg equals 3 or 4!");
  }
}

// 构造函数的实现
OrigincarControl::OrigincarControl(const std::string &node_name,
                                   const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
  // 优化QoS设置
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .best_effort()
                 .durability_volatile();

  auto qos1 = rclcpp::QoS(rclcpp::KeepLast(5))
                  .reliable()
                  .durability_volatile();

  // 提示用户按回车键继续
  std::cout << "请按下任意键开始执行!!!" << std::endl;
  std::cin.get();

  // 创建控制消息发布者
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // 二维码的消息本体
  signmsg_ = std::make_shared<origincar_msg::msg::Sign>();

  // 二维码消息的发布者(给上位机发布的)
  qrcode_info_publisher_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", qos1);

  // 上位机回传消息的订阅者
  subscriber_return_ = this->create_subscription<std_msgs::msg::Int32>(
      "/sign4return", qos,
      std::bind(&OrigincarControl::subscription_callback_return, this, std::placeholders::_1));

  // 二维码的消息订阅者
  subscriber_switch_ = this->create_subscription<origincar_msg::msg::Sign>(
      "/sign_switch", 10,
      std::bind(&OrigincarControl::subscription_callback_switch, this, std::placeholders::_1));

  // 创建订阅者，订阅话题 '/win_detect'（ai消息）
  ai_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "win_detection", 10, std::bind(&OrigincarControl::yolo_callback, this, std::placeholders::_1));
}

// 上位机消息的回调函数
void OrigincarControl::subscription_callback_return(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data == 5)
  {
    RCLCPP_WARN(rclcpp::get_logger("origincar_control"), "Kill origincar_control Task 1");
    RCLCPP_WARN(rclcpp::get_logger("origincar_control"), "Shutting down node after QR code detection and publish.");
    rclcpp::shutdown(); // 停止第一条巡线
  }
}

// 订阅ai消息回调函数的实现
int OrigincarControl::yolo_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  // 检查ros2的工作状态
  if (!rclcpp::ok())
    return 0;

  // 把yolo的结果转化为能用的参数
  std::vector<std::shared_ptr<YoloV8Result>> results_hat;
  std::vector<std::shared_ptr<YoloV8Result>> results_line;
  std::vector<std::shared_ptr<YoloV8Result>> results_p;
  std::vector<std::shared_ptr<YoloV8Result>> results_code;

  for (auto &target : msg->targets)
  {
    if (target.type == "hat")
    {
      for (auto &roi : target.rois)
      {
        auto temp_yolo_res = std::make_shared<YoloV8Result>(
            0,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "hat");
        results_hat.push_back(temp_yolo_res);
      }
    }
    else if (target.type == "line")
    {
      for (auto &roi : target.rois)
      {
        auto temp_yolo_res = std::make_shared<YoloV8Result>(
            1,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "line");
        results_line.push_back(temp_yolo_res);
      }
    }
    else if (target.type == "p")
    {
      for (auto &roi : target.rois)
      {
        auto temp_yolo_res = std::make_shared<YoloV8Result>(
            2,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "p");
        results_p.push_back(temp_yolo_res);
      }
    }
    else if (target.type == "code")
    {
      for (auto &roi : target.rois)
      {
        auto temp_yolo_res = std::make_shared<YoloV8Result>(
            3,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "code");
        results_code.push_back(temp_yolo_res);
      }
    }
  }

  // 判断变量：
  bool is_find_line = false;
  bool is_find_code = false;
  bool is_run_code = false;
  for (auto &rect : results_code)
  {
    if (rect->score < qrcode_score_threshold_) // 二维码置信度小于阈值
      continue;
    code_min_x = rect->xmin;
    code_max_x = rect->xmax; // 用于保证码扫全了
    code_y = (rect->ymax - rect->ymin);
    code_x = (rect->xmax + rect->xmin ) / 2+(code_y / 5 < 20 ? code_y / 5 : 20); // 往右冲一点码
    is_find_code = true;
    is_run_code = true;
    // 判断和障碍物的是否重叠，有重叠则不算
    
    bool is_chongdie=false;
    for (auto &hat : results_hat)
    {
      float xx1 = std::max(rect->xmin-30, hat->xmin);
      float xx_1 = std::max(rect->xmin-int(code_y)-20, hat->xmin);
      float xx2 = std::min(rect->xmax+30, hat->xmax);
      float xx_2 = std::min(rect->xmax+int(code_y)+20, hat->xmax);
      // float yy2 = std::min(rect->ymax, hat->ymax);

      if (xx2 > xx1)
      { // 有重叠
        is_run_code = false;
        RCLCPP_ERROR(rclcpp::get_logger("code"), "被障碍物遮挡");
        is_chongdie=true;
        break; // 跳出循环
      }
      else if(xx_2 > xx_1){
        is_chongdie=true;        
      }
    }
    if(!is_acc_tocode&&(results_hat.size()==0||!is_chongdie)&&code_y<80){
      is_acc_tocode=true;
      stop_code_y = 95;
      to_code_pid.reset();
    }
    RCLCPP_ERROR(rclcpp::get_logger("code"), "%dy%f", is_run_code, code_y);
  }

  // 根据yolo结果改变判断
  float max_rect_y = 0.0, now_rect_y = 0.0, hat_bottom_y = 0.0; // 最大锥桶面积，最大锥桶的宽......，最大锥桶的底部纵坐标

  // hat_num = 0;//需要考虑的障碍物
  if(is_exe_obs) last_target_x=target_x;
  for (auto &rect : results_hat)
  {
    if (rect->xmax < 350 || rect->xmin > 1600) // 利用xmax和xmin限制锥桶范围，太偏左右的锥桶不考虑
      continue;
    if (is_left==-1&&rect->xmax<REAL_MIDX ) // 限制任务一最开始左转后不考虑在任务二线上的桶
      continue;
    if(is_exe_obs&&y_ratio<0.8&&((rect->xmax + rect->xmin) / 2-last_target_x)*is_left>500)//不要看到横着的其他桶避错
      {
        RCLCPP_ERROR(rclcpp::get_logger("bi_cuo"), "%f,%f", (rect->xmax + rect->xmin) / 2,last_target_x);
        continue;//不记入考虑，换一个桶
      }
    now_rect_y = (rect->ymax - rect->ymin);
    if (max_rect_y < now_rect_y)
    {
      target_x = (rect->xmax + rect->xmin) / 2; // 中点
      hat_bottom_y = rect->ymax;
      max_rect_y = now_rect_y;
      max_rect_x = rect->xmax - rect->xmin;
    }
  }

  /* 巡线PlanA，进行范围的一定限制，然后取所有范围内line_x的平均值 */

  float min_line_y = 2000.0; // 离车最远的线

  // 第一条巡线过程中的巡线目标点的确定，取纵坐标y一定范围内的最近line的横坐标
  for (auto &rect : results_line)
  {
    //if(rect->xmax<250) continue;
    if (is_left == -1)
    {
      if (rect->xmax < REAL_MIDX-100)
        continue; // 往左避障后，跳过第2条线
    }
    if (rect->ymin < min_line_y) //&&rect->ymax > hat_bottom_y)
    {
      // 找离车最远且在障碍物前面的线
      line_x = (rect->xmin + rect->xmax) / 2;
      min_line_y = rect->ymin;
      is_find_line = true;
    }
    line_y = (rect->ymin + rect->ymax) / 2;
  }
  // 判断是否需要避障、避障是否结束
  double last_y_ratio = y_ratio;
  y_ratio = max_rect_y / IMAGE_HEIGHT; // 最大障碍物的比率
  RCLCPP_INFO(rclcpp::get_logger("y_ratio"), "%f,last:%f", y_ratio, last_y_ratio);
  bool infer_again = false;
  bool bizhang_end = last_y_ratio>0.5&&(last_y_ratio - y_ratio) > 0.3;
  
  bool last_exe_obs = is_exe_obs;
  is_exe_obs = (y_ratio >= OBS_JUDGE && fabs(target_x - REAL_MIDX) <= (500>1.7*max_rect_x?1.7*max_rect_x:500)) || y_ratio >= 0.60; // 判断是否需要避障
  if(!is_run_code&&is_exe_obs&&((target_x -REAL_MIDX)*is_left>0.4*max_rect_x||(is_find_line&&(target_x -line_x)*is_left>2*max_rect_x))) {//桶不影响回线时不避障
    is_exe_obs=0;
    RCLCPP_ERROR(rclcpp::get_logger("obs_change"), "%f", (target_x -REAL_MIDX));
  }
  if (!is_run_code&&is_left)
    infer_again = !last_exe_obs && is_exe_obs;
  
  if (is_left && !is_exe_obs)
  {                                                                                                                        // 车避完障，且没有回线
    is_exe_obs = (y_ratio >= OBS_JUDGE2 && (target_x -REAL_MIDX)*is_left<-100);
      //fabs(target_x + is_left * REAL_MIDX-REAL_MIDX)<380);//target_x + is_left * REAL_MIDX > 300 && target_x + is_left * REAL_MIDX < 1600); //|| y_ratio > 0.5;
    if(is_find_line&&(fabs(target_x-line_x)>0.8*max_rect_x+100||(target_x-line_x)*is_left<-1.1*max_rect_x))//问题：后半部分湾大，最远的线和桶离得远
      {is_exe_obs=0;}
    if (is_exe_obs)                                                                                                        // 不回线直接开始下一次避障
    {
      bizhang_pid.reset();
      is_straight = false;
    }
    else
    {
      if((is_find_line&&(target_x-line_x)*is_left>50)||last_y_ratio<0.7)
        {is_straight = true;
        RCLCPP_ERROR(rclcpp::get_logger("straight"), "%f", (last_target_x-line_x));}
      // is_left=0;//这里置0，丢线就不知道往哪边转
      else 
        is_straight=false;
    }
    RCLCPP_INFO(rclcpp::get_logger("infer"), "车避完障，且没有回线%d %f %f %f", is_exe_obs, target_x-line_x, target_x,0.8*max_rect_x+100);
  }
  if(is_exe_obs&&is_left&&last_target_x>0){//最大的障碍物不是上一次的，需要重新推理
    if(is_left==-1){
      infer_again =(target_x-last_target_x<-300);
    }
    else{
      infer_again =(target_x-last_target_x>300);
    }
  }
  // if(is_left==0 || is_need_infer==false){
  //     is_exe_obs = (y_ratio>=OBS_JUDGE && fabs(target_x-REAL_MIDX)<=500)||y_ratio>=0.70;
  //     if(is_exe_obs)RCLCPP_INFO(rclcpp::get_logger("infer"), "巡线时或避障时遇到障碍物%f",y_ratio);
  // }
  if (bias_bizhang > 0 && is_find_code && code_y > 60)
  {
    bias_bizhang = -20; // 更往右避障
  }
  if (infer_again || (is_need_infer && is_exe_obs && is_left != -1))
  {
    bizhang_pid.reset();
    if (fabs(REAL_MIDX - target_x) <200&&is_find_code)//((is_find_code&&code_y<90)||is_run_code))
    {
      if (target_x > code_x-0.5*code_y)
        is_left = -1;
      else
        is_left = 1;
      RCLCPP_ERROR(rclcpp::get_logger("is_left"), "target_x:%f,code_x%f", target_x, code_x);
    }
    else if (is_find_line && fabs(REAL_MIDX - target_x) < 200 && (fabs(line_x - REAL_MIDX) > 500 || fabs(line_x - target_x) > 500))
    { // 当车离线远的时候往线那边避障，且障碍物离车不太远
      if (target_x > line_x)
        is_left = -1;
      else
        is_left = 1;
      RCLCPP_ERROR(rclcpp::get_logger("is_left"), "target_x:%f,line_x%f", target_x, line_x);
    }
    else
    {
      if (target_x > REAL_MIDX - bias_bizhang)
        is_left = -1;
      else
        is_left = 1;
      RCLCPP_ERROR(rclcpp::get_logger("is_left"), "target_x:%f", target_x);
    }
  } 
  if(bizhang_end) last_target_x=-1;
    // 判断是否停下
  if (is_find_code && is_run_code&&!is_exe_obs)
  {
    qrcode_score_threshold_ = 0.0;
    
    if (code_min_x > 10 && code_max_x < (IMAGE_WIDTH - 20) && ((code_y > stop_code_y && fabs(code_x - REAL_MIDX) < 2 * code_y) || (code_y > 200&&(code_max_x-code_min_x)>0.5*code_y)))
    {
      is_over_task = true;
      is_find_stop=true;
      RCLCPP_ERROR(rclcpp::get_logger("code"), "码大停车%f,%f,%f", cmd_vel_message.angular.z,code_min_x,code_max_x-code_min_x);
    }
    else
    {
      is_over_task = false;
    }
  }
  else
  {
    is_over_task = false;
  }
  /* ====== !!!车的运动!!! ======  */

  if (is_exe_obs)
  {                                                              // 进入避障状态
    bizhang_x = target_x + bizhang_ratio * is_left * max_rect_x; // 障碍物在左边往右避障
    is_need_infer = false;                                       // 不用再判断往左往右转了
    double error = bizhang_x - REAL_MIDX;
    double vz = bizhang_pid.update(error);
    if (vz > 0)
      vz = vz * 0.5; // 右转转得多，处理一下
    cmd_vel_message.angular.z = -vz;
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("bizhang"), "is_left:%d,bizhang:%f, max_rect_x:%f,angular.z: %f", is_left, bizhang_x, max_rect_x, cmd_vel_message.angular.z);
    // if (!is_over_task)                              // 还在第一条线任务中
    cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
    // std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  else if(is_over_task||is_get_code)
  {
    cmd_vel_message.angular.z = 0.0;
    cmd_vel_message.linear.x = 0.0;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("stop"), "停车");

    cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
    if (is_get_code)
      stop_subscription(0); // 关闭这个函数.
  }
  else if (is_acc_tocode){
    is_need_infer = true;
    // is_left = 0;
    double error = code_x - REAL_MIDX;
    double vz = to_code_pid.update(error);
    if (vz < 0)
      vz = vz * 0.8; // 左转转得多，处理一下
    cmd_vel_message.angular.z = -vz;
    cmd_vel_message.linear.x = ACC_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;

    if (!is_over_task)
    {
      cmd_vel_publisher_->publish(cmd_vel_message);
      RCLCPP_WARN(rclcpp::get_logger("acc_code"), "linear.x: %f, angular.z: %f", cmd_vel_message.linear.x, cmd_vel_message.angular.z);
    }
  }
  else if (!is_run_code && bizhang_pid.continue_cnt()>0&&is_straight && is_left != 0 && (!is_need_infer || bizhang_end))
  { // 障碍物刚避完,直走
    // bizhang_x = REAL_MIDX;
    //int continue_cnt = bizhang_pid.continue_cnt();
    //if (continue_cnt)
    //{
      double vz = bizhang_pid.continue_v();
      if (vz > 0)
        vz = vz * 0.8; // 右转转得多，处理一下
      cmd_vel_message.angular.z = -vz;
      // cmd_vel_message.angular.z=0.0;
      cmd_vel_message.linear.x = STD_LINEAR_X;
      cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
      cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
      RCLCPP_WARN(rclcpp::get_logger("bizhang"), "is_left:%d, angular.z: %f", is_left,  cmd_vel_message.angular.z);
      if (!is_over_task)                              // 还在第一条线任务中
        cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
      //return 0;
    //}
    //bizhang_pid.reset();    
    //is_need_infer = true;
    //is_straight = false;
  }
  else if (is_run_code)
  {
    is_need_infer = true;
    // is_left = 0;
    double error = code_x - REAL_MIDX;
    double vz = to_code_pid.update(error);
    if (vz > 0)
      vz = vz * 0.8; // 右转转得多，处理一下
    cmd_vel_message.angular.z = -vz;
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;

    if (!is_over_task)
    {
      cmd_vel_publisher_->publish(cmd_vel_message);
      RCLCPP_WARN(rclcpp::get_logger("to_code"), "linear.x: %f, angular.z: %f", cmd_vel_message.linear.x, cmd_vel_message.angular.z);
    }
  }
  else if (is_find_line)
  { 
    // 巡线
    // 7. 巡线运动控制
    if (-1*is_left*(line_x - REAL_MIDX) < 150)
      is_left = 0;//已回线
    
    is_need_infer = true;
    cmd_vel_message.angular.z = -1.0 * (line_x - REAL_MIDX) * 0.0026;
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("line_node"), "巡线line_x: %f, angular.z: %f,%d", line_x, cmd_vel_message.angular.z,is_straight);
    if (!is_over_task)
      cmd_vel_publisher_->publish(cmd_vel_message);
  }
  else if(is_find_stop&&!is_over_task){
    cmd_vel_message.angular.z = 0.0;
    cmd_vel_message.linear.x = -0.50;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("stop"), "倒退");
    
    cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
    if(is_get_code)
      stop_subscription(0);//关闭这个函数.
  }
  else
  { 
    // 丢线
    is_need_infer = true;
    if (is_left == 1)
    { // 障碍物在左侧，需要右转，转较多
      cmd_vel_message.angular.z = (float)(1.0) * _1l_lose_anguz;
      RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "往左转%f,%f", hat_bottom_y, cmd_vel_message.angular.z);
    }
    else
    { // 障碍物在右侧，需要左转，转较少
      cmd_vel_message.angular.z = (float)(-1.0) * _1r_lose_anguz;
      RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "往右转%f,%f", hat_bottom_y, cmd_vel_message.angular.z);
    }
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    if (!is_over_task)
      cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
  }
  //RCLCPP_INFO(rclcpp::get_logger("bool"), "%d,%d,%d,%d,%d", is_exe_obs, is_find_code, is_find_line, is_over_task,is_left);
  return 0;
}

int main(int argc, char *argv[])
{
  // 初始化 ROS 2
  rclcpp::init(argc, argv);

  // 创建节点实例
  rclcpp::spin(std::make_shared<OrigincarControl>());

  // 清理 ROS 2
  rclcpp::shutdown();
  return 0;
}