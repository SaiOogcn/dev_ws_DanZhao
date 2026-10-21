//2025-8-2 0
#include <iostream>
#include "origincar_control/origincar_control.h"
//#include <chrono> //获取时间
//#include <thread> //睡眠进程
/* 一些固定的参数 */
int timer=0;
int hat_num=0;
int hat_num_cnt=0;
/* 固定的参数常量 */
const float REAL_MIDX = 920.0;
const float ACC_LINEAR_X = 1.20;
float STD_LINEAR_X = 0.90;
//const float SLOW_LINEAR_X = 0.50;//丢线的时候以这个速度，这样加减速需要时间会影响后面的避障
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
    cnt=0;
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
    RCLCPP_WARN(rclcpp::get_logger("PID"), "error：%f,integral：%f，derivative：%f,v%f", error, integral_, derivative,output);
    // 保存当前误差和时间，为下一次调用做准备
    prev_error_ = error;
    last_time_ = now;

    return output;
  }
  int continue_cnt()
  { // 判断是否需要当看不到障碍物了，以上一次的误差在转一下
    cnt++;
    RCLCPP_WARN(rclcpp::get_logger("PID"), "pre_error：%f", prev_error_);
    if(cnt>7) return 0;
    if (fabs(prev_error_) > straight_min)
    {
      double interval=fabs(prev_error_)>200?straight_interval2:straight_interval1;
      if (prev_error_ > 0)
        prev_error_ -= interval;
      else
        prev_error_ += interval;
      return int(fabs(prev_error_) - straight_min) / interval;
    }
    return 0;
  }
  double continue_v()
  { // 当看不到障碍物了，以上一次的误差在转一下
    if (fabs(prev_error_)>50)
      return Kp_ * prev_error_;
    else
      return 0.0;
  }
  int reset()
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
    cnt=0;
    return 0;
  }
  int reset_straight()
    {
        //prev_error_ = 0.0;
        //integral_ = 0.0;
        cnt = 0;
        return 0;
    }

private:
  double Kp_, Ki_, Kd_;
  const int straight_min=10,straight_interval1=15,straight_interval2=120;
  double prev_error_, integral_;
  int cnt;
  // int cnt;
  std::chrono::high_resolution_clock::time_point last_time_;
};
PIDController to_code_pid(0.003, 0.00007, 0.00007); // 需调参
PIDController bizhang_pid(0.002, 0.00007, 0.00007);
/* 状态控制参数 */
int have_find_line_timer = 0;//避完一个障后，只要看到线九尾true
bool daotui=false;
int is_run_code_cnt=0; //所有受冲码影响的都要改！！！
int start_huixian= 100;
bool is_start = true; //是否是开始状态
bool is_need_infer = true;        // 是否需要策略推理
bool is_exe_obs = false;          // 是否有需要避的障
float bias_bizhang = 180.0; // 判断往左往右转时让其更偏向于向左转
int is_left = 0;                  // 障碍物是否在左侧;1:在左侧；-1：在右侧，0：无需避障且已回线
int loseline_is_left=0; 
int loseline_right_cnt = 0;
bool is_over_task = false;        // 行走任务是否结束
bool last_over_task=false;
int is_over_task_cnt=0;
bool is_find_stop = false;
bool is_get_code = false; //已经扫到码
bool is_straight = false;        // 是否需要避完障直走阶段
int straight_cnt =0; 
bool is_acc_tocode =false; //加速冲码
bool is_scan_code=false;//是否需要慢速以保证码清晰地被扫到
int is_scan_code_cnt=0;
/* 避障相关参数 */
const float _1l_lose_anguz = 1.8; // 丢线的角速度往左转1.2
// //float  _1l_end_anguz = 1.5;
float _1r_lose_anguz = 1.45; // 往右转 0.9
// //float  _1r_end_anguz = 3.0;
float OBS_JUDGE = 0.22;  // 避障判断阈值,普通状态,以y占画面高的比值
const float OBS_JUDGE2 = 0.185; // 避障判断阈值,未回线状态
double y_ratio = 0.0;          // 障碍物所占图片的比率
// 避障2，朝障碍物旁边冲
double bizhang_x = REAL_MIDX;
const double bizhang_ratio = 1.60;
//const int BIZHANG_STRAIGHT = 130; // 避障完后直走阶段的时间，单位ms//试着改小一点
float target_x = REAL_MIDX;       // 障碍物中点
float last_target_x =-1;
double max_rect_x = 0.0;
/* 巡线相关参数 */
float line_x = REAL_MIDX; // 当前线的x坐标
float line_y = 0;         // 当前线的y坐标
const double xunxian_k=0.0016; //巡线的参数
float closest_line_x = 0;// 距离障碍物最近的line的横坐标
float closest_line_y = 0;
/* 二维码相关参数 */
const float qrcode_score_threshold_ = 0.70; // 二维码置信度阈值，移到yolo识别的地方了
double code_y = 0.0;                        // 二维码y长度
double code_x = 0;     // 二维码x坐标
double code_min_x=0;
double code_max_x=IMAGE_WIDTH;     
double stop_code_y=110;              
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
  if (!started_) {
    return;
  }
  // 扫码后立即停止视觉控制并明确释放速度控制权。
  if (msg->sign_data == 3 || msg->sign_data == 4)
  {
    if (control_released_) {
      return;
    }
    is_get_code = true;
    control_released_ = true;
    stop_subscription(0);

    cmd_vel_message = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(cmd_vel_message);

    std_msgs::msg::Bool released;
    released.data = true;
    control_released_publisher_->publish(released);
    RCLCPP_WARN(
      rclcpp::get_logger("origincar_control"),
      "扫到二维码：视觉控制已停车并释放控制权，sign_data=%d", msg->sign_data);
    stop_subscription(1);
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
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  qr_result_topic_ = declare_parameter<std::string>("qr_result_topic", "/sign_switch");
  control_started_topic_ = declare_parameter<std::string>(
    "control_started_topic", "/origincar_control/started");
  control_released_topic_ = declare_parameter<std::string>(
    "control_released_topic", "/origincar_control/control_released");
  wait_for_key_ = declare_parameter<bool>("wait_for_key", true);

  // 优化QoS设置
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .best_effort()
                 .durability_volatile();
  const auto qr_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                        .reliable()
                        .durability_volatile();
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  // 创建控制消息发布者
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  control_started_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      control_started_topic_, state_qos);
  control_released_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      control_released_topic_, state_qos);

  // 上位机回传消息的订阅者
  subscriber_return_ = this->create_subscription<std_msgs::msg::Int32>(
      "/sign4return", qos,
      std::bind(&OrigincarControl::subscription_callback_return, this, std::placeholders::_1));

  if (wait_for_key_) {
    std::cout << "请按下任意键开始执行!!!" << std::endl;
    std::cin.get();
  }
  started_ = true;

  // 按键前不创建运动和二维码订阅，避免队列中残留的识别结果触发提前交接。
  subscriber_switch_ = this->create_subscription<origincar_msg::msg::Sign>(
      qr_result_topic_, qr_qos,
      std::bind(&OrigincarControl::subscription_callback_switch, this, std::placeholders::_1));
  ai_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "win_detection", 10, std::bind(&OrigincarControl::yolo_callback, this, std::placeholders::_1));

  std_msgs::msg::Bool started;
  started.data = true;
  control_started_publisher_->publish(started);
  RCLCPP_INFO(
    rclcpp::get_logger("origincar_control"),
    "第一赛段视觉控制已启动，速度直接发布到 %s", cmd_vel_topic_.c_str());
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
  if (!started_ || control_released_) {
    return 0;
  }
  timer++;
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
    //RCLCPP_ERROR(rclcpp::get_logger("code"), "score%f", rect->score);
    // if (rect->score < qrcode_score_threshold_) // 二维码置信度小于阈值
    //   {
    //     RCLCPP_ERROR(rclcpp::get_logger("code"), "置信度%f",rect->score);
    //     //continue;
    //   }
    if(rect->xmax<150) continue;
    code_min_x=rect->xmin;
    code_max_x=rect->xmax;//用于保证码扫全了
    code_y = (rect->ymax - rect->ymin);
    code_x = (rect->xmax + rect->xmin+(code_y/10<50?code_y/10:50)) / 2;//往右冲一点码
    is_find_code = true;
    is_run_code = true;
    // 判断和障碍物的是否重叠，有重叠则不算
    for (auto &hat : results_hat)
    {
      float xx1 = std::max(rect->xmin, hat->xmin+20);
      // float yy1 = std::max(rect->ymin, hat->ymin);
      float xx2 = std::min(rect->xmax+40, hat->xmax-40);
      // float yy2 = std::min(rect->ymax, hat->ymax);

      if (xx2 > xx1)
      { // 有重叠
        is_run_code = false;
        RCLCPP_ERROR(rclcpp::get_logger("code"), "被障碍物遮挡");
        break; // 跳出循环
      }
    }
    
    //RCLCPP_ERROR(rclcpp::get_logger("code"), "%dy%f", is_run_code, code_y);
    
  }
  if(is_run_code) is_run_code_cnt++;
  else is_run_code_cnt=0;
 
  // 根据yolo结果改变判断
  float max_rect_y = 0.0, now_rect_y = 0.0, hat_bottom_y = 0.0; // 最大锥桶面积，最大锥桶的宽......，最大锥桶的底部纵坐标

  hat_num = 0;//需要考虑的障碍物
  if(is_exe_obs) last_target_x=target_x;
  for (auto &rect : results_hat)
  {
    if (!is_exe_obs&&(rect->xmax < 400 || rect->xmin > 1600||(rect->xmax + rect->xmin) / 2<200)) // 利用xmax和xmin限制锥桶范围，太偏左右的锥桶不考虑
      continue;
    // else if ((rect->xmax < 200 || rect->xmin > 1750||(rect->xmax + rect->xmin) / 2<200))
    //   continue;
    if((start_huixian>2&&start_huixian<16)&&!is_exe_obs&&(rect->xmax + rect->xmin) / 2<(1300-50*(start_huixian-10)))continue;
    if(timer<60&&is_left==-1&&!is_exe_obs&&(rect->xmax + rect->xmin) / 2<750)
      continue; 
    if(is_left==-1&&(!is_exe_obs||(y_ratio>0.40&&last_target_x>REAL_MIDX))&&(rect->xmax + rect->xmin) / 2<550) continue;
    if(is_left==-1&&timer<80&&(!is_exe_obs)&&loseline_right_cnt<8&&loseline_right_cnt>1&&(rect->xmax + rect->xmin) / 2<REAL_MIDX) continue;
    if(is_left==1&&(!is_exe_obs||(y_ratio>0.40&&last_target_x<REAL_MIDX))&&(rect->xmax + rect->xmin) / 2>1400) continue;
    // if (is_first_obs && PostProcess_cnt > 0 && rect->xmin < 400 ) // 限制任务一最开始，不看任务二的桶
    //   continue;
    ++hat_num;
    
    now_rect_y = (rect->ymax - rect->ymin);
    
    if (max_rect_y < now_rect_y)
    {
      if(is_exe_obs&&((rect->xmax - rect->xmin)/IMAGE_HEIGHT<0.7)&&((rect->xmax + rect->xmin) / 2-std::min(float(1500.0),last_target_x))*is_left>500)//不要看到横着的其他桶避错
      {
        RCLCPP_ERROR(rclcpp::get_logger("bi_cuo"), "%f,%f", (rect->xmax + rect->xmin) / 2,last_target_x);
        continue;//不记入考虑，换一个桶 
      }
      target_x = (rect->xmax + rect->xmin) / 2; // 中点
      hat_bottom_y = rect->ymax;
      max_rect_y = now_rect_y;
      max_rect_x = rect->xmax - rect->xmin;
    }
  }
  if(hat_num==0) hat_num_cnt++;
  else hat_num_cnt=0;
  if(!is_acc_tocode&&is_run_code_cnt>3&&(hat_num_cnt>2)&&code_y<80){
      is_acc_tocode=true;
      stop_code_y = 95;
      to_code_pid.reset();
    }
  /* 巡线PlanA，进行范围的一定限制，然后取所有范围内line_x的平均值 */

  float min_line_y = 2000.0; // 离车最远的线
  float dis_hatline_y = 2000.0; //离桶最近的线
  // 第一条巡线过程中的巡线目标点的确定，取纵坐标y一定范围内的最近line的横坐标
  for (auto &rect : results_line)
  {
    if(rect->ymax<50) continue;
    if(is_left==-1&&timer<100&&rect->xmax<REAL_MIDX-50) continue;//往左避障后，跳过第2条线
     if(is_left==-1&&timer<90&&(loseline_right_cnt<13||!is_find_line)&&loseline_right_cnt>0&&(rect->xmax + rect->xmin) / 2<REAL_MIDX+50) continue;
    //找离桶最近的线
    if(fabs(hat_bottom_y - rect->ymin)< dis_hatline_y) { // 找到距离锥桶最近的line的横坐标
        closest_line_x = (rect->xmin + rect->xmax) / 2;
        closest_line_y=rect->ymin;
        dis_hatline_y = fabs(hat_bottom_y - rect->ymin);
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
  //判断是否需要避障、避障是否结束
  double last_y_ratio=y_ratio;
  y_ratio = max_rect_y / IMAGE_HEIGHT;//最大障碍物的比率
  // if(last_y_ratio>y_ratio+0.15){//
  //   bizhang_pid.reset();
  // }
 
  
  bool infer_again=false;
  bool bizhang_end=(last_y_ratio-y_ratio)>0.1;
  bool last_exe_obs=is_exe_obs;
  
  //is_exe_obs = (y_ratio >= OBS_JUDGE && fabs(target_x - REAL_MIDX) <= (500>1.5*max_rect_x?1.5*max_rect_x:500)) || y_ratio >= 0.70;//判断是否需要避障
  
  is_exe_obs = y_ratio >= 0.50||(y_ratio >= OBS_JUDGE && ((last_exe_obs&&last_y_ratio<0.50)||fabs(target_x -REAL_MIDX)<(500>1.55*max_rect_x?1.55*max_rect_x:500)));
  if(!is_exe_obs&&is_find_line&&is_left==0&&bias_bizhang<100){
    is_exe_obs=(y_ratio >= OBS_JUDGE && fabs(target_x -REAL_MIDX)<600);
  }
  if((y_ratio< 0.35||!last_exe_obs)&&is_exe_obs){
    if(is_run_code_cnt<=3&&(is_find_line||timer-have_find_line_timer<3)){//无障碍物则巡线,找到线判断不能这样
      // if(fabs(target_x-closest_line_x)>std::max(300.0,(y_ratio>0.4&&y_ratio<0.70?1.65:1.1)*max_rect_x))
      if(fabs(target_x-line_x)>std::max(300.0,max_rect_x)){
        is_exe_obs=0;
        RCLCPP_ERROR(rclcpp::get_logger("obs_change3"), "%f", closest_line_x);
      }
      else{
        if(is_left) infer_again=!last_exe_obs&&is_exe_obs&&(target_x-closest_line_x)*is_left>100;
        RCLCPP_ERROR(rclcpp::get_logger("obs_change-no3"), "%f,%d", closest_line_x,infer_again);
      }
    } 
    else if(is_run_code_cnt<=3){//无障碍物则丢线
      if((target_x -REAL_MIDX)*is_left>std::min(250.0,1.3*max_rect_x)&&y_ratio<0.3) {
        is_exe_obs=0;
        RCLCPP_ERROR(rclcpp::get_logger("obs_change"), "%f", (target_x -REAL_MIDX));
      }
      else if(timer<70&&!last_exe_obs&&is_left==-1&&fabs(target_x -REAL_MIDX)>std::min(350.0-10*loseline_right_cnt,1.3*max_rect_x)){
         is_exe_obs=0;
        RCLCPP_ERROR(rclcpp::get_logger("obs_change_qian"), "%f", std::min(350.0-10*loseline_right_cnt,1.3*max_rect_x));
      }
      else{
        if(is_left&& ((last_y_ratio-y_ratio)>0.2||last_y_ratio<OBS_JUDGE)) infer_again=!last_exe_obs&&is_exe_obs;//一个需要验证的改动 
      }
    }
    else if(is_run_code_cnt>3){
      if (fabs(target_x -code_x)>max_rect_x){
      is_exe_obs=0;
      RCLCPP_ERROR(rclcpp::get_logger("obs_change2"), "%f", (target_x -code_x));
      }
      else{
        if(is_left) infer_again=!last_exe_obs&&is_exe_obs&&((target_x -code_x)*is_left>-50);
        RCLCPP_ERROR(rclcpp::get_logger("obs_change2-NO"), "code_x%f", code_x);
      }
    }
  }
  if(last_exe_obs&&y_ratio>last_y_ratio-0.05&&(y_ratio>0.3||timer<100))is_exe_obs=true;//后半段右上角的桶容易误避障
  //if(is_left) infer_again=!last_exe_obs&&is_exe_obs&&(!is_find_line||(is_find_line&&fabs(target_x-line_x)>400));
  if ((is_left==-1||(is_left==1&&timer>70) )&& !is_exe_obs )
  { // 车避完障，且没有回线
    //如果障碍物在线上才有可能进行第2次避障，否则就回线就可以了。并且右转太久后也不要走m
    if(is_find_line&&(is_left!=-1||loseline_right_cnt<15)&&fabs(target_x-closest_line_x)<max_rect_x+100&&(target_x-closest_line_x)*is_left<max_rect_x){    
      is_exe_obs = (y_ratio >= OBS_JUDGE2 && (target_x -timer>120?closest_line_x:REAL_MIDX)*is_left<50);
      RCLCPP_INFO(rclcpp::get_logger("infer-m"), "%d %d %f,%f", is_exe_obs, is_left, target_x,closest_line_x);
      //fabs(target_x + is_left * REAL_MIDX-REAL_MIDX)<500);//|| y_ratio > 0.5; //target_x + is_left * REAL_MIDX > 300 && target_x + is_left * REAL_MIDX < 1600) || y_ratio > 0.5;
    }
    if (is_exe_obs){//不回线直接开始下一次避障 
       straight_cnt=0;
      bizhang_pid.reset_straight();
      is_straight = false;
      //防止前半段撞墙
      //if(y_ratio<0.30&&bias_bizhang>100&&is_find_line&&is_left==1&&target_x-line_x>-20) is_left=-1;
    }
    else 
    {
      if((last_y_ratio<0.6||(last_y_ratio<0.85&&straight_cnt<4))){
        if((is_find_line&&(last_target_x-line_x)*is_left>-50))
          {is_straight = true;
          RCLCPP_ERROR(rclcpp::get_logger("straight"), "%f", (last_target_x-line_x));}
        // is_left=0;//这里置0，丢线就不知道往哪边转
        else if(is_run_code_cnt>3&&(last_target_x-code_x)*is_left>-50){
          is_straight = true;
        RCLCPP_ERROR(rclcpp::get_logger("straight2"), "%f", (last_target_x-code_x));
        }
        else if(!is_find_line&&(last_target_x-REAL_MIDX)*is_left<0&&fabs(last_target_x-REAL_MIDX)<300)//
        {
          is_straight = true;
          RCLCPP_ERROR(rclcpp::get_logger("straight3"), "%f", (last_target_x-REAL_MIDX));
        }
        else{
          RCLCPP_ERROR(rclcpp::get_logger("straight-no"), "%f", last_target_x);
        }
      }
      else {
        is_straight=false;
        straight_cnt=0;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("infer"), "车避完障，且没有回线%d %d %f,%f", is_exe_obs, is_left, target_x,line_x);
  }
  // if(is_left==0 || is_need_infer==false){
  //     is_exe_obs = (y_ratio>=OBS_JUDGE && fabs(target_x-REAL_MIDX)<=500)||y_ratio>=0.70;
  //     if(is_exe_obs)RCLCPP_INFO(rclcpp::get_logger("infer"), "巡线时或避障时遇到障碍物%f",y_ratio);
  // }
  
  if( _1r_lose_anguz>1&&timer>45&&!is_exe_obs&&is_left==0) _1r_lose_anguz=0.9;
  if(bias_bizhang>150&&timer>45){
    bias_bizhang=130;//更往右避障
    
    RCLCPP_ERROR(rclcpp::get_logger("change0"),"change bias%d",timer);
  }
  else if( bias_bizhang>100&&((is_find_code&&code_y>70)||timer>70)){
    bias_bizhang=20;//更往右避障
    OBS_JUDGE = 0.25;
     RCLCPP_ERROR(rclcpp::get_logger("change"),"change bias%d",timer);
  }
  if (infer_again||(is_need_infer && is_exe_obs && is_left == 0))
  {
    bizhang_pid.reset();
    if (fabs(code_x - target_x) >200&&is_run_code_cnt>3)//((is_find_code&&code_y<90)||is_run_code))
    {
      if (target_x > code_x-0.5*code_y)
        is_left = -1;
      else
        is_left = 1;
      RCLCPP_ERROR(rclcpp::get_logger("is_left"), "target_x:%f,code_x%f", target_x, code_x);
    }
    //else if (timer>50&&((is_find_code&&code_y<90)||!is_find_code)&&is_find_line &&fabs(REAL_MIDX - target_x) <300&&(line_x>target_x||fabs(line_x - target_x)>70 ))//&&(fabs(line_x - REAL_MIDX) > 300 || fabs(line_x - target_x) > 1.5*max_rect_x))
    else if((is_find_line||timer-have_find_line_timer<3)&&timer>10&&(loseline_right_cnt>9||fabs(closest_line_y-hat_bottom_y)<50)&&fabs(target_x-closest_line_x)>30+0.4*max_rect_x&&fabs(target_x - REAL_MIDX) < 550)//fabs(closest_line_x - REAL_MIDX) < 600&&
    { // 当车离线远的时候往线那边避障，且障碍物离车不太远 
      if(timer==1&&target_x>closest_line_x-300)//开头特判
        is_left=-1;
      else if (target_x > closest_line_x)
        is_left = -1;
      else
        is_left = 1;
      RCLCPP_ERROR(rclcpp::get_logger("is_left"), "hat_bottom_y:%f,closest_line_x%f", hat_bottom_y, closest_line_x);
    }
    else if(timer<=10&&is_find_line&&fabs(closest_line_y-hat_bottom_y)<50&&fabs(closest_line_x - REAL_MIDX) < 400&&(target_x-closest_line_x)<-0.6*max_rect_x){
      if(target_x>closest_line_x-300)//开头特判
        is_left=-1;
      else is_left = 1;
      RCLCPP_ERROR(rclcpp::get_logger("is_left_line"), "target_x:%f,closest_line_x%f", target_x, closest_line_x);
    }
    else
    {
      if(timer==1&&is_find_line&&target_x>closest_line_x-280)//开头特判
        is_left=-1;
      else if(timer==1&&target_x > REAL_MIDX - 260)
         is_left=-1;
      else if (target_x > REAL_MIDX - bias_bizhang)
        is_left = -1;
      else
        is_left = 1;
      RCLCPP_ERROR(rclcpp::get_logger("is_left_mid"), "closest_line_x:%f,liney%f,hat_bottom_y%f", closest_line_x,closest_line_y,hat_bottom_y);
    }
    loseline_is_left=is_left;
  } // 结束状态变量的改变
   if(is_start){
    if(!is_exe_obs) {is_start=false;}
    else if(y_ratio<0.35||(last_y_ratio>0.05&&y_ratio-last_y_ratio>0.06*y_ratio)){ 
      is_start=false;
      //RCLCPP_ERROR(rclcpp::get_logger("start_change"),"1 ");
    }
    else if(is_find_code&&code_y>45) {is_start = false;
    //RCLCPP_ERROR(rclcpp::get_logger("start_change"),"2 ");
    }
    else if(timer>10){is_start=false;}
  }
  RCLCPP_INFO(rclcpp::get_logger("y_ratio"), "%f,last:%f,%d,%f,len%f", y_ratio,last_y_ratio,is_left,target_x,max_rect_x);
 if(bizhang_end) last_target_x=-1;
 //判断是否需要减速以扫码
 if(timer>35&&is_run_code&&code_y>60&&(code_max_x-code_min_x)>40&&!is_exe_obs&&code_min_x>5&&code_max_x<(IMAGE_WIDTH-5)){
  is_scan_code_cnt++;
  bool is_all_slow=false;
  if(code_y>80){
    is_all_slow=true;
    //判断以后还需要避障么,只有以后不用避障了才减速（不然一会加速一会减速的）
    for(auto &rect:results_hat){
      //double mid=1.0*(rect->xmax+rect->xmin)/2;
      //float len=(rect->xmax-rect->xmin);
      if(code_x<REAL_MIDX||(is_find_line&&is_left==1&&line_x<REAL_MIDX)){//车在右边
        if(rect->xmin<REAL_MIDX&&rect->xmax>code_min_x)
          {
            is_all_slow=false;
            RCLCPP_ERROR(rclcpp::get_logger("scan1"), "%f,%f", rect->xmin, rect->xmax);
            break;
          }
      }
      else{
        if(rect->xmax>REAL_MIDX&&rect->xmin<code_max_x)
          {
            is_all_slow=false;
            RCLCPP_ERROR(rclcpp::get_logger("scan2"), "%f,%f", rect->xmin, rect->xmax);
            break;
          }
      }
    }
  }
  if(is_all_slow) STD_LINEAR_X=0.25;
  //else STD_LINEAR_X=0.9
  
  RCLCPP_ERROR(rclcpp::get_logger("scan"), "%dy%flen%f", is_scan_code_cnt, code_y,code_max_x-code_min_x);
 }
 else{
  is_scan_code_cnt=0;
 }
 if(is_scan_code_cnt>2) {
    is_scan_code=true;
    STD_LINEAR_X=0.55;
    //else STD_LINEAR_X=0.9  
  }
  else is_scan_code=false; 
  last_over_task=is_over_task;
  //判断是否停下
  if ( is_find_code&&!is_exe_obs)
  {
    
    // is_run_code = true;
    //RCLCPP_ERROR(rclcpp::get_logger("code"), "%dy%f", is_run_code, code_y);
    //if (code_y < 40)
      //is_run_code = false;
    if (((code_min_x>30&&code_max_x<(IMAGE_WIDTH-30))||(-code_min_x+code_max_x>0.6*code_y))&&(is_run_code||code_y > 185)&&((code_y > (stop_code_y-10)&&fabs(code_x-REAL_MIDX)<2*code_y)||code_y > 150)&&(-code_min_x+code_max_x>0.4*code_y))
    
    {
      is_over_task_cnt++;
      is_find_stop=true;
      //cmd_vel_message.angular.z = 0;
      // cmd_vel_message.linear.x = 0;
      // cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
      // cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
      // cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度      
    }
    else{
      is_over_task_cnt=0;
    }
    RCLCPP_ERROR(rclcpp::get_logger("code"), "码%d,%f,%f",is_over_task_cnt,code_min_x,code_max_x);
  }
  else if(!is_get_code){
    is_over_task_cnt=0;
    
  }
  float code_s=is_find_code?code_y*(-code_min_x+code_max_x):0.0;
  if(is_over_task_cnt>4||(is_over_task_cnt&&code_y > 195)||(timer>80&&code_y >400&&code_min_x>1&&code_max_x<(IMAGE_WIDTH-10))||(timer>80&&code_s>100000)) is_over_task=true;
  else is_over_task = false;
  
  /* ====== !!!车的运动!!! ======  */
  if(is_over_task||is_get_code){    
    cmd_vel_message.angular.z = 0.0;
    cmd_vel_message.linear.x = last_over_task?-0.01:-STD_LINEAR_X-0.2;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("stop"), "停车%f",cmd_vel_message.linear.x);
    STD_LINEAR_X=0.25;
    cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
    if(is_get_code){
      cmd_vel_message.linear.x=0.0;
      cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
      stop_subscription(0);//关闭这个函数.
    }
  }
  else if (is_exe_obs&&((start_huixian<2)||start_huixian>11))
  {         //加一个开头error过大则倒退    
    have_find_line_timer = 0;//避完一个障后，只要看到线就记录看到线的timer
    
    straight_cnt=0;                                                 // 进入避障状态
    bizhang_x = target_x + bizhang_ratio * is_left * max_rect_x; // 障碍物在左边往右避障
    is_need_infer = false;                                       // 不用再判断往左往右转了
    if(!last_exe_obs&&timer>50) start_huixian=100;//2次避障后不用在回线特判
    double error = bizhang_x - REAL_MIDX;
    if(error>-400&&is_start) is_start=false;
    if(timer==1&&is_left==-1&&error<-700){
      start_huixian=0;
    }
    if(timer==1&&is_left==-1&&error<-760){
      daotui=true;
    }
    else daotui=false;
    // 有问题：
    // if((y_ratio<0.7&&timer>80)){//后半段德时候
    //   if(is_left==-1&&error>0) loseline_is_left=1;
    //   else loseline_is_left=is_left;
    // }
    // else if(timer<80) loseline_is_left=is_left;
    double vz = bizhang_pid.update(error)*(1+is_start*0.15);
    if(start_huixian==0&&y_ratio<0.7)vz=1.3*vz;
    if(STD_LINEAR_X<0.5) vz=0.7*vz; 
    if(loseline_right_cnt>5&&is_left==-1) vz-=std::max(1.5,(loseline_right_cnt-5)*0.2);
    loseline_right_cnt=0;
    if(vz*is_left<0) {
      if(y_ratio<0.32)vz=0.8*vz;
      else if(y_ratio<0.5) vz=0.0;
      else if(STD_LINEAR_X<0.6) vz=0.0;
    }
    if (vz > 0&&timer<90&&y_ratio>0.35) //
      vz = vz*0.7; // 右转转得多，处理一下,0.5在转弯处避不过去，0改为0.6
    else if(vz < 0&&y_ratio<0.4&&timer<90) vz=1.1*vz;
    if(vz > 0) vz=std::min(1.5,vz);
    //loseline_right_cnt比较大时，角速度大一点
    if(start_huixian==0&&error<-760&&daotui){//倒退
      timer--;//不影响timer判断
      cmd_vel_message.angular.z = -vz;
      cmd_vel_message.linear.x = -0.5;
      cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
      cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
      RCLCPP_WARN(rclcpp::get_logger("bizhang"), "倒退is_left:%d,bizhang:%f, max_rect_x:%f,angular.z: %f", is_left, bizhang_x, max_rect_x, cmd_vel_message.angular.z);
      return 0;
    }
    cmd_vel_message.angular.z = -vz;
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("bizhang"), "is_left:%d,bizhang:%f, max_rect_x:%f,angular.z: %f", is_left, bizhang_x, max_rect_x, cmd_vel_message.angular.z);
    //if (!is_over_task)                              // 还在第一条线任务中
      cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
    // std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  else if(start_huixian<16&&(!is_find_line||line_x<(1800-start_huixian*70))){//线在右边就巡线
    is_need_infer=true;
    start_huixian++;
    cmd_vel_message.angular.z = (float)(-1.0) * 1.9;
    loseline_right_cnt++;
    RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "开头往右转%f,%f,cnt%d", hat_bottom_y, cmd_vel_message.angular.z,start_huixian);  
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    if (!is_over_task)
      cmd_vel_publisher_->publish(cmd_vel_message); //  发布速度
  }
  else if(is_scan_code){
    is_need_infer = true;
    // is_left = 0;
    double error = code_x - REAL_MIDX;
    double vz = to_code_pid.update(error);
    if (vz < 0)
      vz = vz * 0.8; // 左转转得多，处理一下
    cmd_vel_message.angular.z = -vz*0.5;
    cmd_vel_message.linear.x = 0.25; 
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;

    if (!is_over_task)
    {
      cmd_vel_publisher_->publish(cmd_vel_message);
      RCLCPP_WARN(rclcpp::get_logger("scan_code"), "linear.x: %f, angular.z: %f", cmd_vel_message.linear.x, cmd_vel_message.angular.z);
    }
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
  else if (is_run_code_cnt<=3 && is_straight && is_left != 0 && (!is_need_infer || bizhang_end)&&bizhang_pid.continue_cnt()>0)
  { // 障碍物刚避完,直走
     straight_cnt++;
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
  else if (is_run_code_cnt>3)
  {
    is_need_infer = true;
    //is_left = 0;
    double error = code_x - REAL_MIDX;
    double vz = to_code_pid.update(error);
    if (vz > 0)
      vz = vz * 0.8; // 右转转得多，处理一下
    cmd_vel_message.angular.z = -vz;
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;

    
    if (!is_over_task){
      cmd_vel_publisher_->publish(cmd_vel_message);
      //RCLCPP_WARN(rclcpp::get_logger("to_code"), "linear.x: %f, angular.z: %f", cmd_vel_message.linear.x, cmd_vel_message.angular.z);
    }
  }
  else if (is_find_line)
  { // 巡线
    have_find_line_timer=timer;
    // 7. 巡线运动控制
    if (-1*is_left*(line_x - REAL_MIDX) < 50||(timer<50&&is_left==1&&fabs(line_x - REAL_MIDX)<250))//前半段往右转后早点判回线 
      {is_left = 0;
        loseline_right_cnt=0;}
    //else if(is_left==1&&bias_bizhang>50) is_left=0;//前半段往左避障后
    //RCLCPP_WARN(rclcpp::get_logger("line_node"), "post coor x: %f    y:%f", line_x, line_y);
    // if (is_slowdown)  // slowdown状态减速
    // now_linear_x = SLOW_LINEAR_X;
    // else {
    // if(!is_see_obs) // 前方看不到障碍物，就加速
    //   now_linear_x = ACC_LINEAR_X;
    // else
    start_huixian=100;
    is_need_infer = true;
    //}
    double v=(line_x - REAL_MIDX) * xunxian_k; 
    if(v>1.2) loseline_right_cnt++;
    else if(v<0.4)loseline_right_cnt=0;
    if(v>0&&timer>70) v=std::min(1.7,v);//右转不要太大
    else if(v<0&&timer>80) v=1.2*v;//左转大一点 
    
    if(is_left==0&&v<-2.0) loseline_is_left=1;//左转很大后改变丢线逻辑
    if(loseline_right_cnt>18&&v>0) v=std::min(0.4*v,1.0);//防止右转太多，车横过来
    cmd_vel_message.angular.z = -1.0 * v;
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("line_node"), "巡线line_x: %f, angular.z: %f", line_x, cmd_vel_message.angular.z);
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
    if((loseline_is_left!=is_left&&loseline_is_left==1)||loseline_right_cnt>50){
      if(line_x<200)cmd_vel_message.angular.z = (float)(line_x - REAL_MIDX) * xunxian_k;
      cmd_vel_message.angular.z = 2.5;//(float)(0.7) * _1l_lose_anguz;

      RCLCPP_ERROR(rclcpp::get_logger("lose_line2"), "往左大转%f,%f", line_x, cmd_vel_message.angular.z);
    }
    else if (is_left == 1)
    { // 障碍物在左侧，需要右转，转较多
      cmd_vel_message.angular.z = (float)(timer>105?1.4:1.0) * _1l_lose_anguz;
      RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "往左转%f,%f", hat_bottom_y, cmd_vel_message.angular.z);
    }
    else if(is_left == -1)
    { // 障碍物在右侧，需要左转，转较少
      loseline_right_cnt++;
      if(timer-have_find_line_timer<3)cmd_vel_message.angular.z=(-1.0) * (line_x - REAL_MIDX) * xunxian_k;//上一帧找到线按线来
      else cmd_vel_message.angular.z = (float)(-1.0) * _1r_lose_anguz;
      RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "往右转%f,%f,cnt%d", line_x, cmd_vel_message.angular.z,loseline_right_cnt);
    }
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    if (!is_over_task)
      cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
  }
  RCLCPP_INFO(rclcpp::get_logger("bool"), "obs%d,code%d,line%d,over%d,timer%d", is_exe_obs, is_find_code, is_find_line,is_over_task,timer);
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
