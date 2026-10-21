#include <iostream>
#include "fast_second/fast_second.h"
#include <chrono> //获取时间
#include <thread> //睡眠进程
#include <cstdlib>
 //#define DEBUG 1
/* 一些固定的参数 */
/* 固定的参数常量 */
const float REAL_MIDX = 960.0; // 将905视为中点，因为摄像头像素点偏差
// const float ACC_LINEAR_X = 1.20;
const float STD_LINEAR_X = 0.90;
// const float SLOW_LINEAR_X = 0.50;//丢线的时候以这个速度，这样加减速需要时间会影响后面的避障
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
        RCLCPP_WARN(rclcpp::get_logger("PID"), "error：%f,integral：%f，derivative：%f,v%f", error, integral_, derivative, output);
        // 保存当前误差和时间，为下一次调用做准备
        prev_error_ = error;
        last_time_ = now;

        return output;
    }
    int continue_cnt()
    { // 判断是否需要当看不到障碍物了，以上一次的误差在转一下
        cnt++;
        RCLCPP_WARN(rclcpp::get_logger("PID"), "pre_error：%f,cnt:%d", prev_error_, cnt);
        if (cnt > 9)
            return 0;
        if (fabs(prev_error_) > straight_min)
        {
            double interval = fabs(prev_error_) > 200 ? straight_interval2 : straight_interval1;
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
        if (cnt < 3)
            return Kp_ * prev_error_;
        if (fabs(prev_error_) > 50)
            return Kp_ * prev_error_;
        else
            return 0.0;
    }
    int reset()
    {
        prev_error_ = 0.0;
        integral_ = 0.0;
        cnt = 0;
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
    const int straight_min = 0, straight_interval1 = 3, straight_interval2 = 105;
    double prev_error_, integral_;
    int cnt;
    std::chrono::high_resolution_clock::time_point last_time_;
};
PIDController to_p_pid(0.003, 0.00007, 0.00007); // 需调参
PIDController bizhang_pid(0.002, 0.00007, 0.00007);
int timer = 0; // 计数器
/* 状态控制参数 */
bool is_start = true;      // 是否是开始状态
bool is_need_infer = true; // 是否需要策略推理
bool is_exe_obs = false;   // 是否有需要避的障
float bias_bizhang = 80.0; // 判断往左往右转时让其更偏向于向右转
int is_left = 0;           // 障碍物是否在左侧;1:在左侧；-1：在右侧，0：无需避障且已回线
bool is_find_stop = false;
bool is_straight = false; // 是否需要避完障直走阶段
int straight_cnt = 0;
bool loseline_is_left = false;//回线没回过来
// bool is_acc_to_p =false; //加速冲码
/* 避障相关参数 */
const float _1l_lose_anguz = 2.20; // 丢线的角速度往左转 1.9改大
// //float  _1l_end_anguz = 1.5;
const float _1r_lose_anguz = 1.45; // 往右转
// //float  _1r_end_anguz = 3.0;
float OBS_JUDGE = 0.23;         // 避障判断阈值,普通状态,以y占画面高的比值
const float OBS_JUDGE2 = 0.185; // 避障判断阈值,未回线状态
double y_ratio = 0.0;           // 障碍物所占图片的比率
double last_y_ratio=y_ratio;
// 避障2，朝障碍物旁边冲
double bizhang_x = REAL_MIDX;
const double bizhang_ratio = 1.6;
// const int BIZHANG_STRAIGHT = 130; // 避障完后直走阶段的时间，单位ms//试着改小一点
float target_x = REAL_MIDX; // 障碍物中点
float last_target_x = 0;
double max_rect_x = 0.0;
/* 巡线相关参数 */
float last_line_z = 0.0;  // 记录上一次回线大角度
float line_x = REAL_MIDX; // 当前线的x坐标
float closest_line_x = 0; // 距离障碍物最近的line的横坐标

const double xunxian_k = 0.0021; // 巡线的参数

/* 二维码相关参数 */
// const float qrcode_score_threshold_ = 0.70; // 二维码置信度阈值，移到yolo识别的地方了
double p_s = 0.0; // 二维码y长度
double p_x = 0;   // p x坐标
double p_min_x = 0;
double p_max_x = IMAGE_WIDTH;
double stop_p_s = 2500;
// const double CODE_AREA_RATIO=0.01;

// 速度控制的消息变量
auto cmd_vel_message = geometry_msgs::msg::Twist(); // 原来在这。。。。。。。。。。。。。。。。。。。。。。。
#ifdef DEBUG
// 构造函数的实现
OrigincarControl::OrigincarControl(const std::string &node_name,
                                   const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
    // 提示用户按回车键继续
    std::cout << "请按下任意键开始执行!!!" << std::endl;
    std::cin.get();

    // 创建控制消息发布者
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 创建订阅者，订阅话题 '/win_detect'（ai消息）
    ai_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "win_detection", 10, std::bind(&OrigincarControl::yolo_callback, this, std::placeholders::_1));
}

#else
// 构造函数的实现
OrigincarControl::OrigincarControl(const std::string &node_name,
                                   const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
    // 第二条线的构造函数只用收上位机的消息就行了，剩下的交给上位机消息的回调函数
    // 优化QoS设置
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .best_effort()
                   .durability_volatile();

    // 等待上位机发布"C区出口结束遥测"或者"为大模型节点拍照"
    subscriber_return_ = this->create_subscription<std_msgs::msg::Int32>(
        "/sign4return", qos,
        std::bind(&OrigincarControl::subscription_callback_return, this, std::placeholders::_1));

    // 输出提示
    const bool direct_start = this->declare_parameter<bool>("direct_start", false);
    if (direct_start) {
        RCLCPP_WARN(rclcpp::get_logger("second_control"),
                    "Second line direct_start=true, start immediately for local testing");
        start_second_line();
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("second_control"), "Second line is ready now!");
}

void OrigincarControl::start_second_line()
{
    if (ai_subscription_ && cmd_vel_publisher_) {
        RCLCPP_INFO(rclcpp::get_logger("second_control"), "Second line already started");
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    ai_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "win_detection", 10, std::bind(&OrigincarControl::yolo_callback, this, std::placeholders::_1));

    if (subscriber_return_) {
        subscriber_return_.reset();
        RCLCPP_INFO(rclcpp::get_logger("second_control"), "Subscription topic sign4return stopped");
    }

    RCLCPP_WARN(rclcpp::get_logger("second_control"), "Second line start");
}

// 上位机消息的回调函数
void OrigincarControl::subscription_callback_return(const std_msgs::msg::Int32::SharedPtr msg)
{
    // 第二条线和第一条线不一样
    // 如果按下按钮"C区出口结束遥测"，会被调用，启动订阅者hbmem，关闭订阅者sign4return
    if (msg->data == 6)
    {
        start_second_line();
        return;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        //std::system("pkill -f rosbridge_websocket"); // 杀死上位机节点

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .best_effort()
                       .durability_volatile();

        // 速度消息的发布者
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 创建订阅者，订阅话题 '/win_detect'（ai消息）
        ai_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "win_detection", 10, std::bind(&OrigincarControl::yolo_callback, this, std::placeholders::_1));

        // 停止订阅上位机的消息
        subscriber_return_.reset();
        RCLCPP_INFO(rclcpp::get_logger("origincar_control"), "Subscription topic sign4return stopped");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("origincar_control"), "Topic sign4return did not receive msg.data equals 6!");
    }
}
#endif
// 订阅ai消息回调函数的实现
int OrigincarControl::yolo_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
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
        else if (target.type == "P")
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
                    "P");
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
    bool is_find_p = false;
    bool is_run_p = false;
    for (auto &rect : results_p)
    {
        // RCLCPP_ERROR(rclcpp::get_logger("code"), "s%f", rect->score);
        // if (rect->score < qrcode_score_threshold_) // 二维码置信度小于阈值
        // continue;
        p_min_x = rect->xmin;
        p_max_x = rect->xmax; // 用于保证码扫全了
        p_s = (rect->ymax - rect->ymin) * (rect->xmax - rect->xmin);
        p_x = (rect->xmax + rect->xmin) / 2; // 往右冲一点码
        is_find_p = true;
        is_run_p = true;
        // 判断和障碍物的是否重叠，有重叠则不算
        for (auto &hat : results_hat)
        {
            float xx1 = std::max(rect->xmin, hat->xmin);
            // float yy1 = std::max(rect->ymin, hat->ymin);
            float xx2 = std::min(rect->xmax, hat->xmax);
            // float yy2 = std::min(rect->ymax, hat->ymax);

            if (xx2 > xx1)
            { // 有重叠
                is_run_p = false;
                RCLCPP_ERROR(rclcpp::get_logger("p"), "被障碍物遮挡");
                break; // 跳出循环
            }
        }
        // if(!is_acc_to_p&&(results_hat.size()==0)){
        //   is_acc_to_p=true;
        //   to_p_pid.reset();
        // }
        RCLCPP_ERROR(rclcpp::get_logger("p"), "%dy%f", is_run_p, p_s);
    }

    // 根据yolo结果改变判断
    float max_rect_y = 0.0, now_rect_y = 0.0, hat_bottom_y = 0.0; // 最大锥桶面积，最大锥桶的宽......，最大锥桶的底部纵坐标
    if (is_exe_obs)
        last_target_x = target_x;
    // hat_num = 0;//需要考虑的障碍物
    for (auto &rect : results_hat)
    {
        if (((!is_exe_obs||timer<70)&&rect->xmax < 200) || rect->xmin > 1650 || ((!is_exe_obs||timer<80)&&(rect->xmax + rect->xmin) / 2 < 100)) // 利用xmax和xmin限制锥桶范围，太偏左右的锥桶不考虑
            continue;
        if (is_left == -1 &&!is_run_p &&(!is_exe_obs||timer<70) &&(rect->xmax + rect->xmin) / 2 < 400)
            continue;
        if (is_left == 1 &&!is_run_p &&  (rect->xmax + rect->xmin) / 2 > 1500)
            continue;
        // if (is_first_obs && PostProcess_cnt > 0 && rect->xmin < 400 ) // 限制任务一最开始，不看任务二的桶
        //   continue;
        //++hat_num;
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

    float min_line_y = 2000.0;    // 离车最远的线
    float dis_hatline_y = 2000.0; // 离桶最近的线
    float line_k=0;
    float jin_line_y=0;
    bool last_find_line=is_find_line;
    // 第一条巡线过程中的巡线目标点的确定，取纵坐标y一定范围内的最近line的横坐标
    for (auto &rect : results_line)
    {
        // if(rect->ymin<10) continue;
         if(is_left==-1&&!last_find_line){
           if(rect->xmin<150) continue;//往左避障后，跳过第1条线
         }
        // 找离桶最近的线
        if (fabs(hat_bottom_y - rect->ymin) < fabs(dis_hatline_y))
        { // 找到距离锥桶最近的line的横坐标
            closest_line_x = (rect->xmin + rect->xmax) / 2;
            dis_hatline_y = hat_bottom_y - rect->ymin;
            line_k=(rect->xmax - rect->xmin)/(rect->ymax - rect->ymin);
            jin_line_y=(rect->ymax+rect->ymin)/2;
        }
        // 找最远的线
        if (rect->ymin < min_line_y) //&&rect->ymax > hat_bottom_y)
        {
            // 找离车最远且在障碍物前面的线
            line_x = (rect->xmin + rect->xmax) / 2;
            min_line_y = rect->ymin;
            is_find_line = true;
        }
        // line_y = (rect->ymin + rect->ymax) / 2;
    }
    //斜线调调closest_line_x
    if(timer<50&&is_left==0)closest_line_x+=(jin_line_y-hat_bottom_y)*line_k;
    // 判断是否需要避障、避障是否结束
    if(is_exe_obs) last_y_ratio = y_ratio;
    y_ratio = max_rect_y / IMAGE_HEIGHT; // 最大障碍物的比率
    // if(last_y_ratio>y_ratio+0.15){//
    //   bizhang_pid.reset();
    // }
    if(is_find_line) is_start = false;
    bool infer_again = false;
    bool bizhang_end = (last_y_ratio - y_ratio) > 0.1;
    bool last_exe_obs = is_exe_obs;
    // is_exe_obs = (y_ratio >= OBS_JUDGE && fabs(target_x - REAL_MIDX) <= (500>1.5*max_rect_x?1.5*max_rect_x:500)) || y_ratio >= 0.70;//判断是否需要避障

    is_exe_obs = y_ratio >= 0.50 || (last_exe_obs&&y_ratio>0.35)|| (y_ratio >= OBS_JUDGE && ((is_find_line && fabs(target_x - closest_line_x) < 600)||is_start)); // fabs(target_x -REAL_MIDX)<std::min(250.0,max_rect_x)
    
    if ((y_ratio < 0.40 || !last_exe_obs) && is_exe_obs)
    {
        if (is_find_line && !is_run_p &&!last_exe_obs&& (target_x - closest_line_x) * is_left > std::min(200.0, max_rect_x)&&y_ratio<0.80)//三星连珠时第2个障碍物会撞加一个y的限制
        {
            is_exe_obs = 0;
            RCLCPP_ERROR(rclcpp::get_logger("obs_change3"), "%f", closest_line_x);
        }
        else if(is_find_line && !is_run_p &&timer<30&&!is_start&&!last_exe_obs&&target_x>1250){//多测测~//target_x-max_rect_x>1100   1300
            is_exe_obs = y_ratio>0.4;
            RCLCPP_ERROR(rclcpp::get_logger("obs_change4"), "右弯道%f", target_x);
        }
        else if (!is_run_p &&!last_exe_obs&& ((target_x - REAL_MIDX) * is_left > std::max(200.0, 1.2 * max_rect_x) || fabs((target_x - REAL_MIDX) * is_left) > 600))
        {
            is_exe_obs = 0;
            RCLCPP_ERROR(rclcpp::get_logger("obs_change"), "%f", (target_x - REAL_MIDX));
        }
        else if (is_run_p && !last_exe_obs&&fabs(target_x - p_x) > 1.5*max_rect_x)
        {
            is_exe_obs = 0; 
            RCLCPP_ERROR(rclcpp::get_logger("obs_change2"), "%f", (target_x - p_x));
        }
    }
    if(!is_exe_obs&&!last_exe_obs&&timer>70&&y_ratio>0.25){
         is_exe_obs=(is_find_line && fabs(target_x - closest_line_x) < 800)||(last_exe_obs&&(target_x - REAL_MIDX)*is_left<0);//||fabs(target_x - REAL_MIDX)<500
         RCLCPP_ERROR(rclcpp::get_logger("obs_change5"), "左弯道%f,%d", closest_line_x,is_exe_obs);
    }
    if (is_find_line && last_line_z > 0.15 && is_left == 0)
    {
        is_exe_obs = y_ratio >= 0.19 && target_x < (REAL_MIDX - 20);
    }
    if (is_left&&(y_ratio<0.35||(last_y_ratio-y_ratio)>0.2))
        infer_again = !last_exe_obs && is_exe_obs;
    if (is_left && !is_exe_obs)
    { // 车避完障，且没有回线
        // 如果障碍物在线上
        //findp后说明timer很大，再走左边的m会不去
        if (is_find_line && (!is_find_p||(!is_run_p&&is_left==-1)) && fabs(target_x - closest_line_x) < std::min(max_rect_x, 350.0) && (target_x-closest_line_x)*is_left<std::min(max_rect_x, 200.0))//(target_x - closest_line_x) * is_left > -200
        {
            is_exe_obs = (y_ratio >= OBS_JUDGE2 && fabs(target_x - REAL_MIDX) < 350 && (target_x - REAL_MIDX) * is_left < -50);
            // fabs(target_x + is_left * REAL_MIDX-REAL_MIDX)<500);//|| y_ratio > 0.5; //target_x + is_left * REAL_MIDX > 300 && target_x + is_left * REAL_MIDX < 1600) || y_ratio > 0.5;
            
        }
        // else if(is_find_p&&)
        if (is_exe_obs)
        { // 不回线直接开始下一次避障
            straight_cnt = 0;
            bizhang_pid.reset_straight();
            is_straight = false;
        }
        else
        {
            if (straight_cnt<10&&last_y_ratio>0.28&&(last_y_ratio < 0.6 || (last_y_ratio < 0.85 && straight_cnt < 5)||straight_cnt < 3))
            {
                if (is_find_line && (last_target_x - line_x) * is_left > -500&&(line_x-REAL_MIDX)*is_left<0)
                {
                    is_straight = true;
                    RCLCPP_ERROR(rclcpp::get_logger("straight"), "%f", (last_target_x - line_x));
                }
                // is_left=0;//这里置0，丢线就不知道往哪边转
                else if (is_run_p && (last_target_x - p_x) * is_left > 50)
                {
                    is_straight = true;
                    if(timer>100) straight_cnt++;//少直走一会儿啊
                    RCLCPP_ERROR(rclcpp::get_logger("straight2"), "%f", (last_target_x - p_x));
                }
                else if (!is_find_line && (last_target_x - REAL_MIDX) * is_left < 0 && (fabs(last_target_x - REAL_MIDX)*std::min(last_y_ratio,0.8) < 250 )) //
                {
                    is_straight = true;
                    RCLCPP_ERROR(rclcpp::get_logger("straight3"), "%f", (last_target_x - REAL_MIDX));
                }
                else if(timer>85&&last_y_ratio<0.6&&last_target_x<REAL_MIDX){
                    is_straight = true;
                    straight_cnt+=2;//少直走一会
                    RCLCPP_ERROR(rclcpp::get_logger("straight4"), "最后特判%f", last_target_x);
                }
                else
                {
                    straight_cnt+=2;//不在直走
                    is_straight = false;
                    RCLCPP_ERROR(rclcpp::get_logger("straight-no"), "%f，%d", last_target_x,straight_cnt);
                }
            }
            else
            {
                is_straight = false;
                //straight_cnt = 0;下一个障碍物时才置0
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("infer"), "车避完障，且没有回线%d %d %f,%f,%f", is_exe_obs, is_left, target_x, closest_line_x,max_rect_x);
    }
    // if(is_left==0 || is_need_infer==false){
    //     is_exe_obs = (y_ratio>=OBS_JUDGE && fabs(target_x-REAL_MIDX)<=500)||y_ratio>=0.70;
    //     if(is_exe_obs)RCLCPP_INFO(rclcpp::get_logger("infer"), "巡线时或避障时遇到障碍物%f",y_ratio);
    // }
    if (bias_bizhang > 0 && is_find_p && timer > 45)
    {
        bias_bizhang = -80; // 更往左避障
        // OBS_JUDGE = 0.25;
        RCLCPP_ERROR(rclcpp::get_logger("change"), "change bias");
    }
    if (infer_again || (is_need_infer && is_exe_obs && is_left == 0))
    {
        bizhang_pid.reset();
        if (fabs(REAL_MIDX - target_x) < std::min(500.0, max_rect_x + 100) && is_find_p && p_s > 4200) //((is_find_code&&code_y<90)||is_run_code))
        {
            if (target_x > p_x - 50)
                is_left = -1;
            else
                is_left = 1;
            RCLCPP_ERROR(rclcpp::get_logger("is_left"), "target_x:%f,p_x%f", target_x, p_x);
        } // 加一个往左避障后开到第一条线后不再往左避障
        else if (is_find_line && fabs(target_x - REAL_MIDX) < 600) //(fabs(closest_line_x - REAL_MIDX) < 550)&&fabs(REAL_MIDX - target_x) <200 &&(fabs(line_x - REAL_MIDX) > 500 || fabs(line_x - target_x) > 500))
        {
            if (target_x > closest_line_x + (timer > 65 ? -75 : 75))
                is_left = -1;
            else
                is_left = 1;
            RCLCPP_ERROR(rclcpp::get_logger("is_left"), "target_x:%f,closest_line_x%f,%f", target_x, closest_line_x,(jin_line_y-hat_bottom_y)*line_k);
        }
        else
        {
            if (target_x > REAL_MIDX + bias_bizhang)
                is_left = -1;
            else
                is_left = 1;
            RCLCPP_ERROR(rclcpp::get_logger("is_left"), "target_x:%f", target_x);
        }
    } // 结束状态变量的改变 
    if (is_start)
    {
        if (!is_exe_obs&&y_ratio<0.2)
        {
            is_start = false;
        }
        else if (timer>30)
        {
            is_start = false;
            // RCLCPP_ERROR(rclcpp::get_logger("start_change"),"1 ");
        }
        else if (is_find_p)
        {
            is_start = false;
            // RCLCPP_ERROR(rclcpp::get_logger("start_change"),"2 ");
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("y_ratio"), "%f,last:%f,%d,target_x:%f", y_ratio, last_y_ratio, is_left, target_x);
    if (y_ratio > 0.4)
        last_target_x = target_x;

    /* ====== !!!车的运动!!! ======  */

    if (is_exe_obs)
    {
        straight_cnt = 0;
        double len = y_ratio < 0.4 ? std::max(1.2*max_rect_x, 400.0) : max_rect_x; // 进入避障状态
        bizhang_x = target_x + bizhang_ratio * is_left * len;                  // 障碍物在左边往右避障
        is_need_infer = false;                                                 // 不用再判断往左往右转了
        double error = bizhang_x - REAL_MIDX;
        double vz = bizhang_pid.update(error) * (1 + is_start * 0.15);
        if (vz * is_left < 0)
        {
            if (y_ratio < 0.40)
                vz = (0.6 - fabs(last_line_z) / 2) * vz;
            else
                vz = 0.0;
        }
        if (vz > 0)
           vz = std::max(vz * 0.8,0.5); // 右转转得多，处理一下,0.5在转弯处避不过去，0改为0.6
        if (is_left == 1 && vz < 0)
        {
            vz += 2 * last_line_z;
        }
        if (fabs(last_line_z) > 0.15 && vz * last_line_z > 0 && y_ratio < 0.3)
            vz = 1.5 * vz;
        cmd_vel_message.angular.z = -vz;
        cmd_vel_message.linear.x = STD_LINEAR_X;
        cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("bizhang"), "is_left:%d,bizhang:%f, len:%f,angular.z: %f,last_line_z:%f", is_left, bizhang_x, len, cmd_vel_message.angular.z, last_line_z);
        // if (!is_over_task)                              // 还在第一条线任务中
        cmd_vel_publisher_->publish(cmd_vel_message); // 发布速度
                                                      // std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // else if (is_acc_to_p){
    //   is_need_infer = true;
    //   // is_left = 0;
    //   double error = p_x - REAL_MIDX;
    //   double vz = to_p_pid.update(error);
    //   if (vz < 0)
    //     vz = vz * 0.8; // 左转转得多，处理一下
    //   cmd_vel_message.angular.z = -vz;
    //   cmd_vel_message.linear.x = ACC_LINEAR_X;
    //   cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    //   cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    //   cmd_vel_publisher_->publish(cmd_vel_message);

    // }
    else if (is_straight && is_left != 0 && (!is_need_infer || bizhang_end) && bizhang_pid.continue_cnt() > 0)
    { // 障碍物刚避完,直走
      // bizhang_x = REAL_MIDX;
      // int continue_cnt = bizhang_pid.continue_cnt();
      // if (continue_cnt)
      //{
        straight_cnt++;
        double vz = bizhang_pid.continue_v();
        if (vz > 0.5)
            vz = std::max(vz * 0.8,0.5); // 右转转得多，处理一下
        if (vz * is_left < 0)
            vz = 0.0;
        cmd_vel_message.angular.z = -vz;
        // cmd_vel_message.angular.z=0.0;
        cmd_vel_message.linear.x = STD_LINEAR_X;
        cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("bizhang"), "is_left:%d, angular.z: %f", is_left, cmd_vel_message.angular.z);
        cmd_vel_publisher_->publish(cmd_vel_message);
        // return 0;
        //}
        // bizhang_pid.reset();
        // is_need_infer = true;
        // is_straight = false;
    }
    else if (is_run_p)
    {
        is_need_infer = true;
        // is_left = 0;
        double error = p_x - REAL_MIDX;
        double vz = to_p_pid.update(error);
        if (vz > 0)
            vz = vz * 0.8; // 右转转得多，处理一下
        cmd_vel_message.angular.z = -vz;
        cmd_vel_message.linear.x = STD_LINEAR_X;
        cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_message);
    }
    else if (is_find_line)
    { // 巡线
        // 7. 巡线运动控制
        if (-1 * is_left * (line_x - REAL_MIDX) < 0)
            is_left = 0;
        is_need_infer = true;
        double v = (line_x - REAL_MIDX) * xunxian_k;
        if(is_left==0&&fabs(v)>-2.0) loseline_is_left=true;
        else loseline_is_left=false;
        cmd_vel_message.angular.z = -1.0 * v;
        cmd_vel_message.linear.x = STD_LINEAR_X;
        cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
        if (is_left == 0)
        {
            last_line_z = cmd_vel_message.angular.z;
        }
        else
            last_line_z = 0;
        RCLCPP_INFO(rclcpp::get_logger("line_node"), "巡线line_x: %f, angular.z: %f", line_x, cmd_vel_message.angular.z);
        cmd_vel_publisher_->publish(cmd_vel_message);
    }
    else
    {
        //  丢线
        is_need_infer = true;
        if (is_left == 0)
        {
            if(loseline_is_left) cmd_vel_message.angular.z = last_line_z;
            else cmd_vel_message.angular.z = 0.0;
            RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "直走%d,%f", timer, cmd_vel_message.angular.z);
        }
        else if (is_left == 1)
        { // 障碍物在左侧，需要右转，转较多
            cmd_vel_message.angular.z = (float)(1.0) * (timer>92?2.3:_1l_lose_anguz);
            RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "往左转%f,%f", hat_bottom_y, cmd_vel_message.angular.z);
        }
        else if (is_left == -1)
        { // 障碍物在右侧，需要左转，转较少
            cmd_vel_message.angular.z = (float)(-1.0) * _1r_lose_anguz;
            RCLCPP_ERROR(rclcpp::get_logger("lose_line"), "往右转%f,%f", hat_bottom_y, cmd_vel_message.angular.z);
        }
        cmd_vel_message.linear.x = STD_LINEAR_X;
        cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_message);
    }
    RCLCPP_INFO(rclcpp::get_logger("bool"), "obs%d,code%d,line%d,timer%d", is_exe_obs, is_find_p, is_find_line, timer);
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
