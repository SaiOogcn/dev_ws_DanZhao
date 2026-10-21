#include <iostream>
#include "only_second_line/only_second_line.h"
#include <chrono> //获取时间
#include<thread> //睡眠进程
#include <cstdlib> 
/* 一些固定的参数 */
const float  LINE_REAL_MIDX = 960.0; // 将905视为中点，因为摄像头像素点偏差
const float  STD_LINEAR_X   = 0.60;
const double IMAGE_WIDTH    = 1920.0;
const double IMAGE_HEIGHT   = 1080.0;
const double IMAGE_AREA     = 1920.0 * 1080.0;
const float LSA=1;
const float LEA=2;
const float RSA=2;
const float REA=1;
const float L1=15;
const float L2=20;
const float L3=20;
const float R1=15;
const float R2=20;
const float R3=20;
int temp_angularz_index = 0;
float  L_START_ANGUZ = LSA;
float  L_END_ANGUZ = LEA;
float  R_START_ANGUZ = RSA;
float  R_END_ANGUZ = REA;
int    L_FIRST_INDEX=L1;
int    L_SECOND_INDEX=L2;
int    L_THIRD_INDEX=L3;
int    R_FIRST_INDEX=R1;
int    R_SECOND_INDEX=R2;
int    R_THIRD_INDEX=R3;
int    obs_num=1;
int    timer=0;
bool passed_obs=false;
/* 设置全局变量用于状态控制 */
bool is_exe_obs     = false;
bool is_left        = true;
/* 设置全局变量用于巡线和避障 */
float  line_x = LINE_REAL_MIDX, line_y = 0;
float  target_x = LINE_REAL_MIDX;                   // 障碍物中点
float  closest_line_x = LINE_REAL_MIDX;             // 距离障碍物最近的line的横坐标
bool   last_left=true;
float p_x=LINE_REAL_MIDX;
int p_cnt=0;
int line_cnt=0;
bool rush_p=false;
/* 用于新避障逻辑控制的参数 */ 
double area_ratio = 0;                         // 障碍物的面积
auto   cmd_vel_message = geometry_msgs::msg::Twist();
float revision=0;
//构造函数的实现
OrigincarControl::OrigincarControl(const std::string& node_name,
                                   const rclcpp::NodeOptions& options)
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



//订阅ai消息回调函数的实现
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

    for (auto& target : msg->targets) {
      if(target.type == "hat"){
        for (auto& roi : target.rois){
          auto temp_yolo_res = std::make_shared<YoloV8Result>
          (
            0,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "hat"
          );
          results_hat.push_back(temp_yolo_res);
        }
      } else if(target.type == "line"){
        for (auto& roi : target.rois){
          auto temp_yolo_res = std::make_shared<YoloV8Result>
          (
            1,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "line"
          );
          results_line.push_back(temp_yolo_res);
        }
      } else if(target.type == "P"){
        for (auto& roi : target.rois) { 
          auto temp_yolo_res = std::make_shared<YoloV8Result>
          (
            2,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "P"
          );
          results_p.push_back(temp_yolo_res);
        }
      } else if(target.type == "code"){
        for (auto& roi : target.rois) {
          auto temp_yolo_res = std::make_shared<YoloV8Result>
          (
            3,
            roi.rect.x_offset,
            roi.rect.y_offset,
            roi.rect.x_offset + roi.rect.width,
            roi.rect.y_offset + roi.rect.height,
            roi.confidence,
            "code"
          );
          results_code.push_back(temp_yolo_res);
        }
      }
    }
    if(is_exe_obs)
    { // 进入避障状态
        cmd_vel_message.linear.x  = STD_LINEAR_X;
        if(is_left)
        {
          if(temp_angularz_index==0)
          {
            RCLCPP_ERROR(rclcpp::get_logger("line_node"), "第%d个障碍物，向右避障", obs_num);
            RCLCPP_ERROR(rclcpp::get_logger("line_node"), "L_START_ANGUZ:%f,L_FIRST_INDEX:%d,L_SECOND_INDEX:%d,L_THIRD_INDEX:%d", L_START_ANGUZ,L_FIRST_INDEX,L_SECOND_INDEX,L_THIRD_INDEX);
          }
          RCLCPP_ERROR(rclcpp::get_logger("line_node"), "temp_angularz_index:%d", temp_angularz_index);
          //避障前半部分
        if(temp_angularz_index<L_FIRST_INDEX)
        {
          temp_angularz_index += 1;
          cmd_vel_message.angular.z = (float)(-1.0) * L_START_ANGUZ;
        }
        //避障后半部分
        else if(temp_angularz_index >= L_FIRST_INDEX&&temp_angularz_index<L_SECOND_INDEX) {
          temp_angularz_index += 1;
          cmd_vel_message.angular.z=0;
        }
        else if(temp_angularz_index>=L_SECOND_INDEX&&temp_angularz_index<L_THIRD_INDEX)
        {
          temp_angularz_index += 1;
          cmd_vel_message.angular.z = (float)(1.0) * L_END_ANGUZ;
        }
        else if(temp_angularz_index>=L_THIRD_INDEX)
        {
          revision=30;
          last_left=true;   
          is_exe_obs = false;
          temp_angularz_index = 0;
          passed_obs=true;
          obs_num++;
          L_FIRST_INDEX=L1;
          L_SECOND_INDEX=L2;
          L_THIRD_INDEX=L3;
          R_FIRST_INDEX=R1;
          R_SECOND_INDEX=R2;
          R_THIRD_INDEX=R3;
          L_START_ANGUZ = LSA;
          L_END_ANGUZ = LEA;
          R_START_ANGUZ = RSA;
          R_END_ANGUZ = REA;
          cmd_vel_message.angular.z = 0.0;
        }
        }
        else
        {
          if(temp_angularz_index==0)
          {
            RCLCPP_ERROR(rclcpp::get_logger("line_node"), "第%d个障碍物，向左避障", obs_num);
            RCLCPP_ERROR(rclcpp::get_logger("line_node"), "R_START_ANGUZ:%f,R_FIRST_INDEX:%d,R_SECOND_INDEX:%d,R_THIRD_INDEX:%d", R_START_ANGUZ,R_FIRST_INDEX,R_SECOND_INDEX,R_THIRD_INDEX);
          }
          RCLCPP_ERROR(rclcpp::get_logger("line_node"), "temp_angularz_index:%d", temp_angularz_index);
          //避障前半部分
        if(temp_angularz_index<R_FIRST_INDEX)
        {
          temp_angularz_index += 1;
          cmd_vel_message.angular.z = (float)( 1.0) * R_START_ANGUZ;
        }
        //避障后半部分
        else if(temp_angularz_index >= R_FIRST_INDEX&&temp_angularz_index<R_SECOND_INDEX) {
          temp_angularz_index += 1;
          cmd_vel_message.angular.z=0;
        }
        else if(temp_angularz_index>=R_SECOND_INDEX&&temp_angularz_index<R_THIRD_INDEX)
        {
          temp_angularz_index += 1;
          cmd_vel_message.angular.z = (float)(-1.0) * R_END_ANGUZ;
        }
        else if(temp_angularz_index>=R_THIRD_INDEX)
        {
          revision=-30;
          last_left=false;   
          is_exe_obs = false;
          temp_angularz_index = 0;
          passed_obs=true;
          obs_num++;
          L_FIRST_INDEX=L1;
          L_SECOND_INDEX=L2;
          L_THIRD_INDEX=L3;
          R_FIRST_INDEX=R1;
          R_SECOND_INDEX=R2;
          R_THIRD_INDEX=R3;
          L_START_ANGUZ = LSA;
          L_END_ANGUZ = LEA;
          R_START_ANGUZ = RSA;
          R_END_ANGUZ = REA;
          cmd_vel_message.angular.z = 0.0;
        }
        }
        cmd_vel_message.linear.y  = 0.0, cmd_vel_message.linear.z  = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_message);//发布速度
        return 0;
    }
    if(results_line.size()==0&&passed_obs)
    {
      line_cnt=0;
      p_cnt=0;
      cmd_vel_message.linear.x=STD_LINEAR_X;
      if(last_left)
      {
        cmd_vel_message.angular.z=2.2;
        RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"丢线,往左边回线");
      }
      else
      {
        cmd_vel_message.angular.z=-1.1;
        RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"丢线,往右边回线");
      }
      cmd_vel_message.linear.y  = 0.0, cmd_vel_message.linear.z  = 0.0;
      cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_message);//发布速度
      return 0;
    }
    else
    {
      line_cnt++;
      if(line_cnt==5)
      {
        passed_obs=false;
      }
      std::vector<float> rect_area; // rect的面积
      float max_rect_area = 0.0, now_rect_area = 0.0,hat_bottom_y = 0.0,max_hat_height=0,now_hat_height=0; // 最大锥桶面积，......，最大锥桶的底部纵坐标
      for(auto& rect : results_hat) {
        if (rect->xmax < 150 || rect->xmin > 1800) // 利用xmax和xmin限制锥桶范围，太偏左右的锥桶不考虑
          continue;
        now_rect_area = (rect->xmax - rect->xmin) * (rect->ymax - rect->ymin);
        now_hat_height=rect->ymax - rect->ymin;
        if (max_rect_area < now_rect_area) {
          hat_bottom_y = rect->ymax;
          max_rect_area = now_rect_area;
          target_x = (rect->xmax + rect->xmin) / 2;
        }
        if(max_hat_height<now_hat_height)
        {
          max_hat_height=now_hat_height;
        }
      }
      area_ratio = max_rect_area / IMAGE_AREA;
      /* 巡线 */
      float dis_hatline_y = 2000.0;
      float min_line_y = 2000.0;
      //取纵坐标一定范围内的最远line的横坐标
      for (auto& rect : results_line) {
        line_y = (rect->ymax+rect->ymin)/2;
        if(fabs(hat_bottom_y - line_y)< dis_hatline_y) { // 找到距离锥桶最近的line的横坐标
          closest_line_x = (rect->xmin + rect->xmax) / 2;
          dis_hatline_y = fabs(hat_bottom_y - line_y);
        }
        if(rect->ymin < min_line_y && rect->ymax >= 600){
          line_x = (rect->xmin + rect->xmax) / 2;
          min_line_y = rect->ymin;
        }
      }
      RCLCPP_WARN(rclcpp::get_logger("playfootball_node"),"timer:%d,巡线中,line_x:%f,max_hat_height:%f,area_ratio:%f,target_x:%f,closest_line_x:%f",timer,line_x,max_hat_height,area_ratio,target_x,closest_line_x);
      if((area_ratio>=0.03&&timer>10&&closest_line_x>=target_x-550&&closest_line_x<=target_x+500&&target_x>300&&target_x<1700)||(area_ratio>=0.1&&timer<=10&&target_x<1700))
      {
        if(timer<100)
        {
          if(!last_left&&obs_num>1&&area_ratio>=0.05)
          {
            revision=-1*revision;
          }
          else if(last_left&&obs_num>1&&area_ratio>=0.05)
          {
            revision=-1*revision;
          }
        }
        else
        {
          if(area_ratio<0.04)
          {
            revision=-100;
          }
          else
          {
            revision=100;
          }
        }
        RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"timer:%d,前方障碍物,revision:%f,max_hat_height:%f,area_ratio:%f,target_x:%f,closest_line_x:%f",timer,revision,max_hat_height,area_ratio,target_x,closest_line_x);
        if(timer<=10)//车刚启动时的特判
        {
          is_exe_obs=true;
          if(target_x<LINE_REAL_MIDX)
          {
            is_left=true;
          }
          else
          {
            is_left=false;
          }
          RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"刚启动时避障");
          L_FIRST_INDEX=L1+5;
          L_SECOND_INDEX=L2+5;
          L_THIRD_INDEX=L3+5;
          if(target_x>1300)
          {
            R_FIRST_INDEX=R1+5;
            R_SECOND_INDEX=R2+5;
            R_THIRD_INDEX=R3+5;
          }
          else
          {
            R_FIRST_INDEX=R1+8;
            R_SECOND_INDEX=R2+8;
            R_THIRD_INDEX=R3+8;
          }
          L_START_ANGUZ = LSA+0.2;
          L_END_ANGUZ = LEA+0.5;
          R_START_ANGUZ = RSA+0.5;
          R_END_ANGUZ = REA+0.2;
          return 0;
        }
        else
        {
          if(target_x<closest_line_x+revision)
          {
            is_left=true;
          }
          else
          {
            is_left=false;
          }
          // if(!last_left&&area_ratio>=0.05&&obs_num>1)
          // {
          //   is_left=true;
          // }
          // if(last_left&&area_ratio>=0.05&&obs_num>1)
          // {
          //   is_left=false;
          // }
          if(timer>=100)
          {
            RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"左弯道上");
            // if(!last_left&&obs_num>1&&target_x>700&&area_ratio>=0.04)
            // {
            //   is_exe_obs=true;
            //   is_left=false;
            //   R_FIRST_INDEX=R1+3;
            //   R_SECOND_INDEX=R2+3;
            //   R_THIRD_INDEX=R3+3;
            //   return 0;
            // }
            // else
            // {
              if(!is_left)
              {
                if(area_ratio>=0.03&&target_x<1100&&target_x>600&&area_ratio<0.04)
                {
                  R_FIRST_INDEX=R1+5;
                  R_SECOND_INDEX=R2+5;
                  R_THIRD_INDEX=R3+5;
                  is_exe_obs=true;
                  return 0;
                }
              }
              else
              {
                if(area_ratio>=0.05)
                {
                  is_exe_obs=true;
                  L_FIRST_INDEX=L1-2;
                  L_SECOND_INDEX=L2-2;
                  L_THIRD_INDEX=L3-2;
                  return 0;
                }
              }
            // }
          }
          else
          {
            // if(target_x<500)
            // {
            //   is_left=true;
            // }
            // else if(target_x>1400)
            // {
            //   is_left=false;
            // }
            if(last_left&&is_left&&obs_num>1&&target_x>500&&area_ratio>=0.04)
            {
              is_exe_obs=true;
              RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"连续两次向右避");
              if(target_x<800)
              {
                L_FIRST_INDEX=L1;
                L_SECOND_INDEX=L2+3;
                L_THIRD_INDEX=L3+3;
              }
              else
              {
                L_FIRST_INDEX=L1+3;
                L_SECOND_INDEX=L2+3;
                L_THIRD_INDEX=L3+3;
              }
              return 0;
            }
            else if(!last_left&&!is_left&&obs_num>1&&target_x<1200)
            {
              is_exe_obs=true;
              RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"连续两次向左避");
              if(target_x<1000)
              {
                R_FIRST_INDEX=R1+5;
                R_SECOND_INDEX=R2+5;
                R_THIRD_INDEX=R3+5;
              }
              else
              {
                R_FIRST_INDEX=R1+3;
                R_SECOND_INDEX=R2+5;
                R_THIRD_INDEX=R3+5;
              }
              return 0;
            }
            else if(!last_left&&is_left&&obs_num>1&&target_x>500&&area_ratio>=0.05)
            {
              RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"上次左转这次右转");
              is_exe_obs=true;
              if(target_x<800)
              {
                L_FIRST_INDEX=L1-5;
                L_SECOND_INDEX=L2-5;
                L_THIRD_INDEX=L3-5;
              }
              else
              {
                L_FIRST_INDEX=L1-2;
                L_SECOND_INDEX=L2-2;
                L_THIRD_INDEX=L3-2;
              }
              return 0;
            }
            else if(last_left&&!is_left&&obs_num>1&&target_x<1400&&target_x>600&&area_ratio>=0.05)
            {
              RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"上次右转这次左转");
              is_exe_obs=true;
              if(target_x>1100)
              {
                R_FIRST_INDEX=L1-2;
                R_SECOND_INDEX=L2-2;
                R_THIRD_INDEX=L3-2;
              }
              return 0;
            }
            else if(timer<50&&obs_num==1)
            {
              RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"右弯道上");
              if(!is_left)
              {
                RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"左转");
                if(target_x>1350&&area_ratio>=0.09)
                {
                  is_exe_obs=true;
                  R_FIRST_INDEX=R1;
                  R_SECOND_INDEX=R2;
                  R_THIRD_INDEX=R3;
                  return 0;
                }
                else if(target_x<=1350&&area_ratio>=0.06)
                {
                  is_exe_obs=true;
                  R_FIRST_INDEX=R1+3;
                  R_SECOND_INDEX=R2+3;
                  R_THIRD_INDEX=R3+3;
                  return 0;
                }
              }
              else
              {
                if(area_ratio>=0.05&&target_x>=700)
                {
                  is_exe_obs=true;
                  RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),"右转");
                  if(target_x<1000)
                  {
                    L_FIRST_INDEX=L1+3;
                    L_SECOND_INDEX=L2+3;
                    L_THIRD_INDEX=L3+3;
                  }
                  else
                  {
                    L_FIRST_INDEX=L1+5;
                    L_SECOND_INDEX=L2+5;
                    L_THIRD_INDEX=L3+5;
                  }
                  L_START_ANGUZ = LSA+0.1;
                  return 0;
                }
              }
            }
            else if(obs_num==1)
            {
              if(!is_left)
              {
                if(area_ratio>=0.05&&target_x<1600)
                {
                  is_exe_obs=true;
                  if(target_x>1400)
                  {
                    R_FIRST_INDEX=R1-2;
                    R_SECOND_INDEX=R2-2;
                    R_THIRD_INDEX=R3+3;
                  }
                  else
                  {
                    R_FIRST_INDEX=R1;
                    R_SECOND_INDEX=R2;
                    R_THIRD_INDEX=R3+5;
                  }
                  return 0;
                }
              }
              else
              {
                if(area_ratio>=0.04&&target_x>=800)
                {
                  is_exe_obs=true;
                  if(target_x<1100)
                  {
                    L_FIRST_INDEX=L1;
                    L_SECOND_INDEX=L2;
                    L_THIRD_INDEX=L3;
                  }
                  else
                  {
                    L_FIRST_INDEX=L1+3;
                    L_SECOND_INDEX=L2+3;
                    L_THIRD_INDEX=L3+3;
                  }
                  return 0;
                }
              }
            }
          }
        }
      }
      if(!results_p.empty())
      {
        rush_p=true;
        for(auto& p_rect : results_p)
        {
          for(auto& h_rect : results_hat)
          {
            if(!(h_rect->xmax<p_rect->xmin-150||h_rect->xmin>p_rect->xmax+150))
            {
              rush_p=false;
              RCLCPP_WARN(rclcpp::get_logger("line_node"), "看到P但是周围有障碍物，不能直冲");
              break;
            }
          }
        }
      }
      else
      {
        rush_p=false;
        RCLCPP_WARN(rclcpp::get_logger("line_node"), "没看到P");
      }
      if(rush_p)
      {
        p_cnt++;
        if(p_cnt==5)
        {
          passed_obs=false;
        }
        for(auto& rect : results_p)
        {
          p_x=((rect->xmax + rect->xmin) / 2);
          // RCLCPP_ERROR(rclcpp::get_logger("line_node"), "p框占比:%f",((rect->xmax-rect->xmin)*(rect->ymax-rect->ymin))/IMAGE_AREA);
          // if(((rect->xmax-rect->xmin)*(rect->ymax-rect->ymin))/IMAGE_AREA>=0.04&&!last_left)
          // {
          //   passed_obs=false;
          // }
        }
        cmd_vel_message.angular.z = -1.0 * (p_x - LINE_REAL_MIDX) / 600.0;
        cmd_vel_message.linear.x  = STD_LINEAR_X+0.2;
        cmd_vel_message.linear.y  = 0.0, cmd_vel_message.linear.z  = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
        RCLCPP_ERROR(rclcpp::get_logger("line_node"), "冲P点,p_x:%f,angular.z:%f",p_x, cmd_vel_message.angular.z);
        cmd_vel_publisher_->publish(cmd_vel_message);
        return 0;
      }
      //巡线运动控制
      cmd_vel_message.angular.z = -1.0 * (line_x - LINE_REAL_MIDX) / 600.0;
      if(results_hat.empty())
      {
        cmd_vel_message.linear.x  = STD_LINEAR_X+0.1;
        RCLCPP_WARN(rclcpp::get_logger("playfootball_node"),"加速巡线");
      }
      else
      {
        cmd_vel_message.linear.x  = STD_LINEAR_X;
      }
      cmd_vel_message.linear.y  = 0.0, cmd_vel_message.linear.z  = 0.0;
      cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_message);
      return 0;
    }
}

int main(int argc, char * argv[])
{
  // 初始化 ROS 2
  rclcpp::init(argc, argv);

  // 创建节点实例
  rclcpp::spin(std::make_shared<OrigincarControl>());

  // 清理 ROS 2
  rclcpp::shutdown();
  return 0;
}