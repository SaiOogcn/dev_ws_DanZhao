#include "base_orin/base_orin.h"
#include "rclcpp/rclcpp.hpp"
#include "base_orin/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" 
#include "origincar_msg/msg/data.hpp"

using std::placeholders::_1;
using namespace std;
void sigintHandler(int sig);
sensor_msgs::msg::Imu Mpu6050;
rclcpp::Node::SharedPtr node_handle = nullptr;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    base_orin Robot_Control;
    Robot_Control.Control();
    rclcpp::shutdown();
    return 0;
}

short base_orin::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    return transition_16;
}

float base_orin::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001;
    return data_return;
  }

void base_orin::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
{
    short  transition;
  
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0; 

    transition=0;
    transition = akm_ctl->drive.speed*1000;
    Send_Data.tx[4] = transition;
    Send_Data.tx[3] = transition>>8;

    transition=0;
    transition = akm_ctl->drive.steering_angle*1000/2;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); 
    Send_Data.tx[10]=FRAME_TAIL;

    try {
      Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
    }
}

void base_orin::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    short  transition;
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0; 

    transition=0;
    transition = twist_aux->linear.x*1000;
    Send_Data.tx[4] = transition;
    Send_Data.tx[3] = transition>>8;

    transition=0;
    transition = twist_aux->linear.y*1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition>>8;

    // Calibration is expressed as curvature [rad/m], so the yaw-rate
    // correction naturally changes sign when the vehicle reverses.
    const double corrected_angular_z = twist_aux->angular.z +
      twist_aux->linear.x * chassis_curvature_compensation_;
    transition=0;
    transition = corrected_angular_z*1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);
    Send_Data.tx[10]=FRAME_TAIL;

    try {
      if (akm_cmd_vel == "none") {
        Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
      } 
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
    }
}

void base_orin::Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch)
{
  (void)sign_switch;
    /* if (sign_switch->data == -1) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 0.5;
         Robot_Pos.Y = 0.2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }
     else if (sign_switch->data == 6) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 2;
         Robot_Pos.Y = 2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }*/
}

void base_orin::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = rclcpp::Node::now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id;

    // /imu/data_raw intentionally carries no orientation estimate. Madgwick
    // computes it from the raw angular velocity and acceleration separately.
    Imu_Data_Pub.orientation.w = 1.0;
    Imu_Data_Pub.orientation_covariance[0] = -1.0;
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance[0] = 4e-6;
    Imu_Data_Pub.angular_velocity_covariance[4] = 3e-6;
    Imu_Data_Pub.angular_velocity_covariance[8] = imu_gyro_z_variance_;
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
    Imu_Data_Pub.linear_acceleration_covariance[0] = 0.001;
    Imu_Data_Pub.linear_acceleration_covariance[4] = 0.001;
    Imu_Data_Pub.linear_acceleration_covariance[8] = 0.001;

    imu_publisher->publish(Imu_Data_Pub);

}

void base_orin::Publish_Odom()
{
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    origincar_msg::msg::Data robotpose;
    origincar_msg::msg::Data robotvel;
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = robot_frame_id;

    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;

    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;


    odom.twist.twist.linear.x =  Robot_Vel.X;
    odom.twist.twist.linear.y =  Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z;

    auto &pose_cov = odom.pose.covariance;
    pose_cov.fill(0.0);
    pose_cov[0] = 0.0025;
    pose_cov[7] = 0.0025;
    pose_cov[14] = 1e6;
    pose_cov[21] = 1e6;
    pose_cov[28] = 1e6;
    pose_cov[35] = 0.0076;

    auto &twist_cov = odom.twist.covariance;
    twist_cov.fill(0.0);
    twist_cov[0] = 0.01;
    twist_cov[7] = 0.01;
    twist_cov[14] = 1e6;
    twist_cov[21] = 1e6;
    twist_cov[28] = 1e6;
    twist_cov[35] = 0.03;

    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

    odom_publisher->publish(odom);
    robotpose_publisher->publish(robotpose);
    robotvel_publisher->publish(robotvel); 
}

void base_orin::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;

    if (Count_Voltage_Pub++ > 10) {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;
        voltage_publisher->publish(voltage_msgs);
    }
}

unsigned char base_orin::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum = 0, k;

    if (mode == 0) {
      for(k=0; k < Count_Number; k++) {
        check_sum = check_sum^Receive_Data.rx[k];
      }
    } else if (mode == 1) {
      for (k=0; k < Count_Number; k++) {
        check_sum = check_sum^Send_Data.tx[k];
      }
    }

    return check_sum;
}

bool base_orin::Get_Sensor_Data()
{
    short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0;
    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0}; 
    Stm32_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr)); 
    for (j = 0; j < 24; j++) {
      if (Receive_Data_Pr[j] == FRAME_HEADER)
      Header_Pos=j;
      else if (Receive_Data_Pr[j] == FRAME_TAIL)
      Tail_Pos = j;
    }

    if (Tail_Pos == (Header_Pos + 23)) {
      memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }  else if (Header_Pos == (1 + Tail_Pos)) {
      for (j = 0;j < 24; j++)
      Receive_Data.rx[j] = Receive_Data_Pr[(j+Header_Pos) % 24];
    }  else {
    return false;
    }

    Receive_Data.Frame_Header = Receive_Data.rx[0];
    Receive_Data.Frame_Tail = Receive_Data.rx[23];
    if (Receive_Data.Frame_Header == FRAME_HEADER) {
      if (Receive_Data.Frame_Tail == FRAME_TAIL) {
        if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK)||(Header_Pos == (1 + Tail_Pos))) {
          Receive_Data.Flag_Stop=Receive_Data.rx[1];
          Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]);
        
          Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]);
                                                                          
          Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); 

          Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);
          Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);
          Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);
          Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);
          Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);
          Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);

          Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
          Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
          Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

          Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
          Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
          Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

          transition_16 = 0;
          transition_16 |=  Receive_Data.rx[20]<<8;
          transition_16 |=  Receive_Data.rx[21];
          Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;

          return true;
        }
      }
    }

    return false;
}

void base_orin::Control()
{
    rclcpp::Time current_time, last_time;
    current_time = rclcpp::Node::now();
    last_time = rclcpp::Node::now();
    while(rclcpp::ok()) {
      current_time = rclcpp::Node::now();
      Sampling_Time = (current_time - last_time).seconds();
      if (true == Get_Sensor_Data()) {
		const double raw_gyro_z = Mpu6050.angular_velocity.z;
		if (!imu_gyro_bias_calibrated_) {
		  if (imu_gyro_bias_warmup_count_ < imu_gyro_bias_warmup_samples_) {
			++imu_gyro_bias_warmup_count_;
		  } else if (std::abs(Robot_Vel.X) > 0.01 ||
				 std::abs(Robot_Vel.Y) > 0.01 ||
				 std::abs(Robot_Vel.Z) > 0.01) {
			imu_gyro_bias_sample_count_ = 0;
			imu_gyro_z_sum_ = 0.0;
			imu_gyro_z_sum_squares_ = 0.0;
		  } else {
			imu_gyro_z_sum_ += raw_gyro_z;
			imu_gyro_z_sum_squares_ += raw_gyro_z * raw_gyro_z;
			++imu_gyro_bias_sample_count_;
			if (imu_gyro_bias_sample_count_ >= imu_gyro_bias_calibration_samples_) {
			  const double mean = imu_gyro_z_sum_ / imu_gyro_bias_sample_count_;
			  double variance = imu_gyro_z_sum_squares_ / imu_gyro_bias_sample_count_ - mean * mean;
			  variance = variance > 0.0 ? variance : 0.0;
			  const double stddev = std::sqrt(variance);
			  if (stddev <= imu_gyro_bias_max_stddev_) {
				imu_gyro_z_bias_ = mean;
				imu_gyro_bias_calibrated_ = true;
				RCLCPP_INFO(this->get_logger(),
					"IMU gyro Z bias calibrated from %d samples: %.6f rad/s (stddev %.6f)",
					imu_gyro_bias_sample_count_, imu_gyro_z_bias_, stddev);
			  } else {
				RCLCPP_WARN(this->get_logger(),
					"IMU gyro Z calibration rejected: stddev %.6f exceeds %.6f rad/s; retrying",
					stddev, imu_gyro_bias_max_stddev_);
				imu_gyro_bias_sample_count_ = 0;
				imu_gyro_z_sum_ = 0.0;
				imu_gyro_z_sum_squares_ = 0.0;
			  }
			}
		  }
		}
		Mpu6050.angular_velocity.z = raw_gyro_z - imu_gyro_z_bias_;

        Robot_Pos.X+=1.03*(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
        Robot_Pos.Y+=1.125*(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
        Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time;

        Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                  Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
		if (imu_gyro_bias_calibrated_) {
		  Publish_ImuSensor();
		}
        Publish_Voltage();
        Publish_Odom();
        rclcpp::spin_some(this->get_node_base_interface());
      }
      last_time = current_time;
    }
}

base_orin::base_orin()
: rclcpp::Node ("base_orin")
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  int serial_baud_rate = 115200;

  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<bool>("calibrate_imu_gyro_z_bias", true);
  this->declare_parameter<int>("imu_gyro_bias_warmup_samples", 200);
  this->declare_parameter<int>("imu_gyro_bias_calibration_samples", 200);
  this->declare_parameter<double>("imu_gyro_z_bias", 0.00128);
  this->declare_parameter<double>("imu_gyro_z_variance", 1e-6);
  this->declare_parameter<double>("imu_gyro_bias_max_stddev", 0.0015);
  this->declare_parameter<double>("chassis_curvature_compensation", 0.0);

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("calibrate_imu_gyro_z_bias", calibrate_imu_gyro_z_bias_);
  this->get_parameter("imu_gyro_bias_warmup_samples", imu_gyro_bias_warmup_samples_);
  this->get_parameter("imu_gyro_bias_calibration_samples", imu_gyro_bias_calibration_samples_);
  this->get_parameter("imu_gyro_z_bias", imu_gyro_z_bias_);
  this->get_parameter("imu_gyro_z_variance", imu_gyro_z_variance_);
  this->get_parameter("imu_gyro_bias_max_stddev", imu_gyro_bias_max_stddev_);
  this->get_parameter("chassis_curvature_compensation", chassis_curvature_compensation_);
  if (imu_gyro_bias_warmup_samples_ < 0) {
    imu_gyro_bias_warmup_samples_ = 0;
  }
  if (imu_gyro_bias_calibration_samples_ <= 0) {
    imu_gyro_bias_calibration_samples_ = 1;
  }
  imu_gyro_bias_calibrated_ = !calibrate_imu_gyro_z_bias_;
  if (calibrate_imu_gyro_z_bias_) {
    RCLCPP_INFO(this->get_logger(),
        "IMU gyro Z calibration will discard %d warmup samples, then collect %d stationary samples",
        imu_gyro_bias_warmup_samples_, imu_gyro_bias_calibration_samples_);
  }
  RCLCPP_INFO(this->get_logger(),
      "Chassis curvature compensation: %+.5f rad/m", chassis_curvature_compensation_);

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);

  robotpose_publisher = create_publisher<origincar_msg::msg::Data>("robotpose", 10);

  robotvel_publisher = create_publisher<origincar_msg::msg::Data>("robotvel", 10);

  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 1, std::bind(&base_orin::Cmd_Vel_Callback, this, _1));
  Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      akm_cmd_vel, 1, std::bind(&base_orin::Akm_Cmd_Vel_Callback, this, _1));

  // Sign_Switch_Sub = create_subscription<std_msgs::msg::Int32>(
  //     "/sign4return", 1, std::bind(&base_orin::Sign_Switch_Callback, this, _1));
  try  {
    Stm32_Serial.setPort("/dev/ttyACM0");
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
  } catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(),"base_orin can not open serial port,Please check the serial port cable! ");
  }
  if(Stm32_Serial.isOpen()) {
    RCLCPP_INFO(this->get_logger(),"base_orin serial port opened");
  }
}


void sigintHandler(int sig)
{
    sig = sig;
      printf("OriginBot shutdown...\n");
    serial::Serial Stm32_Serial;
    Stm32_Serial.setPort("/dev/ttyACM0");
    Stm32_Serial.setBaudrate(115200);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();                                       
    SEND_DATA Send_Data;
    if (Stm32_Serial.isOpen()) {
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0;

    Send_Data.tx[4] = 0;
    Send_Data.tx[3] = 0;

    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;

    Send_Data.tx[7] = 0;
    Send_Data.tx[8] = 0;
    int check_sum = 0;
    for (int k = 0; k < 9; k++) {
        check_sum = check_sum^Send_Data.tx[k];
      }
    Send_Data.tx[9]=check_sum;
    Send_Data.tx[10]=FRAME_TAIL;

    try {
        Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
    } catch (serial::IOException& e) {
    }

  }
    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();
}

base_orin::~base_orin()
{
  RCLCPP_INFO(this->get_logger(),"Shutting down");
}
