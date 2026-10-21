#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <string>
#include <thread>
#include "deptrum/common_types.h"
#include "glog/logging.h"

// std::string FrontPcd_path = "";
std::string FrontDepth_path = "";
std::string FrontRgb_path = "";
std::string FrontIr_path = "";

// std::string stof_FrontPcd_path = "";
std::string stof_FrontDepth_path = "";
std::string stof_FrontRgb_path = "";
std::string FrontFlag_path = "";

bool enable_rgb;
bool enable_ir;
bool enable_depth;
bool enable_pointcloud;

#ifdef STREAM_SDK_TYPE_NEBULA
bool enable_stof_rgb;
bool enable_flag;
bool enable_stof_depth;
bool enable_stof_pointcloud;
#endif

enum StreamType {
  kInvalidStreamType = 0,
  kRgb,
  kIr,
  kDepth,
  kPointCloud,  // One point per pixel; some pixels may be missing on depth map
  KStofRgb,
  kFlag,
  KStofDepth,
  kStofPointCloud,

};
// ros::Subscriber sub_rgb;
// ros::Subscriber sub_ir;
// ros::Subscriber sub_depth;
// ros::Subscriber sub_cloud;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ir;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud;
#ifdef STREAM_SDK_TYPE_NEBULA
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stof_rgb;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_flag;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_stof_depth;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_stof_cloud;
#endif

void FrontPoint_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::seconds since_epoch = std::chrono::duration_cast<std::chrono::seconds>(
      now.time_since_epoch());
  // std::cout << "start save pointcloud data to " << FrontPcd_path.c_str() << std::endl;
  static uint64_t time_now = 0;
  static uint64_t last_receive_time = std::chrono::duration_cast<std::chrono::seconds>(
                                          now.time_since_epoch())
                                          .count();
  // 输出帧率
  static uint64_t count = 0;
  count++;

  static auto start_getframe = std::chrono::high_resolution_clock::now();
  if (count % 10 == 0) {
    auto end_getframe = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::micro> tm2 = end_getframe - start_getframe;
    LOG(INFO) << "point cloud subscribe fps=" << 10000000 / tm2.count();
    start_getframe = std::chrono::high_resolution_clock::now();

    // 计算点云中点的个数
    size_t point_cloud_data_size = cloud_msg->data.size();
    size_t num_points = point_cloud_data_size / sizeof(deptrum::PointXyzRgbIr<float>);
    LOG(INFO) << "point number = " << num_points;
  }
  // 输出时间戳
  time_now = since_epoch.count();
  if (time_now - last_receive_time > 2) {
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    LOG(INFO) << "current_time:" << std::ctime(&time) << "subscribe time out ,duration is "
              << time_now - last_receive_time << "s";
  }
  last_receive_time = time_now;
}

void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  LOG(INFO) << "start save rgb pic to " << FrontRgb_path.c_str();
  builtin_interfaces::msg::Time timestamp = img_msg->header.stamp;

  std::string str;
  str = FrontRgb_path + "rgb" + std::to_string(static_cast<int>(timestamp.sec)) + '.' +
        std::to_string(static_cast<int>(timestamp.nanosec)) + ".jpg";
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, "8UC3");
  cv::imwrite(str, ptr->image);
}

void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  LOG(INFO) << "start save depth pic to " << FrontDepth_path.c_str();
  builtin_interfaces::msg::Time timestamp = img_msg->header.stamp;
  std::string str;
  str = FrontDepth_path + "depth" + std::to_string(static_cast<int>(timestamp.sec)) + '.' +
        std::to_string(static_cast<int>(timestamp.nanosec)) + ".jpg";
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, "16UC1");
  cv::imwrite(str, ptr->image);
}

void ir_image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  LOG(INFO) << "start save ir pic to " << FrontIr_path.c_str();
  builtin_interfaces::msg::Time timestamp = img_msg->header.stamp;
  std::string str;
  str = FrontIr_path + "ir" + std::to_string(static_cast<int>(timestamp.sec)) + '.' +
        std::to_string(static_cast<int>(timestamp.nanosec)) + ".jpg";
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, "8UC1");
  cv::imwrite(str, ptr->image);
}

void stof_FrontPoint_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::chrono::seconds since_epoch = std::chrono::duration_cast<std::chrono::seconds>(
      now.time_since_epoch());
  // std::cout << "start save pointcloud data to " << FrontPcd_path.c_str() << std::endl;
  static uint64_t time_now = 0;
  static uint64_t last_receive_time = std::chrono::duration_cast<std::chrono::seconds>(
                                          now.time_since_epoch())
                                          .count();
  // 输出帧率
  static uint64_t count = 0;
  count++;

  static auto start_getframe = std::chrono::high_resolution_clock::now();
  if (count % 10 == 0) {
    auto end_getframe = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::micro> tm2 = end_getframe - start_getframe;
    LOG(INFO) << "stof point cloud subscribe fps=" << 10000000 / tm2.count();
    start_getframe = std::chrono::high_resolution_clock::now();

    // 计算点云中点的个数
    size_t point_cloud_data_size = cloud_msg->data.size();
    size_t num_points = point_cloud_data_size / sizeof(deptrum::PointXyzRgb<float>);
    LOG(INFO) << "stof point number = " << num_points;
  }
  // 输出时间戳
  time_now = since_epoch.count();
  if (time_now - last_receive_time > 2) {
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    LOG(INFO) << "current_time:" << std::ctime(&time) << "subscribe time out ,duration is "
              << time_now - last_receive_time << "s";
  }
  last_receive_time = time_now;
}

void stof_rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  LOG(INFO) << "start save stof rgb pic to " << stof_FrontRgb_path.c_str();
  builtin_interfaces::msg::Time timestamp = img_msg->header.stamp;

  std::string str;
  str = stof_FrontRgb_path + "stof_rgb" + std::to_string(static_cast<int>(timestamp.sec)) + '.' +
        std::to_string(static_cast<int>(timestamp.nanosec)) + ".jpg";
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, "8UC3");
  cv::imwrite(str, ptr->image);
}

void stof_depth_image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  LOG(INFO) << "start save stof depth pic to " << stof_FrontDepth_path.c_str();
  builtin_interfaces::msg::Time timestamp = img_msg->header.stamp;
  std::string str;
  str = stof_FrontDepth_path + "stof_depth" + std::to_string(static_cast<int>(timestamp.sec)) +
        '.' + std::to_string(static_cast<int>(timestamp.nanosec)) + ".jpg";
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, "16UC1");
  cv::imwrite(str, ptr->image);
}

void flag_image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
  LOG(INFO) << "start save flag pic to " << FrontFlag_path.c_str();
  builtin_interfaces::msg::Time timestamp = img_msg->header.stamp;
  std::string str;
  str = FrontFlag_path + "flag" + std::to_string(static_cast<int>(timestamp.sec)) + '.' +
        std::to_string(static_cast<int>(timestamp.nanosec)) + ".jpg";
  cv_bridge::CvImageConstPtr ptr;
  ptr = cv_bridge::toCvCopy(img_msg, "8UC3");
  cv::imwrite(str, ptr->image);
}

void ChooceStreamType(std::vector<StreamType>& stream_types_vector) {
  if (enable_rgb == true) {
    stream_types_vector.emplace_back(kRgb);
  }
  if (enable_ir == true) {
    stream_types_vector.emplace_back(kIr);
  }
  if (enable_depth == true) {
    stream_types_vector.emplace_back(kDepth);
  }
  if (enable_pointcloud == true) {
    stream_types_vector.emplace_back(kPointCloud);
  }
#if defined STREAM_SDK_TYPE_NEBULA
  if (enable_stof_rgb == true) {
    stream_types_vector.emplace_back(KStofRgb);
  }
  if (enable_flag == true) {
    stream_types_vector.emplace_back(kFlag);
  }
  if (enable_stof_depth == true) {
    stream_types_vector.emplace_back(KStofDepth);
  }
  if (enable_stof_pointcloud == true) {
    stream_types_vector.emplace_back(kStofPointCloud);
  }
#endif
  else {
    std::cout << "please input enable_rgb:=true or enable_ir:=true ....: " << std::endl;
  }
}

#ifndef STREAM_SDK_TYPE_NEBULA
void RegisterTopic(StreamType stream_type,
                   std::shared_ptr<rclcpp::Node>& nh,
                   std::string rgb_topic,
                   std::string ir_topic,
                   std::string depth_topic,
                   std::string point_cloud_topic) {
  if (stream_type == kRgb) {
    std::cout << "订阅" << rgb_topic.c_str() << "成功" << std::endl;
    sub_rgb = nh->create_subscription<sensor_msgs::msg::Image>(rgb_topic, 10, rgb_image_callback);
  } else if (stream_type == kIr) {
    std::cout << "订阅" << ir_topic.c_str() << "成功" << std::endl;
    sub_ir = nh->create_subscription<sensor_msgs::msg::Image>(ir_topic, 10, ir_image_callback);
  } else if (stream_type == kDepth) {
    std::cout << "订阅" << depth_topic.c_str() << "成功" << std::endl;
    sub_depth = nh->create_subscription<sensor_msgs::msg::Image>(depth_topic,
                                                                 10,
                                                                 depth_image_callback);
  } else if (stream_type == kPointCloud) {
    std::cout << "订阅" << point_cloud_topic.c_str() << "成功" << std::endl;
    sub_cloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic,
                                                                       10,
                                                                       FrontPoint_callback);
  }
};

#else

void RegisterTopic(StreamType stream_type,
                   std::shared_ptr<rclcpp::Node>& nh,
                   std::string rgb_topic,
                   std::string ir_topic,
                   std::string depth_topic,
                   std::string point_cloud_topic,
                   std::string stof_rgb_topic,
                   std::string flag_topic,
                   std::string stof_depth_topic,
                   std::string stof_point_cloud_topic) {
  if (stream_type == kRgb) {
    std::cout << "订阅" << rgb_topic.c_str() << "成功" << std::endl;
    sub_rgb = nh->create_subscription<sensor_msgs::msg::Image>(rgb_topic, 10, rgb_image_callback);

  } else if (stream_type == kIr) {
    std::cout << "订阅" << ir_topic.c_str() << "成功" << std::endl;
    sub_ir = nh->create_subscription<sensor_msgs::msg::Image>(ir_topic, 10, ir_image_callback);
  } else if (stream_type == kDepth) {
    std::cout << "订阅" << depth_topic.c_str() << "成功" << std::endl;
    sub_depth = nh->create_subscription<sensor_msgs::msg::Image>(depth_topic,
                                                                 10,
                                                                 depth_image_callback);
  } else if (stream_type == kPointCloud) {
    std::cout << "订阅" << point_cloud_topic.c_str() << "成功" << std::endl;
    sub_cloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic,
                                                                       10,
                                                                       FrontPoint_callback);
  } else if (stream_type == KStofRgb) {
    std::cout << "订阅" << stof_rgb_topic.c_str() << "成功" << std::endl;
    sub_stof_rgb = nh->create_subscription<sensor_msgs::msg::Image>(stof_rgb_topic,
                                                                    10,
                                                                    stof_rgb_image_callback);

  } else if (stream_type == kFlag) {
    std::cout << "订阅" << flag_topic.c_str() << "成功" << std::endl;
    sub_ir = nh->create_subscription<sensor_msgs::msg::Image>(flag_topic, 10, flag_image_callback);
  } else if (stream_type == KStofDepth) {
    std::cout << "订阅" << stof_depth_topic.c_str() << "成功" << std::endl;
    sub_stof_depth = nh->create_subscription<sensor_msgs::msg::Image>(stof_depth_topic,
                                                                      10,
                                                                      stof_depth_image_callback);
  } else if (stream_type == kStofPointCloud) {
    std::cout << "订阅" << stof_point_cloud_topic.c_str() << "成功" << std::endl;
    sub_stof_cloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        stof_point_cloud_topic,
        10,
        stof_FrontPoint_callback);
  }
};
#endif

void CreateSaveImageFiles() {
  boost::filesystem::path rgb_path = "/tmp/image/rgb";
  if (!boost::filesystem::exists(rgb_path)) {
    boost::filesystem::create_directories(rgb_path);
  }
  boost::filesystem::path ir_path = "/tmp/image/ir";
  if (!boost::filesystem::exists(ir_path)) {
    boost::filesystem::create_directories(ir_path);
  }
  boost::filesystem::path depth_path = "/tmp/image/depth";
  if (!boost::filesystem::exists(depth_path)) {
    boost::filesystem::create_directories(depth_path);
  }
  // boost::filesystem::path FrontPcd_path = "/tmp/image/point_cloud";
  // if (!boost::filesystem::exists(FrontPcd_path)) {
  //   boost::filesystem::create_directories(FrontPcd_path);
  // }
#ifdef STREAM_SDK_TYPE_NEBULA
  boost::filesystem::path stof_rgb_path = "/tmp/image/stof_rgb";
  if (!boost::filesystem::exists(stof_rgb_path)) {
    boost::filesystem::create_directories(stof_rgb_path);
  }
  boost::filesystem::path flag_path = "/tmp/image/flag";
  if (!boost::filesystem::exists(flag_path)) {
    boost::filesystem::create_directories(flag_path);
  }
  boost::filesystem::path stof_depth_path = "/tmp/image/stof_depth";
  if (!boost::filesystem::exists(stof_depth_path)) {
    boost::filesystem::create_directories(stof_depth_path);
  }
  // boost::filesystem::path stof_point_cloud_path = "/tmp/image/stof_point_cloud";
  // if (!boost::filesystem::exists(stof_point_cloud_path)) {
  //   boost::filesystem::create_directories(stof_point_cloud_path);
  // }
#endif
}

int main(int argc, char** argv) {
  // setlocale(LC_CTYPE, "zh_CN.utf8");
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_logbuflevel = -1;  // 不缓存
  google::SetStderrLogging(google::GLOG_INFO);
  std::string info_log = "/tmp/deptrum_ros_driver_subscribe_";
  google::SetLogDestination(0, info_log.c_str());
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("sub_node_ci");

  std::string point_cloud_topic;
  std::string depth_topic;
  std::string ir_topic;
  std::string rgb_topic;
#if defined STREAM_SDK_TYPE_NEBULA
  std::string stof_point_cloud_topic;
  std::string stof_depth_topic;
  std::string stof_rgb_topic;
  std::string flag_topic;
#endif

  std::vector<StreamType> stream_types_vector;
  std::vector<std::thread*> stream_thread_vector;
  stream_types_vector.clear();
  stream_thread_vector.clear();
  CreateSaveImageFiles();

#if defined STREAM_SDK_TYPE_AURORA900 || defined STREAM_SDK_TYPE_AURORA930
  nh->declare_parameter<std::string>("point_cloud_topic", std::string("/aurora/points2"));
  nh->get_parameter<std::string>("point_cloud_topic", point_cloud_topic);

  nh->declare_parameter<std::string>("depth_topic", std::string("/aurora/depth/image_raw"));
  nh->get_parameter<std::string>("depth_topic", depth_topic);

  nh->declare_parameter<std::string>("rgb_topic", std::string("/aurora/rgb/image_raw"));
  nh->get_parameter<std::string>("rgb_topic", rgb_topic);

  nh->declare_parameter<std::string>("ir_topic", std::string("/aurora/ir/image_raw"));
  nh->get_parameter<std::string>("ir_topic", ir_topic);
#endif
#if defined STREAM_SDK_TYPE_STELLAR400 || defined STREAM_SDK_TYPE_STELLAR420
  nh->declare_parameter<std::string>("point_cloud_topic", std::string("/stellar/points2"));
  nh->get_parameter<std::string>("point_cloud_topic", point_cloud_topic);

  nh->declare_parameter<std::string>("depth_topic", std::string("/stellar/depth/image_raw"));
  nh->get_parameter<std::string>("depth_topic", depth_topic);

  nh->declare_parameter<std::string>("rgb_topic", std::string("/stellar/rgb/image_raw"));
  nh->get_parameter<std::string>("rgb_topic", rgb_topic);

  nh->declare_parameter<std::string>("ir_topic", std::string("/stellar/ir/image_raw"));
  nh->get_parameter<std::string>("ir_topic", ir_topic);
#endif
#ifdef STREAM_SDK_TYPE_NEBULA
  nh->declare_parameter<std::string>("point_cloud_topic", std::string("/nebula/mtof_points2"));
  nh->get_parameter<std::string>("point_cloud_topic", point_cloud_topic);

  nh->declare_parameter<std::string>("depth_topic", std::string("/nebula/mtof_depth/image_raw"));
  nh->get_parameter<std::string>("depth_topic", depth_topic);

  nh->declare_parameter<std::string>("rgb_topic", std::string("/nebula/mtof_rgb/image_raw"));
  nh->get_parameter<std::string>("rgb_topic", rgb_topic);

  nh->declare_parameter<std::string>("ir_topic", std::string("/nebula/ir/image_raw"));
  nh->get_parameter<std::string>("ir_topic", ir_topic);

  nh->declare_parameter<std::string>("stof_point_cloud_topic", std::string("/nebula/stof_points2"));
  nh->get_parameter<std::string>("stof_point_cloud_topic", stof_point_cloud_topic);

  nh->declare_parameter<std::string>("stof_depth_topic",
                                     std::string("/nebula/stof_depth/image_raw"));
  nh->get_parameter<std::string>("stof_depth_topic", stof_depth_topic);

  nh->declare_parameter<std::string>("stof_rgb_topic", std::string("/nebula/stof_rgb/image_raw"));
  nh->get_parameter<std::string>("stof_rgb_topic", stof_rgb_topic);

  nh->declare_parameter<std::string>("flag_topic", std::string("/nebula/flag/image_raw"));
  nh->get_parameter<std::string>("flag_topic", flag_topic);

  // nh->declare_parameter<std::string>("stof_FrontPcd_path",
  //                                    std::string("/tmp/image/stof_point_cloud/"));
  // nh->get_parameter<std::string>("stof_FrontPcd_path", stof_FrontPcd_path);

  nh->declare_parameter<std::string>("stof_depth_path", std::string("/tmp/image/stof_depth/"));
  nh->get_parameter<std::string>("stof_depth_path", stof_FrontDepth_path);

  nh->declare_parameter<std::string>("stof_rgb_path", std::string("/tmp/image/stof_rgb/"));
  nh->get_parameter<std::string>("stof_rgb_path", stof_FrontRgb_path);

  nh->declare_parameter<std::string>("flag_path", std::string("/tmp/image/flag/"));
  nh->get_parameter<std::string>("flag_path", FrontFlag_path);

  nh->declare_parameter<bool>("enable_stof_rgb", true);
  nh->get_parameter<bool>("enable_stof_rgb", enable_stof_rgb);

  nh->declare_parameter<bool>("enable_flag", true);
  nh->get_parameter<bool>("enable_flag", enable_flag);

  nh->declare_parameter<bool>("enable_stof_depth", true);
  nh->get_parameter<bool>("enable_stof_depth", enable_stof_depth);

  nh->declare_parameter<bool>("enable_stof_pointcloud", true);
  nh->get_parameter<bool>("enable_stof_pointcloud", enable_stof_pointcloud);

#endif
  // nh->declare_parameter<std::string>("FrontPcd_path", std::string("/tmp/image/point_cloud/"));
  // nh->get_parameter<std::string>("FrontPcd_path", FrontPcd_path);

  nh->declare_parameter<std::string>("depth_path", std::string("/tmp/image/depth/"));
  nh->get_parameter<std::string>("depth_path", FrontDepth_path);

  nh->declare_parameter<std::string>("rgb_path", std::string("/tmp/image/rgb/"));
  nh->get_parameter<std::string>("rgb_path", FrontRgb_path);

  nh->declare_parameter<std::string>("ir_path", std::string("/tmp/image/ir/"));
  nh->get_parameter<std::string>("ir_path", FrontIr_path);

  nh->declare_parameter<bool>("enable_rgb", true);
  nh->get_parameter<bool>("enable_rgb", enable_rgb);

  nh->declare_parameter<bool>("enable_ir", true);
  nh->get_parameter<bool>("enable_ir", enable_ir);

  nh->declare_parameter<bool>("enable_depth", true);
  nh->get_parameter<bool>("enable_depth", enable_depth);

  nh->declare_parameter<bool>("enable_pointcloud", true);
  nh->get_parameter<bool>("enable_pointcloud", enable_pointcloud);
  ChooceStreamType(stream_types_vector);
  if (stream_types_vector.empty())
    return 0;
  for (int i = 0; i < stream_types_vector.size(); i++) {
    LOG(INFO) << "-----------------" << stream_types_vector[i];

#ifdef STREAM_SDK_TYPE_NEBULA
    stream_thread_vector.emplace_back(new std::thread(RegisterTopic,
                                                      stream_types_vector[i],
                                                      std::ref(nh),
                                                      rgb_topic,
                                                      ir_topic,
                                                      depth_topic,
                                                      point_cloud_topic,
                                                      stof_rgb_topic,
                                                      flag_topic,
                                                      stof_depth_topic,
                                                      stof_point_cloud_topic));
#else

    stream_thread_vector.emplace_back(new std::thread(RegisterTopic,
                                                      stream_types_vector[i],
                                                      std::ref(nh),
                                                      rgb_topic,
                                                      ir_topic,
                                                      depth_topic,
                                                      point_cloud_topic));
#endif
  }
  rclcpp::spin(nh);
  if (!stream_thread_vector.empty()) {
    for (auto th : stream_thread_vector) {
      th->join();
      delete th;
      th = nullptr;
    }
    stream_thread_vector.clear();
  }
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}
