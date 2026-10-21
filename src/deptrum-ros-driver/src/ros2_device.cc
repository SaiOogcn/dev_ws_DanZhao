#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "deptrum_ros_driver/camera_parameters.h"
#include "deptrum_ros_driver/ros2_device.h"
#include "glog/logging.h"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace deptrum;
RosDevice::RosDevice(std::shared_ptr<rclcpp::Node> node) : node_(node), camera_parameters_(node) {
  running_.store(false);
  node_->declare_parameter<bool>("rgb_enable", true);
  node_->declare_parameter<bool>("ir_enable", true);
  node_->declare_parameter<bool>("depth_enable", true);
  node_->declare_parameter<bool>("point_cloud_enable", true);
  node_->declare_parameter<bool>("rgbd_enable", false);
  node_->declare_parameter<bool>("flag_enable", true);
  node_->declare_parameter<bool>("outlier_point_removal_flag", false);
  node_->declare_parameter<bool>("shuffle_enable", false);
  node_->declare_parameter<bool>("undistortion_enable", false);
  node_->declare_parameter<bool>("exposure_enable", false);
  node_->declare_parameter<bool>("gain_enable", false);
  node_->declare_parameter<bool>("heart_enable", false);
  node_->declare_parameter<bool>("stream_sdk_log_enable", true);
  node_->declare_parameter<bool>("depth_correction", false);
  node_->declare_parameter<bool>("align_mode", false);
  node_->declare_parameter<int>("boot_order", 1);
  node_->declare_parameter<int>("ir_fps", 15);
  node_->declare_parameter<int>("rgb_fps", 25);
  node_->declare_parameter<int>("resolution_mode_index", 0);
  node_->declare_parameter<int>("device_numbers", 1);
  node_->declare_parameter<int>("heart_timeout_times", 8);
  node_->declare_parameter<int>("exposure_time", 1000);
  node_->declare_parameter<int>("gain_value", 20);
  node_->declare_parameter<int>("filter_type", 1);
  node_->declare_parameter<int>("ir_frequency_fusion_threshold", 16);
  node_->declare_parameter<float>("depth_frequency_fusion_threshold", 300.f);
  node_->declare_parameter<float>("ratio_scatter_filter_threshold", 0.04f);
  node_->declare_parameter<std::string>("serial_number", std::string(""));
  node_->declare_parameter<std::string>("update_file_path", std::string(""));
  node_->declare_parameter<std::string>("usb_port_number", std::string(""));
  node_->declare_parameter<std::string>("log_dir", std::string("/tmp/"));
  node_->declare_parameter<int>("threshold_size", 110);
  node_->declare_parameter<float>("laser_power", 1.0f);
  node_->declare_parameter<int>("minimum_filter_depth_value", 150);
  node_->declare_parameter<int>("maximum_filter_depth_value", 4000);
  node_->declare_parameter<int>("slam_mode", 0);
  node_->declare_parameter<int>("stof_minimum_range", 10);
  node_->declare_parameter<int>("mtof_filter_level", 1);
  node_->declare_parameter<int>("stof_filter_level", 1);
  node_->declare_parameter<int>("mtof_crop_up", 50);
  node_->declare_parameter<int>("mtof_crop_down", 80);
  node_->declare_parameter<int>("ext_rgb_mode", 0);
  node_->declare_parameter<bool>("enable_imu", 1);

  node_->get_parameter<bool>("rgb_enable", params_.rgb_enable);
  node_->get_parameter<bool>("ir_enable", params_.ir_enable);
  node_->get_parameter<bool>("depth_enable", params_.depth_enable);
  node_->get_parameter<bool>("point_cloud_enable", params_.point_cloud_enable);
  node_->get_parameter<bool>("rgbd_enable", params_.rgbd_enable);
  node_->get_parameter<bool>("flag_enable", params_.flag_enable);
  node_->get_parameter<bool>("outlier_point_removal_flag", params_.outlier_point_removal_flag);
  node_->get_parameter<bool>("shuffle_enable", params_.shuffle_enable);
  node_->get_parameter<bool>("undistortion_enable", params_.undistortion_enable);
  node_->get_parameter<bool>("exposure_enable", params_.exposure_enable);
  node_->get_parameter<bool>("gain_enable", params_.gain_enable);
  node_->get_parameter<bool>("heart_enable", params_.heart_enable);
  node_->get_parameter<bool>("stream_sdk_log_enable", params_.stream_sdk_log_enable);
  node_->get_parameter<bool>("depth_correction", params_.depth_correction);
  node_->get_parameter<bool>("align_mode", params_.align_mode);

  node_->get_parameter<int>("boot_order", params_.boot_order);
  node_->get_parameter<int>("ir_fps", params_.ir_fps);
  node_->get_parameter<int>("rgb_fps", params_.rgb_fps);
  node_->get_parameter<int>("resolution_mode_index", params_.resolution_mode_index);
  node_->get_parameter<int>("device_numbers", params_.device_numbers);
  node_->get_parameter<int>("heart_timeout_times", params_.heart_timeout_times);
  node_->get_parameter<int>("exposure_time", params_.exposure_time);
  node_->get_parameter<int>("gain_value", params_.gain_value);
  node_->get_parameter<int>("filter_type", params_.filter_type);
  node_->get_parameter<int>("ir_frequency_fusion_threshold", params_.ir_th);
  node_->get_parameter<float>("depth_frequency_fusion_threshold", params_.depth_th);
  node_->get_parameter<float>("ratio_scatter_filter_threshold", params_.ratio_th);
  node_->get_parameter<int>("threshold_size", params_.threshold_size);
  node_->get_parameter<std::string>("serial_number", params_.serial_number);
  node_->get_parameter<std::string>("update_file_path", params_.update_file_path);
  node_->get_parameter<std::string>("usb_port_number", params_.usb_port_number);
  node_->get_parameter<std::string>("log_dir", params_.log_dir);
  node_->get_parameter<float>("laser_power", params_.laser_power);
  node_->get_parameter<int>("minimum_filter_depth_value", params_.minimum_filter_depth_value);
  node_->get_parameter<int>("maximum_filter_depth_value", params_.maximum_filter_depth_value);
  node_->get_parameter<int>("slam_mode", params_.slam_mode);
  node_->get_parameter<int>("mtof_filter_level", params_.mtof_filter_level);
  node_->get_parameter<int>("stof_filter_level", params_.stof_filter_level);
  node_->get_parameter<int>("mtof_crop_up", params_.mtof_crop_up);
  node_->get_parameter<int>("mtof_crop_down", params_.mtof_crop_down);
  node_->get_parameter<int>("ext_rgb_mode", params_.ext_rgb_mode);
  node_->get_parameter<bool>("enable_imu", params_.enable_imu);

  node_->get_parameter<int>("stof_minimum_range", params_.stof_minimum_range);
  std::string info_log = params_.log_dir + "deptrum_ros_driver_";
  google::SetLogDestination(0, info_log.c_str());

  handle_ = std::bind(&RosDevice::DeviceConnectCb,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2);
  deptrum::stream::DeviceManager::GetInstance()->RegisterDeviceConnectedCallback(handle_);
  upgrade_handle_ = std::bind(&RosDevice::UpgradeProgress, this, std::placeholders::_1);
}
RosDevice::~RosDevice() {
  if (device_ != nullptr) {
    LOG(INFO) << "close device...";
    device_->Close();
  }
  LOG(INFO) << "quit succeed";
}

int RosDevice::rgb_frame_publish(const deptrum::stream::StreamFrame& rgb_frame,
                                 const builtin_interfaces::msg::Time& capture_time) {
  sensor_msgs::msg::Image::UniquePtr rgb_raw_frame(new sensor_msgs::msg::Image());
  // int status = RgbFrameToRos(rgb_raw_frame, rgb_frame);
  // if (status != 0)
  //   return -1;

  cv_bridge::CvImage cvi_rgb;

  if (rgb_frame.rows * 1.5f * rgb_frame.cols == rgb_frame.size) {
    cv::Mat yuvimg(rgb_frame.rows * 1.5f, rgb_frame.cols, CV_8UC1, rgb_frame.data.get());
    cv::Mat rgb_mat(rgb_frame.rows, rgb_frame.cols, CV_8UC3);
    cv::cvtColor(yuvimg, rgb_mat, cv::COLOR_YUV2BGR_NV12);
    cvi_rgb.header.stamp = capture_time;
    cvi_rgb.header.frame_id = camera_parameters_.rgb_camera_frame_;
    // rgb_raw_frame->height = rgb_frame.rows;
    // rgb_raw_frame->width = rgb_frame.cols;
    cvi_rgb.encoding = "bgr8";
    // cvi_rgb.is_bigendian = false;
    cvi_rgb.image = rgb_mat;
    sensor_msgs::msg::Image im_rgb;
    cvi_rgb.toImageMsg(im_rgb);
    rgb_pub_->publish(im_rgb);
  } else {
    cv::Mat cv_rgb(rgb_frame.rows, rgb_frame.cols, CV_8UC3, rgb_frame.data.get());
    cvi_rgb.header.stamp = capture_time;
    cvi_rgb.header.frame_id = camera_parameters_.rgb_camera_frame_;
    // rgb_raw_frame->height = rgb_frame.rows;
    // rgb_raw_frame->width = rgb_frame.cols;
    cvi_rgb.encoding = "bgr8";
    // cvi_rgb.is_bigendian = false;
    cvi_rgb.image = cv_rgb;
    sensor_msgs::msg::Image im_rgb;
    cvi_rgb.toImageMsg(im_rgb);
    rgb_pub_->publish(im_rgb);
  }

  return 0;
}

int RosDevice::ir_frame_publish(const deptrum::stream::StreamFrame& ir_frame,
                                const builtin_interfaces::msg::Time& capture_time) {
  sensor_msgs::msg::Image::UniquePtr ir_raw_frame(new sensor_msgs::msg::Image());
  cv::Mat cv_ir(ir_frame.rows, ir_frame.cols, CV_8UC1, ir_frame.data.get());

  cv_bridge::CvImage cvi_ir;

  cvi_ir.header.stamp = capture_time;
  cvi_ir.header.frame_id = camera_parameters_.depth_camera_frame_;
  cvi_ir.encoding = "mono8";
  cvi_ir.image = cv_ir;
  sensor_msgs::msg::Image im_ir;
  cvi_ir.toImageMsg(im_ir);
  ir_pub_->publish(im_ir);
  return 0;
}
int RosDevice::depth_frame_publish(const deptrum::stream::StreamFrame& depth_frame,
                                   const builtin_interfaces::msg::Time& capture_time) {
  sensor_msgs::msg::Image::UniquePtr depth_raw_frame(new sensor_msgs::msg::Image());
  // int status = RgbFrameToRos(rgb_raw_frame, rgb_frame);
  // if (status != 0)
  //   return -1;
  cv::Mat cv_depth(depth_frame.rows, depth_frame.cols, CV_16UC1, depth_frame.data.get());

  cv_bridge::CvImage cvi_depth;

  cvi_depth.header.stamp = capture_time;
  cvi_depth.header.frame_id = camera_parameters_.depth_camera_frame_;
  cvi_depth.encoding = "mono16";
  cvi_depth.image = cv_depth;
  sensor_msgs::msg::Image im_depth;
  cvi_depth.toImageMsg(im_depth);
  depth_pub_->publish(im_depth);
  return 0;
}

std::string RosDevice::FrameModeToString(deptrum::FrameMode mode) {
  std::string mode_str = {};
  switch (mode) {
    case kInvalid:
      mode_str = "kInvalid";
      break;
    case kRes320x240RgbJpeg:
      mode_str = "kRes320x240RgbJpeg";
      break;
    case kRes400x256RgbJpeg:
      mode_str = "kRes400x256RgbJpeg";
      break;
    case kRes480x640RgbJpeg:
      mode_str = "kRes480x640RgbJpeg";
      break;
    case kRes640x480RgbJpeg:
      mode_str = "kRes640x480RgbJpeg";
      break;
    case kRes800x512RgbJpeg:
      mode_str = "kRes800x512RgbJpeg";
      break;
    case kRes960x1280RgbJpeg:
      mode_str = "kRes960x1280RgbJpeg";
      break;
    case kRes1280x960RgbJpeg:
      mode_str = "kRes1280x960RgbJpeg";
      break;
    case kRes1600x1080RgbJpeg:
      mode_str = "kRes1600x1080RgbJpeg";
      break;
    case kRes320x240Ir8Bit:
      mode_str = "kRes320x240Ir8Bit";
      break;
    case kRes400x640Ir8Bit:
      mode_str = "kRes400x640Ir8Bit";
      break;
    case kRes400x640Ir16Bit:
      mode_str = "kRes400x640Ir16Bit";
      break;
    case kRes480x640Ir8Bit:
      mode_str = "kRes480x640Ir8Bit";
      break;
    case kRes480x640Ir16Bit:
      mode_str = "kRes480x640Ir16Bit";
      break;
    case kRes640x480Ir8Bit:
      mode_str = "kRes640x480Ir8Bit";
      break;
    case kRes800x1280Ir8Bit:
      mode_str = "kRes800x1280Ir8Bit";
      break;
    case kRes800x1280Ir16Bit:
      mode_str = "kRes800x1280Ir16Bit";
      break;
    case kRes240x180Depth16Bit:
      mode_str = "kRes240x180Depth16Bit";
      break;
    case kRes640x480Depth16Bit:
      mode_str = "kRes640x480Depth16Bit";
      break;
    case kRes400x640Depth16Bit:
      mode_str = "kRes400x640Depth16Bit";
      break;
    case kRes480x640Depth16Bit:
      mode_str = "kRes480x640Depth16Bit";
      break;
    case kRes800x1280Depth16Bit:
      mode_str = "kRes800x1280Depth16Bit";
      break;
    case kRes640x400RgbYuv:
      mode_str = "kRes640x400RgbYuv";
      break;
    case kRes480x300RgbYuv:
      mode_str = "kRes480x300RgbYuv";
      break;
    case kRes320x200RgbYuv:
      mode_str = "kRes320x200RgbYuv";
      break;
    case kRes650x800Ir8Bit:
      mode_str = "kRes650x800Ir8Bit";
      break;
    case kRes736x480Ir8Bit:
      mode_str = "kRes736x480Ir8Bit";
      break;
    case kRes640x400Ir8Bit:
      mode_str = "kRes640x400Ir8Bit";
      break;
    case kRes480x300Ir8Bit:
      mode_str = "kRes480x300Depth16Bit";
      break;
    case kRes1080x1920RgbJpeg:
      mode_str = "kRes1080x1920RgbJpeg";
      break;
    case kRes320x200Ir8Bit:
      mode_str = "kRes320x200Ir8Bit";
      break;

    default:
      LOG(WARNING) << "unknown frame mode!!"
                   << "mode=" << (int) mode;
  }
  return mode_str;
}

std::string RosDevice::mat_type2encoding(int mat_type) {
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void RosDevice::TimestampToRos(const uint64_t& frame_timestamp_us,
                               builtin_interfaces::msg::Time& ros_time) {
  // double frame_timestamp_sec = static_cast<double>(frame_timestamp_ms) / 1000.0;
  ros_time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(frame_timestamp_us /
                                                                       1000000);
  ros_time.nanosec = frame_timestamp_us % 1000000 * 1000;
}

void RosDevice::PrintFPS() {
  static int count = 0;
  count++;
  static auto start_getframe = std::chrono::high_resolution_clock::now();

  if (count % 10 == 0) {
    auto end_getframe = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> tm = end_getframe - start_getframe;

    std::cout << "fps = " << 10000000 / tm.count() << std::endl;

    start_getframe = std::chrono::high_resolution_clock::now();
  }
}

void RosDevice::DeviceConnectCb(int flag, const deptrum::DeviceInformation& device_information) {
  if (1 == flag) {
    LOG(INFO) << "Device [model=" << device_information.model.c_str()
              << ",ir_camera.serial_num=" << device_information.ir_camera.serial_number.c_str()
              << "ir_camera.pid=" << device_information.ir_camera.pid << "]connected";
    if (device_ != nullptr) {
      LOG(INFO) << "[hot plug]reset device parms";
      if (!SetDeviceParams(device_)) {
        LOG(WARNING) << "[hot plug]set device params error";
      }
    }
    hot_plug_num++;
  } else if (2 == flag) {
    LOG(INFO) << "Device [model=" << device_information.model
              << ",ir_camera.serial_num=" << device_information.ir_camera.serial_number
              << "ir_camera.pid=" << device_information.ir_camera.pid << "]disconnected";
  } else {
    LOG(WARNING) << "not known (dis)connect flag, flag=" << flag;
  }
#ifdef SYSTEM_STATUS_TRANSFORM
  if (device_ != nullptr) {
    stellar_monitor_status_.device_information_ = device_information;
    stellar_monitor_status_.device_connect_status_ = flag;
    stellar_monitor_status_.depth_ir_time_out_num_ = 0;
    stellar_monitor_status_.point_cloud_time_out_num_ = 0;
    stellar_monitor_status_.StartPublishStatus();
  }
#endif
  return;
}

void RosDevice::UpgradeProgress(int process) {
  LOG(INFO) << "upgrade progress:" << process << "percent";
}

void RosDevice::FramePublisherThread() {
  int status;
  builtin_interfaces::msg::Time capture_time;

  sensor_msgs::msg::CameraInfo rgb_camera_info;
  camera_parameters_.GetRgbCameraInfo(rgb_camera_info);

  sensor_msgs::msg::CameraInfo ir_camera_info;
  camera_parameters_.GetIrCameraInfo(ir_camera_info);

  deptrum::stream::StreamFrames frames;
  rclcpp::WallRate loop_rate(params_.ir_fps);
  // sensor_msgs::PointCloud2 colored_msg;
  while (running_ && rclcpp::ok()) {
    status = streams_->GetFrames(frames, 2000);
    if (status != 0) {
      if (status == 0x42005) {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Get frames timeout!";
        continue;
      } else if (status == 0x2100c) {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Device disconnect!";
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        continue;
      } else {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Get frames error," << status;
        // rclcpp::shutdown();
        // exit(status);
        continue;
      }
    }

    for (int i = 0; i < frames.count; i++) {
      auto frame = frames.frame_ptr[i];
      TimestampToRos(frame->timestamp * 1000, capture_time);
      if (frame->frame_type == deptrum::FrameType::kRgbFrame && params_.rgb_enable) {
        status = rgb_frame_publish(*frame, capture_time);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get rgb frame";
          rclcpp::shutdown();
          exit(status);
        }
      }
      if (frame->frame_type == deptrum::FrameType::kDepthFrame && params_.depth_enable) {
        status = depth_frame_publish(*frame, capture_time);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get depth frame";
          rclcpp::shutdown();
          exit(status);
        }
      }
      if (frame->frame_type == deptrum::FrameType::kIrFrame && params_.ir_enable) {
        status = ir_frame_publish(*frame, capture_time);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get ir frame";
          rclcpp::shutdown();
          exit(status);
        }
      }
      if ((frame->frame_type == deptrum::FrameType::kRgbdPointCloudFrame ||
           frame->frame_type == deptrum::FrameType::kPointCloudFrame) &&
          params_.point_cloud_enable) {
        // TimestampToRos(frame->timestamp, capture_time);
        sensor_msgs::msg::PointCloud2 tmp_point_cloud;
        if (frame->frame_type == deptrum::FrameType::kPointCloudFrame) {
          status = PointCloudFrameToRos(tmp_point_cloud, *frame);
        } else if (frame->frame_type == deptrum::FrameType::kRgbdPointCloudFrame) {
          status = PointCloudFrameToColorRos(tmp_point_cloud, *frame);
        } else {
          continue;
        }
        if (status != 0) {
          LOG(ERROR) << "Failed to get point cloud frame";
          rclcpp::shutdown();
          exit(status);
        }
        tmp_point_cloud.header.stamp = capture_time;
        tmp_point_cloud.header.frame_id = camera_parameters_.depth_camera_frame_;
        pointcloud_publisher_->publish(tmp_point_cloud);
      }
    }

    if (params_.ir_enable || params_.depth_enable || params_.point_cloud_enable) {
      ir_camera_info.header.stamp = capture_time;
      ir_camerainfo_publisher_->publish(ir_camera_info);
    }

    if (params_.rgb_enable) {
      rgb_camera_info.header.stamp = capture_time;
      rgb_camerainfo_publisher_->publish(rgb_camera_info);
    }

    loop_rate.sleep();
  }
}

int RosDevice::PointCloudFrameToRos(sensor_msgs::msg::PointCloud2& point_cloud,
                                    const deptrum::stream::StreamFrame& point_frame) {
  point_cloud.height = point_frame.rows;
  point_cloud.width = point_frame.cols;

  point_cloud.is_dense = true;
  point_cloud.is_bigendian = false;

  const size_t point_count = point_frame.size / sizeof(PointXyz<float>);
  // LOG(INFO) << "point_count=" << point_count << ","
  //           << "point_frame.size=" << point_frame.size << ","
  //           << "sizeof(PointXyz<float>)=" << sizeof(PointXyz<float>);

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
  pcd_modifier.resize(point_count);
  deptrum::PointXyz<float>* current_point = static_cast<deptrum::PointXyz<float>*>(
      point_frame.data.get());
  deptrum::PointXyz<float> point;
  for (size_t i = 0; i < point_count; i++) {
    point = current_point[i];
    if (point.z <= 0.0f) {
      continue;
    } else {
      *iter_x = point.x / 1000;
      *iter_y = point.y / 1000;
      *iter_z = point.z / 1000;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }
  return 0;
}

int RosDevice::PointCloudFrameToColorRos(sensor_msgs::msg::PointCloud2& point_cloud,
                                         const deptrum::stream::StreamFrame& point_frame) {
  point_cloud.height = point_frame.rows;
  point_cloud.width = point_frame.cols;

  point_cloud.is_dense = true;
  point_cloud.is_bigendian = false;

  const size_t point_count = point_frame.size / sizeof(PointXyzRgb<float>);

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud, "b");
  pcd_modifier.resize(point_count);

  deptrum::PointXyzRgb<float>* current_point = static_cast<deptrum::PointXyzRgb<float>*>(
      point_frame.data.get());
  deptrum::PointXyzRgb<float> point;
  for (size_t i = 0; i < point_count; i++) {
    point = current_point[i];

    if (point.z <= 0.0f) {
      continue;
    } else {
      *iter_x = point.x / 1000;
      *iter_y = point.y / 1000;
      *iter_z = point.z / 1000;
      *iter_b = point.b;
      *iter_g = point.g;
      *iter_r = point.r;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
  }
  return 0;
}
