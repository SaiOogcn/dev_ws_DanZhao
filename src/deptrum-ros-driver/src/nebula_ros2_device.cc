#include <sstream>
#include "deptrum_ros_driver/nebula_ros2_device.h"
using namespace std;
using namespace deptrum;
nebulaRosDevice::nebulaRosDevice(std::shared_ptr<rclcpp::Node> node) : RosDevice(node) {
  if (params_.rgb_enable && (params_.slam_mode == 1 || params_.slam_mode == 0))
    rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("mtof_rgb/image_raw", 1);
  if (params_.rgb_enable && (params_.slam_mode == 2 || params_.slam_mode == 0))
    stof_rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("stof_rgb/image_raw", 1);
  if (params_.ext_rgb_mode != 0)
    ext_rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("ext_rgb/image_raw", 1);
  if (params_.enable_imu)
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);

  if (params_.rgb_enable) {
    rgb_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "rgb/camera_info",
        1);
  }
  if (params_.depth_enable && (params_.slam_mode == 1 || params_.slam_mode == 0)) {
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("mtof_depth/image_raw", 1);
  }
  if (params_.depth_enable && (params_.slam_mode == 2 || params_.slam_mode == 0)) {
    stof_depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("stof_depth/image_raw", 1);
  }

  rd_depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("rd_depth/image_raw", 1);
  rmd_depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("rmd_depth/image_raw", 1);

  if (params_.ir_enable &&
      (params_.slam_mode == 2 || params_.slam_mode == 0))  // 0-mtof and stof 1-motf 2-stof
  {
    ir_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("ir/image_raw", 1);
  }

  if (params_.point_cloud_enable && (params_.slam_mode == 1 || params_.slam_mode == 0)) {
    pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("mtof_points2",
                                                                                   1);
  }
  if (params_.point_cloud_enable && (params_.slam_mode == 2 || params_.slam_mode == 0)) {
    stof_pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "stof_points2",
        1);
  }
  if (params_.flag_enable &&
      (params_.slam_mode == 2 || params_.slam_mode == 0))  // 0-mtof and stof 1-motf 2-stof
  {
    flag_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("flag/image_raw", 1);
  }

  if (params_.depth_enable || params_.ir_enable || params_.point_cloud_enable) {
    ir_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "ir/camera_info",
        1);
  }
  sn_pub_ = node_->create_publisher<std_msgs::msg::String>("serial_number", 1);
}
nebulaRosDevice::~nebulaRosDevice() {
  Stop();
}
int nebulaRosDevice::Start() {
  if (running_ == true) {
    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Device already started";
    return 0;
  }

  if (device_ != nullptr) {
    camera_parameters_.Initialize(device_, params_.boot_order);
  }

  running_ = true;
  int status = 0;
  sn_publisher_thread_ = thread(&nebulaRosDevice::SerialNumberPublisherThread, this);
  std::vector<deptrum::stream::StreamType> types;
  if (params_.rgbd_enable) {
    types.push_back(deptrum::stream::StreamType::kRgbd);
    if (params_.point_cloud_enable) {
      types.push_back(deptrum::stream::StreamType::kRgbdIrPointCloud);
    }
  } else {
    if (params_.depth_enable || params_.ir_enable || params_.rgb_enable) {
      types.push_back(deptrum::stream::StreamType::kRgbdIrFlag);
    }
    if (params_.point_cloud_enable) {
      types.push_back(deptrum::stream::StreamType::kPointCloud);
    }
  }

  if (types.size() != 0) {
    status = device_->CreateStream(streams_, types);
    if (0 != status) {
      LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Create stream failed, error code "
                 << status;
      return status;
    }
    status = streams_->Start();
    if (0 != status) {
      LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Start stream failed, error code "
                 << status;
      return status;
    }
    frame_publisher_thread_ = thread(&nebulaRosDevice::FramePublisherThread, this);
  }

  return 0;
}

void nebulaRosDevice::Stop() {
  LOG(INFO) << "begin stop()";
  if (running_ == false) {
    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Cameras not started";
    return;
  }
  running_ = false;
  if (device_ != nullptr) {
    // if (detector_thread_.joinable())
    // {
    //   detector_thread_.join();
    // }
    if (frame_publisher_thread_.joinable()) {
      frame_publisher_thread_.join();
    }

    if (sn_publisher_thread_.joinable()) {
      sn_publisher_thread_.join();
    }

    if (streams_ != nullptr) {
      LOG(INFO) << "stop stream";
      streams_->Stop();
      LOG(INFO) << "destroy stream";
      device_->DestroyStream(streams_);
      streams_ = nullptr;
    }
  }
  LOG(INFO) << "close glog";
  google::ShutdownGoogleLogging();
}

void nebulaRosDevice::InitDevice() {
  int status = 0;
  std::vector<deptrum::DeviceInformation> device_list;
  status = deptrum::stream::DeviceManager::GetInstance()->GetDeviceList(device_list);
  if (status != 0) {
    LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::GetDeviceList failed, error code"
               << status;
  }
  int device_count = device_list.size();
  std::cout << "device_count==========" << device_count << std::endl;
  if (device_count == 0) {
    // 等待设备插入
    unsigned int wait_num = 0;
    while (1) {
      wait_num++;
      if (hot_plug_num == 0) {
        LOG(INFO) << "wait device insert...";
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else {
        status = deptrum::stream::DeviceManager::GetInstance()->GetDeviceList(device_list);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::GetDeviceList failed, error code "
                     << status;
        }
        if (device_list.size() > 0) {
          LOG(INFO) << "has device insert";
          device_count = device_list.size();
          break;
        }
      }
      if (wait_num == 30) {
        LOG(ERROR) << "device quit force with timeout......";
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::No deptrum device connected! It's going to quit...";
        rclcpp::shutdown();
        exit(-1);
      }
    }
  }
  LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Found " << device_count
            << " deptrum device";
  if (device_ != nullptr) {
    LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::device_ is already init.";
  }

  if (params_.serial_number != "") {
    LOG(INFO) << "Searching for device with serial number: " << params_.serial_number;
  } else {
    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order
              << "::No serial number provided: picking first device";
    // ROS_INFO_COND(device_count > 1,
    //               "BOOT_ORDER_%d::Multiple device connected! Openning first device.",
    //               params_.boot_order);
  }

  if (params_.usb_port_number.size()) {
    for (int i = 0; i < device_count; i++) {
      LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::" << device_count
                << "deptrum device ir usb port" << device_list[i].ir_camera.port_path;
    }
    device_count = 1;
  }

  int i = 0;
  for (; i < device_count; i++) {
    std::shared_ptr<deptrum::stream::Device> device = nullptr;
    if (params_.usb_port_number.size()) {
      LOG(INFO) << "BOOT_ORDER_" << params_.boot_order
                << "::USB port number:" << params_.usb_port_number;
      device = deptrum::stream::DeviceManager::GetInstance()->CreateDeviceByUsbPort(
          params_.usb_port_number);
    } else {
      device = deptrum::stream::DeviceManager::GetInstance()->CreateDevice(device_list[i]);
    }
    if (device == nullptr) {
      LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to create device";

      rclcpp::shutdown();
      exit(-1);
    }
    if (params_.stream_sdk_log_enable) {
      LOG(INFO) << "open sdk log";
      deptrum::stream::DeviceManager::EnableLogging(params_.log_dir + "deptrum-stream.log",
                                                    params_.stream_sdk_log_enable);
    }

    deptrum::FrameMode ir_mode{deptrum::FrameMode::kInvalid};
    deptrum::FrameMode rgb_mode{deptrum::FrameMode::kInvalid};
    deptrum::FrameMode depth_mode{deptrum::FrameMode::kInvalid};
    deptrum::FrameDecodeMode ir_decode_mode{deptrum::FrameDecodeMode::kSoftwareDecode};
    deptrum::FrameDecodeMode rgb_decode_mode{deptrum::FrameDecodeMode::kSoftwareDecode};

    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::" << i << ":device start opening";
    status = device->Open();
    if (status != 0) {
      device->Close();
      LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::open failed, error code " << status;
      continue;
    }
    // after open device

    deptrum::FrameMode ir_mode_ref, rgb_mode_ref, depth_mode_ref;
    int num = 0;
    std::vector<std::tuple<deptrum::FrameMode, deptrum::FrameMode, deptrum::FrameMode>>
        device_resolution_vec;
    status = device->GetSupportedFrameMode(device_resolution_vec);
    if (status != 0) {
      LOG(ERROR) << "GetSupportedFrameMode error";
    }
    if (device_resolution_vec.empty()) {
      device->Close();
      LOG(ERROR) << "GetSupportedFrameMode is empty";
      rclcpp::shutdown();
      exit(-1);
    }
    LOG(INFO) << "device supported frame_mode:";
    for (auto device_resolution : device_resolution_vec) {
      ir_mode_ref = std::get<0>(device_resolution);
      rgb_mode_ref = std::get<1>(device_resolution);
      depth_mode_ref = std::get<2>(device_resolution);
      LOG(INFO) << "[" << num++ << "]:"
                << "ir_mode:=" << int(ir_mode_ref) << " " << FrameModeToString(ir_mode_ref)
                << " rgb_mode:=" << int(rgb_mode_ref) << " " << FrameModeToString(rgb_mode_ref)
                << " depth_mode:=" << int(depth_mode_ref) << " "
                << FrameModeToString(depth_mode_ref).c_str();
    }
    /*400device rgb version 2.2.0->resolution_mode_index->2   2.1.0->resolution_mode_index->0*/
    //
    dynamic_pointer_cast<deptrum::stream::Nebula>(device)->GetDeviceInfo(device_information_);
    // if ("2.2.0.0" == device_information_.rgb_firmware_version) {
    //   LOG(WARNING) << "检测到rgb firmware version
    //   is 2.2.0.0,rgb分辨率强制采用kRes640x480RgbJpeg"; params_.resolution_mode_index = 2;
    // }
    ir_mode = std::get<0>(device_resolution_vec[params_.resolution_mode_index]);
    rgb_mode = std::get<1>(device_resolution_vec[params_.resolution_mode_index]);
    LOG(INFO) << "current set mode "
              << "ir_mode:" << (int) ir_mode << ":" << FrameModeToString(ir_mode)
              << ",rgb_mode:" << (int) rgb_mode << ":" << FrameModeToString(rgb_mode);
    if (params_.output_compressed_rgb) {
      rgb_decode_mode = deptrum::FrameDecodeMode::kNoDecode;
    }

    status = device->SetMode(ir_mode, rgb_mode, depth_mode);
    if (status != 0) {
      LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::SetMode failed, error code "
                 << status;
      continue;
    }
    if (!SetDeviceParams(device)) {
      LOG(WARNING) << "set device params error";
    }

    status = dynamic_pointer_cast<deptrum::stream::Nebula>(device)->GetDeviceInfo(
        device_information_);
    if (status != 0) {
      device->Close();
      LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << ":GetDeviceInfo failed, error code "
                   << status;
      continue;
    } else if (params_.serial_number == "" ||
               device_information_.serial_num == params_.serial_number) {
      LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Device information: ";
      LOG(INFO) << "\t sdk version: " << device_information_.stream_sdk_version;
      LOG(INFO) << "\t device name: " << device_information_.device_name;
      LOG(INFO) << "\t serial number: " << device_information_.serial_num;
      LOG(INFO) << "\t tof firmware version: " << device_information_.ir_firmware_version;
      LOG(INFO) << "\t rgb firmware version: " << device_information_.rgb_firmware_version;

      if (device_ != nullptr) {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Device Number: " << i
                     << ", device_ is already init.";
      }
      device_ = device;
      break;
    } else {
      device->Close();
      continue;
    }
  }

  if (!device_) {
    LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::"
               << "All " << device_count
               << "devices have attempted to connect, Failed to open device ";
    rclcpp::shutdown();
    ;
    exit(-1);
  }
}

bool nebulaRosDevice::SetDeviceParams(std::shared_ptr<deptrum::stream::Device>& device) {
  int status = 0;
  deptrum::stream::HeartbeatParam heartbeat_param;
#ifdef DEVICE_INTERNAL
  status = dynamic_pointer_cast<deptrum::stream::NebulaImpl>(device)->SetExposureMaxValue(5000);
  if (status != 0) {
    LOG(WARNING) << "SetExposureMaxValue failed, error code " << status;
    return false;
  }
#endif

  if (params_.update_file_path != "") {
    status = dynamic_pointer_cast<deptrum::stream::Nebula>(device)->Upgrade(
        params_.update_file_path,
        upgrade_handle_);
    if (status != 0) {
      LOG(WARNING) << "update firmware failed, error code " << status;
      exit(-1);
    } else {
      LOG(INFO) << "update firmware succeed";
      rclcpp::shutdown();
      exit(0);
    }
  }
  if (params_.update_file_path != "") {
    LOG(WARNING) << "launch file update_file_path is not null";
    rclcpp::shutdown();
    exit(0);
  }

  if (params_.exposure_enable) {
    dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SwitchAutoExposure(false);
    status = dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetExposure(
        kLed,
        params_.exposure_time);
    if (status != 0) {
      LOG(WARNING) << "SetExposure failed, error code " << status;
      return false;
    }
  } else {
    dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SwitchAutoExposure(true);
  }

  status = dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SwitchMtofOrStof(
      params_.slam_mode);
  if (status != 0) {
    LOG(WARNING) << "SwitchMtofOrStof failed, error code " << status;
    return false;
  }
  // dynamic_pointer_cast<deptrum::stream::Nebula>(device)->CloseDecodeJpegMode();

  status = dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetStofDepthRange(
      params_.stof_minimum_range);
  if (status != 0) {
    LOG(WARNING) << "SetStofDepthRange failed, error code " << status;
    return false;
  } else {
    LOG(INFO) << "set stof depth range  succeed " << params_.stof_minimum_range;
  }

  deptrum::stream::DeptrumNebulaFilterLevel mtof_level;
  deptrum::stream::DeptrumNebulaFilterLevel stof_level;
  if (params_.mtof_filter_level == 0) {
    mtof_level = deptrum::stream::kLowLevel;
  } else if (params_.mtof_filter_level == 1) {
    mtof_level = deptrum::stream::kMiddleLevel;
  } else {
    mtof_level = deptrum::stream::kHighLevel;
  }

  if (params_.stof_filter_level == 0) {
    stof_level = deptrum::stream::kLowLevel;
  } else if (params_.stof_filter_level == 1) {
    stof_level = deptrum::stream::kMiddleLevel;
  } else {
    stof_level = deptrum::stream::kHighLevel;
  }

  status = dynamic_pointer_cast<deptrum::stream::Nebula>(device)->ChangeFilterLevel(
      mtof_level,
      deptrum::stream::DeptrumNebulaFrameType::kFrameMtof);
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  status = dynamic_pointer_cast<deptrum::stream::Nebula>(device)->ChangeFilterLevel(
      stof_level,
      deptrum::stream::DeptrumNebulaFrameType::kFrameStof);
  if (status != 0) {
    LOG(WARNING) << "ChangeFilterLevel failed, error code " << status;
    return false;
  }

  dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetMtofCropPixels(params_.mtof_crop_up,
                                                                           params_.mtof_crop_down);
  if (status != 0) {
    LOG(WARNING) << "SetMtofCropPixels failed, error code " << status;
    // return false;
  }

  LOG(INFO) << "SetExtRgbMode mode: " << params_.ext_rgb_mode;
  dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetExtRgbMode(params_.ext_rgb_mode);
  if (status != 0) {
    LOG(WARNING) << "SetExtRgbMode failed, error code " << status;
    // return false;
  }

  LOG(INFO) << "imu mode: " << params_.enable_imu;
  if (params_.enable_imu) {
    dynamic_pointer_cast<deptrum::stream::Nebula>(device)->RegisterImuDataHandler(
        [this](const deptrum::stream::OpaqueData<uint8_t>& imu) { imu_frame_publish(imu); });
    if (status != 0) {
      LOG(WARNING) << "RegisterImuDataHandler failed, error code " << status;
      // return false;
    }
  }

  dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetPositionDataScaled(
      deptrum::stream::PointScale::kPointScaleMm);
  dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetPositionDataFeature(
      deptrum::stream::PointFeature::kPointFeatureActualPoint);

  // status =
  // dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetFrequencyFusionThreshold(
  //     params_.depth_th);
  // if (status != 0) {
  //   LOG(WARNING) << "SetFrequencyFusionThreshold failed, error code " << status;
  //   return false;
  // }
  // float depth_th = 0;
  // dynamic_pointer_cast<deptrum::stream::Nebula>(device)->GetFrequencyFusionThreshold(depth_th);
  // LOG(INFO) << "current GetFrequencyFusionThreshold depth value = " << depth_th;
  // status =
  // dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetFrequencyFusionThreshold(
  //     (uint16_t) params_.ir_th);
  // if (status != 0) {
  //   LOG(WARNING) << "SetFrequencyFusionThreshold failed, error code " << status;
  //   return false;
  // }
  // uint16_t ir_th = 0;
  // dynamic_pointer_cast<deptrum::stream::Nebula>(device)->GetFrequencyFusionThreshold(ir_th);
  // LOG(INFO) << "current GetFrequencyFusionThreshold ir value = " << ir_th;
  // status =
  // dynamic_pointer_cast<deptrum::stream::Nebula>(device)->SetScatterFilterThreshold(
  //     params_.ratio_th);
  // if (status != 0) {
  //   LOG(WARNING) << "SetScatterFilterThreshold failed, error code " << status;
  //   return false;
  // }
  // float ratio_th;
  // dynamic_pointer_cast<deptrum::stream::Nebula>(device)->GetScatterFilterThreshold(ratio_th);
  // LOG(INFO) << "current GetScatterFilterThreshold ratio value = " << ratio_th;
  return true;
}

int nebulaRosDevice::PointCloudFrameToRos(sensor_msgs::msg::PointCloud2& point_cloud,
                                          const deptrum::stream::StreamFrame& point_frame) {
  point_cloud.height = point_frame.rows;
  point_cloud.width = point_frame.cols;

  point_cloud.is_dense = true;
  point_cloud.is_bigendian = false;

  const size_t point_count = point_frame.size / sizeof(PointXyz<float>);

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

int nebulaRosDevice::PointCloudFrameToColorRos(sensor_msgs::msg::PointCloud2& point_cloud,
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

void nebulaRosDevice::FramePublisherThread() {
  int status;
  builtin_interfaces::msg::Time capture_time;
  // static int rgb_time_out_num = 0;
  sensor_msgs::msg::CameraInfo ir_camera_info;
  camera_parameters_.GetIrCameraInfo(ir_camera_info);

  deptrum::stream::StreamFrames frames;
  rclcpp::WallRate loop_rate(params_.ir_fps);
  while (running_ && rclcpp::ok()) {
    status = streams_->GetFrames(frames, 2000);
    if (status != 0) {
      if (status == 0x42005) {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Get frame timeout!";
        continue;
      } else if (status == 0x2100c) {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Device disconnect!";
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        continue;
      } else {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Get frame error!" << status;
        // rclcpp::shutdown();
        // exit(status);
        continue;
      }
    }

    auto now = std::chrono::system_clock::now();
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch())
                      .count();
    TimestampToRos((uint64_t) now_us, capture_time);

    for (int i = 0; i < frames.count; i++) {
      auto frame = frames.frame_ptr[i];
      if (frame->frame_type == deptrum::FrameType::kDepthFrame && params_.depth_enable) {
        if ((frame->frame_attr & deptrum::FrameAttr::kAttrMtof) != 0) {
          status = depth_frame_publish(*frame, capture_time);
        } else if ((frame->frame_attr & deptrum::FrameAttr::kAttrStof) != 0) {
          status = stof_depth_frame_publish(*frame, capture_time);
        }
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get depth frame";
          rclcpp::shutdown();
          exit(status);
        }
      } else if (frame->frame_type == deptrum::FrameType::kRgbFrame && params_.rgb_enable) {
        if ((frame->frame_attr & deptrum::FrameAttr::kAttrMtof) != 0) {
          status = rgb_frame_publish(*frame, capture_time);
        } else if ((frame->frame_attr & deptrum::FrameAttr::kAttrStof) != 0) {
          status = stof_rgb_frame_publish(*frame, capture_time);
        }
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get rgb frame";
          rclcpp::shutdown();
          exit(status);
        }
      } else if (frame->frame_type == deptrum::FrameType::kIrFrame && params_.ir_enable &&
                 (params_.slam_mode == 2 || params_.slam_mode == 0)) {
        status = ir_frame_publish(*frame, capture_time);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get ir frame";
          rclcpp::shutdown();
          exit(status);
        }
      } else if (frame->frame_type == deptrum::FrameType::kSemanticFrame && params_.flag_enable &&
                 (params_.slam_mode == 2 || params_.slam_mode == 0)) {
        status = flag_frame_publish(*frame, capture_time);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get flag frame";
          rclcpp::shutdown();
          exit(status);
        }
      }
      if (params_.point_cloud_enable &&
          (frame->frame_type == deptrum::FrameType::kPointCloudFrame ||
           frame->frame_type == deptrum::FrameType::kRgbdPointCloudFrame)) {
        sensor_msgs::msg::PointCloud2 tmp_point_cloud;
        if (frame->frame_type == deptrum::FrameType::kPointCloudFrame) {
          status = PointCloudFrameToRos(tmp_point_cloud, *frame);
        } else {
          status = PointCloudFrameToColorRos(tmp_point_cloud, *frame);
        }

        if (status != 0) {
          LOG(ERROR) << "Failed to get point cloud frame";
          rclcpp::shutdown();
          exit(status);
        }

        tmp_point_cloud.header.stamp = capture_time;
        tmp_point_cloud.header.frame_id = camera_parameters_.depth_camera_frame_;
        if ((frame->frame_attr & deptrum::FrameAttr::kAttrMtof) != 0) {
          pointcloud_publisher_->publish(tmp_point_cloud);
        } else {
          stof_pointcloud_publisher_->publish(tmp_point_cloud);
        }
      }
    }

    if (frames.extra_info != nullptr) {
      for (const auto& debug_frame : frames.extra_info->debug_frames) {
        // LOG(INFO) << "debug_frame.name: " << debug_frame.name;
        if (std::string(debug_frame.name) == "rgb_1") {
          status = ext_rgb_frame_publish(debug_frame, capture_time);
          if (status != 0) {
            LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                       << "::Failed to publish ext rgb frame";
            rclcpp::shutdown();
            exit(status);
          }
        }
        if (std::string(debug_frame.name) == "rd") {
          status = rd_frame_publish(debug_frame, capture_time);
          if (status != 0) {
            LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                       << "::Failed to publish ext rgb frame";
            rclcpp::shutdown();
            exit(status);
          }
        }
        if (std::string(debug_frame.name) == "rmd") {
          status = rmd_frame_publish(debug_frame, capture_time);
          if (status != 0) {
            LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                       << "::Failed to publish ext rgb frame";
            rclcpp::shutdown();
            exit(status);
          }
        }
      }
    }

    loop_rate.sleep();
  }
  LOG(INFO) << "ending stream";
}

void nebulaRosDevice::SerialNumberPublisherThread() {
  while (running_ && rclcpp::ok()) {
    std_msgs::msg::String sn;
    sn.data = device_information_.serial_num;
    sn_pub_->publish(sn);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  sn_pub_.reset();
  rclcpp::shutdown();
}

int nebulaRosDevice::stof_depth_frame_publish(const deptrum::stream::StreamFrame& depth_frame,
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
  stof_depth_pub_->publish(im_depth);
  return 0;
}

int nebulaRosDevice::stof_rgb_frame_publish(const deptrum::stream::StreamFrame& rgb_frame,
                                            const builtin_interfaces::msg::Time& capture_time) {
  sensor_msgs::msg::Image::UniquePtr rgb_raw_frame(new sensor_msgs::msg::Image());
  cv_bridge::CvImage cvi_rgb;
  cv::Mat cv_rgb(rgb_frame.rows, rgb_frame.cols, CV_8UC3, rgb_frame.data.get());
  cvi_rgb.header.stamp = capture_time;
  cvi_rgb.header.frame_id = camera_parameters_.rgb_camera_frame_;
  cvi_rgb.encoding = "bgr8";
  cvi_rgb.image = cv_rgb;
  sensor_msgs::msg::Image im_rgb;
  cvi_rgb.toImageMsg(im_rgb);
  stof_rgb_pub_->publish(im_rgb);

  return 0;
}

int nebulaRosDevice::flag_frame_publish(const deptrum::stream::StreamFrame& flag_frame,
                                        const builtin_interfaces::msg::Time& capture_time) {
  sensor_msgs::msg::Image::UniquePtr rgb_raw_frame(new sensor_msgs::msg::Image());
  cv_bridge::CvImage cvi_rgb;
  cv::Mat cv_rgb(flag_frame.rows, flag_frame.cols, CV_8UC3, flag_frame.data.get());
  cvi_rgb.header.stamp = capture_time;
  cvi_rgb.header.frame_id = camera_parameters_.rgb_camera_frame_;
  cvi_rgb.encoding = "bgr8";
  cvi_rgb.image = cv_rgb;
  sensor_msgs::msg::Image im_rgb;
  cvi_rgb.toImageMsg(im_rgb);
  flag_pub_->publish(im_rgb);

  return 0;
}

int nebulaRosDevice::ext_rgb_frame_publish(const deptrum::stream::CustomFrame<uint8_t>& rgb_frame,
                                           const builtin_interfaces::msg::Time& capture_time) {
  sensor_msgs::msg::Image::UniquePtr rgb_raw_frame(new sensor_msgs::msg::Image());
  // int status = RgbFrameToRos(rgb_raw_frame, rgb_frame);
  // if (status != 0)
  //   return -1;

  cv_bridge::CvImage cvi_rgb;
  cv::Mat cv_rgb(rgb_frame.height, rgb_frame.width, CV_8UC3, rgb_frame.data.get());
  cvi_rgb.header.stamp = capture_time;
  cvi_rgb.header.frame_id = camera_parameters_.rgb_camera_frame_;
  // rgb_raw_frame->height = rgb_frame.rows;
  // rgb_raw_frame->width = rgb_frame.cols;
  cvi_rgb.encoding = "bgr8";
  // cvi_rgb.is_bigendian = false;
  cvi_rgb.image = cv_rgb;
  sensor_msgs::msg::Image im_rgb;
  cvi_rgb.toImageMsg(im_rgb);
  ext_rgb_pub_->publish(im_rgb);

  return 0;
}

int nebulaRosDevice::rd_frame_publish(const deptrum::stream::CustomFrame<uint8_t>& rd_frame,
                                      const builtin_interfaces::msg::Time& capture_time) {
  cv::Mat cv_depth(rd_frame.height, rd_frame.width, CV_16UC1, rd_frame.data.get());

  cv_bridge::CvImage cvi_depth;

  cvi_depth.header.stamp = capture_time;
  cvi_depth.header.frame_id = camera_parameters_.depth_camera_frame_;
  cvi_depth.encoding = "mono16";
  cvi_depth.image = cv_depth;
  sensor_msgs::msg::Image im_depth;
  cvi_depth.toImageMsg(im_depth);
  rd_depth_pub_->publish(im_depth);
  return 0;
}

int nebulaRosDevice::rmd_frame_publish(const deptrum::stream::CustomFrame<uint8_t>& rmd_frame,
                                       const builtin_interfaces::msg::Time& capture_time) {
  cv::Mat cv_depth(rmd_frame.height, rmd_frame.width, CV_16UC1, rmd_frame.data.get());

  cv_bridge::CvImage cvi_depth;

  cvi_depth.header.stamp = capture_time;
  cvi_depth.header.frame_id = camera_parameters_.depth_camera_frame_;
  cvi_depth.encoding = "mono16";
  cvi_depth.image = cv_depth;
  sensor_msgs::msg::Image im_depth;
  cvi_depth.toImageMsg(im_depth);
  rmd_depth_pub_->publish(im_depth);
  return 0;
}

int nebulaRosDevice::imu_frame_publish(const deptrum::stream::OpaqueData<uint8_t>& imu_frame) {
  if (imu_frame.data_len != sizeof(deptrum::stream::ImuData)) {
    LOG(ERROR) << "imu frame size error";
    return -1;
  }
  deptrum::stream::ImuData* imu_data = (deptrum::stream::ImuData*) imu_frame.data.get();
  builtin_interfaces::msg::Time capture_time;
  TimestampToRos(imu_data->timestamp, capture_time);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = capture_time;
  imu_msg.header.frame_id = camera_parameters_.imu_frame_;

  // 加速度（单位：m/s²）
  imu_msg.linear_acceleration.x = imu_data->accel_x;
  imu_msg.linear_acceleration.y = imu_data->accel_y;
  imu_msg.linear_acceleration.z = imu_data->accel_z;

  // 角速度（单位：rad/s）
  imu_msg.angular_velocity.x = imu_data->gyro_x;
  imu_msg.angular_velocity.y = imu_data->gyro_y;
  imu_msg.angular_velocity.z = imu_data->gyro_z;

  // 协方差矩阵（若未提供则置零或标记无效）
  imu_msg.orientation_covariance[0] = -1;  // 示例：标记无效

  imu_pub_->publish(imu_msg);

  return 0;
}