#include <sstream>
#include "deptrum_ros_driver/stellar400_ros2_device.h"
using namespace std;
using namespace deptrum;
Stellar400RosDevice::Stellar400RosDevice(std::shared_ptr<rclcpp::Node> node) : RosDevice(node) {
  if (params_.rgb_enable) {
    rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", 1);
    rgb_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "rgb/camera_info",
        1);
  }
  if (params_.depth_enable) {
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 1);
  }
  if (params_.ir_enable) {
    ir_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("ir/image_raw", 1);
  }
  if (params_.point_cloud_enable) {
    pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1);
  }
  if (params_.depth_enable || params_.ir_enable || params_.point_cloud_enable) {
    ir_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "ir/camera_info",
        1);
  }
  camera_version_publisher_ = node_->create_publisher<std_msgs::msg::String>("camera_version", 1);
  heart_beat_handle_ = std::bind(&Stellar400RosDevice::HeartBeatCb, this, std::placeholders::_1);
}
Stellar400RosDevice::~Stellar400RosDevice() {
  Stop();
}
int Stellar400RosDevice::Start() {
  if (running_ == true) {
    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Device already started";
    return 0;
  }

  if (device_ != nullptr) {
    camera_parameters_.Initialize(device_, params_.boot_order);
  }

  running_ = true;

  int status = 0;
  if (params_.rgbd_enable) {
    status = device_->CreateStream(streams_.rgbd, {deptrum::stream::StreamType::kRgbdIr});
    if (0 != status) {
      LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                 << "::Create kRgbd stream failed, error code " << status;
      return status;
    }
    status = streams_.rgbd->Start();
    if (0 != status) {
      LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                 << "::Start kRgbd stream failed, error code " << status;
      return status;
    }
    rgbd_frame_publisher_thread_ = thread(&RosDevice::RgbdFramePublisherThread, this);

    if (params_.point_cloud_enable) {
      status = device_->CreateStream(streams_.point_cloud,
                                     {deptrum::stream::StreamType::kRgbdIrPointCloud});
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Create kPointDense stream failed, error code " << status;
        return status;
      }
      status = streams_.point_cloud->Start();
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Start kPointDense stream failed, error code " << status;
        return status;
      }

      point_cloud_frame_publisher_thread_ = thread(
          &Stellar400RosDevice::PointCloudFramePublisherThread,
          this);
    }
  } else {
    if (params_.rgb_enable) {
      status = device_->CreateStream(streams_.rgb, {deptrum::stream::StreamType::kRgb});
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Create kRgb stream failed, error code " << status;
        return status;
      }
      status = streams_.rgb->Start();
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Start kRgb stream failed, error code " << status;
        return status;
      }
      rgb_frame_publisher_thread_ = thread(&RosDevice::RgbFramePublisherThread, this);
    }
    if (params_.depth_enable || params_.ir_enable) {
      status = device_->CreateStream(streams_.depth_ir, {deptrum::stream::StreamType::kDepthIr});
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Create kDepthIr stream failed, error code " << status;
        return status;
      }
      status = streams_.depth_ir->Start();
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Start kDepthIr stream failed, error code " << status;
        return status;
      }
      depthir_frame_publisher_thread_ = thread(&RosDevice::DepthIrFramePublisherThread, this);
    }
    if (params_.point_cloud_enable) {
      status = device_->CreateStream(streams_.point_cloud,
                                     {deptrum::stream::StreamType::kPointCloud});
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Create kPointDense stream failed, error code " << status;
        return status;
      }
      status = streams_.point_cloud->Start();
      if (0 != status) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order
                   << "::Start kPointDense stream failed, error code " << status;
        return status;
      }

      point_cloud_frame_publisher_thread_ = thread(
          &Stellar400RosDevice::PointCloudFramePublisherThread,
          this);
    }
  }
  return 0;
}

void Stellar400RosDevice::Stop() {
  LOG(INFO) << "begin stop()";
  if (running_ == false) {
    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Cameras not started";
    return;
  }
  running_ = false;
  if (device_ != nullptr) {
    if (params_.heart_enable) {
      LOG(INFO) << "close heart ...";
      dynamic_pointer_cast<deptrum::stream::Stellar400>(device_)->StopHeartbeat();
    }
    // if (detector_thread_.joinable())
    // {
    //   detector_thread_.join();
    // }
    if (rgb_frame_publisher_thread_.joinable()) {
      rgb_frame_publisher_thread_.join();
    }
    if (depthir_frame_publisher_thread_.joinable()) {
      depthir_frame_publisher_thread_.join();
    }

    if (rgbd_frame_publisher_thread_.joinable()) {
      rgbd_frame_publisher_thread_.join();
    }

    if (point_cloud_frame_publisher_thread_.joinable()) {
      point_cloud_frame_publisher_thread_.join();
    }

    if (params_.rgbd_enable) {
      if (streams_.rgbd != nullptr) {
        LOG(INFO) << "stop rgbd stream";
        streams_.rgbd->Stop();
        LOG(INFO) << "destroy rgbd stream";
        device_->DestroyStream(streams_.rgbd);
        streams_.rgbd = nullptr;
      }

      if (params_.point_cloud_enable) {
        if (streams_.point_cloud != nullptr) {
          LOG(INFO) << "stop point_cloud stream";
          streams_.point_cloud->Stop();
          LOG(INFO) << "destroy point_cloud stream";
          device_->DestroyStream(streams_.point_cloud);
          streams_.point_cloud = nullptr;
        }
      }
    } else {
      if (params_.rgb_enable) {
        if (streams_.rgb != nullptr) {
          LOG(INFO) << "stop rgb stream";
          streams_.rgb->Stop();
          LOG(INFO) << "destroy rgb stream";
          device_->DestroyStream(streams_.rgb);
          streams_.rgb = nullptr;
        }
      }

      if (params_.depth_enable || params_.ir_enable) {
        if (streams_.depth_ir != nullptr) {
          LOG(INFO) << "stop depth_ir stream";
          streams_.depth_ir->Stop();
          LOG(INFO) << "destroy depth_ir stream";
          device_->DestroyStream(streams_.depth_ir);
          streams_.depth_ir = nullptr;
        }
      }

      if (params_.point_cloud_enable) {
        if (streams_.point_cloud != nullptr) {
          LOG(INFO) << "stop point cloud stream";
          streams_.point_cloud->Stop();
          LOG(INFO) << "destroy point cloud stream";
          device_->DestroyStream(streams_.point_cloud);
          streams_.point_cloud = nullptr;
        }
      }
    }
  }
  LOG(INFO) << "close glog";
  google::ShutdownGoogleLogging();
}

void Stellar400RosDevice::InitDevice() {
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
    for (int i = 1; i < 4; i++) {
      // ValidateFps(params_.ir_fps);
      status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetIrFps(params_.ir_fps);
      if (status != 0) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Set Ir fps failed, error code "
                   << status << " num: " << i;
        continue;
      }

      // ValidateFps(params_.rgb_fps);
      status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetRgbFps(
          params_.rgb_fps);
      if (status != 0) {
        LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Set Rgb fps failed, error code "
                   << status << " num: " << i;
        ;
        continue;
      }
      break;
    }

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
    dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetDeviceInfo(device_information_);
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

    status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetDeviceInfo(
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

bool Stellar400RosDevice::SetDeviceParams(std::shared_ptr<deptrum::stream::Device>& device) {
  int status = 0;
  deptrum::stream::HeartbeatParam heartbeat_param;
#ifdef DEVICE_INTERNAL
  status = dynamic_pointer_cast<deptrum::stream::Stellar400Impl>(device)->SetExposureMaxValue(5000);
  if (status != 0) {
    LOG(WARNING) << "SetExposureMaxValue failed, error code " << status;
    return false;
  }
#endif

  if (params_.update_file_path != "") {
    status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->Upgrade(
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
  // open heart beat once only SetParam first
  static int params_num = 0;
  if (params_.heart_enable) {
    heartbeat_param.is_enabled = 1;  // open heart beat
    heartbeat_param.timeout = params_.heart_timeout_times;
    if (params_num == 0) {
      LOG(INFO) << "open heart beat " << (int) heartbeat_param.is_enabled;
      dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetHeartbeat(heartbeat_param,
                                                                              heart_beat_handle_);
    }
  } else {
    heartbeat_param.is_enabled = 0;  // close heart beat
    if (params_num == 0) {
      LOG(INFO) << "close heart beat";
      dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetHeartbeat(heartbeat_param,
                                                                              heart_beat_handle_);
    }
  }

  status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetShuffleMode(
      params_.shuffle_enable);
  if (status != 0) {
    LOG(WARNING) << "SetShuffleMode failed, error code " << status;
    return false;
  }
  params_num++;

  status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetUndistortion(
      params_.undistortion_enable);
  if (status != 0) {
    LOG(WARNING) << "SetUndistortion failed, error code " << status;
    return false;
  }

  status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetFrequencyFusionThreshold(
      params_.depth_th);
  if (status != 0) {
    LOG(WARNING) << "SetFrequencyFusionThreshold failed, error code " << status;
    return false;
  }
  float depth_th = 0;
  dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetFrequencyFusionThreshold(depth_th);
  LOG(INFO) << "current GetFrequencyFusionThreshold value = " << depth_th;
  status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetFrequencyFusionThreshold(
      (uint16_t) params_.ir_th);
  if (status != 0) {
    LOG(WARNING) << "SetFrequencyFusionThreshold failed, error code " << status;
    return false;
  }
  uint16_t ir_th = 0;
  dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetFrequencyFusionThreshold(ir_th);
  LOG(INFO) << "current GetFrequencyFusionThreshold value = " << ir_th;
  status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetScatterFilterThreshold(
      params_.ratio_th);
  if (status != 0) {
    LOG(WARNING) << "SetScatterFilterThreshold failed, error code " << status;
    return false;
  }
  float ratio_th;
  dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetScatterFilterThreshold(ratio_th);
  LOG(INFO) << "current GetScatterFilterThreshold value = " << ratio_th;
  status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetFilterType(
      params_.filter_type);
  if (status != 0) {
    LOG(WARNING) << "SetFilterType failed, error code " << status;
    return false;
  }
  int current_filter_type = 0;
  dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetFilterType(current_filter_type);
  LOG(INFO) << "current filtertype value = " << current_filter_type;
  if (params_.exposure_enable) {
    dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SwitchAutoExposure(false);
    status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetExposure(
        kIrCamera,
        params_.exposure_time);
    if (status != 0) {
      LOG(WARNING) << "SetExposure failed, error code " << status;
      return false;
    }
  } else {
    dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SwitchAutoExposure(true);
  }

  if (params_.outlier_point_removal_flag) {
    status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetOutlierPointRemoval(
        true);
  } else {
    status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetOutlierPointRemoval(
        false);
  }
  if (status != 0) {
    LOG(WARNING) << "SetOutlierPointRemoval failed, error code " << status;
    return false;
  }
  return true;
}

int Stellar400RosDevice::PointCloudFrameToRos(sensor_msgs::msg::PointCloud2& point_cloud,
                                              const deptrum::stream::StreamFrame& point_frame) {
  point_cloud.height = point_frame.rows;
  point_cloud.width = point_frame.cols;

  point_cloud.is_dense = true;
  point_cloud.is_bigendian = false;

  const size_t point_count = point_frame.size / sizeof(PointXyzRgbIr<float>);
  // LOG(INFO) << "point_count=" << point_count << ","
  //           << "point_frame.size=" << point_frame.size << ","
  //           << "sizeof(PointXyz<float>)=" << sizeof(PointXyzRgbIr<float>);

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
  pcd_modifier.resize(point_count);
  deptrum::PointXyzRgbIr<float>* current_point = static_cast<deptrum::PointXyzRgbIr<float>*>(
      point_frame.data.get());
  deptrum::PointXyzRgbIr<float> point;
  for (size_t i = 0; i < point_count; i++) {
    point = current_point[i];
    if (point.z <= 0.0f) {
      continue;
    } else {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }
  return 0;
}

int Stellar400RosDevice::PointCloudFrameToColorRos(
    sensor_msgs::msg::PointCloud2& point_cloud,
    const deptrum::stream::StreamFrame& point_frame) {
  point_cloud.height = point_frame.rows;
  point_cloud.width = point_frame.cols;

  point_cloud.is_dense = true;
  point_cloud.is_bigendian = false;

  const size_t point_count = point_frame.size / sizeof(PointXyzRgbIr<float>);

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud, "b");
  pcd_modifier.resize(point_count);

  deptrum::PointXyzRgbIr<float>* current_point = static_cast<deptrum::PointXyzRgbIr<float>*>(
      point_frame.data.get());
  deptrum::PointXyzRgbIr<float> point;
  for (size_t i = 0; i < point_count; i++) {
    point = current_point[i];

    if (point.z <= 0.0f) {
      continue;
    } else {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
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

void Stellar400RosDevice::PointCloudFramePublisherThread() {
  int status;
  builtin_interfaces::msg::Time capture_time;
  deptrum::stream::StreamFrames point_cloud_frame;
  rclcpp::WallRate loop_rate(params_.ir_fps);
  while (running_ && rclcpp::ok()) {
    status = streams_.point_cloud->GetFrames(point_cloud_frame, 2000);
    if (status != 0) {
      if (status == 0x42005) {
        LOG(WARNING) << "Get point cloud frame timeout!";
        continue;
      } else if (status == 0x2100c) {
        LOG(WARNING) << "Device disconnect!";
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        continue;
      } else {
        LOG(ERROR) << "Get point cloud  frame error " << status;
        // rclcpp::shutdown();
        // exit(status);
        continue;
      }
    }
    if (params_.point_cloud_enable) {
      for (int i = 0; i < point_cloud_frame.count; i++) {
        auto frame = point_cloud_frame.frame_ptr[i];
        TimestampToRos(frame->timestamp * 1000, capture_time);
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
        pointcloud_publisher_->publish(tmp_point_cloud);
      }
    }
    loop_rate.sleep();
  }
  LOG(INFO) << "ending point cloud";
}

void Stellar400RosDevice::RgbFramePublisherThread() {
  int status;
  builtin_interfaces::msg::Time capture_time;
  // static int rgb_time_out_num = 0;

  sensor_msgs::msg::CameraInfo rgb_camera_info;
  camera_parameters_.GetRgbCameraInfo(rgb_camera_info);

  deptrum::stream::StreamFrames rgb_frame;
  // streams_.rgb->AllocateFrame(rgb_frame);
  rclcpp::WallRate loop_rate(params_.rgb_fps);
  while (running_ && rclcpp::ok()) {
    status = streams_.rgb->GetFrames(rgb_frame, 2000);
    if (status != 0) {
      if (status == 0x42005) {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Get rgb frame timeout!";
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

    if (params_.rgb_enable) {
      for (int i = 0; i < rgb_frame.count; i++) {
        auto frame = rgb_frame.frame_ptr[i];
        TimestampToRos(frame->timestamp * 1000, capture_time);
        if (frame->frame_type == deptrum::FrameType::kRgbFrame) {
          status = rgb_frame_publish(*frame, capture_time);
        } else {
          continue;
        }
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get rgb frame!";
          rclcpp::shutdown();
          exit(status);
        }
        rgb_camera_info.header.stamp = capture_time;
        rgb_camerainfo_publisher_->publish(rgb_camera_info);
      }
    }

    // rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
  LOG(INFO) << "ending rgb";
}

void Stellar400RosDevice::DepthIrFramePublisherThread() {
  int status;
  builtin_interfaces::msg::Time capture_time;
  // static int rgb_time_out_num = 0;
  sensor_msgs::msg::CameraInfo ir_camera_info;
  camera_parameters_.GetIrCameraInfo(ir_camera_info);

  deptrum::stream::StreamFrames depth_ir_frames;
  rclcpp::WallRate loop_rate(params_.ir_fps);
  while (running_ && rclcpp::ok()) {
    if (node_->count_subscribers("camera_version")) {
      std_msgs::msg::String camera_version;
      camera_version.data = device_information_.stream_sdk_version + "," +
                            device_information_.device_name + "," + device_information_.serial_num +
                            "," + device_information_.ir_firmware_version + "," +
                            device_information_.rgb_firmware_version;
      camera_version_publisher_->publish(camera_version);
    }

    status = streams_.depth_ir->GetFrames(depth_ir_frames, 2000);
    if (status != 0) {
      if (status == 0x42005) {
        LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << "::Get depth ir frame timeout!";
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

    for (int i = 0; i < depth_ir_frames.count; i++) {
      auto frame = depth_ir_frames.frame_ptr[i];
      TimestampToRos(frame->timestamp * 1000, capture_time);
      if (frame->frame_type == deptrum::FrameType::kDepthFrame && params_.depth_enable) {
        status = depth_frame_publish(*frame, capture_time);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get depth frame!";
          rclcpp::shutdown();
          exit(status);
        }
      } else if (frame->frame_type == deptrum::FrameType::kIrFrame && params_.ir_enable) {
        status = ir_frame_publish(*frame, capture_time);
        if (status != 0) {
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Failed to get ir frame!";
          rclcpp::shutdown();
          exit(status);
        }
      }
    }
    if (params_.ir_enable || params_.depth_enable) {
      ir_camera_info.header.stamp = capture_time;
      ir_camerainfo_publisher_->publish(ir_camera_info);
    }
    loop_rate.sleep();
  }
  LOG(INFO) << "ending depth ir";
}

void Stellar400RosDevice::HeartBeatCb(int result) {
  // 0-error 1-succeed 2-setparam
  if (0 == result) {
    LOG(WARNING) << "send heart beat failed";
  } else if (2 == result) {
    LOG(INFO) << "[heart beat]reset device parms";
    if (!SetDeviceParams(device_)) {
      LOG(WARNING) << "[heart beat]set device params error";
    }
  }
}
