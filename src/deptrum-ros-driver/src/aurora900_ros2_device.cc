#include <sstream>
#include "deptrum_ros_driver/aurora900_ros2_device.h"
using namespace std;
using namespace deptrum;
Aurora900RosDevice::Aurora900RosDevice(std::shared_ptr<rclcpp::Node> node) : RosDevice(node) {
  if (params_.rgb_enable) {
    rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", 1);
    rgb_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "rgb/camera_info",
        1);
  }
  if (params_.ir_enable) {
    ir_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("ir/image_raw", 1);
  }
  if (params_.depth_enable) {
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 1);
  }
  if (params_.point_cloud_enable) {
    pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1);
  }
  if (params_.depth_enable || params_.ir_enable || params_.point_cloud_enable) {
    ir_camerainfo_publisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "ir/camera_info",
        1);
  }
}
Aurora900RosDevice::~Aurora900RosDevice() {
  Stop();
}
int Aurora900RosDevice::Start() {
  if (running_ == true) {
    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Device already started";
    return 0;
  }

  if (device_ != nullptr) {
    camera_parameters_.Initialize(device_, params_.boot_order);
  }

  running_ = true;

  int status = 0;
  std::vector<deptrum::stream::StreamType> types;
  if (params_.rgbd_enable) {
    types.push_back(deptrum::stream::StreamType::kRgbdIr);
    if (params_.point_cloud_enable) {
      types.push_back(deptrum::stream::StreamType::kRgbdPointCloud);
    }
  } else {
    if (params_.rgb_enable) {
      types.push_back(deptrum::stream::StreamType::kRgb);
    }
    if (params_.depth_enable || params_.ir_enable) {
      types.push_back(deptrum::stream::StreamType::kDepthIr);
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

    frame_publisher_thread_ = thread(&Aurora900RosDevice::FramePublisherThread, this);
  }

  return 0;
}

void Aurora900RosDevice::Stop() {
  LOG(INFO) << "begin stop()";
  if (running_ == false) {
    LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Cameras not started";
    return;
  }
  running_ = false;
  if (device_ != nullptr) {
    if (params_.heart_enable) {
      LOG(INFO) << "close heart ...";
      dynamic_pointer_cast<deptrum::stream::Aurora900>(device_)->StopHeartbeat();
    }
    // if (detector_thread_.joinable())
    // {
    //   detector_thread_.join();
    // }
    if (frame_publisher_thread_.joinable()) {
      frame_publisher_thread_.join();
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

void Aurora900RosDevice::InitDevice() {
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
          LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::GetDeviceList failed, error code"
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
                << "deptrum device usb port" << device_list[i].ir_camera.port_path;
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
    // for (int i = 1; i < 4; i++) {
    //   // ValidateFps(params_.ir_fps);
    //   status =
    //   dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetIrFps(params_.ir_fps); if
    //   (status != 0) {
    //     LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Set Ir fps failed, error code "
    //                << status << " num: " << i;
    //     continue;
    //   }

    //   // ValidateFps(params_.rgb_fps);
    //   status = dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->SetRgbFps(
    //       params_.rgb_fps);
    //   if (status != 0) {
    //     LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::Set Rgb fps failed, error code "
    //                << status << " num: " << i;
    //     ;
    //     continue;
    //   }
    //   break;
    // }

    deptrum::FrameMode ir_mode_ref, rgb_mode_ref, depth_mode_ref;
    int num = 0;
    std::vector<std::tuple<deptrum::FrameMode, deptrum::FrameMode, deptrum::FrameMode>>
        device_resolution_vec;
    device->GetSupportedFrameMode(device_resolution_vec);
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
    // dynamic_pointer_cast<deptrum::stream::Stellar400>(device)->GetDeviceInfo(device_information_);
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

    status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetIrFps(params_.ir_fps);
    if (status != 0) {
      LOG(ERROR) << "set ir fps error,set it only 5 or 10 or 12 or 15 fps, try check it"
                 << " error_code=" << status;
      rclcpp::shutdown();
      exit(-1);
    }

    deptrum::DeviceInformation device_information;
    deptrum::stream::DeviceDescription camera_information;
    device->GetDeviceInfo(device_information);
    status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->GetDeviceInfo(
        camera_information);
    if (status != 0) {
      // ROS_WARN("BOOT_ORDER_%d::GetDeviceInfo failed, error code %x", params_.boot_order, status);
      LOG(WARNING) << "BOOT_ORDER_" << params_.boot_order << ":GetDeviceInfo failed, error code "
                   << status;
      device->Close();
      continue;
    } else if (params_.serial_number == "" ||
               device_information.ir_camera.serial_number == params_.serial_number) {
      LOG(INFO) << "BOOT_ORDER_" << params_.boot_order << "::Device information: ";
      LOG(INFO) << "\t sdk version: " << camera_information.stream_sdk_version;
      LOG(INFO) << "\t device name: " << camera_information.device_name;
      LOG(INFO) << "\t serial number: " << camera_information.serial_num;
      LOG(INFO) << "\t firmware version: " << camera_information.ir_firmware_version;

      if (device_ != nullptr) {
        // ROS_WARN("BOOT_ORDER_%d::Device Number: %d, device_ is already init.",
        //          params_.boot_order,
        //          i);
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
    // ROS_ERROR("BOOT_ORDER_%d::All %d devices have attempted to connect, Failed to open device",
    //           params_.boot_order,
    //           device_count);
    LOG(ERROR) << "BOOT_ORDER_" << params_.boot_order << "::"
               << "All " << device_count
               << "devices have attempted to connect, Failed to open device ";
    rclcpp::shutdown();
    exit(-1);
  }
}

bool Aurora900RosDevice::SetDeviceParams(std::shared_ptr<deptrum::stream::Device>& device) {
  int status = 0;

  if (params_.update_file_path != "") {
    status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->Upgrade(
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

  status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->EnableDepthIrMirroring(
      params_.depth_ir_mirroring);
  if (status != 0) {
    // ROS_WARN("EnableDepthIrMirroring failed, error code %x", status);
    LOG(WARNING) << "EnableDepthIrMirroring failed, error code " << status;
    return false;
  }
  if (params_.exposure_enable) {
    if (params_.exposure_time < 1 || params_.exposure_time > 31)
      LOG(WARNING) << "current exposure time is " << params_.exposure_time
                   << " please input [1-31],单位为0.1ms";
    else {
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SwitchAutoExposure(false);
      if (status != 0) {
        LOG(WARNING) << "SwitchAutoExposure failed, error code " << status;
        return false;
      }
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetExposure(
          kIrCamera,
          params_.exposure_time);
      if (status != 0) {
        LOG(WARNING) << "SetExposure failed, error code " << status;
        return false;
      } else {
        LOG(INFO) << "set exposure value " << params_.exposure_time << "succeed";
      }
      // int current_exposure_value = 0;
      // dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->GetExposure(kIrCamera,
      //                                                                       current_exposure_value);
      // LOG(INFO) << "current exposure value is " << current_exposure_value;
    }
  } else {
    if (device->GetDeviceName() == "Aurora930") {
      LOG(INFO) << "Switch Auto Exposure";
      dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SwitchAutoExposure(true);
    }
  }
  if (params_.gain_enable) {
    // ValidateGain(params_.gain_value); 10-160
    if (params_.gain_value < 10 || params_.gain_value >= 160)
      LOG(WARNING) << "current gain value is " << params_.gain_value
                   << " please input [10-160), curren is not valid";
    else {
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetGain(
          kIrCamera,
          params_.gain_value);
      if (status != 0) {
        LOG(WARNING) << "SetGain failed, error code " << status;
        return false;
      }
      uint32_t current_gain_value = 0;
      dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->GetGain(kIrCamera,
                                                                        current_gain_value);
      LOG(INFO) << "current gain value is " << current_gain_value;
    }
  }
  if (device->GetDeviceName() == "Aurora930") {
    if (params_.threshold_size < 30 || params_.threshold_size > 400)
      LOG(WARNING) << "current threshold size is " << params_.threshold_size
                   << " please input [30-400], curren is not valid";
    else {
      status = dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetRemoveFilterSize(
          params_.threshold_size);
      if (status != 0) {
        LOG(WARNING) << "SetRemoveFilterSize failed, error code " << status;
        return false;
      }
    }
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->DepthCorrection(
        params_.depth_correction);
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SwitchAlignedMode(params_.align_mode);
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->FilterOutRangeDepthMap(
        params_.minimum_filter_depth_value,
        params_.maximum_filter_depth_value);
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetLaserDriver(params_.laser_power);

    deptrum::stream::HeartbeatParam heartbeat_param;
    if (params_.heart_enable) {
      heartbeat_param.is_enabled = 1;  // open heart beat
    } else {
      heartbeat_param.is_enabled = 0;  // close heart beat
    }
    dynamic_pointer_cast<deptrum::stream::Aurora900>(device)->SetHeartbeat(
        heartbeat_param,
        [](int result) {
          if (0 == result) {
            LOG(WARNING) << "heartbeat failed!!!";
          }
        });
  }

  return true;
}
