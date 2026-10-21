#ifndef ROS_TYPES_H
#define ROS_TYPES_H

#include <string>
#include "deptrum/common_types.h"
// #include "deptrum/deptrum_stream.h"
#include "deptrum/stream.h"
#include "deptrum/stream_types.h"
#define STELLAR_PID 0x3251
struct BgrPixel {
  uint8_t b;
  uint8_t g;
  uint8_t r;
};

struct DeptrumRosDeviceParams {
  double rotation_matrix[9];  // 3x3 Rotation matrix stored in row major order
  double translation_vector[3];
  std::string serial_number;    // Serial number
  std::string usb_port_number;  // USB port number
  std::string imu_path;
  std::string update_file_path;  // upgrade img path
  std::string log_dir;           // log dir storage
  int boot_order;
  int device_numbers;
  bool rgb_enable;    // True if rgb camera should be enabled
  bool depth_enable;  // True if depth camera should be enabled
  bool ir_enable;     // True if depth camera should be enabled
  // bool speckle_enable;  // True if depth camera should be enabled
  bool imu_enable;
  bool point_cloud_enable;    // True if point cloud should be enabled
  bool get_raw_data_enable;   // open get ir raw data enable
  bool get_raw_data_disable;  // close get ir raw data enable
  bool rgbd_enable;           // True if rgbd should be enabled
  bool flag_enable;
  bool output_compressed_rgb;       // True if you want to output compressed rgb
  bool listen_compressed_rgb;       // True if you want to have a quick view of compressed rgb
  bool outlier_point_removal_flag;  // True if outlier point removal enable
  bool depth_ir_mirroring;          // True if flip depth_ir
  int slam_mode;
  bool imu_mode;       // false标定工厂模式，true 正常模式
  bool image_storage;  // One click image storage
  bool heart_enable;   // Enable or disable  open heart to reboot device while depth_ir stream time
                       // out
  bool stream_sdk_log_enable;  // bool stream_sdk_log_enable;
  bool show_debug_info;
  bool show_system_info;
  bool depth_correction;      // Enable or disable depth correction
  bool align_mode;            // Enable or disable align rgbd-ir
  int rgb_fps;                // The FPS of the Rgb cameras. Options are: 5, 10, 15, 20, 25, 30
  int ir_fps;                 // The FPS of the IR cameras. Options are: 5, 10, 15, 20, 25, 30
  bool shuffle_enable;        // Enable or disable the shuffle mode
  bool undistortion_enable;   // Enable or disable the undistortion
  bool exposure_enable;       // Enable or disable to set camera exposure time
  int exposure_time;          // Set the exposure time for the camera
  bool gain_enable;           // Enable or disable to set camera exposure time
  int gain_value;             // Set the exposure time for the camera
  int resolution_mode_index;  // resolution param index
  int filter_type;            // Set the way for filtering
  float depth_th;             // Set the threshold of depth for frequency fusion,range in [0, 7500]
  int ir_th;       // Set the threshold of ir for frequency fusion, fusion,range in [0, 4095]
  float ratio_th;  // Set the distance percentage threshold for scatter filtering
  int threshold_size;
  int heart_timeout_times;         // heart time out time
  int raw_frame_count;             // get raw frame count once
  int minimum_filter_depth_value;  // minimun filter depth value
  int maximum_filter_depth_value;  // maximum filter depth value
  int stof_minimum_range;          // stof minimom range
  float laser_power;               // adjust laser driver 1-auto 2-indoor
  int mtof_filter_level;           // Set the mtof filter level 0:low  1:middle  2:high
  int stof_filter_level;           // Set the stof filter level 0:low  1:middle  2:high
  int mtof_crop_up;                // Set the mtof crop up
  int mtof_crop_down;              // Set the mtof crop down
  int ext_rgb_mode;                // 0: disable; 1: 1920*1080; 2: 1280*720
  bool enable_imu;                 // 0: disable; 1: enable
};

#endif  // ROS_TYPES_H
