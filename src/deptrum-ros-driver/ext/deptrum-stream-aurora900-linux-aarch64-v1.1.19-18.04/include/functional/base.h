/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_FUNCTIONAL_BASE_H_
#define DEPTRUM_STREAM_INCLUDE_FUNCTIONAL_BASE_H_

#include <iostream>
#include <map>
#include "deptrum/common_types.h"
#include "deptrum/stream_types.h"

namespace deptrum {
namespace stream {

static std::map<int, std::string> err_info = {
    /* Common Error Code */
    {0, "Success"},
    {1, "Unknown error"},
    {2, "Not implemented"},
    {3, "Invalid arguments"},
    {4, "Not supported"},
    {5, "Failed to allocate memory"},
    {6, "Deprecated interface"},
    /* Camera SDK Error Code*/
    {0x21001, "Failed to find devices"},
    {0x21002, "Access to null pointer"},
    {0x21003, "Failed to open the camera"},
    {0x21004, "Failed to close the camera"},
    {0x21005, "Failed to start capture"},
    {0x21006, "Failed to set or get data"},
    {0x21007, "Failed to check data"},
    {0x21008, "Failed to open Ir camera"},
    {0x21009, "Failed to open Rgb camera"},
    {0x2100A, "Failed operate usb serial"},
    {0x2100B, "Camera is not running"},
    {0x2100C, "Camera is not open"},
    {0x2100D, "Driver error"},
    {0x2100E, "Camera is not config"},
    {0x22001, "Data size error"},
    {0x22002, "Data not ready"},
    {0x22004, "Unsupported camera mode"},
    {0x22010, "Camera is timeout"},
    /* Stream API Error Code */
    {0x41001, "Invalid frame format."},
    {0x41002, "Configuration file illegal"},
    {0x41003, "Get serial number failed"},
    {0x42001, "Stream not open in device"},
    {0x42002, "Stream not started"},
    {0x42003, "Invalid stream type"},
    {0x42004, "Invalid stream object"},
    {0x42005, "GetFrame()/GetFrames() timeout"},
    {0x42006, "Same type stream already exists"},
    {0x42007, "Same type stream already exists"},
    {0x42008, "Create depth-fusion failed"},
    {0x42009, "Initialize sniper failed"},
    {0x42010, "Initialize depth-fusion failed"},
    {0x42011, "Calibration size is larger than 4096 bytes"},
    {0x42012, "Device hasn't open"},
    {0x42013, "The sniper type read from register is unsupported"},
    {0x42014, "The exposure time is over-ranged.(0~2000us)"},
    {0x42015, "Unsupported exposure component"},
    {0x42016, "Reset config failed, stream is running"},
    {0x42017, "Specified type stream doesn't exist"},
    {0x42018, "Ir camera has not open"},
    {0x42019, "Rgb stream not started"},
    {0x42020, "AlignDepth stream not started"},
    {0x42022, "Set fps failed, device has open."},
    /* Sniper API Error Code */
    {0x50000, "Raw tof frame buffer is full."},
    {0x50001, "Failed to load calibration.pb or calibration.pbtxt."},
    {0x50002, "Size mismatch between config and calibration data."},
    {0x50003, "Failed create rangefinder."},
    {0x50004, "Get raw tof frame failed, raw tof frame buffer is empty."},
    {0x50005, "Raw ir frame embeded line matched."},
    {0x50006, "Raw ir frame embeded line dismatched."},
    {0x50007, "Has initialized already."},
    {0x50008, "Has not been initialized."},
    /* Depth-fusion API Error Code */
    {0x60001, "Failed to load depth-fusion.pbtxt."},
};

static std::map<StreamType, std::string> stream_type_to_string_map = {
    {StreamType::kRgb, "Rgb"},
    {StreamType::kIr, "IrFrame"},
    {StreamType::kDepth, "DepthFrame"},
    {StreamType::kRgbd, "Rgbd"},
    {StreamType::kRgbIr, "RgbdIr"},
    {StreamType::kDepthIr, "DepthIr"},
    {StreamType::kDepthIrLaser, "DepthIrLaser"},
    {StreamType::kRgbdIr, "RgbdIr"},
    {StreamType::kSpeckleCloud, "PointSparse"},
    {StreamType::kPointCloud, "PointDense"},
    {StreamType::kRgbdPointCloud, "kRgbdPointCloud"},
    {StreamType::kRgbdIrPointCloud, "RgbdIrPointDense"},
    {StreamType::kRgbdIrFlag, "RgbdIrFlag"},
    {StreamType::kDepthIrFlag, "DepthIrFlag"},
};

static std::map<FrameType, std::string> frame_type_to_string_map = {
    {FrameType::kRgbFrame, "RgbFrame"},
    {FrameType::kIrFrame, "IrFrame"},
    {FrameType::kDepthFrame, "DepthFrame"},
    {FrameType::kLaserFrame, "LaserFrame"},
    {FrameType::kLaserFrame, "kLaserFrame"},
    {FrameType::kPointCloudFrame, "PointCloudFrame"},
    {FrameType::kRgbdPointCloudFrame, "RgbdPointCloudFrame"},
    {FrameType::kRgbdIrPointCloudFrame, "RgbdIrPointCloudFrame"},
    {FrameType::kSemanticFrame, "SemanticFrame"},
    {FrameType::kRawFrame, "RawFrame"},
};

static std::map<FrameAttr, std::string> frame_attr_to_string_map = {
    {FrameAttr::kAttrInvalid, "Invalid"},
    {FrameAttr::kAttrLeft, "left"},
    {FrameAttr::kAttrRight, "right"},
    {FrameAttr::kAttrSl, "sl"},
    {FrameAttr::kAttrItof, "itof"},
    {FrameAttr::kAttrMtof, "mtof"},
    {FrameAttr::kAttrStof, "stof"},
    {FrameAttr::kAttrAe, "ae"},
    {FrameAttr(FrameAttr::kAttrMtof + FrameAttr::kAttrAe), "mtof_ae"},
    {FrameAttr(FrameAttr::kAttrStof + FrameAttr::kAttrAe), "stof_ae"},
};

static std::map<FrameMode, std::string> frame_mode_to_string_map = {
    {FrameMode::kInvalid, "kInvalid"},
    {FrameMode::kRes320x200RgbYuv, "kRes320x200RgbYuv"},
    {FrameMode::kRes480x300RgbYuv, "kRes480x300RgbYuv"},
    {FrameMode::kRes640x400RgbYuv, "kRes640x400RgbYuv"},
    {FrameMode::kRes320x240RgbJpeg, "kRes320x240RgbJpeg"},
    {FrameMode::kRes400x256RgbJpeg, "kRes400x256RgbJpeg"},
    {FrameMode::kRes480x640RgbJpeg, "kRes480x640RgbJpeg"},
    {FrameMode::kRes640x480RgbJpeg, "kRes640x480RgbJpeg"},
    {FrameMode::kRes800x512RgbJpeg, "kRes800x512RgbJpeg"},
    {FrameMode::kRes960x1280RgbJpeg, "kRes960x1280RgbJpeg"},
    {FrameMode::kRes1080x1920RgbJpeg, "kRes1080x1920RgbJpeg"},
    {FrameMode::kRes1280x960RgbJpeg, "kRes1280x960RgbJpeg"},
    {FrameMode::kRes1600x1080RgbJpeg, "kRes1600x1080RgbJpeg"},
    {FrameMode::kRes1920x1080RgbJpeg, "kRes1920x1080RgbJpeg"},
    {FrameMode::kRes320x200Ir8Bit, "kRes320x200Ir8Bit"},
    {FrameMode::kRes480x300Ir8Bit, "kRes480x300Ir8Bit"},
    {FrameMode::kRes640x400Ir8Bit, "kRes640x400Ir8Bit"},
    {FrameMode::kRes320x240Ir8Bit, "kRes320x240Ir8Bit"},
    {FrameMode::kRes400x640Ir8Bit, "kRes400x640Ir8Bit"},
    {FrameMode::kRes400x640Ir16Bit, "kRes400x640Ir16Bit"},
    {FrameMode::kRes480x640Ir8Bit, "kRes480x640Ir8Bit"},
    {FrameMode::kRes480x640Ir16Bit, "kRes480x640Ir16Bit"},
    {FrameMode::kRes640x480Ir8Bit, "kRes640x480Ir8Bit"},
    {FrameMode::kRes650x800Ir8Bit, "kRes650x800Ir8Bit"},
    {FrameMode::kRes736x480Ir8Bit, "kRes736x480Ir8Bit"},
    {FrameMode::kRes800x1280Ir8Bit, "kRes800x1280Ir8Bit"},
    {FrameMode::kRes800x1280Ir16Bit, "kRes800x1280Ir16Bit"},
    {FrameMode::kRes240x180Depth16Bit, "kRes240x180Depth16Bit"},
    {FrameMode::kRes640x480Depth16Bit, "kRes640x480Depth16Bit"},
    {FrameMode::kRes400x640Depth16Bit, "kRes400x640Depth16Bit"},
    {FrameMode::kRes480x640Depth16Bit, "kRes480x640Depth16Bit"},
    {FrameMode::kRes800x1280Depth16Bit, "kRes800x1280Depth16Bit"},
};

#define CHECK_SDK_RETURN_VALUE(error_code)                                \
  {                                                                       \
    if (0 != (error_code)) {                                              \
      std::cerr << "Error Code:0x" << std::hex << (error_code)            \
                << ", Description:" << err_info[error_code] << std::endl; \
      getchar();                                                          \
      return -1;                                                          \
    }                                                                     \
  }

#define CHECK_GET_FRAMES(error_code)                                      \
  {                                                                       \
    if (0 != (error_code)) {                                              \
      std::cerr << "Error Code:0x" << std::hex << (error_code)            \
                << ", Description:" << err_info[error_code] << std::endl; \
      if (0x42005 == error_code) {                                        \
        continue;                                                         \
      } else {                                                            \
        break;                                                            \
      }                                                                   \
    }                                                                     \
  }

#define CHECK_DEVICE_COUNT(size)                                           \
  if (0 == (size)) {                                                       \
    std::cerr << "Error:the number of devices is " << (size) << std::endl; \
    getchar();                                                             \
    return -1;                                                             \
  }

#define CHECK_DEVICE_VALID(p)                          \
  if (nullptr == (p)) {                                \
    std::cerr << "Creata Device failed!" << std::endl; \
    getchar();                                         \
    return -1;                                         \
  }

#define COUT_INTRISIC(intrinsic)                                                                \
  std::cout << #intrinsic ":\n"                                                                 \
            << "cols:\t" << intrinsic.cols << "\nrows:\t" << intrinsic.rows << std::endl        \
            << "focal_length[2]:\t[" << intrinsic.focal_length[0] << ","                        \
            << intrinsic.focal_length[1] << "]\n"                                               \
            << "principal_point[2]:\t[" << intrinsic.principal_point[0] << ","                  \
            << intrinsic.principal_point[1] << "]\n"                                            \
            << "distortion_coeffs[5]:\t[" << intrinsic.distortion_coeffs[0] << ","              \
            << intrinsic.distortion_coeffs[1] << "," << intrinsic.distortion_coeffs[2] << ","   \
            << intrinsic.distortion_coeffs[3] << "," << intrinsic.distortion_coeffs[4] << "]\n" \
            << std::endl;

}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_FUNCTIONAL_BASE_H_
