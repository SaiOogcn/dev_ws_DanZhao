#include "utils.h"
#include <chrono>
#include <iomanip>
#include <random>

namespace deptrum {
namespace stream {

std::ostream& operator<<(std::ostream& out, const Version& v) {
  out << std::to_string(v.major) << "." << std::to_string(v.minor) << "."
      << std::to_string(v.revision);
  return out;
}

std::string FeatureCodeToString(int code) {
  switch (code) {
    case 0x101:
      return "image pixel error";
    case 0x102:
      return "timeout";
    case 0x103:
      return "failed to detect face";
    case 0x104:
      return "failed to get feature";
    case 0x105:
      return "failed to allocate memory";
    case 0x106:
      return "image size error";
    case 0x107:
      return "image name error";
    case 0x108:
      return "failed to load image";
    case 0x109:
      return "failed to inster feature";
    default:
      return std::to_string(code);
  }
}

std::ostream& operator<<(std::ostream& out, const DeviceVersionInfo& info) {
  std::cout << "camera version: " << info.camera_sdk_version << std::endl;
  std::cout << "ir firmware version: " << info.firmware_version << std::endl;
  std::cout << "rgb firmware version: " << info.rgb_firmware_version << std::endl;
  std::cout << "calib version: " << info.calib_version << std::endl;
  std::cout << "depth version: " << info.depth_version << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const StreamDeviceVersionInfo& info) {
  std::cout << "camera version: " << info.camera_sdk_version << std::endl;
  std::cout << "kernel version: " << info.kernel_version << std::endl;
  std::cout << "depth version: " << info.depth_version << std::endl;
  return out;
}

bool BitIsOne(uint32_t data, uint32_t bit) {
  return (data & bit) == bit;
}

std::ostream& operator<<(std::ostream& out, const CaptureFaceResult& result) {
  PrintCurrentTime();
  switch (result.capture_handler_status) {
    case CaptureStatus::kFaceDetected:
      out << ", Face Detected";
      if (result.live_face_errors == (uint32_t) LiveFaceErrors::kLiveFaceOk) {
        out << "-->Live !";
        if (result.face_features.data) {
          out << "[face box] x: " << result.face_bbox.x << ", y: " << result.face_bbox.y
              << ", w: " << result.face_bbox.w << ", h: " << result.face_bbox.h << std::endl;

          out << "[score_rgb_occlusion]: " << result.score_rgb_occlusion << std::endl;
          out << "[score_ir_occlusion]: " << result.score_ir_occlusion << std::endl;
          out << "[score_ir_liveness]: " << result.score_ir_liveness << std::endl;
          out << "[score_speckle_liveness]: " << result.score_speckle_liveness << std::endl;
          out << "[score_ir_speckle_liveness]: " << result.score_ir_speckle_liveness << std::endl;
          out << "[score_speckle_depth_liveness]: " << result.score_speckle_depth_liveness
              << std::endl;
          out << "[score_angle]: " << result.score_angle << std::endl;
          out << "[score_blur]: " << result.score_blur << std::endl;

          out << "[face feature]: ";
          for (size_t i = 0; i < result.face_features.data_len; i++) {
            out << std::hex << static_cast<unsigned short>(result.face_features.data.get()[i]);
          }
        }

      } else {
        if (BitIsOne(result.live_face_errors, (uint32_t) LiveFaceErrors::kIncompleteFace)) {
          out << "-->incomplete face";
        } else if (BitIsOne(result.live_face_errors, (uint32_t) LiveFaceErrors::kBlurryFace)) {
          out << "-->blurry face";
        }

        // internal
        out << "(";
        uint32_t error_code = result.live_face_errors >> 4;
        if (BitIsOne(error_code, (uint32_t) HintMap::kTooClose)) {
          out << "Too Close,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kFaceTooSmall)) {
          out << "Face Too Small,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kNotCenteredX)) {
          out << "Not Centered X,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kNotCenteredY)) {
          out << "Not Centered Y,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kAngled)) {
          out << "Angled,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kCovered)) {
          out << "Covered,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kMasked)) {
          out << "Masked,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kIrLiveness)) {
          out << "IR Liveness,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::k3DLiveness)) {
          out << "3D Liveness,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kBadExpression)) {
          out << "Bad Expression,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kBlurred)) {
          out << "Blurred,";
        }
        if (BitIsOne(error_code, (uint32_t) HintMap::kNeedAe)) {
          out << "Need AE,";
        }
        out << ")";
      }
      break;
    case CaptureStatus::kNoFaceDetected:
      out << ", No FaceDetected";
      break;
    case CaptureStatus::kTooClose:
      out << ", Too Close";
      break;
    case CaptureStatus::kManualStop:
      out << ", Manual Stop";
      break;
    case CaptureStatus::kTimeout:
      out << ", Timeout";
      break;
    case CaptureStatus::kInitializing:
      out << ", Initializing";
      break;
    default:
      out << ", Unknow Status";
      break;
  }

  out << std::dec;
  return out;
}

uint64_t GetRandomNum() {
  uint32_t value = std::random_device{}();
  std::cout << "generator num: " << value << std::endl;
  return value;
}

void PrintCurrentTime() {
  auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::cout << std::put_time(std::localtime(&t), "%F %T");
}

}  // namespace stream
}  // namespace deptrum
