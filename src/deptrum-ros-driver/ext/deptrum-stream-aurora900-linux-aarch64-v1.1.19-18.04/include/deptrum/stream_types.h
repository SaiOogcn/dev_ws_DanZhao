/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_STREAM_TYPES_H_
#define DEPTRUM_STREAM_INCLUDE_STREAM_TYPES_H_

// #include <stdlib.h>
// #include <cstdint>
#include <cstring>
#include <functional>
#include <list>
#include <memory>
#include <vector>
#include "common_types.h"
#include "stream_types.h"

namespace deptrum {
namespace stream {

#ifdef __ANDROID__
#define CONFIG_PATH "/sdcard/deptrum/config/"
#else
#define CONFIG_PATH "./config/"
#endif

#if (defined _WIN32) && (defined STREAM_DLL)
#ifdef STREAM_EXPORT
#define STREAM_DLL __declspec(dllexport)
#else
#define STREAM_DLL __declspec(dllimport)
#endif
#else
#define STREAM_DLL
#endif

using ProgressHandler = std::function<void(int)>;
using UpgradeReadyHandler = std::function<void()>;
using HeartbeatResult = std::function<void(int)>;
using IrRawFrameHandler = std::function<void(struct IrRawFrame&)>;

struct DeviceState {
  float rgb_raw_fps;
  float depth_ir_raw_fps;
  float out_fps;
  bool start_stream;
  int error_code;
};

struct IrRawFrame {
  std::shared_ptr<uint8_t> img_raw;
  int img_raw_len = 0;
  uint32_t idx = 0;
  uint64_t timestamp = 0;  // Timestamp of the frame
};

struct HeartbeatParam {
  bool is_enabled{false};
  // Callbacks for each failure, if is_callback_every_failure is 1, then the
  // callback will be called every time the heartbeat fails, otherwise it will be
  // called only when the heartbeat fails for allowable_failures_counts.
  bool is_callback_every_failure{false};
  uint32_t timeout{500};
  uint32_t allowable_failures_counts{4};
};

enum StreamType {
  kInvalidStreamType = 0,
  kRgb,
  kIr,
  kDepth,
  kRgbd,
  kRgbIr,
  kRgbdIr,
  kDepthIr,
  kDepthIrLaser,
  kSpeckleCloud,  // One point per speckle; a speckle may occupy multiple pixels on depth map
  kPointCloud,    // One point per pixel; some pixels may be missing on depth map
  kRgbdPointCloud,
  kRgbdIrPointCloud,
  kRgbdIrFlag,
  kDepthIrFlag,
};

struct DeviceDescription {
  std::string device_name;           // device name
  std::string serial_num;            // serial number
  std::string stream_sdk_version;    // stream sdk version
  std::string rgb_firmware_version;  // rgb firmware version
  std::string ir_firmware_version;   // ir firmware version
  uint16_t pid;                      // device pid
  uint16_t vid;                      // device vid
};

typedef struct Data {
  int32_t data_len;
  const char* data;
} Data;

typedef struct SupportedInfo {
  uint8_t scan_face_mode;
  uint8_t scan_code_mode;
  uint8_t running_7x24_hours;
  Data depth_range;
  uint8_t is_support_se;
  uint8_t is_support_synced_2_img;
} SupportedInfo;

typedef enum TemperatureType {
  kTemperatureCamera = 1,
  kTemperatureVcsel,
  kTemperatureCpu,
} TemperatureType;

struct DeviceSystemInfo {
  int32_t cpu_user{0};
  int32_t cpu_system{0};
  int32_t cpu_idle{0};
  int32_t cpu_io{0};
  int32_t mem_total_kb{0};
  int32_t mem_free_kb{0};
};

struct DeviceDebugInfo {
  uint32_t watch_dog_reboot_count{0};
  uint32_t app_reboot_count{0};
  uint32_t timeout_count{0};
  uint32_t usb_connect_err_count{0};
  uint32_t i2c_transfer_err_count{0};
  uint32_t eeprom_transfer_err_count{0};
  uint32_t tof_get_frame_timeout_count{0};
  uint32_t tof_get_err_frame_count{0};
  uint32_t sniper_get_frame_err_count{0};
  uint32_t sniper_generate_err_count{0};
  std::string last_crash_time;
};

struct CameraParam {  // Camera intrinsic parameters
  float cx;           // Principal point in image, x
  float cy;           // Principal point in image, y
  float fx;           // Focal length x
  float fy;           // Focal length y
};

struct Bbox {
  int16_t x = 0;
  int16_t y = 0;
  int16_t w = 0;
  int16_t h = 0;
};

struct FaceInfo {
  Bbox bbox;
  struct Point {
    float x = 0.0f;
    float y = 0.0f;
  } face5p[5];
};

template<class T>
struct ImuInfo {
  ImuData<T> imu_data;
  int16_t temperature;
  ImuInfo() = default;
  ImuInfo(uint64_t timestmp, T gyro_x, T gyro_y, T gyro_z, T acce_x, T acce_y, T acce_z) :
      imu_data(timestmp, gyro_x, gyro_y, gyro_z, acce_x, acce_y, acce_z) {}
};
template<typename T>
struct OpaqueData {
  std::shared_ptr<T> data;
  int32_t data_len = 0;
};

template<typename T>
struct CustomFrame {
  std::shared_ptr<T> data;
  char name[32]{0};
  int32_t width = 0;
  int32_t height = 0;
  int32_t data_len = 0;
  uint64_t timestamp = 0;
  int bits_per_pixel = 0;                          //!< Number of bits per pixel
  ImageFormat image_format = kInvalidImageFormat;  //!< Format of the image
  uint64_t idx = 0;
};

enum LedMode {
  LED_OFF_MODE = 0,
  LED_RED_MODE,         // 红常亮   6
  LED_GREEN_MODE,       // 绿常亮
  LED_BLUE_MODE,        // 蓝常亮
  LED_RED_GREEN_MODE,   // 红绿常亮
  LED_RED_BLUE_MODE,    // 红蓝常亮
  LED_GREEN_BLUE_MODE,  // 绿蓝常亮

  LED_RED_BLINK_MODE,         // 红闪烁  6-0
  LED_GREEN_BLINK_MODE,       // 绿闪烁
  LED_BLUE_BLINK_MODE,        // 蓝闪烁
  LED_RED_GREEN_BLINK_MODE,   // 红绿闪烁
  LED_RED_BLUE_BLINK_MODE,    // 红蓝闪烁
  LED_GREEN_BLUE_BLINK_MODE,  // 绿蓝闪烁

  LED_RED_BREATH_MODE,         // 红呼吸
  LED_GREEN_BREATH_MODE,       // 绿呼吸
  LED_BLUE_BREATH_MODE,        // 蓝呼吸
  LED_RED_GREEN_BREATH_MODE,   // 红绿呼吸
  LED_RED_BLUE_BREATH_MODE,    // 红蓝呼吸
  LED_GREEN_BLUE_BREATH_MODE,  // 绿蓝呼吸

  LED_WHITE_MODE,    // 白常亮
  LED_STANDBY_MODE,  // 基于psensor  蓝呼吸+白常亮+红绿闪烁

  LED_MAX_MODE = LED_STANDBY_MODE,
};

enum DeptrumNebulaFrameType {
  kFrameStof = 0,
  kFrameMtof = 1,
  kFrameRaw = 2,

  kFrameRgb = 3,  // use in usb device
};

enum DeptrumNebulaFilterLevel {
  kLowLevel = 0,
  kMiddleLevel = 1,
  kHighLevel = 2,
};

enum DeptrumNebulaAeRoiMode {
  kAeRoiModeNear = 0,
  kAeRoiModeMiddle = 1,
  kAeRoiModeFar = 2,
};

#ifdef __ANDROID__
#define CONFIG_PATH "/sdcard/config/"
#else
#define CONFIG_PATH "./config/"
#endif
#define ROI_MINIMUM_WxH_RANGE 3340

struct CaptureResult {
  /// Pointer to the RGB image frame.
  std::shared_ptr<Frame> img_rgb;

  /// Pointer to the IR image frame.
  std::shared_ptr<Frame> img_ir;

  /// Pointer to the depth image frame.
  std::shared_ptr<Frame> img_depth;
};

using CaptureCallback = std::function<void(const CaptureResult&)>;
using UpgradeCallback = std::function<void()>;
using ProgressCallback = std::function<void(int)>;

enum MainLightMode {
  kPalmLightGreenMode = 0,
  kPalmLightBlueMode,
};

enum EventType {
  kTemperatureTooLow = 1,
  kTemperatureTooHigh = 2,
  kLitePsensorEnableError,
  kLitePsensorHardwareError,
  kLitePsensorFaceError,
  kLitePsensorDistanceError,
  kLitePsensorDistanceNormal,
  kLitePsensorClosed,

  kLitePsensorFaceErrCount,
  // kLiteUpgradeEvent,
  // kLiteReportDoe,
  kLiteReportRgbScanFace,
  kLiteReportRgbScanCode,
  kLiteReportDepth,
  kLiteReportFlood,
  kLiteReportSpeckle,
  kLiteReportFaceDetect,

  kPalmReportWorkRgbSensor,
  kPalmReportWorkIrSensor,
  kPalmReportInitRgbSensor,
  kPalmReportInitIrSensor,
  kPalmReportInitAw21036,
  kPalmReportWorkAw21036,
  kPalmReportInitMp3336,
  kPalmReportWorkMp3336,
  kPalmReportInitPsenosrUpperLeft,
  kPalmReportInitPsenosrUpperRight,
  kPalmReportInitPsenosrLowerLeft,
  kPalmReportInitPsenosrLowerRight,
  kPalmReportWorkerPsenosrUpperLeft,
  kPalmReportWorkerPsenosrUpperRight,
  kPalmReportWorkerPsenosrLowerLeft,
  kPalmReportWorkerPsenosrLowerRight,

  kEventCounts,
};

using EventNotifyHandler = std::function<void(EventType, int32_t event_value)>;
using EventHandler = std::function<void(int32_t event_value)>;

enum DeptrumErrorCode {
  kOk = 0,
  kUnknownError = 0x1,
  kNotImplemented = 0x2,
  kInvalidArguments = 0x3,
  kNotSupported = 0x4,
  kFailedToAllocateMemory = 0x5,
  kTransferFailed = 0x20010,
  kConfigFileNotExist = 0x20012,
  kFailedToFindDevices = 0x21001,
  kAccessToNullPointer = 0x21002,
  kFailedToOpenCamera = 0x21003,
  kFailedToCloseCamera = 0x21004,
  kFailedToStartStream = 0x21005,
  kFailedToSetOrGetData = 0x21006,
  kFailedToCheckData = 0x21007,
  kFailedToOpenIrCamera = 0x21008,
  kFailedToOpenRgbCamera = 0x21009,
  kFailedToOperateUsbSerial = 0x2100A,
  kCameraNotRunning = 0x2100B,
  kCameraNotOpened = 0x2100C,
  kFailedToFindDriver = 0x2100D,
  kCameraNotConfigured = 0x2100E,
  kFailedToStoptStream = 0x2100F,
  kDataSizeError = 0x22001,
  kDataNotReady = 0x22002,
  kUnsupportedCameraMode = 0x22004,
  kTimeout = 0x22010,
  kScanModeNotSet = 0x22011,
  kFileNotExist = 0x22100,
  kFailedToOperateFile = 0x22101,
  kFailedToMatchRgbData = 0x22102,
  kUpgradeVersionNotChanged = 0x22103,
  kDeviceIsUpgrading = 0x22104,
  kFailedToUpgrade = 0x22105,
  kFailedToSetExposure = 0x22106,
  kFailedToGetExposure = 0x22107,

  kFailedToInitFaceAlgorithm = 0x23001,
  kFailedToInitDepthAlgorithm = 0x23002,
  kInvalidCalibrationSize = 0x23003,
  kFailedToReadFlash = 0x23004,
  kFailedToGetCalibration = 0x23005,
  kInvalidPath = 0x23006,
  kErrorOccured = 0x23007,
  kFailedToGetLicense = 0x23008,
  kFailedToInitPalmAlgorithm = 0x23009,

  kNoSuchCameraComponent = 0x24001,
  kFailedToGetPreviewFrame = 0x24002,
  kFailedToGetIRframe = 0x24003,
  kFaceCapturing = 0x24004,
  kPreviewOpenFailed = 0x24005,
  KPreviewReadFaided = 0x24006,

  kAlgorithmNotInitialized = 0x25001,
  kStreamNotStarted = 0x25002,
  kDeviceHasOpened = 0x25003,
  kDeviceNotInitialized = 0x25004,
  kDeviceNotInCaptureMode = 0x25005,
  kInvalidFrameFormat = 0x25006
};

typedef struct DeptrumCameraTemperature {
  float temperature_main_board;
  float temperature_led_board;
  float temperature_cpu;
  float temperature_rgb_sensor;
} DeptrumCameraTemperature;

struct StreamDeviceVersionInfo {
  Version camera_sdk_version;    // Camera library version
  Version ir_firmware_version;   // Ir camera firmware version
  Version rgb_firmware_version;  // Rgb camera firmware version
  std::string kernel_version;    // Kernel version
  Version calib_version;         // Calibration version
  Version depth_version;         // Depth algorithm library version
};

struct LightInfo {
  uint8_t light_mode{0xff};
  uint8_t light_color{0xff};
  uint16_t ir_current{0};
};

enum class HintMap {
  kTooClose = 0x1,
  kFaceTooSmall = 0x2,
  kNotCenteredX = 0x4,
  kNotCenteredY = 0x8,
  kAngled = 0x10,
  kCovered = 0x20,
  kMasked = 0x40,
  kIrLiveness = 0x80,
  k3DLiveness = 0x100,
  kBadExpression = 0x200,
  kBlurred = 0x400,  // above match usb serial protocol
  kNeedAe = 0x800,
  kNoFace = 0x1000,
  kk3DLivenessAndMouthMask = 0x2000,
  kOtherDetectError = 0x4000
};

enum DeptrumRecogMode { kRegIrVSIr = 1, kRegIrVSRgb, kRegRgbVSIr, kRegRgbVSRgb, kBiModal };

struct BBox {
  uint16_t x;  // X coordinate of the top-left corner
  uint16_t y;  // Y coordinate of the top-left corner
  uint16_t w;  // Width of the bounding box
  uint16_t h;  // Height of the bounding box
};

struct DeptrumData {
  size_t data_len = 0;            // data_len
  std::shared_ptr<uint8_t> data;  // data
};

struct ExternalInfo {
  uint16_t psnesor_value[4]{0};
  uint32_t palm_roi[4]{0};
  int light_status{0};
  int device_num{0};
};

struct ExtraFrameInfo {
  uint8_t ir_gain = 0;
  uint8_t ir_exp = 0;
  bool ir_exp_mode = false;
  bool is_collect_image = false;
  FaceInfo face_info;
  // debug
  uint16_t prev_exposure{0};
  uint16_t current_exposure{0};
  int32_t rgb_exposure{0};
  int32_t z_angle{0};
  int32_t y_height{0};
  uint64_t calib_timestamp = 0;
  uint64_t capture_timestamp = 0;
  float sensor_temperature{0.0f};
  float driver_temperature{0.0f};
  float rx_ntc{0.0f};
  float tx_ntc{0.0f};

  CustomFrame<uint8_t> raw_frame1;
  // stof debug
  std::list<CustomFrame<uint8_t>> debug_frames;
  OpaqueData<uint8_t> img_speckle;

  OpaqueData<PointXyz<float>> img_depth_sniper_point_cloud;
  OpaqueData<PointXyz<float>> img_depth_stof_magic_point_cloud;
  DeptrumNebulaFrameType frame_type{kFrameRaw};
  bool ae_status{false};

  std::vector<ImuInfo<double>> imu_info_vec;

  uint16_t psensor_value[4]{0};
  uint32_t PalmRoi[4]{0};
  uint8_t light_mode{0};
};

struct StreamFrame {
  int index;                   // Index of the frame
  int size;                    // Frame data size in bytes
  int cols;                    // Number of columns
  int rows;                    // Number of rows
  int bits_per_pixel;          // Number of bits per pixel
  float temperature;           // Driver temperature during this frame
  FrameType frame_type;        // Type of the frame, refer to FrameType
  FrameAttr frame_attr;        // Attributes of the frame, refer to FrameAttr
  ImageFormat image_format;    // Format of the image
  uint64_t timestamp;          // Timestamp of the frame
  std::shared_ptr<void> data;  // Pointer to the frame data
};

struct StreamFrames {
  int count;                                            // Number of frames
  std::vector<std::shared_ptr<StreamFrame>> frame_ptr;  // Pointer to the frames array
  std::shared_ptr<ExtraFrameInfo> extra_info{};         // Extra frames information
};

struct TofPointXyz : public PointXyz<float> {
  float noise{0.0f};
  uint16_t gray_value{0};
  uint8_t confidence{255};
};

template<class T>
struct PointXyzCg {
  T x, y, z;
  float confidence;
  uint16_t gray_value;
  PointXyzCg(T x, T y, T z, float confidence, uint16_t gray_value) :
      x(x),
      y(y),
      z(z),
      confidence(confidence),
      gray_value(gray_value) {}
  PointXyzCg(T x, T y, T z) : PointXyzCg(x, y, z, 0, 0) {}
  PointXyzCg() : PointXyzCg(0, 0, 0, 0, 0) {}
};

template<class T>
struct PointXyzNgc {
  T x, y, z;
  float noise;
  uint16_t gray_value;
  uint8_t confidence;
  PointXyzNgc(T x, T y, T z, float noise, uint16_t gray_value, uint8_t confidence) :
      x(x),
      y(y),
      z(z),
      noise(noise),
      gray_value(gray_value),
      confidence(confidence) {}
  PointXyzNgc(T x, T y, T z) : PointXyzNgc(x, y, z, 0, 0, 255) {}
  PointXyzNgc() : PointXyzNgc(0, 0, 0, 0, 0, 255) {}
};

enum PointFeature {
  kPointFeatureWithZero = 0,     // Full Resolution with zero
  kPointFeatureActualPoint = 1,  // Actual number of points
};

enum PointScale {
  kPointScaleMm = 1,
  kPointScaleM = 1000,
};

enum PointType {
  kPointXy = 0,        // struct PointXy
  kPointXyz = 1,       // struct PointXyz
  kPointXyzAc = 2,     // struct PointXyzAc 240 mipi support
  kPointXyzCg = 3,     // struct PointXyzCg 240 mipi support
  kPointXyzUv = 4,     // struct PointXyzUv
  kPointXyzRgb = 5,    // struct PointXyzRgb
  kPointXyzRgbIr = 6,  // struct PointXyzRgbIr
  kPointXyzNgc = 7,    // struct TofPointXyz 240 mipi support
};

enum LogLevel : int {
  kLogLevelInfo = 0,
  kLogLevelWarning = -1,
  kLogLevelError = -2,
  kLogLevelFatal = -3,
  kLogLevelOff = -9,
};

enum StreamImageResolution {
  kResolutionInvalid = 0,

  kResolution480x768 = 1,
  kResolution432x768 = 1,
  kResolution720x1280 = 2,
  kResolution1080x1920 = 3,
};

enum StreamLightMode {
  kLightAllOff = 1,
  kLightAllOn = 2,
  kLightLaserOnLedOff = 3,
  kLightLaserOffLedOn = 4,
};

class CameraInterface {
 public:
  virtual ~CameraInterface() = default;
  virtual int SetExposure(const uint16_t* exposure, uint32_t size, uint8_t type = 0) = 0;
  virtual int GetExposure(uint16_t* exposure,
                          uint32_t size,
                          uint8_t type = 0) = 0;  // sdk has allocated memory
  virtual int GetSerialNumber(std::string& serial) = 0;
  virtual int ReadCalibFile(const std::string& file_path) = 0;
};

}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_STREAM_TYPES_H_
