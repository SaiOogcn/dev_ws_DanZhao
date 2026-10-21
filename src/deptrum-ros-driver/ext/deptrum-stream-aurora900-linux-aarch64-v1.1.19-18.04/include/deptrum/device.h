/*********************************************************************
 *  Copyright (c) 2018-2024 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_DEPTRUM_DEVICE_H_
#define DEPTRUM_STREAM_INCLUDE_DEPTRUM_DEVICE_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include "stream_types.h"

namespace deptrum {
namespace stream {
class Stream;

class STREAM_DLL Device {
 public:
  virtual ~Device() = default;
  /**
   * Open device in configed frame mode.
   *
   * @param[in] config_path the path of config file.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int Open(const std::string& config_path = "") = 0;

  /**
   * Open device with camera interface callback.
   *
   * @param[in] camera_interface: camera interface callback.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int Open(std::shared_ptr<CameraInterface> camera_interface);

  /**
   * Close device.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int Close() = 0;

  /**
   * Get the serial number of the device.
   *
   * @param[out] sn Serial number of the device.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetSerialNumber(std::string& sn) = 0;

  /**
   * Get the information of the device.
   *
   * @param[out] device_information Information of the Device.
   *
   * @return void
   */
  virtual void GetDeviceInfo(DeviceInformation& device_information) = 0;

  /**
   * Set the information of the device.
   *
   * @param[in] device_information Information of the Device.
   *
   * @return void
   */
  virtual void SetDeviceInfo(const DeviceInformation& device_information) = 0;

  /**
   * Set the frame mode, include resolution, format and decode mode.
   *
   * @param[in] ir_mode Ir frame mode, reference to FrameMode.
   * @param[in] rgb_mode Rgb frame mode, reference to FrameMode.
   * @param[in] depth_mode Depth frame mode, reference to FrameMode.
   * @param[in] ir_decode_mode Ir decode mode, reference to FrameDecodeMode.
   * @param[in] rgb_decode_mode Rgb decode mode, reference to FrameDecodeMode.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetMode(const FrameMode& ir_mode = kInvalid,
                      const FrameMode& rgb_mode = kInvalid,
                      const FrameMode& depth_mode = kInvalid,
                      const FrameDecodeMode& ir_decode_mode = kSoftwareDecode,
                      const FrameDecodeMode& rgb_decode_mode = kSoftwareDecode) = 0;

  /**
   * Get the sdk version.
   *
   * @return String of the sdk version.
   */
  virtual std::string GetSdkVersion() = 0;

  /*
   * Get the intrinsic and extrinsic parameters of camera.
   *
   * @param[out] ir_intrinsic Intrinsic parameters of ir camera.
   * @param[out] rgb_intrinsic Intrinsic parameters of rgb camera.
   * @param[out] extrinsic Extrinsic parameters of the cameras.
   * @return Zero on success, error code otherwise.
   */
  virtual int GetCameraParameters(Intrinsic& ir_intrinsic,
                                  Intrinsic& rgb_intrinsic,
                                  Extrinsic& extrinsic) = 0;

  /**
   * Get the device name.
   *
   * @return String of the device name.
   */
  virtual std::string GetDeviceName() = 0;

  /*--------------------stream-------------------*/
  /**
   * Create a stream according to the stream type.
   *
   * @param[out] stream Secondary pointer to the stream object to be created.
   * @param[in]  types Types of the stream, reference to StreamType.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int CreateStream(Stream*& stream, const std::vector<StreamType>& types) = 0;

  /**
   * Destroy a stream.
   *
   * @param[in] stream Pointer to the stream object to be destroyed.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int DestroyStream(Stream*& stream) = 0;

  /**
   * Restart the device.
   *
   * @param[in] device_information Information of the device to be restarted.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int RestartDevice(const DeviceInformation device_information) = 0;

  /**
   * Stop the device.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int StopDevice() = 0;

  /**
   * Check the device status, opened or not.
   *
   * @return Bool true: opened, false: closed.
   */
  virtual bool IsDeviceOpened() = 0;

  /**
   * Get the supported frame mode of the device.
   *
   * @param[out] framemode Frame mode supported by the device.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetSupportedFrameMode(
      std::vector<std::tuple<FrameMode, FrameMode, FrameMode>>& device_resolution_vec) = 0;

  /**
   * Get the supported stream type of the device.
   *
   * @param[out] stream_type Stream type supported by the device.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetSupportedStreamType(std::vector<StreamType>& device_streamtype_vec) = 0;

  /**
   * Set the exposure time of the camera.
   *
   * @param[in] component Camera component, just support ir camera or rgb camera.
   * @param[in] exposure_in_us Exposure time in microseconds.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetExposure(const CameraComponent& component, int exposure_in_us) = 0;

  /**
   * Get the exposure time for the camera.
   *
   * @param[in] component Camera component, just support ir camera or rgb camera.
   * @param[out] exposure_in_us Exposure time in microseconds.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetExposure(const CameraComponent& component, int& exposure_in_us) = 0;
};

class STREAM_DLL DeviceManager {
 public:
  /**
   * Create a device management singleton class.
   *
   * @return smart ptr for device manager
   */
  static std::shared_ptr<DeviceManager> GetInstance();

#ifndef __ANDROID__

  /**
   * Record the sdk log to a specific directory when it is enable.
   *
   * @param[in] path Storage path of the sdk log.
   * @param[in] enable True:turn on, false:turn off.
   *
   * @return void
   */
  static void EnableLogging(const std::string& path, bool enable);

  /**
   * Set log level.
   *
   * @param[in] level Log level
   *
   * @return void
   */
  static void SetLogLevel(LogLevel level);

  /**
   * Get all device lists connected to host.
   *
   * @param[out] device_list Pointer to device list array, reference to DeviceInformation.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetDeviceList(std::vector<DeviceInformation>& device_list) = 0;
#else
  /**
   * Set log tag for android.
   *
   * @param[in] log_tag Log tag for android.
   *
   * @return void
   */
  static void SetLogTag(const std::string& log_tag);
#endif

  /**
   * Register device hot plug.
   *
   * @param[in] handler Callback handler.
   * @param[in] enable_hotplug Default true, false :disable hot plug.
   *
   * @return void
   */
  virtual void RegisterDeviceConnectedCallback(
      std::function<void(int flag, const DeviceInformation& device_information)> handle = nullptr,
      bool enable_hotplug = true) = 0;

  /**
   * Create a device object.
   *
   * @param[in] device_information Device information.
   *
   * @return Pointer to the device object.
   */
  virtual std::shared_ptr<Device> CreateDevice(const DeviceInformation& device_information) = 0;

  /**
   * Create a device object using usb port.
   *
   * @param[in] usb port.
   *
   * @return smart ptr for device
   */
  virtual std::shared_ptr<Device> CreateDeviceByUsbPort(std::string usb_port) = 0;
};

}  // namespace stream
}  // namespace deptrum

#endif  // DEPTRUM_STREAM_INCLUDE_DEPTRUM_DEVICE_H_
