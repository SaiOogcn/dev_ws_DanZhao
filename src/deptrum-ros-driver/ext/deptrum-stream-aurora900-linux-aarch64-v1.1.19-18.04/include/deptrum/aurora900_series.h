/*********************************************************************
 *  Copyright (c) 2018-2023 Shenzhen Guangjian Technology Co.,Ltd..  *
 *                     All rights reserved.                          *
 *********************************************************************/

#ifndef DEPTRUM_STREAM_INCLUDE_DEPTRUM_AURORA900_SERIES_H_
#define DEPTRUM_STREAM_INCLUDE_DEPTRUM_AURORA900_SERIES_H_

#include "device.h"

namespace deptrum {
namespace stream {

class STREAM_DLL Aurora900 : public Device {
 public:
  /********************************************************************
  Constructor & Destructor
  ********************************************************************/
  using Device::Device;
  virtual ~Aurora900() = default;

  /**
   * Set ir fps.
   * for 930
   * @return Zero on success, error code otherwise.
   */

  virtual int SetIrFps(int ir_fps) = 0;

  /**
   * Reboot the device.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int Reboot() = 0;

  /**
   * Update the device.
   *
   * @param[in] update_file_path the path of update file.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int Upgrade(const std::string& update_file_path,
                      ProgressHandler progress_handler = nullptr) = 0;

  /**
   * SoftResetDevice Reset when there is no data transfer
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SoftResetDevice() = 0;

  /**
   * Set camera auto exposure mode.
   * @param[in] component Camera component to modify.
   * @param[in] auto exposure mode.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetAEMode(const CameraComponent& component, uint8_t mode) = 0;

  /**
   * Set camera auto exposure mode.
   * @param[in] component Camera component to query.
   * @param[out] auto exposure mode.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetAEMode(const CameraComponent& component, uint8_t& mode) = 0;

  /**
   * Switch auto exposure, in frames.
   *
   * @param[in] enable: 0: close, 1: open.
   * @return 0 on success, error code otherwise.
   */
  virtual int SwitchAutoExposure(bool enable) = 0;

  /**
   * Set camera gain .
   * @param[in] component Camera component to modify.
   * @param[in] gain.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetGain(const CameraComponent& component, uint32_t gain) = 0;

  /**
   * Set camera gain .
   * @param[in] component Camera component to query.
   * @param[out] gain.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetGain(const CameraComponent& component, uint32_t& gain) = 0;

  /**
   * SetFaceArea Set camera auto exposure window.
   *
   * @param[in] face_area roi
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetFaceArea(Bbox face_area) = 0;

  /**
   * Get support info.
   *
   * @param[in/out] support_info
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetSupportInfo(SupportedInfo& support_info) = 0;

  /**
   * Get camera temperature.
   *
   * @param[in] temp_type temperature type
   * @param[out] temperature
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetCameraTemperature(TemperatureType temp_type, int16_t* temperature) = 0;

  /**
   * Roll back to the previous firmware version
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int RollBack() = 0;

  /**
   * Get the version of Deptrum camera.
   *
   * @param[out] version.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetDeviceInfo(DeviceDescription& device_info) = 0;

  /**
   * Enable depth ir mirroring.
   *
   * @param[in] enable.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int EnableDepthIrMirroring(bool enable) = 0;

  /**
   * Align rgbd_ir.
   *
   * @param[in] enable. default false
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SwitchAlignedMode(bool mode = false) = 0;

  /**
   * Set remove filter size.
   *
   * @param[in] size Threshold size of noise removal setting
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetRemoveFilterSize(uint32_t size = 110) = 0;

  /**
   * Correct depth or not.
   *
   * @param[in] depth_correction true:correct depth false:not correct depth
   *
   */
  virtual void DepthCorrection(bool depth_correction) = 0;

  /**
   * Filter depth range map.
   *
   * @param[in] min true:minimum filter range,default 150.
   *
   * @param[in] max true:maximum filter range,default 4000.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int FilterOutRangeDepthMap(int min = 150, int max = 4000) = 0;

  /**
   * Set Laser Driver.
   *
   * @param[in] laser driver.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetLaserDriver(float laser_driver) = 0;

  /**
   * Set Filter OutBorder Depth.
   *
   * @param[in] border_depth border_depth.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetFilterOutBorderDepth(int border_depth) = 0;

  /**
   * Set Depth Range.
   *
   * @param[in] min_range Min depth range.
   * @param[in] max_range Min depth range.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetDepthRange(int min_range, int max_range) = 0;

  /**
   * SetHeartbeat
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetHeartbeat(const HeartbeatParam& heartbeat_params,
                           HeartbeatResult heartbeat_callback = nullptr) = 0;

  /**
   * Set image resolution.
   *
   * @param[in] image_resolution image resolution.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetStreamImageResolution(StreamImageResolution image_resolution) = 0;

  /**
   * EnableScanCodeMode
   *
   * @param[in] enalbe
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int EnableScanCodeMode(bool enable = true) = 0;

  /**
   * StopHeartbeat
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int StopHeartbeat() = 0;

  /**
   * Set laser current.
   *
   * @param[in] current the value of current.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetLaserCurrent(int current) = 0;

  /**
   * Get laser current.
   *
   * @param[out] current the value of current.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int GetLaserCurrent(int& current) = 0;

  /**
   * Set light mode.
   *
   * @param[in] light_mode the value of light mode.
   *
   * @return Zero on success, error code otherwise.
   */
  virtual int SetLightMode(StreamLightMode light_mode) = 0;

  /**
   * Enable undistortion to rgb
   *
   * @param[in] Undistort Rgb
   *
   * @return 0 if success, error code otherwise.
   */
  virtual int EnableUndistortRgb(bool enable) = 0;
};

}  // namespace stream
}  // namespace deptrum
#endif  // DEPTRUM_STREAM_INCLUDE_DEPTRUM_AURORA900_SERIES_H_
