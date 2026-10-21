#ifndef TEST_FACULA_AURORA910_H_
#define TEST_FACULA_AURORA910_H_

#include <iostream>
#include <memory>
#include "aurora900_series.h"
#include "face_device.h"
#include "utils.h"
#ifndef DISABLE_INTERFACE
#include "viewer_help.hpp"
#endif
#include <condition_variable>
#include <mutex>

namespace deptrum {
namespace stream {

class FaceDevice {
 public:
  FaceDevice(DeviceInformation device_info);
  ~FaceDevice();
  int Create();
  void Open();
  void CaptureFaceOnce();
  void StartFaceCapture();
  void StopFaceCapture();
  void Close();
  void GetSerialNumber();
  void GetAlgorithmVersion();
  void GetCameraTemperature();
  void SetExposure(int mode, int exposure);
  void SetGain(int mode, int gain);
  void Start();
  void SetFaceRecognitionThrehold();
  void ShowInfo();
  void SetSaveFacePicture(bool save_picture) { save_picture_ = save_picture; }
  void SetFaceFeaturesFromCapture();
  void IsPrintfFps();
  void DepthGenSet();
  void CompareFaceFromJpeg();

 private:
  bool is_open_ = false;
  bool is_start_ = false;
  bool is_printf_fps = true;
  bool depth_gen_ = false;
  DeviceInformation device_info_;
#ifndef DISABLE_INTERFACE
  std::shared_ptr<ViewerHelper> viewer_;
#endif

  std::shared_ptr<stream::Aurora900> device_;

  std::shared_ptr<stream::FaceCapture> face_;
  std::function<void(const stream::CaptureFaceResult& result)> callback_;
  bool save_picture_ = false;
  stream::CaptureFaceResult capture_result_;
  std::mutex mutex_;
  std::atomic<bool> result_flag_;
};

}  // namespace stream
}  // namespace deptrum

#endif  // TEST_FACULA_AURORA910_H_
