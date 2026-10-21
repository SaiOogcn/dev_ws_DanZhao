#include "face_device.h"
#include <thread>
// #include "sqlite3.h"
#include <string.h>
#include <memory>
#include <thread>
#include "face_arithmetic.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"
#include "stream.h"
#include "stream_types.h"

namespace deptrum {
namespace stream {
CameraParam g_camera_parm;

FaceDevice::~FaceDevice() {
  std::cout << "[Test] facula device destory" << std::endl;
}

FaceDevice::FaceDevice(DeviceInformation device_info) : device_info_(device_info) {
#ifndef DISABLE_INTERFACE
  viewer_.reset(new ViewerHelper("aurora910"));
#endif
  result_flag_.store(false);

  callback_ = [&](const stream::CaptureFaceResult& result) {
    result_flag_.store(false);
    std::unique_lock<std::mutex> lock(mutex_);
    capture_result_ = result;
    lock.unlock();
    if (result.capture_handler_status == stream::CaptureStatus::kTimeout) {
      std::cout << "time out " << std::endl;
    }
    // live face preview
    if (result.capture_handler_status == stream::CaptureStatus::kFaceDetected &&
        result.live_face_errors == (uint32_t) stream::LiveFaceErrors::kLiveFaceOk) {
      std::cout << "face detect " << std::endl;
#ifndef DISABLE_INTERFACE
      std::string save_path = std::to_string(
          std::chrono::system_clock::now().time_since_epoch().count());
      result_flag_.store(true);
#endif
    }
  };
}

int FaceDevice::Create() {
  std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_info_);
  device_ = std::dynamic_pointer_cast<Aurora900>(device);
  int ret = FaceCapture::Create(device_, &face_);
  if (ret != 0)
    std::cout << "Create device error" << ret << std::endl;
  return ret;
}

void FaceDevice::Open() {
  if (!device_) {
    std::cout << "[Test] Create device first" << std::endl;
    return;
  }
  if (is_open_) {
    std::cout << "[Test] already open device" << std::endl;
    return;
  }
  int ret = device_->Open();
  // Set camera support mode
  FrameMode ir_mode, rgb_mode, depth_mode;
  std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
  device_->SetMode(kInvalid, kInvalid, kInvalid);
  is_open_ = true;
}

void FaceDevice::Start() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  auto func = [&] {
    Stream* stream;
    int ret = device_->CreateStream(stream, {kRgb, kIr});
    // stream->RegisterFrameCb(callTest);
    ret = stream->Start();

    StreamFrames frames;
#ifndef DISABLE_INTERFACE
    std::shared_ptr<ViewerHelper> viewer_manage = std::make_shared<ViewerHelper>("aurora910");
#endif
    FrameRateHelper frame_rate_helper;
    bool fisrt_get_frame = true;
    long cnt = 0;
    while (is_open_) {
      ret = stream->GetFrames(frames, 2000);
#ifndef DISABLE_INTERFACE

      viewer_manage->ShowFrame(frames,
                               capture_result_.face_bbox.x,
                               capture_result_.face_bbox.y,
                               capture_result_.face_bbox.w,
                               capture_result_.face_bbox.h,
                               g_camera_parm);
#endif
      if (is_printf_fps) {
        frame_rate_helper.RecordTimestamp();
        if (0 == cnt++ % 10) {
          std::cout << "fps: " << frame_rate_helper.GetFrameRate() << std::endl;
        }
      }
    }
    is_start_ = true;
  END:
    stream->Stop();
    device_->DestroyStream(stream);
#ifndef DISABLE_INTERFACE
    viewer_manage.reset();
#endif
  };
  std::thread(func).detach();

#ifndef DISABLE_INTERFACE
  auto img_func = [&] {
    while (is_open_ && device_) {
      if (result_flag_.load()) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (capture_result_.img_rgb.get() != nullptr)
          viewer_->ShowRgbImage((uint8_t*) capture_result_.img_rgb->data,
                                capture_result_.img_rgb->rows,
                                capture_result_.img_rgb->cols,
                                save_picture_,
                                ".rgb.png",
                                capture_result_.face_bbox.x,
                                capture_result_.face_bbox.y,
                                capture_result_.face_bbox.w,
                                capture_result_.face_bbox.h,
                                capture_result_.kps);
        if (capture_result_.img_ir.get() != nullptr)
          viewer_->ShowU8Image((uint8_t*) capture_result_.img_ir->data,
                               capture_result_.img_ir->rows,
                               capture_result_.img_ir->cols,
                               save_picture_,
                               ".ir.png",
                               capture_result_.face_bbox.x,
                               capture_result_.face_bbox.y,
                               capture_result_.face_bbox.w,
                               capture_result_.face_bbox.h,
                               capture_result_.kps);
        if (capture_result_.img_depth.get() != nullptr)
          viewer_->ShowDepthImage((uint8_t*) capture_result_.img_depth->data,
                                  capture_result_.img_depth->rows,
                                  capture_result_.img_depth->cols,
                                  save_picture_,
                                  ".depth.png",
                                  capture_result_.face_bbox.x,
                                  capture_result_.face_bbox.y,
                                  capture_result_.face_bbox.w,
                                  capture_result_.face_bbox.h,
                                  capture_result_.kps);
        result_flag_.store(false);
        lock.unlock();
      }
    }
    viewer_.reset();
  };
  std::thread(img_func).detach();
#endif
}

void FaceDevice::StartFaceCapture() {
  if (!is_open_) {
    std::cout << "[Test] Open device first" << std::endl;
    return;
  }
  face_->StartFaceCapture(callback_, 1000 * 15);
  return;
}

void FaceDevice::CaptureFaceOnce() {
  if (!is_open_) {
    std::cout << "[Test] Open device first" << std::endl;
    return;
  }
  face_->CaptureFaceOnce(callback_, 1000 * 15);
}

void FaceDevice::StopFaceCapture() {
  if (!is_open_) {
    std::cout << "[Test] Open device first" << std::endl;
    return;
  }

  face_->StopFaceCapture();
  std::cout << "[Test] stop face capture success" << std::endl;
}

void FaceDevice::SetFaceRecognitionThrehold() {
  if (!is_open_) {
    std::cout << "[Test] Open device first" << std::endl;
    return;
  }
  std::cout << "enter threhold(0~1): " << std::endl;
  float threhold;
  std::cin >> threhold;
  if (!face_->SetFaceRecognitionThrehold(threhold)) {
    std::cout << "SetFaceRecognitionThrehold succeed !" << std::endl;
  }
}

void FaceDevice::Close() {
  if (!is_open_) {
    std::cout << "[Test] Open device first" << std::endl;
    return;
  }

  is_open_ = false;
  int ret = device_->Close();
  if (ret == kOk) {
    std::cout << "[Test] close device success" << std::endl;
  } else {
    std::cout << "close device failed" << std::endl;
  }
}

void FaceDevice::GetSerialNumber() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }

  std::string serial_num;
  int ret = device_->GetSerialNumber(serial_num);
  if (ret == kOk) {
    std::cout << "serial num: " << serial_num << std::endl;
  } else {
    std::cout << "[Test] GetSerialNumber error" << std::endl;
  }
}

void FaceDevice::GetAlgorithmVersion() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  std::string face_info;
  int ret = face_->GetAlgorithmVersion(AlgorithmType::kFaceAlgorithm, face_info);
  if (ret == kOk) {
    std::cout << "Face Algorithm: " << std::endl;
    std::cout << face_info << std::endl;
  } else {
    std::cout << "[Test] GetAlgorithmVersion face error" << std::endl;
  }

  std::string depth_info;
  if (face_->GetAlgorithmVersion(AlgorithmType::kDepthAlgorithm, depth_info) == kOk) {
    std::cout << "Depth Algorithm: " << std::endl;
    std::cout << depth_info << std::endl;
  } else {
    std::cout << "[Test] GetAlgorithmVersion depth error" << std::endl;
  }
}

void FaceDevice::ShowInfo() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  std::cout << "name: " << device_info_.model
            << ", port path: " << device_info_.ir_camera.port_path;
  std::cout << " , serial num: " << device_info_.ir_camera.serial_number << std::endl;
  std::cout << std::endl;
}

void FaceDevice::SetFaceFeaturesFromCapture() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  std::cout << "enter: 0:disable, 1:enable " << std::endl;
  bool enable;
  std::cin >> enable;
  int ret = face_->SetFaceFeaturesFromCapture(enable);
}

void FaceDevice::IsPrintfFps() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  is_printf_fps = !is_printf_fps;
}

void FaceDevice::DepthGenSet() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  depth_gen_ = !depth_gen_;
  int ret = face_->DepthGenSet(depth_gen_);
}

void FaceDevice::CompareFaceFromJpeg() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  std::string rgb_path_1 = "";
  std::string rgb_path_2 = "";
  std::cout << "Input source rgb img " << std::endl;
  std::cin >> rgb_path_1;
  std::cout << "Input target rgb img " << std::endl;
  std::cin >> rgb_path_2;
  float score;
  face_->CompareFaceFromJpeg(score, rgb_path_1, rgb_path_2);
  std::cout << "score: " << score << std::endl;
}

void FaceDevice::GetCameraTemperature() {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  int16_t temperature;
  int ret = device_->GetCameraTemperature(stream::kTemperatureCamera, &temperature);
  if (ret) {
    std::cout << "GetCameraTemperature failed,ret = " << ret << std::endl;
  } else {
    std::cout << "temperature:" << temperature << std::endl;
  }
}

void FaceDevice::SetExposure(int mode, int exposure) {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  int ret = kUnknownError;
  if (mode == 1) {
    ret = device_->SetExposure(CameraComponent::kLaser, exposure);
  } else if (mode == 2) {
    ret = device_->SetExposure(CameraComponent::kLed, exposure);
  } else {
    ret = kInvalidArguments;
  }
  if (ret) {
    std::cout << "SetExposure failed,ret = " << ret << std::endl;
  } else {
    std::cout << "SetExposure: " << exposure << "success!" << std::endl;
  }
}

void FaceDevice::SetGain(int mode, int gain) {
  if (!is_open_) {
    std::cout << "[Test] open device first" << std::endl;
    return;
  }
  int ret = kUnknownError;
  if (mode == 1) {
    ret = device_->SetGain(CameraComponent::kLaser, gain);
  } else if (mode == 2) {
    ret = device_->SetGain(CameraComponent::kLed, gain);
  } else {
    ret = kInvalidArguments;
  }
  if (ret)
    std::cout << "SetGain failed,ret = " << ret << std::endl;
  else
    std::cout << "SetGain: " << gain << " success!" << std::endl;
}

}  // namespace stream
}  // namespace deptrum
