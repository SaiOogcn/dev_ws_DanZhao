// #include <boost/stacktrace.hpp>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
// #include "opencv2/opencv.hpp"
#include "sample_helper.h"
#ifdef DEVICE_TYPE_STELLAR200
#include "deptrum/stellar200_series.h"
#endif
#ifdef DEVICE_TYPE_AURORA300
#include "deptrum/aurora300_series.h"
#endif
#ifdef DEVICE_TYPE_STELLAR400
#include "deptrum/stellar400_series.h"
#endif
#ifdef DEVICE_TYPE_AURORA500
#include "deptrum/aurora500.h"
#endif
#ifdef DEVICE_TYPE_AURORA700
#include "deptrum/aurora700.h"
#endif
#ifdef DEVICE_TYPE_AURORA900
#include "deptrum/aurora900_series.h"
#endif
#ifdef DEVICE_TYPE_NEBULA100
#include "deptrum/nebula100_series.h"
#endif
#if defined DEVICE_TYPE_NEBULA
#include "deptrum/nebula_series.h"
#endif
#include <unordered_map>
using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

CameraParam g_camera_parm;
static bool enable_rgb2yuv_flag = false;
bool is_running = true;
// bool is_save_image = false;
std::string save_image_path = "./";
int32_t save_image_count = 0;
bool is_ae = true;
bool is_printf_fps = true;

std::unordered_map<int, std::string> stream_type_map = {
    {StreamType::kRgb, "Rgb"},
    {StreamType::kIr, "Ir"},
    {StreamType::kDepth, "Depth"},
    {StreamType::kRgbd, "Rgbd"},
    {StreamType::kRgbIr, "RgbIr"},
    {StreamType::kRgbdIr, "RgbdIr"},
    {StreamType::kDepthIr, "DepthIr"},
    {StreamType::kDepthIrLaser, "DepthIrLaser"},
    {StreamType::kSpeckleCloud, "SpeckleCloud"},
    {StreamType::kPointCloud, "PointCloud"},
    {StreamType::kRgbdPointCloud, "RgbdPointCloud"},
    {StreamType::kRgbdIrPointCloud, "RgbdIrPointCloud"},
    {StreamType::kRgbdIrFlag, "RgbdIrFlag"},
    {StreamType::kDepthIrFlag, "DepthIrFlag"}};
void ChooceStreamType(std::shared_ptr<Device> device,
                      std::vector<StreamType>& stream_types_vector) {
  std::cout << std::endl;
  std::cout << "----------------------"
            << " choose stream type "
            << "----------------------" << std::endl;
  int num = 0;
  std::vector<StreamType> device_support_streamtype_vec;

  device->GetSupportedStreamType(device_support_streamtype_vec);

  std::cout << to_string(num++) + ":  end of input or return main menu" << std::endl;
  for (auto stream_type : device_support_streamtype_vec) {
    std::cout << to_string(num++) + ":  " + stream_type_map[stream_type] << std::endl;
  }
  std::cout << std::endl;
  std::cout << "enter your choice (can choose more than one, 0 means end): " << std::endl;
  int input_index = 0;
  while (1) {
    std::cin >> input_index;
    if (input_index == 0)
      break;

    stream_types_vector.emplace_back(device_support_streamtype_vec[input_index - 1]);
  }
  std::cout << "-------------------------------------- chooce stream type "
               "--------------------------------------"
            << std::endl;
}

int StreamProcess(std::shared_ptr<Device> device,
                  const std::vector<StreamType>& stream_type,
                  long frame_loop_times) {
  Stream* stream = nullptr;
  int ret = device->CreateStream(stream, stream_type);
  CHECK_SDK_RETURN_VALUE(ret);

  ret = stream->Start();
  CHECK_SDK_RETURN_VALUE(ret);

  bool fisrt_get_frame = true;
  bool running = true, fps = false;
  StreamFrames frames;
  FrameRateHelper frame_rate_helper;
  long cnt = 0;

  while (is_running) {
    if (frame_loop_times == -1) {
    } else {
      if (frame_loop_times == 0)
        break;
      frame_loop_times--;
    }
    ret = stream->GetFrames(frames, 2000);
    CHECK_GET_FRAMES(ret);

    if (save_image_count > 0) {
      SaveOneFrame(save_image_path, frames);
      save_image_count--;
      if (save_image_count == 0) {
        std::cout << "save image done!" << std::endl;
      }
    }

    if (is_printf_fps) {
      frame_rate_helper.RecordTimestamp();
      if (0 == cnt++ % 10) {
        std::cout << "fps: " << frame_rate_helper.GetFrameRate() << std::endl;
      }
    }
  }

  stream->Stop();
  device->DestroyStream(stream);
  return 0;
}

#if defined DEVICE_TYPE_NEBULA
int NebulaStofExposureTest(std::shared_ptr<Device> device) {
  std::cout << "input stof exposure:" << std::endl;
  int32_t exposure_value{0};
  std::cin >> exposure_value;
  auto ret = std::dynamic_pointer_cast<Nebula>(device)->SetNebulaExposure(
      exposure_value,
      DeptrumNebulaFrameType::kFrameStof);
  std::cout << "set stof exposure: " << exposure_value << "; ret: " << ret << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  exposure_value = 0;
  ret = std::dynamic_pointer_cast<Nebula>(device)->GetNebulaExposure(
      exposure_value,
      DeptrumNebulaFrameType::kFrameStof);
  std::cout << "get stof exposure: " << exposure_value << "; ret: " << ret << std::endl;

  return 0;
}

int NebulaMtofExposureTest(std::shared_ptr<Device> device) {
  std::cout << "input mtof exposure:" << std::endl;
  int32_t exposure_value{0};
  std::cin >> exposure_value;
  auto ret = std::dynamic_pointer_cast<Nebula>(device)->SetNebulaExposure(
      exposure_value,
      DeptrumNebulaFrameType::kFrameMtof);
  std::cout << "set mtof exposure: " << exposure_value << "; ret: " << ret << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  exposure_value = 0;
  ret = std::dynamic_pointer_cast<Nebula>(device)->GetNebulaExposure(
      exposure_value,
      DeptrumNebulaFrameType::kFrameMtof);
  std::cout << "get mtof exposure: " << exposure_value << "; ret: " << ret << std::endl;

  return 0;
}

int NebulaSetStofAeRoiMode(std::shared_ptr<Device> device) {
  std::cout << "input roi mode: 0: near; 1: middle; 2: far" << std::endl;
  int32_t ae_roi_mode{0};
  std::cin >> ae_roi_mode;
  auto ret = std::dynamic_pointer_cast<Nebula>(device)->SetStofAeRoiMode(
      DeptrumNebulaAeRoiMode(ae_roi_mode));
  std::cout << "set ae roi mode ret: " << ret << std::endl;
  return 0;
}

int NebulaSetTriggerMode(std::shared_ptr<Device> device) {
  std::cout << "input trigger mode: 0: slave; 1: master" << std::endl;
  int32_t trigger_mode{0};
  std::cin >> trigger_mode;
  auto ret = std::dynamic_pointer_cast<Nebula>(device)->SetSensorTriggerMode(trigger_mode);
  std::cout << "set trigger mode ret: " << ret << std::endl;
  return 0;
}

int NebulaSetCalibrationMode(std::shared_ptr<Device> device) {
  std::cout << "input calibration mode: 0: disable; 1: enable" << std::endl;
  int32_t mode{0};
  std::cin >> mode;
  auto ret = std::dynamic_pointer_cast<Nebula>(device)->SetCalibrationMode(mode);
  std::cout << "set calibration mode ret: " << ret << std::endl;
  return 0;
}

int NebulaSetExpectLedAmplitude(std::shared_ptr<Device> device) {
  std::cout
      << "input led_amp 2000-3000, The higher the value, the lower the expected Led amplitude."
      << std::endl;
  int32_t led_amp{0};
  std::cin >> led_amp;
  auto ret = std::dynamic_pointer_cast<Nebula>(device)->SetExpectLedAmplitude(led_amp);
  std::cout << "SetExpectLedAmplitude ret: " << ret << std::endl;
  return 0;
}
#endif

#ifdef DEVICE_TYPE_STELLAR400_MIPI
int Stellar400ExposureTest(std::shared_ptr<Device> device) {
  std::cout << "input exposure:" << std::endl;
  int32_t exposure_value{0};
  std::cin >> exposure_value;
  auto ret = std::dynamic_pointer_cast<Stellar400>(device)->SetExposure(CameraComponent::kIrCamera,
                                                                        exposure_value);
  std::cout << "set exposure: " << exposure_value << "; ret: " << ret << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  exposure_value = 0;
  ret = std::dynamic_pointer_cast<Stellar400>(device)->GetExposure(CameraComponent::kIrCamera,
                                                                   exposure_value);
  std::cout << "get exposure: " << exposure_value << "; ret: " << ret << std::endl;

  return 0;
}

int GetStellar400Exposure(std::shared_ptr<Device> device) {
  int32_t exposure_value{0};
  auto ret = std::dynamic_pointer_cast<Stellar400>(device)->GetExposure(CameraComponent::kIrCamera,
                                                                        exposure_value);
  std::cout << "get exposure: " << exposure_value << "; ret: " << ret << std::endl;

  return 0;
}
#endif

int PrepareDevice() {
  is_running = true;
  // Register device hotplug callback
  DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

  // Query the list of connected devices
  std::vector<DeviceInformation> device_list;
  int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
  CHECK_SDK_RETURN_VALUE(ret);
  // Check device count
  CHECK_DEVICE_COUNT(device_list.size());
  {
    printf("devices info:\n");
    for (int index = 0; index < device_list.size(); index++) {
      printf("index(%d):\t device_addr:%d usb_port(%s)\n",
             index,
             device_list[index].ir_camera.device_addr,
             device_list[index].ir_camera.port_path.c_str());
    }
  }
  int dev_index = 0;
  std::cout << "enter open device index: 0-" << device_list.size() - 1 << std::endl;
  std::cin >> dev_index;
  std::cout << "open dev index is " << dev_index << std::endl;

  // Create a device, 0 means the index of the first device
  std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(
      device_list[dev_index]);
  CHECK_DEVICE_VALID(device);

  // Print the sdk version number, the sdk version number is divided into major version number,
  // minor version number and revision number
  auto sdk_version = device->GetSdkVersion();
  std::cout << "SDK version: " << sdk_version << std::endl;

  ret = device->Open();
  CHECK_SDK_RETURN_VALUE(ret);

  // Get device serial
  // std::string serial;
  // ret = device->GetSerialNumber(serial);
  // CHECK_SDK_RETURN_VALUE(ret);
  // std::cout << "Serial number: " << serial << std::endl;

  // Get device name
  std::string device_name = device->GetDeviceName();
  std::cout << "Device name: " << device_name << std::endl;

#if defined DEVICE_TYPE_NEBULA
  // set crop before set config
  // std::dynamic_pointer_cast<Nebula>(device)->SetMtofCropPixels(0, 0);
#endif

  FrameMode ir_mode, rgb_mode, depth_mode;
  std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
  device->GetSupportedFrameMode(device_resolution_vec);
  int index = ChooceFrameMode(device_resolution_vec);
  ir_mode = std::get<0>(device_resolution_vec[index]);
  rgb_mode = std::get<1>(device_resolution_vec[index]);
  depth_mode = std::get<2>(device_resolution_vec[index]);
  ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
  CHECK_SDK_RETURN_VALUE(ret);

  if (device_name == "Aurora930" || device_name == "Aurora932" || device_name == "Aurora936") {
#ifdef DEVICE_TYPE_AURORA900
    std::shared_ptr<Aurora900> device_unique_ptr = std::dynamic_pointer_cast<Aurora900>(device);
    int ir_fps = 0;
    std::cout << std::endl;
    std::cout << "enter ir fps: ";
    std::cin >> ir_fps;
    std::cout << std::endl;
    device_unique_ptr->SetIrFps(ir_fps);
#endif
  }

  Intrinsic ir_intri, rgb_intri;
  Extrinsic extrinsic;
  if (device_name == "Stellar200") {
#ifdef DEVICE_TYPE_STELLAR200
    std::shared_ptr<Stellar200> device_unique_ptr = std::dynamic_pointer_cast<Stellar200>(device);
#endif
  } else if (device_name == "Aurora300") {
#ifdef DEVICE_TYPE_AURORA300
    std::shared_ptr<Aurora300> device_unique_ptr = std::dynamic_pointer_cast<Aurora300>(device);
#endif
  } else if (device_name == "Stellar400" || device_name == "Stellar420") {
#ifdef DEVICE_TYPE_STELLAR400
    std::shared_ptr<Stellar400> device_unique_ptr = std::dynamic_pointer_cast<Stellar400>(device);
    DeviceDescription device_info;
    device_unique_ptr->GetDeviceInfo(device_info);
    printf("tof_firmware_version: %s    rgb firmware version: %s\n",
           device_info.ir_firmware_version.c_str(),
           device_info.rgb_firmware_version.c_str());
    std::cout << std::endl;
    device->GetCameraParameters(ir_intri, rgb_intri, extrinsic);
    g_camera_parm.cx = ir_intri.principal_point[0];
    g_camera_parm.cy = ir_intri.principal_point[1];
    g_camera_parm.fx = ir_intri.focal_length[0];
    g_camera_parm.fy = ir_intri.focal_length[1];

    // 开启心跳功能
    HeartbeatParam heartbeat_params;
    heartbeat_params.allowable_failures_counts = 8;
    heartbeat_params.timeout = 500;
    heartbeat_params.is_enabled = false;
    device_unique_ptr->SetHeartbeat(heartbeat_params, [](int flag) {
      if (flag != 1) {
        printf("flag: %d\n", flag);
      }
    });

    device_unique_ptr->EnableUndistortRgb(true);
#endif
  } else if (device_name == "Aurora930" || device_name == "Aurora932" ||
             device_name == "Aurora936") {
#ifdef DEVICE_TYPE_AURORA900
    std::shared_ptr<Aurora900> device_unique_ptr = std::dynamic_pointer_cast<Aurora900>(device);
    std::cout << std::endl;
    int align_mode_i = 0;
    bool align_mode = true;
    std::cout << "confirm align mode 1:true 0:false: ";
    std::cin >> align_mode_i;
    align_mode = align_mode_i == 1 ? true : false;
    std::cout << std::endl;
    device_unique_ptr->SwitchAlignedMode(align_mode);
    if (align_mode == true) {
      int depth_correct_i = 0;
      bool depth_correct = true;
      std::cout << "confirm depth correct 1:true 0:false: ";
      std::cin >> depth_correct_i;
      depth_correct = depth_correct_i == 1 ? true : false;
      std::cout << std::endl;
      device_unique_ptr->DepthCorrection(depth_correct);
    }

    // 开启心跳功能
    HeartbeatParam heartbeat_params;
    heartbeat_params.allowable_failures_counts = 5;
    heartbeat_params.timeout = 200;
    heartbeat_params.is_callback_every_failure = true;
    heartbeat_params.is_enabled = true;
    device_unique_ptr->SetHeartbeat(heartbeat_params, [](int flag) {
      if (flag != 1) {
        printf("heartbeat result: %d\n", flag);
      }
    });
#endif
  } else if (device_name == "Nebula100") {
#ifdef DEVICE_TYPE_NEBULA100
    std::shared_ptr<Nebula100> device_unique_ptr = std::dynamic_pointer_cast<Nebula100>(device);
    device_unique_ptr->SwitchSlamMode(2);
#endif
  } else if (device_name == "Nebula" || device_name == "Nebula220 mipi" ||
             device_name == "Nebula220") {
#if defined DEVICE_TYPE_NEBULA
    std::shared_ptr<Nebula> device_unique_ptr = std::dynamic_pointer_cast<Nebula>(device);
    DeviceDescription device_info;
    ret = device_unique_ptr->GetDeviceInfo(device_info);
    if (ret) {
      std::cout << "GetDeviceInfo error" << ret << std::endl;
    }
    std::cout << "device_name: " << device_info.device_name << std::endl
              << "rgb_firmware_verison: " << device_info.rgb_firmware_version << std::endl
              << "stream_sdk_version: " << device_info.stream_sdk_version << std::endl
              << "tof_firmware_version: " << device_info.ir_firmware_version << std::endl
              << "serial_num: " << device_info.serial_num << std::endl;
#if 0
    int mtof_or_stof_flag = 0;
    std::cout << "confirm mtof or stof 0:mtof and stof 1:mtof 2:stof:" << std::endl;
    std::cin >> mtof_or_stof_flag;
    device_unique_ptr->SwitchMtofOrStof(mtof_or_stof_flag);
#endif

    NebulaSetCalibrationMode(device);
#endif
  }

  /********************* create stream **************************/
  std::vector<StreamType> stream_types_vector;
  std::thread stream_thread;
  ChooceStreamType(device, stream_types_vector);
  if (stream_types_vector.empty())
    return 0;
  if (device_name == "Stellar420s") {
#ifdef DEVICE_TYPE_STELLAR400
    //     std::shared_ptr<Stellar400> device_unique_ptr =
    std::shared_ptr<Stellar400> device_unique_ptr = std::dynamic_pointer_cast<Stellar400>(device);
    int index = 0;
    for (index = 0; index < stream_types_vector.size(); index++) {
      if (stream_types_vector[index] == kRgbd || stream_types_vector[index] == kRgbdIr ||
          stream_types_vector[index] == kRgbdIrPointCloud) {
        device_unique_ptr->SetUndistortion(true);
        device_unique_ptr->EnableUndistortRgb(true);
        break;
      }
    }

    if (index >= stream_types_vector.size()) {
      int enable_undistortion = 0;
      std::cout << std::endl;
      std::cout << "set undistortion(other: false, 1:true): ";
      std::cin >> enable_undistortion;
      std::cout << std::endl;
      if (enable_undistortion) {
        device_unique_ptr->SetUndistortion(true);
        device_unique_ptr->EnableUndistortRgb(true);
      } else {
        device_unique_ptr->SetUndistortion(false);
        device_unique_ptr->EnableUndistortRgb(false);
      }
    }
#endif
  }
  long frame_loop_times = -1;
  std::cout << std::endl;
  std::cout << "enter the num of frames loops (-1 means always display): ";
  std::cin >> frame_loop_times;
  std::cout << std::endl;
  stream_thread = std::thread(StreamProcess, device, stream_types_vector, frame_loop_times);

  while (is_running) {
    char key = cin.get();
    switch (key) {
      case 'q': {
        is_running = false;
      } break;
      case 'a': {
        is_printf_fps = !is_printf_fps;
      } break;
      case 'f': {
        std::cout << "save image:" << std::endl;
        std::cout << "input the image save path: " << std::endl;
        std::cin >> save_image_path;
        save_image_path += "/";
        std::cout << "image_save_path=" << save_image_path << std::endl;
        std::cout << "input the image save num: " << std::endl;
        std::cin >> save_image_count;
        std::cout << "image_save_count=" << save_image_count << std::endl;

        // is_save_image = !is_save_image;
      } break;
#if defined DEVICE_TYPE_NEBULA
      case 'e': {
        std::cout << "set ae status: " << is_ae << std::endl;
        std::dynamic_pointer_cast<Nebula>(device)->SwitchAutoExposure(
            is_ae,
            DeptrumNebulaFrameType::kFrameMtof);
        std::dynamic_pointer_cast<Nebula>(device)->SwitchAutoExposure(
            is_ae,
            DeptrumNebulaFrameType::kFrameStof);
        is_ae = !is_ae;
      } break;
      case 'g': {
        std::cout << "set and get stof exposure!!" << std::endl;
        NebulaStofExposureTest(device);
      } break;
      case 'j': {
        std::cout << "set and get mtof exposure!!" << std::endl;
        NebulaMtofExposureTest(device);
      } break;
      case 'k': {
        NebulaSetStofAeRoiMode(device);
      } break;
      case 'l': {
        NebulaSetTriggerMode(device);
      } break;
      case 'm': {
        NebulaSetCalibrationMode(device);
      } break;
      case 'n': {
        NebulaSetExpectLedAmplitude(device);
      } break;
#endif
#ifdef DEVICE_TYPE_STELLAR400_MIPI
      case 'e': {
        std::cout << "set ae status: " << is_ae << std::endl;
        std::dynamic_pointer_cast<Stellar400>(device)->SwitchAutoExposure(is_ae);
        is_ae = !is_ae;
      } break;
      case 'g': {
        std::cout << "set and get exposure!!" << std::endl;
        Stellar400ExposureTest(device);
      } break;
      case 'i': {
        std::cout << "get current exposure!!" << std::endl;
        GetStellar400Exposure(device);
      } break;
#endif
      case 'h': {
        std::cout << "h: help message!!" << std::endl;
        std::cout << "a: printf fps!!" << std::endl;
        std::cout << "q: quit!!" << std::endl;
        std::cout << "e: auto exposure!!" << std::endl;
        std::cout << "f: save image!!" << std::endl;
        std::cout << "g: set and get stof exposure!!" << std::endl;
        std::cout << "i: get current exposure!!" << std::endl;
        std::cout << "j: set and get mtof exposure!!" << std::endl;
        std::cout << "k: set stof ae roi mode!!" << std::endl;
        std::cout << "l: set trigger mode!!" << std::endl;
        std::cout << "m: set calibration!!" << std::endl;
        std::cout << "n: set expect led amp!!" << std::endl;
      } break;
      default:
        break;
    }
  }

  if (stream_thread.joinable()) {
    stream_thread.join();
  }
  ret = device->Close();
  std::cout << "Close device ret=" << ret << std::endl;
  device.reset();
  return 0;
}

// void shutdown(int signal) {
//   if (signal == SIGINT || signal == SIGABRT || signal == SIGSEGV) {
//     // LOG_F(ERROR, "recv shutdown signal!!!!");
//     std::cout << "recv shutdown signal!!!!" << std::endl;
//     std::cout << boost::stacktrace::stacktrace();
//     exit(-1);
//   }
// }

int main() {
  // std::signal(SIGINT, shutdown);
  // std::signal(SIGSEGV, shutdown);
  // std::signal(SIGABRT, shutdown);
  for (int i = 0; i < 2; i++) {  // check open device
    is_running = true;
    is_ae = true;
    is_printf_fps = true;
    PrepareDevice();
  }

  return 0;
}
