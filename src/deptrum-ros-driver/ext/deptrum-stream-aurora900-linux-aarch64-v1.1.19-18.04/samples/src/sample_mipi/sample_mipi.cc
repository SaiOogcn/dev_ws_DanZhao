#include <boost/stacktrace.hpp>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
// #include "opencv2/opencv.hpp"
#include <unordered_map>
#include "deptrum/nebula_mipi_series.h"
#include "record_playback.h"
#include "sample_helper.h"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

bool is_running = true;
// bool is_save_image = false;
std::string save_image_path = "./data/";
std::string record_path = "/tmp/record";
int32_t save_image_count = 0;
bool is_ae = true;
bool is_printf_fps = true;

std::thread save_image_thread;
std::queue<StreamFrames> stream_frames_queue;
std::mutex save_image_mutex;
std::condition_variable save_image_cv;

uint16_t mtof_crop_up = 50;
uint16_t mtof_crop_down = 80;

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
  FrameRateHelper frame_rate_helper;
  long cnt = 0;

  while (is_running) {
    if (frame_loop_times == -1) {
    } else {
      if (frame_loop_times == 0)
        break;
      frame_loop_times--;
    }
    StreamFrames frames;
    ret = stream->GetFrames(frames, 2000);
    // CHECK_GET_FRAMES(ret);
    if (ret != 0) {
      continue;
    }

    // for (int i = 0; i < frames.count; i++) {
    //   std::cout << "frame id: " << frames.frame_ptr[i]->index
    //             << "\ttimestamp: " << frames.frame_ptr[i]->timestamp
    //             << "\tattr: " << frames.frame_ptr[i]->frame_attr
    //             << "\ttype: " << frames.frame_ptr[i]->frame_type
    //             << "\tsize: " << frames.frame_ptr[i]->size << std::endl;
    // }

    if (is_printf_fps) {
      frame_rate_helper.RecordTimestamp();
      if (0 == cnt++ % 10) {
        std::cout << "fps: " << frame_rate_helper.GetFrameRate() << std::endl;
      }
    }

    if (save_image_count > 0) {
      std::lock_guard<std::mutex> lock(save_image_mutex);
      stream_frames_queue.emplace(std::move(frames));
      save_image_cv.notify_one();
      save_image_count--;
      if (save_image_count == 0) {
        std::cout << "save image done!" << std::endl;
      }
    }
  }

  stream->Stop();
  device->DestroyStream(stream);
  return 0;
}

int NebulaExposureTest(std::shared_ptr<Device> device) {
  std::cout << "input stof exposure:" << std::endl;
  int32_t stof_exp_input{0};
  std::cin >> stof_exp_input;
  std::cout << "input mtof exposure:" << std::endl;
  int32_t mtof_exp_input{0};
  std::cin >> mtof_exp_input;
  auto ret = std::dynamic_pointer_cast<NebulaMipi>(device)->SetNebulaExposure(stof_exp_input,
                                                                              mtof_exp_input);
  std::cout << "set exposure ret: " << ret << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  int stof_exp = 0;
  int mtof_exp = 0;
  ret = std::dynamic_pointer_cast<NebulaMipi>(device)->GetNebulaExposure(stof_exp, mtof_exp);
  std::cout << "get stof exposure: " << stof_exp << "; ret: " << ret << std::endl;
  std::cout << "get mtof exposure: " << mtof_exp << "; ret: " << ret << std::endl;

  return 0;
}

int NebulaSetStofAeRoiMode(std::shared_ptr<Device> device) {
  std::cout << "input roi mode: 0: near; 1: middle; 2: far" << std::endl;
  int32_t ae_roi_mode{0};
  std::cin >> ae_roi_mode;
  auto ret = std::dynamic_pointer_cast<NebulaMipi>(device)->SetStofAeRoiMode(
      DeptrumNebulaAeRoiMode(ae_roi_mode));
  std::cout << "set ae roi mode ret: " << ret << std::endl;
  return 0;
}

int NebulaSetTriggerMode(std::shared_ptr<Device> device) {
  std::cout << "input trigger mode: 0: slave; 1: master" << std::endl;
  int32_t trigger_mode{0};
  std::cin >> trigger_mode;
  auto ret = std::dynamic_pointer_cast<NebulaMipi>(device)->SetSensorTriggerMode(trigger_mode);
  std::cout << "set trigger mode ret: " << ret << std::endl;
  return 0;
}

int NebulaSetCalibrationMode(std::shared_ptr<Device> device) {
  std::cout << "input calibration mode: 0: disable; 1: enable" << std::endl;
  int32_t mode{0};
  std::cin >> mode;
  auto ret = std::dynamic_pointer_cast<NebulaMipi>(device)->SetCalibrationMode(mode);
  std::cout << "set calibration mode ret: " << ret << std::endl;
  return 0;
}

int NebulaSetExpectLedAmplitude(std::shared_ptr<Device> device) {
  std::cout
      << "input led_amp 2000-3000, The higher the value, the lower the expected Led amplitude."
      << std::endl;
  int32_t led_amp{0};
  std::cin >> led_amp;
  auto ret = std::dynamic_pointer_cast<NebulaMipi>(device)->SetExpectLedAmplitude(led_amp);
  std::cout << "SetExpectLedAmplitude ret: " << ret << std::endl;
  return 0;
}

int ExportIntrinsicToJsonFile(std::shared_ptr<Device> device) {
  std::string file_name{"tof_intrinsic.json"};
  auto ret = std::dynamic_pointer_cast<NebulaMipi>(device)->ExportIntrinsicToJsonFile(file_name);
  std::cout << "ExportIntrinsicToJsonFile ret: " << ret << std::endl;
  return 0;
}

int PrepareDevice() {
  is_running = true;
  // Register device hotplug callback
  DeviceManager::EnableLogging("./deptrum_log.txt", true);
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

  // Used to save the config file, avoid repeating flash readings at boot.
  // std::string config_path = "/data";
  // ret = device->Open(config_path);
  ret = device->Open();
  CHECK_SDK_RETURN_VALUE(ret);

  std::dynamic_pointer_cast<NebulaMipi>(device)->SetMtofCropPixels(mtof_crop_up, mtof_crop_down);
  std::shared_ptr<Record> record = std::make_shared<Record>(record_path, device);

  // Get device serial
  // std::string serial;
  // ret = device->GetSerialNumber(serial);
  // CHECK_SDK_RETURN_VALUE(ret);
  // std::cout << "Serial number: " << serial << std::endl;

  // Get device name
  std::string device_name = device->GetDeviceName();
  std::cout << "Device name: " << device_name << std::endl;

  FrameMode ir_mode, rgb_mode, depth_mode;
  std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> device_resolution_vec;
  device->GetSupportedFrameMode(device_resolution_vec);
  int index = ChooceFrameMode(device_resolution_vec);
  ir_mode = std::get<0>(device_resolution_vec[index]);
  rgb_mode = std::get<1>(device_resolution_vec[index]);
  depth_mode = std::get<2>(device_resolution_vec[index]);
  ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
  CHECK_SDK_RETURN_VALUE(ret);

  Intrinsic ir_intri, rgb_intri;
  Extrinsic extrinsic;

  if (device_name == "Nebula200" || device_name == "Nebula220 mipi" || device_name == "Nebula220") {
    std::shared_ptr<NebulaMipi> device_unique_ptr = std::dynamic_pointer_cast<NebulaMipi>(device);
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
    NebulaSetCalibrationMode(device);

    // 这里需要根据具体需要的格式进行设置。目前支持kPointXyzAc/kPointXyzNgc/kPointXyzCg
    std::dynamic_pointer_cast<NebulaMipi>(device)->SetPositionDataType(PointType::kPointXyzAc);
    // std::dynamic_pointer_cast<NebulaMipi>(device)->SetPositionDataType(PointType::kPointXyzNgc);
    // std::dynamic_pointer_cast<NebulaMipi>(device)->SetPositionDataType(PointType::kPointXyzCg);
  }

  /********************* create stream **************************/
  std::vector<StreamType> stream_types_vector;
  std::thread stream_thread;
  ChooceStreamType(device, stream_types_vector);
  if (stream_types_vector.empty())
    return 0;

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
      case 'e': {
        std::cout << "set ae status: " << is_ae << std::endl;
        std::dynamic_pointer_cast<NebulaMipi>(device)->SwitchAutoExposure(is_ae);
        is_ae = !is_ae;
      } break;
      case 'g': {
        std::cout << "set and get exposure!!" << std::endl;
        NebulaExposureTest(device);
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
      case 'p': {
        ExportIntrinsicToJsonFile(device);
      } break;
      case 'x': {
        std::cout << "resume record raw data!!" << std::endl;
        record->Resume();
      } break;
      case 'y': {
        std::cout << "pause record raw data!!" << std::endl;
        record->Pause();
      } break;
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
        std::cout << "x: resume record raw data!!" << std::endl;
        std::cout << "y: pause record raw data!!" << std::endl;
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

void shutdown(int signal) {
  if (signal == SIGINT || signal == SIGABRT || signal == SIGSEGV) {
    // LOG_F(ERROR, "recv shutdown signal!!!!");
    std::cout << "recv shutdown signal!!!!" << std::endl;
    std::cout << boost::stacktrace::stacktrace();
    exit(-1);
  }
}

void SaveImageThread() {
  while (is_running) {
    std::unique_lock<std::mutex> lock(save_image_mutex);
    auto status = save_image_cv.wait_for(lock, std::chrono::milliseconds(100), []() {
      return !stream_frames_queue.empty();
    });

    if (!status) {
      continue;
    }

    auto frame = stream_frames_queue.front();
    stream_frames_queue.pop();
    lock.unlock();

    SaveOneFrame(save_image_path, frame);
  }

  std::cout << "SaveImageThread exit!!" << std::endl;
}

int main(int argc, char* argv[]) {
  std::signal(SIGINT, shutdown);
  std::signal(SIGSEGV, shutdown);
  std::signal(SIGABRT, shutdown);

  if (argc == 3) {
    mtof_crop_up = atoi(argv[1]);
    mtof_crop_down = atoi(argv[2]);

    std::cout << "mtof_crop_up=" << mtof_crop_up << std::endl;
    std::cout << "mtof_crop_down=" << mtof_crop_down << std::endl;
  }

  for (int i = 0; i < 2; i++) {
    is_running = true;
    save_image_thread = std::thread(SaveImageThread);
    PrepareDevice();

    if (save_image_thread.joinable()) {
      save_image_thread.join();
    }
  }

  return 0;
}
