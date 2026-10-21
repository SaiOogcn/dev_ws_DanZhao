#include <boost/stacktrace.hpp>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
// #include "opencv2/opencv.hpp"
#include <unordered_map>
#include "deptrum/nebula_mipi_series.h"
#include "sample_helper.h"
using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

bool is_running = true;
bool is_printf_fps = true;

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

    if (is_printf_fps) {
      frame_rate_helper.RecordTimestamp();
      if (0 == cnt++ % 10) {
        std::cout << std::this_thread::get_id() << " fps: " << frame_rate_helper.GetFrameRate()
                  << std::endl;
      }
    }
  }

  stream->Stop();
  device->DestroyStream(stream);
  return 0;
}

int PrepareDevice(const DeviceInformation& device_information) {
  // Create a device, 0 means the index of the first device
  std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_information);
  CHECK_DEVICE_VALID(device);

  // Print the sdk version number, the sdk version number is divided into major version number,
  // minor version number and revision number
  auto sdk_version = device->GetSdkVersion();
  std::cout << "SDK version: " << sdk_version << std::endl;

  auto ret = device->Open();
  CHECK_SDK_RETURN_VALUE(ret);

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
  int index = 0;
  ir_mode = std::get<0>(device_resolution_vec[index]);
  rgb_mode = std::get<1>(device_resolution_vec[index]);
  depth_mode = std::get<2>(device_resolution_vec[index]);
  ret = device->SetMode(ir_mode, rgb_mode, depth_mode);
  CHECK_SDK_RETURN_VALUE(ret);

  Intrinsic ir_intri, rgb_intri;
  Extrinsic extrinsic;

  std::shared_ptr<NebulaMipi> device_unique_ptr = std::dynamic_pointer_cast<NebulaMipi>(device);
  DeviceDescription device_info;
  ret = device_unique_ptr->GetDeviceInfo(device_info);
  if (ret) {
    std::cout << "GetDeviceInfo error" << ret << std::endl;
  } else {
    std::cout << "device_name: " << device_info.device_name << std::endl
              << "rgb_firmware_verison: " << device_info.rgb_firmware_version << std::endl
              << "stream_sdk_version: " << device_info.stream_sdk_version << std::endl
              << "tof_firmware_version: " << device_info.ir_firmware_version << std::endl
              << "serial_num: " << device_info.serial_num << std::endl;
  }

  /********************* create stream **************************/
  std::vector<StreamType> stream_types_vector;
  std::thread stream_thread;
  device->GetSupportedStreamType(stream_types_vector);

  long frame_loop_times = -1;
  // std::cout << std::endl;
  // std::cout << "enter the num of frames loops (-1 means always display): ";
  // std::cin >> frame_loop_times;
  // std::cout << std::endl;
  stream_thread = std::thread(StreamProcess, device, stream_types_vector, frame_loop_times);

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

int main() {
  std::signal(SIGINT, shutdown);
  std::signal(SIGSEGV, shutdown);
  std::signal(SIGABRT, shutdown);

  // for (int i = 0; i < 2; i++) {
  //   is_running = true;
  //   PrepareDevice();
  // }

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

  std::vector<std::thread> thread_camera;

  for (int i = 0; i < device_list.size(); i++) {
    std::thread t = std::thread(PrepareDevice, std::cref(device_list[i]));
    thread_camera.push_back(std::move(t));

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  while (is_running) {
    char key = cin.get();
    switch (key) {
      case 'q': {
        is_running = false;
      } break;
      case 'a': {
        is_printf_fps = !is_printf_fps;
      } break;
      case 'h': {
        std::cout << "h: help message!!" << std::endl;
        std::cout << "a: printf fps!!" << std::endl;
        std::cout << "q: quit!!" << std::endl;
      } break;
      default:
        break;
    }
  }

  for (int i = 0; i < thread_camera.size(); i++) {
    if (thread_camera[i].joinable()) {
      thread_camera[i].join();
    }
  }

  return 0;
}
