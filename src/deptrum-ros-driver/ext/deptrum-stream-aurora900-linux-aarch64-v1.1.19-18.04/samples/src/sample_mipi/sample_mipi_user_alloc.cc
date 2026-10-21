#include <boost/stacktrace.hpp>
#include <csignal>
#include <fstream>
#include <thread>
#include <unordered_map>
#include <vector>
#include "deptrum/device.h"
#include "deptrum/nebula_mipi_series.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "sample_helper.h"
// #include "opencv2/opencv.hpp"
using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

bool is_running = true;
// bool is_save_image = false;
std::string save_image_path = "./";
int32_t save_image_count = 0;
bool is_ae = true;
bool is_printf_fps = true;

uint64_t NowMs() {
  auto nt = std::chrono::high_resolution_clock::now();
  auto n = std::chrono::duration_cast<std::chrono::milliseconds>(nt.time_since_epoch()).count();
  return n;
}

int StreamProcess(std::shared_ptr<Device> device,
                  const std::vector<StreamType>& stream_type,
                  long frame_loop_times) {
  Stream* stream = nullptr;
  int ret = device->CreateStream(stream, stream_type);
  CHECK_SDK_RETURN_VALUE(ret);

  ret = stream->Start();
  CHECK_SDK_RETURN_VALUE(ret);

  // The size of the data will vary depending on the actual data
  // When memory is passed in, but there is no stream, the length is set to 0 at the time of output
  StreamFrames frames;
  auto depth_frame = std::make_shared<StreamFrame>();
  // Set to the maximum resolution 248*180
  depth_frame->cols = 248;
  depth_frame->rows = 180;
  depth_frame->bits_per_pixel = 16;
  depth_frame->size = depth_frame->cols * depth_frame->rows * depth_frame->bits_per_pixel / 8;
  depth_frame->frame_type = FrameType::kDepthFrame;
  // Allocate memory for the data
  uint8_t* depth_buf = new (std::nothrow) uint8_t[depth_frame->size]();
  depth_frame->data = std::shared_ptr<uint8_t>(depth_buf, [](uint8_t* p) {});
  frames.frame_ptr.push_back(depth_frame);

  // auto ir_frame = std::make_shared<StreamFrame>();
  // ir_frame->cols = 248;
  // ir_frame->rows = 180;
  // ir_frame->bits_per_pixel = 8;
  // ir_frame->size = ir_frame->cols * ir_frame->rows * ir_frame->bits_per_pixel / 8;
  // ir_frame->frame_type = FrameType::kIrFrame;
  // ir_frame->data = std::shared_ptr<uint8_t>(new (std::nothrow) uint8_t[ir_frame->size](),
  //                                           std::default_delete<uint8_t[]>());
  // frames.frame_ptr.push_back(ir_frame);

  auto point_frame = std::make_shared<StreamFrame>();
  point_frame->cols = 248;
  point_frame->rows = 180;
  point_frame->frame_type = FrameType::kPointCloudFrame;
  point_frame->size = point_frame->cols * point_frame->rows * sizeof(PointXyzCg<float>);
  uint8_t* point_buf = new (std::nothrow) uint8_t[point_frame->size]();
  point_frame->data = std::shared_ptr<uint8_t>(point_buf, [](uint8_t* p) {});
  point_frame->frame_type = FrameType::kPointCloudFrame;
  frames.frame_ptr.push_back(point_frame);

  frames.count = frames.frame_ptr.size();

  FrameRateHelper frame_rate_helper;
  long cnt = 0;

  while (is_running) {
    if (frame_loop_times == -1) {
    } else {
      if (frame_loop_times == 0)
        break;
      frame_loop_times--;
    }
    // 第一次传入超时时间，若有数据则返回0
    StreamFrames frames_wait;  // 可以传frames，也可以传frames_wait, 当前调用不会修改frames
    ret = stream->GetFramesWithUserAlloc(frames_wait, 2000);
    if (ret != 0) {
      std::cout << "GetFramesWithUserAlloc timeout" << std::endl;
      continue;
    }
    // static uint64_t last_time = NowMs();
    // std::cout << "get frames: " << frames.frame_ptr.size() << "\t; time: " << NowMs() - last_time
    //           << std::endl;
    // last_time = NowMs();
    // 第二次传入超时时间为0，立刻获取数据
    ret = stream->GetFramesWithUserAlloc(frames, 0);
    CHECK_GET_FRAMES(ret);
    // 多次连续调用会返错误码
    // ret = stream->GetFramesWithUserAlloc(frames, 0);
    // CHECK_GET_FRAMES(ret);

    if (save_image_count > 0) {
      for (auto frame : frames.frame_ptr) {
        std::cout << "frame attr: " << frame->frame_attr << std::endl;
        std::cout << "frame type: " << frame->frame_type << " cols: " << frame->cols
                  << " rows: " << frame->rows << " size: " << frame->size
                  << " attr: " << frame->frame_attr << std::endl;
        if (frame->frame_type == FrameType::kPointCloudFrame) {
          std::string save_name = save_image_path + "point_cloud_" +
                                  std::to_string(frame->timestamp) + ".ply";
          SavePointXyzCg(save_name, (char*) frame->data.get(), frame->size);
        } else if (frame->frame_type == FrameType::kDepthFrame) {
          std::string save_name = save_image_path + "depth_" + std::to_string(frame->timestamp) +
                                  ".raw";
          SaveImage(save_name, (char*) frame->data.get(), frame->size);
        }
      }
      save_image_count--;
    }

    if (is_printf_fps) {
      frame_rate_helper.RecordTimestamp();
      if (0 == cnt++ % 10) {
        std::cout << "fps: " << frame_rate_helper.GetFrameRate() << std::endl;
      }
    }
  }

  delete[] depth_buf;
  delete[] point_buf;

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

int PrepareDevice() {
  is_running = true;
  // Register device hotplug callback
  // DeviceManager::EnableLogging("./deptrum_log.txt", true);
  // DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();
  DeviceManager::SetLogLevel(LogLevel::kLogLevelOff);

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
  // std::cout << "enter open device index: 0-" << device_list.size() - 1 << std::endl;
  // std::cin >> dev_index;
  // std::cout << "open dev index is " << dev_index << std::endl;

  // Create a device, 0 means the index of the first device
  std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(
      device_list[dev_index]);
  CHECK_DEVICE_VALID(device);
  std::shared_ptr<NebulaMipi> device_unique_ptr = std::dynamic_pointer_cast<NebulaMipi>(device);

  // Print the sdk version number, the sdk version number is divided into major version number,
  // minor version number and revision number
  auto sdk_version = device->GetSdkVersion();
  std::cout << "SDK version: " << sdk_version << std::endl;

  std::string config_path = "/userdata";
  ret = device_unique_ptr->Open(config_path);
  CHECK_SDK_RETURN_VALUE(ret);
  device_unique_ptr->SetPositionDataType(PointType::kPointXyzCg);
  device_unique_ptr->SetPositionDataFeature(PointFeature::kPointFeatureActualPoint);
  device_unique_ptr->SwitchMtofOrStof(1);  // 0-stof and mtof,1-mtof,2-stof
  device_unique_ptr->SetMtofCropPixels(0, 0);

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
        std::cout << "set and get stof exposure!!" << std::endl;
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
      case 'h': {
        std::cout << "h: help message!!" << std::endl;
        std::cout << "a: printf fps!!" << std::endl;
        std::cout << "q: quit!!" << std::endl;
        std::cout << "e: auto exposure!!" << std::endl;
        std::cout << "f: save image!!" << std::endl;
        std::cout << "g: set and get exposure!!" << std::endl;
        std::cout << "i: get current exposure!!" << std::endl;
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

  is_running = true;
  PrepareDevice();

  return 0;
}
