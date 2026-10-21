#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/stacktrace.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <regex>
#include <thread>
#include <unordered_map>
#include <vector>
#include "base/visualizer.h"
#include "deptrum/device.h"
#include "deptrum/nebula_mipi_series.h"
#include "deptrum/stream.h"
#include "functional/base.h"
#include "functional/frame_rate_helper.h"
#include "opencv2/opencv.hpp"
#include "sample_helper.h"

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

std::string g_config_path_;
std::string g_data_path_;
std::string g_serial_;
std::string g_result_dir_{"./result"};
int32_t g_mtof_crop_up_{50};
int32_t g_mtof_crop_down_{80};

bool is_running = true;
// bool is_save_image = false;
std::string save_image_path = "./";
int32_t save_image_count = 0;
bool is_ae = true;
bool is_printf_fps = true;

class CameraInterfaceImpl : public CameraInterface {
 public:
  CameraInterfaceImpl() = default;
  virtual ~CameraInterfaceImpl() = default;
  int SetExposure(const uint16_t* exposure, uint32_t size, uint8_t type = 0) {
    // for (int i = 0; i < size; i++) {
    //   std::cout << "exposure[" << i << "]:" << exposure[i] << std::endl;
    // }
    return 0;
  };
  int GetExposure(uint16_t* exposure, uint32_t size, uint8_t type = 0) { return 0; }
  int GetSerialNumber(std::string& serial) {
    serial = g_serial_;
    return 0;
  }
  int ReadCalibFile(const std::string& file_path) {
    std::string calib_path = "/tmp/" + g_serial_ + "/patch/usrdata/config/";
    if (boost::filesystem::exists(calib_path)) {
      boost::filesystem::remove_all(calib_path);
    }
    boost::filesystem::create_directories(calib_path);
    std::string cmd = "cp " + g_config_path_ + "/* " + calib_path + " -a";
    std::cout << "cmd:" << cmd << std::endl;
    system(cmd.c_str());
    return 0;
  }
};

void ChooceStreamType(std::shared_ptr<Device> device,
                      std::vector<StreamType>& stream_types_vector) {
  device->GetSupportedStreamType(stream_types_vector);
}

void GetFileNameList(std::vector<std::string>& file_name,
                     std::vector<std::string>& file_path,
                     const std::string& input_str) {
  file_name.clear();
  file_path.clear();
  // Get file name
  boost::filesystem::path input_path(input_str);
  boost::filesystem::path dir_path = input_path.parent_path();
  boost::regex filter(input_path.filename().string());

  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(dir_path); itr != end_itr; itr++) {
    if (!boost::filesystem::is_regular_file(itr->status())) {
      continue;
    }

    boost::cmatch what;
    if (!boost::regex_search(itr->path().filename().string().c_str(), what, filter)) {
      continue;
    }
    // // Ensure the file has a "raw" suffix
    if (itr->path().extension() != ".raw") {
      continue;
    }
    file_name.emplace_back(itr->path().filename().string());
  }

  // Sort ascending
  std::sort(file_name.begin(), file_name.end(), [](const std::string& a, const std::string& b) {
    std::regex number_regex("^[0-9]+$");
    std::string a_num = a.substr(0, a.find_first_of("_"));
    std::string b_num = b.substr(0, b.find_first_of("_"));

    if (std::regex_match(a_num, number_regex) && std::regex_match(b_num, number_regex)) {
      return std::stoi(a_num) < std::stoi(b_num);
    } else {
      return a < b;  // Fallback to lexicographical comparison
    }
  });

  // Get absolute path
  for (auto& file : file_name) {
    file_path.emplace_back(input_path.parent_path().string() + "/" + file);
  }
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

  std::string data_path = g_data_path_;
  std::vector<std::string> file_name;
  std::vector<std::string> file_path;
  GetFileNameList(file_name, file_path, data_path);

  auto tof_raw_paths = file_path;
  for (int n = 0; n < tof_raw_paths.size(); n++) {
    std::replace(tof_raw_paths[n].begin(), tof_raw_paths[n].end(), '\\', '/');
    std::string filename = tof_raw_paths[n].substr(tof_raw_paths[n].find_last_of("/") + 1);
    std::cout << "filename: " << filename << std::endl;

    ifstream fin(tof_raw_paths[n], ifstream::binary);
    if (!fin) {
      std::cout << "Error: can't open file: " << tof_raw_paths[n] << std::endl;
      getchar();
      return -1;
    }
    // get file len
    fin.seekg(0, fin.end);
    int phase_size = fin.tellg();
    fin.seekg(0, fin.beg);
    std::shared_ptr<uint8_t> tof_raw_frames = std::shared_ptr<uint8_t>(
        new uint8_t[phase_size],
        std::default_delete<uint8_t[]>());
    fin.read(reinterpret_cast<char*>(tof_raw_frames.get()), phase_size);
    fin.close();

    // read from file

    StreamFrame frame;
    frame.data = tof_raw_frames;
    frame.size = phase_size;
    if (phase_size == 240 * 2894 * 2) {
      frame.cols = 240;
      frame.rows = 2894;
    } else {
      continue;
    }
    frame.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
    std::dynamic_pointer_cast<NebulaMipi>(device)->EnqueueStreamFrame(frame);

    for (int i = 0; i < 2; i++) {
      ret = stream->GetFrames(frames, 300);
      if (ret == 0) {
        for (int i = 0; i < frames.count; i++) {
          if (frames.frame_ptr[i]->frame_type == FrameType::kDepthFrame) {
            cv::Mat depth_img(frames.frame_ptr[i]->rows,
                              frames.frame_ptr[i]->cols,
                              CV_16UC1,
                              frames.frame_ptr[i]->data.get());
            if ((frames.frame_ptr[i]->frame_attr & FrameAttr::kAttrStof) != 0) {
              std::string depth_path = g_result_dir_ + "/stof_depth_" +
                                       std::to_string(frames.frame_ptr[i]->index) + ".png";
              cv::imwrite(depth_path, depth_img);
            } else {
              std::string depth_path = g_result_dir_ + "/mtof_depth_" +
                                       std::to_string(frames.frame_ptr[i]->index) + ".png";
              cv::imwrite(depth_path, depth_img);
            }
          } else if (frames.frame_ptr[i]->frame_type == FrameType::kPointCloudFrame) {
            if ((frames.frame_ptr[i]->frame_attr & FrameAttr::kAttrStof) != 0) {
              std::string point_path = g_result_dir_ + "/stof_point_" +
                                       std::to_string(frames.frame_ptr[i]->index) + ".ply";
              SavePointXyzAc(point_path,
                             (char*) frames.frame_ptr[i]->data.get(),
                             frames.frame_ptr[i]->size);
            } else {
              std::string point_path = g_result_dir_ + "/mtof_point_" +
                                       std::to_string(frames.frame_ptr[i]->index) + ".ply";
              SavePointXyzAc(point_path,
                             (char*) frames.frame_ptr[i]->data.get(),
                             frames.frame_ptr[i]->size);
            }
          }
        }
        // SaveOneFrame(g_result_dir_, frames);
      } else {
        // std::cout << "get frame failed" << ret << std::endl;
      }
    }
  }

  stream->Stop();
  device->DestroyStream(stream);

  std::cout << "stream process done" << std::endl;
  return 0;
}

int PrepareDevice() {
  is_running = true;
  // Register device hotplug callback
  DeviceManager::EnableLogging("./deptrum_log.txt", true);
  DeviceManager::GetInstance()->RegisterDeviceConnectedCallback();

  DeviceInformation device_info;
  std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_info);
  CHECK_DEVICE_VALID(device);

  // Print the sdk version number, the sdk version number is divided into major version number,
  // minor version number and revision number
  auto sdk_version = device->GetSdkVersion();
  std::cout << "SDK version: " << sdk_version << std::endl;

  std::shared_ptr<CameraInterfaceImpl> camera = std::shared_ptr<CameraInterfaceImpl>(
      new CameraInterfaceImpl());
  auto ret = std::dynamic_pointer_cast<NebulaMipi>(device)->Open(camera);
  CHECK_SDK_RETURN_VALUE(ret);
  std::dynamic_pointer_cast<NebulaMipi>(device)->SetPositionDataScaled(PointScale::kPointScaleMm);
  std::dynamic_pointer_cast<NebulaMipi>(device)->SetPositionDataType(PointType::kPointXyzAc);
  std::dynamic_pointer_cast<NebulaMipi>(device)->SetMtofCropPixels(g_mtof_crop_up_,
                                                                   g_mtof_crop_down_);

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
  {
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
  }

  /********************* create stream **************************/
  std::vector<StreamType> stream_types_vector;
  std::thread stream_thread;
  ChooceStreamType(device, stream_types_vector);
  if (stream_types_vector.empty())
    return 0;

  long frame_loop_times = -1;
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

int CreateDirectoy(const std::string& path) {
  int pos = path.find_first_of("/");
  std::string dir_path = "";
  while (!boost::filesystem::exists(path)) {
    pos = path.find("/", pos + 1);
    dir_path = path.substr(0, pos);
    if (!boost::filesystem::exists(dir_path)) {
      if (!boost::filesystem::create_directory(dir_path)) {
        std::cout << "create directory " << dir_path << " failed!" << std::endl;
        return -1;
      }
    }
  }
  return 0;
}

int main(int argc, char* argv[]) {
  std::signal(SIGINT, shutdown);
  std::signal(SIGSEGV, shutdown);
  std::signal(SIGABRT, shutdown);

  if (argc != 6) {
    printf("\n\
        ***********************************\n\
        Please input the parameters on the command line.\n\
        ./sample_mipi_file path_to_config path_to_data serial mtof_crop_up mtof_crop_down\n\
        eg: ./sample_mipi_file ./config ./data 12345678 50 80\n\
        eg: ./sample_mipi_file ./config ./data 12345678 0 0\n\
        ***********************************\n");
    return -1;
  }

  g_config_path_ = argv[1];
  g_data_path_ = argv[2];
  g_serial_ = argv[3];
  g_mtof_crop_up_ = atoi(argv[4]);
  g_mtof_crop_down_ = atoi(argv[5]);

  boost::filesystem::remove_all(g_result_dir_);
  CreateDirectoy(g_result_dir_);

  is_running = true;
  PrepareDevice();

  return 0;
}
