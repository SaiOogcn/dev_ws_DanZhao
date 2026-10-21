#include <iostream>
#include <thread>
#include <unordered_map>
#include <vector>
#include "deptrum/device.h"
#include "functional/base.h"

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
#include "deptrum/nebula100.h"
#endif
#ifdef DEVICE_TYPE_NEBULA
#include "deptrum/nebula_series.h"
#endif

using namespace std;
using namespace deptrum;
using namespace deptrum::stream;

void callback(int flag, const DeviceInformation& info) {
  printf("callback flag: %d\n", flag);
}

int PrepareDevice(string& path) {
  std::function<void(int, const DeviceInformation&)> handle = callback;

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

  // Create a device, 0 means the index of the first device
  std::shared_ptr<Device> device = DeviceManager::GetInstance()->CreateDevice(device_list[0]);
  CHECK_DEVICE_VALID(device);

  // Print the sdk version number, the sdk version number is divided into major version number,
  // minor version number and revision number
  auto sdk_version = device->GetSdkVersion();
  std::cout << "SDK version: " << sdk_version << std::endl;

  // Open device
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

  if (device_name == "Stellar400" || device_name == "Stellar420" || device_name == "Stellar420s") {
#ifdef DEVICE_TYPE_STELLAR400
    std::shared_ptr<Stellar400> device_unique_ptr = std::dynamic_pointer_cast<Stellar400>(device);
    DeviceDescription device_info;
    device_unique_ptr->GetDeviceInfo(device_info);
    printf("tof_firmware_version: %s    rgb firmware version: %s\n",
           device_info.ir_firmware_version.c_str(),
           device_info.rgb_firmware_version.c_str());
    std::cout << std::endl;
    device_unique_ptr->Upgrade(path, [](int progress) {
      std::cout << "progress=" << progress << std::endl;
    });
#endif
  } else if (device_name == "Aurora930" || device_name == "Aurora931" ||
             device_name == "Aurora932" || device_name == "Aurora936") {
#ifdef DEVICE_TYPE_AURORA900
    std::shared_ptr<Aurora900> device_unique_ptr = std::dynamic_pointer_cast<Aurora900>(device);
    device_unique_ptr->Upgrade(path, [](int progress) {
      std::cout << "progress=" << progress << std::endl;
    });
#endif
  } else if (device_name == "Nebula200") {
#ifdef DEVICE_TYPE_NEBULA
    // std::shared_ptr<Nebula200> device_unique_ptr = std::dynamic_pointer_cast<Nebula200>(device);

    // device_unique_ptr->Upgrade(
    //     "C:\\Users\\ybg\\Desktop\\artosyn-firmware-update-0.0.20-p41752-e720683c.sign.tar.gz",
    //     [](int progress) { std::cout << "progress=" << progress << std::endl; });
#endif
  }

  // Close device
  ret = device->Close();
  CHECK_SDK_RETURN_VALUE(ret);
  return 0;
}

int main(int argc, char* argv[]) {
  std::cout << "Number of arguments: " << argc << std::endl;
  std::cout << "Program name: " << argv[0] << std::endl;

  if (argc != 2) {
    std::cout << "Please pass in the correct parameters: " << std::endl;
    std::cout << "like this: ./upgrade "
                 "/path/Desktop/firmware-2.2.2.0.img"
              << std::endl;
    return 0;
  }
  std::cout << "Arguments: ";
  for (int i = 1; i < argc; ++i) {
    std::cout << argv[i] << " ";
  }

  std::cout << std::endl;
  std::string path(argv[1]);
  PrepareDevice(path);
  return 0;
}
