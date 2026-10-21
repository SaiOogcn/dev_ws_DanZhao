#include <iostream>
#include "deptrum/device.h"
#include "functional/base.h"

#define ESC 27

using namespace deptrum;
using namespace deptrum::stream;

int main(int argc, char** argv) {
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

  std::cout << "Press ESC to exit! " << std::endl;
  while (true) {
    // Get the value of the pressed key, if it is the esc key, exit the program
    int key = getchar();
    if (key == ESC)
      break;
  }

  // Close device
  ret = device->Close();
  CHECK_SDK_RETURN_VALUE(ret);

  return 0;
}