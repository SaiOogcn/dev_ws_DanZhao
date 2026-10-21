#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include "face_device.h"
#include "utils.h"
// #include <loguru.hpp>

using namespace deptrum;
using namespace deptrum::stream;

void PrintUsageInformation() {
  std::flush(std::cout);
  std::cout << "--------------------------------------------------------------------" << std::endl;
  std::cout << "Press following key to set corresponding feature:" << std::endl;
  std::cout << "q: Program (q)uit." << std::endl;
  std::cout << "h: Print the menu." << std::endl;
  std::cout << "c: (C)reate device." << std::endl;
  std::cout << "o: (O)pen device." << std::endl;
  std::cout << "a: Capture face once." << std::endl;
  std::cout << "b: Capture face continuously." << std::endl;
  std::cout << "S: (S)top face capture." << std::endl;
  std::cout << "e: Clos(e) device." << std::endl;
  std::cout << "F: Set Face Recognition Threhold." << std::endl;
  std::cout << "f: Get Face Recognition Threhold." << std::endl;
  std::cout << "p: Is Printf Fps." << std::endl;
  std::cout << "d: enable depth." << std::endl;
  std::cout << "x: Get serial number." << std::endl;
  std::cout << "C: Compare Face From Jpeg." << std::endl;
}

void callback(int flag, const DeviceInformation& info) {
  printf("callback flag: %d\n", flag);
}

std::shared_ptr<deptrum::stream::FaceDevice> face_device = nullptr;

void CreateDevices() {
  std::vector<DeviceInformation> device_list;
  int ret = DeviceManager::GetInstance()->GetDeviceList(device_list);
  if (ret) {
    std::cout << "[Test] Get Device List failed ! ret = " << ret << std::endl;
  }
  printf("devices info:\n");
  for (int index = 0; index < device_list.size(); index++) {
    printf("index(%d):\t device_addr:%d usb_port(%s)\n",
           index,
           device_list[index].ir_camera.device_addr,
           device_list[index].ir_camera.port_path.c_str());
  }
  std::function<void(int, const DeviceInformation&)> handle = callback;

  DeviceManager::GetInstance()->RegisterDeviceConnectedCallback(handle, true);
  face_device = std::make_shared<stream::FaceDevice>(device_list[0]);
  ret = face_device->Create();
  std::cout << "name: " << device_list[0].model
            << ", port path: " << device_list[0].ir_camera.port_path;
  std::cout << " , serial num: " << device_list[0].ir_camera.serial_number << std::endl;
  std::cout << std::endl;
}

int main() {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  PrintUsageInformation();
  int8_t key = '\0';
  while (key != 'q') {
    std::cin >> key;
    switch (key) {
      case 'c':
        CreateDevices();
        break;
      case 'o':
        face_device->Open();
        break;
      case 's':
        face_device->Start();
        break;
      case 'b':
        face_device->StartFaceCapture();
        break;
      case 'a':
        face_device->CaptureFaceOnce();
        break;
      case 'S':
        face_device->StopFaceCapture();
        break;
      case 'x':
        face_device->GetSerialNumber();
        break;
      case 'e':
        face_device->Close();
        break;
      case 'F':
        face_device->SetFaceRecognitionThrehold();
        break;
      case 'f':
        face_device->SetFaceFeaturesFromCapture();
        break;
      case 'p':
        face_device->IsPrintfFps();
        break;
      case 'd':
        face_device->DepthGenSet();
        break;
      case 'C':
        face_device->CompareFaceFromJpeg();
        break;
      case 'h':
        PrintUsageInformation();
        break;
    }
  }
  //::remove_callback("everything.log");
  return 0;
}
