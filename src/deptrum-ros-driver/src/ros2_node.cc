#include <sys/sem.h>
#include <sys/shm.h>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "deptrum_ros_driver/ros2_device.h"
#include "glog/logging.h"

#if defined STREAM_SDK_TYPE_STELLAR400 || defined STREAM_SDK_TYPE_STELLAR420
#include "deptrum_ros_driver/stellar400_ros2_device.h"
#endif
#if defined STREAM_SDK_TYPE_AURORA900 || defined STREAM_SDK_TYPE_AURORA930
#include "deptrum_ros_driver/aurora900_ros2_device.h"
#endif
#if defined STREAM_SDK_TYPE_NEBULA
#include "deptrum_ros_driver/nebula_ros2_device.h"
#endif
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  // std::string home = "/tmp/";
  // std::string info_log = home + "deptrum_ros2_driver_";
  // google::SetLogDestination(0, info_log.c_str());
  FLAGS_colorlogtostderr = true;
  FLAGS_logbuflevel = -1;  // 不缓存
  google::SetStderrLogging(google::GLOG_INFO);

  rclcpp::init(argc, argv);

#if defined STREAM_SDK_TYPE_STELLAR400 || defined STREAM_SDK_TYPE_STELLAR420
  auto node = std::make_shared<rclcpp::Node>("stellar");
  LOG(WARNING) << "hello stellar400/420";
  std::shared_ptr<RosDevice> device(new Stellar400RosDevice(node));
#elif defined(STREAM_SDK_TYPE_AURORA900) || defined STREAM_SDK_TYPE_AURORA930
  auto node = std::make_shared<rclcpp::Node>("aurora");
  LOG(WARNING) << "hello aurora 900/930";
  std::shared_ptr<RosDevice> device(new Aurora900RosDevice(node));
#elif defined(STREAM_SDK_TYPE_NEBULA)
  auto node = std::make_shared<rclcpp::Node>("nebula");
  LOG(WARNING) << "hello neula200";
  std::shared_ptr<RosDevice> device(new nebulaRosDevice(node));
#endif
  DeptrumRosDeviceParams param = {};
  param = device->params_;
  int boot_order = param.boot_order;
  if (param.device_numbers > 1) {
    int shmid;
    char* shm = NULL;
    char* tmp;
    if (boot_order == 1) {
      if ((shmid = shmget((key_t) STELLAR_PID, 1, 0666 | IPC_CREAT)) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order
                   << "::Create Share Memory Error:" << strerror(errno);
      shm = (char*) shmat(shmid, 0, 0);
      *shm = 1;
      device->InitDevice();
      LOG(INFO) << "BOOT_ORDER_" << param.boot_order << "::Device " << param.serial_number.c_str()
                << "already open";
      *shm = 2;
    } else {
      if ((shmid = shmget((key_t) STELLAR_PID, 1, 0666 | IPC_CREAT)) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order
                   << "::Create Share Memory Error:" << strerror(errno);
      shm = (char*) shmat(shmid, 0, 0);

      while (*shm != boot_order)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

      device->InitDevice();
      LOG(INFO) << "BOOT_ORDER_" << param.boot_order << "::Device" << param.serial_number.c_str()
                << "already open.";
      *shm = boot_order + 1;
    }
    if (boot_order == param.device_numbers) {
      if (shmdt(shm) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order << " shmdt failed:" << strerror(errno);
      if (shmctl(shmid, IPC_RMID, 0) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order
                   << "shmctl(IPC_RMID) failed :" << strerror(errno);
    } else {
      if (shmdt(shm) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order << "shmdt failed:" << strerror(errno);
    }
  } else {
    device->InitDevice();
  }

  device->Start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  LOG(INFO) << "process quit succeed";
  return 0;
}
