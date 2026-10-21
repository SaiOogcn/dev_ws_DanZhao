cmake_minimum_required(VERSION 3.8)
project(take_photo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(take_photo
  src/take_photo.cpp
)

ament_target_dependencies(take_photo
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
)

target_link_libraries(take_photo
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  take_photo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()