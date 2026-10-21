cmake_minimum_required(VERSION 3.8)
project(image_binary)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_binary
  src/image_binary.cpp
)

ament_target_dependencies(image_binary
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(image_binary
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  image_binary
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()