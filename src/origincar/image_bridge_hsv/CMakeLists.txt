cmake_minimum_required(VERSION 3.8)
project(image_bridge_hsv)

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
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_bridge_hsv
  src/image_bridge_hsv.cpp
)

ament_target_dependencies(image_bridge_hsv
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(image_bridge_hsv
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  image_bridge_hsv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
