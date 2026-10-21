cmake_minimum_required(VERSION 3.8)
project(demo1_visual_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(visual_controller src/visual_controller.cpp)
target_include_directories(visual_controller PRIVATE include)
ament_target_dependencies(visual_controller rclcpp geometry_msgs ai_msgs sensor_msgs)

install(TARGETS visual_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
