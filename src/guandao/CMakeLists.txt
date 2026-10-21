cmake_minimum_required(VERSION 3.8)
project(guandao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)

add_executable(guandao_node src/guandao_node.cpp)
ament_target_dependencies(guandao_node
  rclcpp nav_msgs geometry_msgs std_msgs ai_msgs)

install(TARGETS guandao_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
