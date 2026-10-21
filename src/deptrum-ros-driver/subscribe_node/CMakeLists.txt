cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) 
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
find_package(geometry_msgs REQUIRED)


find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})



add_executable(sub_node src/sub_node.cc)
ament_target_dependencies(sub_node rclcpp std_msgs sensor_msgs cv_bridge)

add_executable(sub_node_ci src/sub_node_ci.cc)
ament_target_dependencies(sub_node_ci rclcpp std_msgs sensor_msgs cv_bridge)



target_link_libraries(sub_node
  opencv_core
  opencv_highgui
  ${GLog_LIBRARY}
  Boost::system 
  Boost::filesystem
)

target_link_libraries(sub_node_ci
  opencv_core
  opencv_highgui
  ${GLog_LIBRARY}
  Boost::system 
  Boost::filesystem
)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
  target_link_libraries(sub_node
    ${GFlags_LIBRARY}
  )
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
  target_link_libraries(sub_node_ci
    ${GFlags_LIBRARY}
  )
endif()