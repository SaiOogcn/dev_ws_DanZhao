cmake_minimum_required(VERSION 3.5)
string(TOLOWER ${STREAM_SDK_TYPE} LOWER_STREAM_SDK_TYPE)
set(STREAM_SDK_TYPE "STELLAR400" CACHE STRING "Type of Device SDK to build")
project(deptrum-ros-driver-${LOWER_STREAM_SDK_TYPE})


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
string(FIND "${rclcpp_DIR}" "galactic" substring_index)
if(NOT "${substring_index}" EQUAL -1)
  set(ROS_DISTRO galactic)
endif()
string(FIND "${rclcpp_DIR}" "foxy" substring_index)
if(NOT "${substring_index}" EQUAL -1)
  set(ROS_DISTRO foxy)
endif()
string(FIND "${rclcpp_DIR}" "humble" substring_index)
if(NOT "${substring_index}" EQUAL -1)
  set(ROS_DISTRO humble)
endif()


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(STREAM_SDK_TYPE "STELLAR400" CACHE STRING "Type of Device SDK to build")
if ("${STREAM_SDK_TYPE}" MATCHES "^(STELLAR400|STELLAR420|AURORA900|AURORA930|NEBULA)$")
  add_definitions(-DSTREAM_SDK_TYPE_${STREAM_SDK_TYPE})
  message(STATUS "STREAM_SDK_TYPE: ${STREAM_SDK_TYPE}")
else()
  message(FATAL_ERROR "Unsupported STREAM_SDK_TYPE: ${STREAM_SDK_TYPE}")
endif()

add_compile_options(-std=c++17)
#add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
#add_link_options(-fsanitize=address -fno-omit-frame-pointer)

set(ROS_MAJOR_VERSION_STELLAR400 0)
set(ROS_MINOR_VERSION_STELLAR400 1)
set(ROS_REVISION_VERSION_STELLAR400 4)

set(ROS_MAJOR_VERSION_AURORA930 0)
set(ROS_MINOR_VERSION_AURORA930 2)
set(ROS_REVISION_VERSION_AURORA930 1001)

set(ROS_MAJOR_VERSION_NEBULA 0)
set(ROS_MINOR_VERSION_NEBULA 2)
set(ROS_REVISION_VERSION_NEBULA 10)

set(EXTERNAL_DIR_NAME ext)
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_DIR_NAME}")
  execute_process(
    COMMAND echo "make directory ${EXTERNAL_DIR_NAME}..."
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_DIR_NAME}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

string(TOLOWER "${CMAKE_SYSTEM_NAME}" LOWER_SYSTEM_NAME)
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" LOWER_PROCESSOR_NAME)
set(SYSTEM_PLATFORM "${LOWER_SYSTEM_NAME}-${LOWER_PROCESSOR_NAME}")
message(STATUS "SYSTEM_PLATFORM: ${SYSTEM_PLATFORM}")
if("${STREAM_SDK_TYPE}" STREQUAL "STELLAR400" OR "${STREAM_SDK_TYPE}" STREQUAL "STELLAR420")
  set(version_tag "")
  set(DEPTRUM_STREAM_SDK_VERSION 1.1.2)
  set(STREAM_SDK_LIBS deptrum_stream_stellar400)
  set(STREAM_SDK_PKG_BASENAME  deptrum-stream-stellar400-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION})
  if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    set(STREAM_SDK_PKG_NAME "${STREAM_SDK_PKG_BASENAME}${version_tag}-18.04")
  else()
    set(STREAM_SDK_PKG_NAME "${STREAM_SDK_PKG_BASENAME}${version_tag}-18.04")
  endif()
  set(DEPTRUM_STREAM_SDK_NAME stellar400)
  configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/version_ros_stellar400.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/version_ros_stellar400.h"
   )
elseif("${STREAM_SDK_TYPE}" STREQUAL "AURORA930")
  set(version_tag "")
  set(DEPTRUM_STREAM_SDK_VERSION 1.1.19)
  set(STREAM_SDK_LIBS deptrum_stream_aurora900)
  set(STREAM_SDK_PKG_BASENAME  deptrum-stream-aurora900-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION})
  if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    set(STREAM_SDK_PKG_NAME "deptrum-stream-aurora900-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION}${version_tag}-18.04")
  else()
    set(STREAM_SDK_PKG_NAME "deptrum-stream-aurora900-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION}${version_tag}-18.04")
  endif()
  set(DEPTRUM_STREAM_SDK_NAME aurora900)
  configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/version_ros_aurora930.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/version_ros_aurora930.h"
  )
elseif("${STREAM_SDK_TYPE}" STREQUAL "NEBULA")
  set(version_tag "")
  set(DEPTRUM_STREAM_SDK_VERSION 1.2.14) 
  set(STREAM_SDK_LIBS deptrum_stream_nebula)
  set(STREAM_SDK_PKG_BASENAME  deptrum-stream-nebula-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION})
  if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    set(STREAM_SDK_PKG_NAME "deptrum-stream-nebula-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION}${version_tag}-18.04")
  else()
    set(STREAM_SDK_PKG_NAME "deptrum-stream-nebula-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION}${version_tag}-18.04")
  endif()
  set(DEPTRUM_STREAM_SDK_NAME nebula)
  configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/version_ros_nebula.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/version_ros_nebula.h"
  )
else()
  message(FATAL_ERROR,"not known device type ${STREAM_SDK_TYPE}")
endif()
if (BUILD_FOR_INTERNAL)
   if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
     set(STREAM_SDK_PKG_NAME ${STREAM_SDK_PKG_BASENAME}${version_tag}-internal-18.04)
   else()
     set(STREAM_SDK_PKG_NAME ${STREAM_SDK_PKG_BASENAME}${version_tag}-internal-18.04)
   endif()
endif()
include(cmake/Download.cmake)
set(STREAM_SDK_PACKAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_DIR_NAME}/${STREAM_SDK_PKG_NAME})

if(BUILD_FOR_INTERNAL)
  if("${STREAM_SDK_TYPE}" STREQUAL "STELLAR400" OR "${STREAM_SDK_TYPE}" STREQUAL "STELLAR420")
    download_sdk(deptrum-stream/stellar400 v${DEPTRUM_STREAM_SDK_VERSION}${version_tag}-internal)
  else()
    message(STATUS "${STREAM_SDK_TYPE} no have internal version")
  endif()
else()
  if("${STREAM_SDK_TYPE}" STREQUAL "STELLAR400" OR "${STREAM_SDK_TYPE}" STREQUAL "STELLAR420")
    download_sdk(deptrum-stream/stellar400 v${DEPTRUM_STREAM_SDK_VERSION}${version_tag})
  elseif("${STREAM_SDK_TYPE}" STREQUAL "AURORA900" OR "${STREAM_SDK_TYPE}" STREQUAL "AURORA930")
    download_sdk(deptrum-stream/aurora900 v${DEPTRUM_STREAM_SDK_VERSION}${version_tag})
  elseif("${STREAM_SDK_TYPE}" STREQUAL "NEBULA")
    download_sdk(deptrum-stream/nebula v${DEPTRUM_STREAM_SDK_VERSION}${version_tag})
  else()
    message(STATUS "not known device type")
  endif()
endif()

file(REMOVE ${STREAM_SDK_PACKAGE_PATH}/InstallDriver.txt)
file(REMOVE_RECURSE ${STREAM_SDK_PACKAGE_PATH}/doc)
include_directories(include)
if("${STREAM_SDK_TYPE}" STREQUAL "STELLAR400" OR "${STREAM_SDK_TYPE}" STREQUAL "STELLAR420")
 include_directories( 
  ${DEPTRUM_STREAM_STELLAR400_INCLUDE_DIRS}
)
 link_directories(
  ${DEPTRUM_STREAM_STELLAR400_STATIC_LIBRARY_DIRS}
  ${JPEG_TURBO_LIBRARY_DIRS}
)
elseif("${STREAM_SDK_TYPE}" STREQUAL "AURORA900" OR "${STREAM_SDK_TYPE}" STREQUAL "AURORA930")
 include_directories( 
  ${DEPTRUM_STREAM_AURORA900_INCLUDE_DIRS}
)
 link_directories(
  ${DEPTRUM_STREAM_AURORA900_STATIC_LIBRARY_DIRS}
  ${JPEG_TURBO_LIBRARY_DIRS}
)
elseif("${STREAM_SDK_TYPE}" STREQUAL "NEBULA")
 include_directories( 
  ${DEPTRUM_STREAM_NEBULA_INCLUDE_DIRS}
)
 link_directories(
  ${DEPTRUM_STREAM_NEBULA_STATIC_LIBRARY_DIRS}
  ${JPEG_TURBO_LIBRARY_DIRS}
)
else()
  message(FATAL_ERROR,"not known device type ${STREAM_SDK_TYPE}")
endif()
if (BUILD_FOR_INTERNAL)
  include_directories(${STREAM_SDK_PACKAGE_PATH}/include/camera)
  include_directories(${STREAM_SDK_PACKAGE_PATH}/include/deptrum)
  include_directories(${STREAM_SDK_PACKAGE_PATH}/include/functional)
  include_directories(${STREAM_SDK_PACKAGE_PATH}/include/main)
  include_directories(${STREAM_SDK_PACKAGE_PATH}/include/sniper)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED COMPONENTS rclcpp std_msgs sensor_msgs)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) 
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

#glog
find_package(GLog REQUIRED)
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
  find_package(GFlags REQUIRED)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if("${STREAM_SDK_TYPE}" STREQUAL "STELLAR400" OR "${STREAM_SDK_TYPE}" STREQUAL "STELLAR420")
  add_executable(${LOWER_STREAM_SDK_TYPE}_node src/ros2_node.cc src/ros2_device.cc src/stellar400_ros2_device.cc src/camera_parameters.cc)
  #target_link_options(${LOWER_STREAM_SDK_TYPE}_node PRIVATE "-Wl,-rpath=${DEPTRUM_STREAM_STELLAR400_STATIC_LIBRARY_DIRS}")
elseif("${STREAM_SDK_TYPE}" STREQUAL "AURORA900" OR "${STREAM_SDK_TYPE}" STREQUAL "AURORA930")
  add_executable(${LOWER_STREAM_SDK_TYPE}_node src/ros2_node.cc src/ros2_device.cc src/aurora900_ros2_device.cc src/camera_parameters.cc)
  #target_link_options(${LOWER_STREAM_SDK_TYPE}_node PRIVATE "-Wl,-rpath=${DEPTRUM_STREAM_AURORA900_STATIC_LIBRARY_DIRS}")
elseif("${STREAM_SDK_TYPE}" STREQUAL "NEBULA")
  add_executable(${LOWER_STREAM_SDK_TYPE}_node src/ros2_node.cc src/ros2_device.cc src/nebula_ros2_device.cc src/camera_parameters.cc)
  #target_link_options(${LOWER_STREAM_SDK_TYPE}_node PRIVATE "-Wl,-rpath=${DEPTRUM_STREAM_NEBULA_STATIC_LIBRARY_DIRS}")
endif()

#set_target_properties(${LOWER_STREAM_SDK_TYPE}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${LOWER_STREAM_SDK_TYPE}_node
  ${STREAM_SDK_LIBS}
  opencv_core
  opencv_highgui
  ${GLog_LIBRARY}
)
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" AND ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
  target_link_libraries(${LOWER_STREAM_SDK_TYPE}_node
    ${GFlags_LIBRARY}
  )
endif()

#ament_export_libraries(${CMAKE_CURRENT_SOURCE_DIR}/ext/deptrum-stream-aurora900-linux-x86_64-v1.0.39-18.04/lib)
if("${STREAM_SDK_TYPE}" STREQUAL "AURORA900" OR "${STREAM_SDK_TYPE}" STREQUAL "AURORA930")
  ament_export_libraries(deptrum_stream_aurora900.so)
elseif("${STREAM_SDK_TYPE}" STREQUAL "STELLAR400" OR "${STREAM_SDK_TYPE}" STREQUAL "STELLAR420")
  ament_export_libraries(deptrum_stream_stellar400.so)
elseif("${STREAM_SDK_TYPE}" STREQUAL "NEBULA")
  ament_export_libraries(deptrum_stream_nebula.so)
endif()
ament_export_libraries(export_${LOWER_STREAM_SDK_TYPE}_node)
ament_export_libraries(export_${PROJECT_NAME})

ament_target_dependencies(${LOWER_STREAM_SDK_TYPE}_node rclcpp std_msgs sensor_msgs cv_bridge geometry_msgs tf2 tf2_ros tf2_geometry_msgs angles)
add_subdirectory(subscribe_node)

install(TARGETS ${LOWER_STREAM_SDK_TYPE}_node sub_node  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${LOWER_STREAM_SDK_TYPE}_node sub_node_ci  DESTINATION lib/${PROJECT_NAME})


install(FILES
  ${STREAM_SDK_PACKAGE_PATH}/scripts/99-deptrum-libusb.rules
  DESTINATION share/${PROJECT_NAME}
)

if("${STREAM_SDK_TYPE}" STREQUAL "STELLAR400")
  install(FILES 
    ${DEPTRUM_STREAM_STELLAR400_STATIC_LIBRARY_DIRS}/libdeptrum_stream_stellar400.so.${DEPTRUM_STREAM_SDK_VERSION}
    DESTINATION lib
  )
  install(
    DIRECTORY
    launch_stellar400/launch
    DESTINATION share/${PROJECT_NAME}
  )
  install(FILES
    docs/usage_stellar400.md
    RELEASE_STELLAR400.md
    DESTINATION share/${PROJECT_NAME}/doc
  )
  install(FILES rviz/stellar400-ros2.rviz DESTINATION share/${PROJECT_NAME}/rviz)
elseif("${STREAM_SDK_TYPE}" STREQUAL "STELLAR420")
   install(FILES 
    ${DEPTRUM_STREAM_STELLAR400_STATIC_LIBRARY_DIRS}/libdeptrum_stream_stellar400.so.${DEPTRUM_STREAM_SDK_VERSION}
    DESTINATION lib
  )
  install(DIRECTORY launch_stellar420/launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
  install(FILES
    docs/usage_stellar420.md
    RELEASE_STELLAR420.md
    DESTINATION share/${PROJECT_NAME}/doc
  )
  install(FILES rviz/stellar420-ros2.rviz DESTINATION share/${PROJECT_NAME}/rviz)
elseif("${STREAM_SDK_TYPE}" STREQUAL "AURORA930")
  install(FILES 
    ${DEPTRUM_STREAM_AURORA900_STATIC_LIBRARY_DIRS}/libdeptrum_stream_aurora900.so.${DEPTRUM_STREAM_SDK_VERSION}
    DESTINATION lib
  )
  install(DIRECTORY launch_aurora930/launch DESTINATION share/${PROJECT_NAME}   FILES_MATCHING PATTERN "*.py")
  install(FILES
    docs/usage_aurora930.md
    RELEASE_AURORA930.md
    DESTINATION share/${PROJECT_NAME}/doc
  )
  install(FILES rviz/aurora930-ros2.rviz DESTINATION share/${PROJECT_NAME}/rviz)
elseif("${STREAM_SDK_TYPE}" STREQUAL "NEBULA")
   install(FILES 
    ${DEPTRUM_STREAM_NEBULA_STATIC_LIBRARY_DIRS}/libdeptrum_stream_nebula.so.${DEPTRUM_STREAM_SDK_VERSION}
    DESTINATION lib
  )
  install(DIRECTORY launch_nebula/launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
  install(FILES
    docs/usage_nebula.md
    RELEASE_NEBULA.md
    DESTINATION share/${PROJECT_NAME}/doc
  )
  install(FILES rviz/nebula-ros2.rviz DESTINATION share/${PROJECT_NAME}/rviz)
endif()
ament_package()
