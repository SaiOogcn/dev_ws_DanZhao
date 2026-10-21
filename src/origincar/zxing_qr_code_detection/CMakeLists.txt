cmake_minimum_required(VERSION 3.8)
project(zxing_qr_code_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(origincar_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Ubuntu 22.04's libzxingcore-dev exports ZXingConfig.cmake. Keep a small
# fallback for vendor images that install only the headers and libZXing.so.
find_package(ZXing CONFIG QUIET)
if(TARGET ZXing::ZXing)
  set(ZXING_TARGET ZXing::ZXing)
elseif(TARGET ZXing::Core)
  set(ZXING_TARGET ZXing::Core)
else()
  find_path(ZXING_INCLUDE_DIR NAMES ZXing/ReadBarcode.h REQUIRED)
  find_library(ZXING_LIBRARY NAMES ZXing REQUIRED)
  add_library(zxing_qr_code_detection_zxing UNKNOWN IMPORTED)
  set_target_properties(zxing_qr_code_detection_zxing PROPERTIES
    IMPORTED_LOCATION "${ZXING_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ZXING_INCLUDE_DIR}"
  )
  set(ZXING_TARGET zxing_qr_code_detection_zxing)
endif()

add_executable(zxing_qr_detection_node
  src/zxing_qr_detection_node.cpp
)

ament_target_dependencies(zxing_qr_detection_node
  cv_bridge
  origincar_msg
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(zxing_qr_detection_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(zxing_qr_detection_node
  ${OpenCV_LIBRARIES}
  ${ZXING_TARGET}
)

install(TARGETS zxing_qr_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
