cmake_minimum_required(VERSION 3.16)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(sample_firmware_upgrade)
add_subdirectory(sample_hello_deptrum)

# opencv required
if(${OpenCV_FOUND})
  add_subdirectory(sample_viewer)
  add_subdirectory(sample_lite)

  if(DEVICE_TYPE_NEBULA_MIPI)
    add_subdirectory(sample_mipi)
    add_subdirectory(sample_mipi_file)
  endif(DEVICE_TYPE_NEBULA_MIPI)

  if(DEVICE_TYPE_VEINSHINE01)
    add_subdirectory(all_function)
  endif(DEVICE_TYPE_VEINSHINE01)

  # if(DEVICE_TYPE_AURORA900 AND ENABLE_DIMFACE)
  #   add_subdirectory(aurora910)
  # endif(DEVICE_TYPE_AURORA900)

endif(${OpenCV_FOUND})

if(DEVICE_TYPE_AURORA900 AND ENABLE_DIMFACE)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/aurora910 DESTINATION samples/src)
endif()
