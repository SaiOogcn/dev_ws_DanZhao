cmake_minimum_required(VERSION 3.1.6 FATAL_ERROR)

project(samples LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

IF(UNIX)
    set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
ENDIF()

# find opencv
# set(OpenCV_DIR "your/path/to/opencv") # Alternative you can input cmake parameter from when cmake building
# Win32 needs to configure this environment variable
if(NOT DEFINED OpenCV_DIR)
  # Get OpenCV from system environment
  if(DEFINED ENV{OpenCV_INSTALL_PATH})
    set(OpenCV_DIR $ENV{OpenCV_INSTALL_PATH})
  endif()
endif()
find_package(OpenCV REQUIRED)
if(DEFINED OpenCV_LIBS AND DEFINED OpenCV_INCLUDE_DIRS)
  set(OpenCV_FOUND TRUE)
  include_directories(${OpenCV_INCLUDE_DIRS})
else()
  set(OpenCV_FOUND FALSE)
endif()

# deptrum sdk
set(deptrum_stream_ROOT_DIR ${CMAKE_SOURCE_DIR}/..)
set(deptrum_stream_LIBRARY_DIRS ${deptrum_stream_ROOT_DIR}/lib)
set(deptrum_stream_INCLUDE_DIR ${deptrum_stream_ROOT_DIR}/include)
include_directories(${deptrum_stream_INCLUDE_DIR})
link_directories(${deptrum_stream_LIBRARY_DIRS})

if(NOT DEFINED DEVICE_TAG)
  FILE(GLOB INCLUDE_SUBDIRS RELATIVE ${samples_SOURCE_DIR}/../include ${samples_SOURCE_DIR}/../include/*)
  string(REGEX REPLACE "(.*)/(.*)" "\\1" SDK_ROOT_PATH ${samples_SOURCE_DIR})
  message("sdk root dir: ${SDK_ROOT_PATH}")

  if(EXISTS "${SDK_ROOT_PATH}/include/deptrum/stellar200_series.h")
    set(DEVICE_TAG "STELLAR200")
    add_definitions(-DDEVICE_TYPE_STELLAR200)
  endif()

  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/aurora300_series.h)
    if(NOT DEFINED DEVICE_TAG)
      set(DEVICE_TAG "AURORA300")
    else()
      set(DEVICE_TAG "all")
    endif()

    add_definitions(-DDEVICE_TYPE_AURORA300)
  endif()

  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/stellar400_series.h)
    if(NOT DEFINED DEVICE_TAG)
      set(DEVICE_TAG "STELLAR400")
    else()
      set(DEVICE_TAG "all")
    endif()

    add_definitions(-DDEVICE_TYPE_STELLAR400)
  endif()

  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/stellar800_series.h)
    if(NOT DEFINED DEVICE_TAG)
      set(DEVICE_TAG "STELLAR800")
    else()
      set(DEVICE_TAG "all")
    endif()

    add_definitions(-DDEVICE_TYPE_STELLAR800)
  endif()

  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/aurora500.h)
    if(NOT DEFINED DEVICE_TAG)
      set(DEVICE_TAG "AURORA500")
    else()
      set(DEVICE_TAG "all")
    endif()

    add_definitions(-DDEVICE_TYPE_AURORA500)
  endif()

  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/aurora900_series.h)
    if(NOT DEFINED DEVICE_TAG)
      set(DEVICE_TAG "AURORA900")
    else()
      set(DEVICE_TAG "all")
    endif()

    add_definitions(-DDEVICE_TYPE_AURORA900)
  endif()
  
  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/nebula_series.h)
   if(NOT DEFINED DEVICE_TAG)
     set(DEVICE_TAG "NEBULA")
   else()
     set(DEVICE_TAG "all")
   endif()
   
   add_definitions(-DDEVICE_TYPE_NEBULA)
  endif()
  
  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/nebula_mipi_series.h)
   if(NOT DEFINED DEVICE_TAG)
     set(DEVICE_TAG "NEBULA")
   else()
     set(DEVICE_TAG "all")
   endif()
   
   add_definitions(-DDEVICE_TYPE_NEBULA_MIPI)
  endif()

  if(EXISTS ${SDK_ROOT_PATH}/include/deptrum/veinshine01.h)
   if(NOT DEFINED DEVICE_TAG)
     set(DEVICE_TAG "VEINSHINE01")
   else()
     set(DEVICE_TAG "all")
   endif()
   
   add_definitions(-DDEVICE_TYPE_VEINSHINE01)
  endif()

endif()

message(STATUS "DEVICE_TAG:${DEVICE_TAG}")
string(TOLOWER ${DEVICE_TAG} LOWER_DEVICE_TAG)
if(NOT DEVICE_TYPE_VEINSHINE01)
  set(DEPTRUM_STREAM_LIB_NAME "deptrum_stream_${LOWER_DEVICE_TAG}")
else()
  set(DEPTRUM_STREAM_LIB_NAME "${LOWER_DEVICE_TAG}")
endif()

message(STATUS "DEPTRUM_STREAM_LIB_NAME:${DEPTRUM_STREAM_LIB_NAME}")

set(EXECUTABLE_OUTPUT_PATH "${samples_SOURCE_DIR}/bin")
if(NOT EXISTS "${samples_SOURCE_DIR}/bin")
  file(MAKE_DIRECTORY "${samples_SOURCE_DIR}/bin")
endif()

add_subdirectory(src)
