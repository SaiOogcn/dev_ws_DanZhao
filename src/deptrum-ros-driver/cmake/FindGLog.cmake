set(LIB_VERSION 0.4.0)

set(GLog_FOUND FALSE)
if(${SYSTEM_PLATFORM} STREQUAL "windows-amd64")
  set(TARGET_MD5 165535db9a9e1f57a6ca69d0826a35f6)
elseif(${SYSTEM_PLATFORM} STREQUAL "linux-x86_64")
  set(TARGET_MD5 8f0309efb41f7130e2305d1a94f9fb8f)
elseif(${SYSTEM_PLATFORM} STREQUAL "linux-aarch64")
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 5.7)
    set(TARGET_MD5 aded50490dc43f648bfe030a4bfc2e34)
  else()
    set(TARGET_MD5 7078b8c4d5d90a0cf2752e21f3457a61)
  endif()
endif()
set(LIB_NAME "glog-${LIB_VERSION}")

if(WIN32)
  set(TARGET_EXT "zip")
else()
  set(TARGET_EXT "tgz")
endif()

download_3rdparty(${LIB_NAME} ${TARGET_EXT} "")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_DIR_NAME}/glog-${LIB_VERSION}/include/glog/logging.h")
  set(GLog_FOUND TRUE)
  set(GLog_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_DIR_NAME}/glog-${LIB_VERSION}/include")
  set(GLog_LIBRARY_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_DIR_NAME}/glog-${LIB_VERSION}/lib-static")

  if(WIN32)
    add_definitions(-DGOOGLE_GLOG_DLL_DECL=) # avoid link error
    set(POSTFIX $<$<CONFIG:Debug>:d>)
    set(GLog_LIBRARY glog${POSTFIX} shlwapi)
  else()
    set(GLog_LIBRARY glog)
  endif()
  if((NOT WIN32) AND ENABLE_LD_GROUP)
    set(GLog_LIBRARY
        -Wl,--start-group
        ${GLog_LIBRARY}
        -Wl,--end-group
       )
  endif()
endif()
unset(TARGET_MD5)

if(GLog_FOUND)
  message(STATUS "GLog library status:")
  message(STATUS "    include path: ${GLog_INCLUDE_DIRS}")
  message(STATUS "    library path: ${GLog_LIBRARY_DIRS}")
  message(STATUS "    libraries: ${GLog_LIBRARY}")
  include_directories(SYSTEM ${GLog_INCLUDE_DIRS})
  link_directories(${GLog_LIBRARY_DIRS})
else()
  message(FATAL_ERROR "Failed to find GLog!")
endif(GLog_FOUND)
