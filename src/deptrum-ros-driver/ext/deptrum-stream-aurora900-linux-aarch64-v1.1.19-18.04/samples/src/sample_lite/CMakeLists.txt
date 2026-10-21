cmake_minimum_required(VERSION 3.16)
project(sample_lite)

add_executable(sample_lite sample_lite.cc ../sample_helper.cc)
target_link_libraries(sample_lite ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS sample_lite DESTINATION samples/bin)