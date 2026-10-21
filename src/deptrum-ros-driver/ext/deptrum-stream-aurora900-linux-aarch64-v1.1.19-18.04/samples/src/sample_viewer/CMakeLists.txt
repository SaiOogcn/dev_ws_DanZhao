cmake_minimum_required(VERSION 3.16)
project(sample)

add_executable(sample sample.cc ../sample_helper.cc)
target_link_libraries(sample ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS sample DESTINATION samples/bin)