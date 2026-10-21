cmake_minimum_required(VERSION 3.16)
project(pace_test)

add_executable(face_stream_test main.cc face_device.cc  utils.cc ../sample_helper.cc)
target_link_libraries(face_stream_test ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS face_stream_test DESTINATION samples/bin)
