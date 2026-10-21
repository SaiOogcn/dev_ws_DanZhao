cmake_minimum_required(VERSION 3.16)
project(sample_mipi_file)

add_executable(sample_mipi_file sample_mipi_file.cc ../sample_helper.cc)
target_link_libraries(sample_mipi_file ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS} ${Boost_LIBRARIES})
install(TARGETS sample_mipi_file DESTINATION samples/bin)