cmake_minimum_required(VERSION 3.16)
project(sample_mipi)

add_executable(sample_mipi sample_mipi.cc ../sample_helper.cc)
target_link_libraries(sample_mipi ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS sample_mipi DESTINATION samples/bin)

add_executable(sample_mipi_user_alloc sample_mipi_user_alloc.cc ../sample_helper.cc)
target_link_libraries(sample_mipi_user_alloc ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS sample_mipi_user_alloc DESTINATION samples/bin)

add_executable(sample_mipi_mutl sample_mipi_mutl.cc ../sample_helper.cc)
target_link_libraries(sample_mipi_mutl ${DEPTRUM_STREAM_LIB_NAME} ${OpenCV_LIBS} ${EXTRA_LIBS})
install(TARGETS sample_mipi_mutl DESTINATION samples/bin)