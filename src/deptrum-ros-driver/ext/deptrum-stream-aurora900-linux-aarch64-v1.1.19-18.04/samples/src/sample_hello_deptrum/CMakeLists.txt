cmake_minimum_required(VERSION 3.16)
project(sample_hello_deptrum)

add_executable(hello_deptrum hello_deptrum.cc)
target_link_libraries(hello_deptrum ${DEPTRUM_STREAM_LIB_NAME} ${EXTRA_LIBS})
install(TARGETS hello_deptrum DESTINATION samples/bin)