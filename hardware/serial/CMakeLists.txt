add_library(serial STATIC user_serial.cpp common/src/serial.cc)
target_link_libraries(serial PUBLIC hardware_common Eigen3::Eigen ${BUFF_YAML_TARGET} buff_utils)
target_include_directories(serial PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial/impl")
target_sources(serial PRIVATE common/src/impl/unix.cc common/src/impl/list_ports/list_ports_linux.cc)
target_link_libraries(serial PUBLIC rt)
