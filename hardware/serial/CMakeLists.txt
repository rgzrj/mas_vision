# 第三方串口实现独立编译，不继承自有目标的 C++23 和警告设置。
add_library(serial_driver STATIC
    common/src/serial.cc
    common/src/impl/unix.cc
    common/src/impl/list_ports/list_ports_linux.cc)

target_include_directories(serial_driver
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/common/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/common/include/serial/impl")

target_link_libraries(serial_driver PRIVATE Threads::Threads rt)

# 自有协议与配置封装；公开头文件仍可使用 serial.h。
add_library(serial STATIC user_serial.cpp)
vision_target_options(serial)

target_include_directories(serial PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(serial PUBLIC hardware_common Eigen3::Eigen buff_utils serial_driver
                             PRIVATE ${BUFF_YAML_TARGET})
