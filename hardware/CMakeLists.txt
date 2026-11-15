# 与 mas_vision-main 一样，公共定义和各设备库分别管理。
add_library(hardware_common INTERFACE)
target_include_directories(hardware_common INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hardware_common INTERFACE buff_options ${OpenCV_LIBS})
add_subdirectory(hikcamera)
add_subdirectory(usbcamera)
add_subdirectory(serial)

# 只聚合链接关系，已有调用方仍可一次引入全部硬件。
add_library(buff_hardware INTERFACE)
target_link_libraries(buff_hardware INTERFACE hardware_common hikcamera usbcamera serial)
