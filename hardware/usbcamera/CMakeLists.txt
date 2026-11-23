add_library(usbcamera STATIC usbcamera.cpp usbcamera_config.cpp)
vision_target_options(usbcamera)
target_include_directories(usbcamera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(usbcamera PUBLIC hardware_common ${BUFF_YAML_TARGET} buff_utils)
