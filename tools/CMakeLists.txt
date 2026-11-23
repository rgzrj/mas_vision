# 手动诊断和测速工具复用生产模块。
if(BUILD_TOOLS AND NOT VISION_GPU_PROFILE_ONLY)
    add_executable(yolo_profile yolo_profile.cpp)
    vision_target_options(yolo_profile)
    target_link_libraries(yolo_profile PRIVATE auto_buff buff_io ${BUFF_YAML_TARGET})
    target_compile_definitions(yolo_profile PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")

    add_executable(buff_profile buff_profile.cpp)
    vision_target_options(buff_profile)
    target_link_libraries(buff_profile PRIVATE auto_buff buff_io)

    add_executable(sdl_upload_profile sdl_upload_profile.cpp)
    vision_target_options(sdl_upload_profile)
    target_include_directories(sdl_upload_profile PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(sdl_upload_profile PRIVATE buff_options ${OpenCV_LIBS} SDL2::SDL2)
    add_executable(camera_test camera_test.cpp)
    vision_target_options(camera_test)
    target_link_libraries(camera_test PRIVATE hikcamera usbcamera buff_io buff_display ${BUFF_YAML_TARGET})

    add_executable(serial_test serial_test.cpp)
    vision_target_options(serial_test)
    target_link_libraries(serial_test PRIVATE serial buff_io)

    add_executable(buff_preview buff_preview.cpp)
    vision_target_options(buff_preview)
    target_link_libraries(buff_preview PRIVATE auto_buff buff_io ${BUFF_YAML_TARGET})

endif()

if(YOLO_RUNTIME STREQUAL "OPENVINO")
    add_executable(gpu_live_profile gpu_live_profile.cpp)
    vision_target_options(gpu_live_profile)
    target_link_libraries(gpu_live_profile PRIVATE auto_buff hikcamera serial buff_io ${BUFF_YAML_TARGET} openvino::runtime)
endif()
