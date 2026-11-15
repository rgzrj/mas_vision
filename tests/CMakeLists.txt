# tests 下统一登记；子目录只按职责放源码，不再创建独立工程。
add_executable(yolo_profile tools/yolo_profile.cpp)
target_link_libraries(yolo_profile PRIVATE auto_buff buff_io)
target_compile_definitions(yolo_profile PRIVATE BUFF_PROJECT_DIR="${PROJECT_SOURCE_DIR}")
add_test(NAME yolo_profile_help COMMAND yolo_profile --help)

add_executable(buff_detect_test auto_buff/detect_test.cpp auto_buff/traditional_check.cpp auto_buff/yolo_check.cpp)
target_link_libraries(buff_detect_test PRIVATE auto_buff buff_io)
target_compile_definitions(buff_detect_test PRIVATE BUFF_PROJECT_DIR="${PROJECT_SOURCE_DIR}")


add_executable(buff_pose_test auto_buff/pose_test.cpp)
target_link_libraries(buff_pose_test PRIVATE auto_buff)

add_executable(buff_track_test auto_buff/track_test.cpp)
target_link_libraries(buff_track_test PRIVATE auto_buff)

add_executable(buff_predict_test auto_buff/predict_test.cpp)
target_link_libraries(buff_predict_test PRIVATE auto_buff)

add_executable(buff_shoot_test auto_buff/shoot_test.cpp)
target_link_libraries(buff_shoot_test PRIVATE auto_buff)

add_executable(frame_pool_test hardware/frame_pool_test.cpp)
target_link_libraries(frame_pool_test PRIVATE hardware_common buff_utils)

add_executable(replay_test app/replay_test.cpp)
target_link_libraries(replay_test PRIVATE vision_replay vision_session)

add_executable(display_test app/display_test.cpp)
target_link_libraries(display_test PRIVATE auto_buff buff_io)

add_executable(buff_debug_test app/buff_debug_test.cpp)
target_link_libraries(buff_debug_test PRIVATE auto_buff buff_io)

add_executable(sdl_display_test app/sdl_display_test.cpp)
target_link_libraries(sdl_display_test PRIVATE buff_display buff_io SDL2::SDL2)
add_test(NAME sdl_display_test COMMAND sdl_display_test)

add_executable(sdl_input_test app/sdl_input_test.cpp)
target_link_libraries(sdl_input_test PRIVATE buff_display SDL2::SDL2)
add_test(NAME sdl_input_test COMMAND sdl_input_test)

add_executable(display_lifecycle_test app/display_lifecycle_test.cpp)
target_link_libraries(display_lifecycle_test PRIVATE buff_display SDL2::SDL2)
add_test(NAME display_lifecycle_test COMMAND display_lifecycle_test)


# 实机工具只在显式传入配置时才访问设备；CTest仅运行帮助和协议自检。
add_executable(camera_test hardware/camera_test.cpp)
target_link_libraries(camera_test PRIVATE hikcamera usbcamera buff_io)
target_link_libraries(camera_test PRIVATE buff_display)

add_executable(serial_test hardware/serial_test.cpp)
target_link_libraries(serial_test PRIVATE serial buff_io)

set(core_tests buff_pose_test buff_track_test buff_predict_test buff_shoot_test frame_pool_test replay_test)
list(APPEND core_tests buff_detect_test display_test buff_debug_test)
foreach(target ${core_tests})
    add_test(NAME ${target} COMMAND ${target})
endforeach()
add_test(NAME camera_test_help COMMAND camera_test --help)
add_test(NAME serial_test COMMAND serial_test --self-test)
add_executable(playback_control_test app/playback_control_test.cpp)
target_link_libraries(playback_control_test PRIVATE buff_options)
add_test(NAME playback_control_test COMMAND playback_control_test)

add_executable(vision_routing_test hardware/vision_routing_test.cpp)
target_link_libraries(vision_routing_test PRIVATE hardware_common Eigen3::Eigen Threads::Threads)
add_test(NAME vision_routing_test COMMAND vision_routing_test)

# 用内存串口传输运行生产解析器，不打开设备，也不编译 POSIX 串口实现。
add_executable(serial_state_test hardware/serial_state_test.cpp ${PROJECT_SOURCE_DIR}/hardware/serial/user_serial.cpp)
target_include_directories(serial_state_test BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common/mock_serial)
target_link_libraries(serial_state_test PRIVATE hardware_common buff_utils Eigen3::Eigen ${BUFF_YAML_TARGET})
add_test(NAME serial_state_test COMMAND serial_state_test)

add_executable(router_runtime_test hardware/router_runtime_test.cpp
    ${VISION_SOURCE_DIR}/threads/vision_router/vision_router.cpp
    ${VISION_SOURCE_DIR}/hardware/serial/user_serial.cpp)
target_include_directories(router_runtime_test BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common/mock_serial)
target_link_libraries(router_runtime_test PRIVATE hardware_common auto_aim auto_buff)
add_test(NAME router_runtime_test COMMAND router_runtime_test)
set_tests_properties(router_runtime_test PROPERTIES TIMEOUT 10)

add_executable(armor_replay_test app/armor_replay_test.cpp)
target_compile_definitions(armor_replay_test PRIVATE BUFF_PROJECT_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(armor_replay_test PRIVATE vision_replay)
add_test(NAME armor_replay_test COMMAND armor_replay_test)

# 生产绘制代码配图元接收端，验证信息完整性，不启动窗口或Viewer。
add_executable(armor_visual_test app/armor_visual_test.cpp)
target_compile_definitions(armor_visual_test PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")
target_link_libraries(armor_visual_test PRIVATE auto_aim auto_buff)
add_test(NAME armor_visual_test COMMAND armor_visual_test)

add_executable(armor_plotter_test app/armor_plotter_test.cpp)
target_compile_definitions(armor_plotter_test PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")
target_link_libraries(armor_plotter_test PRIVATE auto_aim)
add_test(NAME armor_plotter_test COMMAND armor_plotter_test)
set_tests_properties(armor_plotter_test PROPERTIES RUN_SERIAL TRUE SKIP_RETURN_CODE 77)

# 独立测速不参与主程序；图像只读一次，算法实例跨帧复用。
add_executable(buff_profile tools/buff_profile.cpp)
target_link_libraries(buff_profile PRIVATE auto_buff buff_io)
add_test(NAME buff_profile_help COMMAND buff_profile --help)
