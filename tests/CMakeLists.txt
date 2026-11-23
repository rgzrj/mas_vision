# OpenVINO GPU 轻量测速所需的执行回归。
if(YOLO_RUNTIME STREQUAL "OPENVINO")
    add_executable(live_profile_test auto_buff/live_profile_test.cpp)
    vision_target_options(live_profile_test)
    target_include_directories(live_profile_test PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(live_profile_test PRIVATE buff_options openvino::runtime ${OpenCV_LIBS})
    add_test(NAME live_profile_test COMMAND live_profile_test)

    add_executable(yolo_execution_test auto_buff/yolo_execution_test.cpp)
    vision_target_options(yolo_execution_test)
    target_link_libraries(yolo_execution_test PRIVATE auto_buff openvino::runtime)
    add_test(NAME yolo_execution_test COMMAND yolo_execution_test)
    set_tests_properties(live_profile_test yolo_execution_test PROPERTIES TIMEOUT 60)
endif()

if(VISION_GPU_PROFILE_ONLY)
    return()
endif()

# auto_aim 模块回归。
add_executable(armor_replay_test auto_aim/armor_replay_test.cpp)
vision_target_options(armor_replay_test)
target_compile_definitions(armor_replay_test PRIVATE BUFF_PROJECT_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(armor_replay_test PRIVATE vision_replay ${BUFF_YAML_TARGET})
add_test(NAME armor_replay_test COMMAND armor_replay_test)

add_executable(armor_visual_test auto_aim/armor_visual_test.cpp)
vision_target_options(armor_visual_test)
target_compile_definitions(armor_visual_test PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")
target_link_libraries(armor_visual_test PRIVATE auto_aim auto_buff ${BUFF_YAML_TARGET})
add_test(NAME armor_visual_test COMMAND armor_visual_test)

add_executable(armor_detector_test auto_aim/armor_detector_test.cpp)
vision_target_options(armor_detector_test)
target_compile_definitions(armor_detector_test PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")
target_link_libraries(armor_detector_test PRIVATE auto_aim ${BUFF_YAML_TARGET})
add_test(NAME armor_detector_test COMMAND armor_detector_test)

add_executable(armor_pose_test auto_aim/armor_pose_test.cpp)
vision_target_options(armor_pose_test)
target_compile_definitions(armor_pose_test PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")
target_link_libraries(armor_pose_test PRIVATE auto_aim ${BUFF_YAML_TARGET})
add_test(NAME armor_pose_test COMMAND armor_pose_test)

add_executable(armor_plotter_test auto_aim/armor_plotter_test.cpp)
vision_target_options(armor_plotter_test)
target_compile_definitions(armor_plotter_test PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")
target_link_libraries(armor_plotter_test PRIVATE auto_aim ${BUFF_YAML_TARGET})
add_test(NAME armor_plotter_test COMMAND armor_plotter_test)
set_tests_properties(armor_plotter_test PROPERTIES RUN_SERIAL TRUE SKIP_RETURN_CODE 77)

# auto_buff 模块回归。
add_executable(buff_detect_test auto_buff/detect_test.cpp auto_buff/traditional_check.cpp auto_buff/yolo_check.cpp)
vision_target_options(buff_detect_test)
target_compile_definitions(buff_detect_test PRIVATE BUFF_PROJECT_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(buff_detect_test PRIVATE auto_buff buff_io ${BUFF_YAML_TARGET})
if(YOLO_RUNTIME STREQUAL "TENSORRT")
    add_test(NAME buff_detect_test COMMAND buff_detect_test --self-test geometry)
else()
    add_test(NAME buff_detect_test COMMAND buff_detect_test)
endif()

add_executable(buff_pose_test auto_buff/pose_test.cpp)
vision_target_options(buff_pose_test)
target_link_libraries(buff_pose_test PRIVATE auto_buff ${BUFF_YAML_TARGET})
add_test(NAME buff_pose_test COMMAND buff_pose_test)

add_executable(buff_track_test auto_buff/track_test.cpp)
vision_target_options(buff_track_test)
target_link_libraries(buff_track_test PRIVATE auto_buff ${BUFF_YAML_TARGET})
add_test(NAME buff_track_test COMMAND buff_track_test)

add_executable(buff_predict_test auto_buff/predict_test.cpp)
vision_target_options(buff_predict_test)
target_link_libraries(buff_predict_test PRIVATE auto_buff ${BUFF_YAML_TARGET})
add_test(NAME buff_predict_test COMMAND buff_predict_test)

add_executable(buff_shoot_test auto_buff/shoot_test.cpp)
vision_target_options(buff_shoot_test)
target_link_libraries(buff_shoot_test PRIVATE auto_buff)
add_test(NAME buff_shoot_test COMMAND buff_shoot_test)

add_executable(buff_debug_test auto_buff/buff_debug_test.cpp)
vision_target_options(buff_debug_test)
target_link_libraries(buff_debug_test PRIVATE auto_buff buff_io ${BUFF_YAML_TARGET})
add_test(NAME buff_debug_test COMMAND buff_debug_test)

# rm_utils 模块回归。
add_executable(display_test rm_utils/display/display_test.cpp)
vision_target_options(display_test)
target_link_libraries(display_test PRIVATE auto_buff buff_io ${BUFF_YAML_TARGET})
add_test(NAME display_test COMMAND display_test)

add_executable(sdl_display_test rm_utils/display/sdl_display_test.cpp)
vision_target_options(sdl_display_test)
target_link_libraries(sdl_display_test PRIVATE buff_display buff_io SDL2::SDL2)
add_test(NAME sdl_display_test COMMAND sdl_display_test)

add_executable(sdl_input_test rm_utils/display/sdl_input_test.cpp)
vision_target_options(sdl_input_test)
target_link_libraries(sdl_input_test PRIVATE buff_display SDL2::SDL2)
add_test(NAME sdl_input_test COMMAND sdl_input_test)

add_executable(display_lifecycle_test rm_utils/display/display_lifecycle_test.cpp)
vision_target_options(display_lifecycle_test)
target_link_libraries(display_lifecycle_test PRIVATE buff_display SDL2::SDL2)
add_test(NAME display_lifecycle_test COMMAND display_lifecycle_test)

add_executable(rerun_scalar_test rm_utils/display/rerun_scalar_test.cpp)
vision_target_options(rerun_scalar_test)
target_link_libraries(rerun_scalar_test PRIVATE buff_display)
# 读回 RRD 并校验事件；只有所需 Python SDK 可用时登记。
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    execute_process(COMMAND "${Python3_EXECUTABLE}" -c "from rerun.chunk import RrdReader"
        RESULT_VARIABLE rerun_reader_status OUTPUT_QUIET ERROR_QUIET)
    if(rerun_reader_status EQUAL 0)
        add_test(NAME rerun_scalar_readback
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/rm_utils/display/check_rerun_scalars.py"
                    $<TARGET_FILE:rerun_scalar_test>)
    else()
        message(STATUS "Rerun read-back check requires Python rerun-sdk with RrdReader")
    endif()
endif()

add_executable(frame_metadata_test rm_utils/io/frame_metadata_test.cpp)
vision_target_options(frame_metadata_test)
target_link_libraries(frame_metadata_test PRIVATE buff_io)
add_test(NAME frame_metadata_test COMMAND frame_metadata_test)

add_executable(plotter_test rm_utils/plotter_test.cpp)
vision_target_options(plotter_test)
target_link_libraries(plotter_test PRIVATE rm_plotter buff_options)
add_test(NAME plotter_test COMMAND plotter_test)

# hardware 模块回归。
add_executable(serial_protocol_test hardware/serial_protocol_test.cpp)
vision_target_options(serial_protocol_test)
target_link_libraries(serial_protocol_test PRIVATE hardware_common buff_options Eigen3::Eigen)
add_test(NAME serial_protocol_test COMMAND serial_protocol_test)
set_tests_properties(serial_protocol_test PROPERTIES TIMEOUT 60)

add_executable(frame_pool_test hardware/frame_pool_test.cpp)
vision_target_options(frame_pool_test)
target_link_libraries(frame_pool_test PRIVATE hardware_common buff_utils)
add_test(NAME frame_pool_test COMMAND frame_pool_test)

add_executable(vision_routing_test hardware/vision_routing_test.cpp)
vision_target_options(vision_routing_test)
target_link_libraries(vision_routing_test PRIVATE hardware_common Eigen3::Eigen Threads::Threads)
add_test(NAME vision_routing_test COMMAND vision_routing_test)

add_executable(serial_state_test hardware/serial_state_test.cpp ${PROJECT_SOURCE_DIR}/hardware/serial/user_serial.cpp)
vision_target_options(serial_state_test)
target_include_directories(serial_state_test BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common/mock_serial)
target_link_libraries(serial_state_test PRIVATE hardware_common buff_utils Eigen3::Eigen ${BUFF_YAML_TARGET})
add_test(NAME serial_state_test COMMAND serial_state_test)

# threads 模块回归。
add_executable(router_runtime_test threads/router_runtime_test.cpp
    ${VISION_SOURCE_DIR}/threads/vision_router/vision_router.cpp
    ${VISION_SOURCE_DIR}/hardware/serial/user_serial.cpp)
vision_target_options(router_runtime_test)
target_include_directories(router_runtime_test BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common/mock_serial)
target_link_libraries(router_runtime_test PRIVATE hardware_common auto_aim auto_buff ${BUFF_YAML_TARGET})
add_test(NAME router_runtime_test COMMAND router_runtime_test)
set_tests_properties(router_runtime_test PROPERTIES TIMEOUT 10)

# apps 模块回归。
add_executable(replay_test apps/replay_test.cpp)
vision_target_options(replay_test)
target_link_libraries(replay_test PRIVATE vision_replay vision_session ${BUFF_YAML_TARGET})
add_test(NAME replay_test COMMAND replay_test)

add_executable(playback_control_test apps/playback_control_test.cpp)
vision_target_options(playback_control_test)
target_link_libraries(playback_control_test PRIVATE buff_options)
add_test(NAME playback_control_test COMMAND playback_control_test)

# third_party 模块回归。
add_executable(mas_log_test third_party/mas_log_test.cpp)
vision_target_options(mas_log_test)
target_link_libraries(mas_log_test PRIVATE buff_utils)
add_test(NAME mas_log_test COMMAND mas_log_test)

# 根据推理 SDK 构建对应的执行检查。
if(YOLO_RUNTIME STREQUAL "OPENVINO")
    add_executable(opencv_threads_test threads/opencv_threads_test.cpp)
    vision_target_options(opencv_threads_test)
    target_compile_definitions(opencv_threads_test PRIVATE BUFF_PROJECT_DIR="${VISION_SOURCE_DIR}")
    target_link_libraries(opencv_threads_test PRIVATE auto_aim auto_buff ${BUFF_YAML_TARGET})
    add_test(NAME opencv_threads_test COMMAND opencv_threads_test)
else()
    add_executable(tensorrt_execution_test auto_buff/tensorrt_execution_test.cpp)
    vision_target_options(tensorrt_execution_test)
    target_link_libraries(tensorrt_execution_test PRIVATE auto_buff)
    add_test(NAME tensorrt_execution_test COMMAND tensorrt_execution_test)
    set_tests_properties(tensorrt_execution_test PROPERTIES SKIP_RETURN_CODE 77)
endif()

# 仅对本目录登记的测试补默认超时，保留 router 等测试已有的专用设置。
get_property(project_tests DIRECTORY PROPERTY TESTS)
foreach(test IN LISTS project_tests)
    get_property(has_timeout TEST ${test} PROPERTY TIMEOUT SET)
    if(NOT has_timeout)
        set_tests_properties(${test} PROPERTIES TIMEOUT 60)
    endif()
endforeach()
