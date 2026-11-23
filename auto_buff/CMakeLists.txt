# 打符算法按 YAML 在内部调用调试子类；不依赖应用回放和媒体录制。
add_library(auto_buff STATIC
    buff_detect/buff_detector.cpp
    buff_detect/traditional/rmcs/compat/thinning.cpp
    buff_detect/traditional/rmcs/rune.cpp
    buff_detect/traditional/process.cpp
    buff_detect/traditional/traditional.cpp
    buff_detect/yolo/yolo.cpp
    buff_detect/yolo/inference/inference.cpp
    buff_detect/yolo/observation/pose_output.cpp
    buff_detect/yolo/pixels/pixel_geometry.cpp
    buff_detect/yolo/pixels/pixel_refiner.cpp
    buff_track/pose/big_activated.cpp
    buff_track/pose/rune_camera.cpp
    buff_track/pose/rune_pnp.cpp
    buff_track/pose/rune_pose.cpp
    buff_track/pose/rune_projection.cpp
    buff_track/predict/rune_predictor.cpp
    buff_track/predict/rune_state.cpp
    buff_track/track/buff_tracker.cpp
    buff_track/track/rune_model.cpp
    buff_shoot/rune_shooter.cpp
    debug/buff_debug.cpp
    debug/visualize/detect_debug.cpp
    debug/visualize/track_debug.cpp
    debug/visualize/shoot_debug.cpp
)

vision_target_options(auto_buff)
target_link_libraries(auto_buff PUBLIC rm_utils buff_utils buff_display ${BUFF_YAML_TARGET})

if(YOLO_RUNTIME STREQUAL "OPENVINO")
    target_sources(auto_buff PRIVATE buff_detect/yolo/inference/openvino.cpp)
    target_link_libraries(auto_buff PRIVATE openvino::runtime)
else()
    target_sources(auto_buff PRIVATE buff_detect/yolo/inference/tensorrt.cpp)
    target_link_libraries(auto_buff PRIVATE TensorRT::runtime CUDA::cudart)
endif()

# 预生成布局嵌入程序；运行时不依赖 Python 或额外布局文件。
set(buff_blueprint_header "#pragma once\n#include <array>\n#include <span>\nnamespace auto_buff {\n")
set(buff_blueprint_entries "std::span<const unsigned char>{},")
foreach(mask RANGE 1 7)
    set(blueprint "${CMAKE_CURRENT_SOURCE_DIR}/debug/blueprints/${mask}.rbl")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${blueprint}")
    file(READ "${blueprint}" blueprint_hex HEX)
    string(REGEX REPLACE "(..)" "0x\\1," blueprint_bytes "${blueprint_hex}")
    string(APPEND buff_blueprint_header "inline constexpr unsigned char kBuffBlueprint${mask}[]{${blueprint_bytes}};\n")
    string(APPEND buff_blueprint_entries "std::span<const unsigned char>{kBuffBlueprint${mask}},")
endforeach()
string(APPEND buff_blueprint_header "inline constexpr std::array<std::span<const unsigned char>,8> kBuffBlueprints{${buff_blueprint_entries}};\n}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/buff_blueprints.hpp" CONTENT "${buff_blueprint_header}")
target_include_directories(auto_buff PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")

