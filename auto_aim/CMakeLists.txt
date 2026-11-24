# MAS 装甲板算法按YAML开关直接调用visualize；SDL事件仍由主线程处理。
add_library(auto_aim STATIC
    armor_detector/armor_detector.cpp
    armor_detector/light_corner_corrector.cpp
    armor_detector/number_classifier.cpp
    armor_track/armor_track.cpp
    armor_track/armor_pose.cpp
    armor_track/armor_target.cpp
    armor_shoot/armor_shoot.cpp
    debug/armor_debug.cpp
    debug/visualize/detect_debug.cpp
    debug/visualize/pose_debug.cpp
    debug/visualize/track_debug.cpp
    debug/visualize/shoot_debug.cpp)
vision_target_options(auto_aim)
target_include_directories(auto_aim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_detector
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_track
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_shoot)
target_link_libraries(auto_aim
    PUBLIC rm_utils buff_utils rm_plotter buff_display opencv_dnn
    PRIVATE opencv_imgproc opencv_calib3d ${BUFF_YAML_TARGET})
set(armor_layout_header "#pragma once\n#include <array>\n#include <span>\nnamespace auto_aim {\n")
set(armor_layout_entries "std::span<const unsigned char>{},")
foreach(mask RANGE 1 15)
    set(layout "${CMAKE_CURRENT_SOURCE_DIR}/debug/blueprints/${mask}.rbl")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${layout}")
    file(READ "${layout}" layout_hex HEX)
    string(REGEX REPLACE "(..)" "0x\\1," layout_bytes "${layout_hex}")
    string(APPEND armor_layout_header "inline constexpr unsigned char kArmorBlueprint${mask}[]{${layout_bytes}};\n")
    string(APPEND armor_layout_entries "std::span<const unsigned char>{kArmorBlueprint${mask}},")
endforeach()
string(APPEND armor_layout_header "inline constexpr std::array<std::span<const unsigned char>,16> kArmorBlueprints{${armor_layout_entries}};\n}\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/armor_blueprints.hpp" CONTENT "${armor_layout_header}")
target_include_directories(auto_aim PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
