# 自有显示适配集中于此；各算法调试子类使用同一显示目标。
add_library(buff_display STATIC
    display.cpp
    sdl/sdl_display.cpp
    rerun/rerun_display.cpp)
vision_target_options(buff_display)
target_link_libraries(buff_display
    PUBLIC buff_options ${OpenCV_LIBS}
    PRIVATE buff_utils ${BUFF_YAML_TARGET} SDL2::SDL2 rerun_sdk)
target_include_directories(buff_display PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
