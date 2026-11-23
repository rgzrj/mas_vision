# 工具已移动到 third_party；保留 buff_utils 作为复用工具的总目标。
add_library(buff_utils STATIC mas_log/mas_log.cpp)
vision_target_options(buff_utils)
# 宏在包含 spdlog 头文件前生效；Release 不求值 DEBUG/TRACE 参数。
target_compile_definitions(buff_utils PUBLIC
    $<$<CONFIG:Debug>:SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG>
    $<$<NOT:$<CONFIG:Debug>>:SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO>)

target_link_libraries(buff_utils PUBLIC buff_options spdlog::spdlog Threads::Threads)
target_include_directories(buff_utils PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/mas_log"
    "${CMAKE_CURRENT_SOURCE_DIR}/thread_pool"
    "${CMAKE_CURRENT_SOURCE_DIR}/SPSCQueue"
)
# Tracy 的采集由 detect.tracy_debug 控制；禁用时不创建客户端线程。
set(TRACY_ENABLE ON CACHE BOOL "" FORCE)
set(TRACY_MANUAL_LIFETIME ON CACHE BOOL "" FORCE)
set(TRACY_NO_SAMPLING ON CACHE BOOL "" FORCE)
set(TRACY_NO_CONTEXT_SWITCH ON CACHE BOOL "" FORCE)
set(TRACY_NO_CALLSTACK ON CACHE BOOL "" FORCE)
set(TRACY_NO_FRAME_IMAGE ON CACHE BOOL "" FORCE)
add_subdirectory(tracy EXCLUDE_FROM_ALL)
target_compile_definitions(TracyClient PUBLIC TRACY_DELAYED_INIT)
target_link_libraries(buff_utils PUBLIC Tracy::TracyClient)
# 第三方 SDK 本体；自有显示适配位于 rm_utils/display。
add_subdirectory(rerun/sdk/rerun_cpp_sdk EXCLUDE_FROM_ALL)
