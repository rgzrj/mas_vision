# 实机任务负责算法调用和调试提交；采集程序只链接驱动，不链接此目标。
add_library(vision_threads STATIC
    hikcamera_thread/hikcamera_thread.cpp
    serial_thread/serial_thread.cpp
    armor_thread/armor_thread.cpp
    buff_thread/buff_thread.cpp
    vision_router/vision_router.cpp)
vision_target_options(vision_threads)
target_link_libraries(vision_threads PUBLIC buff_hardware auto_buff auto_aim PRIVATE ${BUFF_YAML_TARGET})
