# 实机与标定入口；独立工具在自己的目录登记，主程序不链接它们。
add_executable(mas_vision main.cpp
    calibration/calibrate_camera.cpp
    calibration/calibrate_handeye.cpp)
vision_target_options(mas_vision)
target_link_libraries(mas_vision PRIVATE vision_threads buff_io ${BUFF_YAML_TARGET})
add_subdirectory(record)
add_subdirectory(replay)
