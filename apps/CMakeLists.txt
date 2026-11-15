# 实机与标定入口；独立工具在自己的目录登记，主程序不链接它们。
add_executable(mas_vision main.cpp
    calibration/calibrate_camera.cpp
    calibration/calibrate_handeye.cpp
    calibration/calibrate_latency.cpp)
target_compile_definitions(mas_vision PRIVATE BUFF_MVS_HAS_LIFECYCLE=$<BOOL:${MVS_HAS_LIFECYCLE}>)
target_link_libraries(mas_vision PRIVATE vision_threads buff_io)
add_subdirectory(record)
add_subdirectory(replay)
