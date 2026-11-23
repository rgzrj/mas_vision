# 海康固定参与构建；必须找到真实MVS SDK，缺失时在配置阶段报错。
add_library(hikcamera STATIC hikcamera.cpp hikcamera_capture.cpp hikcamera_reconnect.cpp)
vision_target_options(hikcamera)
target_include_directories(hikcamera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(hikcamera PUBLIC hardware_common ${BUFF_YAML_TARGET} buff_utils)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
find_path(MVS_INCLUDE_DIR MvCameraControl.h HINTS "${MVS_ROOT}/include" REQUIRED)
find_library(MVS_LIBRARY MvCameraControl HINTS "${MVS_ROOT}/lib/64" REQUIRED)
target_include_directories(hikcamera PUBLIC "${MVS_INCLUDE_DIR}")
target_link_libraries(hikcamera PUBLIC "${MVS_LIBRARY}")
set_property(TARGET hikcamera APPEND PROPERTY BUILD_RPATH "${MVS_ROOT}/lib/64")
