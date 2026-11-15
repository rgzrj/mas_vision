# 海康固定参与构建；必须找到真实MVS SDK，缺失时在配置阶段报错。
add_library(hikcamera STATIC hikcamera.cpp hikcamera_capture.cpp hikcamera_reconnect.cpp)
target_include_directories(hikcamera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(hikcamera PUBLIC hardware_common ${BUFF_YAML_TARGET} buff_utils)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
find_path(MVS_INCLUDE_DIR MvCameraControl.h HINTS "${MVS_ROOT}/include" REQUIRED)
find_library(MVS_LIBRARY MvCameraControl HINTS "${MVS_ROOT}/lib/64" REQUIRED)
# MVS 旧版没有全局 Initialize/Finalize；按实际头文件选择生命周期接口。
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES "${MVS_INCLUDE_DIR}")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
check_cxx_source_compiles("#include <MvCameraControl.h>\nint main() { auto init = &MV_CC_Initialize; auto fini = &MV_CC_Finalize; return init() + fini(); }" MVS_HAS_LIFECYCLE)
target_compile_definitions(hikcamera PRIVATE BUFF_MVS_HAS_LIFECYCLE=$<BOOL:${MVS_HAS_LIFECYCLE}>)
target_include_directories(hikcamera PUBLIC "${MVS_INCLUDE_DIR}")
target_link_libraries(hikcamera PUBLIC "${MVS_LIBRARY}")
set_property(TARGET hikcamera APPEND PROPERTY BUILD_RPATH "${MVS_ROOT}/lib/64")
