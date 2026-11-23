# 每次构建只选择一种推理 SDK；业务 YAML 仍负责模型和设备设置。
set(YOLO_RUNTIME OPENVINO CACHE STRING "YOLO inference SDK: OPENVINO or TENSORRT")
set_property(CACHE YOLO_RUNTIME PROPERTY STRINGS OPENVINO TENSORRT)

if(YOLO_RUNTIME STREQUAL "OPENVINO")
    include("${CMAKE_CURRENT_LIST_DIR}/OpenVINO.cmake")
elseif(YOLO_RUNTIME STREQUAL "TENSORRT")
    find_package(CUDAToolkit REQUIRED)
    find_path(TensorRT_INCLUDE_DIR NvInferRuntime.h
        HINTS "${TensorRT_ROOT}" ENV TensorRT_ROOT PATH_SUFFIXES include REQUIRED)
    find_library(TensorRT_LIBRARY nvinfer
        HINTS "${TensorRT_ROOT}" ENV TensorRT_ROOT PATH_SUFFIXES lib lib64 REQUIRED)

    file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" version_line
        REGEX "^#define NV_TENSORRT_MAJOR [0-9]+")
    string(REGEX MATCH "[0-9]+" TensorRT_MAJOR "${version_line}")
    if(NOT TensorRT_MAJOR OR TensorRT_MAJOR LESS 10)
        message(FATAL_ERROR "YOLO requires TensorRT >= 10 (target API: 10.3)")
    endif()

    add_library(TensorRT::runtime UNKNOWN IMPORTED)
    set_target_properties(TensorRT::runtime PROPERTIES
        IMPORTED_LOCATION "${TensorRT_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "YOLO_RUNTIME must be OPENVINO or TENSORRT")
endif()
message(STATUS "YOLO runtime: ${YOLO_RUNTIME}")
