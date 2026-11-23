# 三个构建入口共用 Linux 要求和自有代码的编译设置。
include_guard(GLOBAL)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "mas_vision targets Linux only. Build on Ubuntu with GCC >= 13; see README_LINUX.md.")
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
    message(FATAL_ERROR "RMCS needs GCC >= 13 and its C++23 standard library")
endif()
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# 兼容随工程提供的旧依赖；不降低自有项目的最低 CMake 版本。
if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(VISION_WARNINGS_AS_ERRORS "Treat warnings in project sources as errors" OFF)

add_library(buff_options INTERFACE)
target_include_directories(buff_options INTERFACE "${VISION_SOURCE_DIR}")

# 仅用于自有目标；第三方库沿用自身的语言标准和警告设置。
function(vision_target_options target)
    target_compile_features(${target} PRIVATE cxx_std_23)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(warnings -Wall -Wextra -Wpedantic)
        if(VISION_WARNINGS_AS_ERRORS)
            list(APPEND warnings -Werror)
        endif()

        target_compile_options(${target} PRIVATE ${warnings})
    endif()
endfunction()
