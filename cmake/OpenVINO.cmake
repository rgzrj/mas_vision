if(NOT DEFINED VISION_SOURCE_DIR)
    set(VISION_SOURCE_DIR "${PROJECT_SOURCE_DIR}")
endif()
# 显式目录/已有缓存优先，其次项目环境、共享环境，最后由 find_package 查系统 SDK。
if(NOT "$ENV{XDG_DATA_HOME}" STREQUAL "")
    set(buff_shared_venv "$ENV{XDG_DATA_HOME}/mas-vision/venv")
elseif(NOT "$ENV{HOME}" STREQUAL "")
    set(buff_shared_venv "$ENV{HOME}/.local/share/mas-vision/venv")
else()
    set(buff_shared_venv "")
endif()

if(NOT OpenVINO_DIR AND "$ENV{OpenVINO_DIR}" STREQUAL "")
    set(buff_openvino_roots "${VISION_SOURCE_DIR}/.venv-openvino")
    if(buff_shared_venv)
        list(APPEND buff_openvino_roots "${buff_shared_venv}")
    endif()
    foreach(venv IN LISTS buff_openvino_roots)
        file(GLOB configs "${venv}/lib/python*/site-packages/openvino/cmake/OpenVINOConfig.cmake")
        if(configs)
            list(GET configs 0 config)
            get_filename_component(directory "${config}" DIRECTORY)
            set(OpenVINO_DIR "${directory}" CACHE PATH "OpenVINO CMake package directory" FORCE)
            break()
        endif()
    endforeach()
endif()

find_package(OpenVINO QUIET CONFIG COMPONENTS Runtime)
if(NOT OpenVINO_FOUND)
    message(FATAL_ERROR
        "OpenVINO Runtime not found. From the project root, run once:\n"
        "  bash environment/setup_linux.sh\n"
        "This installs OpenVINO in a shared user environment outside source archives.\n"
        "Then rerun cmake .. in build; activation is not required.\n"
        "Project environment: ${VISION_SOURCE_DIR}/.venv-openvino\n"
        "Shared environment: ${buff_shared_venv}\n"
        "Explicit SDK: -DOpenVINO_DIR=<directory containing OpenVINOConfig.cmake>.\n"
        "Current OpenVINO_DIR='${OpenVINO_DIR}', environment='$ENV{OpenVINO_DIR}'.")
endif()
message(STATUS "OpenVINO config: ${OpenVINO_DIR}")
