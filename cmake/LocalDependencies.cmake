# 显式 -D / 环境变量优先；否则使用交付包已有源码，避免 Arrow 构建期间再次访问 GitHub。
set(buff_archive_dir "${VISION_SOURCE_DIR}/environment/archives")
foreach(pair IN ITEMS
        "RERUN_ARROW_URL|apache-arrow-18.0.0.tar.gz"
        "ARROW_XSIMD_URL|xsimd-13.0.0.tar.gz"
        "ARROW_MIMALLOC_URL|mimalloc-v2.0.6.tar.gz"
        "ARROW_BOOST_URL|boost-1.81.0.tar.gz")
    string(REPLACE "|" ";" fields "${pair}")
    list(GET fields 0 variable)
    list(GET fields 1 archive)
    if(NOT DEFINED ${variable})
        if(NOT "$ENV{${variable}}" STREQUAL "")
            set(${variable} "$ENV{${variable}}" CACHE STRING "Dependency source archive URL or local path")
        elseif(EXISTS "${buff_archive_dir}/${archive}")
            set(${variable} "${buff_archive_dir}/${archive}" CACHE STRING "Dependency source archive URL or local path")
        endif()
    endif()
    if(DEFINED ${variable})
        message(STATUS "${variable}: ${${variable}}")
    endif()
endforeach()
