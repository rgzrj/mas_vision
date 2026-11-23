add_library(rm_utils STATIC
    algorithm/math_tools.cpp
    algorithm/trajectory.cpp
    algorithm/reprojection.cpp
    algorithm/extended_kalman_filter.cpp
    algorithm/buff/rune_energy_fitter.cpp
)
vision_target_options(rm_utils)
target_include_directories(rm_utils PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/algorithm"
    ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rm_utils PUBLIC buff_options Eigen3::Eigen ${OpenCV_LIBS})
add_subdirectory(io)
add_subdirectory(display)

add_subdirectory(plotter)
