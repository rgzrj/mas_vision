add_library(rm_utils STATIC
    algorithm/math_tools.cpp
    algorithm/trajectory.cpp
    algorithm/reprojection.cpp
    algorithm/buff/rune_energy_fitter.cpp
)
target_include_directories(rm_utils PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rm_utils PUBLIC buff_options Eigen3::Eigen ${OpenCV_LIBS})
add_subdirectory(io)
add_subdirectory(display)

add_subdirectory(plotter)
