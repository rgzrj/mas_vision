find_package(nlohmann_json 3.2 REQUIRED)
add_library(rm_plotter STATIC plotter.cpp)
target_include_directories(rm_plotter PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(rm_plotter PUBLIC nlohmann_json::nlohmann_json)
