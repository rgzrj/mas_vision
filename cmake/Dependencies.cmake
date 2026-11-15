include("${CMAKE_CURRENT_LIST_DIR}/BaseDependencies.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/LocalDependencies.cmake")
find_package(SDL2 REQUIRED CONFIG)
include("${CMAKE_CURRENT_LIST_DIR}/OpenVINO.cmake")
