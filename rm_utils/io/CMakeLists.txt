add_library(buff_io STATIC media_reader.cpp media_recorder.cpp paired_recorder.cpp paired_player.cpp)
target_include_directories(buff_io PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(buff_io PUBLIC buff_options Eigen3::Eigen ${OpenCV_LIBS} Threads::Threads
                            PRIVATE ${BUFF_YAML_TARGET})
