add_library(buff_io STATIC media_reader.cpp media_recorder.cpp paired_recorder.cpp paired_player.cpp frame_metadata.cpp)
vision_target_options(buff_io)
target_include_directories(buff_io PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(buff_io
    PUBLIC buff_options Eigen3::Eigen opencv_core opencv_videoio
    PRIVATE opencv_imgcodecs Threads::Threads buff_utils ${BUFF_YAML_TARGET})
