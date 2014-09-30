## CMake configuration for sample script of depth_from_sequence

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(depth_from_sequence_sample)

add_subdirectory("${PROJECT_SOURCE_DIR}/../lib" "${PROJECT_SOURCE_DIR}/lib")
include_directories("${PROJECT_SOURCE_DIR}/../lib/include")

add_executable(main.out main.cpp)
target_link_libraries(main.out depth_from_sequence)
