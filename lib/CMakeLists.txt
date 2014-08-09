## CMake configuration for depth_from_sequence

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(depth_from_sequence)

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -03")
SET(CMAKE_BUILD_TYPE Release)
SET(SRC_DIR "src")

include_directories("${PROJECT_SOURCE_DIR}/include")

find_package(OpenCV REQUIRED)
add_subdirectory(densecrf)
include_directories("${PROJECT_SOURCE_DIR}/eigen")

file(GLOB SOURCES "${SRC_DIR}/*.cpp" "${SRC_DIR}/*.c")

add_executable(main.out main.cpp ${SOURCES})
target_link_libraries(main.out ${OpenCV_LIBS} )
target_link_libraries(main.out densecrf )
