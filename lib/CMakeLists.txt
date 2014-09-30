## CMake configuration for depth_from_sequence

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(depth_from_sequence)

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -03")
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
SET(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
SET(INC_DIR "${PROJECT_SOURCE_DIR}/include")

include_directories(${INC_DIR})

find_package(OpenCV REQUIRED)
add_subdirectory(densecrf)

file(GLOB SOURCES "${SRC_DIR}/*.cpp" "${SRC_DIR}/*.c")

add_library(depth_from_sequence STATIC ${SOURCES})
target_link_libraries(depth_from_sequence densecrf ${OpenCV_LIBS})
