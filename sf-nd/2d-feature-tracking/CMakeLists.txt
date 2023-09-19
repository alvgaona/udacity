cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 14)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, ${CXX_FLAGS})

find_package(OpenCV 4.1 REQUIRED)

project(camera_fusion)

file(GLOB SOURCES src/*.cpp)

add_executable (feature_tracking ${SOURCES})
add_definitions(${OpenCV_DEFINITIONS})
target_include_directories(feature_tracking PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_directories(feature_tracking PRIVATE ${OpenCV_LIBRARY_DIRS})
target_link_libraries (feature_tracking PRIVATE ${OpenCV_LIBRARIES})
