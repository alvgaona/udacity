cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

project(build-my-world)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(welcome SHARED script/welcome.cpp)
target_link_libraries(welcome ${GAZEBO_LIBRARIES})
