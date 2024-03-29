cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 14)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)
find_package(PCL 1.7 REQUIRED)

file(GLOB SOURCES src/*.cpp)

add_executable (object_tracking ${SOURCES})
add_definitions(${OpenCV_DEFINITIONS} ${PCL_DEFINITIONS})
target_include_directories(object_tracking PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})
target_link_directories(object_tracking PRIVATE ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
target_link_libraries (object_tracking PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
