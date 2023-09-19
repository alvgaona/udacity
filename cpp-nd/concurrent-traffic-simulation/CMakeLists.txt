cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(traffic_simulation)

find_package(OpenCV 4.1 REQUIRED) # Change "OpenCV" to "opencv" depending on opencv installation

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Find all executables
file(GLOB SOURCES src/*.cpp) #src/*.h

# Add GoogleTest library subdirectories
add_subdirectory(thirdparty/googletest)

# Add the testing executable
add_executable(test test/utest_traffic_simulation.cpp) # TODO: Append .cpp files for testing

target_link_libraries(test gtest_main)

# Add project executable
add_executable(traffic_simulation ${SOURCES})
target_link_libraries(traffic_simulation ${OpenCV_LIBRARIES})

