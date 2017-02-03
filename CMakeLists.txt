cmake_minimum_required(VERSION 3.1)
project(Astar)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(Astar ${SOURCE_FILES})
target_link_libraries(Astar ${OpenCV_LIBS})
