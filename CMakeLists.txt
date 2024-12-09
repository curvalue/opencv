cmake_minimum_required(VERSION 3.30)
project(untitled)

set(CMAKE_CXX_STANDARD 20)

add_executable(untitled test.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(untitled ${OpenCV_LIBS})

