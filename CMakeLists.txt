cmake_minimum_required(VERSION 3.21)
project(SimQuestion)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
add_executable(SimQuestion main.cpp Mywarp.cpp HeadFile.h GetFeatureMatchNum.cpp)
target_include_directories(SimQuestion PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_directories(SimQuestion PRIVATE)
target_link_libraries(SimQuestion PRIVATE ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})