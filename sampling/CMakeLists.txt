cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sampling)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (downsampling downsampling.cpp)
target_link_libraries (downsampling ${PCL_LIBRARIES})
