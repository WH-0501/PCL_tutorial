cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcd_combine)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_registration pcd_registration.cpp)
target_link_libraries (pcd_registration ${PCL_LIBRARIES})

add_executable (merge_pcd merge_pcd.cpp)
target_link_libraries (merge_pcd ${PCL_LIBRARIES})
