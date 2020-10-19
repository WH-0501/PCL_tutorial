cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(velodyne_reader)

#find_package(PCL 1.2 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (velodyne_convert velodyne_convert.cpp)
target_link_libraries (velodyne_convert ${PCL_LIBRARIES})

add_executable (ply2pcd convert_ply_to_pcd.cpp)
target_link_libraries (ply2pcd ${PCL_LIBRARIES})
