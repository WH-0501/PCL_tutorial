cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(segmentations)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL LIBRARYS" ${PCL_LIBRARIES})
add_executable (plane_model_seg plane_model_seg.cpp)
target_link_libraries (plane_model_seg ${PCL_LIBRARIES})

add_executable (don_segmentation don_segmentation.cpp)
target_link_libraries (don_segmentation ${PCL_LIBRARIES})

add_executable (region_growing_segmentation region_growing_segmentation.cpp)
target_link_libraries (region_growing_segmentation ${PCL_LIBRARIES})
