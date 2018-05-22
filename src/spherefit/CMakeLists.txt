# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# 
# project(spherefit)

find_package(PCL 1.7 REQUIRED )
#find_package(PCL 1.8 REQUIRED PATHS /home/whu/slam_ws/install/share/pcl-1.8)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (cylinder_segmentation cylinder_segmentation.cpp)
#target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})

add_executable (spherefit_scan spherefit_scan.cpp)
target_link_libraries (spherefit_scan ${PCL_LIBRARIES})

add_executable (spherefit_pointcloud spherefit_pointcloud.cpp)
target_link_libraries (spherefit_pointcloud ${PCL_LIBRARIES})