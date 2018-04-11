# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# 
# project()

#find_package(PCL 1.8 REQUIRED )
find_package(PCL 1.8 REQUIRED PATHS /home/whu/slam_ws/install/share/pcl-1.8)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_fit planar_fit.cpp)
target_link_libraries (planar_fit ${PCL_LIBRARIES})

install(TARGETS  planar_fit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
