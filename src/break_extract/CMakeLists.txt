# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PCL 1.7 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (cylinder_segmentation cylinder_segmentation.cpp)
#target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})

add_executable (break_extract break_extract.cpp)
target_link_libraries (break_extract ${PCL_LIBRARIES})

install(TARGETS  break_extract  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)