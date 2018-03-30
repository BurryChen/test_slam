# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# 
# project(dlg_segmentation)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (dlg_segmentation dlg_segmentation.cpp)
target_link_libraries (dlg_segmentation ${PCL_LIBRARIES})


install(TARGETS  dlg_segmentation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
