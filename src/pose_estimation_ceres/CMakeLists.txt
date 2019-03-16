cmake_minimum_required( VERSION 2.8 )
project( pose_estimation_ceres )

#set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )
link_directories(${CERES_LIBRARY_DIRS})
add_definitions(${CERES_DEFINITIONS})

message(STATUS "CERES_INCLUDE_DIRS:" ${CERES_INCLUDE_DIRS})
message(STATUS "CERES_LIBRARY_DIRS:" ${CERES_LIBRARY_DIRS})
message(STATUS "CERES_DEFINITIONS:" ${CERES_DEFINITIONS})


# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

#add_executable( pose_estimation_ceres simple_bundle_adjuster.cc )
add_executable( pose_estimation_ceres pose_estimation_ceres.cpp)
# 与Ceres和OpenCV链接
target_link_libraries( pose_estimation_ceres ${CERES_LIBRARIES} ${OpenCV_LIBS} )

install(TARGETS  pose_estimation_ceres 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)