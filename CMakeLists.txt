cmake_minimum_required(VERSION 2.8.3)
project(test_slam)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp rospy std_msgs genmsg message_generation
  serial
  sensor_msgs
  tf
  nav_msgs
)

## Declare ROS messages and services
add_message_files(
FILES Num.msg
)
add_service_files(
  FILES
  AddTwoInts.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)

#rosbuild_genmsg()
#rosbuild_gensrv()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hello src/hello.cpp) #其实会生成hello.exe的名字
Target_link_libraries(hello ${catkin_LIBRARIES}) #添加节点编译需要的库文件。

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker test_slam_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener test_slam_generate_messages_cpp)

#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server test_slam_gencpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client test_slam_gencpp)

add_executable(port src/port.cpp)
target_link_libraries(port ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(imuPAGS src/imuPAGS.cpp)
target_link_libraries(imuPAGS ${catkin_LIBRARIES})





