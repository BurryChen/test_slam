#!/bin/sh
#
#roscore
FILE="/media/whu/HD_WHU_4T/linuxdata20180408/T2-L1-1-L2-1.bag"
# 1 scan_to_cloud_converter_node

rosrun scan_to_cloud_converter scan_to_cloud_converter_node __name:=converter1 /scan:=/horizontal_laser_2d /cloud:=/horizontal_point&

rosrun scan_to_cloud_converter scan_to_cloud_converter_node __name:=converter2 /scan:=/vertical_right_laser_2d /cloud:=/vertical_right_point& 
# rosbag record /vertical_right_point

# 2 pointcloud_to_pcd
mkdir ${FILE}_scan_hori ${FILE}_scan_ver

rosrun pcl_ros pointcloud_to_pcd __name:=pointcloud_to_pcd1 input:=/horizontal_point _prefix:=${FILE}_scan_hori/h_ &

rosrun pcl_ros pointcloud_to_pcd __name:=pointcloud_to_pcd2 input:=/vertical_right_point _prefix:=${FILE}_scan_ver/v_ &


# 3

rosbag play ${FILE} -r 0.5  

#x-terminal-emulator &

#rosbag record -O ${FILE}_point /horizontal_laser_2d /vertical_right_laser_2d /horizontal_point /vertical_right_point


