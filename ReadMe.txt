---------------------------------------------------------Burry Chen 2017.9.5
1、project简介
project:test_slam
目前包含三个executable：hello,takler,listener
2、使用说明
1)编译：同git,可用各种cmake 方式编译（命令行、编辑器均可），需根据s路径ource
2）使用,举个栗子
#roslaunch test_slam message.launch
3）command
$ rosrun test_slam add_two_ints_server     (C++)
$ rosrun test_slam add_two_ints_client 1 3 
-------------------------------------------------------Burry Chen 2017.9.7
3、数据读取包port
   1）launch文件：同时运行lidar和imu包并且存储数据为bag，存储目录为home/.ros/
   2）使用说明:
   laser利用hokuyo_node,imu数据读取包依赖serial包
   打开四个端口
   sudo chmod a+rw /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB0
   sudo chmod a+rw /dev/ttyACM1
   sudo chmod a+rw /dev/ttyACM2
   sudo chmod a+rw /dev/ttyUSB0
   运行roslaunch test_slam sensor.launch



参考
http://www.ncnynl.com/archives/201701/1279.html
--------------------------------------------------------- Burry Chen 2017.9.11
$ sudo chmod a+rw /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB0
$ roslaunch test_slam sensor.launch

Running the reconfigure_gui
http://wiki.ros.org/hokuyo_node/Tutorials/UsingReconfigureGUIToChangeHokuyoLaserParameters
$ rosrun rqt_reconfigure rqt_reconfigure 


----------------------------------------------------------Burry Chen 2017.10.18
--------------------------------------------------------- Burry Chen 2017.9.11
改为cartographer的数据借口
$ sudo chmod a+rw /dev/ttyACM0  /dev/ttyUSB0
$ roslaunch test_slam sensor2.launch
$ rosbag play '/home/whu/laser_slam_openSources-master/hector_slam_example/data/_2017-10-20-17-11-22.bag

--------------------------------------------------------- Burry Chen 2017.10.22
project name =sensorpub;
node_xsens_name=xsenspub;

---------------------------------------------------------  2018.04.03
PCL_DIR /home/whu/slam_ws/install/share/pcl-1.8


---------------------------------------------------------  2018.04.10
1)src/pose_estimation/pose_estimation_laser  
3D连接点求取坐标系转换参数RT
CMakeList.txt  list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/pose_estimation/cmake_modules )
2）已知路径下第三方安装库的使用
exp:fing_package PATHS 后面加 <path-of-PCLConfig.cmake>
find_package(PCL 1.8 REQUIRED PATHS /home/whu/slam_ws/install/share/pcl-1.8)
