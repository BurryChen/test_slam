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
------------------------------------------    20170327
1、linux git 下文件拷入拷出
git config  core.fileMode false

------------------------------------------   20170327
1、cmake 指定目录
/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/home/whu/slam_ws/install -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_DEVEL_PREFIX=/home/whu/slam_ws/devel -DCMAKE_INSTALL_PREFIX=/home/whu/slam_ws/install /home/whu/slam_ws/src/VeloView/

    cd pcl  
    mkdir build  
    cd build  
    cmake -DCATKIN_DEVEL_PREFIX=/home/whu/slam_ws/devel -DCMAKE_INSTALL_PREFIX=/home/whu/slam_ws/install ..  
    sudo make install  

2、catkin_make 指定安装目录
catkin_make  -DCATKIN_DEVEL_PREFIX=/home/whu/slam_ws/devel -DCMAKE_INSTALL_PREFIX=/home/whu/slam_ws/install install

catkin_make  -DCATKIN_DEVEL_PREFIX=/home/whu/slam_ws/devel -DCMAKE_INSTALL_PREFIX=/usr/local install

---------------------------------------20180504
#find_package(PCL 1.8 REQUIRED COMPONENTS common io PATHS /home/whu/slam_ws/install/share/pcl-1.8) 

---------------------------------------------------------  2018.04.03
PCL_DIR /home/whu/slam_ws/install/share/pcl-1.8


---------------------------------------------------------  2018.04.10
1)src/pose_estimation/pose_estimation_laser  
3D连接点求取坐标系转换参数RT
CMakeList.txt  list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/pose_estimation/cmake_modules )
2）已知路径下第三方安装库的使用
exp:fing_package PATHS 后面加 <path-of-PCLConfig.cmake>
find_package(PCL 1.8 REQUIRED PATHS /home/whu/slam_ws/install/share/pcl-1.8)

-------------------------------------------------------   2018.05.10
lib example 不再加入工程中， build 及debug 全在下载source中进行，debug 使用gdb工具
cmake -DCMAKE_BUILD_TYPE=Debug .. 
cmakelist中#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )  不能为O3,优化层度太高，无debug信息
-------------------------------------------------------   2018.8.12)
1 启动gdb的两种方式
1）gdb app
break  1
run [argv1] [argv2]
2) (gdb)file app 
    break 1
    run [argv1] [argv2] ...

-------------------------------------------------------   2018.05.10
ceres lib的使用：

1） # 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )
link_directories(${CERES_LIBRARY_DIRS})
add_definitions(${CERES_DEFINITIONS})

2） ndt_omp依赖于ceres，链接器从左往右扫描目标问题和库，先扫描main.o，把未定义的符号放入集合，然后扫描libceres.a
（此时libceres.a的函数没有被用到，跳过），最后扫描libndt_omp.so（发现要调用ceres，
但是已经被略过了，就出现了未定义引用的问题），，
总之关键是从左往右扫描，尽量填补未定义符号集合，否则跳过

target_link_libraries(match
  ndt_omp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${Sophus_LIBRARIES}
)

-------------------------------------------------------   2019.3.15
lidar2visual calibartion
pose_estimation_2d3d_l2v.cpp