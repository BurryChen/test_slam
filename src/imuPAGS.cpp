#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/Byte.h>
#include <std_msgs/Empty.h> 
#include <iostream>
#include <std_msgs/Float32.h>
#include <cstdlib>
#include <string>
#include <stdio.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include "GlobalConst.h"
using namespace std;

serial::Serial ser; //声明串口对象 

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imuPAGS");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 50);
	tf::TransformBroadcaster imu_broadcaster;

	try
	{
		//设置串口属性，并打开串口 
		ser.setPort("/dev/ttyUSB0"); 
		ser.setBaudrate(115200); 
		serial::stopbits_t st=serial::stopbits_one;
		ser.setStopbits(st);
		serial::parity_t pt=serial::parity_none;
		ser.setParity(pt);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
		ser.setTimeout(to); 
		ser.open(); 
	} 
	catch (serial::IOException& e) 
	{
		ROS_ERROR_STREAM("Unable to open port aaa ");
		return -1; 
	} 
  
	//检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()) 
	{ 
		ROS_INFO_STREAM("Serial Port initialized"); 
	} 
	else 
	{
		return -1;      
	}   
   
	double x,y,z,vx,vy,vz,q1,q2,q3,q4,ptich,roll,yaw;   //INS,融合后的结果
	double ang_vx,ang_vy,ang_vz,line_ax,line_ay,line_az,dt;   //

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(100.0);
	//初始位置姿态
	x=y=z=0;
	while(n.ok()&&ser.available())
	{
		ros::spinOnce();              
		// check for incoming messages
		uint8_t head[2];
		ser.read(head, 1);
		if (head[0] == 170)
		{
			ser.read(head, 1);
			if (head[0] == 85)
			{
				uint8_t data[98];
				ser.read(data, 98);
				//int time = data[30] * 256 * 256* 256 + data[31] * 256 * 256 + data[32] * 256 + data[33];
					
				uint8_t index;
				index = data[0];
				uint8_t length = data[1];
					
				current_time = ros::Time::now();
				dt = (current_time - last_time).toSec();
					
				//Gyro,陀螺					
				ang_vx = data[2] * 65536 + data[3] * 256 + data[4];
				ang_vy = data[5] * 65536 + data[6] * 256 + data[7];
				ang_vz = data[8] * 65536 + data[9] * 256 + data[10];
				if (ang_vx >= pow(2, 23)) ang_vx = ang_vx - pow(2, 24);
				if (ang_vy >= pow(2, 23)) ang_vy = ang_vy - pow(2, 24);
				if (ang_vz >= pow(2, 23)) ang_vz = ang_vz - pow(2, 24);
				ang_vx *= 1e-4;
				ang_vy *= 1e-4;
				ang_vz *= 1e-4;
				//Acc，加速度计
				line_ax = data[11] * 65536 + data[12] * 256 + data[13];
				line_ay = data[14] * 65536 + data[15] * 256 + data[16];
				line_az = data[17] * 65536 + data[18] * 256 + data[19];
				if (line_ax >= pow(2, 23)) line_ax = line_ax - pow(2, 24);
				if (line_ay >= pow(2, 23)) line_ay = line_ay - pow(2, 24);
				if (line_az >= pow(2, 23)) line_az = line_az - pow(2, 24);
				line_ax *= 1e-5;
				line_ay *= 1e-5;
				line_az *= 1e-5;
				

				
				//INS_Att 
				ptich = data[62] * 256 + data[63];
				roll = data[64] * 256 + data[65];
				yaw = data[66] * 256 + data[67];
				if (ptich >= pow(2, 15)) ptich -= pow(2, 16);
				if (roll >= pow(2, 15)) roll -= pow(2, 16);
				if (yaw >= pow(2, 15)) yaw -= pow(2, 16);
				ptich *= 0.01;
				roll *= 0.01;
				yaw *= 0.01;
				ptich = ptich / 2;
				roll = roll / 2;
				yaw = yaw / 2;
				ptich *=Pi/180 ;
				roll *=Pi/180;
				yaw *=Pi/180;

							
				//since all imu is 6DOF we'll need a quaternion created from yaw
				geometry_msgs::Quaternion imu_quat ;
				imu_quat.w = cos(ptich)*cos(roll)*cos(yaw) + sin(ptich)*sin(roll)*sin(yaw);
				imu_quat.x = sin(roll)*cos(yaw)*cos(ptich) - cos(roll)*sin(yaw)*sin(ptich);
				imu_quat.y = cos(roll)*sin(yaw)*cos(ptich) + sin(roll)*cos(yaw)*sin(ptich);
				imu_quat.z = cos(roll)*cos(yaw)*sin(ptich) - sin(roll)*sin(yaw)*cos(ptich);
				
				
				//next, we'll publish the imu message over ROS
				// Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
				sensor_msgs::Imu  imu;
				imu.header.stamp = current_time;
				imu.header.frame_id = "imu_link";
				
				//set the orientation				
				//imu.orientation=imu_quat;				
							
				//set the angular_velocity				
				imu.angular_velocity.x=ang_vx*Pi/180;
				imu.angular_velocity.y=ang_vy*Pi/180;
				imu.angular_velocity.z=ang_vz*Pi/180;
				
				//set the  linear_acceleration
				double g=9.794;                  //武汉重力加速速
				imu.linear_acceleration.x=line_ax*g;
				imu.linear_acceleration.y=line_ay*g;
				imu.linear_acceleration.z=line_az*g;

				//publish the message
				imu_pub.publish(imu);
				last_time = current_time;
					
			}
		}
		r.sleep();
	}
}