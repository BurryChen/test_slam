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
	ros::init(argc, argv, "odometry");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

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
				line_az-=1;
				
				//加速度积分位置
				x += line_ax * dt* dt/2;
                                y += line_ay * dt* dt/2;
				z += line_az * dt* dt/2;
			        printf("%f,(%f,%f,%f)\n",dt,x,y,z);
				
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
				q1 = cos(ptich)*cos(roll)*cos(yaw) + sin(ptich)*sin(roll)*sin(yaw);
				q2= sin(roll)*cos(yaw)*cos(ptich) - cos(roll)*sin(yaw)*sin(ptich);
				q3 = cos(roll)*sin(yaw)*cos(ptich) + sin(roll)*cos(yaw)*sin(ptich);
				q4 = cos(roll)*cos(yaw)*sin(ptich) - sin(roll)*sin(yaw)*cos(ptich);
				
				//INS_Pos
				bool GPS_Valid=data[61];
				//printf("%d\n",data[61]);
				if(GPS_Valid == true)
				{				  				//INS_Vn   
				  vx = data[68] * 65536 + data[69] * 256 + data[70];
				  vy = data[71] * 65536 + data[72] * 256 + data[73];
				  vz = data[74] * 65536 + data[75] * 256 + data[76];
				  if (vx >= pow(2, 23)) vx-= pow(2, 24);
				  if (vy >= pow(2, 23)) vy -= pow(2, 24);
				  if (vz >= pow(2, 23)) vz -= pow(2, 24);
				  vx *= 1e-4;
				  vy *= 1e-4;
				  vz *= 1e-4;				  
				  LLA_POS lla_pos ;
				  XYZ_POS xyz_pos;				  
				  lla_pos.dLon = 16777216*data[77]+data[78] * 65536 + data[79] * 256 + data[80];				
				  lla_pos.dLat = 16777216*data[81]+data[82] * 65536 + data[83] * 256 + data[84];
			          lla_pos.dAlt = data[85] * 65536 + data[86] * 256 + data[87];
				  if (lla_pos.dLon >= pow(2, 31))  lla_pos.dLon -= pow(2, 32);
				  if (lla_pos.dLat >= pow(2, 31))  lla_pos.dLat -= pow(2, 32);
				  if (lla_pos.dAlt >= pow(2, 23))  lla_pos.dAlt -= pow(2, 24);
				  lla_pos.dLon *= 1e-7; 
				  lla_pos.dLon*=Pi/180;
				  lla_pos.dLat *= 1e-7; 
				  lla_pos.dLat*=Pi/180;
				  lla_pos.dAlt *= 1e-3;	
				  LLA2XYZ(&lla_pos,&xyz_pos);
				  XYZ2LLA(&xyz_pos,&lla_pos);
				  x=xyz_pos.dX;
				  y=xyz_pos.dY;
				  z=xyz_pos.dZ;
				}
				else
				{
				  //x /= 1000;
                                  //y /= 1000;
				  //z /= 1000;
				}
							
				//since all odometry is 6DOF we'll need a quaternion created from yaw
				geometry_msgs::Quaternion odom_quat ;
				odom_quat.w = cos(ptich)*cos(roll)*cos(yaw) + sin(ptich)*sin(roll)*sin(yaw);
				odom_quat.x = sin(roll)*cos(yaw)*cos(ptich) - cos(roll)*sin(yaw)*sin(ptich);
				odom_quat.y = cos(roll)*sin(yaw)*cos(ptich) + sin(roll)*cos(yaw)*sin(ptich);
				odom_quat.z = cos(roll)*cos(yaw)*sin(ptich) - sin(roll)*sin(yaw)*cos(ptich);

				//first, we'll publish the transform over tf
				geometry_msgs::TransformStamped odom_trans;
				odom_trans.header.stamp = current_time;
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_link";
    
				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = z;
				odom_trans.transform.rotation = odom_quat;

				//send the transform
				odom_broadcaster.sendTransform(odom_trans);

				//next, we'll publish the odometry message over ROS
				nav_msgs::Odometry odom;
				odom.header.stamp = current_time;
				odom.header.frame_id = "odom";

				//set the position
				odom.pose.pose.position.x = x;
				odom.pose.pose.position.y = y;
				odom.pose.pose.position.z = z;
				odom.pose.pose.orientation = odom_quat;

				//set the velocity
				odom.child_frame_id = "base_link";
				odom.twist.twist.linear.x = vx;
				odom.twist.twist.linear.y = vy;
				odom.twist.twist.linear.z = vz;
				odom.twist.twist.angular.z = ang_vx;
				odom.twist.twist.angular.z = ang_vy;
				odom.twist.twist.angular.z = ang_vz;

				//publish the message
				odom_pub.publish(odom);
				last_time = current_time;
					
			}
		}
		r.sleep();
	}
}