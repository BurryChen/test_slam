#include <ros/ros.h> 
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
using namespace std;

serial::Serial ser; //声明串口对象 

//回调函数 
//void write_callback(const std_msgs::String::ConstPtr& msg)
//{
//    ROS_INFO_STREAM("Writing to serial port" <<msg->data);
//    ser.write(msg->data);   //发送串口数据
//}
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "port"); 
    //声明节点句柄 
    ros::NodeHandle imu_nh; 
  
    //发布主题 
  static ros::Publisher imu_pub = imu_nh.advertise<sensor_msgs::Imu>("imudata", 1, false);
  //磁力计
  static ros::Publisher mag_pub = imu_nh.advertise<geometry_msgs::Vector3Stamped>("mag", 1, false);
  //陀螺仪
  //static ros::Publisher att_pub = imu_nh->advertise<geometry_msgs::Pose2D>("att", 1, false);
  //温度
  static ros::Publisher temp_pub = imu_nh.advertise<std_msgs::Float32>("temperature", 1, false);
  //速度计
  static ros::Publisher vel_pub = imu_nh.advertise<geometry_msgs::Vector3Stamped>("vel", 1, false);
  //气压计
  static ros::Publisher hbar_pub = imu_nh.advertise<std_msgs::Float32>("hbar", 1, false);
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

    //指定循环的频率 
    ros::Rate loop_rate(100); 
    sensor_msgs::Imu imu_msg;
    while(ros::ok()&&ser.available())
    { 
  
		if (ser.available()){
			uint8_t head[2];
			ser.read(head, 1);
			if (head[0] == 170)
			{
				ser.read(head, 1);
				if (head[0] == 85)
				{
					uint8_t data[98];
					ser.read(data, 98);
					int time = data[30] * 256 * 256* 256 + data[31] * 256 * 256 + data[32] * 256 + data[33];
					printf("  %d  ", time);

					uint8_t index;
					index = data[0];
					uint8_t length = data[1];
					//printf(" %2x ",data[0]);
					//printf(" %2x ",data[1]);
					//Gyro,Acc
					imu_msg.header.stamp.useSystemTime();
					imu_msg.header.stamp.sec=int(time/1000);
					imu_msg.header.stamp.nsec=time-int(time/1000)*1000;
					
					imu_msg.angular_velocity.x = data[2] * 65536 + data[3] * 256 + data[4];
					imu_msg.angular_velocity.y = data[5] * 65536 + data[6] * 256 + data[7];
					imu_msg.angular_velocity.z = data[8] * 65536 + data[9] * 256 + data[10];
					if (imu_msg.angular_velocity.x >= pow(2, 23)) imu_msg.angular_velocity.x = imu_msg.angular_velocity.x - pow(2, 24);
					if (imu_msg.angular_velocity.y >= pow(2, 23)) imu_msg.angular_velocity.y = imu_msg.angular_velocity.y - pow(2, 24);
					if (imu_msg.angular_velocity.z >= pow(2, 23)) imu_msg.angular_velocity.z = imu_msg.angular_velocity.z - pow(2, 24);
					imu_msg.angular_velocity.x *= 0.0001;
					imu_msg.angular_velocity.y *= 0.0001;
					imu_msg.angular_velocity.z *= 0.0001;
					imu_msg.linear_acceleration.x = data[11] * 65536 + data[12] * 256 + data[13];
					imu_msg.linear_acceleration.y = data[14] * 65536 + data[15] * 256 + data[16];
					imu_msg.linear_acceleration.z = data[17] * 65536 + data[18] * 256 + data[19];
					if (imu_msg.linear_acceleration.x >= pow(2, 23)) imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.x - pow(2, 24);
					if (imu_msg.linear_acceleration.y >= pow(2, 23)) imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.y - pow(2, 24);
					if (imu_msg.linear_acceleration.z >= pow(2, 23)) imu_msg.linear_acceleration.z = imu_msg.linear_acceleration.z - pow(2, 24);
					imu_msg.linear_acceleration.x *= 0.00001;
					imu_msg.linear_acceleration.y *= 0.00001;
					imu_msg.linear_acceleration.z *= 0.00001;
					
					//magn
					if (data[26] == 1)
					{
						geometry_msgs::Vector3Stamped mag_msg;
						//mag_msg.header=time;
						mag_msg.vector.x = data[20] * 256 + data[21];
						mag_msg.vector.y = data[22] * 256 + data[23];
						mag_msg.vector.z = data[24] * 256 + data[25];
						if (mag_msg.vector.x >= pow(2, 15)) mag_msg.vector.x -= pow(2, 16);
						if (mag_msg.vector.y >= pow(2, 15)) mag_msg.vector.y -= pow(2, 16);
						if (mag_msg.vector.z >= pow(2, 15)) mag_msg.vector.z -= pow(2, 16);
						mag_msg.vector.x *= 0.01;
						mag_msg.vector.y *= 0.01;
						mag_msg.vector.z *= 0.01;
						mag_pub.publish(mag_msg);
					}
					//hbar气压计输出
					if (data[29] == 1)
					{
						std_msgs::Float32 hbar_msgs;
						hbar_msgs.data = (data[27] * 256 + data[28])*0.1;
						hbar_pub.publish(hbar_msgs);
					}
					//GPS
					if (data[61] == 1)
					{
						/*int gps_time=data[35]*16*16*16+data[36]*256+data[37]*16+data[38];
						=data[39]*256+data[40]*16+data[41];
						data[42]*256+data[43]*16+data[44];
						data[45]*256+data[46]*16+data[47];*/
					}
					//GPS
					//姿态
					double ptich = data[62] * 256 + data[63];
					double roll = data[64] * 256 + data[65];
					double yaw = data[66] * 256 + data[67];
					if (ptich >= pow(2, 15)) ptich -= pow(2, 16);
					if (roll >= pow(2, 15)) roll -= pow(2, 16);
					if (yaw >= pow(2, 15)) yaw -= pow(2, 16);
					ptich *= 0.01;
					roll *= 0.01;
					yaw *= 0.01;
					ptich = ptich / 2;
					roll = roll / 2;
					yaw = yaw / 2;
					imu_msg.orientation.w = cos(ptich)*cos(roll)*cos(yaw) + sin(ptich)*sin(roll)*sin(yaw);
					imu_msg.orientation.x = sin(roll)*cos(yaw)*cos(ptich) - cos(roll)*sin(yaw)*sin(ptich);
					imu_msg.orientation.y = cos(roll)*sin(yaw)*cos(ptich) + sin(roll)*cos(yaw)*sin(ptich);
					imu_msg.orientation.z = cos(roll)*cos(yaw)*sin(ptich) - sin(roll)*sin(yaw)*cos(ptich);
					imu_pub.publish(imu_msg);
					//vn
					geometry_msgs::Vector3Stamped vel_msg;
					vel_msg.vector.x = data[68] * 65536 + data[69] * 256 + data[70];
					vel_msg.vector.y = data[71] * 65536 + data[72] * 256 + data[73];
					vel_msg.vector.z = data[74] * 65536 + data[75] * 256 + data[76];
					if (vel_msg.vector.x >= pow(2, 23)) vel_msg.vector.x -= pow(2, 24);
					if (vel_msg.vector.y >= pow(2, 23)) vel_msg.vector.y -= pow(2, 24);
					if (vel_msg.vector.z >= pow(2, 23)) vel_msg.vector.z -= pow(2, 24);
					vel_msg.vector.x *= 0.0001;
					vel_msg.vector.y *= 0.0001;
					vel_msg.vector.z *= 0.0001;
					
					vel_pub.publish(vel_msg);
					//temperature
					std_msgs::Float32 temp_msg;
					temp_msg.data = data[93] * 256 + data[94];
					if (temp_msg.data >= pow(2, 15)) temp_msg.data -= pow(2, 16);
					temp_msg.data *= 0.1;
					temp_pub.publish(temp_msg);
					//printf(" %d .", data[93]*16+data[94]);
				}
			}
			//geometry_msgs::Pose2D att_msg;
			//att_msg.=(data[13]*16*16+data[14]*16+data[15])*0.01;//Pitch
			//att_msg.vector.y=(data[13]*16*16+data[14]*16+data[15])*0.01;//Pitch
			//att_pub.publish(att_msg);
		}

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
} 


