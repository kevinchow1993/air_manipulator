#include "ros/ros.h"
#include "ros/time.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

using namespace std;


double mav_x,mav_y,mav_z,mav_t;
double mav_qx,mav_qy,mav_qz,mav_qw;


void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
	mav_x=msg->pose.position.x;
  	mav_y=msg->pose.position.y;
  	mav_z=msg->pose.position.z;
  	mav_qx=msg->pose.orientation.x;
	mav_qy=msg->pose.orientation.y;
	mav_qz=msg->pose.orientation.z;
	mav_qw=msg->pose.orientation.w;
  	mav_t=msg->header.stamp.toSec();
}

void get_direct_by_line(double m_x,double m_y,double d_x,double d_y,double &qw,double &qx,double &qy,double &qz)
{
	double fai,theta,psai;
	fai=0.0;theta=0.0;
	psai=atan2(d_y-m_y,d_x-m_x);
	qw=cos(fai/2)*cos(theta/2)*cos(psai/2)+sin(fai/2)*sin(theta/2)*sin(psai/2);
	qx=sin(fai/2)*cos(theta/2)*cos(psai/2)-cos(fai/2)*sin(theta/2)*sin(psai/2);
	qy=cos(fai/2)*sin(theta/2)*cos(psai/2)+sin(fai/2)*cos(theta/2)*sin(psai/2);
	qz=cos(fai/2)*cos(theta/2)*sin(psai/2)-sin(fai/2)*sin(theta/2)*cos(psai/2);
}

double get_err(double mx,double my,double mz,double des_x,double des_y,double des_z)
{
	double pose_err_x=fabs(mx-des_x);
	double pose_err_y=fabs(my-des_y);
	double pose_err_z=fabs(mz-des_z);
	double move_err=sqrt(pose_err_x*pose_err_x+pose_err_y*pose_err_y+pose_err_z*pose_err_z);
	return 	move_err;
}

void print_test(geometry_msgs::PoseStamped pose)
{
	printf("x:%lf y:%lf z:%lf\n", pose.pose.position.x ,pose.pose.position.y,pose.pose.position.z);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "path_generator_test");
  	ros::NodeHandle n;


  	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
  	ros::Publisher Path_puber = n.advertise<nav_msgs::Path>("/flx_path_plan",1);


  	ros::Rate loop_rate(50);
  	while (ros::ok())
	{
		static int cnt=0;
		cnt++;
		nav_msgs::Path m_path;
		m_path.header.stamp = ros::Time::now();
		m_path.header.frame_id = "world";
		
		double qx,qy,qz,qw;
		//Fast take off
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = ros::Time::now();
			pose.header.frame_id = "fcu";
			pose.pose.orientation.x = mav_qx;
			pose.pose.orientation.y = mav_qy;
			pose.pose.orientation.z = mav_qz;
			pose.pose.orientation.w = mav_qw;
			pose.pose.position.x = mav_x;
			pose.pose.position.y =mav_y;
			pose.pose.position.z = 0.30;
			m_path.poses.push_back(pose);
			pose.pose.position.z = 0.50;
			m_path.poses.push_back(pose);
		//fly to (-1,-1)
			double height=pose.pose.position.z ;
			double des_x,des_y,des_z,des_step,des_length;
			des_x=	-1.0;	des_y=	-1.0;	des_z=	height;
			des_step=0.1;
			des_length=get_err(mav_x,mav_y,height,des_x,des_y,des_z);
			double delt_x=des_step/des_length*(des_x-mav_x);
			double delt_y=des_step/des_length*(des_y-mav_y);
			double delt_z=des_step/des_length*(des_z-height);
			//get_direct_by_line();
			for (int i = 0; i < des_length/des_step; ++i)
			{
				pose.pose.position.x =mav_x+i*delt_x;
				pose.pose.position.y =mav_y+i*delt_y;
				pose.pose.position.z = height+i*delt_z;
				m_path.poses.push_back(pose);
			}
			pose.pose.position.x =des_x;
			pose.pose.position.y =des_y;
			pose.pose.position.z = des_z;
			m_path.poses.push_back(pose);
		//landing
			pose.pose.position.z = 0.30;
			m_path.poses.push_back(pose);
			pose.pose.position.z = 0.0;
			m_path.poses.push_back(pose);
			
		Path_puber.publish(m_path);
	
		ros::spinOnce();
		loop_rate.sleep();
		if(cnt>100)break;
	}

	return 0;

}