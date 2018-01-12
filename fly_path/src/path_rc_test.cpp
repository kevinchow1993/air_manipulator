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
#include <mavros_msgs/RCIn.h>

using namespace std;

enum chan{c1,c2,c3,c4,c5,c6,c7,c8};
uint16_t channel[8]={0};

double mav_x,mav_y,mav_z,mav_t;
double mav_qx,mav_qy,mav_qz,mav_qw;

double sx,sy,sz,sw;

void RC_read_Callback(mavros_msgs::RCIn RCmsg)
{

	for (int i = 0; i < RCmsg.channels.size(); ++i)
		channel[i]=RCmsg.channels[i];
}


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

  	static int cnt_s=0;
  	if(cnt_s<10)
  	{
  		sx=mav_qx;
  		sy=mav_qy;
  		sz=mav_qz;
  		sw=mav_qw;
  		cnt_s++;
  	}

}

void get_direction_by_line(double m_x,double m_y,double d_x,double d_y,double &qw,double &qx,double &qy,double &qz)
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

void generate_path(nav_msgs::Path &m_path,double des_x,double des_y,double des_z,double prevent_xy_gap,bool use_path_dir)
{
	//if(des_z>1.2)des_z=1.2;
	m_path.header.stamp = ros::Time::now();
	m_path.header.frame_id = "/world";
	double qx,qy,qz,qw;
	if(use_path_dir)
		get_direction_by_line(mav_x,mav_y,des_x,des_y,qw,qx,qy,qz);
	else 
	{
		qx=sx;
		qy=sy;
		qz=sz;
		qw=sw;
	}
	//Fast take off
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "fcu";
	pose.pose.orientation.x = qx;
	pose.pose.orientation.y = qy;
	pose.pose.orientation.z = qz;
	pose.pose.orientation.w =qw;
	pose.pose.position.x = mav_x;
	pose.pose.position.y =mav_y;
	pose.pose.position.z = des_z;
	//if(get_err(pose.pose.position.x,pose.pose.position.y,0,des_x,des_y,0)>=prevent_xy_gap)m_path.poses.push_back(pose);
	double des_step,des_length;
	des_step=0.05;
	des_length=get_err(mav_x,mav_y,des_z,des_x,des_y,des_z);
	double delt_x=des_step/des_length*(des_x-mav_x);
	double delt_y=des_step/des_length*(des_y-mav_y);
	//get_direct_by_line();
	for (int i = 1; i < des_length/des_step; ++i)
	{
		pose.pose.position.x =mav_x+i*delt_x;
		pose.pose.position.y =mav_y+i*delt_y;
		pose.pose.position.z = des_z;
		if(get_err(pose.pose.position.x,pose.pose.position.y,0,des_x,des_y,0)>=prevent_xy_gap)m_path.poses.push_back(pose);
	}
	pose.pose.position.x =des_x;
	pose.pose.position.y =des_y;
	pose.pose.position.z = des_z;
	if(get_err(pose.pose.position.x,pose.pose.position.y,0,des_x,des_y,0)>=prevent_xy_gap)m_path.poses.push_back(pose);

}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "path_generator_test");
  	ros::NodeHandle n;


  	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
  	ros::Publisher Path_puber = n.advertise<nav_msgs::Path>("/flx_path_plan",1);

  	ros::Subscriber RC_suber = n.subscribe("/mavros/rc/in", 1, RC_read_Callback);

  	sx=0;sy=0;sz=0;sw=1;

  	ros::Rate loop_rate(10);
  	while (ros::ok())
	{
		if(mav_z>0.8)printf("z:%lf\n",mav_z );
		if(channel[c1])
		{
			//printf("%d %d %d %d %d %d %d %d\n",channel[c1],channel[c2],channel[c3],channel[c4],channel[c5],channel[c6],channel[c7],channel[c8]);
			if (channel[c8]<=1800)
			{
				nav_msgs::Path m_path;
				generate_path(m_path,	-1.0,
							-1.0,
							0.9,
							0,false);
				if(m_path.poses.size())Path_puber.publish(m_path);
				
			}
			else if (channel[c8]>1800)
			{
				nav_msgs::Path m_path;
				generate_path(m_path,mav_x,mav_y,-5,0,false);
				if(m_path.poses.size())Path_puber.publish(m_path);
				//ROS_INFO("FORCE_LANGING.....");
			}
		
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}