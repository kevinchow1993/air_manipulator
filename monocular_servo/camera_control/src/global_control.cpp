
#include "ros/ros.h"
#include <stdio.h>
#include "serial.h"
#include <camera_control/ServoPos.h>
#include <camera_control/TagLost.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen> 

#include <camera_control/set_interest_ID.h>
#include <camera_control/World_pose.h>

//#include "serial_am/ikMsg.h"
#include "am_controller/servoset_srv.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

#include <camera_control/rc_state.h>

int global_state,process_state;
enum state_condition{	FLY_ALONG_MARKER,
			FLY_ALONG_TARGET,
			HOLD_AND_REACH,
			HOLD_AND_GRASP,
			GRASP_RISE,
			GRASP_DONE,
			BACK_HOME_AND_LANGING,
			FORCE_LANGING
			};

double mav_x,mav_y,mav_z,mav_t;
double mav_qx,mav_qy,mav_qz,mav_qw;
int valid_target_cnt,valid_marker_cnt,valid_work_space;

double marker_wx,marker_wy,marker_wz;
double target_wx,target_wy,target_wz;
double target_bx,target_by,target_bz;

double sx,sy,sz,sw;

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

// bool target_pose_callback(camera_control::World_pose::Request &msg,camera_control::World_pose::Response &res)
void target_pose_callback(camera_control::World_pose msg)
{
	if(msg.id==25)
	{
		valid_target_cnt++;
		target_wx=msg.wx;target_wy=msg.wy;target_wz=msg.wz;
		target_bx=msg.bx;target_by=msg.by;target_bz=msg.bz;
	}
	else if(msg.id==35)
	{
		valid_marker_cnt++;
		marker_wx=msg.wx;marker_wy=msg.wy;marker_wz=msg.wz;
	}




}

bool safe_state_callback(camera_control::rc_state::Request &msg,camera_control::rc_state::Response &res)
{
	if(msg.state==1)global_state=process_state;
	else if(msg.state==2)global_state=BACK_HOME_AND_LANGING;
	else if(msg.state==3)global_state=FORCE_LANGING;
	res.is_done=1;
	return true;
}

void generate_path(nav_msgs::Path &m_path,double des_x,double des_y,double des_z,double prevent_xy_gap,bool use_path_dir)
{
	if(des_z>1.2)des_z=1.2;
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

void limited_grasp_call(ros::ServiceClient &servoseter,am_controller::servoset_srv &msg)
{
	static double last_call_time=0;

	if(ros::Time::now().toSec()-last_call_time>0.5)
	{
		servoseter.call(msg);	
		last_call_time=ros::Time::now().toSec();
	}
	
}


int main(int argc, char** argv)
{

	ros::init(argc,argv,"global_control");
	ros::NodeHandle n;


	ros::Rate loop_rate(50);
	global_state=FLY_ALONG_MARKER;
	process_state=FLY_ALONG_MARKER;

	// ros::ServiceClient ik_puber = n.serviceClient<serial_am::ikSrv>("/serial_am/Link_pose");
	// ros::ServiceClient graper_puber = n.serviceClient<serial_am::graper>("/serial_am/graper");
	//ros::Publisher ik_puber = n.advertise<serial_am::ikMsg>("/Camera_servo_avtion/msg",1);

	ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");

	// ros::ServiceServer target_rev = n.advertiseService("/target_tracker/target_world_pose", target_pose_callback);
	ros::Subscriber target_rev=n.subscribe("/target_tracker/target_world_pose",1,target_pose_callback);

	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose

	ros::Publisher Path_puber = n.advertise<nav_msgs::Path>("/flx_path_plan",1);//gererator path and pub

	//ros::ServiceClient m_track_target = n.serviceClient<camera_control::set_interest_ID>("/target_tracker/set_interest_id");

	ros::ServiceServer m_safe_state = n.advertiseService("/flx_safe_state", safe_state_callback);

	double time_control;// =ros::Time::now().toSec(); 
	valid_target_cnt=0;
	valid_marker_cnt=0;

	double rise_pose_x,rise_pose_y,rise_pose_z;
	sx=0;sy=0;sz=0;sw=1;

	while(ros::ok())
	{
		if(valid_marker_cnt>1000)valid_marker_cnt=1000;
		if(valid_target_cnt>1000)valid_target_cnt=1000;
		if(valid_work_space>1000)valid_work_space=1000;
		switch(global_state)
		{
			case FLY_ALONG_MARKER:
			{
				nav_msgs::Path m_path;
				generate_path(m_path,marker_wx,marker_wy,marker_wz+0.9,0.25,true);
				if(m_path.poses.size())Path_puber.publish(m_path);

				if(valid_target_cnt>4)
				{
					// camera_control::set_interest_ID msg;
					// msg.request.id=25;
					// ROS_INFO("set_interest_ID 25.....");
					// m_track_target.call(msg);
					// ROS_INFO("set_interest_ID 25 done.....");
					// if (msg.response.id_ret==25)
					// {
						global_state=FLY_ALONG_TARGET;
						process_state=FLY_ALONG_TARGET;
					// }
				}
				ROS_INFO("FLY_ALONG_MARKER.....");
				if(mav_z>0.6)
				{
					am_controller::servoset_srv msg;
					msg.request.cmd=6;
					msg.request.pos1=0;
					msg.request.pos2=0;//pi/2;
					msg.request.pos3=0;
					msg.request.pos4=-0.19;
					msg.request.action_time=3000;
					limited_grasp_call(servoseter,msg);
				}
				break;	
			}
			case FLY_ALONG_TARGET:
			{
				nav_msgs::Path m_path;
				generate_path(m_path,target_wx,target_wy,target_wz+0.45,0.25,true);
				if(m_path.poses.size())Path_puber.publish(m_path);

				am_controller::servoset_srv msg;
				msg.request.cmd=7;
				msg.request.pos1=0;
				msg.request.pos2=0;//pi/2;
				msg.request.pos3=target_bx;
				msg.request.pos4=target_bz;
				msg.request.action_time=800;
				servoseter.call(msg);
				valid_work_space+=msg.response.is_done;
				if(valid_work_space>2)
				{
					global_state=HOLD_AND_REACH;
					process_state=HOLD_AND_REACH;
					time_control=ros::Time::now().toSec(); 
				}

				ROS_INFO("FLY_ALONG_TARGET.....");
				break;	
			}
			case HOLD_AND_REACH:
			{
				am_controller::servoset_srv msg;
				msg.request.cmd=6;
				msg.request.pos1=0;
				msg.request.pos2=0;//pi/2;
				msg.request.pos3=target_bx;
				msg.request.pos4=target_bz;
				msg.request.action_time=1000;
				// servoseter.call(msg);
				//6 0 0 0.35 -0.09 1000
				limited_grasp_call(servoseter,msg);
				ROS_INFO("HOLD_AND_REACH.....");

				if(ros::Time::now().toSec()-time_control>2.0)
				{
					global_state=HOLD_AND_GRASP;
					process_state=HOLD_AND_GRASP;
					time_control=ros::Time::now().toSec(); 
					sx=mav_qx;sy=mav_qy;sz=mav_qz;sw=mav_qw;
				}

				break;	
			}
			case HOLD_AND_GRASP:
			{
				am_controller::servoset_srv msg;
				msg.request.cmd=4;
				msg.request.action_time=2;
				// servoseter.call(msg);
				//6 0 0 0.35 -0.09 1000
				limited_grasp_call(servoseter,msg);
				ROS_INFO("HOLD_AND_GRASP.....");

				if(ros::Time::now().toSec()-time_control>1.2)
				{
					global_state=GRASP_RISE;
					process_state=GRASP_RISE;
					time_control=ros::Time::now().toSec(); 
					rise_pose_x=mav_x;
					rise_pose_y=mav_y;
					rise_pose_z=mav_z;
				}

				break;	
			}
			case GRASP_RISE:
			{
				nav_msgs::Path m_path;
				generate_path(m_path,rise_pose_x,rise_pose_y,rise_pose_z+0.05,0,false);
				if(m_path.poses.size())Path_puber.publish(m_path);


				ROS_INFO("GRASP_RISE.....");

				if(ros::Time::now().toSec()-time_control>3.0)
				{
					global_state=GRASP_DONE;
					process_state=GRASP_DONE;
					time_control=ros::Time::now().toSec(); 
				}

				break;	
			}
			case GRASP_DONE:
			{
				am_controller::servoset_srv msg;
				msg.request.cmd=5;
				msg.request.pos1=-10;
				msg.request.pos2=170;//pi/2;
				msg.request.pos3=-70;
				msg.request.pos4=0;
				msg.request.action_time=3000;
				limited_grasp_call(servoseter,msg);
				ROS_INFO("GRASP_DONE.....");

				if(ros::Time::now().toSec()-time_control>1.0)
				{
					global_state=BACK_HOME_AND_LANGING;
					process_state=BACK_HOME_AND_LANGING;
					time_control=ros::Time::now().toSec(); 
				}

				break;	
			}
			case BACK_HOME_AND_LANGING:
			{
				am_controller::servoset_srv msg;
				msg.request.cmd=5;
				msg.request.pos1=-10;
				msg.request.pos2=170;//pi/2;
				msg.request.pos3=-70;
				msg.request.pos4=0;
				msg.request.action_time=2000;
				limited_grasp_call(servoseter,msg);
				if(ros::Time::now().toSec()-time_control>0.5)
				{
					nav_msgs::Path m_path;
					generate_path(m_path,0,0,mav_z,0,false);
					generate_path(m_path,0,0,-5,0,false);
					if(m_path.poses.size())Path_puber.publish(m_path);
					time_control=ros::Time::now().toSec(); 
				}



				ROS_INFO("BACK_HOME_AND_LANGING.....");
				break;	
			}
			case FORCE_LANGING:
			{	
				am_controller::servoset_srv msg;
				msg.request.cmd=5;
				msg.request.pos1=-10;
				msg.request.pos2=170;//pi/2;
				msg.request.pos3=-70;
				msg.request.pos4=0;
				msg.request.action_time=2000;
				limited_grasp_call(servoseter,msg);
				if(ros::Time::now().toSec()-time_control>0.5)
				{
					nav_msgs::Path m_path;
					generate_path(m_path,mav_x,mav_y,-5,0,false);
					if(m_path.poses.size())Path_puber.publish(m_path);
					time_control=ros::Time::now().toSec(); 
				}
				ROS_INFO("FORCE_LANGING.....");
				break;	
			}
		}



		ros::spinOnce();
		loop_rate.sleep();
	}


	
	return 0;
}
