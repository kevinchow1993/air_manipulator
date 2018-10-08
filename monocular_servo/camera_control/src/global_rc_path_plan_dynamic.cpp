
#include "ros/ros.h"
#include <stdio.h>
#include "serial.h"
#include <camera_control/ServoPos.h>
#include <camera_control/TagLost.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen> 

#include <camera_control/set_interest_ID.h>
#include <am_controller/Mat_Tba.h>

//#include "serial_am/ikMsg.h"
#include "am_controller/servoset_srv.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

#include <camera_control/rc_state.h>
#include <camera_control/CameraPos.h>
#include <am_controller/success_flag.h>
#include <camera_control/do_trajectory_task.h>

using namespace Eigen; 

int global_state,process_state;
enum state_condition{	FLY_ALONG_MARKER,
			FLY_ALONG_TARGET,
			HOLD_AND_GRASP,
			GRASP_RISE,
			GRASP_DONE,
			BACK_HOME_AND_LANGING,
			FORCE_LANGING
			};

Matrix4d Twb;
double mav_x,mav_y,mav_z,mav_t;
double mav_qx,mav_qy,mav_qz,mav_qw;
int valid_target_cnt,valid_marker_cnt,valid_work_space;

double marker_wx,marker_wy,marker_wz;
double target_wx,target_wy,target_wz;
double target_bx,target_by,target_bz;

double sx,sy,sz,sw;
bool trajectory_follow_success;
double set_yaw,current_yaw;
geometry_msgs::Quaternion direction_yaw;

double last_time;
double now;
double x_dot,y_dot,z_dot;
void get_direction_by_line(double m_x,double m_y,double d_x,double d_y,double &qw,double &qx,double &qy,double &qz,double offset_angle)
{
	double fai,theta,psai;
	fai=0.0;theta=0.0;
	psai=atan2(d_y-m_y,d_x-m_x);
	psai+=offset_angle;
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

void trajectory_follow_success_flag_CallBack(const am_controller::success_flagConstPtr &msg)
{
	if(msg->success_flag==1)
	{
		trajectory_follow_success=true;
	}
}

//***********add by kevin**************//
void CameraPos_CallBack(const camera_control::CameraPos::ConstPtr &msg){
	float yaw_err =msg->cx - 320;
		
	 set_yaw = current_yaw -yaw_err*0.004;
	// ROS_INFO("set_yaw :%f\t*****curret_yaw :%f",set_yaw*180/3.14,current_yaw*180/3.14);
	 
	 tf::quaternionTFToMsg(tf::createQuaternionFromYaw(set_yaw),direction_yaw);
	 
}
//将机体坐标装进全局变量中使用，并发布tf
void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
	now = ros::Time::now().toSec();
	double dt = now-last_time;
	last_time=now;
	//if(dt>0.05){
	x_dot = (msg->pose.position.x-mav_x)/dt;
	y_dot = (msg->pose.position.y-mav_y)/dt;
	z_dot = (msg->pose.position.z-mav_z)/dt;
	mav_x= msg->pose.position.x;
	mav_y = msg->pose.position.y;
	mav_z = msg->pose.position.z;

	mav_x=msg->pose.position.x;
  	mav_y=msg->pose.position.y;
  	mav_z=msg->pose.position.z;
  	mav_qx=msg->pose.orientation.x;
	mav_qy=msg->pose.orientation.y;
	mav_qz=msg->pose.orientation.z;
	mav_qw=msg->pose.orientation.w;
  	mav_t=msg->header.stamp.toSec();
	current_yaw =  tf::getYaw(msg->pose.orientation);
  	Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
	Matrix3d Rwb = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	Vector3d Owb(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	Twb=MatrixXd::Zero(4,4);Twb(3,3)=1.0;
	Twb<<Rwb;Twb.col(3)<<Owb;

    //发布世界坐标系到body坐标系的转换
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(mav_x,mav_y,mav_z ));
	tf::Quaternion tf_q(mav_qx,mav_qy,mav_qz,mav_qw);
	transform.setRotation(tf_q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/body"));

}

// bool target_pose_callback(camera_control::World_pose::Request &msg,camera_control::World_pose::Response &res)
void target_pose_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg)
{
	// if(msg.id==25)
	// {
	// 	valid_target_cnt++;
	// 	target_wx=msg.wx;target_wy=msg.wy;target_wz=msg.wz;
	// 	target_bx=msg.bx;target_by=msg.by;target_bz=msg.bz;
	// }
	// else if(msg.id==35)
	// {
	// 	valid_marker_cnt++;
	// 	marker_wx=msg.wx;marker_wy=msg.wy;marker_wz=msg.wz;
	// }
	Matrix4d Tbar,Twa;
	Tbar(0,0)=Tba_msg->r11;Tbar(0,1)=Tba_msg->r12;Tbar(0,2)=Tba_msg->r13;Tbar(0,3)=Tba_msg->t1;
	Tbar(1,0)=Tba_msg->r21;Tbar(1,1)=Tba_msg->r22;Tbar(1,2)=Tba_msg->r23;Tbar(1,3)=Tba_msg->t2;
	Tbar(2,0)=Tba_msg->r31;Tbar(2,1)=Tba_msg->r32;Tbar(2,2)=Tba_msg->r33;Tbar(2,3)=Tba_msg->t3;
	Tbar(3,0)=0;Tbar(3,1)=0;Tbar(3,2)=0;Tbar(3,3)=1;
	Twa=Twb*Tbar;
	if(Tba_msg->id==25)
	{
		valid_target_cnt++;
		target_wx=Twa(0,3);target_wy=Twa(1,3);target_wz=Twa(2,3);
		target_bx=Tbar(0,3);target_by=Tbar(1,3);target_bz=Tbar(2,3);
	}
	else if(Tba_msg->id==35)
	{
		valid_marker_cnt++;
		marker_wx=Twa(0,3);marker_wy=Twa(1,3);marker_wz=Twa(2,3);
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

void generate_path(nav_msgs::Path &m_path,double des_x,double des_y,double des_z,double prevent_xy_gap,bool use_path_dir,double offset_angle)
{
	if(des_z>1.2)des_z=1.2;
	m_path.header.stamp = ros::Time::now();
	m_path.header.frame_id = "/world";
	double qx,qy,qz,qw;
	if(use_path_dir)//机头朝向目标物飞过去
	{	
		qx = direction_yaw.x;
		qy = direction_yaw.y;
		qz = direction_yaw.z;
		qw = direction_yaw.w;
		//get_direction_by_line(mav_x,mav_y,des_x,des_y,qw,qx,qy,qz,offset_angle);
		}
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
	//pose.pose.orientation = direction_yaw;	//ADD BY KEVIN

	pose.pose.position.x = mav_x;
	pose.pose.position.y = mav_y;
	pose.pose.position.z = des_z;
	//if(get_err(pose.pose.position.x,pose.pose.position.y,0,des_x,des_y,0)>=prevent_xy_gap)m_path.poses.push_back(pose);
	double des_step,des_length;
	des_step=0.05;
	des_length=get_err(mav_x,mav_y,des_z,des_x,des_y,des_z);																								
	double delt_x=des_step/des_length*(des_x-mav_x);
	double delt_y=des_step/des_length*(des_y-mav_y);
	//get_direct_by_line();
	//首先生成路上的轨迹
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
	//如果距离大于限定值，则将点压入轨迹
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

void set_hold_on_path(nav_msgs::Path &m_path)
{
	double qx,qy,qz,qw;
	get_direction_by_line(mav_x,mav_y,target_bx,target_by,qw,qx,qy,qz,-0.08);
	for (int i = 0; i < m_path.poses.size(); ++i)
	{
		 m_path.poses[i].pose.orientation.x = qx;
		 m_path.poses[i].pose.orientation.y = qy;
		 m_path.poses[i].pose.orientation.z = qz;
		 m_path.poses[i].pose.orientation.w = qw;
	}
}



int main(int argc, char** argv)
{

	ros::init(argc,argv,"global_rc_path_plan");
	ros::NodeHandle n;


	ros::Rate loop_rate(50);
	global_state=FLY_ALONG_MARKER;
	process_state=FLY_ALONG_MARKER;

	// ros::ServiceClient ik_puber = n.serviceClient<serial_am::ikSrv>("/serial_am/Link_pose");
	// ros::ServiceClient graper_puber = n.serviceClsxaient<serial_am::graper>("/serial_am/graper");
	//ros::Publisher ik_puber = n.advertise<serial_am::ikMsg>("/Camera_servo_avtion/msg",1);

	ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");

	// ros::ServiceServer target_rev = n.advertiseService("/target_tracker/target_world_pose", target_pose_callback);
	ros::Subscriber target_rev=n.subscribe("/Tba_current",1,target_pose_callback);

	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
	ros::Subscriber camera_pose_suber = n.subscribe("/camera_servo_pose",1,&CameraPos_CallBack);
	

	ros::Publisher Path_puber = n.advertise<nav_msgs::Path>("/flx_path_plan",1);//gererator path and pub
	//设置感兴趣的目标物的apriltag ID
	ros::ServiceClient m_track_target = n.serviceClient<camera_control::set_interest_ID>("/target_tracker/set_interest_id");

	ros::ServiceServer m_safe_state = n.advertiseService("/flx_safe_state", safe_state_callback);

	ros::Subscriber trajectory_follow_success_flag_suber=n.subscribe("/trajectory_follow_success_flag",1,trajectory_follow_success_flag_CallBack);///mavros/vision_pose/pose

	ros::Publisher Trajectory_task_puber = n.advertise<camera_control::do_trajectory_task>("/trajectory_task_ready",1);//gererator path and pub

	double time_control;// =ros::Time::now().toSec(); 
	valid_target_cnt=0;
	valid_marker_cnt=0;

	double rise_pose_x,rise_pose_y,rise_pose_z;
	sx=0;sy=0;sz=0;sw=1;

	trajectory_follow_success=false;

	bool along_marker_path=false;

	int last_state=-1;


	while(ros::ok())
	{
		if(valid_marker_cnt>1000)valid_marker_cnt=1000;
		if(valid_target_cnt>1000)valid_target_cnt=1000;
		if(valid_work_space>1000)valid_work_space=1000;
		switch(global_state)
		{
			case FLY_ALONG_MARKER:
			{ //标志位，判断是否还在这个循环中，这样写则进入这个状态的第一次会出现状态提示
				if(global_state!=last_state)
				{
					ROS_INFO("FLY_ALONG_MARKER.....");
					last_state=global_state;
				}
				if(along_marker_path==false)
				{
					nav_msgs::Path m_path;
					generate_path(m_path,marker_wx,marker_wy,marker_wz+0.87,0.18,true,0.0);// 实时在这里把偏航角的信息加进去 offset angle =0
					if(m_path.poses.size()>0)Path_puber.publish(m_path);//发布轨迹
					//距离目标物足够近了，转而看小目标物，不看大目标物。  
					if(m_path.poses.size()>0&&m_path.poses.size()<7)//8
					{
						along_marker_path=true;
						camera_control::set_interest_ID IDmsg;
						IDmsg.request.id=25;
						ROS_INFO("set_interest_ID 25.....");
						m_track_target.call(IDmsg);
						if (IDmsg.response.id_ret==25)
						{
							ROS_INFO("set_interest_ID 25 done.....");
						}
					}
				}
				if(valid_target_cnt>2)
				{
						global_state=FLY_ALONG_TARGET;
						process_state=FLY_ALONG_TARGET;
						time_control=ros::Time::now().toSec(); 
						sx=mav_qx;sy=mav_qy;sz=mav_qz;sw=mav_qw;
				}
				if(mav_z>0.6)//start signal for trajectory controller node.
				{
					camera_control::do_trajectory_task msg;
					msg.do_trajectory_task=1;
					Trajectory_task_puber.publish(msg);
				}
				break;	
			}
			case FLY_ALONG_TARGET:
			{
				if(global_state!=last_state)
				{
					ROS_INFO("FLY_ALONG_TARGET.....");
					last_state=global_state;
				}
				nav_msgs::Path m_path;
				generate_path(m_path,target_wx,target_wy,target_wz+0.395,0.29,true,-0.001);//39 25
				if(m_path.poses.size())Path_puber.publish(m_path);
				// am_controller::servoset_srv msg;
				// msg.request.cmd=7;//check valid space
				// msg.request.pos1=0;
				// msg.request.pos2=0;//pi/2;
				// msg.request.pos3=target_bx;
				// msg.request.pos4=target_bz;
				// msg.request.action_time=800;
				// servoseter.call(msg);
				// valid_work_space+=msg.response.is_done;
				// if(valid_work_space>2)
				// {
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
				if(global_state!=last_state)
				{
					ROS_INFO("HOLD_AND_GRASP.....");
					last_state=global_state;
				}
				if(trajectory_follow_success)//wait for grasping done.           signal from trajectory controller node
				{
					global_state=GRASP_RISE;
					process_state=GRASP_RISE;
					time_control=ros::Time::now().toSec(); 
					rise_pose_x=mav_x;
					rise_pose_y=mav_y;
					rise_pose_z=mav_z;
					sx=mav_qx;sy=mav_qy;sz=mav_qz;sw=mav_qw;
				}

				break;	
			}
			case GRASP_RISE:
			{
				if(global_state!=last_state)
				{
					ROS_INFO("GRASP_RISE.....");
					last_state=global_state;
				}
				nav_msgs::Path m_path;//抓取完成后，将目标物抬起5cm
				generate_path(m_path,rise_pose_x,rise_pose_y,rise_pose_z+0.05,0,false,0);

				if(m_path.poses.size())Path_puber.publish(m_path);
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
				if(global_state!=last_state)
				{
					ROS_INFO("GRASP_DONE.....");
					last_state=global_state;
				}
				am_controller::servoset_srv msg;
				msg.request.cmd=5;
				msg.request.pos1=-10;
				msg.request.pos2=170;//pi/2;
				msg.request.pos3=-70;
				msg.request.pos4=0;
				msg.request.action_time=3000;
				limited_grasp_call(servoseter,msg);
				if(ros::Time::now().toSec()-time_control>1.0)
				{
					global_state=BACK_HOME_AND_LANGING;
					process_state=BACK_HOME_AND_LANGING;
					time_control=ros::Time::now().toSec(); 
					sx=mav_qx;sy=mav_qy;sz=mav_qz;sw=mav_qw;
				}

				break;	 
			}
			case BACK_HOME_AND_LANGING:
			{
				if(global_state!=last_state)
				{
					ROS_INFO("BACK_HOME_AND_LANGING.....");
					last_state=global_state;
				}
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
					generate_path(m_path,0,0,mav_z,0,false,0);
					generate_path(m_path,0,0,-5,0,false,0);
					if(m_path.poses.size())Path_puber.publish(m_path);
					time_control=ros::Time::now().toSec(); 
				}
				break;	
			}
			case FORCE_LANGING:
			{	
				if(global_state!=last_state)
				{
					ROS_INFO("FORCE_LANGING.....");
					last_state=global_state;
				}
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
					generate_path(m_path,mav_x,mav_y,-5,0,false,0);
					if(m_path.poses.size())Path_puber.publish(m_path);
					time_control=ros::Time::now().toSec(); 
				}
				break;	
			}
		}



		ros::spinOnce();
		loop_rate.sleep();
	}


	
	return 0;
}
