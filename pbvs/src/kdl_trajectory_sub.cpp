#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <am_controller/Mat_Tba.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "Eigen/Eigen"
#include <cmath>


//#include <camera_control/mav_vel.h>
using namespace KDL;
using namespace std;

int global_state,process_state;
int valid_work_space;
double last_time,now;

enum state_condition{	FLY_ALONG_MARKER,
			FLY_ALONG_TARGET,
			HOLD_AND_GRASP,
			GRASP_RISE,
			GRASP_DONE,
			BACK_HOME_AND_LANGING,
			FORCE_LANGING
			};

Matrix4d Twb;
geometry_msgs::Pose pose;
geometry_msgs::PoseStamped poseStamped;
nav_msgs::Path path_msg;

class Mav
{
public:
	geometry_msgs::Pose Local_pose;
	geometry_msgs::Pose set_pose;
	geometry_msgs::Vector3 vel;
	double last_time;
	double distance;
	double current_yaw;
	double set_yaw;
	Eigen::Isometry3d Twb;
	Mav(){
		last_time = 0.0;
		current_yaw = 0.0;
		set_yaw = 0.0;
		distance = 0;
	}

	void get_vel(const geometry_msgs::PoseStampedConstPtr &msg){
		double	now = ros::Time::now().toSec();
		double dt = now-last_time;
		last_time=now;
		vel.x = (msg->pose.position.x-Local_pose.position.x)/dt;
		vel.y = (msg->pose.position.y-Local_pose.position.y)/dt;
		vel.z = (msg->pose.position.z-Local_pose.position.z)/dt;
			

	}
	double get_distance(const geometry_msgs::Pose mav,const geometry_msgs::Point target){
			double dist=0;
			distance = sqrt(pow(target.x-mav.position.x)+pow(target.y - mav.position.y));
			return dist;
	}
//	void trajectory_plan();
//	void fly_
};

class Target//初始化
{
public:

	int valid_target_cnt;
	geometry_msgs::Point position;

	geometry_msgs::Vector3 vel;
	double last_time;
	Target(){
		valid_target_cnt = 0;
		last_time = 0.0;
		
	}
	~Target();
	void get_vel(const geometry_msgs::PoseStampedConstPtr &msg){
		double	now = ros::Time::now().toSec();
		double dt = now-last_time;
		last_time=now;
		//if(dt>0.05){
		vel.x = (msg->pose.position.x-position.x)/dt;
		vel.y = (msg->pose.position.y-position.y)/dt;
		vel.z = (msg->pose.position.z-position.z)/dt;
	
	}

};

class Marker//初始化
{
public:
	int valid_marker_cnt;
	geometry_msgs::Point position;
	Marker(){
		valid_marker_cnt = 0;
	}

};

/*************
实例化 无人机，目标物，标记tag
*************/
Mav mav;
Target target;
Marker marker;


void CameraPos_CallBack(const camera_control::CameraPos::ConstPtr &msg){
	float yaw_err =msg->cx - 320;
		
	 mav.set_yaw = mav.current_yaw -yaw_err*0.004;
	// ROS_INFO("set_yaw :%f\t*****curret_yaw :%f",set_yaw*180/3.14,current_yaw*180/3.14);
	 
	 tf::quaternionTFToMsg(tf::createQuaternionFromYaw(set_yaw),direction_yaw);
	 
}
void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg){

	mav.Local_pose.position.x = msg->pose.position.x;
	mav.Local_pose.position.y = msg->pose.position.y;
	mav.Local_pose.position.z = msg->pose.position.z;
  	mav.Local_pose.orientation.x=msg->pose.orientation.x;
	mav.Local_pose.orientation.y=msg->pose.orientation.y;
	mav.Local_pose.orientation.z=msg->pose.orientation.z;
	mav.Local_pose.orientation.w=msg->pose.orientation.w;
	mav.get_vel(msg);
	mav.current_yaw = tf::getYaw(msg->pose.orientation);
	geometry_msgs::Pose m = msg->pose;
	tf::poseMsgToEigen(m,mav.Twb);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(mav_x,mav_y,mav_z ));
	tf::Quaternion tf_q(mav_qx,mav_qy,mav_qz,mav_qw);
	transform.setRotation(tf_q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/body"));

}
void target_pose_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg){

		Eigen::Matrix4d Tbar,Twa;
		Tbar(0,0)=Tba_msg->r11;Tbar(0,1)=Tba_msg->r12;Tbar(0,2)=Tba_msg->r13;Tbar(0,3)=Tba_msg->t1;
		Tbar(1,0)=Tba_msg->r21;Tbar(1,1)=Tba_msg->r22;Tbar(1,2)=Tba_msg->r23;Tbar(1,3)=Tba_msg->t2;
		Tbar(2,0)=Tba_msg->r31;Tbar(2,1)=Tba_msg->r32;Tbar(2,2)=Tba_msg->r33;Tbar(2,3)=Tba_msg->t3;
		Tbar(3,0)=0;Tbar(3,1)=0;Tbar(3,2)=0;Tbar(3,3)=1;
		Twa=mav.Twb*Tbar;
		if(Tba_msg->id==25)
		{
			target.valid_target_cnt++;
			target.position.x=Twa(0,3);target.position.y=Twa(1,3);target.position.z=Twa(2,3);
			//target_bx=Tbar(0,3);target_by=Tbar(1,3);target_bz=Tbar(2,3);
			mav.distance = mav.get_distance(mav.Local_pose,target.position);
		}
		else if(Tba_msg->id==35)
		{
			marker.valid_marker_cnt++;
			marker.position.x=Twa(0,3);marker.position.y=Twa(1,3);marker.position.z=Twa(2,3);
			mav.distance = mav.get_distance(mav.Local_pose,target.position);
		}

}

class Path
{
public:
	// you can get some meta-info on the path:
	void get_path_info(Path_RoundedComposite* path){
		for (int segmentnr=0;  segmentnr < path->GetNrOfSegments(); segmentnr++) {
		double starts,ends;
		Path::IdentifierType pathtype;
		if (segmentnr==0) {
			starts = 0.0;
		} else {
			starts = path->GetLengthToEndOfSegment(segmentnr-1);
		}
		ends = path->GetLengthToEndOfSegment(segmentnr);
		pathtype = path->GetSegment(segmentnr)->getIdentifier();
		std::cout << "segment " << segmentnr << " runs from s="<<starts << " to s=" <<ends;
		switch(pathtype) {
			case Path::ID_CIRCLE:
				std::cout << " circle";
				break;
			case Path::ID_LINE:
				std::cout << " line ";
				break;
			default:
				std::cout << " unknown ";
				break;
		}
		std::cout << std::endl;
	}
	}
	void generate_path2target(geometry_msgs::Pose &mav_pose,geometry_msgs::Point &target_pose,double yaw,double z_compensate,nav_msgs::Path &msg_path){

		//0.2 0.01 是啥?
		try{
			Path_RoundedComposite* path = new Path_RoundedComposite(0.2,0.01,new RotationalInterpolation_SingleAxis());
			//double radius,double eqradius,RotationalInterpolation* orient, bool aggregate=true
			/*@param radius : 圆弧的半径
			* @param eqradius :旋转和速度的过度 equivalent radius to compare rotations/velocities
			* @param orient   : method of rotational_interpolation interpolation 角轴插补方法*/
			
			geometry_msgs::Vector3 target_back;
			double x_dir,y_dir;
			double delta = 0.4;
			x_dir = target.position.x -  mav.Local_pose.position.x;
			y_dir = target.position.y -  mav.Local_pose.position.y;
			double D_x = x_dir/(sqrt(x_dir*x_dir+y_dir*y_dir));
			double D_y = y_dir/(sqrt(x_dir*x_dir+y_dir*y_dir));
			path->Add(Frame(Rotation::RPY(yaw,0,0), Vector(mav_pose.position.x,mav_pose.position.y,mav_pose.position.z)));
			path->Add(Frame(Rotation::RPY(yaw,0,0), Vector(target_pose.position.x,target_pose.position.y,target_pose.position.z+z_compensate)));
			path->Add(Frame(Rotation::RPY(yaw,0,0,Vector(target_pose.position.x+D_x*delta,target_pose.position.y+D_y*delta,target_pose.position.z+z_compensate))));


			path->Finish();

			//这几段表示将从起始到终点的速度设定为梯形波，最大速度为0.5，加速度为0.1。
			VelocityProfile* velpref = new VelocityProfile_Trap(0.5,0.1);
			velpref->SetProfile(0,path->PathLength());  
			//给路径加上速度，加速度信息
			Trajectory* traject = new Trajectory_Segment(path, velpref);
			// use the trajectory

			geometry_msgs::Pose poses;
			geometry_msgs::PoseStamped poseStampeds;
			double dt=0.1;
			msg_path.poses.clear();
			std::ofstream of("./trajectory.dat");
			for (double t=0.0; t <= traject->Duration(); t+= dt) {
				Frame current_pose;
				current_pose = traject->Pos(t);
				tf::poseKDLToMsg(current_pose,poses);
				path_msg.header.frame_id = "world";
				poseStamped.header.frame_id = "world";
				poseStamped.pose  = poses;
				path_msg.poses.push_back(poseStampeds);
				path_pub.publish(msg_path);

				for (int i=0;i<4;++i)
					for (int j=0;j<4;++j)
						of << current_pose(i,j) << "\t";
				of << "\n";
				// also velocities and accelerations are available !
				//traject->Vel(t);
				//traject->Acc(t);
			}
			of.close();
		} catch(Error& error) {
			std::cout <<"I encountered this error : " << error.Description() << std::endl;
			std::cout << "with the following type " << error.GetType() << std::endl;
		}

	}


};

Path path;
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "kdl_trajectory");
	ros::NodeHandle n("~");	

	/**************订阅**************/
	ros::Subscriber camera_pose_suber = n.subscribe("/camera_servo_pose",1,&CameraPos_CallBack);
	ros::Subscriber trajectory_follow_success_flag_suber=n.subscribe("/trajectory_follow_success_flag",1,trajectory_follow_success_flag_CallBack);///mavros/vision_pose/pose
	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
	ros::Subscriber target_rev=n.subscribe("/Tba_current",1,target_pose_callback);

	/***************客户端*************/
	//设置感兴趣的目标物的apriltag ID
	ros::ServiceClient m_track_target = n.serviceClient<camera_control::set_interest_ID>("/target_tracker/set_interest_id");
	ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");

	/***************服务端*************/
	ros::ServiceServer m_safe_state = n.advertiseService("/flx_safe_state", safe_state_callback);

	/***************发布**************/
	ros::Publisher Path_puber = n.advertise<nav_msgs::Path>("/flx_path_plan",1);//gererator path and pub
	ros::Publisher Trajectory_task_puber = n.advertise<camera_control::do_trajectory_task>("/trajectory_task_ready",1);//gererator path and pub
	ros::Publisher path_pub =n.advertise<nav_msgs::Path>("/path_planned",1000);

	ros::Rate loop_rate(50);
	global_state=FLY_ALONG_MARKER;
	process_state=FLY_ALONG_MARKER;

	while(ros::ok()){

		if(marker.valid_marker_cnt>1000)marker.valid_marker_cnt=1000;
		if(target.valid_target_cnt>1000)target.valid_target_cnt=1000;
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

					//这里还没有爬升0.87米
					path.generate_path2target(mav.Local_pose,marker.position,mav.set_yaw,0.87,m_path);
					//TO DO publish tf 
					//generate_path(m_path,marker_wx,marker_wy,marker_wz+0.87,0.18,true,0.0);// 实时在这里把偏航角的信息加进去 offset angle =0
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

				}

				// To Do
				if(mav_z>0.6)//start signal for trajectory controller node.
				{
					camera_control::do_trajectory_task msg;
					//开始执行任务
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

				path.generate_path2target(mav.Local_pose,target.position,mav.set_yaw,0.395,m_path);

				if(m_path.poses.size())Path_puber.publish(m_path);
		
		
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
				am_controller::servoset_srv msg;ruabn
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


	}	



	return 0;
}