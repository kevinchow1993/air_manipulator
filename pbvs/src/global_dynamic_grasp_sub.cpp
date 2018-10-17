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
#include <visualization_msgs/Marker.h>
#include "am_controller/servoset_srv.h"
#include <geometry_msgs/PoseStamped.h>
#include "serial.h"
#include <camera_control/ServoPos.h>
#include <camera_control/CameraPos.h>
#include <camera_control/TagLost.h>
#include <camera_control/rc_state.h>
#include <am_controller/success_flag.h>
#include <camera_control/do_trajectory_task.h>
#include <camera_control/set_interest_ID.h>

#include <queue>

#include "Eigen/Eigen"
#include <cmath>
using namespace KDL;
using namespace std;


 #define pi  3.1415926535898
int global_state,process_state;
enum state_condition{	
			FLY_ALONG_MARKER,
			FLY_ALONG_TARGET,
			HOLD_AND_GRASP,
			GRASP_RISE,
			GRASP_DONE,
			BACK_HOME_AND_LANGING,
			FORCE_LANGING
			};

camera_control::TagLost TagLost_msg;
am_controller::servoset_srv servoset_srv_msg;
//nav_msgs::Path path_msg,grasp_msg_path;
//nav_msgs::Path grasp_path_msg;
//ros::Publisher grasp_path_pub;
ros::Publisher path_pub;
ros::Publisher marker_pub;
ros::Publisher big_marker_pub;
ros::Publisher endeft_pub;
ros::Publisher body_pub;
ros::ServiceClient servoseter;
double first,second;
bool trajectory_follow_success  = false;
Eigen::Isometry3d T_eft_lasttime;
Eigen::Vector3d temp_target;
//tf::TransformBroadcaster br;

Eigen::Vector2d set_point(0.22,-0.17);
//double i=0.1;
//geometry_msgs::PoseStamped poseStamped,grasp_poseStamped;


Eigen::Isometry3d Tmb;

class Mav
{
public:
	Eigen::Isometry3d T_mav_last_time;
	geometry_msgs::Pose Local_pose;
	geometry_msgs::Pose set_pose;
	geometry_msgs::Vector3 vel;
	double last_time;
	double distance;
	double current_yaw;
	double set_yaw;
	geometry_msgs::Quaternion direction_yaw;
	geometry_msgs::Point landing_posi; //降落点
	geometry_msgs::Quaternion s;
	geometry_msgs::Point rise_position;// 飞机起始点
	Eigen::Isometry3d Twb;
	visualization_msgs::Marker body;
	Eigen::Vector3d delta_trans;
	Eigen::Vector3d start_point;


	
	Mav(){
		
		delta_trans.setZero();
		T_mav_last_time.setIdentity();
		last_time = 0.0;

		rise_position.x = 0.0;
		rise_position.y = 0.0;
		rise_position.z = 0.0;
		current_yaw = 0.0;
		set_yaw = 0.0;
		distance = 0;
		body.header.frame_id="/body";
		//body.header.stamp = ros::Time::now();
		body.id = 0;
		body.ns = "basic_shapes";
		body.type = visualization_msgs::Marker::SPHERE;
		body.pose.position.x = 0;
		body.pose.position.y = 0;
		body.pose.position.z = 0;
		body.pose.orientation.x= 0;
		body.pose.orientation.y = 0;
		body.pose.orientation.z = 0;
		body.pose.orientation.w = 1;
		body.scale.x = 0.05;
		body.scale.y=0.05;
		body.scale.z = 0.05;
		body.color.r = 0.0f;
		body.color.g = 1.0f;
		body.color.b = 0.0f;
		body.color.a = 1.0;
		body.lifetime = ros::Duration();
		start_point<<0,0,0;
		ROS_ERROR("****mav object has created!!***");
		
		

	}
	/**
	 * @brief Get the err object
	 * 
	 * @param mx 
	 * @param my 
	 * @param mz 
	 * @param des_x 
	 * @param des_y 
	 * @param des_z 
	 * @return double 
	 */
	double get_err(double mx,double my,double mz,double des_x,double des_y,double des_z)
	{
		double pose_err_x=fabs(mx-des_x);
		double pose_err_y=fabs(my-des_y);
		double pose_err_z=fabs(mz-des_z);
		double move_err=sqrt(pose_err_x*pose_err_x+pose_err_y*pose_err_y+pose_err_z*pose_err_z);
		return 	move_err;
	}
	/**
	 * @brief Get the vel object 
	 * 
	 * @param msg 
	 */
	void get_vel(const geometry_msgs::PoseStampedConstPtr &msg){
		double	now = ros::Time::now().toSec();
		double dt = now-last_time;
		last_time=now;
		vel.x = (msg->pose.position.x-Local_pose.position.x)/dt;
		vel.y = (msg->pose.position.y-Local_pose.position.y)/dt;
		vel.z = (msg->pose.position.z-Local_pose.position.z)/dt;
			

	}

	/**
	 * @brief Get the distance object
	 * 
	 * @param mav 
	 * @param target 
	 * @return double 
	 */
	double get_distance(const geometry_msgs::Pose mav,const geometry_msgs::Point target){
			double dist=0;
			distance = sqrt(pow(target.x-mav.position.x,2)+pow(target.y - mav.position.y,2));
			return dist;
	}

	/**
	 * @brief Set the Twb object
	 * 
	 * @param msg 
	 */ 
	void set_Twb(const geometry_msgs::PoseStampedConstPtr &msg){
		
		
		Local_pose.position.x = msg->pose.position.x;
		Local_pose.position.y = msg->pose.position.y;
		Local_pose.position.z = msg->pose.position.z;
		Local_pose.orientation.x=msg->pose.orientation.x;
		Local_pose.orientation.y=msg->pose.orientation.y;
		Local_pose.orientation.z=msg->pose.orientation.z;
		Local_pose.orientation.w=msg->pose.orientation.w;
		get_vel(msg);
		current_yaw = tf::getYaw(msg->pose.orientation);
		geometry_msgs::Pose m = msg->pose;
		tf::poseMsgToEigen(m,Twb);
		body.header.stamp = ros::Time::now();
		body.pose.position.x = Local_pose.position.x;
		body.pose.position.y = Local_pose.position.y;
		body.pose.position.z = Local_pose.position.z;
		body.pose.orientation.x= Local_pose.orientation.x;
		body.pose.orientation.y = Local_pose.orientation.y;
		body.pose.orientation.z = Local_pose.orientation.z;
		body.pose.orientation.w = Local_pose.orientation.w;

	}
	/**
	 * @brief 发布坐标变换
	 * 
	 */
	void sendTransformTwb(){
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(Local_pose.position.x,Local_pose.position.y,Local_pose.position.z ));
		tf::Quaternion tf_q(Local_pose.orientation.x,Local_pose.orientation.y,Local_pose.orientation.z,Local_pose.orientation.w);
		transform.setRotation(tf_q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/body"));
	}

	/**
	 * @brief  发布rviz
	 * 
	 * @param mav_marker_pub 
	 */
	void pub_mavmarker(ros::Publisher &mav_marker_pub)
	{
		mav_marker_pub.publish(body);
	}
	/**
	 * @brief for debug
	 * 
	 */
	void display(){
		cout<<"-----mav info------"<<endl;
		cout.precision(3);
		cout<<"--mav position--";
		cout<<"x:"<<Local_pose.position.x<<"y:"<<Local_pose.position.y<<"z:"<<Local_pose.position.z<<endl;
		cout<<"--yaw--"<<current_yaw*180.0/pi<< endl;
		cout<<"--Twb--"<<endl;;
		cout<<Twb.matrix()<<endl;
		cout<<"--delta_trans--"<<endl;
		cout<<delta_trans<<endl;


	}
	/**
	 * @brief Set the stratPoint object设置起始点
	 * 
	 */
	void set_stratPoint(){
		start_point<<Local_pose.position.x,Local_pose.position.y,Local_pose.position.z;
	}


	
	/**
	 * @brief  计算相对位移
	 * 
	 * @return Eigen::Vector2d 
	 */
	//TODO: 低通滤波
	Eigen::Vector2d compute_translation(){

		Eigen::Vector2d res(0,0);
		if(start_point(0)){
			Eigen::Vector3d now_positin(Local_pose.position.x,Local_pose.position.y,Local_pose.position.z);
			delta_trans = now_positin - start_point;
			//res(0)= 0.0;
			res(0) = delta_trans(0)/cos(current_yaw);
			//res(0) = sqrt(delta_trans(0)*delta_trans(0)+delta_trans(1)*delta_trans(1));
			res(1) = delta_trans(2);
			cout<<"start point\t"<<start_point(0)<<" "<<start_point(1)<<" "<<start_point(2)<<endl;
			cout<<"res:\n"<<res(0)<<res(1)<<endl;
			//cout<<"current yaw\n"<<cos(current_yaw)<<endl;
			//	res(2) = delta_trans(2); 
		}
		return res;
	}

	void landing(nav_msgs::Path &m_path){

		m_path.header.stamp = ros::Time::now();
		m_path.header.frame_id = "/world";
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "fcu";
		double step = 0.07;
		double hight = Local_pose.position.z +0.5;

		pose.pose.orientation = s;
		int size =hight/step;

		
		for(size_t i = 1; i < size; i++)
		{
			pose.pose.position.x = landing_posi.x;
			pose.pose.position.y = landing_posi.y;
			pose.pose.position.z = Local_pose.position.z- step*i;
			m_path.poses.push_back(pose);

		}
		

		
		



	}
	/**
	 * @brief 生成无人机轨迹
	 * 
	 * @param m_path 
	 * @param des_x 
	 * @param des_y 
	 * @param des_z 
	 * @param prevent_xy_gap 
	 * @param use_path_dir 
	 * @param offset_angle 
	 */
	//TODO: 没有飞超过目标物
	void  generate_path(nav_msgs::Path &m_path,double des_x,double des_y,double des_z,double des_step,bool use_path_dir, bool flytoback)
	{
		if(des_z>0.85)des_z=0.85;//
		m_path.header.stamp = ros::Time::now();
		m_path.header.frame_id = "/world";
		geometry_msgs::Quaternion q;
		double targetx = 0,targety = 0;
		if(flytoback){
			//飞到目标物后方 0.2 米处
			targetx = des_x+cos(current_yaw)*0.2;
			targety = des_y+sin(current_yaw)*0.2;

		}else{
			targetx = des_x;
			targety = des_y;


		}
		if(use_path_dir)//机头朝向目标物飞过去
		{	
			q.x = direction_yaw.x;
			q.y = direction_yaw.y;
			q.z = direction_yaw.z;
			q.w = direction_yaw.w;
			//get_direction_by_line(mav_x,mav_y,des_x,des_y,qw,qx,qy,qz,offset_angle);
			}
		else 
		{
			q.x=s.x;
			q.y=s.y;
			q.z=s.z;
			q.w=s.w;
		}
		//Fast take off
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "fcu";

		pose.pose.orientation = q;

		//pose.pose.orientation = direction_yaw;	//ADD BY KEVIN

		pose.pose.position.x =Local_pose.position.x;
		pose.pose.position.y = Local_pose.position.y;
		pose.pose.position.z = des_z;
		//if(get_err(pose.pose.position.x,pose.pose.position.y,0,des_x,des_y,0)>=prevent_xy_gap)m_path.poses.push_back(pose);
		double des_length;
		//des_step=0.02;

		des_length=get_err(Local_pose.position.x,Local_pose.position.y,des_z,des_x,des_y,des_z);																								
		double delt_x=des_step/des_length*(targetx-Local_pose.position.x);
		double delt_y=des_step/des_length*(targety-Local_pose.position.y);
		//get_direct_by_line();
		//首先生成路上的轨迹
		//TODO:: 
		for (int i = 1; i < des_length/des_step; ++i)
		{
			pose.pose.position.x =Local_pose.position.x+i*delt_x;
			pose.pose.position.y =Local_pose.position.y+i*delt_y;
			pose.pose.position.z = des_z;
			if(get_err(pose.pose.position.x,pose.pose.position.y,0,targetx,targety,0)>=0)m_path.poses.push_back(pose);
		}
		// pose.pose.position.x =des_x;
		// pose.pose.position.y =des_y;
		// pose.pose.position.z = des_z;
		// //如果距离大于限定值，则将点压入轨迹
		// if(get_err(pose.pose.position.x,pose.pose.position.y,0,des_x,des_y,0)>=prevent_xy_gap)m_path.poses.push_back(pose);

	}

	
};

/**
 * @brief 目标物类
 * 
 */
class Target//初始化
{
public:
	
	
	geometry_msgs::Point Posi_m;		//目标物在机械臂基座下坐标
	geometry_msgs::Point Posi_w;		//目标物世界坐标	
	geometry_msgs::Point marker_pos;

	visualization_msgs::Marker visual;  //目标物rviz
	visualization_msgs::Marker bigmarker;
	geometry_msgs::Vector3 vel;			//目标物移动速度
	double last_time;
	bool Delta_T_is_ok;
	bool path_is_ok=0; 			
	bool grasp_flag = 0;
	bool grasp_is_done =0; 				//判断有没有生成轨迹
	int valid_marker_cnt = 0;			//有效marker计数
	int valid_target_cnt = 0;			//有效target计数
	Eigen::Isometry3d Twa;				//世界坐标系到目标物的变换矩阵	
	Eigen::Isometry3d Tba;				//机体坐标系到目标物的变换矩阵	
	queue<Eigen::Vector2d> q_compansate; //动态补偿队列
	//Eigen::Quaterniond tar_q;
	nav_msgs::Path grasp_traj;			//机械臂轨迹for visual
	queue<Eigen::Vector2d> grasp_path;	//机械臂轨迹for track
	

	Target(){
		valid_target_cnt = 0;
		last_time = 0.0;
		Delta_T_is_ok = 0;
		visual.header.frame_id="/world";
		//target marker
		visual.id = 0;
		visual.ns = "basic_shapes";
		visual.type = visualization_msgs::Marker::SPHERE;
		visual.pose.position.x = 0;
		visual.pose.position.y = 0;
		visual.pose.position.z = 0;
		visual.pose.orientation.x= 0;
		visual.pose.orientation.y = 0;
		visual.pose.orientation.z = 0;
		visual.pose.orientation.w = 1;
		visual.scale.x = 0.1;
		visual.scale.y=0.1;
		visual.scale.z = 0.1;
		visual.color.r = 0.0f;
		visual.color.g = 1.0f;
		visual.color.b = 0.0f;
		visual.color.a = 1.0;
		visual.lifetime = ros::Duration();
		//
		bigmarker.header.frame_id="/world";
		bigmarker.id = 0;
		bigmarker.ns = "basic_shapes";
		bigmarker.type = visualization_msgs::Marker::CUBE;
		bigmarker.pose.position.x = 0;
		bigmarker.pose.position.y = 0;
		bigmarker.pose.position.z = 0;
		bigmarker.pose.orientation.x= 0;
		bigmarker.pose.orientation.y = 0;
		bigmarker.pose.orientation.z = 0;
		bigmarker.pose.orientation.w = 1;
		bigmarker.scale.x = 0.2;
		bigmarker.scale.y=0.2;
		bigmarker.scale.z = 0.2;
		bigmarker.color.r = 1.0f;
		bigmarker.color.g = 0.0f;
		bigmarker.color.b = 0.0f;
		bigmarker.color.a = 1.0;
		bigmarker.lifetime = ros::Duration();
		ROS_ERROR("****target object has created!!***");

		
	}
	~Target(){};
	/**
	 * @brief Get the vel object
	 * 
	 * @param msg 
	 */
	void get_vel(const geometry_msgs::PoseStampedConstPtr &msg){
		double	now = ros::Time::now().toSec();
		double dt = now-last_time;
		last_time=now;
		//if(dt>0.05){
		vel.x = (msg->pose.position.x-Posi_m.x)/dt;
		vel.y = (msg->pose.position.y-Posi_m.y)/dt;
		vel.z = (msg->pose.position.z-Posi_m.z)/dt;
	
	}
	/**
	 * @brief Set the target Twa object
	 * 
	 * @param Tba 
	 * @param Twb 
	 */
	void set_Twa(Eigen::Isometry3d &Tba,Eigen::Isometry3d &Twb){
		
		Eigen::Isometry3d Tma = Tmb*Tba;
		
		
		this->Twa = Twb*Tba;
		this->Tba = Tba;
		Posi_w.x = this->Twa(0,3);
		Posi_w.y = this->Twa(1,3);
		Posi_w.z = this->Twa(2,3);
		Posi_m.x = Tma(0,3);
		Posi_m.y = Tma(1,3);
		Posi_m.z = Tma(2,3);
		visual.header.stamp = ros::Time::now();
		visual.pose.position.x = Posi_w.x;
		visual.pose.position.y = Posi_w.y;
		visual.pose.position.z = Posi_w.z;
	
	}

	/**
	 * @brief Set the marker pos object
	 * 
	 * @param Tba 
	 * @param Twb 
	 */
	void set_marker_pos(Eigen::Isometry3d &Tba,Eigen::Isometry3d &Twb){
		Eigen::Isometry3d twa = Twb*Tba;
		marker_pos.x = twa(0,3);
		marker_pos.y = twa(1,3);
		marker_pos.z = twa(2,3);
		bigmarker.header.stamp = ros::Time::now();
		bigmarker.pose.position.x = marker_pos.x;
		bigmarker.pose.position.y = marker_pos.y;
		bigmarker.pose.position.z = marker_pos.z;
		//cout<<"big marker:"<<marker_pos.x<<" "<<marker_pos.y<<" "<<marker_pos.z<<endl;


	}


	/**
	 * @brief 发布rviz
	 * 
	 * @param marker_pub 
	 */
	void pub_marker(ros::Publisher &marker_pub,visualization_msgs::Marker visual){
		
		marker_pub.publish(visual);
		

	}
	/**
	 * @brief for debug
	 * 
	 */
	void display(){
		cout<<"-----target info------"<<endl;
		cout.precision(3);
		cout<<"--World Frame--"<<endl;
		cout<<"x:"<<Posi_w.x<<"y:"<<Posi_w.y<<"z:"<<Posi_w.z<<endl;
		cout<<"----"<<endl;
		cout<<"--Body Frame--"<<endl;
		cout<<"x:"<<Posi_m.x<<"y:"<<Posi_m.y<<"z:"<<Posi_m.z<<endl;


	}

	/**
	 * @brief 发布 世界坐标系到 目标物的坐标变换
	 * 
	 */
	void sendTransformTarget(){
		static tf::TransformBroadcaster Tar_br;
		tf::Transform transform;
		tf::transformEigenToTF(Twa,transform);

		
		Tar_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/target"));
		
		
	}
	//void target::target_pose_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg);
	/**
	 * @brief 生成抓取轨迹
	 * 
	 * @return true 
	 * @return false 
	 */
		bool generate_grasp_trajectory(){	

			double target_x  = Posi_m.x;
			double target_y  = Posi_m.y;

			ROS_INFO("target_x,y=%f  %f/n",target_x,target_y);
		try{
			
			double start,duration=0;
			start = ros::Time::now().toSec();
			Path_RoundedComposite* path = new Path_RoundedComposite(0.1,0.01,new RotationalInterpolation_SingleAxis());
			path->Add(Frame(Rotation::RPY(0,0,0), Vector(0.20,-0.02,0)));//收起位置
			//path->Add(Frame(Rotation::RPY(0,0,0), Vector(0.20,-0.02,0)));//收起位置
		//	path->Add(Frame(Rotation::RPY(0,0,0), Vector(target_x,target_y+0.05,0)));
			path->Add(Frame(Rotation::RPY(0,0,0), Vector(target_x-0.12,target_y-0.03,0)));

			path->Add(Frame(Rotation::RPY(0,0,0), Vector(target_x,target_y,0)));
			path->Finish();
			VelocityProfile* velpref = new VelocityProfile_Trap(0.5,0.1);
			velpref->SetProfile(0,path->PathLength());  
			//给路径加上速度，加速度信息
			Trajectory* traject = new Trajectory_Segment(path, velpref);


			geometry_msgs::Pose poses;
			geometry_msgs::PoseStamped poseStampeds;
			double dt=0.08;
			grasp_traj.poses.clear();

			for (double t=0.0; t <= traject->Duration(); t+= dt) {
				Frame current_pose;
				current_pose = traject->Pos(t);
				tf::poseKDLToMsg(current_pose,poses);
				grasp_traj.header.frame_id = "body";
				poseStampeds.header.frame_id = "body";
				poseStampeds.pose  = poses;
				Eigen::Vector2d temp_point;
				temp_point<<poses.position.x,poses.position.y;
				grasp_traj.poses.push_back(poseStampeds);
				grasp_path.push(temp_point);
			}
			duration= ros::Time::now().toSec()-start;
			ROS_INFO("grasp_trij size is:%d---use%f sec",grasp_traj.poses.size(),duration);
			path_is_ok = true;
			return true;
			
		}catch(Error& error) {
			std::cout <<"I encountered this error : " << error.Description() << std::endl;
			std::cout << "with the following type " << error.GetType() << std::endl;
		
			return false;
			
		}

	
		}
	/**
	 * @brief 闭合抓子
	 * 
	 * @param servoseter 
	 */
	void grasp(ros::ServiceClient &servoseter){

			am_controller::servoset_srv msg;
			msg.request.cmd=4;
			msg.request.pos1=10;
			msg.request.action_time=100;
			servoseter.call(msg);
	}

	/**
	 * @brief 张开爪子
	 * 
	 * @param servoseter 
	 */
	void rise(ros::ServiceClient &servoseter){

		am_controller::servoset_srv msg;
		msg.request.cmd=6;
		msg.request.pos1=0;
		msg.request.pos2=0;//pi/2;
		msg.request.pos3=0.20;
		msg.request.pos4=-0.01;
		msg.request.action_time=1000;
		servoseter.call(msg);
		

	}

};
	Mav mav;
	Target target;


void init_Tmb(void)
{
	//Tmb init
	//Tmb=Eigen::MatrixXd::Zero(4,4);Tmb(3,3)=1.0;
	Tmb(0,0)=1.0;		Tmb(0,1)=0.0;		Tmb(0,2)=0.0;		Tmb(0,3)=0.0; //Tmb(0,3)=0.04;	//从manipulator到body
	Tmb(1,0)=0.0;		Tmb(1,1)=0.0;		Tmb(1,2)=1.0;		Tmb(1,3)=0.24;
	Tmb(2,0)=0.0;		Tmb(2,1)=-1.0;		Tmb(2,2)=0.0;		Tmb(2,3)=0.0;
	Tmb(3,0)=0.0;		Tmb(3,1)=0.0;		Tmb(3,2)=0.0;		Tmb(3,3)=1.0;
}


// 获得delat_t矩阵 
void target_pose_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg)

{
	//cout<<"enter target!"<<endl;
	/*Eigen::Isometry3d Delta_T;
	geometry_msgs::Pose mav_pose=mav.Local_pose;
	Eigen::Isometry3d T_mav_thistime;
	tf::poseMsgToEigen(mav_pose,T_mav_thistime);
	if(!target.Delta_T_is_ok){
		Delta_T.setIdentity();
		mav.T_mav_last_time = T_mav_thistime;
		target.Delta_T_is_ok = 1;
	}else{
		Delta_T = T_mav_thistime*mav.T_mav_last_time.inverse();
		mav.T_mav_last_time = T_mav_thistime;}*/
	
	static int target_pose_flag;
	if(target_pose_flag==0){
		ROS_INFO("Enter target_pose callback!");
		target_pose_flag = 1;
	}

	

	Eigen::Isometry3d Tbar,Tma,Twa;
	//visualization_msgs::Marker marker;
	Tbar(0,0)=Tba_msg->r11;Tbar(0,1)=Tba_msg->r12;Tbar(0,2)=Tba_msg->r13;Tbar(0,3)=Tba_msg->t1;
	Tbar(1,0)=Tba_msg->r21;Tbar(1,1)=Tba_msg->r22;Tbar(1,2)=Tba_msg->r23;Tbar(1,3)=Tba_msg->t2;
	Tbar(2,0)=Tba_msg->r31;Tbar(2,1)=Tba_msg->r32;Tbar(2,2)=Tba_msg->r33;Tbar(2,3)=Tba_msg->t3;
	Tbar(3,0)=0;Tbar(3,1)=0;Tbar(3,2)=0;Tbar(3,3)=1;
	Tma = Tmb*Tbar;
	if(Tba_msg->id==25){
		target.valid_target_cnt++;
		target.set_Twa(Tbar,mav.Twb);

		target.pub_marker(marker_pub,target.visual);
		target.sendTransformTarget();
	}
	else if(Tba_msg->id==35){
		target.valid_marker_cnt++;
		target.set_marker_pos(Tbar,mav.Twb);
		target.pub_marker(big_marker_pub,target.bigmarker);

	}


	


}

/**
 * @brief apriltag 的回调函数 计算当前 设置的yaw角值 
 * 
 * @param msg 
 */
void CameraPos_CallBack(const camera_control::CameraPos::ConstPtr &msg){
	static int camera_pose_flag;
	if(camera_pose_flag ==0){
		ROS_INFO("Enter camera pose flag");
		camera_pose_flag = 1;
	}
	float yaw_err =msg->cx - 320;
		
	 mav.set_yaw = mav.current_yaw -yaw_err*0.003;//0.004
	// ROS_INFO("set_yaw :%f\t*****curret_yaw :%f",set_yaw*180/3.14,current_yaw*180/3.14);
	 
	 tf::quaternionTFToMsg(tf::createQuaternionFromYaw(mav.set_yaw),mav.direction_yaw);
	 
}

/**

得到无人机的位姿,速度,yaw角,发布坐标变换,


**/
void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg){
	static int local_pose_flag;
	
	//cout<<"enter mav"<<endl;

	mav.set_Twb(msg);
	mav.sendTransformTwb();
	mav.pub_mavmarker(body_pub);
	if(local_pose_flag ==0){
		ROS_INFO("Enter local pose callback");
		local_pose_flag = 1;
		mav.rise_position = mav.Local_pose.position;
		ROS_INFO("START POINT IS:X:%2f  Y:%2f  Z:%2f",mav.rise_position.x,mav.rise_position.y,mav.rise_position.z);
	}
	//mav.display();



}

void call_servo(Eigen::Vector2d &setpoint,Eigen::Vector2d delta){

	
	Eigen::Vector2d actual_point;
	actual_point(0) = setpoint(0)  - delta(0);
	actual_point(1) = setpoint(1) -  delta(1);

	servoset_srv_msg.request.cmd=6;
	servoset_srv_msg.request.pos1=0;
	servoset_srv_msg.request.pos2=0;
	servoset_srv_msg.request.pos3=actual_point(0);
	servoset_srv_msg.request.pos4=actual_point(1);
	servoset_srv_msg.request.action_time=200;
	//cout<<"delta_trans\n"<<mav.delta_trans<<endl;
	cout<<"set_point:\t"<<setpoint(0)<<setpoint(1)<<endl;
	cout<<"delta\t"<<delta(0)<<delta(1)<<endl;

}

void timerCallback(const ros::TimerEvent&){
	static int time_flag;
	if(time_flag==0){
		ROS_INFO("Enter timer!");
		time_flag = 1;
	}

	if(target.grasp_flag == true&&!target.grasp_is_done){
		cout<<"enter timer"<<"---"<<"path is OK  "<<target.path_is_ok<<endl;

		// target.q_compansate.push(mav.compute_translation());
		// call_servo(set_point,target.q_compansate.front());
		// target.q_compansate.pop();
		// servoseter.call(servoset_srv_msg);

		if(target.path_is_ok){
			if(target.grasp_path.size()>5){
				cout<<"grasp_path size is   "<<target.grasp_path.size()<<endl;

				target.q_compansate.push(mav.compute_translation());
				call_servo(target.grasp_path.front(),target.q_compansate.front());
				target.q_compansate.pop();
				target.grasp_path.pop();
				servoseter.call(servoset_srv_msg);}
			else if(!target.grasp_is_done){//判断抓取完成
				target.grasp(servoseter);
				ros::Duration(0.3).sleep();
				target.rise(servoseter);
				target.grasp_is_done = true;
			}

		}
		else{
			target.generate_grasp_trajectory();
			mav.set_stratPoint();
			
		}
	}
}

/**
 * @brief 遥控器控制
 * 
 * @param msg 
 * @param res 
 * @return true 
 * @return false 
 */
bool safe_state_callback(camera_control::rc_state::Request &msg,camera_control::rc_state::Response &res)
{
	ROS_INFO("*****RC control!*****");
	if(msg.state==1)global_state=process_state;
	else if(msg.state==2)global_state=BACK_HOME_AND_LANGING;
	else if(msg.state==3)global_state=FORCE_LANGING;
	res.is_done=1;
	return true;
}

// void trajectory_follow_success_flag_CallBack(const am_controller::success_flagConstPtr &msg)
// {
// 	if(msg->success_flag==1)
// 	{
// 		trajectory_follow_success=true;
// 	}
// }

void init_grasp(ros::ServiceClient &servoseter){

	am_controller::servoset_srv msg;
	msg.request.cmd=5;
	msg.request.pos1=-10;
	msg.request.pos2=170;//pi/2;
	msg.request.pos3=-70;
	msg.request.pos4=0;
	msg.request.action_time=2000;
	servoseter.call(msg);


	msg.request.cmd=4;
	msg.request.pos1=60;

	msg.request.action_time=1000;
	servoseter.call(msg);

	

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
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dynamic_grasp");

	init_Tmb();
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	global_state=FLY_ALONG_MARKER;
	process_state=FLY_ALONG_MARKER;
	
	// 订阅 
	ros::Subscriber camera_pose_suber = n.subscribe("/camera_servo_pose",1,&CameraPos_CallBack);
	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
	ros::Subscriber target_rev=n.subscribe("/Tba_current",1,target_pose_callback);
	//ros::Subscriber trajectory_follow_success_flag_suber=n.subscribe("/trajectory_follow_success_flag",1,trajectory_follow_success_flag_CallBack);///mavros/vision_pose/pose
    //发布
	ros::Publisher Path_puber = n.advertise<nav_msgs::Path>("/flx_path_plan",1);//gererator path and pub
	marker_pub = n.advertise<visualization_msgs::Marker>("/target_marker",1);
	big_marker_pub = n.advertise<visualization_msgs::Marker>("/big_marker",1);
	body_pub = n.advertise<visualization_msgs::Marker>("/body_marker",1);
	endeft_pub = n.advertise<visualization_msgs::Marker>("/endeffector",1);
	ros::Publisher Trajectory_task_puber = n.advertise<camera_control::do_trajectory_task>("/trajectory_task_ready",1);//gererator path and pub
 	//fabu 

	//客户端
	ros::ServiceClient m_track_target = n.serviceClient<camera_control::set_interest_ID>("/target_tracker/set_interest_id");
	servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");

	//服务端
	ros::ServiceServer m_safe_state = n.advertiseService("/flx_safe_state", safe_state_callback);

	
	//定时器
	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

	init_grasp(servoseter);

	//variables
	int last_state = -1;
	bool along_marker_path = false;
	//bool trajectory_follow_success = false;
	double time_control;
	double time_counter;
	while(ros::ok()){
		if(target.valid_marker_cnt>1000) target.valid_marker_cnt = 1000;
		if(target.valid_target_cnt>1000) target.valid_target_cnt = 1000;

		switch(global_state){
			case FLY_ALONG_MARKER:
			{
				 
				//标志位，判断是否还在这个循环中，这样写则进入这个状态的第一次会出现状态提示
				if(global_state!=last_state)
				{
					
					ROS_INFO("FLY_ALONG_MARKER.....");
					last_state=global_state;

				} 
				if(along_marker_path==false)
				{
					nav_msgs::Path m_path;

					//TODO: 规划到目标物后面
					mav.generate_path(m_path,target.marker_pos.x,target.marker_pos.y,target.marker_pos.z+0.75,0.03,true,true);// 
					if(m_path.poses.size()>0) Path_puber.publish(m_path);//发布轨迹


					//距离目标物足够近了，转而看小目标物，不看大目标物。  
					if(m_path.poses.size()>0&&m_path.poses.size()<30)//32
					{
						cout<<"path size is "<<m_path.poses.size()<<endl;
						along_marker_path=true;
						camera_control::set_interest_ID IDmsg;
						IDmsg.request.id=25;
						ROS_INFO("set_interest_ID 25.....");
						//target_path_track node 返回的是小tag的数据
						m_track_target.call(IDmsg);
						if (IDmsg.response.id_ret==25)
						{
							ROS_INFO("set_interest_ID 25 done.....");
						}
					}
				}
				//TODO: 需要明确大概多远能够看到小tag
				if(along_marker_path&&target.valid_target_cnt>2)//看到两次小tag后
				{
						//状态跳转
						
						global_state=FLY_ALONG_TARGET;
						process_state=FLY_ALONG_TARGET;
						//时间控制
						time_control=ros::Time::now().toSec(); 

						//mav.s 记录当前yaw角 四元数
						mav.s  = mav.Local_pose.orientation;
						
						
						//sx=Local_pose.position.x;sy=Local_pose.position.y;sz=mav_qz;sw=mav_qw;
				}
				

				//mav.Local_pose.position.z

				// if(mav.Local_pose.position.z>0.6)//start signal for trajectory controller node. 
				// {
				// 	camera_control::do_trajectory_task msg;
				// 	msg.do_trajectory_task=1;
				// 	Trajectory_task_puber.publish(msg);
				// }
				mav.s  = mav.Local_pose.orientation;
				break;		
			}
			case FLY_ALONG_TARGET:
			{
				if(global_state!=last_state)
				{
					ROS_INFO("FLY_ALONG_TARGET.....");
					time_counter=ros::Time::now().toSec(); 
					last_state=global_state;
				}
				nav_msgs::Path m_path;
				//ROS_INFO("target world pos is %2f %2f %2f\n",target.Posi_w.x,target.Posi_w.y,target.Posi_w.z);
				mav.generate_path(m_path,target.Posi_w.x,target.Posi_w.y,target.Posi_w.z+0.35,0.03,true,true);//39 25
				ROS_ERROR("target path size id %d",m_path.poses.size());
				if(m_path.poses.size()){
					Path_puber.publish(m_path);
					ROS_INFO("target path has planned!");
				}
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
				if(ros::Time::now().toSec()-time_control>1.6)//1.4
				{
					//cout<<"did run this?"<<endl;

					//ROS_INFO("fly to target use %2f",ros::Time::now().toSec()-time_control);
					 
					mav.s  = mav.Local_pose.orientation;
					//规划轨迹
					target.grasp_flag = true;
					if(target.grasp_is_done == true){
						global_state=GRASP_DONE;
						process_state=GRASP_DONE;
						
						ROS_INFO("grasp done use %2f", ros::Time::now().toSec()-time_control-1.3);
						time_control=ros::Time::now().toSec();
					}
					//sx=Local_pose.position.x;sy=Local_pose.position.y;sz=mav_qz;sw=mav_qw;
				}
				mav.s  = mav.Local_pose.orientation;

			
				break;	
			}

			// case HOLD_AND_GRASP:
			// {
			// 	if(global_state!=last_state)
			// 	{
			// 		ROS_INFO("HOLD_AND_GRASP.....");
			// 		last_state=global_state;
			// 	}
			// 	if(trajectory_follow_success)//wait for grasping done.           signal from trajectory controller node
			// 	{
			// 		global_state=GRASP_RISE;
			// 		process_state=GRASP_RISE;
			// 		time_control=ros::Time::now().toSec(); 
					
			// 		mav.rise_position = mav.Local_pose.position;
			// 		// rise_pose_x=mav_x;
			// 		// rise_pose_y=mav_y;
			// 		// rise_pose_z=mav_z;
			// 		mav.s = mav.Local_pose.orientation;
					
			// 	}

			// 	break;	
			// }
			// case GRASP_RISE:
			// {
			// 	if(global_state!=last_state)
			// 	{
			// 		ROS_INFO("GRASP_RISE.....");
			// 		last_state=global_state;
			// 	}
			// 	nav_msgs::Path m_path;//抓取完成后，将目标物抬起5cm

			// 	mav.generate_path(m_path,mav.rise_position.x,mav.rise_position.y,mav.rise_position.z+0.05,0,false,false);

			// 	if(m_path.poses.size())Path_puber.publish(m_path);
			// 	if(ros::Time::now().toSec()-time_control>3.0)
			// 	{
			// 		global_state=GRASP_DONE;
			// 		process_state=GRASP_DONE;
			// 		time_control=ros::Time::now().toSec(); 
			// 	}

			// 	break;	
			// }
			case GRASP_DONE:
			{
				if(global_state!=last_state)
				{
					ROS_INFO("GRASP_DONE.....");
					last_state=global_state;
				}

		
				if(ros::Time::now().toSec()-time_control>3.0)
				{
					
					global_state=BACK_HOME_AND_LANGING;
					process_state=BACK_HOME_AND_LANGING;
					time_control=ros::Time::now().toSec(); 

					mav.s  = mav.Local_pose.orientation;
				//	sx=Local_pose.position.x;sy=Local_pose.position.y;sz=mav_qz;sw=mav_qw;
				}
					mav.s  = mav.Local_pose.orientation;

				break;	 
			}
			//TODO:x
			case BACK_HOME_AND_LANGING://B3中间段
			{
				if(global_state!=last_state)
				{
					ROS_INFO("BACK_HOME_AND_LANGING.....");
					ROS_INFO("from fly2Target to Backhome use %2f",time_counter - ros::Time::now().toSec());

					mav.landing_posi = mav.Local_pose.position;
					//ROS_INFO("START POINT IS:X:%2f  Y:%2f  Z:%2f",mav.rise_position.x,mav.rise_position.y,mav.rise_position.z);
					last_state=global_state;
				}
				//爪子已经收起来了
				am_controller::servoset_srv msg;
				msg.request.cmd=5;
				msg.request.pos1=-10;
				msg.request.pos2=170;//pi/2;
				msg.request.pos3=-70;
				msg.request.pos4=0;
				msg.request.action_time=2000;
				limited_grasp_call(servoseter,msg);
				if(ros::Time::now().toSec()-time_control>1.5)

				{


					nav_msgs::Path m_path;
					//返回原点
					mav.generate_path(m_path,mav.rise_position.x,mav.rise_position.y,mav.landing_posi.z+0.05,0.06,false,false);
					//下降
					//mav.generate_path(m_path,0,0,-5,0.05,false,false);
					if(m_path.poses.size())Path_puber.publish(m_path);
					if(m_path.poses.size()<5 ){
						global_state  = FORCE_LANGING;
						process_state=FORCE_LANGING;
					}
				}
				break;	
			}
			case FORCE_LANGING://B3 最下段模式
			{	
				if(global_state!=last_state)
				{
					ROS_INFO("FORCE_LANGING.....");
					mav.landing_posi = mav.Local_pose.position;
					mav.s = mav.Local_pose.orientation;
					last_state=global_state;
				}
				am_controller::servoset_srv msg;
				msg.request.cmd=5;
				msg.request.pos1=-10;
				msg.request.pos2=170;//pi/2;
				msg.request.pos3=-70;
				msg.request.pos4=0;
				msg.request.action_time=1500;
				limited_grasp_call(servoseter,msg);
				if(ros::Time::now().toSec()-time_control>0.5)
				{
					nav_msgs::Path m_path;
					mav.landing(m_path);
					if(m_path.poses.size())Path_puber.publish(m_path);
					time_control=ros::Time::now().toSec(); 
				}
				break;	
			}


		}
	

	//TODO: 
	ros::spinOnce();
	loop_rate.sleep();
	}


	return 0;
}