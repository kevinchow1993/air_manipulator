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

#include "Eigen/Eigen"
#include <cmath>
using namespace KDL;
using namespace std;


 #define pi  3.1415926535898

camera_control::TagLost TagLost_msg;
am_controller::servoset_srv servoset_srv_msg;
//nav_msgs::Path path_msg,grasp_msg_path;
//nav_msgs::Path grasp_path_msg;
//ros::Publisher grasp_path_pub;
ros::Publisher path_pub;
ros::Publisher marker_pub;
ros::Publisher endeft_pub;
ros::Publisher body_pub;
ros::ServiceClient servoseter;
double first,second;
Eigen::Isometry3d T_eft_lasttime;
Eigen::Vector3d temp_target;
//tf::TransformBroadcaster br;


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
	Eigen::Isometry3d Twb;
	visualization_msgs::Marker body;
	
	Mav(){
		

		T_mav_last_time.setIdentity();
		last_time = 0.0;
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
		ROS_ERROR("****mav object has created!!***");
		

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
			distance = sqrt(pow(target.x-mav.position.x,2)+pow(target.y - mav.position.y,2));
			return dist;
	}

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
	void sendTransformTwb(){
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(Local_pose.position.x,Local_pose.position.y,Local_pose.position.z ));
		tf::Quaternion tf_q(Local_pose.orientation.x,Local_pose.orientation.y,Local_pose.orientation.z,Local_pose.orientation.w);
		transform.setRotation(tf_q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/body"));
	}

	void pub_mavmarker(ros::Publisher &mav_marker_pub)
	{
		mav_marker_pub.publish(body);
	}
	void display(){
		cout<<"-----mav info------"<<endl;
		cout.precision(3);
		cout<<"--mav position--";
		cout<<"x:"<<Local_pose.position.x<<"y:"<<Local_pose.position.y<<"z:"<<Local_pose.position.z<<endl;
		cout<<"--yaw--"<<current_yaw*180.0/pi<< endl;
		cout<<"--Twb--"<<endl;;
		cout<<Twb.matrix()<<endl;


	}
	
};
class Target//初始化
{
public:
	
	int valid_target_cnt;
	geometry_msgs::Point Posi_m;
	geometry_msgs::Point Posi_w;
	visualization_msgs::Marker visual;
	geometry_msgs::Vector3 vel;
	double last_time;
	bool Delta_T_is_ok;
	Eigen::Isometry3d Twa;
	Eigen::Isometry3d Tba;
	//Eigen::Quaterniond tar_q;

	Target(){
		valid_target_cnt = 0;
		last_time = 0.0;
		Delta_T_is_ok = 0;
		visual.header.frame_id="/world";
		
		visual.id = 0;
		visual.ns = "basic_shapes";
		visual.type = visualization_msgs::Marker::CUBE;
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
		ROS_ERROR("****target object has created!!***");

		
	}
	~Target(){};
	void get_vel(const geometry_msgs::PoseStampedConstPtr &msg){
		double	now = ros::Time::now().toSec();
		double dt = now-last_time;
		last_time=now;
		//if(dt>0.05){
		vel.x = (msg->pose.position.x-Posi_m.x)/dt;
		vel.y = (msg->pose.position.y-Posi_m.y)/dt;
		vel.z = (msg->pose.position.z-Posi_m.z)/dt;
	
	}
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
	void pub_marker(ros::Publisher &marker_pub){
		
		marker_pub.publish(visual);

	}
	void display(){
		cout<<"-----target info------"<<endl;
		cout.precision(3);
		cout<<"--World Frame--"<<endl;
		cout<<"x:"<<Posi_w.x<<"y:"<<Posi_w.y<<"z:"<<Posi_w.z<<endl;
		cout<<"----"<<endl;
		cout<<"--Body Frame--"<<endl;
		cout<<"x:"<<Posi_m.x<<"y:"<<Posi_m.y<<"z:"<<Posi_m.z<<endl;

	}
	void sendTransformTarget(){
		static tf::TransformBroadcaster Tar_br;
		tf::Transform transform;
		tf::transformEigenToTF(Twa,transform);
		
		Tar_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/target"));
		
		
	}
	//void target::target_pose_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg);
	

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

void call_servo(Eigen::Vector3d &p){

	servoset_srv_msg.request.cmd=6;
	servoset_srv_msg.request.pos1=0;
	servoset_srv_msg.request.pos2=0;
	servoset_srv_msg.request.pos3=p(0);
	servoset_srv_msg.request.pos4=p(1);
	servoset_srv_msg.request.action_time=500;
}

// 获得delat_t矩阵 
void target_pose_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg)

{
	cout<<"enter target!"<<endl;
	Eigen::Isometry3d Delta_T;
	geometry_msgs::Pose mav_pose=mav.Local_pose;
	Eigen::Isometry3d T_mav_thistime;
	tf::poseMsgToEigen(mav_pose,T_mav_thistime);
	if(!target.Delta_T_is_ok){
		Delta_T.setIdentity();
		mav.T_mav_last_time = T_mav_thistime;
		target.Delta_T_is_ok = 1;
	}else{
		Delta_T = T_mav_thistime*mav.T_mav_last_time.inverse();
		mav.T_mav_last_time = T_mav_thistime;}
	

	//ROS_INFO("Enter callback!");

	Eigen::Isometry3d Tbar,Tma,Twa;
	//visualization_msgs::Marker marker;
	Tbar(0,0)=Tba_msg->r11;Tbar(0,1)=Tba_msg->r12;Tbar(0,2)=Tba_msg->r13;Tbar(0,3)=Tba_msg->t1;
	Tbar(1,0)=Tba_msg->r21;Tbar(1,1)=Tba_msg->r22;Tbar(1,2)=Tba_msg->r23;Tbar(1,3)=Tba_msg->t2;
	Tbar(2,0)=Tba_msg->r31;Tbar(2,1)=Tba_msg->r32;Tbar(2,2)=Tba_msg->r33;Tbar(2,3)=Tba_msg->t3;
	Tbar(3,0)=0;Tbar(3,1)=0;Tbar(3,2)=0;Tbar(3,3)=1;
	Tma = Tmb*Tbar;
	target.set_Twa(Tbar,mav.Twb);

	target.pub_marker(marker_pub);
	target.sendTransformTarget();
	target.display();
	cout.precision(3);
	cout<<"Tmax:"<<Tma(0,3)<<"Tmay:"<<Tma(1,3)<<"Tmaz:"<<Tma(2,3)<<endl;
	//Twa = mav.Twb*Tbar;
	/*
	target.position.x=Tma(0,3);
	target.position.y=Tma(1,3);
	target.position.z=Tma(2,3);


	Delta_T=Tmb*Delta_T;
	temp_target = Delta_T.inverse()*temp_target;
	cout<<"Delta_t is  \n"<<Delta_T.matrix()<<endl;
	cout<<"temp_target \n"<<temp_target<<endl;
	//call_servo(temp_target);*/

	//规划轨迹
	//path.generate_grasp_trajectory(target_x,target_y,grasp_msg_path);
	//grasp_path_pub.publish(grasp_msg_path);
	
/*
	target.visual.header.stamp = ros::Time::now();
	target.visual.pose.position.x = Twa(0,3);
	target.visual.pose.position.y = Twa(1,3);
	target.visual.pose.position.z = Twa(2,3);
	target.visual.pose.orientation.x= 0;
	target.visual.pose.orientation.y = 0;
	target.visual.pose.orientation.z = 0;
	target.visual.pose.orientation.w = 1;

	marker_pub.publish(target.visual);*/

}

/**

得到无人机的位姿,速度,yaw角,发布坐标变换,


**/
void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg){
	//cout<<"enter mav"<<endl;
	/*mav.get_vel(msg);
	mav.current_yaw = tf::getYaw(msg->pose.orientation);
	geometry_msgs::Pose m = msg->pose;
	tf::poseMsgToEigen(m,mav.Twb);

	mav.body.header.stamp = ros::Time::now();
	mav.body.pose.position.x = mav.Local_pose.position.x ;
	mav.body.pose.position.y = mav.Local_pose.position.y;
	mav.body.pose.position.z = mav.Local_pose.position.z;
	mav.body.pose.orientation.x= mav.Local_pose.orientation.x;
	mav.body.pose.orientation.y = mav.Local_pose.orientation.y;
	mav.body.pose.orientation.z = mav.Local_pose.orientation.z;
	mav.body.pose.orientation.w = mav.Local_pose.orientation.w;
	body_pub.publish(mav.body);*/
	mav.set_Twb(msg);
	mav.sendTransformTwb();
	mav.pub_mavmarker(body_pub);
	mav.display();



}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dynamic_grasp");

	init_Tmb();
	ros::NodeHandle n;
	
	// 订阅 
	//ros::Subscriber camera_pose_suber = n.subscribe("/camera_servo_pose",1,&CameraPos_CallBack);
	//ros::Subscriber trajectory_follow_success_flag_suber=n.subscribe("/trajectory_follow_success_flag",1,trajectory_follow_success_flag_CallBack);///mavros/vision_pose/pose
	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
	ros::Subscriber target_rev=n.subscribe("/Tba_current",1,target_pose_callback);
    //发布
	marker_pub = n.advertise<visualization_msgs::Marker>("/target_marker",1);
	body_pub = n.advertise<visualization_msgs::Marker>("/body_marker",1);
	endeft_pub = n.advertise<visualization_msgs::Marker>("/endeffector",1);
 	//fabu 

	servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");



	ros::spin();

	return 0;
}