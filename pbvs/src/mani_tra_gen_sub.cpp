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
#include <queue>

#include "Eigen/Eigen"
#include <cmath>
using namespace KDL;
using namespace std;
 
#define pi  3.1415926535898



camera_control::TagLost TagLost_msg;
am_controller::servoset_srv servoset_srv_msg;
nav_msgs::Path path_msg,grasp_msg_path;
Eigen::Vector2d grasp_point_eigen;

//nav_msgs::Path grasp_path_msg;
ros::Publisher grasp_path_pub;
ros::Publisher path_pub;
ros::Publisher marker_pub;
ros::Publisher endeft_pub;
ros::Publisher base_pub;
ros::ServiceClient servoseter;
double first= 0.1,second=0.01;
double i=0.1;

queue<Eigen::Vector2d> grasp_path_queue;

//geometry_msgs::PoseStamped poseStamped,grasp_poseStamped;


Eigen::Matrix4d Tmb;
int flag=0;

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

void grasp(ros::ServiceClient &servoseter){

	am_controller::servoset_srv msg;
	msg.request.cmd=4;
	msg.request.pos1=10;
	msg.request.action_time=100;
	servoseter.call(msg);
}

void loose(ros::ServiceClient &servoseter){

	am_controller::servoset_srv msg;
	msg.request.cmd=6;
	msg.request.pos1=0;
	msg.request.pos2=0;//pi/2;
	msg.request.pos3=0.21;
	msg.request.pos4=-0.02;
	msg.request.action_time=2000;
	servoseter.call(msg);
	

}

void limited_grasp_call(ros::ServiceClient &servoseter,am_controller::servoset_srv &msg)
{
	static double last_call_time=0;

	if(ros::Time::now().toSec()-last_call_time>0.25)
	{
		servoseter.call(msg);	
		last_call_time=ros::Time::now().toSec();
	}
	
}

void grasp_path_track(ros::ServiceClient &servoseter,nav_msgs::Path &path){
	int pointer=0;
	am_controller::servoset_srv servoset_srv_msg;
	// for(pointer=0;pointer<path.poses.size();pointer++){
	// 	ros::Duration(0.08).sleep();

	// 	servoset_srv_msg.request.cmd=6;
	// 	servoset_srv_msg.request.pos1=0;
	// 	servoset_srv_msg.request.pos2=0;
	// 	servoset_srv_msg.request.pos3=path.poses[pointer].pose.position.x;
	// 	servoset_srv_msg.request.pos4=path.poses[pointer].pose.position.y;
	// 	servoset_srv_msg.request.action_time=80;
	// 	servoseter.call(servoset_srv_msg);
	// 	if(servoset_srv_msg.response.is_done == 0){
	// 		ROS_ERROR("BAD x=%f,y=%f",path.poses[pointer].pose.position.x,path.poses[pointer].pose.position.y);
	// 	}
	// 	visualization_msgs::Marker endef;
	// 	endef.header.frame_id="/body";
	// 	endef.header.stamp = ros::Time::now();
	// 	endef.id = 0;
	// 	endef.ns = "basic_shapes";
	// 	endef.type = visualization_msgs::Marker::ARROW;
	// 	endef.pose.position.x = path.poses[pointer].pose.position.x;
	// 	endef.pose.position.y = path.poses[pointer].pose.position.y;
	// 	endef.pose.position.z = 0;
	// 	endef.pose.orientation.x= 0;
	// 	endef.pose.orientation.y = 0;
	// 	endef.pose.orientation.z = 0;
	// 	endef.pose.orientation.w = 1;
	// 	endef.scale.x = 0.01;
	// 	endef.scale.y=0.01;
	// 	endef.scale.z = 0.01;
	// 	endef.color.r = 1.0f;
	// 	endef.color.g = 0.0f;
	// 	endef.color.b = 0.0f;
	// 	endef.color.a = 1.0;
	// 	endef.lifetime = ros::Duration();
	// 	endeft_pub.publish(endef);
	// }

   while(!grasp_path_queue.empty()){
		ros::Duration(0.08).sleep();
		Eigen::Vector2d eigen_vector;
		eigen_vector = grasp_path_queue.front();
		servoset_srv_msg.request.cmd=6;
		servoset_srv_msg.request.pos1=0;
		servoset_srv_msg.request.pos2=0;
		servoset_srv_msg.request.pos3=eigen_vector(0);
		servoset_srv_msg.request.pos4=eigen_vector(1);
		servoset_srv_msg.request.action_time=80;
		servoseter.call(servoset_srv_msg);
		if(servoset_srv_msg.response.is_done == 0){
			ROS_ERROR("BAD x=%f,y=%f",eigen_vector(0),eigen_vector(1));
		}
		visualization_msgs::Marker endef;
		endef.header.frame_id="/body";
		endef.header.stamp = ros::Time::now();
		endef.id = 0;
		endef.ns = "basic_shapes";
		endef.type = visualization_msgs::Marker::ARROW;
		endef.pose.position.x = eigen_vector(0);
		endef.pose.position.y = eigen_vector(1);
		endef.pose.position.z = 0;
		endef.pose.orientation.x= 0;
		endef.pose.orientation.y = 0;
		endef.pose.orientation.z = 0;
		endef.pose.orientation.w = 1;
		endef.scale.x = 0.01;
		endef.scale.y=0.01;
		endef.scale.z = 0.01;
		endef.color.r = 1.0f;
		endef.color.g = 0.0f;
		endef.color.b = 0.0f;
		endef.color.a = 1.0;
		endef.lifetime = ros::Duration();
		endeft_pub.publish(endef);
		grasp_path_queue.pop();
	}
}





void init_Tmb(void)
{
	//Tmb init
	Tmb=Eigen::MatrixXd::Zero(4,4);Tmb(3,3)=1.0;
	Tmb(0,0)=1.0;		Tmb(0,1)=0.0;		Tmb(0,2)=0.0;		Tmb(0,3)=0.0; //Tmb(0,3)=0.04;	//从manipulator到body
	Tmb(1,0)=0.0;		Tmb(1,1)=0.0;		Tmb(1,2)=1.0;		Tmb(1,3)=0.24;
	Tmb(2,0)=0.0;		Tmb(2,1)=-1.0;		Tmb(2,2)=0.0;		Tmb(2,3)=0.0;
	Tmb(3,0)=0.0;		Tmb(3,1)=0.0;		Tmb(3,2)=0.0;		Tmb(3,3)=1.0;
}



class MyPath
{
public:
	// you can get some meta-info on the path:

	/*void get_path_info(Path_RoundedComposite* path){
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
	}*/
	void generate_path2target(geometry_msgs::Pose &mav_pose,geometry_msgs::Point &target_pose,double &yaw,double &z_compensate,nav_msgs::Path &msg_path){
		
		//0.2 0.01 是啥?
		try{
			Path_RoundedComposite* path = new Path_RoundedComposite(i,0.01,new RotationalInterpolation_SingleAxis());
			//double radius,double eqradius,RotationalInterpolation* orient, bool aggregate=true
			/*@param radius : 圆弧的半径
			* @param eqradius :旋转和速度的过度 equivalent radius to compare rotations/velocities
			* @param orient   : method of rotational_interpolation interpolation 角轴插补方法
			* @param aggregate : if true, this object will own the _orient pointer, i.e. it will delete the _orient pointer
			*                    when the destructor of this object is called.*/
	
			double x_dir,y_dir;
			double delta = 0.4;
			x_dir = target_pose.x -  mav_pose.position.x;
			y_dir = target_pose.y -  mav_pose.position.y;
			double D_x = x_dir/(sqrt(x_dir*x_dir+y_dir*y_dir));
			double D_y = y_dir/(sqrt(x_dir*x_dir+y_dir*y_dir));
			path->Add(Frame(Rotation::RPY(yaw,0,0), Vector(mav_pose.position.x,mav_pose.position.y,mav_pose.position.z+z_compensate)));
			path->Add(Frame(Rotation::RPY(yaw,0,0), Vector(target_pose.x,target_pose.y,target_pose.z+z_compensate)));
			path->Add(Frame(Rotation::RPY(yaw,0,0),Vector(target_pose.x+D_x*delta,target_pose.y+D_y*delta,target_pose.z+z_compensate)));


			path->Finish();

			//这几段表示将从起始到终点的速度设定为梯形波，最大速度为0.5，加速度为0.1。
			VelocityProfile* velpref = new VelocityProfile_Trap(0.5,0.1);
			velpref->SetProfile(0,path->PathLength());  
			//给路径加上速度，加速度信息
			Trajectory* traject = new Trajectory_Segment(path, velpref);
			// use the trajectory

			geometry_msgs::Pose poses;
			geometry_msgs::PoseStamped poseStampeds;
			double dt=0.05;
			msg_path.poses.clear();
			std::ofstream of("./trajectory.dat");
			for (double t=0.0; t <= traject->Duration(); t+= dt) {
				Frame current_pose;
				current_pose = traject->Pos(t);
				tf::poseKDLToMsg(current_pose,poses);
				msg_path.header.frame_id = "world";
				poseStampeds.header.frame_id = "world";
				poseStampeds.pose  = poses;

				msg_path.poses.push_back(poseStampeds);

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
	void generate_grasp_trajectory(double &target_x,double &target_y,nav_msgs::Path &grasp_traj){
			ROS_INFO("target_x,y=%f  %f/n",target_x,target_y);
		try{
			
			double start,duration=0;
			start = ros::Time::now().toSec();
			Path_RoundedComposite* path = new Path_RoundedComposite(first,second,new RotationalInterpolation_SingleAxis());
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
			double dt=0.05;
			queue<Eigen::Vector2d> empty;
			swap(grasp_path_queue,empty);
			grasp_traj.poses.clear();


			for (double t=0.0; t <= traject->Duration(); t+= dt) {
				Frame current_pose;
				current_pose = traject->Pos(t);
				tf::poseKDLToMsg(current_pose,poses);
				grasp_traj.header.frame_id = "body";
				poseStampeds.header.frame_id = "body";
				poseStampeds.pose  = poses;
				grasp_traj.poses.push_back(poseStampeds);
				grasp_point_eigen<<poses.position.x,poses.position.y;
				grasp_path_queue.push(grasp_point_eigen);
			}
			duration= ros::Time::now().toSec()-start;
			ROS_INFO("grasp_trij size is:%d eigensize:%d---use%f sec",grasp_msg_path.poses.size(),grasp_path_queue.size(),duration);
		}catch(Error& error) {
			std::cout <<"I encountered this error : " << error.Description() << std::endl;
			std::cout << "with the following type " << error.GetType() << std::endl;
		}

	}


};

MyPath path;

void target_pose_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg)

{
	ROS_INFO("Enter callback!");
	double target_x =0.0,target_y=0.0,target_z=0.0;
	Eigen::Matrix4d Tbar,Tma;
	visualization_msgs::Marker marker;
	Tbar(0,0)=Tba_msg->r11;Tbar(0,1)=Tba_msg->r12;Tbar(0,2)=Tba_msg->r13;Tbar(0,3)=Tba_msg->t1;
	Tbar(1,0)=Tba_msg->r21;Tbar(1,1)=Tba_msg->r22;Tbar(1,2)=Tba_msg->r23;Tbar(1,3)=Tba_msg->t2;
	Tbar(2,0)=Tba_msg->r31;Tbar(2,1)=Tba_msg->r32;Tbar(2,2)=Tba_msg->r33;Tbar(2,3)=Tba_msg->t3;
	Tbar(3,0)=0;Tbar(3,1)=0;Tbar(3,2)=0;Tbar(3,3)=1;
	Tma = Tmb*Tbar;
	target_x=Tma(0,3);target_y=Tma(1,3);target_z=Tma(2,3);

	path.generate_grasp_trajectory(target_x,target_y,grasp_msg_path);
	grasp_path_pub.publish(grasp_msg_path);

	marker.header.frame_id="/body";
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::Marker::CUBE;
	marker.pose.position.x = target_x;
	marker.pose.position.y = target_y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x= 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.scale.x = 0.1;
	marker.scale.y=0.1;
	marker.scale.z = 0.1;
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
	visualization_msgs::Marker base;
	base.header.frame_id="/body";
	base.header.stamp = ros::Time::now();
	base.id = 0;
	base.ns = "basic_shapes";
	base.type = visualization_msgs::Marker::SPHERE;
	base.pose.position.x = 0;
	base.pose.position.y = 0;
	base.pose.position.z = 0;
	base.pose.orientation.x= 0;
	base.pose.orientation.y = 0;
	base.pose.orientation.z = 0;
	base.pose.orientation.w = 1;
	base.scale.x = 0.05;
	base.scale.y=0.05;
	base.scale.z = 0.05;
	base.color.r = 0.0f;
	base.color.g = 1.0f;
	base.color.b = 0.0f;
	base.color.a = 1.0;
	base.lifetime = ros::Duration();
	base_pub.publish(base);

	if(grasp_msg_path.poses.size()>10&&flag==0){

		grasp_path_track(servoseter,grasp_msg_path);

		grasp(servoseter);

		ros::Duration(1).sleep();
		loose(servoseter);
		ROS_INFO("-------------");
		ROS_INFO("--done--");
		flag=1;

}





}


int main(int argc, char *argv[])
{

	init_Tmb();
	ros::init(argc, argv, "mani_tra_gen");


	ros::NodeHandle n;
	n.param<double>("first",first,0.1);
	n.param<double>("second",second,0.01);


	ros::Subscriber target_rev=n.subscribe("/Tba_current",1,target_pose_callback);

	servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");

	grasp_path_pub =n.advertise<nav_msgs::Path>("/grasp_path_planned",1000);
	path_pub =n.advertise<nav_msgs::Path>("/path_planned",1000);
	marker_pub = n.advertise<visualization_msgs::Marker>("/target_marker",1);
	base_pub = n.advertise<visualization_msgs::Marker>("/base_marker",1);
	endeft_pub = n.advertise<visualization_msgs::Marker>("/endeffector",1);
	init_grasp(servoseter);
	ros::Rate loop_rate(10);
	while (ros::ok())
	{

			n.getParam("first",first);
			n.getParam("second",second);
		//	ROS_INFO("first para init is:%f",first);	
		//	ROS_INFO("second para init is:%f",second);
			ros::spinOnce();
	}
	




	return 0;
}